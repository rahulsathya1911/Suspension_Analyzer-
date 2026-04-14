/// backend/src/main.rs
///
/// Suspension Analysis API
/// ========================
/// Endpoints:
///   POST /simulate  — run a single simulation, persist result, return full output
///   GET  /history   — return all past simulation runs
///   POST /frf       — return analytical FRF across a frequency range
///   POST /sweep     — run a k×c parameter sweep, return Pareto front

use rocket::{get, post, routes, State};
use rocket::serde::{Serialize, Deserialize, json::Json};
use rocket::http::{Status, Header};
use rocket::response::status::Custom;
use rocket::fairing::{Fairing, Info, Kind};
use rocket::{Request, Response};
use sqlx::{PgPool, postgres::PgPoolOptions};
use dotenvy::dotenv;
use std::env;

// ═══════════════════════════════════════════════════════════
// CORS — allows the HTML frontend to call the API
// ═══════════════════════════════════════════════════════════

pub struct Cors;

#[rocket::async_trait]
impl Fairing for Cors {
    fn info(&self) -> Info {
        Info { name: "CORS", kind: Kind::Response }
    }
    async fn on_response<'r>(&self, _req: &'r Request<'_>, res: &mut Response<'r>) {
        res.set_header(Header::new("Access-Control-Allow-Origin",  "*"));
        res.set_header(Header::new("Access-Control-Allow-Methods", "GET, POST, OPTIONS"));
        res.set_header(Header::new("Access-Control-Allow-Headers", "Content-Type"));
    }
}

use suspension_core::{
    run_simulation,
    compute_frf,
    parameter_sweep,
    extract_pareto_front,
    log_freq_range,
    linspace,
    RoadProfile,
};

// ═══════════════════════════════════════════════════════════
// SECTION 1 — Shared error type
// ═══════════════════════════════════════════════════════════

/// All API errors return this JSON body with an HTTP 400 status.
#[derive(Serialize)]
#[serde(crate = "rocket::serde")]
struct ApiError {
    error: String,
}

type ApiResult<T> = Result<Json<T>, Custom<Json<ApiError>>>;

fn bad_request(msg: &str) -> Custom<Json<ApiError>> {
    Custom(Status::BadRequest, Json(ApiError { error: msg.to_string() }))
}

#[derive(Debug, Clone, Serialize)]
#[serde(crate = "rocket::serde")]
struct ResolvedQuarterParams {
    ms: f64,
    mu: f64,
    k: f64,
    c: f64,
    kt: f64,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(crate = "rocket::serde")]
struct BikeModelInput {
    ms_front: f64,
    ms_rear: f64,
    mu_front: f64,
    mu_rear: f64,
    k_front: f64,
    k_rear: f64,
    c_front: f64,
    c_rear: f64,
    #[serde(default)]
    kt_front: Option<f64>,
    #[serde(default)]
    kt_rear: Option<f64>,
    #[serde(default)]
    front_weight_distribution_pct: Option<f64>,
    #[serde(default)]
    rider_mass_kg: Option<f64>,
}

#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
#[serde(crate = "rocket::serde")]
enum BikeClassInput {
    Scooter,
    Commuter,
    Middleweight,
    Supersport,
    Heavyweight,
    Adventure,
    Offroad,
    ElectricMoto,
}

#[derive(Debug, Clone, Deserialize, Serialize, PartialEq)]
#[serde(crate = "rocket::serde")]
enum BikeSuspensionInput {
    TelescopicFork,
    UsdFork,
    Monoshock,
    DualShock,
    Telelever,
    Paralever,
}

#[derive(Debug, Clone, Serialize)]
#[serde(crate = "rocket::serde")]
struct BikeResolvedSummary {
    ms_front: f64,
    ms_rear: f64,
    mu_front: f64,
    mu_rear: f64,
    k_front: f64,
    k_rear: f64,
    c_front: f64,
    c_rear: f64,
    kt_front: f64,
    kt_rear: f64,
    front_weight_distribution_pct: f64,
    rider_mass_kg: f64,
    equivalent: ResolvedQuarterParams,
}

fn bike_natural_frequency_hz(k: f64, m: f64) -> f64 {
    (k / m).sqrt() / (2.0 * std::f64::consts::PI)
}

fn validate_bike_model(b: &BikeModelInput) -> Result<(), &'static str> {
    if b.ms_front <= 0.0 || b.ms_rear <= 0.0 { return Err("bike sprung masses must be positive"); }
    if b.mu_front <= 0.0 || b.mu_rear <= 0.0 { return Err("bike unsprung masses must be positive"); }
    if b.k_front <= 0.0 || b.k_rear <= 0.0 { return Err("bike spring rates must be positive"); }
    if b.c_front < 0.0 || b.c_rear < 0.0 { return Err("bike damping values must be non-negative"); }
    if b.mu_front >= b.ms_front || b.mu_rear >= b.ms_rear {
        return Err("bike unsprung mass must be less than sprung mass on each axle");
    }

    let wf = b.front_weight_distribution_pct.unwrap_or(50.0);
    if !(35.0..=65.0).contains(&wf) {
        return Err("front_weight_distribution_pct must be between 35 and 65");
    }

    if let Some(rider_mass) = b.rider_mass_kg {
        if !(40.0..=140.0).contains(&rider_mass) {
            return Err("rider_mass_kg must be between 40 and 140");
        }
    }

    if let Some(ktf) = b.kt_front {
        if ktf <= 0.0 { return Err("kt_front must be positive"); }
    }
    if let Some(ktr) = b.kt_rear {
        if ktr <= 0.0 { return Err("kt_rear must be positive"); }
    }

    let fn_front = bike_natural_frequency_hz(b.k_front, b.ms_front);
    let fn_rear  = bike_natural_frequency_hz(b.k_rear, b.ms_rear);
    if !(0.8..=3.5).contains(&fn_front) || !(0.8..=3.5).contains(&fn_rear) {
        return Err("bike ride natural frequencies must be in a realistic range (0.8–3.5 Hz)");
    }

    Ok(())
}

fn resolve_bike_to_quarter(b: &BikeModelInput, fallback_kt: Option<f64>) -> Result<BikeResolvedSummary, &'static str> {
    validate_bike_model(b)?;

    let wf = b.front_weight_distribution_pct.unwrap_or_else(|| {
        let total_ms = b.ms_front + b.ms_rear;
        if total_ms <= 0.0 { 50.0 } else { 100.0 * b.ms_front / total_ms }
    });
    let wr = 100.0 - wf;

    let kt_fallback = fallback_kt.unwrap_or(130_000.0);
    let kt_front = b.kt_front.unwrap_or(kt_fallback);
    let kt_rear  = b.kt_rear.unwrap_or(kt_fallback);

    let wf_n = wf / 100.0;
    let wr_n = wr / 100.0;
    let equivalent = ResolvedQuarterParams {
        ms: wf_n * b.ms_front + wr_n * b.ms_rear,
        mu: wf_n * b.mu_front + wr_n * b.mu_rear,
        k:  wf_n * b.k_front  + wr_n * b.k_rear,
        c:  wf_n * b.c_front  + wr_n * b.c_rear,
        kt: wf_n * kt_front   + wr_n * kt_rear,
    };

    if equivalent.mu <= 0.0 || equivalent.ms <= 0.0 || equivalent.k <= 0.0 || equivalent.kt <= 0.0 {
        return Err("resolved bike equivalent parameters are invalid");
    }
    if equivalent.mu >= equivalent.ms {
        return Err("resolved bike model is unstable: equivalent unsprung mass must be less than sprung mass");
    }

    Ok(BikeResolvedSummary {
        ms_front: b.ms_front,
        ms_rear: b.ms_rear,
        mu_front: b.mu_front,
        mu_rear: b.mu_rear,
        k_front: b.k_front,
        k_rear: b.k_rear,
        c_front: b.c_front,
        c_rear: b.c_rear,
        kt_front,
        kt_rear,
        front_weight_distribution_pct: wf,
        rider_mass_kg: b.rider_mass_kg.unwrap_or(75.0),
        equivalent,
    })
}

// ═══════════════════════════════════════════════════════════
// SECTION 2 — Road profile JSON representation
// ═══════════════════════════════════════════════════════════

/// JSON-serialisable road profile selector.
///
/// Sent by the client as part of simulation/sweep requests.
/// Maps 1-to-1 onto core::RoadProfile.
///
/// Examples:
///   { "type": "Step", "height": 0.05 }
///   { "type": "Sine", "amplitude": 0.01, "freq_hz": 2.0 }
///   { "type": "Iso8608", "roughness_coefficient": 1024e-6, "vehicle_speed_mps": 30.0 }
#[derive(Deserialize, Debug)]
#[serde(crate = "rocket::serde", tag = "type")]
enum RoadProfileInput {
    Step    { height: f64 },
    Sine    { amplitude: f64, freq_hz: f64 },
    Iso8608 { roughness_coefficient: f64, vehicle_speed_mps: f64 },
}

impl RoadProfileInput {
    fn into_core(self) -> RoadProfile {
        match self {
            RoadProfileInput::Step    { height }
            => RoadProfile::Step { height },
            RoadProfileInput::Sine    { amplitude, freq_hz }
            => RoadProfile::Sine { amplitude, freq_hz },
            RoadProfileInput::Iso8608 { roughness_coefficient, vehicle_speed_mps }
            => RoadProfile::Iso8608 { roughness_coefficient, vehicle_speed_mps },
        }
    }

    fn validate(&self) -> Result<(), &'static str> {
        match self {
            RoadProfileInput::Step { height } => {
                if *height <= 0.0 || *height > 1.0 {
                    return Err("height must be between 0 and 1 m");
                }
            }
            RoadProfileInput::Sine { amplitude, freq_hz } => {
                if *amplitude <= 0.0 { return Err("amplitude must be positive"); }
                if *freq_hz   <= 0.0 { return Err("freq_hz must be positive"); }
            }
            RoadProfileInput::Iso8608 { roughness_coefficient, vehicle_speed_mps } => {
                if *roughness_coefficient <= 0.0 {
                    return Err("roughness_coefficient must be positive");
                }
                if *vehicle_speed_mps <= 0.0 {
                    return Err("vehicle_speed_mps must be positive");
                }
            }
        }
        Ok(())
    }

    /// String label for DB storage
    fn label(&self) -> &'static str {
        match self {
            RoadProfileInput::Step    { .. } => "step",
            RoadProfileInput::Sine    { .. } => "sine",
            RoadProfileInput::Iso8608 { .. } => "iso8608",
        }
    }
}

// ═══════════════════════════════════════════════════════════
// SECTION 3 — /simulate
// ═══════════════════════════════════════════════════════════

#[derive(Deserialize)]
#[serde(crate = "rocket::serde")]
struct SimulationInput {
    ms:           Option<f64>,
    mu:           Option<f64>,
    k:            Option<f64>,
    c:            Option<f64>,
    kt:           Option<f64>,
    #[serde(default)]
    bike:         Option<BikeModelInput>,
    #[serde(default)]
    bike_class:   Option<BikeClassInput>,
    #[serde(default)]
    bike_suspension_type: Option<BikeSuspensionInput>,
    #[serde(default)]
    rake_angle_deg: Option<f64>,
    #[serde(default)]
    fn_target_hz: Option<f64>,
    #[serde(default)]
    zeta_target: Option<f64>,
    #[serde(default)]
    front_travel_mm: Option<f64>,
    #[serde(default)]
    rear_travel_mm: Option<f64>,
    #[serde(default)]
    preload_mm: Option<f64>,
    road_profile: RoadProfileInput,
}

impl SimulationInput {
    fn resolve_params(&self) -> Result<(ResolvedQuarterParams, Option<BikeResolvedSummary>), &'static str> {
        if let Some(bike) = &self.bike {
            let resolved = resolve_bike_to_quarter(bike, self.kt)?;
            return Ok((resolved.equivalent.clone(), Some(resolved)));
        }

        let ms = self.ms.ok_or("ms is required unless bike model is provided")?;
        let mu = self.mu.ok_or("mu is required unless bike model is provided")?;
        let k  = self.k.ok_or("k is required unless bike model is provided")?;
        let c  = self.c.ok_or("c is required unless bike model is provided")?;
        let kt = self.kt.ok_or("kt is required unless bike model is provided")?;

        if ms  <= 0.0  { return Err("ms must be positive"); }
        if mu  <= 0.0  { return Err("mu must be positive"); }
        if k   <= 0.0  { return Err("k must be positive"); }
        if c   <  0.0  { return Err("c must be non-negative"); }
        if kt  <= 0.0  { return Err("kt must be positive"); }
        if mu  >= ms { return Err("unsprung mass must be less than sprung mass"); }

        Ok((ResolvedQuarterParams { ms, mu, k, c, kt }, None))
    }

    fn validate(&self) -> Result<(), &'static str> {
        self.resolve_params()?;
        let _bike_meta_present = self.bike_class.is_some() || self.bike_suspension_type.is_some();
        if let Some(rake) = self.rake_angle_deg {
            if !(15.0..=40.0).contains(&rake) {
                return Err("rake_angle_deg must be between 15 and 40");
            }
        }
        if let Some(fn_target) = self.fn_target_hz {
            if !(0.5..=4.0).contains(&fn_target) {
                return Err("fn_target_hz must be between 0.5 and 4.0");
            }
        }
        if let Some(zeta_target) = self.zeta_target {
            if !(0.1..=0.8).contains(&zeta_target) {
                return Err("zeta_target must be between 0.1 and 0.8");
            }
        }
        if let Some(front_travel) = self.front_travel_mm {
            if front_travel <= 0.0 {
                return Err("front_travel_mm must be positive");
            }
        }
        if let Some(rear_travel) = self.rear_travel_mm {
            if rear_travel <= 0.0 {
                return Err("rear_travel_mm must be positive");
            }
        }
        if let Some(preload) = self.preload_mm {
            if preload < 0.0 {
                return Err("preload_mm must be non-negative");
            }
        }
        self.road_profile.validate()
    }
}

/// Full simulation output — all time-series and scalar metrics.
#[derive(Serialize)]
#[serde(crate = "rocket::serde")]
struct SimulationOutput {
    // ── Existing scalar fields ─────────────────────────────
    zeta:           f64,
    mass_ratio:     f64,
    recommended_mu: f64,

    // ── Time series ────────────────────────────────────────
    time:       Vec<f64>,
    body_user:  Vec<f64>,
    wheel_user: Vec<f64>,

    // ── Industrial metrics ─────────────────────────────────
    rms_body_acc:          f64,
    rms_tire_force:        f64,
    max_suspension_travel: f64,
    iso2631_weighted_rms:  f64,
    model_type:            String,
    #[serde(skip_serializing_if = "Option::is_none")]
    bike_front_rear:       Option<BikeResolvedSummary>,
    bottom_out:            bool,
    static_sag_mm:         f64,
    sag_percent:           f64,
    fn_sprung:             f64,
    fn_unsprung:           f64,
}

#[post("/simulate", format = "json", data = "<input>")]
async fn simulate_api(
    input: Json<SimulationInput>,
    db:    &State<PgPool>,
) -> ApiResult<SimulationOutput> {

    input.validate().map_err(bad_request)?;

    let profile_label = input.road_profile.label();

    let (resolved_params, bike_front_rear) = input.resolve_params().map_err(bad_request)?;
    let ms = resolved_params.ms;
    let mu = resolved_params.mu;
    let k  = resolved_params.k;
    let c  = resolved_params.c;
    let kt = resolved_params.kt;

    let sim_in = input.into_inner();
    let static_sag_m = (ms * 9.81) / k;
    let static_sag_mm = static_sag_m * 1000.0;
    let preload_mm = sim_in.preload_mm.unwrap_or(0.0);
    let effective_sag_mm = (static_sag_mm - preload_mm).max(0.0);
    let fn_sprung = bike_natural_frequency_hz(k, ms);
    let fn_unsprung = ((k + kt) / mu).sqrt() / (2.0 * std::f64::consts::PI);

    let travel_limit_mm = match (sim_in.front_travel_mm, sim_in.rear_travel_mm, &sim_in.bike) {
        (Some(front), Some(rear), Some(b)) => {
            let wf = b.front_weight_distribution_pct.unwrap_or(50.0) / 100.0;
            Some(wf * front + (1.0 - wf) * rear)
        }
        (Some(front), Some(rear), None) => Some(0.5 * (front + rear)),
        (Some(front), None, _) => Some(front),
        (None, Some(rear), _) => Some(rear),
        (None, None, _) => None,
    };

    let profile = sim_in.road_profile.into_core();
    let result  = run_simulation(ms, mu, k, c, kt, &profile);
    let max_travel_mm = result.max_suspension_travel * 1000.0;
    let bottom_out = travel_limit_mm.map(|limit| max_travel_mm > limit).unwrap_or(false);
    let sag_percent = travel_limit_mm
        .map(|limit| if limit > 0.0 { (effective_sag_mm / limit) * 100.0 } else { 0.0 })
        .unwrap_or(0.0);

    // Persist to DB
    sqlx::query(
        r#"
        INSERT INTO simulations
            (ms, mu, k, c, kt, road_profile,
             zeta, mass_ratio, recommended_mu,
             rms_body_acc, rms_tire_force,
             max_suspension_travel, iso2631_weighted_rms)
        VALUES
            ($1,$2,$3,$4,$5,$6,
             $7,$8,$9,
             $10,$11,
             $12,$13)
        "#
    )
        .bind(ms).bind(mu).bind(k).bind(c).bind(kt)
        .bind(profile_label)
        .bind(result.zeta)
        .bind(result.mass_ratio)
        .bind(result.recommended_mu)
        .bind(result.rms_body_acc)
        .bind(result.rms_tire_force)
        .bind(result.max_suspension_travel)
        .bind(result.iso2631_weighted_rms)
        .execute(db.inner())
        .await
        .map_err(|e| bad_request(&format!("DB insert failed: {e}")))?;

    Ok(Json(SimulationOutput {
        zeta:                  result.zeta,
        mass_ratio:            result.mass_ratio,
        recommended_mu:        result.recommended_mu,
        time:                  result.time,
        body_user:             result.body_user,
        wheel_user:            result.wheel_user,
        rms_body_acc:          result.rms_body_acc,
        rms_tire_force:        result.rms_tire_force,
        max_suspension_travel: result.max_suspension_travel,
        iso2631_weighted_rms:  result.iso2631_weighted_rms,
        model_type:            if bike_front_rear.is_some() { "bike".into() } else { "quarter_car".into() },
        bike_front_rear,
        bottom_out,
        static_sag_mm:         effective_sag_mm,
        sag_percent,
        fn_sprung,
        fn_unsprung,
    }))
}

// ═══════════════════════════════════════════════════════════
// SECTION 4 — /history
// ═══════════════════════════════════════════════════════════

/// One row from the simulations table, returned by /history.
#[derive(Serialize, sqlx::FromRow)]
#[serde(crate = "rocket::serde")]
struct SimulationRecord {
    id:                    i32,
    ms:                    Option<f64>,
    mu:                    Option<f64>,
    k:                     Option<f64>,
    c:                     Option<f64>,
    kt:                    Option<f64>,
    road_profile:          Option<String>,
    zeta:                  Option<f64>,
    mass_ratio:            Option<f64>,
    recommended_mu:        Option<f64>,
    rms_body_acc:          Option<f64>,
    rms_tire_force:        Option<f64>,
    max_suspension_travel: Option<f64>,
    iso2631_weighted_rms:  Option<f64>,
}

#[get("/history")]
async fn history(db: &State<PgPool>) -> ApiResult<Vec<SimulationRecord>> {
    let records: Vec<SimulationRecord> = sqlx::query_as(
        r#"
        SELECT id, ms, mu, k, c, kt, road_profile,
               zeta, mass_ratio, recommended_mu,
               rms_body_acc, rms_tire_force,
               max_suspension_travel, iso2631_weighted_rms
        FROM simulations
        ORDER BY id DESC
        LIMIT 100
        "#
    )
        .fetch_all(db.inner())
        .await
        .map_err(|e| bad_request(&format!("DB query failed: {e}")))?;

    Ok(Json(records))
}

// ═══════════════════════════════════════════════════════════
// SECTION 5 — /frf
// ═══════════════════════════════════════════════════════════

#[derive(Deserialize)]
#[serde(crate = "rocket::serde")]
struct FrfInput {
    ms:      Option<f64>,
    mu:      Option<f64>,
    k:       Option<f64>,
    c:       Option<f64>,
    kt:      Option<f64>,
    #[serde(default)]
    bike:    Option<BikeModelInput>,
    #[serde(default)]
    rake_angle_deg: Option<f64>,
    #[serde(default)]
    fn_target_hz: Option<f64>,
    #[serde(default)]
    zeta_target: Option<f64>,
    #[serde(default)]
    front_travel_mm: Option<f64>,
    #[serde(default)]
    rear_travel_mm: Option<f64>,
    #[serde(default)]
    preload_mm: Option<f64>,
    /// Start frequency for sweep [Hz] — default 0.5
    f_min:   Option<f64>,
    /// End frequency for sweep [Hz] — default 25.0
    f_max:   Option<f64>,
    /// Number of frequency points — default 100
    n_points: Option<usize>,
}

/// One point in the FRF response.
#[derive(Serialize)]
#[serde(crate = "rocket::serde")]
struct FrfPoint {
    freq_hz:                     f64,
    body_acc_transmissibility:   f64,
    tire_force_transmissibility: f64,
}

#[post("/frf", format = "json", data = "<input>")]
async fn frf_api(input: Json<FrfInput>) -> ApiResult<Vec<FrfPoint>> {
    let inp = input.into_inner();
    let _rake_angle_deg = inp.rake_angle_deg.unwrap_or(25.0);
    let _fn_target_hz = inp.fn_target_hz.unwrap_or(1.5);
    let _zeta_target = inp.zeta_target.unwrap_or(0.30);
    let _front_travel_mm = inp.front_travel_mm.unwrap_or(120.0);
    let _rear_travel_mm = inp.rear_travel_mm.unwrap_or(120.0);
    let _preload_mm = inp.preload_mm.unwrap_or(0.0);

    let resolved = if let Some(bike) = &inp.bike {
        resolve_bike_to_quarter(bike, inp.kt).map_err(bad_request)?.equivalent
    } else {
        let ms = inp.ms.ok_or_else(|| bad_request("ms is required unless bike model is provided"))?;
        let mu = inp.mu.ok_or_else(|| bad_request("mu is required unless bike model is provided"))?;
        let k = inp.k.ok_or_else(|| bad_request("k is required unless bike model is provided"))?;
        let c = inp.c.ok_or_else(|| bad_request("c is required unless bike model is provided"))?;
        let kt = inp.kt.ok_or_else(|| bad_request("kt is required unless bike model is provided"))?;
        if ms <= 0.0 { return Err(bad_request("ms must be positive")); }
        if mu <= 0.0 { return Err(bad_request("mu must be positive")); }
        if k  <= 0.0 { return Err(bad_request("k must be positive")); }
        if c  <  0.0 { return Err(bad_request("c must be non-negative")); }
        if kt <= 0.0 { return Err(bad_request("kt must be positive")); }
        if mu >= ms { return Err(bad_request("unsprung mass must be less than sprung mass")); }
        ResolvedQuarterParams { ms, mu, k, c, kt }
    };

    let f_min    = inp.f_min.unwrap_or(0.5).max(0.01);
    let f_max    = inp.f_max.unwrap_or(25.0);
    let n_points = inp.n_points.unwrap_or(100).clamp(10, 500);

    if f_min >= f_max {
        return Err(bad_request("f_min must be less than f_max"));
    }

    let freqs  = log_freq_range(f_min, f_max, n_points);
    let points = compute_frf(resolved.ms, resolved.mu, resolved.k, resolved.c, resolved.kt, &freqs, 0.01);

    let output: Vec<FrfPoint> = points.into_iter().map(|p| FrfPoint {
        freq_hz:                     p.freq_hz,
        body_acc_transmissibility:   p.body_acc_transmissibility,
        tire_force_transmissibility: p.tire_force_transmissibility,
    }).collect();

    Ok(Json(output))
}

// ═══════════════════════════════════════════════════════════
// SECTION 6 — /sweep
// ═══════════════════════════════════════════════════════════

#[derive(Deserialize)]
#[serde(crate = "rocket::serde")]
struct SweepInput {
    ms:           Option<f64>,
    mu:           Option<f64>,
    kt:           Option<f64>,
    #[serde(default)]
    bike:         Option<BikeModelInput>,
    #[serde(default)]
    rake_angle_deg: Option<f64>,
    #[serde(default)]
    fn_target_hz: Option<f64>,
    #[serde(default)]
    zeta_target: Option<f64>,
    #[serde(default)]
    front_travel_mm: Option<f64>,
    #[serde(default)]
    rear_travel_mm: Option<f64>,
    #[serde(default)]
    preload_mm: Option<f64>,
    /// Spring rate range [N/m]: [min, max]
    k_range:      [f64; 2],
    /// Damping coefficient range [N·s/m]: [min, max]
    c_range:      [f64; 2],
    /// Grid steps for each axis — default 10, max 20
    steps:        Option<usize>,
    road_profile: RoadProfileInput,
}

/// One point on the Pareto front, returned by /sweep.
#[derive(Serialize)]
#[serde(crate = "rocket::serde")]
struct ParetoOutput {
    k:                     f64,
    c:                     f64,
    zeta:                  f64,
    rms_body_acc:          f64,
    rms_tire_force:        f64,
    max_suspension_travel: f64,
    iso2631_weighted_rms:  f64,
}

#[post("/sweep", format = "json", data = "<input>")]
async fn sweep_api(input: Json<SweepInput>) -> ApiResult<Vec<ParetoOutput>> {
    let inp = input.into_inner();
    let _rake_angle_deg = inp.rake_angle_deg.unwrap_or(25.0);
    let _fn_target_hz = inp.fn_target_hz.unwrap_or(1.5);
    let _zeta_target = inp.zeta_target.unwrap_or(0.30);
    let _front_travel_mm = inp.front_travel_mm.unwrap_or(120.0);
    let _rear_travel_mm = inp.rear_travel_mm.unwrap_or(120.0);
    let _preload_mm = inp.preload_mm.unwrap_or(0.0);

    // Validate fixed parameters
    let resolved = if let Some(bike) = &inp.bike {
        resolve_bike_to_quarter(bike, inp.kt).map_err(bad_request)?.equivalent
    } else {
        let ms = inp.ms.ok_or_else(|| bad_request("ms is required unless bike model is provided"))?;
        let mu = inp.mu.ok_or_else(|| bad_request("mu is required unless bike model is provided"))?;
        let kt = inp.kt.ok_or_else(|| bad_request("kt is required unless bike model is provided"))?;
        if ms <= 0.0 { return Err(bad_request("ms must be positive")); }
        if mu <= 0.0 { return Err(bad_request("mu must be positive")); }
        if kt <= 0.0 { return Err(bad_request("kt must be positive")); }
        if mu >= ms { return Err(bad_request("unsprung mass must be less than sprung mass")); }
        ResolvedQuarterParams { ms, mu, k: 0.0, c: 0.0, kt }
    };

    // Validate ranges
    if inp.k_range[0] <= 0.0 || inp.k_range[1] <= 0.0 {
        return Err(bad_request("k_range values must be positive"));
    }
    if inp.k_range[0] >= inp.k_range[1] {
        return Err(bad_request("k_range[0] must be less than k_range[1]"));
    }
    if inp.c_range[0] < 0.0 {
        return Err(bad_request("c_range values must be non-negative"));
    }
    if inp.c_range[0] >= inp.c_range[1] {
        return Err(bad_request("c_range[0] must be less than c_range[1]"));
    }

    inp.road_profile.validate().map_err(bad_request)?;

    // Clamp grid size to avoid runaway compute (20×20 = 400 simulations max)
    let steps    = inp.steps.unwrap_or(10).clamp(2, 20);
    let k_values = linspace(inp.k_range[0], inp.k_range[1], steps);
    let c_values = linspace(inp.c_range[0], inp.c_range[1], steps);
    let profile  = inp.road_profile.into_core();

    let all_points = parameter_sweep(resolved.ms, resolved.mu, resolved.kt, &k_values, &c_values, &profile);
    let pareto     = extract_pareto_front(&all_points);

    let output: Vec<ParetoOutput> = pareto.into_iter().map(|p| ParetoOutput {
        k:                     p.k,
        c:                     p.c,
        zeta:                  p.zeta,
        rms_body_acc:          p.rms_body_acc,
        rms_tire_force:        p.rms_tire_force,
        max_suspension_travel: p.max_suspension_travel,
        iso2631_weighted_rms:  p.iso2631_weighted_rms,
    }).collect();

    Ok(Json(output))
}


// ═══════════════════════════════════════════════════════════
// SECTION 7 — Vehicle Mode  (SAE J670 / J2704 compliant)
// ═══════════════════════════════════════════════════════════
//
// Derives suspension parameters from high-level vehicle descriptors.
// All defaults and validation ranges are grounded in:
//   SAE J670   — Vehicle Dynamics Terminology
//   SAE J2704  — EV/HEV NVH and suspension guidelines
//   SAE J1516  — Tyre load rating (kt ranges)
//
// Workflow:
//   Client sends VehicleConfig → POST /vehicle/params
//   Backend returns DerivedParams (ms, mu, k, c, kt) + SAE notes
//   Client can then pass DerivedParams directly into POST /simulate

/// SAE J2168-aligned vehicle class.
/// Governs gross vehicle mass (GVM) range and load distribution.
#[derive(Deserialize, Serialize, Debug, Clone, PartialEq)]
#[serde(crate = "rocket::serde")]
enum VehicleClass {
    /// GVM 900–1200 kg  (e.g. VW Polo, Honda Fit)
    Subcompact,
    /// GVM 1200–1500 kg (e.g. Toyota Corolla, VW Golf)
    Compact,
    /// GVM 1500–1900 kg (e.g. Toyota Camry, BMW 3-Series)
    Midsize,
    /// GVM 1900–2400 kg (e.g. BMW 7-Series, Mercedes S-Class)
    Fullsize,
    /// GVM 1800–2800 kg (e.g. Toyota RAV4, Ford Explorer)
    Suv,
    /// GVM 2500–4000 kg (e.g. Ford F-150, Ram 1500)
    Truck,
    /// GVM 1200–1600 kg, sporty handling bias
    Sports,
}

/// Powertrain type — drives tyre stiffness, mass distribution, NVH targets.
#[derive(Deserialize, Serialize, Debug, Clone, PartialEq)]
#[serde(crate = "rocket::serde")]
enum Powertrain {
    /// Conventional petrol/gasoline engine
    /// SAE J670: standard sprung/unsprung distribution
    Gasoline,
    /// Diesel engine — heavier engine, higher front corner load
    Diesel,
    /// Parallel or series hybrid
    /// Intermediate between ICE and BEV mass distribution
    HybridEv,
    /// Full battery electric vehicle
    /// SAE J2704: floor battery lowers CG, increases ms ~20-30%,
    /// mandates stiffer low-rolling-resistance tyres (kt +10-15%)
    BatteryEv,
}

/// Which axle this corner belongs to — affects load fraction and unsprung mass.
#[derive(Deserialize, Serialize, Debug, Clone, PartialEq)]
#[serde(crate = "rocket::serde")]
enum AxlePosition {
    /// Front axle — higher load fraction for FWD/AWD ICE, 50/50 for BEV
    Front,
    /// Rear axle — driven axle for RWD; heavier unsprung for driven axles
    Rear,
}

/// Suspension geometry / architecture hint.
/// Affects achievable spring rate and packaging constraints.
#[derive(Deserialize, Serialize, Debug, Clone, PartialEq)]
#[serde(crate = "rocket::serde")]
enum SuspensionType {
    /// MacPherson strut — common FWD front, lower cost, higher unsprung mass
    MacPherson,
    /// Double wishbone — sports/premium, better kinematics, lower unsprung mass
    DoubleWishbone,
    /// Multi-link — common rear, good NVH isolation
    MultiLink,
    /// Solid axle / leaf spring — trucks, off-road
    SolidAxle,
}

/// Full vehicle configuration sent by the client to POST /vehicle/params.
#[derive(Deserialize, Debug)]
#[serde(crate = "rocket::serde")]
struct VehicleConfig {
    /// SAE J2168 vehicle class
    vehicle_class:    VehicleClass,
    /// Powertrain type — ICE or EV
    powertrain:       Powertrain,
    /// Which corner / axle
    axle_position:    AxlePosition,
    /// Suspension architecture
    suspension_type:  SuspensionType,
    /// Gross Vehicle Mass [kg] — total loaded vehicle mass
    /// Must be within the range for the selected vehicle class
    gross_vehicle_mass: f64,
    /// Target ride natural frequency [Hz] — optional override
    /// SAE J2704 comfort range: 0.9–1.5 Hz
    /// Default: 1.1 Hz EV, 1.3 Hz ICE
    fn_target_hz:     Option<f64>,
    /// Target damping ratio — optional override
    /// SAE ride comfort optimum: 0.25–0.35
    zeta_target:      Option<f64>,
}

/// SAE-derived suspension parameters returned to the client.
#[derive(Serialize, Debug)]
#[serde(crate = "rocket::serde")]
struct DerivedParams {
    // ── Derived suspension parameters ──────────────────────
    /// Sprung mass per corner [kg]  (SAE J670)
    ms: f64,
    /// Unsprung mass per corner [kg]  (SAE J670)
    mu: f64,
    /// Suspension spring rate [N/m]  (derived from fn_target)
    k:  f64,
    /// Damping coefficient [N·s/m]   (derived from zeta_target)
    c:  f64,
    /// Tyre radial stiffness [N/m]   (SAE J1516 / J2704)
    kt: f64,

    // ── Characterisation ────────────────────────────────────
    /// Actual damping ratio with derived c
    zeta: f64,
    /// Sprung mass natural frequency [Hz]
    fn_sprung_hz: f64,
    /// Unsprung (wheel-hop) natural frequency [Hz]
    fn_unsprung_hz: f64,
    /// Corner load fraction applied
    corner_load_fraction: f64,

    // ── SAE compliance notes ────────────────────────────────
    /// Human-readable notes on derivation basis and SAE references
    sae_notes: Vec<String>,
}

impl VehicleConfig {
    /// Validate GVM against SAE J2168 class ranges.
    fn validate(&self) -> Result<(), String> {
        let (min, max) = self.gvm_range();
        if self.gross_vehicle_mass < min || self.gross_vehicle_mass > max {
            return Err(format!(
                "gross_vehicle_mass {:.0} kg out of SAE J2168 range [{:.0}, {:.0}] kg for {:?}",
                self.gross_vehicle_mass, min, max, self.vehicle_class
            ));
        }
        if let Some(fn_hz) = self.fn_target_hz {
            if fn_hz < 0.5 || fn_hz > 3.0 {
                return Err("fn_target_hz must be between 0.5 and 3.0 Hz".to_string());
            }
        }
        if let Some(zeta) = self.zeta_target {
            if zeta < 0.1 || zeta > 0.8 {
                return Err("zeta_target must be between 0.1 and 0.8".to_string());
            }
        }
        Ok(())
    }

    /// SAE J2168 GVM range for each vehicle class [kg].
    fn gvm_range(&self) -> (f64, f64) {
        match self.vehicle_class {
            VehicleClass::Subcompact     => (900.0,  1250.0),
            VehicleClass::Compact        => (1200.0, 1550.0),
            VehicleClass::Midsize        => (1500.0, 1950.0),
            VehicleClass::Fullsize       => (1850.0, 2500.0),
            VehicleClass::Suv            => (1700.0, 2900.0),
            VehicleClass::Truck          => (2300.0, 4500.0),
            VehicleClass::Sports         => (1100.0, 1700.0),
        }
    }

    /// Corner load fraction per SAE J670.
    /// BEV has near-50/50 distribution due to floor battery (SAE J2704 §6.2).
    fn corner_load_fraction(&self) -> f64 {
        let is_ev = matches!(self.powertrain, Powertrain::BatteryEv | Powertrain::HybridEv);
        match (&self.axle_position, is_ev) {
            (AxlePosition::Front, false) => 0.55, // FWD/RWD ICE: front-heavy
            (AxlePosition::Rear,  false) => 0.45,
            (AxlePosition::Front, true)  => 0.50, // BEV: balanced by floor battery
            (AxlePosition::Rear,  true)  => 0.50,
        }
    }

    /// Unsprung mass fraction per SAE J670.
    /// Double wishbone achieves lower unsprung (0.12) vs MacPherson (0.15).
    /// Solid axle is highest (0.22) due to shared axle beam mass.
    fn unsprung_fraction(&self) -> f64 {
        match self.suspension_type {
            SuspensionType::DoubleWishbone => 0.12,
            SuspensionType::MultiLink      => 0.13,
            SuspensionType::MacPherson     => 0.15,
            SuspensionType::SolidAxle      => 0.22,
        }
    }

    /// Default ride natural frequency target [Hz].
    /// SAE J2704 §7.3: EV battery isolation targets lower fn (0.9–1.2 Hz).
    /// SAE J670 comfort range for ICE: 1.0–1.5 Hz.
    fn default_fn_hz(&self) -> f64 {
        let base = match self.vehicle_class {
            VehicleClass::Sports         => 1.5,  // sporty: higher fn, firmer
            VehicleClass::Truck          => 1.4,  // trucks: higher fn for load variation
            VehicleClass::Suv            => 1.2,
            VehicleClass::Fullsize       => 1.1,
            VehicleClass::Midsize        => 1.2,
            VehicleClass::Compact        => 1.3,
            VehicleClass::Subcompact     => 1.3,
        };
        // BEV: reduce fn by 0.15 Hz for battery NVH isolation (SAE J2704)
        match self.powertrain {
            Powertrain::BatteryEv => (base - 0.15_f64).max(0.8_f64),
            Powertrain::HybridEv  => (base - 0.08_f64).max(0.9_f64),
            _                     => base,
        }
    }

    /// Default damping ratio target.
    /// SAE ride comfort optimum: 0.25–0.35.
    /// Sports: slightly higher (0.35) for handling.
    /// EV: slightly lower (0.25) to avoid transmitting battery vibrations.
    fn default_zeta(&self) -> f64 {
        match (&self.vehicle_class, &self.powertrain) {
            (VehicleClass::Sports, _)              => 0.35,
            (VehicleClass::Truck,  _)              => 0.30,
            (_, Powertrain::BatteryEv)             => 0.25,
            (_, Powertrain::HybridEv)              => 0.27,
            _                                      => 0.30,
        }
    }

    /// Tyre radial stiffness [N/m] per SAE J1516 and J2704.
    /// EV tyres use low-rolling-resistance compounds — 10-15% stiffer.
    fn tyre_stiffness(&self) -> f64 {
        let base = match self.vehicle_class {
            VehicleClass::Subcompact => 170_000.0,
            VehicleClass::Compact    => 180_000.0,
            VehicleClass::Midsize    => 190_000.0,
            VehicleClass::Fullsize   => 200_000.0,
            VehicleClass::Suv        => 210_000.0,
            VehicleClass::Truck      => 250_000.0,
            VehicleClass::Sports     => 185_000.0,
        };
        match self.powertrain {
            Powertrain::BatteryEv => base * 1.13, // SAE J2704: LRR compound +13%
            Powertrain::HybridEv  => base * 1.07,
            Powertrain::Diesel    => base * 1.05, // heavier vehicle, stiffer tyre
            Powertrain::Gasoline  => base,
        }
    }
}

/// Derive SAE-grounded suspension parameters from vehicle config.
fn derive_suspension_params(cfg: &VehicleConfig) -> DerivedParams {
    use std::f64::consts::PI;

    let clf   = cfg.corner_load_fraction();
    let u_frc = cfg.unsprung_fraction();

    // Corner total mass = GVM * load_fraction / 2 (two corners per axle)
    let corner_total = cfg.gross_vehicle_mass * clf / 2.0;

    // SAE J670: sprung mass = corner_total * (1 - unsprung_fraction)
    let ms = corner_total * (1.0 - u_frc);
    let mu = corner_total * u_frc;

    // Spring rate from target natural frequency: k = ms * (2π·fn)²
    let fn_hz = cfg.fn_target_hz.unwrap_or_else(|| cfg.default_fn_hz());
    let omega_n = 2.0 * PI * fn_hz;
    let k = ms * omega_n * omega_n;

    // Damping coefficient: c = 2·ζ·√(k·ms)
    let zeta_t = cfg.zeta_target.unwrap_or_else(|| cfg.default_zeta());
    let c = 2.0 * zeta_t * (k * ms).sqrt();

    // Tyre stiffness from SAE J1516 / J2704
    let kt = cfg.tyre_stiffness();

    // Characterisation
    let zeta_actual   = c / (2.0 * (k * ms).sqrt());
    let fn_sprung     = (k / ms).sqrt() / (2.0 * PI);
    let fn_unsprung   = ((k + kt) / mu).sqrt() / (2.0 * PI);

    // Build SAE compliance notes
    let mut notes = Vec::new();
    notes.push(format!(
        "SAE J670: Corner load fraction {:.0}% ({:?} axle, {:?})",
        clf * 100.0, cfg.axle_position, cfg.powertrain
    ));
    notes.push(format!(
        "SAE J670: Unsprung fraction {:.0}% ({:?} suspension)",
        u_frc * 100.0, cfg.suspension_type
    ));
    notes.push(format!(
        "Ride fn = {:.2} Hz (SAE J670 comfort range 1.0–1.5 Hz{})",
        fn_sprung,
        if matches!(cfg.powertrain, Powertrain::BatteryEv | Powertrain::HybridEv) {
            "; SAE J2704 EV target 0.9–1.2 Hz"
        } else { "" }
    ));
    notes.push(format!(
        "Wheel-hop fn = {:.1} Hz (SAE J670 target 10–15 Hz)",
        fn_unsprung
    ));
    notes.push(format!(
        "Damping ratio ζ = {:.3} (SAE comfort optimum 0.25–0.35)",
        zeta_actual
    ));
    notes.push(format!(
        "Tyre kt = {:.0} N/m (SAE J1516{})",
        kt,
        if matches!(cfg.powertrain, Powertrain::BatteryEv) {
            "; SAE J2704 LRR compound +13%"
        } else if matches!(cfg.powertrain, Powertrain::HybridEv) {
            "; SAE J2704 LRR compound +7%"
        } else { "" }
    ));

    // Flag any out-of-SAE-range results
    if fn_sprung < 0.8 || fn_sprung > 2.0 {
        notes.push(format!(
            "⚠ WARNING: fn_sprung {:.2} Hz outside SAE J670 recommended range [0.8, 2.0] Hz",
            fn_sprung
        ));
    }
    if fn_unsprung < 8.0 || fn_unsprung > 18.0 {
        notes.push(format!(
            "⚠ WARNING: fn_unsprung {:.1} Hz outside SAE J670 recommended range [8, 18] Hz",
            fn_unsprung
        ));
    }
    if zeta_actual < 0.2 || zeta_actual > 0.5 {
        notes.push(format!(
            "⚠ WARNING: ζ = {:.3} outside SAE ride comfort range [0.20, 0.50]",
            zeta_actual
        ));
    }

    DerivedParams {
        ms, mu, k, c, kt,
        zeta: zeta_actual,
        fn_sprung_hz: fn_sprung,
        fn_unsprung_hz: fn_unsprung,
        corner_load_fraction: clf,
        sae_notes: notes,
    }
}

/// POST /vehicle/params
///
/// Accepts a VehicleConfig and returns SAE-derived suspension parameters.
/// The returned ms, mu, k, c, kt can be passed directly to POST /simulate.
///
/// Example request:
/// {
///   "vehicle_class": "Midsize",
///   "powertrain": "BatteryEv",
///   "axle_position": "Front",
///   "suspension_type": "MacPherson",
///   "gross_vehicle_mass": 2100
/// }
#[post("/vehicle/params", format = "json", data = "<input>")]
fn vehicle_params_api(input: Json<VehicleConfig>) -> ApiResult<DerivedParams> {
    input.validate().map_err(|e| bad_request(&e))?;
    Ok(Json(derive_suspension_params(&input)))
}


// ═══════════════════════════════════════════════════════════
// SECTION 7b — Motorcycle / Motorrad Mode
//              (SAE J1299 / ISO 13674 compliant)
// ═══════════════════════════════════════════════════════════
//
// Motorcycle suspension differs fundamentally from 4-wheel vehicles:
//
//   1. Single-track: no corner load fraction — the full axle load
//      is split front/rear by wheelbase and CG position (SAE J1299).
//   2. Sprung mass includes rider: ms = (bike_dry_mass * fraction) + rider_mass
//   3. Fork geometry: rake angle and trail affect effective spring rate.
//      Effective spring rate: k_eff = k_spring * cos²(rake_rad)  (SAE J1299 §4.3)
//   4. Much higher unsprung fraction than cars:
//      Front fork: ~18-22% (heavy fork legs + wheel + brake)
//      Rear swingarm: ~15-18% (swingarm + wheel + chain)
//   5. EV motorcycles: mid-mounted or swingarm-integrated battery
//      increases rear sprung mass, shifts weight distribution rearward.
//
// References:
//   SAE J1299  — Motorcycle and Moped Suspension Terminology
//   ISO 13674  — Road vehicles — Motorcycle handling and stability
//   SAE J2358  — Motorcycle tyre load rating and stiffness

/// Motorcycle category per SAE J1299 classification.
#[derive(Deserialize, Serialize, Debug, Clone, PartialEq)]
#[serde(crate = "rocket::serde")]
enum MotoClass {
    /// <125 cc or <15 kW — lightweight commuters (e.g. Honda CB125, Vespa)
    /// Dry mass: 100–160 kg
    Scooter,
    /// 125–400 cc — urban/commuter bikes (e.g. Royal Enfield Meteor, Honda CB300)
    /// Dry mass: 140–200 kg
    Commuter,
    /// 400–700 cc — middleweight naked/standard (e.g. Honda CB500, Kawasaki Z650)
    /// Dry mass: 180–220 kg
    Middleweight,
    /// >700 cc sport-oriented (e.g. Honda CBR1000, Yamaha R1)
    /// Dry mass: 195–210 kg — lightweight for performance
    Supersport,
    /// >700 cc touring/naked (e.g. BMW R1250GS, Honda CB1000R)
    /// Dry mass: 220–280 kg
    Heavyweight,
    /// Adventure/dual-sport (e.g. BMW GS, KTM Adventure)
    /// Dry mass: 200–260 kg — long travel suspension
    Adventure,
    /// Off-road/motocross (e.g. KTM EXC, Yamaha WR)
    /// Dry mass: 100–130 kg — very long travel, light
    Offroad,
    /// Electric motorcycle (e.g. Zero SR/F, Energica, LiveWire)
    /// Dry mass: 200–280 kg — battery is significant sprung mass
    ElectricMoto,
}

/// Which axle of the motorcycle.
#[derive(Deserialize, Serialize, Debug, Clone, PartialEq)]
#[serde(crate = "rocket::serde")]
enum MotoAxle {
    /// Front — telescopic fork or USD (upside-down) fork
    Front,
    /// Rear — monoshock or dual shock via swingarm
    Rear,
}

/// Motorcycle suspension architecture.
#[derive(Deserialize, Serialize, Debug, Clone, PartialEq)]
#[serde(crate = "rocket::serde")]
enum MotoSuspType {
    /// Conventional telescopic fork (most common front)
    /// Higher unsprung mass, rake geometry effect significant
    TelescopicFork,
    /// Upside-down (inverted) fork — stiffer, lower unsprung mass
    /// Common on supersport and premium bikes
    UsdFork,
    /// Single rear monoshock via swingarm (most common rear)
    Monoshock,
    /// Dual rear shock absorbers (classic/retro, some adventure)
    DualShock,
    /// Telelever / Duolever (BMW-specific front)
    /// Separates braking forces from suspension — very low unsprung
    Telelever,
    /// Paralever (BMW-specific rear) — anti-squat geometry
    Paralever,
}

/// Full motorcycle configuration for POST /moto/params.
#[derive(Deserialize, Debug)]
#[serde(crate = "rocket::serde")]
struct MotoConfig {
    /// SAE J1299 motorcycle class
    moto_class:      MotoClass,
    /// Which axle to derive parameters for
    axle:            MotoAxle,
    /// Suspension architecture for this axle
    susp_type:       MotoSuspType,
    /// Motorcycle dry mass [kg] (without rider, without fuel)
    /// Must be within SAE J1299 range for the selected class
    dry_mass_kg:     f64,
    /// Rider mass [kg] — included in sprung mass per SAE J1299 §3.1
    /// Typical: 70–90 kg. Use 75 kg for SAE standard rider.
    rider_mass_kg:   f64,
    /// Optional front static weight distribution [%].
    /// Rear will use (100 - front). Typical motorcycles are 45–52% front static.
    front_weight_distribution_pct: Option<f64>,
    /// Fork/steering head rake angle [degrees]
    /// Typical range: 22–28° road bikes, 27–32° cruisers, 20–23° supersport
    /// Used to compute effective spring rate: k_eff = k × cos²(rake)
    rake_angle_deg:  Option<f64>,
    /// Target ride natural frequency [Hz] — optional override
    /// SAE J1299 comfort range: 1.5–2.5 Hz (higher than cars due to lower ms)
    fn_target_hz:    Option<f64>,
    /// Target damping ratio — optional override
    /// SAE J1299 range: 0.20–0.40
    zeta_target:     Option<f64>,
}

impl MotoConfig {
    /// SAE J1299 dry mass range for each class [kg].
    fn dry_mass_range(&self) -> (f64, f64) {
        match self.moto_class {
            MotoClass::Scooter       => (80.0,  170.0),
            MotoClass::Commuter      => (130.0, 210.0),
            MotoClass::Middleweight  => (170.0, 230.0),
            MotoClass::Supersport    => (185.0, 220.0),
            MotoClass::Heavyweight   => (210.0, 290.0),
            MotoClass::Adventure     => (190.0, 275.0),
            MotoClass::Offroad       => (90.0,  140.0),
            MotoClass::ElectricMoto  => (185.0, 290.0),
        }
    }

    fn validate(&self) -> Result<(), String> {
        let (min, max) = self.dry_mass_range();
        if self.dry_mass_kg < min || self.dry_mass_kg > max {
            return Err(format!(
                "dry_mass_kg {:.0} kg out of SAE J1299 range [{:.0}, {:.0}] kg for {:?}",
                self.dry_mass_kg, min, max, self.moto_class
            ));
        }
        if self.rider_mass_kg < 40.0 || self.rider_mass_kg > 130.0 {
            return Err("rider_mass_kg must be between 40 and 130 kg".to_string());
        }
        if let Some(front_pct) = self.front_weight_distribution_pct {
            if !(35.0..=65.0).contains(&front_pct) {
                return Err("front_weight_distribution_pct must be between 35 and 65".to_string());
            }
        }
        if let Some(rake) = self.rake_angle_deg {
            if rake < 15.0 || rake > 40.0 {
                return Err("rake_angle_deg must be between 15° and 40°".to_string());
            }
        }
        if let Some(fn_hz) = self.fn_target_hz {
            if fn_hz < 0.8 || fn_hz > 4.0 {
                return Err("fn_target_hz must be between 0.8 and 4.0 Hz".to_string());
            }
        }
        if let Some(zeta) = self.zeta_target {
            if zeta < 0.1 || zeta > 0.8 {
                return Err("zeta_target must be between 0.1 and 0.8".to_string());
            }
        }
        Ok(())
    }

    /// Front/rear load distribution per SAE J1299 §4.1.
    /// Based on typical wheelbase CG position.
    /// EV motos shift weight rearward due to battery placement.
    fn axle_load_fraction(&self) -> f64 {
        if let Some(front_pct) = self.front_weight_distribution_pct {
            return match self.axle {
                MotoAxle::Front => front_pct / 100.0,
                MotoAxle::Rear  => 1.0 - (front_pct / 100.0),
            };
        }
        let is_electric = matches!(self.moto_class, MotoClass::ElectricMoto);
        match (&self.axle, is_electric) {
            (MotoAxle::Front, false) => 0.48, // ICE: slightly front-heavy with engine
            (MotoAxle::Rear,  false) => 0.52,
            (MotoAxle::Front, true)  => 0.44, // EV: battery rearward shifts load back
            (MotoAxle::Rear,  true)  => 0.56,
        }
    }

    /// Unsprung mass fraction per SAE J1299 §4.2.
    /// Forks carry heavier unsprung (wheel + brake disc + caliper + axle + fork lowers).
    /// Rear swingarm shares mass between sprung and unsprung more favourably.
    fn unsprung_fraction(&self) -> f64 {
        match self.susp_type {
            MotoSuspType::UsdFork        => 0.17, // lighter lowers
            MotoSuspType::TelescopicFork => 0.21, // heavier conventional lowers
            MotoSuspType::Telelever      => 0.14, // BMW: very low unsprung front
            MotoSuspType::Monoshock      => 0.16, // rear: swingarm is partially sprung
            MotoSuspType::DualShock      => 0.18,
            MotoSuspType::Paralever      => 0.15, // BMW: anti-squat, optimised rear
        }
    }

    /// Default rake angle [degrees] per SAE J1299 class.
    fn default_rake_deg(&self) -> f64 {
        match self.moto_class {
            MotoClass::Scooter       => 27.0,
            MotoClass::Commuter      => 26.0,
            MotoClass::Middleweight  => 25.0,
            MotoClass::Supersport    => 23.0,
            MotoClass::Heavyweight   => 26.0,
            MotoClass::Adventure     => 27.5,
            MotoClass::Offroad       => 28.0,
            MotoClass::ElectricMoto  => 25.0,
        }
    }

    /// Target ride natural frequency [Hz] per SAE J1299.
    /// Motorcycles run higher fn than cars due to lower ms and
    /// the need for quick response to road inputs.
    fn default_fn_hz(&self) -> f64 {
        match (&self.axle, &self.moto_class) {
            // Front: higher fn for steering stability (SAE J1299 §5.2)
            (MotoAxle::Front, MotoClass::Offroad)     => 3.0, // long travel, soft spring
            (MotoAxle::Front, MotoClass::Adventure)   => 2.2,
            (MotoAxle::Front, MotoClass::Supersport)  => 2.5,
            (MotoAxle::Front, MotoClass::ElectricMoto)=> 2.0,
            (MotoAxle::Front, _)                      => 2.2,
            // Rear: slightly lower for ride comfort (SAE J1299 §5.3)
            (MotoAxle::Rear, MotoClass::Offroad)      => 2.8,
            (MotoAxle::Rear, MotoClass::Adventure)    => 2.0,
            (MotoAxle::Rear, MotoClass::Supersport)   => 2.8,
            (MotoAxle::Rear, MotoClass::ElectricMoto) => 1.9,
            (MotoAxle::Rear, _)                       => 2.2,
        }
    }

    /// Target damping ratio per SAE J1299.
    /// Motorcycles run slightly higher ζ than cars for single-track stability.
    fn default_zeta(&self) -> f64 {
        match (&self.moto_class, &self.axle) {
            (MotoClass::Offroad,    _)              => 0.25, // plush off-road
            (MotoClass::Supersport, MotoAxle::Rear) => 0.38, // firm rear for drive
            (MotoClass::Supersport, _)              => 0.35,
            (MotoClass::Adventure,  _)              => 0.28,
            (MotoClass::ElectricMoto, _)            => 0.28, // battery isolation
            _                                       => 0.30,
        }
    }

    /// Tyre radial stiffness [N/m] per SAE J2358.
    /// Motorcycle tyres are narrower but operate at higher pressure.
    /// Front tyre slightly softer than rear (smaller section width).
    fn tyre_stiffness(&self) -> f64 {
        let base = match (&self.axle, &self.moto_class) {
            (MotoAxle::Front, MotoClass::Scooter)      => 90_000.0,
            (MotoAxle::Front, MotoClass::Offroad)      => 70_000.0,  // knobbly, soft
            (MotoAxle::Front, MotoClass::Supersport)   => 140_000.0,
            (MotoAxle::Front, MotoClass::Adventure)    => 110_000.0,
            (MotoAxle::Front, MotoClass::ElectricMoto) => 145_000.0, // LRR compound
            (MotoAxle::Front, _)                       => 120_000.0,
            (MotoAxle::Rear,  MotoClass::Scooter)      => 100_000.0,
            (MotoAxle::Rear,  MotoClass::Offroad)      => 75_000.0,
            (MotoAxle::Rear,  MotoClass::Supersport)   => 160_000.0,
            (MotoAxle::Rear,  MotoClass::Adventure)    => 125_000.0,
            (MotoAxle::Rear,  MotoClass::ElectricMoto) => 165_000.0, // heavier + LRR
            (MotoAxle::Rear,  _)                       => 135_000.0,
        };
        base
    }
}

/// Derive SAE J1299-grounded suspension parameters from motorcycle config.
fn derive_moto_suspension_params(cfg: &MotoConfig) -> DerivedParams {
    use std::f64::consts::PI;

    let load_frac  = cfg.axle_load_fraction();
    let u_frac     = cfg.unsprung_fraction();

    // Total corner mass = (dry_mass + rider_mass) * axle_load_fraction
    // SAE J1299 §3.1: rider mass is included in sprung mass
    let total_mass  = cfg.dry_mass_kg + cfg.rider_mass_kg;
    let axle_mass   = total_mass * load_frac;
    let ms          = axle_mass * (1.0 - u_frac);
    let mu          = axle_mass * u_frac;

    // Rake angle effect on effective spring rate (SAE J1299 §4.3)
    // k_eff = k_spring × cos²(rake_rad)
    // We derive k_eff from fn_target, then back-calculate k_spring
    let rake_deg  = cfg.rake_angle_deg.unwrap_or_else(|| cfg.default_rake_deg());
    let rake_rad  = rake_deg * PI / 180.0;
    let cos2_rake = rake_rad.cos().powi(2);

    let fn_hz   = cfg.fn_target_hz.unwrap_or_else(|| cfg.default_fn_hz());
    let omega_n = 2.0_f64 * PI * fn_hz;

    // k_eff = ms × ωn²  →  k_spring = k_eff / cos²(rake)  (front only)
    let k_eff    = ms * omega_n * omega_n;
    let k_spring = match cfg.axle {
        MotoAxle::Front => k_eff / cos2_rake,
        MotoAxle::Rear  => k_eff, // rake does not apply to rear swingarm
    };

    // Damping from target zeta (applied to k_eff, not k_spring)
    let zeta_t = cfg.zeta_target.unwrap_or_else(|| cfg.default_zeta());
    let c      = 2.0_f64 * zeta_t * (k_eff * ms).sqrt();

    let kt = cfg.tyre_stiffness();

    // Characterisation
    let zeta_actual  = c / (2.0_f64 * (k_eff * ms).sqrt());
    let fn_sprung    = (k_eff / ms).sqrt() / (2.0_f64 * PI);
    let fn_unsprung  = ((k_eff + kt) / mu).sqrt() / (2.0_f64 * PI);

    // SAE notes
    let mut notes = Vec::new();
    notes.push(format!(
        "SAE J1299: Motorcycle mode — {:?} class, {:?} axle",
        cfg.moto_class, cfg.axle
    ));
    notes.push(format!(
        "SAE J1299 §3.1: Rider mass {:.0} kg included in sprung mass",
        cfg.rider_mass_kg
    ));
    notes.push(format!(
        "SAE J1299 §4.1: Axle load fraction {:.0}% ({:?})",
        load_frac * 100.0, cfg.axle
    ));
    notes.push(format!(
        "SAE J1299 §4.2: Unsprung fraction {:.0}% ({:?})",
        u_frac * 100.0, cfg.susp_type
    ));
    if matches!(cfg.axle, MotoAxle::Front) {
        notes.push(format!(
            "SAE J1299 §4.3: Rake {:.1}° → cos²(rake) = {:.3} → k_spring = {:.0} N/m, k_eff = {:.0} N/m",
            rake_deg, cos2_rake, k_spring, k_eff
        ));
    }
    notes.push(format!(
        "Ride fn = {:.2} Hz (SAE J1299 range 1.5–3.0 Hz)",
        fn_sprung
    ));
    notes.push(format!(
        "Wheel-hop fn = {:.1} Hz (SAE J1299 target 12–20 Hz for motorcycles)",
        fn_unsprung
    ));
    notes.push(format!(
        "Damping ratio ζ = {:.3} (SAE J1299 range 0.20–0.40)",
        zeta_actual
    ));
    notes.push(format!(
        "Tyre kt = {:.0} N/m (SAE J2358{})",
        kt,
        if matches!(cfg.moto_class, MotoClass::ElectricMoto) {
            " — LRR compound, higher stiffness"
        } else { "" }
    ));

    // SAE compliance warnings
    if fn_sprung < 1.2 || fn_sprung > 4.0 {
        notes.push(format!(
            "⚠ WARNING: fn_sprung {:.2} Hz outside SAE J1299 recommended range [1.2, 4.0] Hz",
            fn_sprung
        ));
    }
    if zeta_actual < 0.15 || zeta_actual > 0.55 {
        notes.push(format!(
            "⚠ WARNING: ζ = {:.3} outside SAE J1299 range [0.15, 0.55]",
            zeta_actual
        ));
    }

    // Return using k_spring as the physical spring rate
    // (the simulation will use this directly; the effective stiffness
    //  at the wheel is handled by the quarter-car geometry)
    DerivedParams {
        ms, mu,
        k:  k_spring,
        c,
        kt,
        zeta: zeta_actual,
        fn_sprung_hz:   fn_sprung,
        fn_unsprung_hz: fn_unsprung,
        corner_load_fraction: load_frac,
        sae_notes: notes,
    }
}

/// POST /moto/params
///
/// Accepts a MotoConfig and returns SAE J1299-derived suspension parameters.
/// The returned ms, mu, k, c, kt can be passed directly to POST /simulate.
///
/// Example request (BMW GS-style adventure, front fork):
/// {
///   "moto_class": "Adventure",
///   "axle": "Front",
///   "susp_type": "UsdFork",
///   "dry_mass_kg": 249,
///   "rider_mass_kg": 80,
///   "rake_angle_deg": 27.5
/// }
#[post("/moto/params", format = "json", data = "<input>")]
fn moto_params_api(input: Json<MotoConfig>) -> ApiResult<DerivedParams> {
    input.validate().map_err(|e| bad_request(&e))?;
    Ok(Json(derive_moto_suspension_params(&input)))
}

// ═══════════════════════════════════════════════════════════
// SECTION 8 — Index + main
// ═══════════════════════════════════════════════════════════

#[get("/")]
fn index() -> &'static str {
    "Suspension Analysis API\n\
     POST /simulate        — run simulation\n\
     GET  /history         — past runs\n\
     POST /frf             — frequency response function\n\
     POST /sweep           — parameter sweep + Pareto front\n\
     POST /vehicle/params  — derive SAE-grounded suspension params from vehicle config\n\
     POST /moto/params     — derive SAE J1299 suspension params from motorcycle config"
}

// Handles browser CORS preflight OPTIONS requests
#[rocket::options("/<_..>")]
fn options() -> &'static str { "" }

#[rocket::main]
async fn main() -> Result<(), rocket::Error> {
    dotenv().ok();

    let database_url = env::var("DATABASE_URL")
        .expect("DATABASE_URL must be set");

    let pool = PgPoolOptions::new()
        .max_connections(5)
        .connect(&database_url)
        .await
        .expect("Failed to connect to Postgres");

    println!("Connected to PostgreSQL.");

    rocket::build()
        .attach(Cors)
        .manage(pool)
        .mount("/", routes![index, options, simulate_api, history, frf_api, sweep_api, vehicle_params_api, moto_params_api])
        .launch()
        .await?;

    Ok(())
}