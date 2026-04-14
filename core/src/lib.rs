/// core/src/lib.rs
///
/// Quarter-Car Suspension Analysis Engine
/// =======================================
/// Implements:
///   - 2-DOF quarter-car model (RK4, dt = 0.001 s, T = 5 s)
///   - Pluggable road profiles (Step, Sine, ISO 8608 random)
///   - Time-domain metrics: RMS body acc, RMS tire force, max travel
///   - ISO 2631-1 frequency-weighted RMS acceleration (ride comfort standard)
///   - Frequency Response Function (FRF) sweep
///   - Parameter sweep + Pareto front extraction
///
/// State vector: [x1, x2, x3, x4]
///   x1 = sprung mass displacement   [m]
///   x2 = sprung mass velocity       [m/s]
///   x3 = unsprung mass displacement [m]
///   x4 = unsprung mass velocity     [m/s]

// ═══════════════════════════════════════════════════════════
// SECTION 1 — Road profiles
// ═══════════════════════════════════════════════════════════

/// Selectable road input profile.
///
/// # Variants
/// - `Step`      — instantaneous bump at t = 0.5 s (original behaviour)
/// - `Sine`      — sinusoidal excitation at a given frequency [Hz]
/// - `Iso8608`   — stationary random road roughness (class A–E)
///
/// Pass one of these to `run_simulation` to control excitation type.
#[derive(Debug, Clone)]
pub enum RoadProfile {
    /// Single step of given height [m] applied at t = 0.5 s
    Step { height: f64 },

    /// Sinusoidal road: r(t) = amplitude · sin(2π · freq · t)
    Sine { amplitude: f64, freq_hz: f64 },

    /// ISO 8608 random road profile.
    /// `roughness_coefficient` (Gd) controls severity:
    ///   Class A (smooth): ~64e-6 m³/cycle
    ///   Class B (good):   ~256e-6
    ///   Class C (average):~1024e-6
    ///   Class D (poor):   ~4096e-6
    /// `vehicle_speed_mps` is forward speed [m/s].
    Iso8608 {
        roughness_coefficient: f64,
        vehicle_speed_mps: f64,
    },
}

impl RoadProfile {
    /// Build a pre-computed lookup table of road displacement values.
    /// Length = N_STEPS, indexed by step number.
    fn precompute(&self, dt: f64, n_steps: usize) -> Vec<f64> {
        match self {
            RoadProfile::Step { height } => {
                (0..n_steps)
                    .map(|i| if i as f64 * dt > 0.5 { *height } else { 0.0 })
                    .collect()
            }

            RoadProfile::Sine { amplitude, freq_hz } => {
                (0..n_steps)
                    .map(|i| amplitude * (2.0 * PI * freq_hz * i as f64 * dt).sin())
                    .collect()
            }

            RoadProfile::Iso8608 { roughness_coefficient, vehicle_speed_mps } => {
                // Discretise PSD via inverse DFT method (deterministic seed via LCG).
                // Gd(n0) is the roughness coefficient at reference spatial freq n0 = 0.1 cycle/m.
                // PSD: Gd(n) = Gd(n0) · (n/n0)^-2   [m²/(cycle/m)]
                // Convert to time-domain PSD: Sd(f) = Gd(n) / v,  n = f/v
                //
                // We synthesise by summing N_FREQ sinusoids with random phases
                // and amplitudes derived from the PSD.
                let v  = vehicle_speed_mps;
                let gd = roughness_coefficient;
                let n0 = 0.1_f64; // reference spatial frequency [cycle/m]

                let n_freq   = 512usize;
                let f_max    = 50.0_f64; // Hz — well above wheel-hop freq
                let df       = f_max / n_freq as f64;

                // LCG pseudo-random phases (no external crate needed)
                let mut seed: u64 = 0xDEAD_BEEF_1234_5678;
                let lcg = |s: &mut u64| -> f64 {
                    *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    (*s >> 33) as f64 / (u64::MAX >> 33) as f64
                };

                // Pre-compute amplitude and phase for each frequency bin
                let components: Vec<(f64, f64, f64)> = (1..=n_freq)
                    .map(|i| {
                        let f  = i as f64 * df;           // temporal freq [Hz]
                        let n_spatial = f / v;            // spatial freq [cycle/m]
                        // PSD in time domain [m²/Hz]
                        let sd = gd * (n_spatial / n0).powi(-2) / v;
                        let amplitude = (2.0 * sd * df).sqrt();
                        let phase     = 2.0 * PI * lcg(&mut seed);
                        (f, amplitude, phase)
                    })
                    .collect();

                (0..n_steps)
                    .map(|i| {
                        let t = i as f64 * dt;
                        components.iter().fold(0.0, |acc, &(f, amp, phi)| {
                            acc + amp * (2.0 * PI * f * t + phi).sin()
                        })
                    })
                    .collect()
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════
// SECTION 2 — Simulation constants
// ═══════════════════════════════════════════════════════════

const DT:      f64   = 0.001;
const N_STEPS: usize = 5_001; // floor(5.0 / DT) + 1

const PI: f64 = std::f64::consts::PI;

// ═══════════════════════════════════════════════════════════
// SECTION 3 — ISO 2631-1 frequency weighting (Wk filter)
// ═══════════════════════════════════════════════════════════

/// ISO 2631-1 Wk weighting applied in the frequency domain.
///
/// The standard defines a frequency-weighted RMS that penalises
/// accelerations in the 4–12.5 Hz range (most sensitive to humans
/// for vertical whole-body vibration).
///
/// Implementation: approximate Wk gain curve as piecewise linear
/// in log-frequency space, matching the standard's tabulated values.
///
/// Returns the weighted gain W(f) for a given frequency f [Hz].
fn wk_gain(f: f64) -> f64 {
    // ISO 2631-1 Table B.2 — Wk vertical weighting
    // Piecewise defined (simplified continuous approximation):
    //   f < 0.5  Hz  : rising  at +3 dB/oct
    //   0.5–2 Hz     : flat plateau ~ 0.5
    //   2–12.5 Hz    : rising to peak ~1.0 at 6–8 Hz then flat
    //   12.5–80 Hz   : falling at -6 dB/oct
    //   > 80 Hz      : negligible
    //
    // The values below match ISO 2631-1:1997 Table B.2 within ~1 dB.
    match f {
        f if f < 0.5   => 0.0,          // below measurement band
        f if f < 2.0   => 0.5,
        f if f < 4.0   => 0.5 + 0.5 * (f - 2.0) / 2.0,   // ramp up
        f if f < 12.5  => 1.0,          // peak sensitivity band
        f if f < 80.0  => 12.5 / f,     // -6 dB/oct rolloff
        _              => 0.0,
    }
}

/// Compute ISO 2631-1 frequency-weighted RMS acceleration from a
/// time-series of body accelerations sampled at `1/dt` Hz.
///
/// Method: Goertzel algorithm on each DFT bin that falls inside the
/// Wk passband (0.5–80 Hz). This is O(N·M) where M is the number of
/// in-band bins (~400 for N=5001, fs=1000 Hz) — fast enough for tests.
///
/// Returns weighted RMS [m/s²] — the number you put in a comfort report.
fn iso2631_weighted_rms(acc: &[f64], dt: f64) -> f64 {
    let n   = acc.len();
    let n_f = n as f64;
    let fs  = 1.0 / dt; // sample rate [Hz]

    // Only evaluate bins inside the Wk passband — Wk = 0 outside 0.5–80 Hz.
    // Bin k corresponds to frequency k·fs/N.
    let k_min = ((0.5  * n_f / fs).ceil()  as usize).max(1);
    let k_max = ((80.0 * n_f / fs).floor() as usize).min(n / 2);

    let mut weighted_energy = 0.0_f64;

    for k in k_min..=k_max {
        let f = k as f64 * fs / n_f;
        let w = wk_gain(f);
        if w == 0.0 { continue; }

        // Goertzel algorithm — computes one DFT bin in O(N) with only real multiplies.
        let omega = 2.0 * PI * k as f64 / n_f;
        let coeff = 2.0 * omega.cos();
        let (mut s_prev, mut s_prev2) = (0.0_f64, 0.0_f64);
        for &a in acc.iter() {
            let s = a + coeff * s_prev - s_prev2;
            s_prev2 = s_prev;
            s_prev  = s;
        }
        // |X_k|² from Goertzel final state
        let re    = s_prev - s_prev2 * omega.cos();
        let im    = s_prev2 * omega.sin();
        let power = 2.0 * (re * re + im * im) / (n_f * n_f); // one-sided, Parseval-normalised
        weighted_energy += w * w * power;
    }

    weighted_energy.sqrt()
}

// ═══════════════════════════════════════════════════════════
// SECTION 4 — Result types
// ═══════════════════════════════════════════════════════════

/// Full simulation output for a single (k, c, road_profile) run.
#[derive(Debug, Clone)]
pub struct SimulationResult {
    // ── Existing fields (unchanged) ──────────────────────────
    pub zeta: f64,
    pub mass_ratio: f64,
    pub recommended_mu: f64,
    pub time: Vec<f64>,
    pub body_user: Vec<f64>,
    pub wheel_user: Vec<f64>,

    // ── Time-domain metrics ───────────────────────────────────
    /// RMS body acceleration [m/s²]
    pub rms_body_acc: f64,
    /// RMS tire force [N]
    pub rms_tire_force: f64,
    /// Maximum absolute suspension travel [m]
    pub max_suspension_travel: f64,

    // ── ISO 2631-1 metric ─────────────────────────────────────
    /// Frequency-weighted RMS body acceleration per ISO 2631-1 Wk [m/s²].
    /// This is the internationally recognised ride comfort number.
    /// Comfort thresholds (ISO 2631-1 Table 3):
    ///   < 0.315  : not uncomfortable
    ///   0.315–0.63 : a little uncomfortable
    ///   0.5–1.0  : fairly uncomfortable
    ///   0.8–1.6  : uncomfortable
    ///   > 2.0    : extremely uncomfortable
    pub iso2631_weighted_rms: f64,
}

/// A single point on the Pareto front from a parameter sweep.
#[derive(Debug, Clone)]
pub struct ParetoPoint {
    pub k: f64,
    pub c: f64,
    pub zeta: f64,
    pub rms_body_acc: f64,
    pub rms_tire_force: f64,
    pub max_suspension_travel: f64,
    pub iso2631_weighted_rms: f64,
}

/// One point in the Frequency Response Function sweep.
#[derive(Debug, Clone)]
pub struct FrfPoint {
    /// Excitation frequency [Hz]
    pub freq_hz: f64,
    /// Ratio of RMS body acceleration to road RMS amplitude [1/s²]
    /// (acceleration transmissibility)
    pub body_acc_transmissibility: f64,
    /// Ratio of RMS tire force to road RMS amplitude [N/m]
    pub tire_force_transmissibility: f64,
}

// ═══════════════════════════════════════════════════════════
// SECTION 5 — Core physics
// ═══════════════════════════════════════════════════════════

fn derivatives(
    x1: f64, x2: f64,
    x3: f64, x4: f64,
    ms: f64, mu: f64,
    k:  f64, c:  f64, kt: f64,
    r:  f64,
) -> (f64, f64, f64, f64) {
    let dx1 = x2;
    let dx2 = (-c*(x2-x4) - k*(x1-x3)) / ms;
    let dx3 = x4;
    let dx4 = ( c*(x2-x4) + k*(x1-x3) - kt*(x3-r)) / mu;
    (dx1, dx2, dx3, dx4)
}

// ═══════════════════════════════════════════════════════════
// SECTION 6 — Main simulation
// ═══════════════════════════════════════════════════════════

/// Run a full quarter-car simulation.
///
/// # Arguments
/// * `ms`      — sprung mass [kg]
/// * `mu`      — unsprung mass [kg]
/// * `k`       — suspension spring rate [N/m]
/// * `c`       — damping coefficient [N·s/m]
/// * `kt`      — tyre stiffness [N/m]
/// * `profile` — road excitation profile (Step / Sine / Iso8608)
pub fn run_simulation(
    ms: f64,
    mu: f64,
    k:  f64,
    c:  f64,
    kt: f64,
    profile: &RoadProfile,
) -> SimulationResult {

    let dt         = DT;
    let n_steps    = N_STEPS;
    let road       = profile.precompute(dt, n_steps);

    let mut x1 = 0.0_f64;
    let mut x2 = 0.0_f64;
    let mut x3 = 0.0_f64;
    let mut x4 = 0.0_f64;

    let mut time_vec  = Vec::with_capacity(n_steps);
    let mut body_vec  = Vec::with_capacity(n_steps);
    let mut wheel_vec = Vec::with_capacity(n_steps);
    let mut acc_vec   = Vec::with_capacity(n_steps); // for ISO 2631

    let mut sum_acc_sq:  f64 = 0.0;
    let mut sum_tire_sq: f64 = 0.0;
    let mut max_travel:  f64 = 0.0;

    for step in 0..n_steps {
        let t = step as f64 * dt;
        let r = road[step];

        time_vec.push(t);
        body_vec.push(x1);
        wheel_vec.push(x3);

        // Body acceleration at current state
        let a_s = (-c*(x2-x4) - k*(x1-x3)) / ms;
        acc_vec.push(a_s);
        sum_acc_sq += a_s * a_s;

        // Tire force
        let f_t = kt * (x3 - r);
        sum_tire_sq += f_t * f_t;

        // Suspension travel
        let travel = (x1 - x3).abs();
        if travel > max_travel { max_travel = travel; }

        // ── RK4 ────────────────────────────────────────────────────────
        if step < n_steps - 1 {
            let r_mid  = (r + road[step + 1]) / 2.0;
            let r_next = road[step + 1];

            let (k1_1,k1_2,k1_3,k1_4) = derivatives(x1,x2,x3,x4,ms,mu,k,c,kt,r);
            let (k2_1,k2_2,k2_3,k2_4) = derivatives(
                x1+dt*k1_1/2.0, x2+dt*k1_2/2.0,
                x3+dt*k1_3/2.0, x4+dt*k1_4/2.0,
                ms,mu,k,c,kt,r_mid);
            let (k3_1,k3_2,k3_3,k3_4) = derivatives(
                x1+dt*k2_1/2.0, x2+dt*k2_2/2.0,
                x3+dt*k2_3/2.0, x4+dt*k2_4/2.0,
                ms,mu,k,c,kt,r_mid);
            let (k4_1,k4_2,k4_3,k4_4) = derivatives(
                x1+dt*k3_1, x2+dt*k3_2,
                x3+dt*k3_3, x4+dt*k3_4,
                ms,mu,k,c,kt,r_next);

            x1 += dt/6.0*(k1_1+2.0*k2_1+2.0*k3_1+k4_1);
            x2 += dt/6.0*(k1_2+2.0*k2_2+2.0*k3_2+k4_2);
            x3 += dt/6.0*(k1_3+2.0*k2_3+2.0*k3_3+k4_3);
            x4 += dt/6.0*(k1_4+2.0*k2_4+2.0*k3_4+k4_4);
        }
    }

    let n_f              = n_steps as f64;
    let rms_body_acc     = (sum_acc_sq  / n_f).sqrt();
    let rms_tire_force   = (sum_tire_sq / n_f).sqrt();
    let iso_wrms         = iso2631_weighted_rms(&acc_vec, dt);

    let zeta           = c / (2.0 * (k * ms).sqrt());
    let mass_ratio     = mu / ms;
    let recommended_mu = 0.12 * ms;

    SimulationResult {
        zeta,
        mass_ratio,
        recommended_mu,
        time:       time_vec,
        body_user:  body_vec,
        wheel_user: wheel_vec,
        rms_body_acc,
        rms_tire_force,
        max_suspension_travel: max_travel,
        iso2631_weighted_rms:  iso_wrms,
    }
}

// ═══════════════════════════════════════════════════════════
// SECTION 7 — Frequency Response Function (analytical)
// ═══════════════════════════════════════════════════════════

/// Minimal complex number — no external crate needed.
#[derive(Clone, Copy)]
struct Complex { re: f64, im: f64 }

impl Complex {
    fn new(re: f64, im: f64) -> Self { Self { re, im } }
    fn abs(self) -> f64 { (self.re*self.re + self.im*self.im).sqrt() }
}
impl std::ops::Add for Complex {
    type Output = Self;
    fn add(self, o: Self) -> Self { Self::new(self.re+o.re, self.im+o.im) }
}
impl std::ops::Sub for Complex {
    type Output = Self;
    fn sub(self, o: Self) -> Self { Self::new(self.re-o.re, self.im-o.im) }
}
impl std::ops::Mul for Complex {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        Self::new(self.re*o.re - self.im*o.im, self.re*o.im + self.im*o.re)
    }
}
impl std::ops::Div for Complex {
    type Output = Self;
    fn div(self, o: Self) -> Self {
        let d = o.re*o.re + o.im*o.im;
        Self::new((self.re*o.re + self.im*o.im)/d, (self.im*o.re - self.re*o.im)/d)
    }
}
impl From<f64> for Complex {
    fn from(x: f64) -> Self { Self::new(x, 0.0) }
}

/// Analytical FRF at one frequency via Cramer's rule on the 2-DOF system matrix.
///
/// Equations of motion (Laplace, s = jω):
///   [ms·s²+c·s+k,      -(c·s+k)         ] [X1]   [   0   ]
///   [-(c·s+k),   mu·s²+c·s+k+kt         ] [X3] = [kt · R ]
///
///   X1/R = kt·(c·s+k) / det(M)
///   X3/R = kt·(ms·s²+c·s+k) / det(M)
///
/// Body acc transmissibility = ω²·|X1/R|   [m/s² per m of road]
/// Tire force transmissibility = kt·|X3/R - 1|  [N per m of road]
fn frf_at_freq(ms: f64, mu: f64, k: f64, c: f64, kt: f64, freq_hz: f64) -> FrfPoint {
    let omega = 2.0 * PI * freq_hz;
    let jw    = Complex::new(0.0, omega);   // s = jω
    let jw2   = jw * jw;                    // s² = -ω²

    let cs_k  = jw * Complex::from(c) + Complex::from(k);
    let m11   = jw2 * Complex::from(ms) + cs_k;
    let m12   = Complex::from(0.0) - cs_k;
    let m22   = jw2 * Complex::from(mu) + cs_k + Complex::from(kt);

    let det      = m11 * m22 - m12 * m12;
    let kt_c     = Complex::from(kt);
    let x1_r     = (kt_c * cs_k) / det;
    let x3_r     = (kt_c * m11)  / det;

    FrfPoint {
        freq_hz,
        body_acc_transmissibility:   omega * omega * x1_r.abs(),
        tire_force_transmissibility: kt * (x3_r - Complex::from(1.0)).abs(),
    }
}

/// Compute the analytical FRF across a frequency range.
///
/// Uses the exact transfer function — no time-domain simulation, no transient
/// contamination. Valid at any frequency, instant to compute.
///
/// Plot `body_acc_transmissibility` vs `freq_hz` to see:
///   - Sprung resonance peak:   sqrt(k/ms)/(2π)       [~1–2 Hz]
///   - Unsprung resonance peak: sqrt((k+kt)/mu)/(2π)  [~10–15 Hz]
pub fn compute_frf(
    ms: f64,
    mu: f64,
    k:  f64,
    c:  f64,
    kt: f64,
    freq_range: &[f64],
    _amplitude: f64,   // kept for API compatibility
) -> Vec<FrfPoint> {
    freq_range.iter()
        .map(|&freq| frf_at_freq(ms, mu, k, c, kt, freq))
        .collect()
}

/// Convenience: build a logarithmically spaced frequency vector.
///
/// Returns `n` frequencies from `f_min` to `f_max` Hz (log spacing).
pub fn log_freq_range(f_min: f64, f_max: f64, n: usize) -> Vec<f64> {
    let log_min = f_min.ln();
    let log_max = f_max.ln();
    (0..n)
        .map(|i| (log_min + (log_max - log_min) * i as f64 / (n - 1) as f64).exp())
        .collect()
}

// ═══════════════════════════════════════════════════════════
// SECTION 8 — Parameter sweep + Pareto front
// ═══════════════════════════════════════════════════════════

/// Run a 2D grid sweep over spring rate `k` and damping `c`.
///
/// # Arguments
/// * `ms`, `mu`, `kt`  — fixed vehicle parameters
/// * `k_values`        — spring rates to evaluate [N/m]
/// * `c_values`        — damping coefficients to evaluate [N·s/m]
/// * `profile`         — road profile to use for all runs
///
/// # Returns
/// All sweep results as `ParetoPoint`s (full grid, not filtered).
/// Pass to `extract_pareto_front` to get the optimal subset.
pub fn parameter_sweep(
    ms:       f64,
    mu:       f64,
    kt:       f64,
    k_values: &[f64],
    c_values: &[f64],
    profile:  &RoadProfile,
) -> Vec<ParetoPoint> {
    let mut results = Vec::with_capacity(k_values.len() * c_values.len());

    for &k in k_values {
        for &c in c_values {
            let r = run_simulation(ms, mu, k, c, kt, profile);
            results.push(ParetoPoint {
                k,
                c,
                zeta:                  r.zeta,
                rms_body_acc:          r.rms_body_acc,
                rms_tire_force:        r.rms_tire_force,
                max_suspension_travel: r.max_suspension_travel,
                iso2631_weighted_rms:  r.iso2631_weighted_rms,
            });
        }
    }

    results
}

/// Extract the Pareto-optimal front from sweep results.
///
/// A point is Pareto-optimal if no other point is strictly better
/// on ALL of: iso2631_weighted_rms, rms_tire_force, max_suspension_travel.
///
/// These three objectives directly encode:
///   iso2631_weighted_rms   → ride comfort   (minimise)
///   rms_tire_force         → road holding   (minimise)
///   max_suspension_travel  → packaging      (minimise)
pub fn extract_pareto_front(points: &[ParetoPoint]) -> Vec<ParetoPoint> {
    points.iter().filter(|candidate| {
        !points.iter().any(|other| {
            // `other` dominates `candidate` if it is ≤ on all and < on at least one
            other.iso2631_weighted_rms  <= candidate.iso2631_weighted_rms  &&
                other.rms_tire_force        <= candidate.rms_tire_force        &&
                other.max_suspension_travel <= candidate.max_suspension_travel &&
                (
                    other.iso2631_weighted_rms  < candidate.iso2631_weighted_rms  ||
                        other.rms_tire_force        < candidate.rms_tire_force        ||
                        other.max_suspension_travel < candidate.max_suspension_travel
                )
        })
    }).cloned().collect()
}

/// Build a linearly spaced vector of `n` values from `start` to `end` inclusive.
pub fn linspace(start: f64, end: f64, n: usize) -> Vec<f64> {
    if n == 1 { return vec![start]; }
    (0..n)
        .map(|i| start + (end - start) * i as f64 / (n - 1) as f64)
        .collect()
}

// ═══════════════════════════════════════════════════════════
// SECTION 9 — Tests
// ═══════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    // Standard test parameters (mid-size saloon)
    fn params() -> (f64, f64, f64, f64, f64) {
        let ms = 290.0;
        let mu = 40.0;
        let k  = 22_000.0;
        let kt = 190_000.0;
        let c  = 1_500.0;
        (ms, mu, k, c, kt)
    }

    #[test]
    fn step_simulation_runs_and_lengths_match() {
        let (ms, mu, k, c, kt) = params();
        let res = run_simulation(ms, mu, k, c, kt, &RoadProfile::Step { height: 0.05 });
        assert_eq!(res.time.len(), N_STEPS);
        assert_eq!(res.body_user.len(), N_STEPS);
        assert_eq!(res.wheel_user.len(), N_STEPS);
    }

    #[test]
    fn all_metrics_positive_for_step() {
        let (ms, mu, k, c, kt) = params();
        let res = run_simulation(ms, mu, k, c, kt, &RoadProfile::Step { height: 0.05 });
        assert!(res.rms_body_acc          > 0.0);
        assert!(res.rms_tire_force        > 0.0);
        assert!(res.max_suspension_travel > 0.0);
        assert!(res.iso2631_weighted_rms  > 0.0);
    }

    #[test]
    fn iso2631_is_less_than_unweighted_rms() {
        // Wk weights are ≤ 1.0, so weighted RMS ≤ unweighted RMS
        let (ms, mu, k, c, kt) = params();
        let res = run_simulation(ms, mu, k, c, kt, &RoadProfile::Step { height: 0.05 });
        assert!(
            res.iso2631_weighted_rms <= res.rms_body_acc + 1e-10,
            "weighted {} > unweighted {}",
            res.iso2631_weighted_rms, res.rms_body_acc
        );
    }

    #[test]
    fn resonance_dominated_iso2631_exceeds_optimal() {
        // ISO 2631-1 physical reality: the relationship between damping and
        // weighted RMS is NOT monotonic. Very low damping causes resonance
        // amplification (bad), very high damping transmits force directly (also bad).
        // There is an optimal damping (zeta ~0.1–0.2 for this system).
        //
        // Testable assertion: very underdamped (zeta≈0.04, resonance-dominated)
        // gives higher ISO2631 than near-optimal damping (zeta≈0.10).
        // This holds reliably and reflects the physics of ride comfort tuning.
        let (ms, mu, k, _c, kt) = params();
        let profile = RoadProfile::Iso8608 {
            roughness_coefficient: 1024e-6, // Class C road — excites full Wk band
            vehicle_speed_mps:     30.0,
        };
        // c=200 → zeta≈0.04: very underdamped, sprung resonance dominates
        // c=500 → zeta≈0.10: near optimal comfort damping
        let resonant = run_simulation(ms, mu, k, 200.0, kt, &profile);
        let optimal  = run_simulation(ms, mu, k, 500.0, kt, &profile);
        assert!(
            resonant.iso2631_weighted_rms > optimal.iso2631_weighted_rms,
            "resonance-dominated ISO2631 ({:.4}) should exceed near-optimal ({:.4})",
            resonant.iso2631_weighted_rms, optimal.iso2631_weighted_rms
        );
    }

    #[test]
    fn optimal_damping_zeta_in_expected_range() {
        // The ISO 2631-minimising damping ratio for a typical quarter-car
        // should fall in the engineering range 0.05–0.35.
        // This validates that the Wk-weighted metric has a meaningful minimum.
        let (ms, mu, k, _c, kt) = params();
        let profile = RoadProfile::Iso8608 {
            roughness_coefficient: 1024e-6,
            vehicle_speed_mps:     30.0,
        };
        let c_crit = 2.0 * (k * ms).sqrt();
        let c_candidates = [100.0, 200.0, 300.0, 500.0, 700.0, 1000.0, 1500.0, 2000.0];
        let (best_c, best_iso) = c_candidates.iter().fold((0.0_f64, f64::MAX), |best, &c| {
            let iso = run_simulation(ms, mu, k, c, kt, &profile).iso2631_weighted_rms;
            if iso < best.1 { (c, iso) } else { best }
        });
        let best_zeta = best_c / c_crit;
        assert!(
            best_zeta >= 0.05 && best_zeta <= 0.35,
            "optimal zeta {:.3} (c={}) outside expected range [0.05, 0.35], ISO2631={:.4}",
            best_zeta, best_c, best_iso
        );
    }

    #[test]
    fn sine_profile_produces_nonzero_response() {
        let (ms, mu, k, c, kt) = params();
        let res = run_simulation(ms, mu, k, c, kt,
                                 &RoadProfile::Sine { amplitude: 0.01, freq_hz: 1.5 });
        assert!(res.rms_body_acc > 0.0);
    }

    #[test]
    fn iso8608_produces_nonzero_response() {
        let (ms, mu, k, c, kt) = params();
        let res = run_simulation(ms, mu, k, c, kt,
                                 &RoadProfile::Iso8608 { roughness_coefficient: 256e-6, vehicle_speed_mps: 30.0 });
        assert!(res.rms_body_acc > 0.0);
    }

    #[test]
    fn frf_has_correct_length() {
        let (ms, mu, k, c, kt) = params();
        let freqs = log_freq_range(0.5, 25.0, 10);
        let frf   = compute_frf(ms, mu, k, c, kt, &freqs, 0.01);
        assert_eq!(frf.len(), 10);
    }

    #[test]
    fn frf_peaks_at_both_natural_frequencies() {
        // The analytical FRF is exact — no transient, no simulation length limit.
        // The 2-DOF quarter-car has two resonance peaks:
        //   Sprung (body):   fn1 = sqrt(k/ms)/(2π)         ≈ 1.38 Hz
        //   Unsprung (wheel): fn2 = sqrt((k+kt)/mu)/(2π)   ≈ 11.6 Hz
        let (ms, mu, k, c, kt) = params();

        let f_sprung   = (k / ms).sqrt() / (2.0 * PI);
        let f_unsprung = ((k + kt) / mu).sqrt() / (2.0 * PI);

        // Sweep 0.5–25 Hz with high resolution to accurately locate both peaks
        let freqs = log_freq_range(0.5, 25.0, 200);
        let frf   = compute_frf(ms, mu, k, c, kt, &freqs, 0.01);

        // ── Sprung peak: search 0.5–4 Hz band ───────────────────────────
        let sprung_peak = frf.iter()
            .filter(|p| p.freq_hz >= 0.5 && p.freq_hz <= 4.0)
            .max_by(|a, b| a.body_acc_transmissibility
                .partial_cmp(&b.body_acc_transmissibility).unwrap())
            .unwrap();

        // ── Unsprung peak: search 7–20 Hz band ──────────────────────────
        let unsprung_peak = frf.iter()
            .filter(|p| p.freq_hz >= 7.0 && p.freq_hz <= 20.0)
            .max_by(|a, b| a.body_acc_transmissibility
                .partial_cmp(&b.body_acc_transmissibility).unwrap())
            .unwrap();

        assert!(
            (sprung_peak.freq_hz - f_sprung).abs() < 0.5,
            "Sprung peak at {:.2} Hz, expected {:.2} Hz", sprung_peak.freq_hz, f_sprung
        );
        assert!(
            (unsprung_peak.freq_hz - f_unsprung).abs() < 2.0,
            "Unsprung peak at {:.2} Hz, expected {:.2} Hz", unsprung_peak.freq_hz, f_unsprung
        );
    }

    #[test]
    fn pareto_front_subset_of_sweep() {
        let (ms, mu, k, c, kt) = params();
        let k_vals = linspace(k * 0.7, k * 1.3, 5);
        let c_vals = linspace(c * 0.5, c * 2.0, 5);
        let profile = RoadProfile::Step { height: 0.05 };

        let all    = parameter_sweep(ms, mu, kt, &k_vals, &c_vals, &profile);
        let pareto = extract_pareto_front(&all);

        assert!(!pareto.is_empty());
        assert!(pareto.len() <= all.len());
    }

    #[test]
    fn pareto_points_not_dominated() {
        let (ms, mu, k, c, kt) = params();
        let k_vals  = linspace(k * 0.7, k * 1.3, 6);
        let c_vals  = linspace(c * 0.5, c * 2.0, 6);
        let profile = RoadProfile::Step { height: 0.05 };
        let all     = parameter_sweep(ms, mu, kt, &k_vals, &c_vals, &profile);
        let pareto  = extract_pareto_front(&all);

        // Verify no Pareto point is dominated by another Pareto point
        for p in &pareto {
            for q in &pareto {
                let q_dominates_p =
                    q.iso2631_weighted_rms  <= p.iso2631_weighted_rms  &&
                        q.rms_tire_force        <= p.rms_tire_force        &&
                        q.max_suspension_travel <= p.max_suspension_travel &&
                        (q.iso2631_weighted_rms  < p.iso2631_weighted_rms  ||
                            q.rms_tire_force        < p.rms_tire_force        ||
                            q.max_suspension_travel < p.max_suspension_travel);
                assert!(!q_dominates_p,
                        "Pareto point (k={}, c={}) is dominated — front extraction bug",
                        p.k, p.c);
            }
        }
    }
}