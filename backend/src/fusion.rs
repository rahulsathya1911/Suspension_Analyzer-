//! FUSION routes: PRD C section 4.1 / docs/CONTRACTS.md section 6.
//!
//! Routes (all JSON, errors are `{"error": "..."}`):
//!   POST /fusion/run                           run one scenario live
//!   GET  /fusion/data/<name>                   fusion_summary | comfort_predictions |
//!                                              envelope | semiactive_ranking | run_meta
//!   GET  /fusion/data/timeseries/<scenario_id> <FUSION_OUT_DIR>/timeseries/<id>.json
//!   GET  /fusion/scenarios                     ids of the time series files present
//!
//! `ScenarioSpec`, `ScenarioResult` and `run_scenario` come from Person A's crate
//! (`suspension_fusion`, folder `fusion/`). The spec type is NOT redefined here, so the
//! HTTP body and the engine can never drift apart.
//!
//! Cargo.toml additions (backend):
//!   suspension_fusion = { path = "../fusion" }
//!   regex = "1"
//!   once_cell = "1"
//! Dev-only, for the tests below:
//!   tempfile = "3"
//!
//! Mount in main.rs next to the existing /simulate, /frf, /sweep:
//!   mod fusion;
//!   rocket::build()
//!       .manage(fusion::FusionConfig::from_env())
//!       .mount("/", fusion::routes())
//!       ...
//!
//! NOTE: the error type below is a best guess at the backend's style. If the existing
//! handlers already have an AppError / response wrapper, swap it in for `FusionError`
//! so the backend keeps one error style.

use std::fs;
use std::path::{Path, PathBuf};

use once_cell::sync::Lazy;
use regex::Regex;
use rocket::http::Status;
use rocket::response::status::Custom;
use rocket::serde::json::{json, Error as JsonError, Json, Value};
use rocket::{get, post, routes, Route, State};

use suspension_fusion::{run_scenario, ScenarioSpec};

/// Where `FUSION_OUT_DIR` resolves to, read once at launch so handlers do not touch the
/// environment per request and tests can each use their own directory.
pub struct FusionConfig {
    pub out_dir: PathBuf,
}

impl FusionConfig {
    pub fn from_env() -> Self {
        let out_dir = std::env::var("FUSION_OUT_DIR").unwrap_or_else(|_| "out/".to_string());
        Self { out_dir: PathBuf::from(out_dir) }
    }

    fn timeseries_dir(&self) -> PathBuf {
        self.out_dir.join("timeseries")
    }
}

// ---------- shared error type ----------

#[derive(Debug)]
pub struct FusionError {
    status: Status,
    message: String,
}

impl FusionError {
    fn unprocessable(message: impl Into<String>) -> Self {
        Self { status: Status::UnprocessableEntity, message: message.into() }
    }
    fn not_found() -> Self {
        Self { status: Status::NotFound, message: "not_found".to_string() }
    }
    fn internal(message: impl Into<String>) -> Self {
        Self { status: Status::InternalServerError, message: message.into() }
    }
}

impl<'r> rocket::response::Responder<'r, 'static> for FusionError {
    fn respond_to(self, req: &'r rocket::Request<'_>) -> rocket::response::Result<'static> {
        Custom(self.status, Json(json!({ "error": self.message }))).respond_to(req)
    }
}

type FusionResult<T> = Result<T, FusionError>;

// ---------- validation (CONTRACTS section 6) ----------

fn validate(spec: &ScenarioSpec) -> FusionResult<()> {
    let mut errors: Vec<&str> = Vec::new();

    if !matches!(spec.road_class.as_str(), "A" | "B" | "C" | "D") {
        errors.push("road_class must be one of A, B, C, D");
    }
    if spec.speed_mps.is_nan() || spec.speed_mps <= 0.0 {
        errors.push("speed_mps must be > 0");
    }
    if !(100.0..=1500.0).contains(&spec.ms_kg) {
        errors.push("ms_kg must be between 100 and 1500");
    }
    if spec.c_nsm.is_nan() || spec.c_nsm <= 0.0 {
        errors.push("c_nsm must be > 0");
    }
    if !(0.0..=50.0).contains(&spec.dropout_pct) {
        errors.push("dropout_pct must be between 0 and 50");
    }
    if !(5.0..=60.0).contains(&spec.duration_s) {
        errors.push("duration_s must be between 5 and 60");
    }
    // Optional vehicle overrides: if present they must be positive and finite.
    for (name, v) in [("k_nm", spec.k_nm), ("mu_kg", spec.mu_kg), ("kt_nm", spec.kt_nm)] {
        if let Some(x) = v {
            if !x.is_finite() || x <= 0.0 {
                errors.push(match name {
                    "k_nm" => "k_nm must be > 0",
                    "mu_kg" => "mu_kg must be > 0",
                    _ => "kt_nm must be > 0",
                });
            }
        }
    }

    if errors.is_empty() {
        Ok(())
    } else {
        Err(FusionError::unprocessable(errors.join("; ")))
    }
}

// ---------- POST /fusion/run ----------

/// The engine can take seconds for a 20 s scenario, so it runs on Rocket's blocking pool
/// instead of stalling the async workers (other requests keep being served during a live demo).
#[post("/fusion/run", data = "<spec>")]
pub async fn run(spec: Result<Json<ScenarioSpec>, JsonError<'_>>) -> FusionResult<Json<Value>> {
    // A malformed or incomplete body gets the contract's 422 + JSON error, not Rocket's HTML page.
    let spec = spec
        .map_err(|e| FusionError::unprocessable(format!("invalid request body: {e}")))?
        .into_inner();
    validate(&spec)?;

    let result = rocket::tokio::task::spawn_blocking(move || run_scenario(&spec))
        .await
        .map_err(|e| FusionError::internal(format!("scenario task failed: {e}")))?
        .map_err(|e| FusionError::internal(format!("scenario run failed: {e}")))?;

    let value = rocket::serde::json::serde_json::to_value(result)
        .map_err(|e| FusionError::internal(format!("failed to serialize result: {e}")))?;

    Ok(Json(value))
}

// ---------- GET /fusion/data/<name> ----------

#[derive(Debug, Clone, Copy)]
pub enum FusionDataset {
    FusionSummary,
    ComfortPredictions,
    Envelope,
    SemiactiveRanking,
    RunMeta,
}

impl FusionDataset {
    fn filename(self) -> &'static str {
        match self {
            Self::FusionSummary => "fusion_summary.json",
            Self::ComfortPredictions => "comfort_predictions.json",
            Self::Envelope => "envelope.json",
            Self::SemiactiveRanking => "semiactive_ranking.json",
            Self::RunMeta => "run_meta.json",
        }
    }
}

// A fixed enum (instead of a free string joined onto out_dir) rules out path traversal on
// this route by construction: only these five names can be expressed.
impl<'a> rocket::request::FromParam<'a> for FusionDataset {
    type Error = ();
    fn from_param(param: &'a str) -> Result<Self, Self::Error> {
        match param {
            "fusion_summary" => Ok(Self::FusionSummary),
            "comfort_predictions" => Ok(Self::ComfortPredictions),
            "envelope" => Ok(Self::Envelope),
            "semiactive_ranking" => Ok(Self::SemiactiveRanking),
            "run_meta" => Ok(Self::RunMeta),
            _ => Err(()),
        }
    }
}

fn read_json_file(path: &Path) -> FusionResult<Value> {
    let bytes = fs::read(path).map_err(|_| FusionError::not_found())?;
    rocket::serde::json::serde_json::from_slice(&bytes)
        .map_err(|e| FusionError::internal(format!("corrupt json at {}: {e}", path.display())))
}

// `Result<_, ()>` params: an unknown name reaches the handler as Err, so it can answer with the
// contract's JSON 404 instead of Rocket's default HTML 404 page.
#[get("/fusion/data/<name>")]
pub fn get_data(
    name: Result<FusionDataset, ()>,
    config: &State<FusionConfig>,
) -> FusionResult<Json<Value>> {
    let name = name.map_err(|_| FusionError::not_found())?;
    read_json_file(&config.out_dir.join(name.filename())).map(Json)
}

// ---------- GET /fusion/data/timeseries/<scenario_id> ----------

static SCENARIO_ID_RE: Lazy<Regex> =
    Lazy::new(|| Regex::new(r"^r[A-D]_v\d+_m\d+_c\d+_d\d+_s\d+$").expect("hardcoded valid regex"));

pub struct ScenarioId(String);

impl<'a> rocket::request::FromParam<'a> for ScenarioId {
    type Error = ();
    fn from_param(param: &'a str) -> Result<Self, Self::Error> {
        if SCENARIO_ID_RE.is_match(param) {
            Ok(ScenarioId(param.to_string()))
        } else {
            Err(())
        }
    }
}

#[get("/fusion/data/timeseries/<scenario_id>")]
pub fn get_timeseries(
    scenario_id: Result<ScenarioId, ()>,
    config: &State<FusionConfig>,
) -> FusionResult<Json<Value>> {
    let scenario_id = scenario_id.map_err(|_| FusionError::not_found())?;

    // Time series live in <FUSION_OUT_DIR>/timeseries/<id>.json (CONTRACTS 4.3).
    let dir = config.timeseries_dir();
    let path = dir.join(format!("{}.json", scenario_id.0));

    // Belt and braces on top of the regex (which already rejects `/` and `.`):
    // refuse to serve anything that resolves outside the timeseries folder.
    let canonical_dir = fs::canonicalize(&dir).map_err(|_| FusionError::not_found())?;
    let canonical_path = fs::canonicalize(&path).map_err(|_| FusionError::not_found())?;
    if !canonical_path.starts_with(&canonical_dir) {
        return Err(FusionError::not_found());
    }

    read_json_file(&canonical_path).map(Json)
}

// ---------- GET /fusion/scenarios ----------

#[get("/fusion/scenarios")]
pub fn list_scenarios(config: &State<FusionConfig>) -> FusionResult<Json<Value>> {
    let dir = config.timeseries_dir();

    // No folder yet (matrix not run) is not an error: the dashboard shows "No data yet".
    if !dir.is_dir() {
        return Ok(Json(json!({ "timeseries": Vec::<String>::new() })));
    }

    let entries = fs::read_dir(&dir)
        .map_err(|e| FusionError::internal(format!("cannot read {}: {e}", dir.display())))?;

    let mut ids: Vec<String> = entries
        .filter_map(|entry| entry.ok())
        .filter_map(|entry| {
            let file_name = entry.file_name();
            let name = file_name.to_str()?;
            let stem = name.strip_suffix(".json")?;
            SCENARIO_ID_RE.is_match(stem).then(|| stem.to_string())
        })
        .collect();

    ids.sort();

    Ok(Json(json!({ "timeseries": ids })))
}

// ---------- mounting ----------

pub fn routes() -> Vec<Route> {
    routes![run, get_data, get_timeseries, list_scenarios]
}

// ---------- tests (PRD C task C2: right status for good and bad input, including a
// traversal attempt) ----------

#[cfg(test)]
mod tests {
    use super::*;
    use rocket::http::{ContentType, Status};
    use rocket::local::blocking::Client;
    use std::fs;

    fn test_client(out_dir: &Path) -> Client {
        let rocket = rocket::build()
            .manage(FusionConfig { out_dir: out_dir.to_path_buf() })
            .mount("/", routes());
        Client::tracked(rocket).expect("valid rocket instance")
    }

    fn write_timeseries(dir: &Path, id: &str, body: &str) {
        let ts = dir.join("timeseries");
        fs::create_dir_all(&ts).unwrap();
        fs::write(ts.join(format!("{id}.json")), body).unwrap();
    }

    #[test]
    fn scenario_id_regex_accepts_well_formed_ids() {
        assert!(SCENARIO_ID_RE.is_match("rA_v20_m440_c1000_d0_s0"));
        assert!(SCENARIO_ID_RE.is_match("rD_v40_m700_c1800_d30_s100"));
    }

    #[test]
    fn scenario_id_regex_rejects_malformed_and_traversal_ids() {
        assert!(!SCENARIO_ID_RE.is_match("rE_v20_m440_c1000_d0_s0")); // bad road class
        assert!(!SCENARIO_ID_RE.is_match("../../etc/passwd"));
        assert!(!SCENARIO_ID_RE.is_match("rA_v20_m440_c1000_d0_s0/../../secret"));
        assert!(!SCENARIO_ID_RE.is_match("rA_v20_m440_c1000_d0_s0.json"));
    }

    // ----- POST /fusion/run -----

    #[test]
    fn run_rejects_out_of_range_values_with_422_and_json_error() {
        let dir = tempfile::tempdir().unwrap();
        let client = test_client(dir.path());

        let bad_body = json!({
            "road_class": "Z",
            "speed_mps": -1.0,
            "ms_kg": 10.0,
            "c_nsm": 1000.0,
            "dropout_pct": 90.0,
            "seed": 0,
            "duration_s": 1.0
        });

        let response = client
            .post("/fusion/run")
            .header(ContentType::JSON)
            .body(bad_body.to_string())
            .dispatch();

        assert_eq!(response.status(), Status::UnprocessableEntity);
        let body: Value = response.into_json().unwrap();
        assert!(body["error"].as_str().unwrap().contains("road_class"));
    }

    #[test]
    fn run_rejects_incomplete_body_with_422_and_json_error() {
        let dir = tempfile::tempdir().unwrap();
        let client = test_client(dir.path());

        let response = client
            .post("/fusion/run")
            .header(ContentType::JSON)
            .body(r#"{"road_class": "B"}"#)
            .dispatch();

        assert_eq!(response.status(), Status::UnprocessableEntity);
        let body: Value = response.into_json().unwrap();
        assert!(body["error"].is_string());
    }

    /// Calls the real engine, so it is ignored by default (a debug-build run can be slow).
    /// Run it with:  cargo test -p backend -- --ignored run_returns_200
    #[test]
    #[ignore]
    fn run_returns_200_for_valid_body() {
        let dir = tempfile::tempdir().unwrap();
        let client = test_client(dir.path());

        let ok_body = json!({
            "road_class": "B",
            "speed_mps": 20.0,
            "ms_kg": 440.0,
            "c_nsm": 1800.0,
            "dropout_pct": 5.0,
            "seed": 0,
            "duration_s": 5.0
        });

        let response = client
            .post("/fusion/run")
            .header(ContentType::JSON)
            .body(ok_body.to_string())
            .dispatch();

        assert_eq!(response.status(), Status::Ok);
        let body: Value = response.into_json().unwrap();
        assert!(body["t"].is_array());
        assert!(body["fused"].is_object());
    }

    // ----- GET /fusion/data/<name> -----

    #[test]
    fn get_data_404s_when_file_missing() {
        let dir = tempfile::tempdir().unwrap();
        let client = test_client(dir.path());

        let response = client.get("/fusion/data/fusion_summary").dispatch();
        assert_eq!(response.status(), Status::NotFound);
        let body: Value = response.into_json().unwrap();
        assert_eq!(body["error"], "not_found");
    }

    #[test]
    fn get_data_200s_when_file_present() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("run_meta.json"), r#"{"ok": true}"#).unwrap();
        let client = test_client(dir.path());

        let response = client.get("/fusion/data/run_meta").dispatch();
        assert_eq!(response.status(), Status::Ok);
        let body: Value = response.into_json().unwrap();
        assert_eq!(body["ok"], true);
    }

    #[test]
    fn get_data_unknown_name_is_json_404() {
        let dir = tempfile::tempdir().unwrap();
        let client = test_client(dir.path());

        let response = client.get("/fusion/data/passwd").dispatch();
        assert_eq!(response.status(), Status::NotFound);
        let body: Value = response.into_json().unwrap();
        assert_eq!(body["error"], "not_found");
    }

    // ----- GET /fusion/data/timeseries/<id> -----

    #[test]
    fn timeseries_200s_for_file_in_timeseries_folder() {
        let dir = tempfile::tempdir().unwrap();
        write_timeseries(dir.path(), "rB_v20_m440_c1800_d5_s0", r#"{"fs_hz": 100}"#);
        let client = test_client(dir.path());

        let response = client.get("/fusion/data/timeseries/rB_v20_m440_c1800_d5_s0").dispatch();
        assert_eq!(response.status(), Status::Ok);
        let body: Value = response.into_json().unwrap();
        assert_eq!(body["fs_hz"], 100);
    }

    #[test]
    fn timeseries_404s_when_missing() {
        let dir = tempfile::tempdir().unwrap();
        write_timeseries(dir.path(), "rB_v20_m440_c1800_d5_s0", "{}");
        let client = test_client(dir.path());

        let response = client.get("/fusion/data/timeseries/rA_v10_m300_c1000_d0_s0").dispatch();
        assert_eq!(response.status(), Status::NotFound);
    }

    #[test]
    fn timeseries_404s_on_traversal_attempts() {
        let dir = tempfile::tempdir().unwrap();
        // A secret one level above the timeseries folder, and a decoy file with a valid name.
        fs::write(dir.path().join("secret.json"), r#"{"secret": true}"#).unwrap();
        write_timeseries(dir.path(), "rB_v20_m440_c1800_d5_s0", "{}");
        let client = test_client(dir.path());

        for attempt in [
            "/fusion/data/timeseries/..%2Fsecret",
            "/fusion/data/timeseries/rB_v20_m440_c1800_d5_s0%2F..%2F..%2Fsecret",
            "/fusion/data/timeseries/..%2F..%2Fetc%2Fpasswd",
            "/fusion/data/timeseries/rB_v20_m440_c1800_d5_s0.json",
        ] {
            let response = client.get(attempt).dispatch();
            assert_eq!(response.status(), Status::NotFound, "attempt: {attempt}");
        }
    }

    // ----- GET /fusion/scenarios -----

    #[test]
    fn scenarios_lists_only_matching_timeseries_files() {
        let dir = tempfile::tempdir().unwrap();
        write_timeseries(dir.path(), "rA_v20_m440_c1000_d0_s0", "{}");
        write_timeseries(dir.path(), "not_a_scenario", "{}");
        fs::write(dir.path().join("fusion_summary.json"), "{}").unwrap();
        let client = test_client(dir.path());

        let response = client.get("/fusion/scenarios").dispatch();
        assert_eq!(response.status(), Status::Ok);
        let body: Value = response.into_json().unwrap();
        assert_eq!(body["timeseries"], json!(["rA_v20_m440_c1000_d0_s0"]));
    }

    #[test]
    fn scenarios_is_empty_list_when_folder_missing() {
        let dir = tempfile::tempdir().unwrap();
        let client = test_client(dir.path());

        let response = client.get("/fusion/scenarios").dispatch();
        assert_eq!(response.status(), Status::Ok);
        let body: Value = response.into_json().unwrap();
        assert_eq!(body["timeseries"], json!([]));
    }
}