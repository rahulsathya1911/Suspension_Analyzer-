//! File export: CSV, JSON output files per CONTRACTS §4.
//!
//! PRD §5.8: comfort_dataset.csv, fusion_summary.json,
//! timeseries/<id>.json, run_meta.json


use std::path::Path;
use anyhow::Result;
use serde_json::{json, Value};

use crate::config::{VehicleConfig, SensorConfig, PerturbationRanges};
use crate::ScenarioResult;

/// Round a float to n significant digits.
pub fn round_sig(x: f64, n: u32) -> f64 {
    if x == 0.0 || !x.is_finite() { return x; }
    let d = (x.abs().log10().ceil()) as i32;
    let power = 10.0_f64.powi(n as i32 - d);
    (x * power).round() / power
}

/// Write a single timeseries JSON file.
pub fn write_timeseries(result: &ScenarioResult, out_dir: &Path) -> Result<()> {
    let ts_dir = out_dir.join("timeseries");
    std::fs::create_dir_all(&ts_dir)?;
    let path = ts_dir.join(format!("{}.json", result.scenario_id));
    let json = serde_json::to_string_pretty(result)?;
    std::fs::write(path, json)?;
    Ok(())
}

/// CSV row for comfort_dataset.csv.
pub struct CsvRow {
    pub scenario_id: String,
    pub split: String,
    pub road_class: String,
    pub speed_mps: f64,
    pub ms_kg: f64,
    pub preset: String,
    pub c_nsm: f64,
    pub dropout_pct: f64,
    pub seed: u64,
    pub p_phys: f64,
    pub y_ref: f64,
    pub cov_trace: f64,
    pub dropout_frac: f64,
    pub band_lo: f64,
    pub band_mid: f64,
    pub band_hi: f64,
    pub rms_tyre_force_n: f64,
    pub max_travel_mm: f64,
    pub bottom_out: bool,
}

/// Write comfort_dataset.csv.
pub fn write_comfort_csv(rows: &[CsvRow], out_dir: &Path) -> Result<()> {
    let path = out_dir.join("comfort_dataset.csv");
    let mut wtr = csv::Writer::from_path(path)?;

    wtr.write_record([
        "scenario_id", "split", "road_class", "speed_mps", "ms_kg", "preset",
        "c_nsm", "dropout_pct", "seed", "p_phys", "y_ref", "cov_trace",
        "dropout_frac", "band_lo", "band_mid", "band_hi",
        "rms_tyre_force_n", "max_travel_mm", "bottom_out",
    ])?;

    for r in rows {
        wtr.write_record(&[
            r.scenario_id.clone(),
            r.split.clone(),
            r.road_class.clone(),
            format!("{}", r.speed_mps),
            format!("{}", r.ms_kg),
            r.preset.clone(),
            format!("{}", r.c_nsm),
            format!("{}", r.dropout_pct),
            format!("{}", r.seed),
            format!("{:.6}", r.p_phys),
            format!("{:.6}", r.y_ref),
            format!("{:.6}", r.cov_trace),
            format!("{:.6}", r.dropout_frac),
            format!("{:.6}", r.band_lo),
            format!("{:.6}", r.band_mid),
            format!("{:.6}", r.band_hi),
            format!("{:.4}", r.rms_tyre_force_n),
            format!("{:.4}", r.max_travel_mm),
            format!("{}", if r.bottom_out { 1 } else { 0 }),
        ])?;
    }
    wtr.flush()?;
    Ok(())
}

/// Write fusion_summary.json (CONTRACTS §4.2).
pub fn write_fusion_summary(
    summary: &Value,
    out_dir: &Path,
) -> Result<()> {
    let path = out_dir.join("fusion_summary.json");
    let json = serde_json::to_string_pretty(summary)?;
    std::fs::write(path, json)?;
    Ok(())
}

/// Write run_meta.json (CONTRACTS §4.7).
pub fn write_run_meta(
    vehicle: &VehicleConfig,
    sensors: &[SensorConfig],
    perturbation_ranges: &PerturbationRanges,
    filter_params: &Value,
    out_dir: &Path,
) -> Result<()> {
    let meta = json!({
        "fixture": false,
        "git_commit": "",
        "vehicle": vehicle,
        "sensors": sensors,
        "filter": filter_params,
        "perturbation_ranges": perturbation_ranges,
        "seeds": {"train": [0, 9], "test": [100, 104]}
    });

    let path = out_dir.join("run_meta.json");
    let json = serde_json::to_string_pretty(&meta)?;
    std::fs::write(path, json)?;
    Ok(())
}

