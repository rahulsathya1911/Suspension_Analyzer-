//! Sensor fusion engine for AI-enabled suspension digital twin.
//!
//! Crate: `suspension_fusion`
//!
//! Signal chain: perturbed plant → sensors → sync → Kalman filter → comfort → export.

pub mod config;
pub mod plant;
pub mod sensors;
pub mod sync;
pub mod kf;
pub mod comfort;
pub mod metrics;
pub mod export;

use anyhow::{Result, bail};
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use nalgebra::SVector;

use config::{VehicleConfig, SensorConfig, road_class_gd};
use export::round_sig;

/// Scenario specification — CONTRACTS §5.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ScenarioSpec {
    pub road_class: String,
    pub speed_mps: f64,
    pub ms_kg: f64,
    pub c_nsm: f64,
    pub dropout_pct: f64,
    pub seed: u64,
    pub duration_s: f64,
    #[serde(default)]
    pub k_nm: Option<f64>,
    #[serde(default)]
    pub mu_kg: Option<f64>,
    #[serde(default)]
    pub kt_nm: Option<f64>,
}

impl ScenarioSpec {
    /// Build scenario ID per CONTRACTS §3.
    pub fn scenario_id(&self) -> String {
        format!(
            "r{}_v{}_m{}_c{}_d{}_s{}",
            self.road_class,
            self.speed_mps as i64,
            self.ms_kg as i64,
            self.c_nsm as i64,
            self.dropout_pct as i64,
            self.seed,
        )
    }
}

/// Scenario result — serialises to CONTRACTS §4.3 timeseries JSON.
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ScenarioResult {
    pub fixture: bool,
    pub scenario_id: String,
    pub scenario: ScenarioInfo,
    pub fs_hz: f64,
    pub t: Vec<f64>,
    pub truth: TruthBlock,
    pub raw: RawBlock,
    pub fused: FusedBlock,
    pub sigma: SigmaBlock,
    pub flags: FlagsBlock,
    pub metrics: MetricsBlock,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct ScenarioInfo {
    pub road_class: String,
    pub speed_mps: f64,
    pub ms_kg: f64,
    pub c_nsm: f64,
    pub preset: String,
    pub dropout_pct: f64,
    pub seed: u64,
    pub duration_s: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct TruthBlock {
    pub susp_travel_m: Vec<f64>,
    pub body_vel_mps: Vec<f64>,
    pub wheel_vel_mps: Vec<f64>,
    pub road_height_m: Vec<f64>,
    pub body_acc_mps2: Vec<f64>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct RawBlock {
    pub body_acc_mps2: Vec<Option<f64>>,
    pub wheel_acc_mps2: Vec<Option<f64>>,
    pub susp_travel_m: Vec<Option<f64>>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct FusedBlock {
    pub susp_travel_m: Vec<f64>,
    pub body_vel_mps: Vec<f64>,
    pub wheel_vel_mps: Vec<f64>,
    pub road_height_m: Vec<f64>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct SigmaBlock {
    pub susp_travel_m: Vec<f64>,
    pub body_vel_mps: Vec<f64>,
    pub wheel_vel_mps: Vec<f64>,
    pub road_height_m: Vec<f64>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct FlagsBlock {
    pub dropout: Vec<u8>,
    pub anomaly_rejected: Vec<u8>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct MetricsBlock {
    pub rmse_fused: RmseFused,
    pub rmse_raw: RmseRaw,
    pub p_phys: f64,
    pub y_ref: f64,
    pub dropout_frac: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct RmseFused {
    pub susp_travel_m: f64,
    pub body_vel_mps: f64,
    pub wheel_vel_mps: f64,
    pub road_height_m: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct RmseRaw {
    pub susp_travel_m: f64,
}

/// Run a complete fusion scenario.
///
/// CONTRACTS §5: `run_scenario(&ScenarioSpec) -> Result<ScenarioResult>`
pub fn run_scenario(spec: &ScenarioSpec) -> Result<ScenarioResult> {
    // Validate inputs
    if !["A", "B", "C", "D"].contains(&spec.road_class.as_str()) {
        bail!("Invalid road class: {}", spec.road_class);
    }
    if spec.speed_mps <= 0.0 {
        bail!("Speed must be positive");
    }
    if !(100.0..=1500.0).contains(&spec.ms_kg) {
        bail!("ms_kg must be in 100-1500");
    }
    if !(0.0..=50.0).contains(&spec.dropout_pct) {
        bail!("dropout_pct must be in 0-50");
    }
    if !(5.0..=60.0).contains(&spec.duration_s) {
        bail!("duration_s must be in 5-60");
    }

    let vehicle = VehicleConfig::nominal();
    let k = spec.k_nm.unwrap_or(vehicle.k_nm);
    let mu = spec.mu_kg.unwrap_or(vehicle.mu_kg);
    let kt = spec.kt_nm.unwrap_or(vehicle.kt_nm);
    let ms = spec.ms_kg;
    let c = spec.c_nsm;
    let travel_limit_m = vehicle.travel_limit_mm / 1000.0;
    let preset = vehicle.preset_name(c);

    let plant_dt = 0.001; // 1 kHz
    let fusion_fs = 200.0;
    let fusion_dt = 1.0 / fusion_fs;
    let output_fs = 100.0; // output at 100 Hz per CONTRACTS §4.3

    // Seeded RNG
    let mut rng = ChaCha8Rng::seed_from_u64(spec.seed);

    // ── 1. Generate road profile ──
    let gd = road_class_gd(&spec.road_class)
        .ok_or_else(|| anyhow::anyhow!("Unknown road class"))?;
    let n_plant = (spec.duration_s / plant_dt).round() as usize + 1;
    let road_profile = suspension_core::RoadProfile::Iso8608 {
        roughness_coefficient: gd,
        vehicle_speed_mps: spec.speed_mps,
        seed: spec.seed,
    };
    let road = suspension_core::generate_road_profile(&road_profile, plant_dt, n_plant);

    // ── 2. Run perturbed plant ──
    let pert = plant::PlantPerturbation::draw(
        &mut rng,
        config::PerturbationRanges::default().k_pct,
        config::PerturbationRanges::default().kt_pct,
        config::PerturbationRanges::default().asym_rebound_ratio,
    );
    let plant_perturbed = plant::run_perturbed(ms, mu, k, c, kt, travel_limit_m, &road, plant_dt, &pert);

    // ── 3. Simulate sensors ──
    let sensor_configs = SensorConfig::defaults();
    let sensor_out = sensors::simulate_sensors(
        &plant_perturbed, plant_dt, spec.speed_mps,
        &sensor_configs, spec.dropout_pct, &mut rng,
    );

    // ── 4. Synchronise ──
    let sync_out = sync::synchronise(&sensor_out);

    // ── 5. Run Kalman filter ──
    let kf_params = kf::KfParams::from_nominal(ms, mu, k, c, kt, fusion_dt);
    let meas_noise = kf::MeasNoise::default();
    let kf_out = kf::run_filter(&sync_out.frames, &kf_params, &meas_noise);

    // ── 6. Compute comfort ──
    // Interpolate fused road estimate to plant rate for p_phys
    let fused_road_at_fusion: Vec<f64> = kf_out.states.iter().map(|s| s[4]).collect();
    let zr_hat_plant = interpolate_to_plant_rate(&fused_road_at_fusion, fusion_fs, plant_dt, n_plant);

    let p_phys = comfort::compute_p_phys(
        &zr_hat_plant, ms, mu, k, c, kt, travel_limit_m, plant_dt, fusion_fs,
    );

    let y_ref = comfort::compute_y_ref(&plant_perturbed.body_acc, plant_dt, fusion_fs);

    // Band RMS of fused body acceleration
    let fused_body_acc: Vec<f64> = kf_out.states.iter().map(|s| {
        let zs = s[0]; let vs = s[1]; let zu = s[2]; let vu = s[3];
        (-k * (zs - zu) - c * (vs - vu)) / ms
    }).collect();
    let band_lo = comfort::band_rms(&fused_body_acc, fusion_fs, 0.5, 2.0);
    let band_mid = comfort::band_rms(&fused_body_acc, fusion_fs, 2.0, 8.0);
    let band_hi = comfort::band_rms(&fused_body_acc, fusion_fs, 8.0, 20.0);

    // ── 7. Compute metrics ──
    let n_fusion = kf_out.states.len();
    let decimate_plant = (1.0 / plant_dt / fusion_fs).round() as usize;

    let truth = metrics::TruthData {
        zs: decimate_vec(&plant_perturbed.zs, decimate_plant, n_fusion),
        vs: decimate_vec(&plant_perturbed.vs, decimate_plant, n_fusion),
        zu: decimate_vec(&plant_perturbed.zu, decimate_plant, n_fusion),
        vu: decimate_vec(&plant_perturbed.vu, decimate_plant, n_fusion),
        zr: decimate_vec(&plant_perturbed.road, decimate_plant, n_fusion),
    };

    // Open-loop nominal twin
    let plant_nominal = plant::run_nominal(ms, mu, k, c, kt, travel_limit_m, &road, plant_dt);
    let ol_states: Vec<SVector<f64, 5>> = (0..n_fusion).map(|i| {
        let pi = (i * decimate_plant).min(plant_nominal.zs.len() - 1);
        SVector::<f64, 5>::new(
            plant_nominal.zs[pi], plant_nominal.vs[pi],
            plant_nominal.zu[pi], plant_nominal.vu[pi],
            plant_nominal.road[pi],
        )
    }).collect();

    // Detection flags
    let anomaly_detected: Vec<bool> = (0..n_fusion).map(|i| {
        kf_out.gate_log.iter().any(|e| {
            let idx = (e.t / fusion_dt).round() as usize;
            idx == i
        })
    }).collect();

    let dropout_detected: Vec<bool> = sync_out.frames.iter().map(|f| {
        !f.body_acc_valid || !f.wheel_acc_valid
    }).collect();

    let scenario_metrics = metrics::compute_metrics(
        &kf_out, &truth,
        &sensor_out.susp_travel,
        Some(&ol_states),
        &sensor_out.fault_log,
        &anomaly_detected,
        &dropout_detected,
    );

    // ── 8. Build output at 100 Hz ──
    let output_decimate = (fusion_fs / output_fs).round() as usize; // 2
    let n_out = (n_fusion + output_decimate - 1) / output_decimate;

    let t_out: Vec<f64> = (0..n_out).map(|i| round_sig(i as f64 / output_fs, 5)).collect();

    let truth_out = TruthBlock {
        susp_travel_m: decimate_round(&truth.zs.iter().zip(truth.zu.iter())
            .map(|(zs, zu)| zs - zu).collect::<Vec<_>>(), output_decimate, n_out),
        body_vel_mps: decimate_round(&truth.vs, output_decimate, n_out),
        wheel_vel_mps: decimate_round(&truth.vu, output_decimate, n_out),
        road_height_m: decimate_round(&truth.zr, output_decimate, n_out),
        body_acc_mps2: decimate_round(&fused_body_acc, output_decimate, n_out),
    };

    let raw_out = RawBlock {
        body_acc_mps2: decimate_optional(&sensor_out.body_acc, output_decimate, n_out),
        wheel_acc_mps2: decimate_optional(&sensor_out.wheel_acc, output_decimate, n_out),
        susp_travel_m: decimate_optional(&sensor_out.susp_travel, output_decimate, n_out),
    };

    let fused_out = FusedBlock {
        susp_travel_m: decimate_round(&kf_out.states.iter().map(|s| s[0] - s[2]).collect::<Vec<_>>(), output_decimate, n_out),
        body_vel_mps: decimate_round(&kf_out.states.iter().map(|s| s[1]).collect::<Vec<_>>(), output_decimate, n_out),
        wheel_vel_mps: decimate_round(&kf_out.states.iter().map(|s| s[3]).collect::<Vec<_>>(), output_decimate, n_out),
        road_height_m: decimate_round(&kf_out.states.iter().map(|s| s[4]).collect::<Vec<_>>(), output_decimate, n_out),
    };

    let sigma_out = SigmaBlock {
        susp_travel_m: decimate_round(&kf_out.cov_diag.iter().map(|d| (d[0] + d[2]).sqrt()).collect::<Vec<_>>(), output_decimate, n_out),
        body_vel_mps: decimate_round(&kf_out.cov_diag.iter().map(|d| d[1].sqrt()).collect::<Vec<_>>(), output_decimate, n_out),
        wheel_vel_mps: decimate_round(&kf_out.cov_diag.iter().map(|d| d[3].sqrt()).collect::<Vec<_>>(), output_decimate, n_out),
        road_height_m: decimate_round(&kf_out.cov_diag.iter().map(|d| d[4].sqrt()).collect::<Vec<_>>(), output_decimate, n_out),
    };

    let flags_out = FlagsBlock {
        dropout: decimate_flags(&dropout_detected, output_decimate, n_out),
        anomaly_rejected: decimate_flags(&anomaly_detected, output_decimate, n_out),
    };

    // Dropout fraction
    let dropout_frac = dropout_detected.iter().filter(|&&d| d).count() as f64 / n_fusion as f64;

    // Mean cov trace
    let mean_cov_trace = kf_out.cov_trace.iter().sum::<f64>() / kf_out.cov_trace.len() as f64;

    // RMS tyre force from perturbed plant
    let n_plant_f = plant_perturbed.tyre_force.len() as f64;
    let rms_tyre = (plant_perturbed.tyre_force.iter().map(|f| f * f).sum::<f64>() / n_plant_f).sqrt();

    // Max travel from perturbed plant
    let max_travel_m = plant_perturbed.zs.iter().zip(plant_perturbed.zu.iter())
        .map(|(zs, zu)| (zs - zu).abs())
        .fold(0.0_f64, f64::max);

    let scenario_id = spec.scenario_id();

    Ok(ScenarioResult {
        fixture: false,
        scenario_id,
        scenario: ScenarioInfo {
            road_class: spec.road_class.clone(),
            speed_mps: spec.speed_mps,
            ms_kg: spec.ms_kg,
            c_nsm: spec.c_nsm,
            preset: preset.to_string(),
            dropout_pct: spec.dropout_pct,
            seed: spec.seed,
            duration_s: spec.duration_s,
        },
        fs_hz: output_fs,
        t: t_out,
        truth: truth_out,
        raw: raw_out,
        fused: fused_out,
        sigma: sigma_out,
        flags: flags_out,
        metrics: MetricsBlock {
            rmse_fused: RmseFused {
                susp_travel_m: round_sig(scenario_metrics.rmse_susp_travel.fused, 5),
                body_vel_mps: round_sig(scenario_metrics.rmse_body_vel.fused, 5),
                wheel_vel_mps: round_sig(scenario_metrics.rmse_wheel_vel.fused, 5),
                road_height_m: round_sig(scenario_metrics.rmse_road_height.fused, 5),
            },
            rmse_raw: RmseRaw {
                susp_travel_m: round_sig(scenario_metrics.rmse_susp_travel.raw.unwrap_or(0.0), 5),
            },
            p_phys: round_sig(p_phys, 5),
            y_ref: round_sig(y_ref, 5),
            dropout_frac: round_sig(dropout_frac, 5),
        },
    })
}

// ── Helper functions ──

fn interpolate_to_plant_rate(signal: &[f64], signal_fs: f64, plant_dt: f64, n_plant: usize) -> Vec<f64> {
    let mut result = Vec::with_capacity(n_plant);
    let signal_dt = 1.0 / signal_fs;
    for i in 0..n_plant {
        let t = i as f64 * plant_dt;
        let idx_f = t / signal_dt;
        let idx_lo = (idx_f.floor() as usize).min(signal.len().saturating_sub(2));
        let idx_hi = (idx_lo + 1).min(signal.len() - 1);
        let frac = idx_f - idx_lo as f64;
        result.push(signal[idx_lo] * (1.0 - frac) + signal[idx_hi] * frac);
    }
    result
}

fn decimate_vec(v: &[f64], step: usize, n_out: usize) -> Vec<f64> {
    (0..n_out).map(|i| {
        let idx = (i * step).min(v.len().saturating_sub(1));
        v[idx]
    }).collect()
}

fn decimate_round(v: &[f64], step: usize, n_out: usize) -> Vec<f64> {
    (0..n_out).map(|i| {
        let idx = (i * step).min(v.len().saturating_sub(1));
        round_sig(v[idx], 5)
    }).collect()
}

fn decimate_optional(v: &[Option<f64>], step: usize, n_out: usize) -> Vec<Option<f64>> {
    (0..n_out).map(|i| {
        let idx = (i * step).min(v.len().saturating_sub(1));
        v[idx].map(|x| round_sig(x, 5))
    }).collect()
}

fn decimate_flags(v: &[bool], step: usize, n_out: usize) -> Vec<u8> {
    (0..n_out).map(|i| {
        let idx = (i * step).min(v.len().saturating_sub(1));
        if v[idx] { 1 } else { 0 }
    }).collect()
}

#[cfg(test)]
mod tests {
    use serde_json::Value;
    use super::*;

    #[test]
    fn stub_scenario_is_schema_valid() {
        let spec = ScenarioSpec {
            road_class: "B".into(),
            speed_mps: 20.0,
            ms_kg: 440.0,
            c_nsm: 1800.0,
            dropout_pct: 5.0,
            seed: 0,
            duration_s: 20.0,
            k_nm: None,
            mu_kg: None,
            kt_nm: None,
        };
        let result = run_scenario(&spec).unwrap();
        assert!(!result.fixture);
        assert_eq!(result.fs_hz, 100.0);
        assert!(!result.t.is_empty());
        assert_eq!(result.t.len(), result.truth.susp_travel_m.len());
        assert_eq!(result.t.len(), result.fused.susp_travel_m.len());
        assert_eq!(result.t.len(), result.sigma.susp_travel_m.len());
        assert_eq!(result.t.len(), result.flags.dropout.len());

        // Verify scenario ID format
        assert_eq!(result.scenario_id, "rB_v20_m440_c1800_d5_s0");
    }

    #[test]
    fn rejects_invalid_road_class() {
        let spec = ScenarioSpec {
            road_class: "Z".into(),
            speed_mps: 20.0,
            ms_kg: 440.0,
            c_nsm: 1800.0,
            dropout_pct: 5.0,
            seed: 0,
            duration_s: 20.0,
            k_nm: None, mu_kg: None, kt_nm: None,
        };
        assert!(run_scenario(&spec).is_err());
    }

    #[test]
    fn result_serialises_to_valid_json() {
        let spec = ScenarioSpec {
            road_class: "A".into(),
            speed_mps: 10.0,
            ms_kg: 300.0,
            c_nsm: 1000.0,
            dropout_pct: 0.0,
            seed: 42,
            duration_s: 5.0,
            k_nm: None, mu_kg: None, kt_nm: None,
        };
        let result = run_scenario(&spec).unwrap();
        let json = serde_json::to_string(&result).unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed["fixture"], false);
        assert!(parsed["t"].is_array());
    }

    #[test]
    fn deterministic_output() {
        let spec = ScenarioSpec {
            road_class: "B".into(),
            speed_mps: 20.0,
            ms_kg: 440.0,
            c_nsm: 1800.0,
            dropout_pct: 5.0,
            seed: 7,
            duration_s: 5.0,
            k_nm: None, mu_kg: None, kt_nm: None,
        };
        let r1 = run_scenario(&spec).unwrap();
        let r2 = run_scenario(&spec).unwrap();

        let j1 = serde_json::to_string(&r1).unwrap();
        let j2 = serde_json::to_string(&r2).unwrap();
        assert_eq!(j1, j2, "Same seed must produce byte-identical output");
    }
}

