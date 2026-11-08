//! Metrics computation: RMSE, consistency, detection performance.
//!
//! PRD §5.7: RMSE per state vs truth (raw, open-loop, fused),
//! fraction within ±2σ, NEES, anomaly/dropout detection P/R.

use crate::kf::KfOutput;
use crate::sensors::FaultEvent;
use nalgebra::SVector;
use serde_json::{json, Value};

/// RMSE result for one state variable.
#[derive(Debug, Clone, Default)]
pub struct RmseResult {
    pub raw: Option<f64>,
    pub open_loop: Option<f64>,
    pub fused: f64,
}

/// Full metrics for a scenario run.
#[derive(Debug, Clone)]
pub struct ScenarioMetrics {
    pub rmse_susp_travel: RmseResult,
    pub rmse_body_vel: RmseResult,
    pub rmse_wheel_vel: RmseResult,
    pub rmse_road_height: RmseResult,
    pub within_2sigma: ConsistencyMetrics,
    pub nees_mean: f64,
    pub anomaly_detection: DetectionMetrics,
    pub dropout_detection: DetectionMetrics,
}

/// Consistency metrics: fraction of truth within ±2σ.
#[derive(Debug, Clone, Default)]
pub struct ConsistencyMetrics {
    pub susp_travel: f64,
    pub body_vel: f64,
    pub wheel_vel: f64,
    pub road_height: f64,
}

/// Detection precision and recall.
#[derive(Debug, Clone, Default)]
pub struct DetectionMetrics {
    pub precision: f64,
    pub recall: f64,
    pub tp: usize,
    pub fp: usize,
    pub fn_: usize,
}

/// Truth data at the fusion clock rate.
pub struct TruthData {
    pub zs: Vec<f64>,
    pub vs: Vec<f64>,
    pub zu: Vec<f64>,
    pub vu: Vec<f64>,
    pub zr: Vec<f64>,
}

/// Compute RMSE between two signals.
fn rmse(estimated: &[f64], truth: &[f64]) -> f64 {
    let n = estimated.len().min(truth.len());
    if n == 0 { return 0.0; }
    let sum_sq: f64 = estimated.iter().zip(truth.iter())
        .map(|(e, t)| (e - t).powi(2))
        .sum();
    (sum_sq / n as f64).sqrt()
}

/// Compute RMSE for optional raw sensor data.
fn rmse_optional(raw: &[Option<f64>], truth: &[f64]) -> Option<f64> {
    let pairs: Vec<(f64, f64)> = raw.iter().zip(truth.iter())
        .filter_map(|(r, t)| r.map(|rv| (rv, *t)))
        .collect();
    if pairs.is_empty() { return None; }
    let sum_sq: f64 = pairs.iter().map(|(r, t)| (r - t).powi(2)).sum();
    Some((sum_sq / pairs.len() as f64).sqrt())
}

/// Compute fraction of truth within ±2σ of the fused estimate.
fn within_2sigma(
    fused: &[f64],
    truth: &[f64],
    sigma: &[f64],
) -> f64 {
    let n = fused.len().min(truth.len()).min(sigma.len());
    if n == 0 { return 0.0; }
    let count = fused.iter().zip(truth.iter()).zip(sigma.iter())
        .filter(|((f, t), s)| (*f - *t).abs() <= 2.0 * *s)
        .count();
    count as f64 / n as f64
}

/// Compute Normalised Estimation Error Squared (NEES).
fn compute_nees(
    states: &[SVector<f64, 5>],
    truth: &TruthData,
    cov_diag: &[[f64; 5]],
) -> f64 {
    let n = states.len().min(truth.zs.len());
    if n == 0 { return 0.0; }

    let mut sum_nees = 0.0;
    for i in 0..n {
        let err = [
            states[i][0] - truth.zs[i],
            states[i][1] - truth.vs[i],
            states[i][2] - truth.zu[i],
            states[i][3] - truth.vu[i],
            states[i][4] - truth.zr[i],
        ];
        let nees: f64 = err.iter().zip(cov_diag[i].iter())
            .map(|(e, p)| if *p > 1e-15 { e * e / p } else { 0.0 })
            .sum();
        sum_nees += nees;
    }

    sum_nees / n as f64
}

/// Compute detection precision and recall for fault events.
fn compute_detection(
    detected_flags: &[bool],
    fault_log: &[FaultEvent],
    fault_type: &str,
    total_samples: usize,
) -> DetectionMetrics {
    // Build ground truth mask from fault log
    let mut truth_mask = vec![false; total_samples];
    for event in fault_log {
        if event.fault_type == fault_type {
            for &idx in &event.sample_indices {
                if idx < total_samples {
                    truth_mask[idx] = true;
                }
            }
        }
    }

    let mut tp = 0;
    let mut fp = 0;
    let mut _tn = 0;
    let mut fn_count = 0;

    for i in 0..total_samples.min(detected_flags.len()) {
        match (detected_flags[i], truth_mask[i]) {
            (true, true) => tp += 1,
            (true, false) => fp += 1,
            (false, true) => fn_count += 1,
            (false, false) => _tn += 1,
        }
    }

    let precision = if tp + fp > 0 { tp as f64 / (tp + fp) as f64 } else { 0.0 };
    let recall = if tp + fn_count > 0 { tp as f64 / (tp + fn_count) as f64 } else { 0.0 };

    DetectionMetrics { precision, recall, tp, fp, fn_: fn_count }
}

/// Compute all metrics for a scenario.
pub fn compute_metrics(
    kf_out: &KfOutput,
    truth: &TruthData,
    raw_susp_travel: &[Option<f64>],
    open_loop_states: Option<&[SVector<f64, 5>]>,
    fault_log: &[FaultEvent],
    anomaly_detected: &[bool],
    dropout_detected: &[bool],
) -> ScenarioMetrics {
    let n = kf_out.states.len();

    // Extract fused state time series
    let fused_travel: Vec<f64> = kf_out.states.iter().map(|s| s[0] - s[2]).collect();
    let fused_body_vel: Vec<f64> = kf_out.states.iter().map(|s| s[1]).collect();
    let fused_wheel_vel: Vec<f64> = kf_out.states.iter().map(|s| s[3]).collect();
    let fused_road: Vec<f64> = kf_out.states.iter().map(|s| s[4]).collect();

    // Truth state time series
    let truth_travel: Vec<f64> = truth.zs.iter().zip(truth.zu.iter())
        .map(|(zs, zu)| zs - zu).collect();

    // RMSE: fused
    let rmse_fused_travel = rmse(&fused_travel, &truth_travel);
    let rmse_fused_body_vel = rmse(&fused_body_vel, &truth.vs);
    let rmse_fused_wheel_vel = rmse(&fused_wheel_vel, &truth.vu);
    let rmse_fused_road = rmse(&fused_road, &truth.zr);

    // RMSE: raw (only for susp_travel — others don't have direct sensor)
    let rmse_raw_travel = rmse_optional(raw_susp_travel, &truth_travel);

    // RMSE: open-loop nominal twin
    let (ol_travel, ol_bv, ol_wv, ol_road) = if let Some(ol) = open_loop_states {
        let ol_t: Vec<f64> = ol.iter().map(|s| s[0] - s[2]).collect();
        let ol_bv: Vec<f64> = ol.iter().map(|s| s[1]).collect();
        let ol_wv: Vec<f64> = ol.iter().map(|s| s[3]).collect();
        let ol_r: Vec<f64> = ol.iter().map(|s| s[4]).collect();
        (
            Some(rmse(&ol_t, &truth_travel)),
            Some(rmse(&ol_bv, &truth.vs)),
            Some(rmse(&ol_wv, &truth.vu)),
            Some(rmse(&ol_r, &truth.zr)),
        )
    } else {
        (None, None, None, None)
    };

    // Consistency: fraction within ±2σ
    let sigma_travel: Vec<f64> = kf_out.cov_diag.iter()
        .map(|d| (d[0] + d[2]).sqrt()) // approximate σ(zs-zu)
        .collect();
    let sigma_bv: Vec<f64> = kf_out.cov_diag.iter().map(|d| d[1].sqrt()).collect();
    let sigma_wv: Vec<f64> = kf_out.cov_diag.iter().map(|d| d[3].sqrt()).collect();
    let sigma_road: Vec<f64> = kf_out.cov_diag.iter().map(|d| d[4].sqrt()).collect();

    let w2s_travel = within_2sigma(&fused_travel, &truth_travel, &sigma_travel);
    let w2s_bv = within_2sigma(&fused_body_vel, &truth.vs, &sigma_bv);
    let w2s_wv = within_2sigma(&fused_wheel_vel, &truth.vu, &sigma_wv);
    let w2s_road = within_2sigma(&fused_road, &truth.zr, &sigma_road);

    // NEES
    let nees = compute_nees(&kf_out.states, truth, &kf_out.cov_diag);

    // Detection metrics
    let anomaly_det = compute_detection(anomaly_detected, fault_log, "spike", n);
    let dropout_det = compute_detection(dropout_detected, fault_log, "dropout", n);

    ScenarioMetrics {
        rmse_susp_travel: RmseResult {
            raw: rmse_raw_travel,
            open_loop: ol_travel,
            fused: rmse_fused_travel,
        },
        rmse_body_vel: RmseResult {
            raw: None,
            open_loop: ol_bv,
            fused: rmse_fused_body_vel,
        },
        rmse_wheel_vel: RmseResult {
            raw: None,
            open_loop: ol_wv,
            fused: rmse_fused_wheel_vel,
        },
        rmse_road_height: RmseResult {
            raw: None,
            open_loop: ol_road,
            fused: rmse_fused_road,
        },
        within_2sigma: ConsistencyMetrics {
            susp_travel: w2s_travel,
            body_vel: w2s_bv,
            wheel_vel: w2s_wv,
            road_height: w2s_road,
        },
        nees_mean: nees,
        anomaly_detection: anomaly_det,
        dropout_detection: dropout_det,
    }
}

/// Per-run record kept for the matrix summary.
pub struct RunStats {
    pub dropout_pct: f64,
    pub seed: u64,
    pub m: ScenarioMetrics,
    /// Estimated delay per channel [samples at 200 Hz]
    pub delays: Vec<(String, f64)>,
}

const STATE_KEYS: [&str; 4] = ["susp_travel_m", "body_vel_mps", "wheel_vel_mps", "road_height_m"];

fn rmses(m: &ScenarioMetrics) -> [&RmseResult; 4] {
    [&m.rmse_susp_travel, &m.rmse_body_vel, &m.rmse_wheel_vel, &m.rmse_road_height]
}

fn w2s(m: &ScenarioMetrics) -> [f64; 4] {
    let c = &m.within_2sigma;
    [c.susp_travel, c.body_vel, c.wheel_vel, c.road_height]
}

fn mean_std(v: &[f64]) -> (f64, f64) {
    let n = v.len();
    if n == 0 { return (0.0, 0.0); }
    let m = v.iter().sum::<f64>() / n as f64;
    let var = if n > 1 {
        v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / (n - 1) as f64
    } else {
        0.0
    };
    (m, var.sqrt())
}

fn opt_ms(v: Vec<f64>) -> Value {
    if v.is_empty() { return Value::Null; }
    let (mean, std) = mean_std(&v);
    json!({"mean": mean, "std": std})
}

fn pooled<'a>(it: impl Iterator<Item = &'a DetectionMetrics>) -> Value {
    let (tp, fp, fn_) = it.fold((0usize, 0usize, 0usize), |a, d| (a.0 + d.tp, a.1 + d.fp, a.2 + d.fn_));
    let ratio = |a: usize, b: usize| if b == 0 { 0.0 } else { a as f64 / b as f64 };
    json!({"precision": ratio(tp, tp + fp), "recall": ratio(tp, tp + fn_)})
}

/// CONTRACTS §4.2. `injected_ms`: (channel, configured delay).
pub fn build_fusion_summary(
    runs: &[RunStats],
    injected_ms: &[(String, f64)],
    duration_s: f64,
    sweep_pcts: &[f64],
) -> Value {
    let mut seeds: Vec<u64> = runs.iter().map(|r| r.seed).collect();
    seeds.sort_unstable();
    seeds.dedup();

    let mut state_rmse = serde_json::Map::new();
    let mut cover = serde_json::Map::new();
    for (k, key) in STATE_KEYS.iter().enumerate() {
        let col = |f: fn(&RmseResult) -> Option<f64>| -> Vec<f64> {
            runs.iter().filter_map(|r| f(rmses(&r.m)[k])).collect()
        };
        state_rmse.insert(key.to_string(), json!({
            "raw": opt_ms(col(|r| r.raw)),
            "open_loop": opt_ms(col(|r| r.open_loop)),
            "fused": opt_ms(col(|r| Some(r.fused))),
        }));
        let f: Vec<f64> = runs.iter().map(|r| w2s(&r.m)[k]).collect();
        cover.insert(key.to_string(), json!(mean_std(&f).0));
    }
    let nees: Vec<f64> = runs.iter().map(|r| r.m.nees_mean).collect();

    let sweep: Vec<Value> = sweep_pcts.iter().map(|&d| {
        let sel: Vec<&RunStats> = runs.iter().filter(|r| (r.dropout_pct - d).abs() < 1e-9).collect();
        let mut o = serde_json::Map::new();
        for (k, key) in STATE_KEYS.iter().enumerate() {
            let v: Vec<f64> = sel.iter().map(|r| rmses(&r.m)[k].fused).collect();
            o.insert(key.to_string(), json!(mean_std(&v).0));
        }
        json!({"dropout_pct": d, "fused_rmse": o})
    }).collect();

    let delay: Vec<Value> = injected_ms.iter().map(|(ch, inj)| {
        let v: Vec<f64> = runs.iter()
            .filter_map(|r| r.delays.iter().find(|(c, _)| c == ch).map(|(_, s)| s * 5.0)) // 5 ms/sample @ 200 Hz
            .collect();
        json!({"channel": ch, "injected_ms": inj, "estimated_ms": mean_std(&v).0})
    }).collect();

    json!({
        "meta": {"fixture": false, "n_runs": runs.len(), "seeds": seeds, "duration_s": duration_s},
        "state_rmse": state_rmse,
        "consistency": {"within_2sigma_frac": cover, "nees_mean": mean_std(&nees).0},
        "dropout_sweep": sweep,
        "detection": {
            "anomaly": pooled(runs.iter().map(|r| &r.m.anomaly_detection)),
            "dropout": pooled(runs.iter().map(|r| &r.m.dropout_detection)),
        },
        "delay_estimation": delay,
    })
}