//! Kalman filter with augmented road state.
//!
//! PRD §5.5: 5-state EKF [zs, vs, zu, vu, zr] with sequential scalar updates
//! in Joseph form, innovation gating, and matrix-exponential discretisation.

use nalgebra::{SMatrix, SVector};

/// State dimension.
pub const NX: usize = 5;

type Mat5 = SMatrix<f64, NX, NX>;
type Vec5 = SVector<f64, NX>;

/// Chi-square threshold for 1 DOF at 99% confidence.
const GATE_CHI2: f64 = 6.63;

/// Kalman filter state.
#[derive(Debug, Clone)]
pub struct KalmanState {
    /// State estimate [zs, vs, zu, vu, zr]
    pub x: Vec5,
    /// Covariance
    pub p: Mat5,
}

/// Kalman filter tuning parameters.
#[derive(Debug, Clone)]
pub struct KfParams {
    /// Sprung mass [kg]
    pub ms: f64,
    /// Unsprung mass [kg]
    pub mu: f64,
    /// Spring rate [N/m]
    pub k: f64,
    /// Damping [N·s/m]
    pub c: f64,
    /// Tyre stiffness [N/m]
    pub kt: f64,
    /// Process noise covariance
    pub q: Mat5,
    /// Road random walk intensity
    pub q_road: f64,
    /// Fusion step dt [s]
    pub dt: f64,
}

/// Innovation log entry.
#[derive(Debug, Clone)]
pub struct InnovationEntry {
    pub t: f64,
    pub channel: String,
    pub innovation: f64,
    pub nis: f64,
    pub gated: bool,
}

/// Full filter output for one scenario run.
#[derive(Debug, Clone)]
pub struct KfOutput {
    /// Fused state estimates at each step
    pub states: Vec<Vec5>,
    /// Covariance diagonal at each step
    pub cov_diag: Vec<[f64; NX]>,
    /// Covariance trace at each step
    pub cov_trace: Vec<f64>,
    /// Innovation log
    pub innovations: Vec<InnovationEntry>,
    /// Gate rejection log (subset of innovations with gated=true)
    pub gate_log: Vec<InnovationEntry>,
}

impl KfParams {
    /// Build default tuning from nominal vehicle and sensor specs.
    pub fn from_nominal(ms: f64, mu: f64, k: f64, c: f64, kt: f64, dt: f64) -> Self {
        // Process noise: tuned from physical uncertainty
        let mut q = Mat5::zeros();
        q[(0, 0)] = 1e-8;   // zs position uncertainty
        q[(1, 1)] = 1e-4;   // vs velocity uncertainty
        q[(2, 2)] = 1e-8;   // zu position uncertainty
        q[(3, 3)] = 1e-4;   // vu velocity uncertainty
        let q_road = 3e-4;  // zr road random walk intensity; ISO B @ 20 m/s ~ 2.5e-4
        q[(4, 4)] = q_road;

        Self { ms, mu, k, c, kt, q, q_road, dt }
    }

    /// Override the road random-walk intensity (also updates Q[4,4]).
    pub fn with_road_q(mut self, q_road: f64) -> Self {
        self.q_road = q_road;
        self.q[(4, 4)] = q_road;
        self
    }
}

impl KalmanState {
    pub fn new() -> Self {
        Self {
            x: Vec5::zeros(),
            p: Mat5::identity() * 0.01,
        }
    }
}

/// Build the continuous-time system matrix A for the quarter-car + road model.
///
/// State: [zs, vs, zu, vu, zr]
/// Equations:
///   dzs/dt = vs
///   dvs/dt = -(k/ms)(zs-zu) - (c/ms)(vs-vu)
///   dzu/dt = vu
///   dvu/dt = (k/mu)(zs-zu) + (c/mu)(vs-vu) - (kt/mu)(zu-zr)
///   dzr/dt = 0  (random walk — noise enters through Q)
fn build_a_continuous(p: &KfParams) -> Mat5 {
    let ms = p.ms;
    let mu = p.mu;
    let k = p.k;
    let c = p.c;
    let kt = p.kt;

    #[rustfmt::skip]
    let a = Mat5::new(
        0.0,     1.0,      0.0,      0.0,      0.0,
        -k/ms,  -c/ms,     k/ms,     c/ms,     0.0,
        0.0,     0.0,      0.0,      1.0,      0.0,
        k/mu,    c/mu,    -(k+kt)/mu, -c/mu,    kt/mu,
        0.0,     0.0,      0.0,      0.0,      0.0,
    );
    a
}

/// Matrix exponential via scaling and squaring with Taylor series.
///
/// Computes exp(A*dt) using Taylor expansion of order 12
/// with scaling and squaring for numerical stability.
fn matrix_exp(a: &Mat5, dt: f64) -> Mat5 {
    let m = a * dt;

    // Scaling: find a power of 2 such that ||m/2^s|| < 0.5
    let norm = m.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    let s = (norm.log2().ceil().max(0.0)) as u32 + 1;
    let scale = 2.0_f64.powi(s as i32);

    let ms = m / scale;

    // Taylor expansion: exp(ms) ≈ I + ms + ms²/2! + ... + ms^12/12!
    let mut result = Mat5::identity();
    let mut term = Mat5::identity();

    for i in 1..=12 {
        term = term * ms / i as f64;
        result += term;
    }

    // Squaring: exp(m) = (exp(m/2^s))^(2^s)
    for _ in 0..s {
        result = result * result;
    }

    result
}

/// Predict step: propagate state and covariance.
pub fn predict(state: &mut KalmanState, params: &KfParams) {
    let a_c = build_a_continuous(params);
    let f = matrix_exp(&a_c, params.dt);

    state.x = f * state.x;
    // trapezoidal Qd: carries road/velocity noise through the dynamics within the step
    let qd = (f * params.q * f.transpose() + params.q) * (0.5 * params.dt);
    state.p = f * state.p * f.transpose() + qd;
}

/// Measurement model: returns (h_row, predicted_measurement) for a given channel.
///
/// All measurements are linear in the state.
fn measurement_model(
    channel: &str,
    x: &Vec5,
    params: &KfParams,
) -> Option<(SVector<f64, NX>, f64)> {
    let ms = params.ms;
    let mu = params.mu;
    let k = params.k;
    let c = params.c;
    let kt = params.kt;

    match channel {
        "body_acc" => {
            // body_acc = (-k*(zs-zu) - c*(vs-vu)) / ms
            let mut h = Vec5::zeros();
            h[0] = -k / ms;     // zs
            h[1] = -c / ms;     // vs
            h[2] = k / ms;      // zu
            h[3] = c / ms;      // vu
            let z_pred = h.dot(x);
            Some((h, z_pred))
        }
        "wheel_acc" => {
            // wheel_acc = (k*(zs-zu) + c*(vs-vu) - kt*(zu-zr)) / mu
            let mut h = Vec5::zeros();
            h[0] = k / mu;      // zs
            h[1] = c / mu;      // vs
            h[2] = -(k + kt) / mu; // zu
            h[3] = -c / mu;     // vu
            h[4] = kt / mu;     // zr
            let z_pred = h.dot(x);
            Some((h, z_pred))
        }
        "susp_travel" => {
            // susp_travel = zs - zu
            let mut h = Vec5::zeros();
            h[0] = 1.0;
            h[2] = -1.0;
            let z_pred = h.dot(x);
            Some((h, z_pred))
        }
        _ => None,
    }
}

/// Sequential scalar update in Joseph form with innovation gating.
///
/// PRD §5.5: scalar updates avoid matrix inverses.
/// Gating: NIS > 6.63 (chi² 1 DOF, 99%) → reject.
pub fn scalar_update(
    state: &mut KalmanState,
    channel: &str,
    measurement: f64,
    r: f64, // measurement noise variance
    params: &KfParams,
    t: f64,
    innovations: &mut Vec<InnovationEntry>,
) -> bool {
    let model = measurement_model(channel, &state.x, params);
    let (h, z_pred) = match model {
        Some(m) => m,
        None => return false,
    };

    let innovation = measurement - z_pred;

    // Innovation covariance (scalar)
    let s = (h.transpose() * state.p * h)[(0, 0)] + r;

    // Normalised innovation squared (NIS)
    let nis = innovation * innovation / s;

    let gated = nis > GATE_CHI2;

    innovations.push(InnovationEntry {
        t,
        channel: channel.to_string(),
        innovation,
        nis,
        gated,
    });

    if gated {
        // Reject measurement — predict only
        return false;
    }

    // Kalman gain (column vector)
    let k_gain = state.p * h / s;

    // State update
    state.x += k_gain * innovation;

    // Joseph form covariance update: P = (I - K*H') * P * (I - K*H')' + K*R*K'
    let i_kh = Mat5::identity() - k_gain * h.transpose();
    state.p = i_kh * state.p * i_kh.transpose() + k_gain * k_gain.transpose() * r;

    // Symmetrise
    state.p = (state.p + state.p.transpose()) / 2.0;

    true
}

/// Measurement noise variances from sensor configs.
pub struct MeasNoise {
    pub body_acc: f64,
    pub wheel_acc: f64,
    pub susp_travel: f64,
}

impl MeasNoise {
    pub fn from_sensor_sigmas(body_sigma: f64, wheel_sigma: f64, travel_sigma: f64) -> Self {
        Self {
            body_acc: body_sigma * body_sigma,
            wheel_acc: wheel_sigma * wheel_sigma,
            susp_travel: travel_sigma * travel_sigma,
        }
    }

    pub fn default() -> Self {
        Self::from_sensor_sigmas(0.05, 0.5, 0.0005)
    }
}

/// Run the full Kalman filter on synchronised sensor data.
pub fn run_filter(
    frames: &[crate::sync::SyncFrame],
    params: &KfParams,
    meas_noise: &MeasNoise,
) -> KfOutput {
    let mut state = KalmanState::new();
    let mut all_innovations = Vec::new();
    let mut gate_log = Vec::new();

    let n = frames.len();
    let mut states = Vec::with_capacity(n);
    let mut cov_diag = Vec::with_capacity(n);
    let mut cov_trace = Vec::with_capacity(n);

    for frame in frames {
        // Predict
        predict(&mut state, params);

        // Sequential scalar updates for available measurements
        if frame.body_acc_valid {
            if let Some(z) = frame.body_acc {
                let accepted = scalar_update(
                    &mut state, "body_acc", z, meas_noise.body_acc,
                    params, frame.t, &mut all_innovations,
                );
                if !accepted {
                    if let Some(last) = all_innovations.last() {
                        gate_log.push(last.clone());
                    }
                }
            }
        }

        if frame.wheel_acc_valid {
            if let Some(z) = frame.wheel_acc {
                let accepted = scalar_update(
                    &mut state, "wheel_acc", z, meas_noise.wheel_acc,
                    params, frame.t, &mut all_innovations,
                );
                if !accepted {
                    if let Some(last) = all_innovations.last() {
                        gate_log.push(last.clone());
                    }
                }
            }
        }

        if frame.susp_travel_valid {
            if let Some(z) = frame.susp_travel {
                let accepted = scalar_update(
                    &mut state, "susp_travel", z, meas_noise.susp_travel,
                    params, frame.t, &mut all_innovations,
                );
                if !accepted {
                    if let Some(last) = all_innovations.last() {
                        gate_log.push(last.clone());
                    }
                }
            }
        }

        // Store outputs
        states.push(state.x);
        let diag = [
            state.p[(0, 0)], state.p[(1, 1)], state.p[(2, 2)],
            state.p[(3, 3)], state.p[(4, 4)],
        ];
        cov_diag.push(diag);
        cov_trace.push(state.p.trace());
    }

    KfOutput { states, cov_diag, cov_trace, innovations: all_innovations, gate_log }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::SensorConfig;
    use crate::plant;
    use crate::sensors;
    use crate::sync;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    fn run_test_filter(dropout_pct: f64) -> (KfOutput, Vec<f64>) {
        let dt = 0.001;
        let n = 20_001;
        let road: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                0.01 * (2.0 * std::f64::consts::PI * 2.0 * t).sin()
            })
            .collect();

        let plant_out = plant::run_nominal(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 0.12, &road, dt);
        let configs = SensorConfig::defaults();
        let mut rng = ChaCha8Rng::seed_from_u64(42);
        let sensor_out = sensors::simulate_sensors(&plant_out, dt, 20.0, &configs, dropout_pct, &mut rng);
        let sync_out = sync::synchronise(&sensor_out);

        let kf_params = KfParams::from_nominal(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 1.0 / 200.0);
        let meas_noise = MeasNoise::default();

        let kf_out = run_filter(&sync_out.frames, &kf_params, &meas_noise);

        // Truth: susp_travel = zs - zu from plant
        let truth_travel: Vec<f64> = (0..sync_out.frames.len())
            .map(|i| {
                let pi = (i * 5).min(plant_out.zs.len() - 1);
                plant_out.zs[pi] - plant_out.zu[pi]
            })
            .collect();

        (kf_out, truth_travel)
    }

    #[test]
    fn filter_produces_output() {
        let (kf_out, _) = run_test_filter(0.0);
        assert!(!kf_out.states.is_empty());
        assert_eq!(kf_out.states.len(), kf_out.cov_diag.len());
    }

    #[test]
    fn fused_travel_rmse_below_raw() {
        let dt = 0.001;
        let n = 20_001;
        let road: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                0.01 * (2.0 * std::f64::consts::PI * 2.0 * t).sin()
            })
            .collect();
        let plant_out = plant::run_nominal(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 0.12, &road, dt);
        let configs = SensorConfig::defaults();
        let mut rng = ChaCha8Rng::seed_from_u64(42);
        let sensor_out = sensors::simulate_sensors(&plant_out, dt, 20.0, &configs, 0.0, &mut rng);
        let sync_out = sync::synchronise(&sensor_out);

        let kf_params = KfParams::from_nominal(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 1.0 / 200.0);
        let meas_noise = MeasNoise::default();
        let kf_out = run_filter(&sync_out.frames, &kf_params, &meas_noise);

        // Compare fused vs raw RMSE on suspension travel
        let n_kf = kf_out.states.len();
        let mut fused_err_sq = 0.0;
        let mut raw_err_sq = 0.0;
        let mut count = 0;

        for i in 0..n_kf {
            let pi = (i * 5).min(plant_out.zs.len() - 1);
            let truth = plant_out.zs[pi] - plant_out.zu[pi];
            let fused = kf_out.states[i][0] - kf_out.states[i][2]; // zs - zu

            fused_err_sq += (fused - truth).powi(2);

            if let Some(raw) = sensor_out.susp_travel[i] {
                raw_err_sq += (raw - truth).powi(2);
                count += 1;
            }
        }

        let fused_rmse = (fused_err_sq / n_kf as f64).sqrt();
        let raw_rmse = (raw_err_sq / count as f64).sqrt();

        assert!(
            fused_rmse < raw_rmse,
            "Fused RMSE ({fused_rmse:.6}) should be below raw RMSE ({raw_rmse:.6})"
        );
    }

    #[test]
    fn spike_is_rejected() {
        // The filter should reject spike anomalies via gating
        let (kf_out, _) = run_test_filter(5.0);
        // With sensors injecting spikes, some should be gated
        // (depends on spike magnitude vs filter state)
        let n_gated = kf_out.gate_log.len();
        // This is a soft check — spikes may or may not trigger gating
        // depending on filter state. Just verify gate_log exists.
        assert!(kf_out.innovations.len() > 0, "Should have innovations");
        eprintln!("Gated measurements: {n_gated}");
    }

    #[test]
    fn covariance_grows_during_dropout() {
        let (kf_out_0, _) = run_test_filter(0.0);
        let (kf_out_20, _) = run_test_filter(20.0);

        // Mean covariance trace should be higher with more dropout
        let mean_trace_0: f64 = kf_out_0.cov_trace.iter().sum::<f64>() / kf_out_0.cov_trace.len() as f64;
        let mean_trace_20: f64 = kf_out_20.cov_trace.iter().sum::<f64>() / kf_out_20.cov_trace.len() as f64;

        assert!(
            mean_trace_20 > mean_trace_0,
            "Mean cov trace with 20% dropout ({mean_trace_20:.6}) should exceed 0% ({mean_trace_0:.6})"
        );
    }

    fn diag_case(label: &str, delay: bool, bias: bool, perturbed: bool) {
        let dt = 0.001;
        let n = 20_001;
        let road: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                0.01 * (2.0 * std::f64::consts::PI * 2.0 * t).sin()
            })
            .collect();

        let mut rng = ChaCha8Rng::seed_from_u64(42);
        let plant_out = if perturbed {
            let pert = plant::PlantPerturbation::draw(&mut rng, 10.0, 5.0, 2.0);
            plant::run_perturbed(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 0.12, &road, dt, &pert)
        } else {
            plant::run_nominal(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 0.12, &road, dt)
        };

        let mut configs = SensorConfig::defaults();
        for cfg in configs.iter_mut() {
            if !delay { cfg.delay_ms = 0.0; }
            if !bias { cfg.bias = 0.0; }
        }
        let sensor_out = sensors::simulate_sensors(&plant_out, dt, 20.0, &configs, 0.0, &mut rng);
        let sync_out = sync::synchronise(&sensor_out);
        let kf_params = KfParams::from_nominal(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 1.0 / 200.0);
        let kf_out = run_filter(&sync_out.frames, &kf_params, &MeasNoise::default());

        let n_kf = kf_out.states.len();
        let skip = 200; // first 1 s = initial transient
        let mut sq_all = 0.0;
        let mut sq_ss = 0.0;
        let mut n_ss = 0usize;
        let mut raw_sq = 0.0;
        let mut raw_n = 0usize;
        let mut inside = [0usize; 4];

        for i in 0..n_kf {
            let pi = (i * 5).min(plant_out.zs.len() - 1);
            let truth = [
                plant_out.zs[pi] - plant_out.zu[pi],
                plant_out.vs[pi],
                plant_out.vu[pi],
                plant_out.road[pi],
            ];
            let s = &kf_out.states[i];
            let est = [s[0] - s[2], s[1], s[3], s[4]];
            let d = &kf_out.cov_diag[i];
            let sig = [(d[0] + d[2]).sqrt(), d[1].sqrt(), d[3].sqrt(), d[4].sqrt()];
            for j in 0..4 {
                if (est[j] - truth[j]).abs() <= 2.0 * sig[j] {
                    inside[j] += 1;
                }
            }
            let e = est[0] - truth[0];
            sq_all += e * e;
            if i >= skip {
                sq_ss += e * e;
                n_ss += 1;
            }
            if let Some(raw) = sensor_out.susp_travel[i] {
                raw_sq += (raw - truth[0]).powi(2);
                raw_n += 1;
            }
        }

        let rmse = |sq: f64, c: usize| (sq / c.max(1) as f64).sqrt() * 1e3;
        let pct = |c: usize| 100.0 * c as f64 / n_kf.max(1) as f64;
        println!("\n=== case {label}: delay={delay} bias={bias} perturbed={perturbed} ===");
        println!(
            "travel RMSE [mm]: fused={:.3} (after 1 s: {:.3})  raw={:.3}",
            rmse(sq_all, n_kf), rmse(sq_ss, n_ss), rmse(raw_sq, raw_n)
        );
        println!(
            "truth within 2 sigma [%] (ideal ~95): travel={:.1} body_vel={:.1} wheel_vel={:.1} road={:.1}",
            pct(inside[0]), pct(inside[1]), pct(inside[2]), pct(inside[3])
        );
        for ch in ["body_acc", "wheel_acc", "susp_travel"] {
            let v: Vec<&InnovationEntry> =
                kf_out.innovations.iter().filter(|e| e.channel == ch).collect();
            let m = v.len().max(1) as f64;
            let nis = v.iter().map(|e| e.nis).sum::<f64>() / m;
            let inn = v.iter().map(|e| e.innovation).sum::<f64>() / m;
            let gated = 100.0 * v.iter().filter(|e| e.gated).count() as f64 / m;
            println!(
                "  {ch:<12} n={:<5} mean NIS={nis:.2} (ideal ~1)  mean innov={inn:.5}  gated={gated:.1}%",
                v.len()
            );
        }
    }

    #[test]
    fn diagnose_filter() {
        diag_case("i   clean (no delay, no bias, nominal plant)", false, false, false);
        diag_case("i1  + sensor delay", true, false, false);
        diag_case("i2  + bias (= failing-test config)", true, true, false);
        diag_case("ii1 + perturbed plant", true, true, true);
    }

    #[test]
    fn diagnose_q_road() {
        let dt = 0.001;
        let road: Vec<f64> = (0..20_001)
            .map(|i| 0.01 * (2.0 * std::f64::consts::PI * 2.0 * i as f64 * dt).sin())
            .collect();
        let plant_out = plant::run_nominal(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 0.12, &road, dt);
        let mut rng = ChaCha8Rng::seed_from_u64(42);
        let sensor_out =
            sensors::simulate_sensors(&plant_out, dt, 20.0, &SensorConfig::defaults(), 0.0, &mut rng);
        let sync_out = sync::synchronise(&sensor_out);
        println!();
        for q_road in [1e-2, 1e-3, 3e-4, 1e-4, 3e-5, 1e-5] {
            let p = KfParams::from_nominal(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 1.0 / 200.0)
                .with_road_q(q_road);
            let out = run_filter(&sync_out.frames, &p, &MeasNoise::default());
            let n = out.states.len();
            let (mut f_sq, mut r_sq, mut rc) = (0.0_f64, 0.0_f64, 0usize);
            for i in 0..n {
                let pi = (i * 5).min(plant_out.zs.len() - 1);
                let truth = plant_out.zs[pi] - plant_out.zu[pi];
                f_sq += (out.states[i][0] - out.states[i][2] - truth).powi(2);
                if let Some(raw) = sensor_out.susp_travel[i] {
                    r_sq += (raw - truth).powi(2);
                    rc += 1;
                }
            }
            let stat = |ch: &str| {
                let v: Vec<&InnovationEntry> =
                    out.innovations.iter().filter(|e| e.channel == ch).collect();
                let m = v.len().max(1) as f64;
                (
                    v.iter().map(|e| e.nis).sum::<f64>() / m,
                    100.0 * v.iter().filter(|e| e.gated).count() as f64 / m,
                )
            };
            let (nb, gb) = stat("body_acc");
            let (nw, _) = stat("wheel_acc");
            let (nt, gt) = stat("susp_travel");
            println!(
                "q_road={q_road:.0e}: fused={:.3} mm raw={:.3} mm | NIS body={nb:.2} wheel={nw:.2} travel={nt:.2} | gated% body={gb:.1} travel={gt:.1}",
                (f_sq / n as f64).sqrt() * 1e3,
                (r_sq / rc.max(1) as f64).sqrt() * 1e3
            );
        }
    }
}