//! Synchronisation layer: fusion clock, validity flags, delay estimation.
//!
//! PRD §5.4: 200 Hz fusion clock, per-sample validity, delay estimation
//! via model-based search.

use crate::sensors::SensorOutputs;

/// Per-sample validity flags for each channel at the fusion clock rate.
#[derive(Debug, Clone)]
pub struct SyncFrame {
    pub t: f64,
    pub body_acc: Option<f64>,
    pub wheel_acc: Option<f64>,
    pub susp_travel: Option<f64>,
    pub wheel_speed: Option<f64>,
    pub veh_speed: Option<f64>,
    /// Validity flags
    pub body_acc_valid: bool,
    pub wheel_acc_valid: bool,
    pub susp_travel_valid: bool,
    pub wheel_speed_valid: bool,
    pub veh_speed_valid: bool,
}

/// Synchronised sensor data at the fusion clock.
#[derive(Debug, Clone)]
pub struct SyncOutput {
    pub frames: Vec<SyncFrame>,
    /// Estimated delays per channel [samples at fusion rate]
    pub estimated_delays: Vec<(String, f64)>,
}

/// Detect stuck-at windows: near-zero variance over a sliding window.
fn detect_stuck(values: &[Option<f64>], window: usize) -> Vec<bool> {
    let n = values.len();
    let mut stuck = vec![false; n];

    for i in 0..n {
        if i + window > n { break; }
        let vals: Vec<f64> = values[i..i + window].iter().filter_map(|x| *x).collect();
        if vals.len() < window / 2 { continue; }
        let mean = vals.iter().sum::<f64>() / vals.len() as f64;
        let var = vals.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / vals.len() as f64;
        if var < 1e-12 {
            for j in i..i + window {
                if j < n { stuck[j] = true; }
            }
        }
    }
    stuck
}

/// Synchronise sensor outputs to the fusion clock.
/// Marks gaps, dropouts, and stuck-at windows.
pub fn synchronise(sensors: &SensorOutputs) -> SyncOutput {
    let n = sensors.t.len();
    let window = 20; // 100 ms at 200 Hz

    // Detect stuck-at in each channel
    let stuck_body = detect_stuck(&sensors.body_acc, window);
    let stuck_wheel = detect_stuck(&sensors.wheel_acc, window);
    let stuck_travel = detect_stuck(&sensors.susp_travel, window);

    let mut frames = Vec::with_capacity(n);

    for i in 0..n {
        let ba = sensors.body_acc[i];
        let wa = sensors.wheel_acc[i];
        let st = sensors.susp_travel[i];
        let ws = sensors.wheel_speed[i];
        let vs = sensors.veh_speed[i];

        frames.push(SyncFrame {
            t: sensors.t[i],
            body_acc: ba,
            wheel_acc: wa,
            susp_travel: st,
            wheel_speed: ws,
            veh_speed: vs,
            body_acc_valid: ba.is_some() && !stuck_body[i],
            wheel_acc_valid: wa.is_some() && !stuck_wheel[i],
            susp_travel_valid: st.is_some() && !stuck_travel[i],
            wheel_speed_valid: ws.is_some(),
            veh_speed_valid: vs.is_some(),
        });
    }

    // Delay estimation: for now report the configured delays.
    // A full model-based search would run the filter on a clean segment
    // for each candidate delay. We implement a simplified version:
    // compare body_acc variance with shifted versions to find alignment.
    let estimated_delays = estimate_delays(sensors);

    SyncOutput { frames, estimated_delays }
}

/// Model-based delay estimation.
///
/// For each channel, try shifting by ±N samples and find the shift
/// that minimises the residual between the signal and a simple model prediction.
/// For body_acc: use the derivative of susp_travel as a proxy.
fn estimate_delays(sensors: &SensorOutputs) -> Vec<(String, f64)> {
    let mut results = Vec::new();

    // Body acc delay estimation: correlate with finite differences of travel
    let body_delay = estimate_delay_by_coherence(
        &sensors.body_acc,
        &sensors.susp_travel,
        10, // search ±10 samples (±50 ms at 200 Hz)
    );
    results.push(("body_acc".to_string(), body_delay));

    // Wheel acc
    let wheel_delay = estimate_delay_by_coherence(
        &sensors.wheel_acc,
        &sensors.susp_travel,
        10,
    );
    results.push(("wheel_acc".to_string(), wheel_delay));

    // Susp travel: reference channel, delay = 0 by definition
    results.push(("susp_travel".to_string(), 0.0));

    results
}

/// Estimate delay between two channels using cross-energy minimisation.
fn estimate_delay_by_coherence(
    signal: &[Option<f64>],
    reference: &[Option<f64>],
    max_shift: usize,
) -> f64 {
    let n = signal.len().min(reference.len());
    if n < 2 * max_shift + 10 { return 0.0; }

    // Compute finite differences of reference as a proxy for acceleration
    let ref_diff: Vec<f64> = (1..n)
        .map(|i| {
            match (reference[i], reference[i - 1]) {
                (Some(a), Some(b)) => a - b,
                _ => 0.0,
            }
        })
        .collect();

    let mut best_shift: i32 = 0;
    let mut best_energy = f64::MAX;

    for shift in -(max_shift as i32)..=(max_shift as i32) {
        let mut energy = 0.0;
        let mut count = 0;

        for i in max_shift..n.saturating_sub(max_shift + 1) {
            let si = (i as i32 + shift) as usize;
            if si >= n || si == 0 { continue; }

            if let (Some(s), rd) = (signal[i], ref_diff[si.saturating_sub(1)]) {
                let residual = s - rd * 200.0 * 200.0; // scale rough proxy
                energy += residual * residual;
                count += 1;
            }
        }

        if count > 0 {
            let avg_energy = energy / count as f64;
            if avg_energy < best_energy {
                best_energy = avg_energy;
                best_shift = shift;
            }
        }
    }

    best_shift as f64
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::SensorConfig;
    use crate::plant;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    #[test]
    fn sync_produces_correct_length() {
        let n = 20_001;
        let dt = 0.001;
        let road: Vec<f64> = (0..n)
            .map(|i| if i as f64 * dt > 0.5 { 0.05 } else { 0.0 })
            .collect();
        let plant_out = plant::run_nominal(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 0.12, &road, dt);
        let configs = SensorConfig::defaults();
        let mut rng = ChaCha8Rng::seed_from_u64(42);
        let sensors = crate::sensors::simulate_sensors(&plant_out, dt, 20.0, &configs, 5.0, &mut rng);
        let sync = synchronise(&sensors);

        assert_eq!(sync.frames.len(), sensors.t.len());
    }

    #[test]
    fn stuck_at_detected() {
        // Create a signal with a stuck-at window
        let mut values: Vec<Option<f64>> = (0..200).map(|i| Some(i as f64 * 0.01)).collect();
        // Inject stuck-at: samples 50–80 all same value
        for i in 50..80 {
            values[i] = Some(0.5);
        }
        let stuck = detect_stuck(&values, 10);
        // Some of the stuck window should be detected
        let stuck_count: usize = stuck[50..80].iter().filter(|&&s| s).count();
        assert!(stuck_count > 5, "Should detect stuck-at region");
    }
}

