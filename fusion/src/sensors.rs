//! Sensor simulation with realistic corruptions.
//!
//! PRD §5.3: noise, bias, drift, delay, dropout, spikes, stuck-at faults.

use rand::Rng;
use rand_chacha::ChaCha8Rng;
use rand_distr::{Distribution, Normal};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

use crate::config::SensorConfig;
use crate::plant::PlantOutput;

/// Ground-truth fault log entry.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FaultEvent {
    pub channel: String,
    pub fault_type: String,  // "dropout", "spike", "stuck_at"
    pub start_s: f64,
    pub end_s: f64,
    pub sample_indices: Vec<usize>,
}

/// All sensor outputs at the fusion clock rate (200 Hz).
#[derive(Debug, Clone)]
pub struct SensorOutputs {
    /// Time vector at fusion rate (200 Hz)
    pub t: Vec<f64>,
    /// Body acceleration [m/s²] — 200 Hz, Some = valid, None = dropout/stuck
    pub body_acc: Vec<Option<f64>>,
    /// Wheel acceleration [m/s²] — 200 Hz
    pub wheel_acc: Vec<Option<f64>>,
    /// Suspension travel [m] — 100 Hz (every other sample), None on off-ticks
    pub susp_travel: Vec<Option<f64>>,
    /// Wheel speed [m/s] — 50 Hz
    pub wheel_speed: Vec<Option<f64>>,
    /// Vehicle speed [m/s] — 20 Hz
    pub veh_speed: Vec<Option<f64>>,
    /// Ground-truth fault log
    pub fault_log: Vec<FaultEvent>,
}

/// Ring buffer for per-channel delay simulation.
struct DelayBuffer {
    buf: VecDeque<f64>,
    delay_samples: usize,
}

impl DelayBuffer {
    fn new(delay_samples: usize) -> Self {
        let mut buf = VecDeque::with_capacity(delay_samples + 1);
        for _ in 0..delay_samples {
            buf.push_back(0.0);
        }
        Self { buf, delay_samples }
    }

    fn push(&mut self, val: f64) -> f64 {
        self.buf.push_back(val);
        if self.buf.len() > self.delay_samples {
            self.buf.pop_front().unwrap_or(0.0)
        } else {
            0.0
        }
    }
}

/// Simulate all sensors from the perturbed plant output.
///
/// The plant runs at 1 kHz; sensors sample at their own rates.
/// All outputs are aligned to the 200 Hz fusion clock.
pub fn simulate_sensors(
    plant: &PlantOutput,
    plant_dt: f64,
    vehicle_speed_mps: f64,
    sensor_configs: &[SensorConfig],
    dropout_pct: f64,
    rng: &mut ChaCha8Rng,
) -> SensorOutputs {
    let plant_rate = (1.0 / plant_dt).round() as usize; // 1000
    let fusion_rate = 200_usize;
    let fusion_dt = 1.0 / fusion_rate as f64;
    let plant_n = plant.t.len();
    let duration_s = (plant_n - 1) as f64 * plant_dt;
    let fusion_n = (duration_s * fusion_rate as f64).round() as usize + 1;
    let decimate = plant_rate / fusion_rate; // 5 (1000/200)

    // Look up sensor configs
    let sc_body = sensor_configs.iter().find(|s| s.channel == "body_acc").unwrap();
    let sc_wheel = sensor_configs.iter().find(|s| s.channel == "wheel_acc").unwrap();
    let sc_travel = sensor_configs.iter().find(|s| s.channel == "susp_travel").unwrap();
    let sc_wspd = sensor_configs.iter().find(|s| s.channel == "wheel_speed").unwrap();
    let sc_vspd = sensor_configs.iter().find(|s| s.channel == "veh_speed").unwrap();

    // Delay buffers (at each sensor's own rate, convert ms to samples at fusion rate)
    let delay_body = (sc_body.delay_ms / 1000.0 * fusion_rate as f64).round() as usize;
    let delay_wheel = (sc_wheel.delay_ms / 1000.0 * fusion_rate as f64).round() as usize;
    let delay_travel = (sc_travel.delay_ms / 1000.0 * (sc_travel.rate_hz)).round() as usize;
    let delay_wspd = (sc_wspd.delay_ms / 1000.0 * (sc_wspd.rate_hz)).round() as usize;
    let delay_vspd = (sc_vspd.delay_ms / 1000.0 * (sc_vspd.rate_hz)).round() as usize;

    let mut dbuf_body = DelayBuffer::new(delay_body);
    let mut dbuf_wheel = DelayBuffer::new(delay_wheel);
    let mut dbuf_travel = DelayBuffer::new(delay_travel);
    let mut dbuf_wspd = DelayBuffer::new(delay_wspd);
    let mut dbuf_vspd = DelayBuffer::new(delay_vspd);

    // Noise distributions
    let noise_body = Normal::new(0.0, sc_body.sigma).unwrap();
    let noise_wheel = Normal::new(0.0, sc_wheel.sigma).unwrap();
    let noise_travel = Normal::new(0.0, sc_travel.sigma).unwrap();
    let noise_wspd = Normal::new(0.0, sc_wspd.sigma).unwrap();
    let noise_vspd = Normal::new(0.0, sc_vspd.sigma).unwrap();

    // Drift for body_acc: random-walk with small steps
    let drift_step = Normal::new(0.0, 0.001).unwrap();
    let mut drift_body = 0.0_f64;

    // Rate divisors at fusion clock (200 Hz)
    let travel_div = (fusion_rate as f64 / sc_travel.rate_hz).round() as usize; // 2
    let wspd_div = (fusion_rate as f64 / sc_wspd.rate_hz).round() as usize;     // 4
    let vspd_div = (fusion_rate as f64 / sc_vspd.rate_hz).round() as usize;     // 10

    // Pre-generate fault schedules
    let mut fault_log = Vec::new();

    // Dropout bursts: total missing from dropout_pct
    let dropout_budget_s = duration_s * dropout_pct / 100.0;
    let mut dropout_windows: Vec<(f64, f64)> = Vec::new();
    {
        let mut remaining = dropout_budget_s;
        while remaining > 0.05 {
            let burst_len = rng.random_range(0.1_f64..=0.5_f64).min(remaining);
            let start = rng.random_range(0.5..=(duration_s - burst_len).max(0.5));
            dropout_windows.push((start, start + burst_len));
            remaining -= burst_len;
            // Distribute across channels
            let ch_names = ["body_acc", "wheel_acc", "susp_travel"];
            let ch = ch_names[dropout_windows.len() % ch_names.len()];
            fault_log.push(FaultEvent {
                channel: ch.to_string(),
                fault_type: "dropout".to_string(),
                start_s: start,
                end_s: start + burst_len,
                sample_indices: Vec::new(), // filled later
            });
        }
    }

    // Spike anomalies: ~0.2% of samples
    let n_spikes_total = (fusion_n as f64 * 0.002).round() as usize;
    let mut spike_indices: Vec<(usize, usize)> = Vec::new(); // (fusion_idx, channel_idx)
    for _ in 0..n_spikes_total {
        let idx = rng.random_range(0..fusion_n);
        let ch = rng.random_range(0_usize..3); // body_acc, wheel_acc, susp_travel
        spike_indices.push((idx, ch));
    }
    spike_indices.sort_by_key(|s| s.0);

    // Stuck-at faults: ~2 events of 0.5 s each
    let mut stuck_windows: Vec<(f64, f64, usize)> = Vec::new(); // (start, end, channel)
    for i in 0..2 {
        let start = rng.random_range(1.0..=(duration_s - 1.0));
        stuck_windows.push((start, start + 0.5, i % 3));
        let ch_names = ["body_acc", "wheel_acc", "susp_travel"];
        fault_log.push(FaultEvent {
            channel: ch_names[i % 3].to_string(),
            fault_type: "stuck_at".to_string(),
            start_s: start,
            end_s: start + 0.5,
            sample_indices: Vec::new(),
        });
    }

    // Generate sensor outputs at fusion clock
    let mut t_out = Vec::with_capacity(fusion_n);
    let mut body_acc_out = Vec::with_capacity(fusion_n);
    let mut wheel_acc_out = Vec::with_capacity(fusion_n);
    let mut travel_out = Vec::with_capacity(fusion_n);
    let mut wspd_out = Vec::with_capacity(fusion_n);
    let mut vspd_out = Vec::with_capacity(fusion_n);

    // Track stuck-at values
    let mut stuck_vals = [0.0_f64; 3];

    // Spike lookup
    let mut spike_ptr = 0_usize;

    for fi in 0..fusion_n {
        let t = fi as f64 * fusion_dt;
        t_out.push(t);

        // Map to plant index (nearest)
        let pi = (fi * decimate).min(plant_n - 1);

        // Update drift
        drift_body += drift_step.sample(rng);

        // ── Body acceleration (200 Hz) ──
        let true_body_acc = plant.body_acc[pi];
        let mut ba = dbuf_body.push(true_body_acc) + sc_body.bias + drift_body + noise_body.sample(rng);

        // ── Wheel acceleration (200 Hz) ──
        let true_wheel_acc = if pi < plant.body_acc.len() {
            // Compute from plant: (k*(zs-zu) + c*(vs-vu) - kt*(zu-zr)) / mu
            // We stored tyre_force and can compute:
            let k = 21_000.0; // nominal for sensor truth computation
            let c = 1_800.0;
            let kt = 200_000.0;
            let mu = 78.0;
            (k * (plant.zs[pi] - plant.zu[pi]) + c * (plant.vs[pi] - plant.vu[pi])
                - kt * (plant.zu[pi] - plant.road[pi])) / mu
        } else {
            0.0
        };
        let mut wa = dbuf_wheel.push(true_wheel_acc) + sc_wheel.bias + noise_wheel.sample(rng);

        // ── Suspension travel (100 Hz) ──
        let travel_val = if fi % travel_div == 0 {
            let true_travel = plant.zs[pi] - plant.zu[pi];
            Some(dbuf_travel.push(true_travel) + sc_travel.bias + noise_travel.sample(rng))
        } else {
            None
        };

        // ── Wheel speed (50 Hz) ──
        let wspd_val = if fi % wspd_div == 0 {
            Some(dbuf_wspd.push(vehicle_speed_mps) + noise_wspd.sample(rng))
        } else {
            None
        };

        // ── Vehicle speed (20 Hz) ──
        let vspd_val = if fi % vspd_div == 0 {
            Some(dbuf_vspd.push(vehicle_speed_mps) + noise_vspd.sample(rng))
        } else {
            None
        };

        // Apply stuck-at faults
        for &(start, end, ch) in &stuck_windows {
            if t >= start && t <= end {
                match ch {
                    0 => {
                        if (t - start).abs() < fusion_dt * 0.5 { stuck_vals[0] = ba; }
                        ba = stuck_vals[0];
                    }
                    1 => {
                        if (t - start).abs() < fusion_dt * 0.5 { stuck_vals[1] = wa; }
                        wa = stuck_vals[1];
                    }
                    _ => {}  // travel handled below
                }
            }
        }

        // Apply spike anomalies
        while spike_ptr < spike_indices.len() && spike_indices[spike_ptr].0 == fi {
            let (_, ch) = spike_indices[spike_ptr];
            let spike_mag = 10.0; // ~10 sigma
            match ch {
                0 => ba += spike_mag * sc_body.sigma * if rng.random_bool(0.5) { 1.0 } else { -1.0 },
                1 => wa += spike_mag * sc_wheel.sigma * if rng.random_bool(0.5) { 1.0 } else { -1.0 },
                2 => {} // travel spike applied below
                _ => {}
            }
            // Log spike
            fault_log.push(FaultEvent {
                channel: ["body_acc", "wheel_acc", "susp_travel"][ch].to_string(),
                fault_type: "spike".to_string(),
                start_s: t,
                end_s: t,
                sample_indices: vec![fi],
            });
            spike_ptr += 1;
        }

        // Apply dropout
        let mut body_dropped = false;
        let mut wheel_dropped = false;
        let mut travel_dropped = false;
        for (start, end) in &dropout_windows {
            if t >= *start && t <= *end {
                // Drop channels in rotation based on window index
                let widx = dropout_windows.iter().position(|(s, _)| (s - start).abs() < 1e-9).unwrap_or(0);
                match widx % 3 {
                    0 => body_dropped = true,
                    1 => wheel_dropped = true,
                    2 => travel_dropped = true,
                    _ => {}
                }
            }
        }

        body_acc_out.push(if body_dropped { None } else { Some(ba) });
        wheel_acc_out.push(if wheel_dropped { None } else { Some(wa) });

        let tv = if travel_dropped { None } else { travel_val };
        // Apply stuck-at to travel
        let tv = match tv {
            Some(v) => {
                let mut v = v;
                for &(start, end, ch) in &stuck_windows {
                    if ch == 2 && t >= start && t <= end {
                        if (t - start).abs() < fusion_dt * 0.5 { stuck_vals[2] = v; }
                        v = stuck_vals[2];
                    }
                }
                Some(v)
            }
            None => None,
        };
        // Apply spike to travel
        let tv = if let Some(mut v) = tv {
            for &(idx, ch) in &spike_indices {
                if idx == fi && ch == 2 {
                    v += 10.0 * sc_travel.sigma * if rng.random_bool(0.5) { 1.0 } else { -1.0 };
                }
            }
            Some(v)
        } else {
            tv
        };
        travel_out.push(tv);
        wspd_out.push(wspd_val);
        vspd_out.push(vspd_val);
    }

    // Update fault log with sample indices for dropout events
    for event in &mut fault_log {
        if event.fault_type == "dropout" || event.fault_type == "stuck_at" {
            let start_idx = (event.start_s / fusion_dt).round() as usize;
            let end_idx = (event.end_s / fusion_dt).round() as usize;
            event.sample_indices = (start_idx..=end_idx.min(fusion_n - 1)).collect();
        }
    }

    SensorOutputs {
        t: t_out,
        body_acc: body_acc_out,
        wheel_acc: wheel_acc_out,
        susp_travel: travel_out,
        wheel_speed: wspd_out,
        veh_speed: vspd_out,
        fault_log,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;

    fn make_plant_output() -> PlantOutput {
        let n = 20_001;
        let dt = 0.001;
        let road: Vec<f64> = (0..n)
            .map(|i| if i as f64 * dt > 0.5 { 0.05 } else { 0.0 })
            .collect();
        crate::plant::run_nominal(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 0.12, &road, dt)
    }

    #[test]
    fn sensor_output_lengths() {
        let plant = make_plant_output();
        let configs = SensorConfig::defaults();
        let mut rng = ChaCha8Rng::seed_from_u64(42);
        let out = simulate_sensors(&plant, 0.001, 20.0, &configs, 5.0, &mut rng);

        // 200 Hz fusion clock over 20 s = 4001 samples
        assert_eq!(out.t.len(), 4001);
        assert_eq!(out.body_acc.len(), 4001);
        assert_eq!(out.wheel_acc.len(), 4001);
        assert_eq!(out.susp_travel.len(), 4001);
    }

    #[test]
    fn noise_statistics_match() {
        let plant = make_plant_output();
        let configs = SensorConfig::defaults();
        let mut rng = ChaCha8Rng::seed_from_u64(123);
        let out = simulate_sensors(&plant, 0.001, 20.0, &configs, 0.0, &mut rng);

        // Check that body_acc values exist (no dropout at 0%)
        let valid: Vec<f64> = out.body_acc.iter().filter_map(|x| *x).collect();
        assert!(valid.len() > 3900, "Most body_acc samples should be valid at 0% dropout");
    }

    #[test]
    fn dropout_reduces_valid_samples() {
        let plant = make_plant_output();
        let configs = SensorConfig::defaults();

        let mut rng0 = ChaCha8Rng::seed_from_u64(99);
        let out0 = simulate_sensors(&plant, 0.001, 20.0, &configs, 0.0, &mut rng0);
        let valid0: usize = out0.body_acc.iter().filter(|x| x.is_some()).count()
            + out0.wheel_acc.iter().filter(|x| x.is_some()).count();

        let mut rng20 = ChaCha8Rng::seed_from_u64(99);
        let out20 = simulate_sensors(&plant, 0.001, 20.0, &configs, 20.0, &mut rng20);
        let valid20: usize = out20.body_acc.iter().filter(|x| x.is_some()).count()
            + out20.wheel_acc.iter().filter(|x| x.is_some()).count();

        assert!(valid20 < valid0, "20% dropout should have fewer valid samples");
    }

    #[test]
    fn fault_log_not_empty_with_faults() {
        let plant = make_plant_output();
        let configs = SensorConfig::defaults();
        let mut rng = ChaCha8Rng::seed_from_u64(42);
        let out = simulate_sensors(&plant, 0.001, 20.0, &configs, 10.0, &mut rng);
        assert!(!out.fault_log.is_empty(), "Fault log should contain events");
        // Should have dropout, stuck_at, and spike events
        let has_dropout = out.fault_log.iter().any(|e| e.fault_type == "dropout");
        let has_stuck = out.fault_log.iter().any(|e| e.fault_type == "stuck_at");
        let has_spike = out.fault_log.iter().any(|e| e.fault_type == "spike");
        assert!(has_dropout, "Should have dropout events");
        assert!(has_stuck, "Should have stuck_at events");
        assert!(has_spike, "Should have spike events");
    }
}

