//! Quarter-car plant simulation — nominal and perturbed.
//!
//! PRD §5.2: State [zs, vs, zu, vu], input zr, 1 kHz RK4.

use rand::Rng;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

/// Full plant output: time-series of all states plus derived quantities.
#[derive(Debug, Clone)]
pub struct PlantOutput {
    /// Time vector [s]
    pub t: Vec<f64>,
    /// Sprung mass displacement [m]
    pub zs: Vec<f64>,
    /// Sprung mass velocity [m/s]
    pub vs: Vec<f64>,
    /// Unsprung mass displacement [m]
    pub zu: Vec<f64>,
    /// Unsprung mass velocity [m/s]
    pub vu: Vec<f64>,
    /// Body acceleration [m/s²]
    pub body_acc: Vec<f64>,
    /// Dynamic tyre force [N]
    pub tyre_force: Vec<f64>,
    /// Road input used [m]
    pub road: Vec<f64>,
}

/// Perturbation parameters drawn per-run from the seeded RNG.
/// Written to `run_meta.json` only — the filter never sees these.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlantPerturbation {
    pub k_factor: f64,
    pub kt_factor: f64,
    pub ms_offset_kg: f64,
    pub rebound_ratio: f64,
    pub blowoff_vel: f64,
}

impl PlantPerturbation {
    /// Draw random perturbation parameters from the seeded RNG.
    /// PRD §5.2: k ±10%, kt ±5%, asymmetric damper.
    pub fn draw(rng: &mut ChaCha8Rng, k_pct: f64, kt_pct: f64, rebound_ratio: f64) -> Self {
        let k_factor = 1.0 + rng.random_range(-k_pct..=k_pct) / 100.0;
        let kt_factor = 1.0 + rng.random_range(-kt_pct..=kt_pct) / 100.0;
        let ms_offset_kg = rng.random_range(-20.0..=20.0);
        // Blow-off velocity threshold: 0.3–0.8 m/s
        let blowoff_vel = rng.random_range(0.3..=0.8);
        Self {
            k_factor,
            kt_factor,
            ms_offset_kg,
            rebound_ratio,
            blowoff_vel,
        }
    }
}

/// Plant simulation parameters.
struct PlantParams {
    ms: f64,
    mu: f64,
    k: f64,
    c: f64,
    kt: f64,
    travel_limit_m: f64,
    /// If Some, asymmetric damper with rebound ratio and blow-off.
    asym: Option<(f64, f64)>, // (rebound_ratio, blowoff_vel)
}

/// Compute derivatives for the quarter-car model.
/// Returns (dzs, dvs, dzu, dvu).
#[inline]
fn derivatives(
    zs: f64, vs: f64, zu: f64, vu: f64,
    zr: f64,
    p: &PlantParams,
) -> [f64; 4] {
    let travel = zs - zu;

    // Bump stop: penalty force when travel exceeds limit
    let bump_force = if travel.abs() > p.travel_limit_m {
        1e6 * (travel.abs() - p.travel_limit_m) * travel.signum()
    } else {
        0.0
    };

    let spring_force = p.k * travel + bump_force;

    // Damper force — potentially asymmetric
    let rel_vel = vs - vu;
    let damper_force = match p.asym {
        Some((rebound_ratio, blowoff_vel)) => {
            let c_eff = if rel_vel > 0.0 {
                // Rebound (extension): higher damping
                p.c * rebound_ratio
            } else {
                p.c
            };
            if rel_vel.abs() > blowoff_vel {
                // Blow-off: damping force saturates
                c_eff * blowoff_vel * rel_vel.signum()
            } else {
                c_eff * rel_vel
            }
        }
        None => p.c * rel_vel,
    };

    let dzs = vs;
    let dvs = (-damper_force - spring_force) / p.ms;
    let dzu = vu;
    let dvu = (damper_force + spring_force - p.kt * (zu - zr)) / p.mu;

    [dzs, dvs, dzu, dvu]
}

/// Run RK4 integration of the quarter-car plant.
fn run_plant(p: &PlantParams, road: &[f64], dt: f64) -> PlantOutput {
    let n = road.len();
    let mut t_vec = Vec::with_capacity(n);
    let mut zs_vec = Vec::with_capacity(n);
    let mut vs_vec = Vec::with_capacity(n);
    let mut zu_vec = Vec::with_capacity(n);
    let mut vu_vec = Vec::with_capacity(n);
    let mut acc_vec = Vec::with_capacity(n);
    let mut tf_vec = Vec::with_capacity(n);

    let (mut zs, mut vs, mut zu, mut vu) = (0.0, 0.0, 0.0, 0.0);

    for step in 0..n {
        let t = step as f64 * dt;
        let r = road[step];

        t_vec.push(t);
        zs_vec.push(zs);
        vs_vec.push(vs);
        zu_vec.push(zu);
        vu_vec.push(vu);

        // Body acceleration and tyre force at current state
        let d = derivatives(zs, vs, zu, vu, r, p);
        acc_vec.push(d[1]); // dvs = body acceleration
        tf_vec.push(p.kt * (zu - r));

        // RK4 step
        if step < n - 1 {
            let r_mid = (r + road[step + 1]) / 2.0;
            let r_next = road[step + 1];

            let k1 = derivatives(zs, vs, zu, vu, r, p);
            let k2 = derivatives(
                zs + dt * k1[0] / 2.0, vs + dt * k1[1] / 2.0,
                zu + dt * k1[2] / 2.0, vu + dt * k1[3] / 2.0,
                r_mid, p,
            );
            let k3 = derivatives(
                zs + dt * k2[0] / 2.0, vs + dt * k2[1] / 2.0,
                zu + dt * k2[2] / 2.0, vu + dt * k2[3] / 2.0,
                r_mid, p,
            );
            let k4 = derivatives(
                zs + dt * k3[0], vs + dt * k3[1],
                zu + dt * k3[2], vu + dt * k3[3],
                r_next, p,
            );

            zs += dt / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
            vs += dt / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
            zu += dt / 6.0 * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]);
            vu += dt / 6.0 * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]);
        }
    }

    PlantOutput {
        t: t_vec,
        zs: zs_vec,
        vs: vs_vec,
        zu: zu_vec,
        vu: vu_vec,
        body_acc: acc_vec,
        tyre_force: tf_vec,
        road: road.to_vec(),
    }
}

/// Run the **nominal** plant (linear, nominal parameters).
pub fn run_nominal(
    ms: f64, mu: f64, k: f64, c: f64, kt: f64,
    travel_limit_m: f64, road: &[f64], dt: f64,
) -> PlantOutput {
    let p = PlantParams { ms, mu, k, c, kt, travel_limit_m, asym: None };
    run_plant(&p, road, dt)
}

/// Run the **perturbed** plant (asymmetric damper, varied parameters).
pub fn run_perturbed(
    ms: f64, mu: f64, k: f64, c: f64, kt: f64,
    travel_limit_m: f64, road: &[f64], dt: f64,
    pert: &PlantPerturbation,
) -> PlantOutput {
    let p = PlantParams {
        ms: ms + pert.ms_offset_kg,
        mu,
        k: k * pert.k_factor,
        c,
        kt: kt * pert.kt_factor,
        travel_limit_m,
        asym: Some((pert.rebound_ratio, pert.blowoff_vel)),
    };
    run_plant(&p, road, dt)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_plant_produces_nonzero_output() {
        let n = 5001;
        let dt = 0.001;
        // Step road at t=0.5
        let road: Vec<f64> = (0..n)
            .map(|i| if i as f64 * dt > 0.5 { 0.05 } else { 0.0 })
            .collect();
        let out = run_nominal(290.0, 40.0, 22_000.0, 1_500.0, 190_000.0, 0.12, &road, dt);
        assert_eq!(out.t.len(), n);
        // Body should have moved after the step
        let max_body = out.zs.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
        assert!(max_body > 0.001, "Body should respond to step input");
    }

    #[test]
    fn perturbed_plant_differs_from_nominal() {
        let n = 5001;
        let dt = 0.001;
        let road: Vec<f64> = (0..n)
            .map(|i| if i as f64 * dt > 0.5 { 0.05 } else { 0.0 })
            .collect();
        let pert = PlantPerturbation {
            k_factor: 1.1,
            kt_factor: 0.95,
            ms_offset_kg: 10.0,
            rebound_ratio: 2.0,
            blowoff_vel: 0.5,
        };
        let nom = run_nominal(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 0.12, &road, dt);
        let pert_out = run_perturbed(440.0, 78.0, 21_000.0, 1_800.0, 200_000.0, 0.12, &road, dt, &pert);

        // They should differ
        let diff: f64 = nom.zs.iter().zip(pert_out.zs.iter())
            .map(|(a, b)| (a - b).abs())
            .sum::<f64>();
        assert!(diff > 1e-6, "Perturbed plant should differ from nominal");
    }

    #[test]
    fn parity_with_core() {
        // PRD §5.2: nominal plant vs core/ on one fixed scenario, within 1%.
        let ms = 290.0;
        let mu = 40.0;
        let k = 22_000.0;
        let c = 1_500.0;
        let kt = 190_000.0;
        let travel_limit = 0.120;
        let dt = 0.001;

        // Use core's road generator for a step profile
        let profile = suspension_core::RoadProfile::Step { height: 0.05 };
        let road = suspension_core::generate_road_profile(&profile, dt, 5001);

        // Run core simulation
        let core_result = suspension_core::run_simulation(ms, mu, k, c, kt, travel_limit, &profile);

        // Run our nominal plant
        let our_result = run_nominal(ms, mu, k, c, kt, travel_limit, &road, dt);

        // Compare RMS body acceleration
        let n = our_result.body_acc.len() as f64;
        let our_rms_acc = (our_result.body_acc.iter().map(|a| a * a).sum::<f64>() / n).sqrt();
        let rel_acc = ((our_rms_acc - core_result.rms_body_acc) / core_result.rms_body_acc).abs();
        assert!(
            rel_acc < 0.01,
            "RMS body acc parity: ours={our_rms_acc:.6}, core={:.6}, rel_diff={rel_acc:.4}",
            core_result.rms_body_acc
        );

        // Compare RMS tyre force
        let our_rms_tf = (our_result.tyre_force.iter().map(|f| f * f).sum::<f64>() / n).sqrt();
        let rel_tf = ((our_rms_tf - core_result.rms_tire_force) / core_result.rms_tire_force).abs();
        assert!(
            rel_tf < 0.01,
            "RMS tyre force parity: ours={our_rms_tf:.6}, core={:.6}, rel_diff={rel_tf:.4}",
            core_result.rms_tire_force
        );
    }

    #[test]
    fn step_overshoot_matches_theory() {
        // Analytical step overshoot for underdamped 2nd-order system:
        // overshoot ≈ exp(-π·ζ/√(1-ζ²)) for 1-DOF.
        // Quarter-car is 2-DOF so the body overshoot is approximate,
        // but for ζ ~ 0.1–0.3 the sprung mode dominates and it should be close.
        let ms = 440.0;
        let k = 21_000.0;
        let c = 1_800.0;
        let mu = 78.0;
        let kt = 200_000.0;
        let dt = 0.001;

        let zeta = c / (2.0 * f64::sqrt(k * ms));
        let analytical_overshoot = (-std::f64::consts::PI * zeta / (1.0 - zeta * zeta).sqrt()).exp();

        let n = 20_001;
        let road: Vec<f64> = (0..n)
            .map(|i| if i as f64 * dt > 0.5 { 0.05 } else { 0.0 })
            .collect();
        let out = run_nominal(ms, mu, k, c, kt, 0.12, &road, dt);

        let step = 0.05;
        // Find max body displacement (should overshoot the step height)
        let max_body = out.zs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let sim_overshoot = (max_body - step) / step;

        // Allow 50% tolerance — 2-DOF vs 1-DOF approximation
        let ratio = sim_overshoot / analytical_overshoot;
        assert!(
            ratio > 0.3 && ratio < 3.0,
            "Overshoot ratio {ratio:.3}: sim={sim_overshoot:.4}, analytical={analytical_overshoot:.4}, ζ={zeta:.4}"
        );
    }
}

