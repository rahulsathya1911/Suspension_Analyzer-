//! Configuration types for vehicle, sensors, and scenario matrix.

use serde::{Deserialize, Serialize};

/// Nominal vehicle parameters — CONTRACTS §2 conventions.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleConfig {
    pub ms_kg: f64,
    pub mu_kg: f64,
    pub k_nm: f64,
    pub kt_nm: f64,
    pub travel_limit_mm: f64,
    pub presets: DampingPresets,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DampingPresets {
    pub soft: f64,
    pub medium: f64,
    pub firm: f64,
}

impl VehicleConfig {
    /// Default nominal vehicle from CONTRACTS §2.
    pub fn nominal() -> Self {
        Self {
            ms_kg: 440.0,
            mu_kg: 78.0,
            k_nm: 21_000.0,
            kt_nm: 200_000.0,
            travel_limit_mm: 120.0,
            presets: DampingPresets {
                soft: 1000.0,
                medium: 1800.0,
                firm: 3000.0,
            },
        }
    }

    /// Look up damping preset name from c value.
    pub fn preset_name(&self, c_nsm: f64) -> &'static str {
        if (c_nsm - self.presets.soft).abs() < 1.0 {
            "soft"
        } else if (c_nsm - self.presets.medium).abs() < 1.0 {
            "medium"
        } else if (c_nsm - self.presets.firm).abs() < 1.0 {
            "firm"
        } else {
            "medium" // default for OOD
        }
    }
}

/// Sensor configuration for one channel.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorConfig {
    pub channel: String,
    pub rate_hz: f64,
    pub sigma: f64,
    pub bias: f64,
    pub delay_ms: f64,
}

impl SensorConfig {
    /// Default sensor configs from PRD §5.3.
    pub fn defaults() -> Vec<Self> {
        vec![
            Self { channel: "body_acc".into(),    rate_hz: 200.0, sigma: 0.05,   bias: 0.02,   delay_ms: 5.0  },
            Self { channel: "wheel_acc".into(),   rate_hz: 200.0, sigma: 0.5,    bias: 0.1,    delay_ms: 5.0  },
            Self { channel: "susp_travel".into(), rate_hz: 100.0, sigma: 0.0005, bias: 0.0005, delay_ms: 10.0 },
            Self { channel: "wheel_speed".into(), rate_hz: 50.0,  sigma: 0.05,   bias: 0.0,    delay_ms: 20.0 },
            Self { channel: "veh_speed".into(),   rate_hz: 20.0,  sigma: 0.1,    bias: 0.0,    delay_ms: 50.0 },
        ]
    }
}

/// Perturbation ranges for the perturbed plant.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerturbationRanges {
    pub k_pct: f64,
    pub kt_pct: f64,
    pub asym_rebound_ratio: f64,
}

impl Default for PerturbationRanges {
    fn default() -> Self {
        Self {
            k_pct: 10.0,
            kt_pct: 5.0,
            asym_rebound_ratio: 2.0,
        }
    }
}

/// ISO 8608 roughness coefficients at n0 = 0.1 cycles/m [m³].
pub fn road_class_gd(class: &str) -> Option<f64> {
    match class {
        "A" => Some(16e-6),
        "B" => Some(64e-6),
        "C" => Some(256e-6),
        "D" => Some(1024e-6),
        _ => None,
    }
}

/// Scenario matrix parsed from TOML.
#[derive(Debug, Clone, Deserialize)]
pub struct MatrixConfig {
    pub road_classes: Vec<String>,
    pub speeds_mps: Vec<f64>,
    pub loads_kg: Vec<f64>,
    pub presets: Vec<MatrixPreset>,
    pub dropout_pcts: Vec<f64>,
    pub seeds_train: Vec<u64>,
    pub seeds_test: Vec<u64>,
    pub duration_s: f64,
    pub ood: Option<OodConfig>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct MatrixPreset {
    pub name: String,
    pub c_nsm: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct OodConfig {
    pub road_class: String,
    pub speed_mps: f64,
    pub load_kg: f64,
    pub dropout_pct: f64,
    pub c_nsm: f64,
    pub seeds: Vec<u64>,
}

