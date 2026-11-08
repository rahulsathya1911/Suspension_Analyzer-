use std::path::{Path, PathBuf};

use anyhow::Result;
use clap::{Parser, Subcommand};
use rayon::prelude::*;
use serde_json::json;

use suspension_fusion::config::{PerturbationRanges, SensorConfig, VehicleConfig};
use suspension_fusion::export::{write_fusion_summary, write_run_meta, write_timeseries};
use suspension_fusion::metrics::{build_fusion_summary, RunStats};
use suspension_fusion::{run_scenario_diag, DatasetRow, ScenarioSpec};

#[derive(Parser)]
#[command(name = "fusion_cli")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Run the full scenario matrix and write the dataset, summary, timeseries and run_meta.json
    Matrix {
        #[arg(long, default_value = "out/")]
        out: PathBuf,
    },
}

fn spec(rc: &str, v: f64, m: f64, c: f64, d: f64, seed: u64) -> ScenarioSpec {
    ScenarioSpec {
        road_class: rc.to_string(),
        speed_mps: v,
        ms_kg: m,
        c_nsm: c,
        dropout_pct: d,
        seed,
        duration_s: 20.0,
        k_nm: None,
        mu_kg: None,
        kt_nm: None,
    }
}

fn run_matrix(out: &Path) -> Result<()> {
    let presets = [1000.0, 1800.0, 3000.0];
    let mut jobs: Vec<(&'static str, ScenarioSpec)> = Vec::new();

    for (split, seeds) in [("train", 0..10u64), ("test", 100..105u64)] {
        for rc in ["A", "B", "C"] {
            for v in [10.0, 20.0, 30.0] {
                for m in [300.0, 440.0, 580.0] {
                    for c in presets {
                        for d in [0.0, 5.0, 20.0] {
                            for seed in seeds.clone() {
                                jobs.push((split, spec(rc, v, m, c, d, seed)));
                            }
                        }
                    }
                }
            }
        }
    }
    for c in presets {
        for seed in 100..105u64 {
            jobs.push(("ood", spec("D", 40.0, 700.0, c, 30.0, seed)));
        }
    }

    std::fs::create_dir_all(out)?;

    // par_iter + collect keeps the order of `jobs`, so row order is deterministic.
    // Timeseries are written inside the closure so full results are never all held in memory.
    let (rows, stats): (Vec<DatasetRow>, Vec<RunStats>) = jobs
        .par_iter()
        .map(|(split, s)| -> Result<(DatasetRow, RunStats)> {
            let (res, mut row, st) = run_scenario_diag(s)?;
            row.split = split.to_string();
            if s.seed == 0 && s.speed_mps == 20.0 && s.ms_kg == 440.0 {
                write_timeseries(&res, out)?; // 3 roads x 3 presets x 3 dropouts = 27 files
            }
            Ok((row, st))
        })
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .unzip();

    let mut wtr = csv::Writer::from_path(out.join("comfort_dataset.csv"))?;
    for r in &rows {
        wtr.serialize(r)?; // header comes from DatasetRow field names (contract order)
    }
    wtr.flush()?;

    write_run_meta(
        &VehicleConfig::nominal(),
        &SensorConfig::defaults(),
        &PerturbationRanges::default(),
        &json!({"q": {}, "r": {}, "gate_chi2_1dof": 6.63}),
        out,
    )?;

    let injected: Vec<(String, f64)> = SensorConfig::defaults()
        .into_iter()
        .filter(|s| ["body_acc", "wheel_acc", "susp_travel"].contains(&s.channel.as_str()))
        .map(|s| (s.channel, s.delay_ms))
        .collect();
    write_fusion_summary(
        &build_fusion_summary(&stats, &injected, 20.0, &[0.0, 5.0, 20.0]),
        out,
    )?;

    for s in ["train", "test", "ood"] {
        println!("{s}: {}", rows.iter().filter(|r| r.split == s).count());
    }
    println!("total: {}", rows.len());
    Ok(())
}

fn main() -> Result<()> {
    match Cli::parse().cmd {
        Cmd::Matrix { out } => run_matrix(&out),
    }
}