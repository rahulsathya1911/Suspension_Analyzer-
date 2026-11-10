# Suspension Analyzer

**AI-enabled suspension digital twin with sensor fusion for ride comfort prediction.**

A proof of concept that links four things in one pipeline: simulated vehicle sensors, an Extended Kalman Filter (EKF) that estimates suspension and road states, a quarter-car digital twin, and a machine-learning model that predicts an ISO 2631-1 ride-comfort score with a 90% prediction interval. The model also reports whether a condition is inside the range it was validated on, and a semi-active mode compares preset damping settings and recommends one only when the evidence is clear.

> **Scope:** everything here runs on **simulated data**. There is no real-vehicle validation yet. See [Known limitations](#known-limitations) before relying on any number.

[![Rust](https://img.shields.io/badge/rust-%23000000.svg?style=for-the-badge&logo=rust&logoColor=white)](#)
[![Python](https://img.shields.io/badge/python-3776AB?style=for-the-badge&logo=python&logoColor=white)](#)
[![License: Academic](https://img.shields.io/badge/License-Academic-blue.svg?style=for-the-badge)](#license)

## What it does

**1. Suspension analysis tools** (the original platform)
- Time-domain quarter-car simulation (RK4) against step, sine and ISO 8608 random roads
- ISO 2631-1 weighted ride comfort, analytical frequency-response sweep, speed sweep
- Spring-rate vs damping parameter sweep with Pareto front
- Vehicle and motorcycle parameter helpers, with simulation history stored in PostgreSQL

**2. Fusion and comfort pipeline** (the digital-twin extension)
- Synchronises five simulated sensors (body acceleration, wheel acceleration, suspension travel, wheel speed, vehicle speed) onto a 200 Hz clock, with noise, bias, delay, dropout and spike or stuck-at faults
- 5-state EKF (sprung and unsprung displacement and velocity, plus road height) with innovation gating; dropped readings skip the update step
- Quarter-car twin driven by the fused road estimate to give a physics comfort value (`p_phys`)
- Residual ML model (quantile gradient boosting plus conformal widening) predicting comfort with a 90% interval
- Supported-range check: flags inputs outside the training range, far from it (Mahalanobis distance), or from an unseen road class
- Semi-active ranking of soft / medium / firm damping, recommended only when the two best 90% intervals do not overlap and the tyre-load and travel limits hold
- Dashboard "Fusion Mode" showing raw signals, fused states, simulated response, predicted comfort, confidence and validation results

## Architecture

```mermaid
graph TD
    CLI["fusion_cli matrix (Rust)"] --> DS["comfort_dataset.csv + timeseries/*.json"]
    CLI --> SUM["fusion_summary.json + run_meta.json"]
    DS --> ML["ml/ml/train.py (Python)"]
    ML --> RES["comfort_predictions / envelope / semiactive_ranking .json"]
    UI["frontend/Index.html"] -->|HTTP JSON| API["Rocket backend :8000"]
    RES --> API
    SUM --> API
    API --> CORE["core: simulate / FRF / sweep"]
    API -->|"/fusion/run"| FUS["fusion crate: sensors, sync, EKF, twin"]
    API --> DB[("PostgreSQL: simulation history")]
```

The Rust CLI runs the scenario matrix and writes the dataset. The Python script trains on it and writes the result files. The backend serves those files (`FUSION_OUT_DIR`, default `out/`) and can also run one fusion scenario live. The dashboard is a static page that calls the backend at `http://localhost:8000`.

## Project structure

```text
Suspension_Analyzer-/
├── core/                 # Quarter-car physics: simulate, FRF, sweep (Rust)
├── fusion/               # Sensors, sync, EKF, twin, comfort metric (Rust)
│   └── src/bin/fusion_cli.rs   # `fusion_cli matrix` builds the whole dataset
├── ml/ml/                # Comfort model, envelope, semi-active ranking (Python)
│   ├── train.py          # Entry point
│   └── tests/
├── backend/              # Rocket API (+ fusion routes in src/fusion.rs)
├── frontend/             # Index.html (dashboard), fusion_mode.html, landing, login
├── contracts/fixtures/   # Small sample outputs for the dashboard (shown with a SAMPLE DATA badge)
├── out/                  # Generated dataset, predictions, envelope, plots, time series
├── check_runs.py         # Prints per-scenario filter diagnostics from a running backend
├── compose.yml           # PostgreSQL for simulation history
└── Cargo.toml            # Cargo workspace (backend, core, frontend, fusion)
```

## Getting started

### Prerequisites

- [Rust](https://rustup.rs/) (stable)
- Python 3.11+ with `pip install -r ml/ml/requirements.txt`
- [Docker](https://www.docker.com/) (for PostgreSQL)

### 1. Start PostgreSQL

The backend **exits at startup** if it cannot connect to PostgreSQL, so start the database first. Choose a password and keep it out of git.

```bash
export POSTGRES_PASSWORD='choose-a-password'
docker compose up -d
```

Create the history table once (columns inferred from the backend's `INSERT` and `/history` query):

```sql
CREATE TABLE IF NOT EXISTS simulations (
  id SERIAL PRIMARY KEY,
  ms DOUBLE PRECISION, mu DOUBLE PRECISION, k DOUBLE PRECISION,
  c DOUBLE PRECISION, kt DOUBLE PRECISION, road_profile TEXT,
  zeta DOUBLE PRECISION, mass_ratio DOUBLE PRECISION, recommended_mu DOUBLE PRECISION,
  rms_body_acc DOUBLE PRECISION, rms_tire_force DOUBLE PRECISION,
  max_suspension_travel DOUBLE PRECISION, iso2631_weighted_rms DOUBLE PRECISION
);
```

### 2. Run the backend (from the repository root)

```bash
export DATABASE_URL="postgres://postgres:${POSTGRES_PASSWORD}@localhost:5432/postgres"
cargo run --release -p backend
```

The API listens on port 8000. Run it from the repository root so the default `FUSION_OUT_DIR=out/` resolves.

### 3. Open the dashboard

Open `frontend/Index.html` in a browser and choose **Fusion Mode**. It reads the generated files through the backend.

### Rebuild the dataset and models

```bash
# 1. Simulate the full matrix (3,660 runs; about 8 minutes on 2 CPU cores)
cargo run --release -p suspension_fusion --bin fusion_cli -- matrix --out out/

# 2. Train the comfort model, envelope and damping ranking (a few seconds)
cd ml/ml
python train.py --in ../../out/comfort_dataset.csv --out ../../out
```

Run `train.py` from inside `ml/ml/`; it imports its sibling modules directly. Add `--mock` to mark outputs as generated from mock data.

### Tests

```bash
cargo test --workspace      # 51 Rust tests
```

The Python tests in `ml/ml/tests/` currently look for data under `ml/out/` instead of `out/`, so they fail until that path is fixed (see [Known limitations](#known-limitations)).

## Configuration

| Variable | Required | Description |
| --- | --- | --- |
| `DATABASE_URL` | yes | PostgreSQL connection string. No default; the backend exits without it. |
| `POSTGRES_PASSWORD` | for `compose.yml` | Password for the database container. Never commit it. |
| `FUSION_OUT_DIR` | no | Folder the `/fusion/*` routes read from. Default `out/`. |

## API

| Method | Route | Purpose |
| --- | --- | --- |
| POST | `/simulate` | Run one quarter-car simulation and store it |
| GET | `/history` | Past simulations |
| POST | `/frf` | Frequency-response function |
| POST | `/sweep` | Parameter sweep with Pareto front |
| POST | `/speed_sweep`, `/frf_compare` | Speed sweep, FRF comparison |
| POST | `/vehicle/params`, `/moto/params` | Derive suspension parameters from a vehicle or motorcycle config |
| GET | `/validation-schema` | Allowed parameter ranges |
| POST | `/fusion/run` | Run one fusion scenario live |
| GET | `/fusion/scenarios` | List available scenario ids |
| GET | `/fusion/data/<name>` | `fusion_summary`, `comfort_predictions`, `envelope`, `semiactive_ranking`, `run_meta` |
| GET | `/fusion/data/timeseries/<id>` | Time series for one scenario |

## Results (simulated data)

Scenario matrix: road classes A to C, 10 / 20 / 30 m/s, 300 / 440 / 580 kg, damping 1000 / 1800 / 3000 N·s/m, dropout 0 / 5 / 20%, 15 seeds, plus an out-of-distribution set (road D, 40 m/s, 700 kg, 30% dropout). That gives 3,660 runs: 2,430 train, 1,215 test (new seeds, same grid) and 15 out-of-distribution.

| Check | Result |
| --- | --- |
| Comfort MAE on test (m/s²): linear baseline / ML | 0.047 / 0.033 |
| 90% interval coverage on test | 89.8% |
| Out-of-distribution runs flagged | 100% (false flags: 1.2% train, 2.0% test) |
| Semi-active: soft recommended / no clear winner (81 groups) | 17 / 64 |
| Suspension-travel RMSE: raw sensor / fused | 1.5 mm / 8.2 mm |

## Known limitations

- **Simulated data only.** No real-vehicle or measured-data validation yet.
- **Sensor fusion is not validated.** The filter uses three of the five channels (body acceleration, wheel acceleration, suspension travel); wheel and vehicle speed are simulated but not used. Fused suspension-travel error is larger than the raw sensor's. The velocity and road-height estimates are poor, and the filter's uncertainty is too narrow (truth lies within its ±2σ band about 34% of the time for the velocity states). The innovation gate rejects most samples and needs retuning.
- **The twin adds no measured accuracy yet.** In an ablation, a model on scenario inputs alone is as accurate as one that also uses the twin output `p_phys`. Within one road class `p_phys` tracks comfort, but its level is far too low.
- **The test set measures interpolation.** Test runs use new seeds on the same scenario grid. The out-of-distribution check uses a single configuration.
- **Quarter-car only.** 13 of 3,660 runs exceed the 120 mm travel limit; how to handle this is still undecided.
- **Python tests** resolve the data folder one level too deep (`ml/out/` instead of `out/`).
- **Standards:** the comfort weighting follows ISO 2631-1 and the road profiles follow ISO 8608. Any SAE references in the vehicle and motorcycle parameter helpers should be re-verified before you cite them.

## Roadmap

- [ ] Retune the filter gate and noise settings, then rerun the matrix
- [ ] Use wheel speed and vehicle speed in the filter
- [ ] Fix the road-estimate level so the twin output tracks comfort across road classes
- [ ] Decide how to handle suspension travel beyond the limit
- [ ] Test against public measured vehicle data
- [ ] Half-car (4-DOF) pitch model

## License

This project is intended for academic and research purposes.
