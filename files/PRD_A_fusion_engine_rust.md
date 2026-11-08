# PRD A: Fusion Engine (Rust)

**Owner:** Person A. **Folder you own:** `fusion/` (new crate). You may add thin `pub` wrappers in `core/` without changing existing signatures.
**Read first:** `docs/CONTRACTS.md` (shared interfaces, file schemas, Rust API, timeline). That file is the source of truth for everything other people consume from you.
**Project:** Techgium 10 POC, *AI-Enabled Suspension Digital Twin with Sensor Fusion for Ride Comfort Prediction*. Simulated data only.

---

## 0. Instructions for the coding agent

1. Read this file and `docs/CONTRACTS.md` fully. Then audit `core/` (task A0) before writing code.
2. **Do not duplicate `core/`.** Reuse its ISO 8608 road generator, solver and ISO 2631 code where they are `pub`. If something is private or does not return a time series, add a thin new `pub` wrapper, do not copy the code. Do not change existing function signatures.
3. Work task by task. After each: `cargo fmt`, `cargo clippy -- -D warnings`, `cargo test`. Small commits.
4. All randomness uses `rand_chacha::ChaCha8Rng` seeded from the scenario seed. Same seed gives byte-identical output.
5. **Never invent result numbers.** Every number in outputs comes from computation.
6. Output file formats are fixed by `docs/CONTRACTS.md`. If you must change one, stop and tell the other owners.
7. If this PRD conflicts with the repo, stop and report the conflict.

## 1. Goal

Build the signal chain: a true (perturbed) quarter-car plant produces data; simulated sensors corrupt it; a synchronisation layer aligns it; a Kalman filter fuses it into road and suspension states with uncertainty; a physics comfort value is computed from the fused estimates. Export datasets and time series for the ML and dashboard people.

## 2. Deliverables (what others get from you)

| Output | Consumer | Contract |
|---|---|---|
| Stub `run_scenario` (first hour), real one later | Person C | Section 5 |
| `comfort_dataset.csv` | Person B | 4.1 |
| `fusion_summary.json` | Person C | 4.2 |
| `timeseries/<id>.json` | Person C | 4.3 |
| `run_meta.json` | B and C | 4.7 |
| Fixtures in `contracts/fixtures/` | B and C | Section 7 |

## 3. Scope

**In:** quarter-car (2-DOF) only; sensor simulation (noise, bias, drift, delay, dropout, anomalies); synchronisation and delay estimation; Kalman fusion with road as augmented state; innovation gating; physics ISO 2631-1 comfort; metrics; CLI; scenario matrix.
**Out:** ML, envelope, semi-active ranking (Person B); HTTP routes and UI (Person C); half-car model; real vehicle data.

## 4. Crate layout

```
fusion/
  Cargo.toml            # add "fusion" to workspace members
  configs/matrix.toml   # scenario matrix (section 6.9)
  configs/vehicle.toml  # nominal vehicle, travel_limit_mm, sensor specs
  src/
    lib.rs              # ScenarioSpec, ScenarioResult, run_scenario
    config.rs
    plant.rs
    sensors.rs
    sync.rs
    kf.rs
    comfort.rs
    metrics.rs
    export.rs
    bin/fusion_cli.rs   # run | matrix | report
  tests/
```
Dependencies (keep short): `suspension_core` (path), `nalgebra`, `rand`, `rand_chacha`, `rand_distr`, `serde`, `serde_json`, `csv`, `clap`, `anyhow`, optional `rayon`; dev: `approx`.

## 5. Technical specification

### 5.1 Road
Reuse the `core/` ISO 8608 generator, seeded. Needs `(t, zr)` for class, speed, duration, sample rate, seed. Roughness `Gd(n0)` at `n0 = 0.1 cycles/m`: A `16e-6`, B `64e-6`, C `256e-6`, D `1024e-6` m^3.

### 5.2 Plant (`plant.rs`)
Nominal equations:
```
ms*zs'' = -k*(zs-zu) - c*(vs-vu)
mu*zu'' =  k*(zs-zu) + c*(vs-vu) - kt*(zu-zr)
```
State `[zs, vs, zu, vu]`, input `zr`, 1 kHz.
- **Nominal plant:** linear, nominal parameters. Reuse the `core/` solver if it exposes a time series, otherwise RK4 here.
- **Perturbed plant (own RK4):** spring rate +/-10%, tyre stiffness +/-5%, sprung-mass variation, asymmetric damper (rebound = 2x compression) with blow-off above a velocity threshold. Parameters drawn per run from the seeded RNG, written to `run_meta.json` only. **The filter never receives them.** This is what makes the ML residual non-trivial and the setup honest.
- **Parity test:** nominal plant vs `core/` on one fixed scenario, RMS body acceleration and RMS tyre force within 1%.

### 5.3 Sensors (`sensors.rs`)

| Channel | Rate | Noise sigma | Bias | Delay |
|---|---|---|---|---|
| Body vertical acceleration | 200 Hz | 0.05 m/s^2 | 0.02 m/s^2 + slow drift | 5 ms |
| Wheel (hub) acceleration | 200 Hz | 0.5 m/s^2 | 0.1 m/s^2 | 5 ms |
| Suspension travel | 100 Hz | 0.5 mm | 0.5 mm | 10 ms |
| Wheel speed | 50 Hz | 0.05 m/s | none | 20 ms |
| Vehicle speed | 20 Hz | 0.1 m/s | none | 50 ms |

These are assumptions typical of MEMS and automotive sensors. Everything is configurable in `configs/vehicle.toml` and echoed to `run_meta.json`.
Corruptions: Gaussian noise, constant bias, random-walk drift, per-channel delay (ring buffer), dropout bursts 0.1-0.5 s (total missing from `dropout_pct`), spike anomalies (about 10 sigma, 0.2% of samples), stuck-at faults (0.5 s). Also output a **ground-truth fault log** (what was injected, when) for scoring.

### 5.4 Synchronisation (`sync.rs`)
- One fusion clock at 200 Hz. Per-sample validity flag per channel. Mark gaps and stuck-at windows (near-zero variance in a window).
- **Delay estimation:** channels measure different quantities, so plain cross-correlation does not work. Use a model-based search: run the filter on a clean segment for each candidate delay (grid of +/-N samples per channel) and pick the delay that minimises the mean normalised innovation squared. Report estimated vs injected delay.

### 5.5 Kalman fusion (`kf.rs`)
- State `x = [zs, vs, zu, vu, zr]`. `nalgebra::SMatrix<f64, 5, 5>`.
- Discretise with a matrix exponential (scaling and squaring, or Taylor order >= 8).
- Road model: start with a random walk on `zr`. If drift appears, upgrade to a first-order Gauss-Markov process. Document the choice and tuning values.
- Measurements (linear in the state):
  - body acceleration = `(-k(zs-zu) - c(vs-vu)) / ms`
  - wheel acceleration = `(k(zs-zu) + c(vs-vu) - kt(zu-zr)) / mu`
  - suspension travel = `zs - zu`
  - wheel and vehicle speed: set the road model's speed and run plausibility checks (slip).
- **Sequential scalar updates in Joseph form.** Handles asynchronous multi-rate data and gating without matrix inverses.
- **Gating:** per-measurement normalised innovation squared against the 99% chi-square threshold for 1 degree of freedom (6.63). A rejected measurement is treated as dropped and logged.
- During dropout: predict only, covariance grows.
- The filter uses **nominal parameters only**.
- Output: fused states, covariance diagonal and trace, innovation log, gate log.

### 5.6 Comfort (`comfort.rs`)
- ISO 2631-1 `Wk` weighting as a biquad cascade (bilinear transform). Weighted RMS over the run.
- **Use one weighting function for both `p_phys` and `y_ref`.** Then the residual measures plant mismatch, not a weighting difference.
- `p_phys`: weighted RMS of body acceleration from the **nominal twin driven by the fused road estimate** `zr_hat(t)`.
- `y_ref`: weighted RMS of body acceleration from the **perturbed plant on the true road**.
- Cross-check against the `core/` ISO value on one scenario and report the difference (the repo notes `core/` uses an approximation).
- Band RMS of fused body acceleration: 0.5-2, 2-8, 8-20 Hz (`band_lo`, `band_mid`, `band_hi`).
- Assumption to keep visible: body acceleration is the comfort proxy, seat transmissibility is not modelled.

### 5.7 Metrics (`metrics.rs`)
RMSE per state vs truth for: raw or naive baseline (where a sensor measures the state directly, otherwise null), open-loop nominal twin, fused. Fraction of truth inside +/-2 sigma and NEES. Anomaly and dropout detection precision and recall vs the fault log. Delay-estimation error.

### 5.8 Library API and CLI
- `run_scenario(&ScenarioSpec) -> Result<ScenarioResult>` exactly as in CONTRACTS section 5. **Push the stub in the first hour.**
- `fusion-cli run --scenario <json> --out out/` writes one `timeseries/<id>.json`.
- `fusion-cli matrix --config fusion/configs/matrix.toml --out out/` runs the whole matrix, writes `comfort_dataset.csv`, `fusion_summary.json`, `run_meta.json`, and `timeseries/` for scenarios with seed 0, speed 20, load 440 (27 files) plus `rD_v40_m700_c1800_d20_s0`. Round time-series values to 5 significant digits.
- `fusion-cli report --in out/` prints the RMSE table.

### 5.9 Scenario matrix (`configs/matrix.toml`)
Road class {A, B, C} x speed {10, 20, 30} m/s x load {300, 440, 580} kg x preset {soft 1000, medium 1800, firm 3000} x dropout {0, 5, 20}%, seeds 0-9 (`train`) and 100-104 (`test`). Duration 20 s. Out-of-distribution (`ood`): class D, speed 40, load 700, dropout above 20 (for example 30), seeds 100-104. Use `rayon` for speed if it is easy.

## 6. Tasks (in order)

| Task | What | Done when |
|---|---|---|
| **A0** | Audit `core/`: which functions are `pub`, signatures, units, whether the solver returns a time series, `dt`, how ISO 2631 is implemented, Rust edition and deps. Write `docs/ARCH_NOTES.md`. | Every question answered; reusable functions listed |
| **A1** | **Skeleton and stub, push to `main` within the first hour.** Crate in workspace, `ScenarioSpec`, `ScenarioResult`, stub `run_scenario` returning schema-valid data with `fixture: true`. | Workspace builds; `cargo test` passes |
| **A2** | Fixtures into `contracts/fixtures/` (section 7 of CONTRACTS). | Files exist, schema-valid, `fixture: true` |
| **A3** | Plant (nominal and perturbed) and parity test. | Parity within 1%; analytical step overshoot matches theory in a test |
| **A4** | Sensors and fault log. | Statistics of noise and bias match config; faults logged |
| **A5** | Synchronisation and delay search. | Injected delay recovered within one sample; dropout and stuck flags match the fault log |
| **A6** | Kalman filter and gating. | Fused travel RMSE below raw RMSE at nominal noise; a deliberate spike is rejected; covariance grows in a dropout |
| **A7** | Comfort (`Wk`, `p_phys`, `y_ref`, bands) and metrics. | `Wk` gain at 4-8 Hz near 1 and rolls off above 8 Hz (test); cross-check with `core/` reported |
| **A8** | Real `run_scenario`, CLI `run`, `report`. | `fixture: false`; schema-valid |
| **A9** | CLI `matrix`; all output files. | Files in CONTRACTS 4.1, 4.2, 4.3, 4.7 exist; matrix finishes in reasonable time |
| **A10** | Determinism test (same seed, same bytes) and final run on the integration freeze (Sun 4 PM). | Tests pass; `out/` complete |

If time runs out, finish A0-A6 and A9 with a reduced matrix (smaller seed set) first. Tell the others which outputs are reduced.

## 7. Acceptance criteria and metrics to report

Targets are goals to measure, not claims to make in advance.

| Metric | Target |
|---|---|
| Fused vs raw RMSE on suspension travel | Fused lower |
| Fused vs open-loop twin RMSE (body velocity, wheel velocity, road height) | Fused lower |
| Fraction of truth within +/-2 sigma | About 90-98% |
| RMSE at dropout 0, 5, 20% | Degrades gracefully, reported in `dropout_sweep` |
| Anomaly detection | Precision and recall reported |
| Delay estimation | Within one sample |
| Nominal plant parity with `core/` | Within 1% |
| Determinism | Byte-identical on rerun |

## 8. Risks

| Risk | Mitigation |
|---|---|
| `core/` hides the time series | Add thin `pub` wrapper, do not copy code |
| Filter tuning takes long | Fix `Q`, `R` from sensor specs first, tune one parameter at a time |
| Matrix run too slow | `rayon`; reduce seeds first, never reduce the schema |
| Others blocked | Stub in hour 1, fixtures in hour 2 |
