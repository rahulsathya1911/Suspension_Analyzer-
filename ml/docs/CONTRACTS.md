# CONTRACTS: shared interfaces for the three-person split

Commit this file to the repo as `docs/CONTRACTS.md` **before anyone starts coding**. All three PRDs depend on it. If you need to change a contract, tell the other two owners and update this file in the same commit.

**Project:** Sensor-fusion and comfort-prediction layer for Adaptive Suspension AI (Techgium 10 POC). Simulated data only.

---

## 1. Who owns what

| PRD | Role | Owns (only edit here) | Language |
|---|---|---|---|
| **A** | Fusion engine | `fusion/` (new crate). May add thin `pub` wrappers in `core/` without changing existing signatures. | Rust |
| **B** | ML, envelope, semi-active | `ml/` | Python |
| **C** | Backend routes, dashboard, sag fix | `backend/` (new routes only), `frontend/` (or the active UI folder), `docs/` | Rust + JS |

Shared and generated:
- `contracts/fixtures/`: sample files in the exact shapes below. Each owner writes the fixtures for the files they produce (see "Fixtures").
- `out/`: generated outputs. Git-ignored except `contracts/fixtures/`.

**Git:** branches `feat/fusion-engine`, `feat/ml`, `feat/dashboard`. Merge order: A skeleton first (within the first hour), then whoever finishes next. Nobody force-pushes `main`.

**First hour (blocking):** Person A pushes a skeleton `fusion/` crate to `main` with a **stub** `run_scenario` (section 5). Person C's backend depends on this crate by path, and Cargo fails if the folder is missing. B and C branch from `main` after that push.

## 2. Conventions

- **Units:** SI in all files. Displacement in metres, velocity m/s, acceleration m/s², force N, time s, speed m/s. The dashboard converts to mm or km/h for display.
- **Seeds:** every random process takes a seed. Same seed must give byte-identical output.
- **Sample data flag:** every JSON file has `meta.fixture` (bool). Fixtures and mock data have `true`. Real outputs have `false`. Python mock data also sets `meta.mock: true`. **The dashboard shows a visible "SAMPLE DATA" badge whenever either is true**, so mock numbers can never reach the slides.
- **No invented numbers:** every number shown to users must come from a generated file, never from a constant in code.
- **Nominal vehicle:** `ms = 440 kg`, `mu = 78 kg`, `k = 21000 N/m`, `kt = 200000 N/m`. Damping presets: soft `1000`, medium `1800`, firm `3000` N·s/m.
- **Travel limit:** `vehicle.travel_limit_mm` comes from `run_meta.json`. Default `120` until the team lead decides the sag fix (see PRD C, task C1). Nobody hard-codes it.

## 3. Scenario IDs

Format: `r{class}_v{speed}_m{load}_c{damping}_d{dropout}_s{seed}`
Example: `rB_v20_m440_c1800_d5_s0` (road class B, 20 m/s, 440 kg, c = 1800, 5% dropout, seed 0).
All numbers are integers in the ID. Group ID (used by semi-active ranking) drops damping and seed: `rB_v20_m440_d5`.

## 4. File contracts

### 4.1 `out/comfort_dataset.csv` (A writes, B reads)

One row per run. Header exactly:

```
scenario_id,split,road_class,speed_mps,ms_kg,preset,c_nsm,dropout_pct,seed,p_phys,y_ref,cov_trace,dropout_frac,band_lo,band_mid,band_hi,rms_tyre_force_n,max_travel_mm,bottom_out
```

| Column | Type | Meaning |
|---|---|---|
| `scenario_id` | string | Section 3 |
| `split` | `train` \| `test` \| `ood` | Train seeds 0-9, test seeds 100-104, out-of-distribution sets |
| `road_class` | `A`..`D` | ISO 8608 class |
| `speed_mps`, `ms_kg`, `c_nsm`, `dropout_pct`, `seed` | number | Scenario inputs |
| `preset` | `soft` \| `medium` \| `firm` | Damping preset (the OOD set may use `medium`) |
| `p_phys` | float, m/s² | ISO 2631-1 weighted RMS from the **nominal twin driven by the fused road estimate** |
| `y_ref` | float, m/s² | Same weighting, from the **perturbed plant on the true road** (stand-in for a measured value) |
| `cov_trace` | float | Mean trace of the filter covariance over the run |
| `dropout_frac` | float 0-1 | Fraction of fusion steps where at least one channel was invalid or rejected |
| `band_lo`, `band_mid`, `band_hi` | float, m/s² | RMS of fused body acceleration in 0.5-2, 2-8, 8-20 Hz |
| `rms_tyre_force_n` | float, N | RMS dynamic tyre force (perturbed plant) |
| `max_travel_mm` | float | Max suspension travel (perturbed plant) |
| `bottom_out` | 0 \| 1 | `max_travel_mm > travel_limit_mm` |

Out-of-distribution rows: road class `D`, speed `40`, load `700`, dropout above `20`. Marked `split = ood`.

### 4.2 `out/fusion_summary.json` (A writes, C reads)

```json
{
  "meta": {"fixture": false, "n_runs": 0, "seeds": [0], "duration_s": 20.0},
  "state_rmse": {
    "susp_travel_m":  {"raw": {"mean": 0, "std": 0}, "open_loop": {"mean": 0, "std": 0}, "fused": {"mean": 0, "std": 0}},
    "body_vel_mps":   {"raw": null, "open_loop": {"mean": 0, "std": 0}, "fused": {"mean": 0, "std": 0}},
    "wheel_vel_mps":  {"raw": null, "open_loop": {"mean": 0, "std": 0}, "fused": {"mean": 0, "std": 0}},
    "road_height_m":  {"raw": null, "open_loop": {"mean": 0, "std": 0}, "fused": {"mean": 0, "std": 0}}
  },
  "consistency": {"within_2sigma_frac": {"susp_travel_m": 0, "body_vel_mps": 0, "wheel_vel_mps": 0, "road_height_m": 0}, "nees_mean": 0},
  "dropout_sweep": [{"dropout_pct": 0, "fused_rmse": {"susp_travel_m": 0, "body_vel_mps": 0, "wheel_vel_mps": 0, "road_height_m": 0}}],
  "detection": {"anomaly": {"precision": 0, "recall": 0}, "dropout": {"precision": 0, "recall": 0}},
  "delay_estimation": [{"channel": "susp_travel", "injected_ms": 0, "estimated_ms": 0}]
}
```
`raw` is `null` where no raw sensor measures that state directly. `mean` and `std` are over seeds and scenarios.

### 4.3 `out/timeseries/<scenario_id>.json` (A writes, C reads) and the `POST /fusion/run` response

Same shape. Arrays all have the same length as `t` (100 Hz). Missing raw samples are `null`.

```json
{
  "fixture": false,
  "scenario": {"road_class": "B", "speed_mps": 20.0, "ms_kg": 440.0, "c_nsm": 1800.0, "preset": "medium", "dropout_pct": 5.0, "seed": 0, "duration_s": 20.0},
  "fs_hz": 100,
  "t": [],
  "truth": {"susp_travel_m": [], "body_vel_mps": [], "wheel_vel_mps": [], "road_height_m": [], "body_acc_mps2": []},
  "raw":   {"body_acc_mps2": [], "wheel_acc_mps2": [], "susp_travel_m": []},
  "fused": {"susp_travel_m": [], "body_vel_mps": [], "wheel_vel_mps": [], "road_height_m": []},
  "sigma": {"susp_travel_m": [], "body_vel_mps": [], "wheel_vel_mps": [], "road_height_m": []},
  "flags": {"dropout": [], "anomaly_rejected": []},
  "metrics": {"rmse_fused": {"susp_travel_m": 0, "body_vel_mps": 0, "wheel_vel_mps": 0, "road_height_m": 0},
              "rmse_raw":   {"susp_travel_m": 0},
              "p_phys": 0, "y_ref": 0, "dropout_frac": 0}
}
```
`flags.*` are 0/1 per sample. `sigma` is the 1-sigma of each fused state.

### 4.4 `out/comfort_predictions.json` (B writes, C reads)

```json
{
  "meta": {"fixture": false, "mock": false, "model": "gbr_quantile+cqr", "alpha": 0.1,
           "metrics": {"mae_physics_only": 0, "mae_linear": 0, "mae_ml": 0, "interval_coverage": 0, "n_test": 0}},
  "rows": [
    {"scenario_id": "rB_v20_m440_c1800_d5_s100", "split": "test", "p_phys": 0, "comfort_pred": 0,
     "lo": 0, "hi": 0, "y_ref": 0, "residual": 0, "supported": true}
  ]
}
```
`residual = y_ref - comfort_pred`. `lo`/`hi` are the 90% interval on the final comfort value.

### 4.5 `out/envelope.json` (B writes, C reads)

```json
{
  "meta": {"fixture": false, "mock": false},
  "features": [{"name": "speed_mps", "min": 0, "max": 0}],
  "mahalanobis_threshold": 0,
  "ood_detection_rate": 0,
  "ood_results": [{"scenario_id": "rD_v40_m700_c1800_d20_s100", "flagged": true, "reason": "range:speed_mps"}]
}
```
`reason` is `range:<feature>` or `mahalanobis` or `none`.

### 4.6 `out/semiactive_ranking.json` (B writes, C reads)

```json
{
  "meta": {"fixture": false, "mock": false, "dlc_limit": 0.33, "travel_limit_mm": 120},
  "groups": [
    {"group_id": "rB_v20_m440_d5",
     "options": [{"preset": "soft", "c_nsm": 1000, "comfort_pred": 0, "lo": 0, "hi": 0, "feasible": true, "violations": []}],
     "recommended": "medium",
     "note": ""}
  ]
}
```
`violations` entries: `tyre_load`, `travel`, `bottom_out`. `recommended` is `null` and `note` is `"no clear winner"` when the intervals of the best two feasible options overlap, or `"no feasible option"` when none is feasible. DLC = `rms_tyre_force_n / ((ms_kg + mu_kg) * 9.81)`.

### 4.7 `out/run_meta.json` (A writes, B and C read)

```json
{
  "fixture": false,
  "git_commit": "",
  "vehicle": {"ms_kg": 440, "mu_kg": 78, "k_nm": 21000, "kt_nm": 200000, "travel_limit_mm": 120, "presets": {"soft": 1000, "medium": 1800, "firm": 3000}},
  "sensors": [{"channel": "body_acc", "rate_hz": 200, "sigma": 0.05, "bias": 0.02, "delay_ms": 5}],
  "filter": {"q": {}, "r": {}, "gate_chi2_1dof": 6.63},
  "perturbation_ranges": {"k_pct": 10, "kt_pct": 5, "asym_rebound_ratio": 2.0},
  "seeds": {"train": [0, 9], "test": [100, 104]}
}
```

## 5. Rust API (A implements, C calls)

Crate `suspension_fusion` (folder `fusion/`), in the workspace. Public items:

```rust
#[derive(serde::Deserialize, serde::Serialize, Clone, Debug)]
pub struct ScenarioSpec {
    pub road_class: String,      // "A" | "B" | "C" | "D"
    pub speed_mps: f64,
    pub ms_kg: f64,
    pub c_nsm: f64,
    pub dropout_pct: f64,        // 0..50
    pub seed: u64,
    pub duration_s: f64,         // 5..60
    #[serde(default)] pub k_nm: Option<f64>,
    #[serde(default)] pub mu_kg: Option<f64>,
    #[serde(default)] pub kt_nm: Option<f64>,
}

#[derive(serde::Serialize)]
pub struct ScenarioResult { /* serialises to the JSON in section 4.3, including "fixture" */ }

pub fn run_scenario(spec: &ScenarioSpec) -> anyhow::Result<ScenarioResult>;
```
The **stub** (first hour) returns a short, schema-valid result with `fixture: true`. The real version sets `fixture: false`.

## 6. HTTP API (C implements)

Existing routes `/simulate`, `/frf`, `/sweep` stay unchanged.

| Route | Behaviour |
|---|---|
| `POST /fusion/run` | Body is a `ScenarioSpec` as JSON. Returns the 4.3 JSON. `422` with `{"error": "..."}` on invalid input (road class not A-D, speed <= 0, `ms_kg` outside 100-1500, dropout outside 0-50, duration outside 5-60). `500` on internal error. |
| `GET /fusion/data/<name>` | `name` is one of `fusion_summary`, `comfort_predictions`, `envelope`, `semiactive_ranking`, `run_meta`. Reads `<FUSION_OUT_DIR>/<name>.json`, default `out/`. `404` with `{"error": "not_found"}` if absent. |
| `GET /fusion/data/timeseries/<scenario_id>` | Reads `<FUSION_OUT_DIR>/timeseries/<scenario_id>.json`. Scenario ID is validated against the section 3 pattern, no path traversal. |
| `GET /fusion/scenarios` | Returns `{"timeseries": ["<id>", ...]}` from the files present. |

## 7. Fixtures

Each owner writes **schema-valid** fixture files into `contracts/fixtures/` within their first two hours, with `fixture: true` (A: `comfort_dataset.csv` with about 20 rows, `fusion_summary.json`, one `timeseries/<id>.json`, `run_meta.json`; B: `comfort_predictions.json`, `envelope.json`, `semiactive_ranking.json`). Values can be fake. Shapes cannot. C builds the UI against fixtures. If a fixture is not there after two hours, C writes it from this file.

## 8. Timeline (today is Sat 7 Nov, deadline Mon 9 Nov)

| When | A | B | C |
|---|---|---|---|
| Hour 0-1 | Skeleton and stub pushed to `main` | Environment, mock dataset generator | Audit, read contracts |
| Hour 1-2 | Fixtures pushed | Fixtures pushed | UI skeleton on fixtures |
| Sat night | Plant, sensors, sync | Residual model on mock data | Routes, Signals sub-tab |
| Sun morning | Kalman filter, comfort | Envelope, semi-active | Comfort sub-tab |
| Sun afternoon | Metrics, matrix run, real outputs | Re-run on real CSV | Validation sub-tab, real data |
| **Sun 4 PM: integration freeze** | Outputs in `out/` | Outputs in `out/` | Point at real `out/` |
| Sun evening | Fix bugs found in testing | Fix bugs | Fix bugs, take screenshots |
| Mon morning | Final numbers | | Final screenshots, slides |

## 9. Integration test (run by the tester at the end)

1. **Build:** `cargo build --workspace` and `cargo test --workspace` pass. `pip install -r ml/requirements.txt` works.
2. **A:** `cargo run -p suspension_fusion --bin fusion_cli -- matrix --config fusion/configs/matrix.toml --out out/` finishes and creates `comfort_dataset.csv`, `fusion_summary.json`, `run_meta.json`, `timeseries/`. All have `fixture: false`.
3. **A determinism:** run the matrix twice, outputs are byte-identical.
4. **A parity:** the nominal plant matches `core/` within 1% on RMS body acceleration and RMS tyre force.
5. **B:** `python ml/train.py --in out/comfort_dataset.csv --out out/` creates `comfort_predictions.json`, `envelope.json`, `semiactive_ranking.json`, all with `mock: false`.
6. **B sanity:** ML MAE is below physics-only MAE on the test split, or the deck says it is not. Interval coverage is reported. OOD scenarios are flagged unsupported.
7. **C routes:** each route in section 6 returns the right status for good and bad input (including a path-traversal attempt on the timeseries route).
8. **C dashboard:** the FUSION tab loads from real data. No "SAMPLE DATA" badge appears. Signals, Comfort and Validation sub-tabs show numbers that match the JSON files. Panels without data say "Planned" and show no numbers.
9. **Live run:** click "Run live" with a new seed. A result appears within a few seconds and passes the contract.
10. **Honesty check:** the limits and assumptions panel is visible. No text claims validation against a real vehicle.
