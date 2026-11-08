# PRD B: ML Comfort Model, Envelope Monitor, Semi-Active Ranking (Python)

**Owner:** Person B. **Folder you own:** `ml/`.
**Read first:** `docs/CONTRACTS.md` (file schemas, scenario IDs, timeline). It is the source of truth for the files you read and write.
**Project:** Techgium 10 POC, *AI-Enabled Suspension Digital Twin with Sensor Fusion for Ride Comfort Prediction*. Simulated data only.

---

## 0. Instructions for the coding agent

1. Read this file and `docs/CONTRACTS.md` fully.
2. You do **not** touch Rust, the backend or the frontend. You read `out/comfort_dataset.csv` (from Person A) and write three JSON files.
3. **You are not blocked on A.** Start with a mock dataset generator that matches the CSV schema (task B0). Switch to the real CSV at the integration freeze.
4. Everything is seeded. Same input and seed give identical output.
5. **Mock results must never be mistaken for real ones.** Anything produced from mock data has `meta.mock: true` and `meta.fixture: true`. Never print or save metrics from mock data in a form that looks final.
6. Never invent numbers in outputs. Every number comes from computation on the dataset.
7. Output formats are fixed by `docs/CONTRACTS.md`. If you must change one, stop and tell the other owners.

## 1. Goal

Turn the physics comfort value into a better prediction by learning only what physics misses, with a calibrated confidence interval. Flag scenarios outside the training coverage as unsupported. For the semi-active case, compare predefined damping presets and recommend the best one, or say there is no clear winner.

## 2. Inputs and outputs

| | File | Contract |
|---|---|---|
| Input | `out/comfort_dataset.csv` | 4.1 |
| Input (optional) | `out/run_meta.json` for `travel_limit_mm`, `mu_kg` | 4.7 |
| Output | `out/comfort_predictions.json` | 4.4 |
| Output | `out/envelope.json` | 4.5 |
| Output | `out/semiactive_ranking.json` | 4.6 |
| Fixtures | the three outputs, with `fixture: true`, in `contracts/fixtures/` | Section 7 |

## 3. Layout

```
ml/
  requirements.txt     # numpy, pandas, scikit-learn, scipy, matplotlib, pytest
  make_mock_dataset.py
  train.py             # entry: python ml/train.py --in out/comfort_dataset.csv --out out/
  features.py
  model.py             # quantile GBR + conformal
  envelope.py
  semiactive.py
  plots.py
  tests/
```
`train.py` runs the whole chain: model, envelope, semi-active, writes all three JSON files and the plots.

## 4. Technical specification

### 4.1 Idea
`p_phys` is the physics value (nominal twin driven by the fused road estimate). `y_ref` is the reference from a deliberately different "real" plant. The model learns the residual `r = y_ref - p_phys` and predicts `comfort_pred = p_phys + r_hat`. Physics stays the backbone, ML only corrects what physics misses.

### 4.2 Data split (important, prevents leakage)
- Split by **seed and scenario**, never by random row.
- `split == train`: seeds 0-6 for fitting, seeds 7-9 for conformal calibration.
- `split == test`: seeds 100-104, never used for fitting or calibration.
- `split == ood`: out-of-distribution scenarios, only for the envelope test and reported separately.

### 4.3 Features
`p_phys`, road class (ordinal A=0..C=2), `speed_mps`, `ms_kg`, `c_nsm`, `cov_trace`, `dropout_frac`, `band_lo`, `band_mid`, `band_hi`. Use these columns only. Do not use `y_ref`, `rms_tyre_force_n`, `max_travel_mm` or `bottom_out` as features (they come from the reference plant).

### 4.4 Model (`model.py`)
- `GradientBoostingRegressor` with `loss="quantile"` at 0.05, 0.5, 0.95 on the residual `r`.
- **Conformalised quantile regression:** on the calibration seeds compute the score `s_i = max(q_lo_i - r_i, r_i - q_hi_i)`, take the `ceil((n+1)(1-alpha)) / n` quantile `q_hat` (alpha = 0.1), and widen each interval to `[q_lo - q_hat, q_hi + q_hat]`.
- Point prediction: the median quantile model. Final `comfort_pred = max(p_phys + r_hat_median, 0)`. `lo` and `hi` are `p_phys` plus the widened bounds, clipped at 0.
- Baselines: physics only (`r_hat = 0`) and `LinearRegression` on the same features.
- Keep hyperparameters modest (for example `n_estimators = 200`, `max_depth = 3`, `learning_rate = 0.05`). Fix `random_state`. Tune only on the calibration seeds.

### 4.5 Envelope (`envelope.py`)
- Training ranges per feature: road class, `speed_mps`, `ms_kg`, `c_nsm`, `dropout_pct`.
- Mahalanobis distance on those features, threshold at the 99th percentile of the training distances.
- A scenario is **unsupported** if any feature is outside its training range, or the distance exceeds the threshold. `reason` is `range:<feature>`, `mahalanobis` or `none`.
- Apply it to every row. Set `supported` in `comfort_predictions.json` accordingly. OOD detection rate = share of `ood` rows flagged.

### 4.6 Semi-active (`semiactive.py`)
- Group rows by `group_id` (scenario ID without damping and seed). Use the **test** split. For each group and preset, average `comfort_pred`, `lo`, `hi` and the tyre-load and travel columns over seeds.
- Constraints per option: DLC = `rms_tyre_force_n / ((ms_kg + mu_kg) * 9.81)` must be at most `dlc_limit` (default 0.33, configurable); `max_travel_mm` at most `travel_limit_mm` (from `run_meta.json`, else a CLI argument); no bottom-out. Violations list `tyre_load`, `travel`, `bottom_out`.
- Recommend the feasible option with the lowest `comfort_pred`. If the intervals of the best two feasible options overlap, set `recommended = null` and `note = "no clear winner"`. If none is feasible, `recommended = null` and `note = "no feasible option"`.
- `mu_kg` is 78 unless `run_meta.json` says otherwise.

### 4.7 Mock dataset (`make_mock_dataset.py`)
Generates a CSV with the exact header from CONTRACTS 4.1 and the three splits, using the scenario matrix: class {A, B, C} x speed {10, 20, 30} x load {300, 440, 580} x preset {soft, medium, firm} x dropout {0, 5, 20}, seeds 0-9 train, 100-104 test; OOD rows class D, speed 40, load 700, dropout 30.
Make the data **learnable but imperfect**: `p_phys` grows with road class and speed and shrinks with damping near the sweet spot; `y_ref = p_phys * (1 + bias(c, ms)) + heteroscedastic noise`, where the bias depends smoothly on damping and load (so the model has something to learn). Keep values in a plausible 0.2-2.5 m/s² range. Fill the other columns plausibly (`rms_tyre_force_n` rises with speed and class and falls with damping, and so on). Write `meta.mock: true` into every JSON output derived from it. Print a loud warning in the console.

## 5. Tasks (in order)

| Task | What | Done when |
|---|---|---|
| **B0** | Environment, `requirements.txt`, `make_mock_dataset.py` | Mock CSV matches the schema; row counts per split are right |
| **B1** | Fixtures for the three outputs in `contracts/fixtures/` (within 2 hours) | Schema-valid, `fixture: true`, `mock: true` |
| **B2** | `features.py`, `model.py`: quantile GBR, CQR, baselines | Test-split MAE for physics-only, linear and ML printed; coverage printed |
| **B3** | `envelope.py` | Training rows mostly supported; OOD rows flagged; `reason` filled |
| **B4** | `semiactive.py` | Valid JSON; "no clear winner" and "no feasible option" cases produced in tests |
| **B5** | `train.py` end to end, `plots.py` (predicted vs reference scatter, coverage, residual histogram, OOD flags) as PNG in `out/plots/` | One command writes all outputs and plots |
| **B6** | Unit tests: no train/test leakage by seed; interval is never inverted (`lo <= hi`); conformal coverage on held-out data is near 90%; envelope reasons are valid; JSON matches schema | `pytest` passes |
| **B7** | **Integration freeze (Sun 4 PM):** run on A's real `out/comfort_dataset.csv`. Outputs now have `mock: false`, `fixture: false`. Report the real metrics honestly | Real files written; any surprise reported |
| **B8** (optional) | Export the fitted trees to JSON so Rust can evaluate them for the live endpoint | JSON round-trips to the same predictions |

## 6. Acceptance criteria and metrics to report

Targets are goals to measure, not claims to make in advance. **If a target is missed on real data, report it as it is.**

| Metric | Target |
|---|---|
| MAE of `comfort_pred` vs `y_ref` on test, vs physics-only and linear baselines | ML below physics-only |
| Interval coverage on test | About 90% |
| Mean interval width | Reported |
| OOD detection rate | Reported |
| Leakage | None, enforced by tests |
| Reproducibility | Same input gives the same output |

## 7. Risks

| Risk | Mitigation |
|---|---|
| Model looks too good because the data is synthetic | Say so in `meta`, use the OOD set, always report the physics-only baseline |
| Real CSV from A arrives late | Develop on the mock, schema is fixed, switching is one path change |
| Intervals too wide to be useful | Report width, do not hide it; tune only on the calibration seeds |
| Small test set gives noisy metrics | Report the number of test rows next to every metric |
