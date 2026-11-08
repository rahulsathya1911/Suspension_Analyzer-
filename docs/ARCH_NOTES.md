# Architecture Notes — Task A0 Audit of `core/`

## Rust Edition & Dependencies

- **Edition:** 2024
- **External dependencies:** none (zero deps — uses only `std`)

## Public API surface

| Item | Signature | Notes |
|---|---|---|
| `RoadProfile` (enum) | `Step { height }`, `Sine { amplitude, freq_hz }`, `Iso8608 { roughness_coefficient, vehicle_speed_mps, seed }` | Variants are public; method `precompute()` is **private** |
| `SimulationResult` (struct) | Fields: `zeta`, `mass_ratio`, `recommended_mu`, `time: Vec<f64>`, `body_user: Vec<f64>`, `wheel_user: Vec<f64>`, `rms_body_acc`, `rms_tire_force`, `max_suspension_travel`, `iso2631_weighted_rms` | Displacement time series only — no velocities, no acceleration series |
| `run_simulation` | `(ms, mu, k, c, kt, travel_limit_m, &RoadProfile) -> SimulationResult` | Hard-coded `dt=0.001`, `N_STEPS=5001` (5 s). Includes bump-stop logic. |
| `compute_frf` | `(ms, mu, k, c, kt, &[f64], f64) -> Vec<FrfPoint>` | Analytical FRF via Cramer's rule |
| `log_freq_range` | `(f_min, f_max, n) -> Vec<f64>` | Log-spaced frequency vector |
| `parameter_sweep` | `(ms, mu, kt, &[f64], &[f64], &RoadProfile) -> Vec<ParetoPoint>` | Grid sweep |
| `extract_pareto_front` | `(&[ParetoPoint]) -> Vec<ParetoPoint>` | 3-objective Pareto filter |
| `linspace` | `(start, end, n) -> Vec<f64>` | Linear spacing |

## Private items we need access to

| Item | Why private | Resolution |
|---|---|---|
| `RoadProfile::precompute(&self, dt, n_steps) -> Vec<f64>` | `fn` (not `pub`) | **Added thin `pub` wrapper `generate_road_profile`** in `core/` |
| `wk_gain(f) -> f64` | `fn` | Not reused — fusion implements proper biquad cascade per PRD §5.6 |
| `iso2631_weighted_rms(acc, dt) -> f64` | `fn` | Exposed via new `pub fn iso2631_weighted_rms_pub` for cross-check |
| `derivatives(...)` | `fn` | Not reused — fusion has its own plant with perturbations |

## Units (confirmed SI throughout)

- Displacement: metres
- Velocity: m/s
- Acceleration: m/s²
- Force: N
- Time: s
- Spring/stiffness: N/m
- Damping: N·s/m

## ISO 8608 roughness coefficients

| Class | core/ uses | PRD specifies (n0=0.1 c/m) |
|---|---|---|
| A | 64e-6 | **16e-6** |
| B | 256e-6 | **64e-6** |
| C | 1024e-6 | **256e-6** |
| D | 4096e-6 | **1024e-6** |

The core values are 4× higher. The fusion crate passes the correct PRD values
to `generate_road_profile` (the coefficient is a parameter, not hard-coded in the generator).

## ISO 2631-1 implementation in core/

- **Method:** Goertzel-based frequency-domain computation per DFT bin
- **Weighting:** Piecewise linear approximation of Wk in log-frequency space
- The PRD requires a biquad cascade (bilinear transform) — fusion implements this
- `iso2631_weighted_rms_pub` is exposed for **cross-check only**

## Solver characteristics

- **Method:** Classical RK4, dt = 0.001 s
- **Duration:** Fixed 5 s (N_STEPS = 5001) — fusion needs configurable 20 s
- **Road input:** midpoint interpolation between steps for RK4 k2/k3
- **State vector:** `[x1=zs, x2=vs, x3=zu, x4=vu]`, input `zr`
- **Bump stop:** penalty force `1e6 * (|travel| - limit) * sign(travel)` when `|travel| > travel_limit_m`
- **No time-series return** for velocities — only displacements
- **Resolution:** fusion writes its own RK4 (full state output + perturbations)

## Random number generation

- core/ uses a **simple LCG** for ISO 8608 phases: `s = s * 6364136223846793005 + 1442695040888963407`
- Deterministic per seed, but not ChaCha8Rng
- fusion reuses core's generator (via wrapper) for road profiles — the LCG seed is deterministic
- All **new** randomness in fusion (sensor noise, perturbations) uses `rand_chacha::ChaCha8Rng`

## Reuse plan

| What | How |
|---|---|
| Road profile generation | Via new `core::generate_road_profile()` wrapper |
| Quarter-car equations | Own RK4 in `plant.rs` (matches core equations, adds perturbations) |
| ISO 2631 Wk weighting | Own biquad cascade in `comfort.rs`; cross-check vs `core::iso2631_weighted_rms_pub` |
| RMS body acc / tire force | Computed from own simulation (trivial) |
| FRF, Pareto, linspace | Not needed by fusion |

