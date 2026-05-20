# DTC Parameter Defaults

Recommended starting points for typical PMSM-DTC servo applications. Always confirm with user — these are **defaults**, not prescriptions.

## Sampling

| Parameter | Default | Acceptable range | Notes |
|---|---|---|---|
| `Tsc` (control period) | `50e-6` | `[20e-6, 100e-6]` | Hysteresis-decision sampling. For `fs_max = 5–20 kHz`, `Tsc ≈ 1 / (10–20·fs_max)` |
| `step_size` (plant solver) | `1e-6` | `[1e-7, Tsc/50]` | Power-electronics fixed-step solver. Must satisfy `step_size ≤ Tsc/50` |
| `fs_max` (switching freq cap) | `10e3` | `[5e3, 20e3]` | Used by HB sizing back-calculation |

## PI Design

| Parameter | Default | Acceptable range | Notes |
|---|---|---|---|
| `T_eq_factor` | `15` (DTC) | `[10, 20]` | DTC-specific (NOT FCS-MPC's 5). Hysteresis inner-loop bandwidth is much lower than current PI |
| `a_so` (SO factor) | `4` | `[2, 6]` | `a=4` gives `ζ_eq ≈ 0.71` (Kessler standard) |

## Mode

| Parameter | Default | Notes |
|---|---|---|
| `switching_table_mode` | `'6state'` | PMSM mandatory (Sutikno Table 2). Never default to `'8state'` for PMSM |
| `torque_hysteresis_levels` | `2` | 3-level for reverse-braking scenarios (out of v1) |
| `Te_feedback_mode` | `'alphabeta'` | αβ cross-product estimator (closer to real hardware). `'plant'` reads PMSM bus Te directly (educational) |
| `flux_drift_compensation` | `'none'` | Pure integrator; OK for sim < 1 s. v2+ must upgrade for long-run/hardware (LPF / HPF / observer) |

## Solver

| Parameter | Default | Notes |
|---|---|---|
| `SolverType` | `Fixed-step` | |
| `solver` | `'ode3'` | Fixed-step Bogacki-Shampine. SimPowerSystems standard pairing with powergui Discrete + IGBT |
| `FixedStep` | `step_size` | i.e., `1e-6` |
| `StopTime` | `sim_time` | |

## Hysteresis Bands (when `fs_max` unknown — fallback)

| Parameter | Default | Notes |
|---|---|---|
| `HB_T` | `0.075 · T_max` | 7.5% of torque limit |
| `HB_psi` | `0.025 · psi_ref` | 2.5% of flux reference |

When `fs_max` is supplied, use the back-calculation formulas in [hb_sizing.md](hb_sizing.md).

## Voltage Source

| Parameter | Notes |
|---|---|
| `Vdc` | Compute from `1.5 · sqrt(3) · ω_e_max · ψ_f` (base/pre_build_grid.md headroom rule). Higher Vdc improves slew rate and torque transient response |

## Scenario (Typical Servo Test)

| Parameter | Default | Notes |
|---|---|---|
| `sim_time` | `0.6 s` | Allows settling + load step + post-load recovery |
| `omega_ref_rpm` | 1000–2000 RPM | Match machine rated speed |
| `TL_step_t` | `0.3 s` | Place after `ω_ref` settled |
| `TL_after` | 30–80% of rated torque | Must satisfy `TL_after < 1.5·Pn·ψf·iq_max` |

## ψ_ref Selection (from B-CRIT)

| Scenario | `psi_ref` |
|---|---|
| Reference value supplied | Copy from reference |
| id=0 load-point stator flux | `sqrt(ψ_f² + (Lq · iq_max)²)` (not true MTPA; id<0 needed for that) |
| SPMSM light-load only | `ψ_f` (verify `\|ψ_s\|_load` does not exceed `ψ_f`) |
| ⛔ NEVER for IPMSM | `ψ_f` (motor will reverse — see [crit_conditions.md §B-CRIT](crit_conditions.md)) |

## T_max Selection

| Scenario | `T_max` |
|---|---|
| Reference value supplied | Copy from reference |
| Default | `1.5 · Pn · ψ_f · iq_max` |

## Logger

| Parameter | Default | Notes |
|---|---|---|
| `SaveFormat` | `StructureWithTime` | Compatible with offline overlay |
| `SampleTime` | `Tsc` | Match control period |
| Channel count | 19 | Includes both ψ_α and ψ_β (F-CRIT) |

## Initial Conditions

| Variable | Default | Notes |
|---|---|---|
| `psi_alpha_0` | `psi_f` | Stator flux initial value α-axis |
| `psi_beta_0` | `0` | β-axis starts at zero |
