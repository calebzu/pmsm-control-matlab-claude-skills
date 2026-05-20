---
name: motor-dtc-pmsm
description: PMSM Direct Torque Control Builder. Build a Direct Torque Control (DTC) outer-loop torque/flux controller for a three-phase voltage-source-inverter-driven PMSM (SPMSM/IPMSM via parameterization) in Simulink using Sutikno 6-state switching table, αβ stationary frame, 2-level hysteresis on (T, ψ), with outer-loop Speed PI providing Te_ref. Use when constructing, reproducing, porting, or extending a DTC simulation in Simulink (keywords DTC, direct torque control, Takahashi DTC, Sutikno DTC, hysteresis-based torque control, switching table, 磁链滞环, 转矩滞环). Skip for FCS-MPC, FOC, sensorless, scalar V/Hz, BLDC trapezoidal, induction-motor DTC, DTC-SVM, MP-DTC, or pure theory questions. Layered on motor-pmsm-base.
metadata:
  version: "1.0"
---

# motor-dtc-pmsm — PMSM Direct Torque Control Builder

Three-phase 2-level voltage-source inverter + PMSM (SPMSM / IPMSM via parameterization). Outer-loop = Speed PI providing `Te_ref`. Inner-loop = **Direct Torque Control** in αβ stationary frame: stator flux integrator + magnitude/angle/sector + 2-level hysteresis on (T, ψ) + Sutikno 6-state switching table → V_k → gate.

Layered on [motor-pmsm-base](../motor-pmsm-base/SKILL.md). All base discipline applies.

## Must-Follow Rules

1. **Plan first** — Numbered plan with 21 input table, design-decision choices ([design_decisions.md](references/design_decisions.md)), build-script structure. Get user approval.
2. **One-click reproducibility** — All parameters injected via `set_param(mdl, 'InitFcn', ...)`. Model must Run from `.slx` double-click in fresh MATLAB session. See [crit_conditions.md §E-CRIT](references/crit_conditions.md).
3. **Default 6-state switching table for PMSM** — `switching_table_mode='6state'` (Sutikno 2011 Table 2). Never default to 8-state Takahashi for PMSM — V0/V7 zero vectors cause flux to decay. See [switching_table.md](references/switching_table.md) and [crit_conditions.md §A-CRIT](references/crit_conditions.md).
4. **`ψ_ref ≠ ψ_f` by default** — Take `psi_ref` from reference if supplied; else compute MTPA load operating point `|ψ_s|_load = sqrt(ψ_f² + (Lq·iq_max)²)`. Never default `ψ_ref = ψ_f` for IPMSM. See [crit_conditions.md §B-CRIT](references/crit_conditions.md).
5. **`T_eq_factor = 15` for DTC, NOT 5** — DTC's hysteresis inner loop has lower bandwidth than FCS-MPC's current PI. Compute Speed PI gains via `pi_design('SO', J, 1, T_eq, a_so)` with `T_eq = 15·Tsc`, `a_so = 4`. **Pass `Kt = 1`** because DTC outer PI directly outputs `Te_ref` [N·m] (plant has no Kt). See [crit_conditions.md §C-CRIT](references/crit_conditions.md).
6. **Chart config = INHERITED + dual ZOH** — `ch.SampleTime='-1'`, `ch.ChartUpdate='INHERITED'`. ZOH @ Tsc on every chart input (5 inputs: `Te_ref, ia, ib, ua, ub`) AND every chart output (9 outputs: `gate, Te_meas, mag_psi, ψ_α, ψ_β, sector, V_k, C_ψ, C_T`). See [crit_conditions.md §D-CRIT](references/crit_conditions.md).
7. **Speed PI saturation is mandatory** — `Saturation` block after PI with limits `[-T_max, +T_max]`. Anti-windup off for v1 baseline; production should add clamp or back-calc.
8. **Add 4 Scopes for human inspection** — `Scope_wm_RPM` / `Scope_Te` / `Scope_psi_mag` / `Scope_psi_alphabeta` (XY Graph). The αβ XY plot is the most diagnostic — circular trajectory means healthy 6-state; hexagonal-with-inner-circle means 8-state pollution. See [crit_conditions.md §F-CRIT](references/crit_conditions.md).
9. **Non-overlapping wiring** — Every `add_block` specifies `Position` per X/Y bands; `arrangeSystem(mdl, 'FullLayout')` as final fallback.
10. **HB reverse-calculation from `fs_max`** — Use plant + Tsc to back-calc `HB_T_min` and `HB_psi_min`. Percentages (HB_T = 7.5% T_max, HB_ψ = 2.5% ψ_ref) are fallback only when `fs_max` is unknown. See [hb_sizing.md](references/hb_sizing.md).

## Build Flow

| Phase | Action | Reference |
|---|---|---|
| 0 | Validate inputs + sanity grid | base/[pre_build_grid.md](../motor-pmsm-base/references/pre_build_grid.md) |
| 1 | Plant layer (powergui Discrete @ step_size, DC, UB, PMSM, TL Step, current/voltage measurement) | (this skill) |
| 2 | Measurement + αβ transform layer (Clark for currents, Clark for voltages) | (this skill) |
| 3 | Outer Speed PI (RPM domain, mandatory saturation, SO method `T_eq=15·Tsc`) | [crit_conditions.md §C-CRIT](references/crit_conditions.md) |
| 4 | Inner DTC chart (flux integrator + magnitude/sector + hysteresis + 6-state table) | [chart_algorithm.md](references/chart_algorithm.md) + [switching_table.md](references/switching_table.md) |
| 5 | Logger (19 channels, including ψ_α and ψ_β) | [crit_conditions.md §F-CRIT](references/crit_conditions.md) |
| 6 | 4 Scopes (wm_RPM / Te / mag_psi / αβ XY) | [crit_conditions.md §F-CRIT](references/crit_conditions.md) |
| 7 | Solver (Fixed-step ode3, FixedStep = step_size; powergui Discrete) | (this skill) |
| 8 | InitFcn injection | [crit_conditions.md §E-CRIT](references/crit_conditions.md) |
| 9 | Layout cleanup (`arrangeSystem('FullLayout')`) | (this skill) |
| 10 | Self-tests + acceptance | [acceptance_criteria.md](references/acceptance_criteria.md) |

## Required User Inputs (21)

Ask user before starting. Defaults in [parameter_defaults.md](references/parameter_defaults.md).

| Group | Parameters |
|---|---|
| **Machine** (6) | `Pn`, `Rs`, `Ld, Lq`, `psi_f`, `J`, `B` (B=0 forces SO method for Speed PI) |
| **Power stage** (1) | `Vdc` |
| **Control** (3) | `psi_ref` (reference value if supplied; else MTPA — never `ψ_f` for IPMSM), `T_max` (reference value if supplied; else `1.5·Pn·ψf·iq_max`), `iq_max` |
| **Sampling** (3) | `Tsc` (default 50 μs), `step_size` (default 1 μs; `step_size ≤ Tsc/50`), `fs_max` (default 10 kHz) |
| **PI design** (2) | `T_eq_factor` (default **15** for DTC), `a_so` (default 4 for SO ζ_eq ≈ 0.71) |
| **Mode** (4) | `switching_table_mode` (default `'6state'`), `torque_hysteresis_levels` (default 2), `Te_feedback_mode` (default `'alphabeta'`), `flux_drift_compensation` (default `'none'`) |
| **Solver** (1) | `solver` (default `'ode3'`) |
| **Scenario** (4) | `omega_ref_rpm`, `sim_time`, `TL_step_t`, `TL_after` |

## Triggers / Skip

| ✅ Use | ❌ Skip |
|---|---|
| Build / port / extend DTC simulation in Simulink | FCS-MPC, FOC, sensorless, scalar V/Hz, BLDC trapezoidal |
| Takahashi DTC, Sutikno DTC, hysteresis-based torque control | DTC-SVM (constant switching freq variant), MP-DTC (predictive DTC), 12-sector schemes, deadbeat flux/torque control |
| Switching table for PMSM DTC (6-state) | Online parameter adaptation, MTPA mid-loop, weak-field |
| Generalizing DTC to a new PMSM machine parameter set | Pure theory questions, paper writing, MATLAB perf, unit tests |

## Generalization Across Machine Sub-Types

| Sub-type | Parameter constraint | Strategy |
|---|---|---|
| SPMSM | `Ld == Lq` | `psi_ref ≈ ψ_f` workable (verify load doesn't push `\|ψ_s\|_load` above `ψ_f`) |
| IPMSM mild saliency | `Lq > Ld`, `Lq/Ld ≤ 1.5` | `psi_ref` from MTPA: `sqrt(ψ_f² + (Lq·iq_max)²)` |
| IPMSM strong saliency | `Lq/Ld ≥ 2` | Same MTPA formula; `T_max` may need higher headroom |

Topology does not change — same blocks, same wiring, same chart, same CRIT conditions. Only `psi_ref` and `iq_max` budgets differ.

Out-of-scope sub-types: SynRM (`ψ_f ≈ 0`), IM, BLDC trapezoidal — different prediction equations and switching tables.

## Known Limitation: Classical DTC Te Ripple

Hysteresis-driven switching produces an inherent torque ripple intrinsic to the bang-bang structure. Steady-state mean Te tracks reference within < 1%, but the instantaneous-Te ripple band may exceed simple ±5% bands. Acceptance Criteria Mode A allows S2 to be the marginal metric in the 4/5 budget per [acceptance_criteria.md](references/acceptance_criteria.md). Advanced variants that reduce ripple (DTC-SVM, MP-DTC, Sutikno Table 3) are out of scope.

## Sibling Skills

- [motor-pmsm-base](../motor-pmsm-base/SKILL.md) — base infrastructure
- [motor-fcs-mpc](../motor-fcs-mpc/SKILL.md) — FCS-MPC alternative
- [motor-smc-pmsm](../motor-smc-pmsm/SKILL.md) — SMC alternative
