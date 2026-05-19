---
name: motor-fcs-mpc
description: PMSM Single-Vector Finite-Control-Set MPC Builder. Build an inner-loop finite-control-set MPC current controller for a three-phase voltage-source-inverter-driven PMSM (SPMSM/IPMSM via parameterization) in Simulink, with optional outer speed PI providing iq_ref. Use when constructing, reproducing, porting, or extending an FCS-MPC simulation in Simulink (keywords FCS-MPC, finite control set MPC, single-vector MPC, 7-vector / 8-vector MPC, predictive current control). Skip for FOC, DTC, sensorless, scalar control, BLDC trapezoidal, induction-motor MPC, multi-step horizon, weak-field, MTPA optimization, or pure theory questions. Layered on motor-pmsm-base.
metadata:
  version: "1.0"
---

# motor-fcs-mpc — PMSM Single-Vector Finite-Control-Set MPC Builder

Three-phase 2-level voltage-source inverter + PMSM (SPMSM / IPMSM via parameterization). Inner-loop = FCS-MPC current control in dq frame. Outer-loop (optional) = speed PI providing `iq_ref`. Generalizes across machine sub-types by parameterizing `Rs, Ld, Lq, ψf, Pn, J, F` — no topology change between sub-types.

Layered on [motor-pmsm-base](../motor-pmsm-base/SKILL.md). All base discipline applies (Goto TagVisibility, Vdc/BEMF rule, Visual 4-check, broken-FOC defense).

## Must-Follow Rules

1. **Plan first.** Before any `add_block`, write a numbered plan: parameter table, design-decision choices ([design_decisions.md](references/design_decisions.md)), build-script structure. Get user approval.
2. **One-click reproducibility.** Inject all parameters via `set_param(mdl, 'InitFcn', sprintf(...))`. Model must Run from `.slx` double-click in fresh MATLAB session. See [crit_conditions.md §J-CRIT](references/crit_conditions.md).
3. **MPC chart params via `sprintf` from build-script workspace.** Chart hardcodes `Rs/Ld/Lq/ψf/Pn/Tsc` as numeric literals at build time. Never put MPC params in an external `.m` file (drifts from plant). See [crit_conditions.md §K-CRIT](references/crit_conditions.md).
4. **Chart config = INHERITED + dual ZOH** (NOT DISCRETE+Tsc). `ch.SampleTime='-1'`, `ch.ChartUpdate='INHERITED'`, leave Inputs DataType/Size at Inherit. Rate is locked by ZOH @ Tsc on every chart input + ZOH @ Tsc on chart output. See [crit_conditions.md §G-CRIT](references/crit_conditions.md).
5. **Outer PI saturation is mandatory.** `LimitOutput='on'`, limits `[-iq_max, +iq_max]` with `1.5·Pn·ψf·iq_max ≥ 1.3·TL_max`. Without saturation, runaway `iq_ref` collapses the cost-weight ratio and MPC abandons d-axis. See [outer_pi_saturation.md](references/outer_pi_saturation.md).
6. **ZOH every chart input** even if upstream is already discrete. INHERITED chart's effective trigger rate is set by the fastest upstream signal; one un-ZOH'd input poisons the whole chart. See [crit_conditions.md §A-CRIT](references/crit_conditions.md).
7. **Use From Workspace + inline matrix for `ω_ref`**, NOT Step+RateLimiter. Continuous-mode RateLimiter escapes its slew limit on first solver step. See [crit_conditions.md §H-CRIT](references/crit_conditions.md).

## Build Flow

| Phase | Action | Reference |
|---|---|---|
| 0 | Validate inputs + sanity grid | base/[pre_build_grid.md](../motor-pmsm-base/references/pre_build_grid.md) |
| 1 | Plant layer (powergui Discrete @ Ts, DC, UB Inverter, PMSM Salient-pole, TL Step) | (this skill) |
| 2 | Measurement layer (Bus Selector → Goto/From) | (this skill) |
| 3 | Outer speed PI (optional, RPM domain, mandatory saturation) | [outer_pi_saturation.md](references/outer_pi_saturation.md) |
| 4 | Inner FCS-MPC chart | [algorithm_pseudocode.md](references/algorithm_pseudocode.md) + [crit_conditions.md §G/§A/§K](references/crit_conditions.md) |
| 5 | Logging (To Workspace @ Tsc) | base/[measurement_logger.md](../motor-pmsm-base/references/measurement_logger.md) |
| 6 | Solver (Variable-step Auto, MaxStep ≤ Ts; powergui handles SPS at Ts) | (this skill) |
| 7 | InitFcn injection | base/[plant_modeling.md](../motor-pmsm-base/references/plant_modeling.md) §InitFcn |
| 8 | Self-tests + acceptance | [acceptance_criteria.md](references/acceptance_criteria.md) |

If issues arise, consult [crit_conditions.md](references/crit_conditions.md) (A/D/G/H/J/K) and [anti_patterns.md](references/anti_patterns.md) (15 common mistakes).

## Required User Inputs

Ask the user before starting. Defaults in [parameter_defaults.md](references/parameter_defaults.md).

| Group | Parameter |
|---|---|
| Machine | `Rs` (Ω), `Ld, Lq` (H), `ψf` (V·s), `Pn`, `J` (kg·m²), `F` (N·m·s) |
| Power stage | `Vdc` (V) |
| Sampling | `Ts` (plant solver, ~1–5 μs), `Tsc` (control period, ~20–50 μs; `Tsc/Ts ≥ 10`) |
| Outer loop | `Kp_w, Ki_w` (RPM domain; recommend `pi_design.m`), `iq_max` (`1.5·Pn·ψf·iq_max ≥ 1.3·TL_max`) |
| MPC | control objective (`ripple-priority` / `balanced` / `torque-priority`), `λ_d, λ_q` per objective, `id_ref` (SPMSM: 0; IPMSM: ask) |
| Scenario | `StopTime`, `ramp_time`, `ω_ref_target` (RPM), `TL_step_time`, `TL_value` (`< 1.5·Pn·ψf·iq_max`) |

## Triggers / Skip

| ✅ Use | ❌ Skip |
|---|---|
| Build / port / extend FCS-MPC simulation in Simulink | FOC, DTC, sensorless, scalar V/Hz, BLDC trapezoidal |
| Single-vector / 7-vector / 8-vector MPC, predictive current control | Multi-step horizon (N>1), delay compensation, weak-field, MTPA optimization |
| Generalizing FCS-MPC to a new PMSM machine parameter set | Pure theory questions about MPC math |
| | MATLAB perf (`matlab-performance-optimizer`) or unit tests (`matlab-test-creator`) |

## Generalization Across Machine Sub-Types

| Sub-type | Parameter constraint | Strategy |
|---|---|---|
| SPMSM | `Ld == Lq` | `id_ref = 0` (no reluctance torque) |
| IPMSM mild saliency | `Lq > Ld`, `Lq/Ld ≤ 1.5` | `id_ref = 0` workable; MTPA gives ~5% torque-per-amp gain |
| IPMSM strong saliency | `Lq/Ld ≥ 2` | `id_ref` from MTPA solver (out of scope; ask user) |

Topology does not change — same blocks, same wiring, same chart algorithm, same CRIT conditions. Only parameters and `id_ref` strategy differ.

Out-of-scope sub-types: SynRM (`ψf ≈ 0`), IM, BLDC trapezoidal — different prediction equations.

## Sibling Skills

- [motor-pmsm-base](../motor-pmsm-base/SKILL.md) — base infrastructure (this skill layers on it)
- [motor-dtc-pmsm](../motor-dtc-pmsm/SKILL.md) — Direct Torque Control alternative
- [motor-smc-pmsm](../motor-smc-pmsm/SKILL.md) — Sliding Mode Control alternative
