---
name: motor-smc-pmsm
description: PMSM Sliding Mode Control Speed-Loop Builder. Build a Sliding Mode Control (SMC) speed-loop outer controller (PD-type sliding surface + Super-Twisting Algorithm reaching law) + dq current-loop PI inner controller with cross-decoupling feedforward + Anti_Park library block + SVPWM library block + Universal Bridge IGBT inverter + PMSM Discrete plant for a three-phase voltage-source-inverter-driven PMSM in Simulink. v1 baseline supports SPMSM and mild-saliency IPMSM (id_ref=0). Use when constructing, reproducing, porting, or extending an SMC-based PMSM speed-loop simulation in Simulink (keywords SMC, sliding mode control, super-twisting, STA, PD-type sliding, boundary layer SMC, 滑模控制, 滑模面). Skip for FCS-MPC, FOC, DTC, sensorless, scalar V/Hz, BLDC trapezoidal, induction-motor SMC, observer-based / adaptive / disturbance-observer / neural-network / fuzzy SMC variants, strong-saliency IPMSM MTPA, weak-field, or pure theory questions. Layered on motor-pmsm-base.
metadata:
  version: "1.0"
---

# motor-smc-pmsm — PMSM Sliding Mode Control Speed-Loop Builder

Three-phase 2-level voltage-source inverter + PMSM. Outer-loop = **PD-type sliding surface + Super-Twisting Algorithm (STA)** providing `iq_ref` (continuous through `s=0`, second-order SMC). Inner-loop = **dq current PI ×2 with cross-decoupling feedforward** (PZC for RL plant + BEMF compensation). Modulation = **Anti_Park + SVPWM** library blocks. v1 baseline supports SPMSM and mild-saliency IPMSM (`Lq/Ld < 2`); strong-saliency IPMSM MTPA mid-loop deferred.

Layered on [motor-pmsm-base](../motor-pmsm-base/SKILL.md). All base discipline applies.

## ⚠️ Signing Authority Note (SMC-specific)

Most users may lack independent SMC domain expertise. When the user declares this gap, switch to **AI-self-audit mode** (see base/[theory_anchor.md](../motor-pmsm-base/references/theory_anchor.md)):

- §A/§B (PI / DTC) formulas: user-signed
- §C (SMC) formulas: AI-self-audited with verifiable sources (≥ 2 independent cross-check + ≥ 1 link openable + ≥ 1 PDF cached)

**Fail-safe replacement for user formula sign-off**: a Phase 8 G4 user visual review on MATLAB desktop (4 Scopes + abc currents AC sinusoidal verification). G4 cannot be skipped.

## Must-Follow Rules

1. **Plan first** — Numbered plan with 25-input table, design-decision choices ([design_decisions.md](references/design_decisions.md)), build-script structure. Get user approval.
2. **One-click reproducibility** — Inject all parameters via `set_param(mdl, 'InitFcn', ...)`. See [crit_conditions.md §D-CRIT](references/crit_conditions.md).
3. **PD-type sliding + STA architecture** — `s = e + λ·de/dt` (Filtered Derivative `s/(Tf·s+1)`) + STA reaching law `u = K1·|s|^0.5·sgn(s) + K2·∫sgn(s)`. Do NOT use PI-type sliding (integrator wind-up voids Lyapunov proof) or classic sgn / boundary-layer sat (chattering). See [control_law.md](references/control_law.md).
4. **iq_ref Saturation is mandatory** — Saturation block at `iq_ref` output BEFORE feeding to current PI, limits `±iq_max`. See [crit_conditions.md §B-CRIT](references/crit_conditions.md).
5. **Lyapunov STA gains auto-computed; `B > 0` mandatory** — `K1 > 1.5·√M`, `K2 > 1.1·M` where `M = (TL_max + B·ω_max)/J`. Build script asserts both. Plant friction `B > 0` is mandatory (SMC needs a dissipation port). See [control_law.md](references/control_law.md) and [crit_conditions.md §C-CRIT](references/crit_conditions.md).
6. **`Goto_The TagVisibility='global'` MANDATORY** — Anti_Park's internal `From "The"` requires the parent's `Goto_The` to publish globally; default `'local'` is silent failure mode (FOC degenerates to lab-frame open-loop). See base/[broken_foc_diagnostics.md](../motor-pmsm-base/references/broken_foc_diagnostics.md) §F-CRIT 1 + [crit_conditions.md §G-CRIT](references/crit_conditions.md).
7. **FF Mux input is `ω_e` (NOT `θ_e`)** — Cross-decoupling FF formulas need electrical angular velocity, not position. Use a dedicated `Gain_Pn_omega` block independent of `Gain_Pn` (which provides θ_e for `Goto_The`). See [ff_decoupling.md](references/ff_decoupling.md) and [crit_conditions.md §H-CRIT](references/crit_conditions.md).
8. **Add 4 Scopes for human inspection** — `Scope_wm_RPM` / `Scope_iq` / `Scope_s` / `Scope_Te`. See [crit_conditions.md §F-CRIT](references/crit_conditions.md). Visual 4-check (motor rotates / iq tracks / abc AC sinusoidal NOT DC-locked / Te energy balance) is mandatory pre-condition for trusting any numerical metric.
9. **Solver = `ode3` + `ZeroCrossControl='DisableAll'`** — STA's `Sign` block is discontinuous. Variable-step or ZC ON causes step explosion. See [crit_conditions.md §E-CRIT](references/crit_conditions.md).
10. **SVPWM sector=7 startup workaround** — At t=0 transient, `Vα=Vβ=0` makes the SVPWM library's `Sector_Caculate` output sector=7 (invalid). Apply local-instance workaround on the library block. See [svpwm_workaround.md](references/svpwm_workaround.md).

## Build Flow

| Phase | Action | Reference |
|---|---|---|
| 0 | Validate inputs + sanity grid | base/[pre_build_grid.md](../motor-pmsm-base/references/pre_build_grid.md) |
| 1 | Plant layer (powergui, DC, UB, PMSM, TL Step, BusSelector with internal dq) | (this skill) |
| 2 | θ_e calc + `Goto_The TagVisibility='global'` (G-CRIT) | [crit_conditions.md](references/crit_conditions.md) |
| 3 | Outer Speed SMC (PD sliding + STA + Saturation_iq) | [control_law.md](references/control_law.md) |
| 4 | Inner Current PI ×2 (PZC, with `LimitOutput` + AW clamping) | [control_law.md](references/control_law.md) |
| 4.5 | Cross-decoupling Feedforward (`Gain_Pn_omega` independent block) | [ff_decoupling.md](references/ff_decoupling.md) |
| 5 | Modulation (Anti_Park + SVPWM library + Universal Bridge) | [svpwm_workaround.md](references/svpwm_workaround.md) |
| 6 | Logger (14 channels, 4 SMC-specific) | [crit_conditions.md §F-CRIT](references/crit_conditions.md) |
| 7 | 4 Scopes (wm_RPM / iq / s / Te) | [crit_conditions.md §F-CRIT](references/crit_conditions.md) |
| 8 | Solver (Fixed-step ode3 + ZC OFF) | [crit_conditions.md §E-CRIT](references/crit_conditions.md) |
| 9 | InitFcn injection | [crit_conditions.md §D-CRIT](references/crit_conditions.md) |
| 10 | Layout cleanup + self-tests | [acceptance_criteria.md](references/acceptance_criteria.md) |

## Required User Inputs (25)

Ask user before starting. Defaults in [parameter_defaults.md](references/parameter_defaults.md).

| Group | Parameters |
|---|---|
| **Machine** (6) | `Pn`, `Rs`, `Ld, Lq` (mild-saliency `Lq/Ld < 2` v1; strong saliency deferred), `psi_f`, `J`, **`B > 0` mandatory** (default 0.008) |
| **Power stage** (1) | `Vdc` (default ≥ 1.5× ω_max·ψ_f / √3 peak phase BEMF) |
| **Control** (3) | `iq_max`, `T_max` (default `1.5·Pn·ψf·iq_max`), **`TL_max` always asked** (Lyapunov bound input) |
| **SMC design** (4) | `lambda_pd_settling` (default 10 ms), `Tf_deriv` (default `Tsc`), `K1_sta` (auto), `K2_sta` (auto) |
| **Current PI** (1) | `omega_c_inner` (default 2000 rad/s, ≈ 5× SMC bandwidth) |
| **Sampling** (3) | `Tsc` (default 50 μs), `step_size` (default 1 μs; `step_size ≤ Tsc/50`), `ramp_time` (default 0.5 s) |
| **Mode** (1) | `motor_type` (default `'SPMSM'`; `'IPMSM-mild'` for `Lq/Ld < 2`) |
| **Solver** (1) | `solver` (default `'ode3'`) |
| **Scenario** (4) | `omega_ref_rpm`, `sim_time`, `TL_step_t`, `TL_after` |

## Triggers / Skip

| ✅ Use | ❌ Skip |
|---|---|
| Build / port / extend SMC speed-loop simulation in Simulink | FCS-MPC, FOC, DTC, sensorless, scalar V/Hz, BLDC trapezoidal |
| Super-twisting / PD-type sliding / boundary layer SMC | Observer-based / adaptive / disturbance-observer / neural-network / fuzzy SMC |
| Robust speed control on PMSM | Strong-saliency IPMSM MTPA mid-loop, weak-field |
| Generalizing SMC to a new PMSM machine parameter set | Pure theory questions, paper writing, MATLAB perf, unit tests |

## Generalization Across Machine Sub-Types

| Sub-type | Parameter constraint | Strategy |
|---|---|---|
| SPMSM | `Ld ≈ Lq` | `id_ref = 0` |
| IPMSM mild | `Lq > Ld`, `Lq/Ld < 2` | `id_ref = 0` workable |
| IPMSM strong | `Lq/Ld ≥ 2` | MTPA mid-loop required (out of v1 scope) |

Topology does not change — same blocks, same wiring, same sliding surface, same STA, same CRIT conditions.

Out-of-scope sub-types: SynRM (`ψ_f ≈ 0`), IM, BLDC trapezoidal — different prediction equations.

## Sibling Skills

- [motor-pmsm-base](../motor-pmsm-base/SKILL.md) — base infrastructure
- [motor-fcs-mpc](../motor-fcs-mpc/SKILL.md) — FCS-MPC alternative
- [motor-dtc-pmsm](../motor-dtc-pmsm/SKILL.md) — DTC alternative
