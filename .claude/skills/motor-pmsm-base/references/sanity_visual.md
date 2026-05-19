# Sanity Check + Visual 4-Check (Phase 4.5 + Phase 7)

## Phase 4.5: Closed-Loop Transfer Function Sanity Check

Before any extensive simulation, hand-derive three closed-loop transfer functions (~30 minutes):

- **Reference tracking**: `y(s) / r(s)`
- **Disturbance rejection**: `y(s) / d(s)`  (load step is the critical disturbance for motor control)
- **Noise** (when relevant): `y(s) / n(s)`

Find the **slowest pole**: `s_slow = min(|Re(s_i)|)` → `τ_max = 1 / |Re(s_slow)|`. Verify `T_window ≥ 5 · τ_max` where `T_window` is the test scenario evaluation window length.

If the rule is violated:
- (a) Lengthen `sim_time` until `T_window` is sufficient (do not change controller), OR
- (b) Redesign the controller to reduce `τ_max` (do not change `sim_time`)

**Never** "looks fine, should be OK." Implicit shortcut here is the leading cause of late-stage discoveries that the controller simply isn't fast enough.

## PI-Heuristic Transferability Check (Cross-Family)

Heuristics derived from one control family ("decrease `a` to speed up", "increase ωc to reject disturbance") do not transfer linearly across families. Verify the scaling law before applying:

| Control family | Key scaling | Inverse order | Implication |
|---|---|---|---|
| PI (PZC + Symmetric Optimum) | `ωc ∝ 1/(a · T_eq)` | linear | a=2 → 2× faster |
| **SMC** (PD-type sliding) | **`λ ∝ 1/(a² · T_eq)`** | **squared** | a=2 → **4×** faster (cascade time-scale separation may be violated) |
| DTC (cubic τ_max paradox) | non-monotonic | non-monotonic | a=4 may be slower than a=3 |

→ **Cross-family heuristics do not transfer directly.** Verify the scaling for each new method before borrowing intuition.

## Vdc / BEMF ≥ 1.5× Headroom Rule (FOC-Based)

Any FOC-based control (PI inner / SMC + PI cascade / MPC inner / any method that outputs `dq` voltage through SVPWM) must verify:

```
Vdc_min = 1.5 · √3 · ω_e_max · ψ_f
```

where `ω_e_max = ω_max_rpm · 2π/60 · Pn`. Tight headroom (< 1.2×) causes the inner loop to saturate against the voltage limit; saturation produces bang-bang waveforms that are easily misdiagnosed as control-law instability. See [pre_build_grid.md](pre_build_grid.md) for the function template.

DTC αβ hysteresis is exempt (no PI saturation concept; switching table directly selects voltage vectors).

## Phase 8 Pre-Flight Checklist (Brief Design)

Before writing a fresh-session generalization test brief, confirm the test case parameters are within the skill's default envelope:

| Parameter | Constraint |
|---|---|
| `J / B / Pn` | Within 2× of skill's default envelope |
| `iq_max` | ≥ 1.4 · iq_steady_at_TL_max (avoid starvation saturation) |
| `sim_time` | ≥ 5 · τ_max (from Phase 4.5 derivation) |
| `Vdc` | ≥ 1.5 · √3 · ω_e_max · ψ_f |
| `TL_max` and `TL_step_time` | `TL_step_time ≥ ramp_time + 5 · τ_inner` |

If a brief reports "scenario falls outside skill envelope," the fix is to adjust the **brief** (J, iq_max, sim_time), not to change the skill.

## Visual 4-Check (Mandatory Pre-Condition for Numerical Metrics)

Any 5-metric Mode A / Mode B scoring at acceptance time must be preceded by visual confirmation of these four signatures. If any check fails, the numerical metrics cannot be trusted — fix the implementation first.

| # | Check | Signal | PASS criterion | FAIL signature |
|---|---|---|---|---|
| 1 | Motor rotates | `wm` | Tracks `ω_ref`; not stuck near 0 or oscillating around stalled equilibrium | `wm` < 5% of `ω_ref` after settling |
| 2 | iq tracks reference | `iq_meas` vs `iq_ref` | Smooth tracking; not bang-bang at ±iq_max | `iq_meas` permanently at ±iq_max |
| 3 | abc AC sinusoidal | logger ch ia / ib / ic | AC sinusoidal at electrical frequency | abc DC-locked (single angle) — **F-CRIT signature: lab-frame open-loop** |
| 4 | Te energy balance | `Te` vs `TL + B·ω` | Steady-state `Te ≈ TL + B·ω` | `Te` deviates from `TL + B·ω` → energy imbalance, model-layer bug |

If any check fails:
1. Stop computing metrics
2. Open [broken_foc_diagnostics.md](broken_foc_diagnostics.md)
3. Fix the implementation
4. Re-run sim and re-check
