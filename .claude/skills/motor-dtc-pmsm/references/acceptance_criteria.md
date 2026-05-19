# DTC Acceptance Criteria

Two layers — pass both before declaring the build complete.

## Layer 1 — Build-Time Self-Tests

The build script must auto-run before declaring done:

- [ ] **T1**: `get_param(mdl, 'InitFcn')` returns ≥ 20 non-empty lines (DTC has 22 parameter assignments)
- [ ] **T2**: chart `Script` ≥ 50 lines (full DTC algorithm inline)
- [ ] **T3**: chart config — `ch.SampleTime == '-1'` AND `ch.ChartUpdate == 'INHERITED'`
- [ ] **T4**: self-contained reload + sim — close model, `clear all`, reload `.slx`, call `sim(mdl)`; must succeed
- [ ] **T5**: all 4 Scopes present — `find_system(mdl, 'BlockType', 'Scope')` ≥ 3 (`Scope_wm_RPM`, `Scope_Te`, `Scope_psi_mag`) AND `find_system(mdl, 'BlockType', 'XYGraph')` ≥ 1 (`Scope_psi_alphabeta`)
- [ ] Model runs from `.slx` double-click in fresh MATLAB session
- [ ] No Simulink warnings other than benign back-inheritance / rate-transition diagnostics

## Layer 2 — Behavior Verification

Two modes depending on whether a behavior baseline is available.

### Mode A — Baseline-Comparison

**Use when**: a `baseline_waveforms.mat` from a known-correct reference DTC simulation (same control method, same machine, same scenario) exists.

**Inputs**:

- Built model's `dtc_waveforms.mat` (19-channel logger per F-CRIT)
- User-provided `baseline_waveforms.mat` (must contain at least: `t`, `wm_rpm`, `Te`, `psi_mag`, `psi_alpha`, `psi_beta`, `iabc`)

**5 metrics**:

| # | Metric | Window | Pass condition |
|---|---|---|---|
| M1 | wm steady-state hit-rate | last 20% of `StopTime` (after settling) | `mean(\|wm_built − wm_baseline\| / ω_ref_target ≤ 0.05) ≥ 0.80` |
| M2 | Te ripple band hit-rate | last 20% of `StopTime` (steady-state) | `mean(\|Te_built − Te_baseline\| / T_max ≤ 0.05) ≥ 0.80`. ⚠️ Per Known Limitations (classical DTC ripple), M2 may fail and counts toward the 4/5 budget but does not by itself block Pass |
| M3 | `\|ψ_s\|` steady hit-rate | last 20% of `StopTime` | `mean(\|\|ψ_s\|_built − \|ψ_s\|_baseline\| / ψ_ref ≤ 0.05) ≥ 0.80` |
| M4 | ψ_α correlation | last electrical cycle in steady-state | `corrcoef(ψ_α_built, ψ_α_baseline) ≥ 0.90` (after time-align) |
| M5 | ψ_β correlation | last electrical cycle in steady-state | `corrcoef(ψ_β_built, ψ_β_baseline) ≥ 0.90` (after time-align) |

**Acceptance threshold**: ≥ 4/5 = 80% PASS. Classical DTC ripple may push M2 below threshold; 4/5 with M2 marginal is documented-acceptable.

### Mode B — Sanity-Check (no baseline)

**Use when**: no baseline exists for the same control method (e.g., generalizing DTC to a new machine where no prior DTC reference exists).

**Inputs**:

- Built model's `dtc_waveforms.mat` (only)
- User-supplied scenario: `omega_ref_rpm`, `TL_step_t`, `TL_after`, `psi_ref`, `Pn`, `Tsc`, `fs_max`, `T_eq`, `a_so`

**5 physical-constraint metrics** (DTC-specific):

| # | Metric | Window | Pass condition |
|---|---|---|---|
| S1 | wm steady tracking | last 20% of `StopTime` (after settling, post-load) | `\|mean(wm_built_rpm) − omega_ref_rpm\| < 0.05 · omega_ref_rpm` |
| S2 | wm post-load recovery | from `TL_step_t + 50 ms` to `StopTime` | wm reaches `≥ 0.95 · omega_ref_rpm` within `t_recovery_window = max(20·T_eq, 0.05)` seconds AND never collapses to `< 0.50 · omega_ref_rpm` during dip |
| S3 | `\|ψ_s\|` steady tracking | last 20% of `StopTime` | `\|mean(\|ψ_s\|_built) − psi_ref\| < 0.05 · psi_ref` (within 5%). DTC-specific — replaces FCS-MPC's iq saturation. Failure most likely = 8-state on PMSM ([crit_conditions.md §A-CRIT](crit_conditions.md)) or `psi_ref < \|ψ_s\|_load` ([crit_conditions.md §B-CRIT](crit_conditions.md)) |
| S4 | sector coverage + net θ_ψ rotation | last 20% of `StopTime` (post-load, steady-state) | (a) `unique(sector_built) ⊇ {1,2,3,4,5,6}`; (b) `theta_unwrapped = unwrap(atan2(ψ_β, ψ_α))`; require `\|theta_unwrapped(end) − theta_unwrapped(1)\| ≥ 4π` (≥ 2 electrical revs); (c) average rotation rate matches expected: `\|mean(diff(theta_unwrapped)) − sign(omega_ref_rpm) · omega_e · Tsc\| < 0.10 · \|omega_e · Tsc\|`. **Per-sample chatter is expected**: hysteresis dead-band naturally produces ~30% samples with `diff(theta) < 0`; what matters is **net direction and average rate**, not per-sample monotonicity |
| S5 | numerical sanity | full duration | (a) no NaN / no Inf; (b) `max\|ψ_s\| < 1.5 · psi_ref` (no divergence); (c) approximate switching frequency cap: `count(diff(V_k) ≠ 0) / sim_time < 3 · fs_max`; (d) `\|mean(iabc)\| < 0.5 A` (no DC offset); (e) wm no monotonic drift after settling |

**Acceptance threshold**: ≥ 4/5 = 80% PASS. Physical-constraint Pass is stricter than baseline overlay (must satisfy engineering reality, not just match a reference); 80% threshold compensates for absence of a known-correct reference.

### Below Threshold (Either Mode)

Log per-metric diagnostic, escalate to user. Do NOT silently patch numbers.

## Implementation Note

A reusable overlay/sanity script is **not** provided in the skill because per-scenario windows differ (`TL_step_t`, `StopTime`, `T_eq`, `fs_max`). Write a 30–60-line evaluation script per acceptance run.

## Visual 4-Check (Pre-Condition)

Before computing 5-metric scores in either mode, pass the [base/sanity_visual.md](../../motor-pmsm-base/references/sanity_visual.md) Visual 4-Check (motor rotates / iq tracks / abc AC sinusoidal / Te energy balance). Numerical metrics are not trustworthy if any visual check fails — fix the implementation first. The αβ XY phase plot (`Scope_psi_alphabeta`) is a 5th DTC-specific visual check.
