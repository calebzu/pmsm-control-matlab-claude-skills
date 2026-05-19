# Acceptance Criteria

Two layers — pass both before declaring the build complete.

## Layer 1 — Build-Time Self-Tests

The build script (`build_template.m`) must auto-run these self-tests before declaring done:

- [ ] **T1**: `get_param(mdl, 'InitFcn')` returns ≥ 5 non-empty lines
- [ ] **T2**: chart `Script` ≥ 30 lines (full algorithm inline, not 1-line wrapper)
- [ ] **T3**: chart config — `ch.SampleTime == '-1'` AND `ch.ChartUpdate == 'INHERITED'`
- [ ] **T4**: self-contained reload + sim — close model, `clear all`, reload `.slx`, call `sim(mdl)`; must succeed
- [ ] **T5**: reference signal sanity — sample `ω_ref` at three time points (e.g., 25%, 50%, 100% of `ramp_time`); verify linear ramp
- [ ] Model runs from `.slx` double-click in fresh MATLAB session (J-CRIT)
- [ ] No Simulink warnings other than benign back-inheritance / rate-transition diagnostics

If any self-test fails, fix and re-run. Do not declare the build complete with failing tests.

## Layer 2 — Behavior Verification

Two modes depending on whether a behavior baseline is available.

### Mode A — Baseline-Comparison

**Use when**: a `baseline_waveforms.mat` from a known-correct reference (same control method, same machine, same scenario) exists. Typical: reproducing a textbook or published-paper FCS-MPC simulation.

**Inputs**:

- Built model's `fcs_mpc_waveforms.mat`
- User-provided `baseline_waveforms.mat` (must contain at least: `t`, `wm_rpm`, `Te`, `id`, `iq`, `iabc`)

**5 metrics**:

| # | Metric | Window | Pass condition |
|---|---|---|---|
| M1 | wm steady-state | last 20% of `StopTime` | `\|mean(wm_built) − mean(wm_baseline)\| / ω_ref_target < 0.05` |
| M2 | wm post-load recovery | from `TL_step_time + 50 ms` to `StopTime` | sustained tracking after disturbance |
| M3 | iq peak (transient) | full duration | `\|max\|iq_built\| − max\|iq_baseline\|\| / max\|iq_baseline\| < 0.15` |
| M4 | id steady | last 20% of `StopTime` | `\|mean(id_built) − id_ref\| < 0.5` AND deviation from baseline < 1 A |
| M5 | iabc shape (RMS) | one full electrical cycle in steady-state | per-phase RMS within 10% of baseline |

**Acceptance threshold**: ≥ 5/5 PASS (95%).

### Mode B — Sanity-Check (no baseline)

**Use when**: no behavior baseline exists for the same control method (e.g., generalizing FCS-MPC to a new machine where no prior FCS-MPC reference exists). Borrow only **machine parameters** and **scenario timing** from a different-method reference; verify behavior against physical constraints alone.

**Inputs**:

- Built model's `fcs_mpc_waveforms.mat`
- User-supplied scenario: `ω_ref_target` (RPM), `TL_step_time`, `TL_value`, `iq_max`, `id_ref`, `Pn`, `flux`

**5 physical-constraint metrics**:

| # | Metric | Window | Pass condition |
|---|---|---|---|
| S1 | wm tracking | last 20% of `StopTime` (after settling, post-load) | `\|mean(wm_built) − ω_ref_target\| < 0.05 · ω_ref_target` |
| S2 | wm post-load recovery | from `TL_step_time` to `StopTime` | wm reaches `≥ 0.95 · ω_ref_target` within `5 / inner_loop_BW_Hz` seconds; never collapses to < 50% of `ω_ref_target` |
| S3 | iq saturation | full duration | `max\|iq_built\| ≤ iq_max · 1.15` AND `mean\|iq_steady\| ≥ TL_value / (1.5·Pn·flux) · 0.9` |
| S4 | id steady | last 20% of `StopTime` | `\|mean(id_built) − id_ref\| < 0.5` A |
| S5 | numerical sanity | full duration | no NaN / no Inf; iabc waveform sinusoidal with no DC offset (`\|mean(iabc)\| < 0.5 A`); no wm divergence after settling |

**S3 note**: the 1.15× margin (vs naive 1.05×) accommodates FCS-MPC single-step prediction ripple, which is intrinsic at typical 10–12% of `iq_max` and not removable without delay compensation (D08) or multi-step horizon (D09).

**Acceptance threshold**: ≥ 4/5 = 80% PASS (one metric may be marginal). Physical-constraint Pass is stricter than baseline overlay (must satisfy engineering reality, not just match a reference); the 80% threshold compensates for the absence of a known-correct reference.

### Below Threshold (Either Mode)

Log per-metric diagnostic, escalate to user. Do **not** silently patch numbers.

## Implementation Note

A reusable overlay/sanity script is **not** provided in the skill because per-scenario windows differ (`TL_step_time`, `StopTime`, `inner_loop_BW`). Write a 30–60-line evaluation script per acceptance run.

## Visual 4-Check (Pre-Condition)

Before computing 5-metric scores in either mode, pass the [base/sanity_visual.md](../../motor-pmsm-base/references/sanity_visual.md) Visual 4-Check (motor rotates / iq tracks / abc AC sinusoidal / Te energy balance). Numerical metrics are not trustworthy if any visual check fails — fix the implementation first.
