# SMC Acceptance Criteria

Two layers — pass both before declaring the build complete.

## Layer 1 — Build-Time Self-Tests

The build script must auto-run before declaring done:

- [ ] **T1**: `get_param(mdl, 'InitFcn')` returns ≥ 20 non-empty lines
- [ ] **T2**: All 4 Scopes present — `find_system(mdl, 'BlockType', 'Scope')` returns ≥ 4 blocks
- [ ] **T3**: `Goto_The` published with global visibility (G-CRIT) — `find_system(mdl, 'BlockType', 'Goto')` returns block with `GotoTag='The'` AND `TagVisibility='global'`
- [ ] **T4**: Anti_Park + SVPWM_blk blocks present
- [ ] **T5**: Self-contained reload — close model, `clear all`, reload `.slx`, call `sim(mdl)`; must succeed
- [ ] **T6**: Lyapunov gain bounds satisfied — `K1_sta > 1.5·sqrt(M)` AND `K2_sta > 1.1·M` where `M = (TL_max + B·ω_max)/J`

## Layer 2 — Mode B Sanity-Check (5 SMC-specific metrics)

Use when no behavior baseline exists for the same control method (typical for SMC generalization to a new machine). Verify against physical constraints alone.

| # | Metric | Pass threshold (v1 baseline) |
|---|---|---|
| S1 | wm tracking | `wm_steady ≥ 90% of wref_rad` (sliding-phase, after ramp + 4·λ_pd) |
| S2 | TL recovery | wm returns within 5% of pre-step within `5·λ_pd` post `TL_step` |
| S3 | sliding surface | `mean(|s|)` after sliding-phase < 5 rad/s; `max|s|` < 50. Per-sample chatter expected; what matters is mean and max bounds |
| S4 | iq tracking + abc sinusoidal | `id_meas` mean near 0 (no FOC degradation); `ia` AC sinusoidal at electrical frequency (NOT DC-locked) |
| S5 | Te energy balance | `Te_steady ≈ TL + B·ω_steady` within ±10% |

**Acceptance threshold**: ≥ 4/5 PASS. S3 sliding-phase per-sample chatter is expected physics — `|s|` mean within bound is what matters; per-sample chatter does not block Pass.

## Phase 8 4-Gate Sequence (Production Promote)

| Gate | Who | Action |
|---|---|---|
| **G1 Numerical PASS** | fresh subagent | Mode B 4/5 (with S3 marginal noted as expected) on a fresh machine + Lyapunov gain assertion + Layer 1 6/6 |
| **G2 Reverse-leak audit** | main session | grep clean for any project-internal references that should not be in the skill (paths to private reference models, internal plan files, etc.) |
| **G3 Doc precision review** | main session | SKILL.md wording / metric thresholds / G-CRIT and H-CRIT visibility assertion language unambiguous |
| **G4 User visual review** ⭐ | user | Open `.slx` on MATLAB desktop, run sim, **inspect 4 Scopes AND verify abc currents are AC sinusoidal (not DC-locked)**, confirm basic control achieved |

⭐ **G4 cannot be skipped**. Visual inspection catches silent FOC failures (e.g., G-CRIT violation) that numerical metrics may miss. Specifically, the user must verify the four Visual 4-check signatures (motor rotates / iq tracks / abc AC sinusoidal NOT DC-locked / Te energy balance — see base/[sanity_visual.md](../../motor-pmsm-base/references/sanity_visual.md)).

## Below Threshold

If Layer 1 fails: fix and re-run; do NOT declare the build complete with failing self-tests.
If Layer 2 fails: log per-metric diagnostic, escalate to user; do NOT silently patch numbers.

## Implementation Note

A reusable overlay/sanity script is **not** provided in the skill because per-scenario windows differ (`TL_step_t`, `sim_time`, `λ_pd`, `ramp_time`). Write a 30–60-line evaluation script per acceptance run.

## Visual 4-Check (Pre-Condition)

Before computing 5-metric scores, pass the [base/sanity_visual.md](../../motor-pmsm-base/references/sanity_visual.md) Visual 4-Check (motor rotates / iq tracks / abc AC sinusoidal / Te energy balance). Numerical metrics are not trustworthy if any visual check fails — fix the implementation first.

For SMC specifically, an additional **5th visual check** is the sliding surface trajectory in `Scope_s`: `|s|` should reach near-zero in `t ≈ 4·λ_pd` then stay bounded. Per-sample sliding-phase chatter is expected physics, not failure.
