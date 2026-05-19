# Outer PI Saturation (D-CRIT) and `id` Drift Diagnostic

## Mandatory Configuration

Outer-loop speed PI must include:

```matlab
'Controller',              'PI',
'P',                       'Kp_w',
'I',                       'Ki_w',
'IntegratorMethod',        'Forward Euler',
'SampleTime',              'Tsc',
'LimitOutput',             'on',                  % MANDATORY
'UpperSaturationLimit',    'iq_max',
'LowerSaturationLimit',   '-iq_max',
```

Sizing constraint: `1.5 · Pn · ψf · iq_max ≥ 1.3 · TL_max`.

The `1.3×` headroom (vs. naive `1.05×`) prevents borderline saturation under transients. Tighter than `1.3×` is risky.

PI gains: recommend computing via `pi_design.m` from `(J, Kt, T_eq, a)` per Symmetric Optimum (B=0 case) rather than hand-tuning. Hand-tuned `(wn, ζ)` 2nd-order standard form is supported but emits a warning — it ignores inner-loop dynamics. See `shared/formulas/pmsm_formulas.md §A` for 4-method comparison + decision tree.

## Anti-Windup

Recommended in production. Basic builds may omit; the retreat-from-saturation overshoot is a controlled secondary problem. The primary problem (cost-function collapse from runaway `iq_ref`) is what mandatory output saturation prevents.

## `id` Drift Diagnostic — Two Branches

If `|mean(id_steady) − id_ref| > 0.5 A` (drift, not ripple), check branches in order:

### Branch 1 — PI Saturation Root Cause (SPMSM or low/medium speed)

**Applicability**: `Lq/Ld < 1.3` (SPMSM or mild IPMSM) **OR** `we < 100 rad/s` (low-speed regime where cross-coupling is negligible).

**Suspect**: PI saturation missing, or `iq_max` set too high, allowing runaway `iq_ref` to collapse the cost-weight ratio. MPC abandons d-axis tracking to chase the runaway `iq_ref`.

**Verify**:

```matlab
get_param(<PI block>, 'LimitOutput')  % should be 'on'
get_param(<PI block>, 'UpperSaturationLimit')  % should be 'iq_max'
% iq_max should be sized via 1.5·Pn·ψf·iq_max slightly above TL_max (1.3-1.5× headroom)
```

**Fix**: enable `LimitOutput='on'` with sized `iq_max`, OR resize `iq_max` smaller (do NOT just raise `iq_max` — that defeats saturation).

### Branch 2 — IPMSM Cross-Coupling at Mid/High Speed

**Applicability**: `Lq/Ld > 1.3` (IPMSM with meaningful saliency) **AND** `we > 300 rad/s` (mid-to-high speed) **AND** PI saturation correctly configured per Branch 1 verify.

**Suspect**: cross-coupling term `(we·Tsc·Ld/Lq) · id` in the `iq_pred` Forward Euler equation. At high `we` with significant `Lq/Ld`, this term per Tsc step becomes comparable in magnitude to `λ_q · (iq_ref − iq_pred)²` in the cost. The optimizer trades off d-axis tracking to minimize the dominant q-axis cost — even when PI saturation is healthy. The cost weight ratio cannot suppress this physical coupling; it is a structural limitation of single-step FCS-MPC on IPMSM at speed.

**Numerical signature**: use `|mean(id) − id_ref|` (drift component only), NOT `mean|id|` (which mixes ripple + drift and silently rejects healthy FCS-MPC controllers). Branch 2 trigger requires both:

- `|mean(id) − id_ref| > 0.3 A`
- ripple-vs-drift ratio `|mean(id)| / mean|id| > 0.4` (drift-dominated, not ripple-dominated)

A ratio < 0.4 means ripple-dominated = healthy FCS-MPC behavior, NOT a Branch 2 case.

Branch 2 does NOT respond to changing `iq_max` or PI bandwidth — this is the key distinguisher from Branch 1.

**Mitigation** (in priority order):

- (a) **Raise `λ_d` to 5–10** — increases d-axis tracking budget; trade-off: q-axis ripple increases ~10–20%
- (b) **Explicit decoupling feedforward** — add `Vd_ff = -we·Lq·iq` and `Vq_ff = we·Ld·id + we·flux` to the Vd/Vq passed to the cost; decouples physically rather than via cost weights (out of skill scope; consider for IPMSM strong-saliency)
- (c) **MTPA `id_ref ≠ 0`** — at high speed IPMSM should run at MTPA-optimal id, which makes `id_ref` bias the cross-coupling baseline (out of skill scope; ask user for `id_ref_mtpa`)

### If Neither Branch Applies

Open an issue, collect:

```
(Lq/Ld, we_steady, mean|id|, mean|iq|, λ_d/λ_q, iq_max, PI saturation status)
```

Escalate to user. Do not silently patch numbers.
