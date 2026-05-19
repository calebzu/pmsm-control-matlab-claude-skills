# FCS-MPC Algorithm Pseudocode

One control period @ Tsc. Embed this into the chart Script via `sprintf` (see [crit_conditions.md §K-CRIT](crit_conditions.md)).

## Pseudocode

```
# 1. Sample (chart inputs already ZOH'd @ Tsc by upstream — see A-CRIT)
ia, ib, ic, w, iq_ref, id_ref, Vdc ← chart inputs

# 2. Electrical angle integration (persistent var, step assumes Tsc)
we ← Pn · w
theta_e ← theta_e + Tsc · we
theta_e ← atan2(sin(theta_e), cos(theta_e))   # wrap to [-pi, pi]

# 3. Clarke (amplitude-invariant 2/3) + Park (original)
i_alpha ← (2/3) · (ia − 0.5·ib − 0.5·ic)
i_beta  ← (2/3) · (sqrt(3)/2) · (ib − ic)
id ←  i_alpha · cos(theta_e) + i_beta · sin(theta_e)
iq ← -i_alpha · sin(theta_e) + i_beta · cos(theta_e)

# 4. Enumerate candidate vectors
#    7-vector form (V7 deduplicated to V0=[0 0 0]; ~12% loop-count savings)
#    OR equivalently 8-vector form (V7=[1 1 1] explicit, no dedup) — same minimum cost
states = [
    [0 0 0],   # V0 (zero)
    [1 0 0],   # V1
    [1 1 0],   # V2
    [0 1 0],   # V3
    [0 1 1],   # V4
    [0 0 1],   # V5
    [1 0 1],   # V6
]

best_cost ← +inf
for k in 1..7:
    Sa, Sb, Sc ← states[k]
    # Phase voltages (referenced to neutral)
    Va ← Vdc · (2·Sa − Sb − Sc) / 3
    Vb ← Vdc · (-Sa + 2·Sb − Sc) / 3
    Vc ← Vdc · (-Sa − Sb + 2·Sc) / 3
    # Clarke + Park on voltage
    Va_a ← (2/3) · (Va − 0.5·Vb − 0.5·Vc)
    Vb_b ← (2/3) · (sqrt(3)/2) · (Vb − Vc)
    Vd ←  Va_a · cos(theta_e) + Vb_b · sin(theta_e)
    Vq ← -Va_a · sin(theta_e) + Vb_b · cos(theta_e)
    # Forward Euler one-step prediction
    id_p ← (1 − Tsc·Rs/Ld) · id + (we·Tsc·Lq/Ld) · iq + (Tsc/Ld) · Vd
    iq_p ← (1 − Tsc·Rs/Lq) · iq − (we·Tsc·Ld/Lq) · id + (Tsc/Lq) · Vq − (we·flux·Tsc/Lq)
    # Cost
    cost ← lambda_d · (id_ref − id_p)² + lambda_q · (iq_ref − iq_p)²
    if cost < best_cost:
        best_cost, best_idx ← cost, k

# 5. Output 6-bit gate (pair-adjacent: Sa_up Sa_dn Sb_up Sb_dn Sc_up Sc_dn)
Sa, Sb, Sc ← states[best_idx]
gate ← [Sa, 1−Sa, Sb, 1−Sb, Sc, 1−Sc]
```

## Cost Function Weight Selection

The `λ_q : λ_d` ratio implicitly assumes `iq_ref` is bounded to the same order as `id_ref` (this is what D-CRIT PI saturation enforces). Choose ratio per control objective:

| Objective | Range | Notes |
|---|---|---|
| Ripple-priority | `λ_q/λ_d ∈ [5, 10]` | Low current ripple; switching-frequency-sensitive scenarios |
| Balanced (default) | `λ_q/λ_d ∈ [10, 30]` | General servo; common starting point: `λ_d=1, λ_q=20` |
| Torque-priority | `λ_q/λ_d ∈ [50, 100]` | High-precision torque tracking; transient-critical |
| Symmetric `1:1` | (research) | q-axis tracking suffers; useful for ripple research |

Do not silently inherit the `20:1` starting point — confirm the chosen objective matches user's requirement.

## Control Period Discretization

Forward Euler one-step is the default (Phase 9 self-test verifies). Backward Euler / Tustin / multi-step horizon are out of skill scope. See [design_decisions.md](design_decisions.md) D08.

## Theta_e Source

Integrate `Pn · w` with persistent var inside the chart. Do NOT use the PMSM bus `theta` output (subject to bus consistency caveats across R2024b configurations). See [design_decisions.md](design_decisions.md) D13.

## Audit

After build, open the `.slx`, double-click the chart, view its Script. The numeric literals for `Rs/Ld/Lq/ψf/Pn/Tsc` must match `get_param(mdl, 'InitFcn')` digit-for-digit (K-CRIT). The full Script text length should be ≥ 30 lines (Phase 9 self-test 2).
