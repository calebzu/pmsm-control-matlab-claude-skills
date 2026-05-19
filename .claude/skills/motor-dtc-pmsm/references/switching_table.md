# Switching Table — Sutikno 6-State (Table 2)

The switching table maps `(C_ψ, C_T, sector)` → voltage vector `V_k`. PMSM **must** use the 6-state Sutikno 2011 Table 2 (no zero vectors). 8-state Takahashi 1986 Table 1 (with V0/V7 zero vectors) causes flux to decay in PMSM steady state.

## Sutikno 6-State Table 2

Rows = `(C_ψ, C_T)`, columns = sector 1–6. Each cell holds one of `{V1, V2, V3, V4, V5, V6}` (no V0 or V7).

| (C_ψ, C_T) | Sec 1 | Sec 2 | Sec 3 | Sec 4 | Sec 5 | Sec 6 | Physical effect |
|---|---|---|---|---|---|---|---|
| (1, 1) | V2 | V3 | V4 | V5 | V6 | V1 | flux↑ + Te↑ (forward) |
| (1, 0) | V6 | V1 | V2 | V3 | V4 | V5 | flux↑ + Te↓ (backward) |
| (0, 1) | V3 | V4 | V5 | V6 | V1 | V2 | flux↓ + Te↑ (forward) |
| (0, 0) | V5 | V6 | V1 | V2 | V3 | V4 | flux↓ + Te↓ (backward) |

## Voltage Vector Numbering (V_k Standard)

| V_k | [Sa Sb Sc] | Angle |
|---|---|---|
| V1 | [1 0 0] | 0° |
| V2 | [1 1 0] | 60° |
| V3 | [0 1 0] | 120° |
| V4 | [0 1 1] | 180° |
| V5 | [0 0 1] | 240° |
| V6 | [1 0 1] | 300° |
| V0 | [0 0 0] | (zero — not used in 6-state) |
| V7 | [1 1 1] | (zero — not used in 6-state) |

## Sector Detection

Convention A (Sec 1 = `[0°, 60°)`, V1 at α-axis start):

```
theta_psi = atan2(psi_beta, psi_alpha)
sector = floor(mod(theta_psi + 2*pi, 2*pi) / (pi/3)) + 1   % 1..6
```

## Hysteresis State Memory

Both `C_ψ` and `C_T` are 2-level with **state memory** (Schmitt-trigger semantics):

```
% Flux hysteresis
E_psi = psi_ref - mag_psi
if      E_psi >  HB_psi/2,   C_psi = 1
elseif  E_psi < -HB_psi/2,   C_psi = 0
else                          C_psi = C_psi_prev   % retain previous in dead band

% Torque hysteresis
E_T = Te_ref - Te_meas
if      E_T >  HB_T/2,    C_T = 1
elseif  E_T < -HB_T/2,    C_T = 0
else                       C_T = CT_prev

% Update persistent state for next sample
C_psi_prev = C_psi
CT_prev    = C_T
```

State memory is essential — without it, hysteresis becomes a comparator that toggles every sample around the dead-band edge.

## V_k → Gate Conversion

```
% V_k is 1..6 → look up [Sa Sb Sc] from table, output 6×1 column
S = vector_table(V_k);       % 1×3 row
gate = [S(1); 1-S(1); S(2); 1-S(2); S(3); 1-S(3)];
% pair-adjacent: [Sa_up; Sa_dn; Sb_up; Sb_dn; Sc_up; Sc_dn]
```

The 6×1 column format matches the Universal Bridge `Arms=3` Inport pin order.

## 8-State Comparison (do NOT use for PMSM)

For reference only. The 8-state Takahashi 1986 Table 1 has these `(C_ψ, C_T)` rows that map to V0/V7:

```
(1, 0): V0 (in some sectors)
(0, 0): V7 (in some sectors)
```

In PMSM steady state, `(C_T = 0)` occurs frequently as Te toggles around `Te_ref`. The zero-vector cells dominate (>70% of samples in some sub-types) → flux decays steadily because:

```
during V0/V7:  u_alpha = u_beta = 0
               d psi_s / dt = -Rs · i_s   (decay-only, no growth)
```

PMSM has no induction-motor-style slip-frequency mechanism to rebuild flux during zero-vector intervals. The 6-state table avoids this by always selecting active vectors.
