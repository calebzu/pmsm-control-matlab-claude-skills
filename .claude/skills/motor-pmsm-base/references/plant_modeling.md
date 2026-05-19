# PMSM Plant Modeling

## Frame Conventions (Mandatory)

- **dq frame**: amplitude-invariant (NOT power-invariant). All `motor-pmsm-base`-derived skills must use amplitude-invariant for cross-method comparability.
- **Park transform**:  `[id; iq]   = [cos θ_e, sin θ_e; -sin θ_e, cos θ_e] · [iα; iβ]`
- **Clarke transform**: `[iα; iβ]   = (2/3) · [1, -1/2, -1/2; 0, √3/2, -√3/2] · [ia; ib; ic]`  (factor **2/3**, NOT √(2/3))
- **Anti-Park (inverse)**: `[uα; uβ] = [cos θ_e, -sin θ_e; sin θ_e, cos θ_e] · [ud; uq]`
- **Anti-Clarke** (amplitude-invariant): `ia = iα`, `ib = -0.5·iα + √3/2·iβ`, `ic = -0.5·iα - √3/2·iβ`

## PMSM dq Voltage Equations (Signed)

The 8 plant equations are signed off in `shared/formulas/pmsm_formulas.md #0–#7`. **Do not copy their content into a build script** — reference by index. Listed here for context:

- `ud = Rs·id + Ld·d(id)/dt − ω_e·Lq·iq`     (#0)
- `uq = Rs·iq + Lq·d(iq)/dt + ω_e·(Ld·id + ψ_f)`  (#1)
- `Te = (3/2)·Pn·[(Ld − Lq)·id·iq + ψ_f·iq]`  (#4)
- `J·d(ω_m)/dt + B·ω_m = Te − TL`   (#5)
- `ω_e = Pn · ω_m`,   `θ_e = Pn · θ_m`  (#6, #7)

Any new PMSM method's Phase 1.5 must reference these existing signed formulas; only **new** equations specific to the method need fresh sign-off.

## PMSM Block Selection: SimPowerSystems vs Custom

| Option | Pros | Cons | Recommended |
|---|---|---|---|
| **R2024b SPS PMSM block** | dq + abc bus simultaneously; `BusSel/7 (iqs)` and `BusSel/8 (ids)` give consistent internal dq; directly connects to `Universal_Bridge` | Black box — internal Park convention must be verified (amplitude vs power invariant); mass parameter is `MotorMass`, not `J` directly — read mask carefully | ✅ Default for v1 baseline (saves ~30 min) |
| Custom Simulink (Integrator + Sum + Gain) | Transparent — every equation visible, easy to debug | 11+ blocks + 20+ wires per build | ⚠️ Only when SPS PMSM block cannot cover an edge case (rare) |

When using the SPS PMSM block, **feedback signals must come from the bus** (`BusSel/7`, `BusSel/8`) — NOT from an external Park transform of measured `abc` currents. The two diverge by 20A+ during transients due to different convention assumptions; only the internal dq matches the block's internal state.

## InitFcn Injection (One-Click Reproducibility)

All PMSM parameters (Rs / Ld / Lq / ψ_f / Pn / J / B / Vdc / Tsc / iq_max / TL_max / controller gains) must be injected via the model's `InitFcn` property — NOT via `assignin` from an external script:

```matlab
init_str = sprintf([ ...
  'Rs = %g; Ld = %g; Lq = %g; psi_f = %g; ', ...
  'Pn = %d; J = %g; B = %g; Vdc = %g; Tsc = %g; ', ...
  'iq_max = %g; TL_max = %g;'], ...
  p.Rs, p.Ld, p.Lq, p.psi_f, ...
  p.Pn, p.J, p.B, p.Vdc, p.Tsc, ...
  p.iq_max, p.TL_max);
set_param(mdl, 'InitFcn', init_str);
```

**Why**: a downstream user (or fresh agent session) must be able to double-click the `.slx` and Run directly. Requiring "first run build script then run sim" breaks the standalone reproducibility contract.

Common `InitFcn` size: ≥ 10 lines for any non-trivial PMSM build (validated by Phase 9 self-test `count(init, ';') >= 10`).
