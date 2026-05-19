# Cross-Decoupling Feedforward (Phase 4.5)

Critical for proper FOC under load. Without FF, the current PI must compensate BEMF (`ψ_f · ω_e`) and cross-coupling (`Lq · ω_e · iq`, `Ld · ω_e · id`) entirely via integral wind-up → saturation → tracking degrades.

## Block Sequence

| # | Block | Configuration |
|---|---|---|
| 1 | `Gain_Pn_omega` (`Gain`) | `Gain='Pn'`. Input = `BusSel/4 (ω_m, rad/s)`, output = `ω_e` (rad/s). ⭐ **Independent block** from `Gain_Pn` (which gives `θ_e` for `Goto_The`) |
| 2 | `Mux_FF_d` (`Mux`, 2 inputs) | Port 1 = `Gain_Pn_omega/1 (ω_e)`, Port 2 = `BusSel/7 (iq_meas)`. ⭐ **Port 1 must be ω_e (rad/s), NOT θ_e (rad)** (H-CRIT) |
| 3 | `Fcn_Vd_ff` (`Fcn`) | `Expr='-Lq*u(1)*u(2)'`. Output = `-Lq · ω_e · iq` (cross-coupling FF for d-axis) |
| 4 | `Mux_FF_q` (`Mux`, 2 inputs) | Port 1 = `Gain_Pn_omega/1 (ω_e)`, Port 2 = `BusSel/8 (id_meas)` |
| 5 | `Fcn_Vq_ff` (`Fcn`) | `Expr='Ld*u(1)*u(2) + psi_f*u(1)'`. Output = `Ld · ω_e · id + ψ_f · ω_e` (BEMF + cross-coupling FF for q-axis) |
| 6 | `Sum_Vd_total` | `Inputs='++'`. `Vd_total = PID_id_output + Vd_ff` |
| 7 | `Sum_Vq_total` | `Inputs='++'`. `Vq_total = PID_iq_output + Vq_ff` |

## Why ω_e and Not θ_e (H-CRIT)

The FF formulas

```
Vd_ff = -Lq · ω_e · iq_meas
Vq_ff =  Ld · ω_e · id_meas + ψ_f · ω_e
```

require electrical **angular velocity** `ω_e (rad/s)`. Routing `θ_e (rad, position)` into the FF Mux produces `Vq_ff = θ_e · ψ_f` instead of `ω_e · ψ_f`.

Numerical comparison (typical operating point):

```
ω_m = 200 rad/s, Pn = 4 → ω_e = 800 rad/s
θ_e ≤ 6.28 rad (modulo 2π)
Magnitude error ratio ≈ 127×
```

→ BEMF compensation collapses to position-proportional. PI integrator winds up trying to compensate. Saturation. Tracking degrades.

## Why a Separate `Gain_Pn_omega` Block

Two distinct quantities are needed at different points:

- `Gain_Pn` (input: `BusSel/6 (θ_m)`) → output `θ_e` → routed to `Goto_The` (G-CRIT) for Anti_Park
- `Gain_Pn_omega` (input: `BusSel/4 (ω_m)`) → output `ω_e` → routed to FF Mux

Reusing `Gain_Pn` for FF (route θ_e into FF Mux) is the H-CRIT bug. Use two independent Gain blocks.

## Self-Test

```matlab
% Verify Gain_Pn_omega block exists
omega_blk = find_system(mdl, 'BlockType', 'Gain', 'Name', 'Gain_Pn_omega');
assert(~isempty(omega_blk), 'H-CRIT: Gain_Pn_omega block missing');

% Verify FF Mux input wiring source
% (This is harder to verify programmatically; rely on visual review at G4)
```

## Diagnostic

If `id_meas` oscillates large under load (instead of small near 0) AND `Vq` command shows a position-modulated waveform (sawtooth at ω_e frequency), first suspect = FF Mux input dimensional swap.

## After Phase 4.5

`Sum_Vd_total / Sum_Vq_total` outputs feed into Phase 5 modulation:

- `Sum_Vq_total/1 → Anti_Park/1` (Vq port)
- `Sum_Vd_total/1 → Anti_Park/2` (Vd port)
- `Anti_Park/1 → SVPWM_blk/1` (Vα)
- `Anti_Park/2 → SVPWM_blk/2` (Vβ)
- `SVPWM_blk/1 → Universal_Bridge/1` (gate)

See [svpwm_workaround.md](svpwm_workaround.md) for the SVPWM library block sector=7 startup fix.
