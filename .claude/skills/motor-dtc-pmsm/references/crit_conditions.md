# DTC CRIT Conditions

Six non-negotiable conditions. Each is a known silent-failure mode.

## A-CRIT — 6-State Default (PMSM Must)

`switching_table_mode = '6state'` (Sutikno 2011 Table 2) is mandatory for PMSM. **Reasoning**: PMSM in steady state has small `E_T` ripple → `C_T` toggles 0/1 in a tight band. Whenever `(C_ψ, C_T) = (1, 0)` or `(0, 0)`, the 8-state Takahashi Table 1 maps to `V0` or `V7` (zero vectors). Zero vectors give `u_α = u_β = 0` → flux dynamics `dψ/dt = -Rs·i` is **decay-only**. PMSM cannot rebuild flux to `ψ_ref` between active-vector pulses → flux collapses.

**Diagnostic**: if `|ψ_s|` decays toward 0 in steady state, or the αβ XY plot shows a hexagonal inner pattern instead of a circle of radius `ψ_ref`, first suspect = 8-state default leaked in. Verify the chart's table is the 4×6 Sutikno Table 2 (see [switching_table.md](switching_table.md)).

8-state remains available as `switching_table_mode='8state'` only for IM-DTC educational comparison.

## B-CRIT — `ψ_ref` Selection (No `ψ_f` Default)

Default `psi_ref` must come from one of (in priority order):

1. **Reference model value** if supplied: copy as-is
2. **MTPA load operating point**: `|ψ_s|_load = sqrt(ψ_f² + (Lq · iq_max)²)`
3. **SPMSM light-load only**: `ψ_f` is acceptable as a lower bound, but verify load doesn't push `|ψ_s|_load` above `ψ_f + small margin`

⛔ **Never default `ψ_ref = ψ_f` for IPMSM**. IPMSM has `Lq >> Ld`; MTPA gives `|ψ_s|_load = sqrt(ψ_f² + (Lq·iq)²) >> ψ_f`. If `ψ_ref = ψ_f`, then `E_ψ = ψ_ref − |ψ_s| < 0` always → `C_ψ = 0` always → switching table only selects vectors that decrease flux → motor cannot reach demanded torque, **reverses direction**.

**Diagnostic**: if wm reverses (negative steady-state) under positive `ω_ref` and positive load, first suspect = `ψ_ref` set below `|ψ_s|_load`. Verify `ψ_ref ≥ sqrt(ψ_f² + (Lq · iq_actual)²)`.

## C-CRIT — `T_eq_factor` Selection

Speed PI design via `pi_design('SO', J, Kt, T_eq, a_so)` requires the **inner-loop equivalent time constant** `T_eq`. For DTC, `T_eq = 15 · Tsc` is the default; for FCS-MPC, `T_eq = 5 · Tsc`. They differ because the inner loops are physically different. **DTC must pass `Kt = 1`** (outer PI outputs `Te_ref` [N·m], plant has no Kt); FCS-MPC passes `Kt = 1.5·Pn·ψf`. The unified 5-arg signature (since v1.0.2) keeps both skills' `pi_design.m` interface-compatible; old 4-arg DTC calls trigger a Kt-range-check error.

- **FCS-MPC inner loop**: dq current PI cost-min @ each Tsc → bandwidth ≈ 1 / (5·Tsc) ≈ 4 kHz @ Tsc = 50 μs
- **DTC inner loop**: hysteresis switching, no current PI; switching frequency limited by `fs_max` (typically 5–20 kHz) and HB amplitude. Effective bandwidth is much lower than FCS-MPC's current PI → `T_eq` must be larger

Setting `T_eq = 5·Tsc` for DTC produces an outer-loop bandwidth that exceeds the hysteresis inner-loop bandwidth → wm 33% overshoot. `T_eq = 15·Tsc` tracks reference within 1%.

**Range**: 10–20·Tsc is acceptable; 15 is mid-recommended. Always assert `pi_design.verdict == 'OK'` before proceeding.

## D-CRIT — Chart Configuration + Dual ZOH

The DTC chart contains:

- 4 persistent variables (`psi_a`, `psi_b`, `Cpsi_prev`, `CT_prev`)
- A 4×6 switching table (`int32`)
- Forward Euler integrator (`psi_a += Tsc · (ua - Rs·ia)`)
- 2-level hysteresis with state memory (dead-band retention)

Long stateful charts must use INHERITED + dual ZOH, NOT `DISCRETE + Tsc`. Configuration:

- `ch.SampleTime = '-1'` (INHERITED)
- `ch.ChartUpdate = 'INHERITED'`
- Do NOT set `ch.Inputs(k).DataType` or `ch.Inputs(k).Props.Array.Size` — leave at Inherit

ZOH topology:

- **Every chart input through ZOH @ Tsc**: 5 inputs (`Te_ref, ia, ib, ua, ub`). Even if upstream is already discrete (Saturation @ Tsc), still ZOH it. Defensive normalization.
- **Every chart output through ZOH @ Tsc**: 9 outputs (`gate, Te_meas, mag_psi, ψ_α, ψ_β, sector, V_k, C_ψ, C_T`). Without output ZOH, downstream (UB, Mux logger) sees the chart's internal trigger rate.

**Diagnostic**: if `wm` reaches < 50% of `ω_ref` despite no obvious algorithm bug, first suspect = chart input ZOH missing on at least one port. If chart raises Stateflow type-propagation deadlock during model build, first suspect = `DISCRETE + Tsc` configured instead of INHERITED.

## E-CRIT — InitFcn Self-Contained

The `.slx` must run via "double-click + Run" in a fresh MATLAB session — no external init script required. All workspace variables referenced by mask fields, Saturation limits, Solver settings, etc. must be injected via the model's `InitFcn` field.

```matlab
init_lines = {
    sprintf('Pn=%d;',         Pn),
    sprintf('Rs=%g;',         Rs),
    sprintf('Ld=%g;',         Ld),
    sprintf('Lq=%g;',         Lq),
    sprintf('psi_f=%g;',      psi_f),
    sprintf('J=%g;',          J),
    sprintf('B_visc=%g;',     B),
    sprintf('Vdc=%g;',        Vdc),
    sprintf('T_max=%g;',      T_max),
    sprintf('psi_ref=%g;',    psi_ref),
    sprintf('HB_psi=%g;',     HB_psi),
    sprintf('HB_T=%g;',       HB_T),
    sprintf('Tsc=%g;',        Tsc),
    sprintf('Ts_sps=%g;',     step_size),
    sprintf('T_eq=%g;',       T_eq),
    sprintf('a_so=%g;',       a_so),
    sprintf('Kp_w=%.10g;',    Kp_rpm),
    sprintf('Ki_w=%.10g;',    Ki_rpm),
    sprintf('wref_rpm=%g;',   wref_rpm),
    sprintf('sim_time=%g;',   sim_time),
    sprintf('psi_alpha_0=%g;', psi_alpha_0),
    sprintf('psi_beta_0=%g;',  psi_beta_0),
};
set_param(mdl, 'InitFcn', strjoin(init_lines, newline));
```

Use `%g` for floats (avoid loss of precision); `%.10g` for PI gains specifically. `assignin('base', ...)` alone is **not sufficient**.

## F-CRIT — 19-Channel Logger + 4 Scopes

**Logger** channel order (compatibility with offline overlay scripts):

```
1: t        | 2: wref      | 3: wm (rad/s) | 4: Te_ref
5: Te_meas  | 6: |ψ_s|     | 7: ψ_ref      | 8: ψ_α   | 9: ψ_β
10: sector  | 11: V_k      | 12: C_ψ       | 13: C_T
14-16: ia, ib, ic           | 17-19: ua, ub, uc
```

⭐ **Channels 8 AND 9 (both ψ_α and ψ_β)** are non-negotiable. Missing either makes XY αβ phase plot impossible offline → can't visually distinguish 6-state vs 8-state → can't diagnose A-CRIT failures.

**Scopes** — 4 mandatory:

- `Scope_wm_RPM`: 2-input (`wm_rad · 30/π`, `wref_rpm`)
- `Scope_Te`: 2-input (`Te_meas`, `Te_ref`)
- `Scope_psi_mag`: 2-input (`mag_psi`, `psi_ref`)
- `Scope_psi_alphabeta`: XY Graph block (X = ψ_α, Y = ψ_β), `xmin/xmax/ymin/ymax = ±1.5 · psi_ref`

The αβ XY phase plot is the most diagnostic of all four. A circular trajectory of radius `psi_ref` confirms healthy 6-state operation. Hexagonal pattern with inner-circle (V0/V7 pulling toward origin) is the classic 8-state pollution signature.

Scopes do not replace the logger — they are independent visual aids. Do not feed Scopes through additional ZOH; they tap the same ZOH-output as the logger.
