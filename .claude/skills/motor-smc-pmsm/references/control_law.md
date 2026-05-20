# SMC Control Law (PD Sliding + STA + Lyapunov)

## Outer Speed Loop — PD-Type Sliding + STA

Build flat in main model (no Subsystem grouping needed for v1 baseline).

### Block sequence

| # | Block | Configuration |
|---|---|---|
| 1 | `wref_rpm_const` (`From Workspace`) | `VariableName='omega_ref_data'`, `Interpolate='on'`, `OutputAfterFinalValue='Holding final value'`. InitFcn writes `omega_ref_data = [0 0; ramp_time wref_rpm; sim_time wref_rpm]` |
| 2 | `Gain_RPM_to_rad` | `Gain='pi/30'`. ω_ref_rpm → ω_ref_rad |
| 3 | `Sum_ew` | `Inputs='+-'`. `e_w = ω_ref_rad − ω_m` (where ω_m from `BusSel/4`) |
| 4 | `FilteredDeriv_ew` (`Transfer Fcn`) | `Numerator='[1 0]'`, `Denominator='[Tf 1]'`. Filtered Derivative `s/(Tf·s+1)` outputs `de_w/dt`. HF gain bounded to `1/Tf` prevents derivative kick on noise |
| 5 | `Gain_lambda_pd` | `Gain='lambda_pd'`. Output = `λ · de_w/dt` |
| 6 | `Sum_s` | `Inputs='++'`. `s = e_w + λ·de_w/dt` (sliding surface). **NO integrator** — PD-type avoids wind-up |
| 7 | `Sign_s_sta` (`Sign`) | Output ∈ {−1, 0, +1} |
| 8 | `Fcn_abs_sqrt` (`Fcn`) | `Expr='sqrt(abs(u(1)))'`. Output = `|s|^0.5` |
| 9 | `Prod_sta1` (`Product`, 2 inputs) | Output = `|s|^0.5 · sgn(s)` (continuous through s=0) |
| 10 | `Gain_K1` | `Gain='K1_sta'`. Output = `K1 · |s|^0.5 · sgn(s)` |
| 11 | `Int_sgn_s` (`Integrator`) | `InitialCondition='0'`. Input = `sgn(s)`, output = `∫sgn(s)` |
| 12 | `Gain_K2` | `Gain='K2_sta'`. Output = `K2 · ∫sgn(s)` |
| 13 | `Sum_sta` | `Inputs='++'`. Output = `K1·|s|^0.5·sgn(s) + K2·∫sgn(s) = u_sta` (acceleration, rad/s²) |
| 14 | `Gain_J_Kt` | `Gain='J/Kt'`. Convert acceleration → iq amplitude (A). Output = `iq_ref_unsat` |
| 15 | `Saturation_iq` (`Saturation`) | `UpperLimit='iq_max'`, `LowerLimit='-iq_max'` (B-CRIT mandatory). Output = `iq_ref` to inner PI |
| 16 | `id_ref` (`Constant`) | `Value='0'` (SPMSM / mild-saliency IPMSM v1 baseline) |

### Sign Convention Derivation

Preserve as a comment in your build script:

```
ds/dt ≈ -Kt·iq/J + disturbance   (because e_w = wref - wm; wm acceleration reduces e_w)
Setting iq_ref = (J/Kt)·u_smc maps to ds/dt = -u_smc + φ
Standard STA: u = -K1·|s|^0.5·sgn(s) - K2·∫sgn(s) drives ds/dt → 0
Therefore u_smc = +K1·|s|^0.5·sgn(s) + K2·∫sgn(s) for our system (positive K1/K2).
```

## Inner Current Loop — PI ×2 (PZC)

For each axis (q and d), build `Sum_iq_err / PID_iq` and `Sum_id_err / PID_id`:

```matlab
% Discrete PID Controller mask:
'Controller',              'PI',
'P',                       'Kp_iq',     % or 'Kp_id'
'I',                       'Ki_iq',     % or 'Ki_id'
'IntegratorMethod',        'Forward Euler',
'SampleTime',              'Tsc',
'LimitOutput',             'on',
'UpperSaturationLimit',    'Vdc/sqrt(3)',
'LowerSaturationLimit',   '-Vdc/sqrt(3)',
'AntiWindupMode',          'clamping',
```

Compute `Kp / Ki` via PZC at build time:

```
Kp_iq = omega_c_inner * Lq;   Ki_iq = omega_c_inner * Rs
Kp_id = omega_c_inner * Ld;   Ki_id = omega_c_inner * Rs
omega_c_inner default = 2000 rad/s   (≈ 5× SMC bandwidth 1/λ_pd = 100 rad/s)
```

Bandwidth ratio inner/outer ≥ 5× ensures cascade time-scale separation.

## Lyapunov STA Gain Bounds (C-CRIT)

```
M = (TL_max + B · ω_max) / J     [disturbance bound, rad/s²]
K1_sta > 1.5 · sqrt(M)
K2_sta > 1.1 · M
```

Build script asserts both inequalities at construction time:

```matlab
omega_max = omega_ref_rpm * 2*pi / 60;
disturbance_bound = (params.TL_max + params.B * omega_max) / params.J;
K1_min = 1.5 * sqrt(disturbance_bound);
K2_min = 1.1 * disturbance_bound;
assert(K1_sta > K1_min, 'STA: K1=%.1f < 1.5*sqrt(M)=%.1f', K1_sta, K1_min);
assert(K2_sta > K2_min, 'STA: K2=%.1f < 1.1*M=%.1f',  K2_sta, K2_min);
```

Default auto-computation with margin:

```
K1_sta = max(200,  1.5 · sqrt(M) · 1.5)
K2_sta = max(8000, 1.1 · M · 1.3)
```

The margin (1.5 / 1.3) is empirical buffer above the Lyapunov-required lower bound.

## Plant Friction `B > 0` — v1 Baseline Assumption (not a theoretical requirement)

`B > 0` is the **v1 baseline assumption, not a theoretical requirement** of SMC/STA. STA finite-time convergence follows from the K1/K2 gain conditions above, **independent of plant viscous damping**; a `B = 0` pure-integrator speed loop (`J·ω̇ = Te − TL`) is relative-degree-1 and STA-controllable in principle.

- Default `B = 0.008` (≈ 26× small-motor hardware spec) for the v1 baseline
- The v1 baseline was developed and validated **entirely with `B > 0`**, so `B = 0` is **outside the validated envelope**
- If your plant has `B ≈ 0`: re-validate the STA gains (`K1, K2`) for that case rather than assuming the controller requires damping

## TL Trough on High-M Plants

The Lyapunov K1/K2 lower bound formula provides a **necessary condition for finite-time convergence**, NOT optimal disturbance rejection. On high-M plants (typically `M ≥ 2×` v1 baseline), the standard floor formula may produce a TL-step trough below 70% of pre-step speed.

**Mitigation when TL trough must stay above 70%**: pump K1 to 3–5× K1_min and K2 to 3× K2_min. Adaptive TL observer (deferred v1.x extension) would also fix this without manual gain pumping.
