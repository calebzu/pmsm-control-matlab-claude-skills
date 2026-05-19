# SMC Parameter Defaults

Recommended starting points for typical PMSM-SMC servo applications. Always confirm with user.

## Sampling

| Parameter | Default | Acceptable range | Notes |
|---|---|---|---|
| `Tsc` (control period) | `50e-6` | `[25e-6, 100e-6]` | Speed-loop SMC + current-loop PI synchronous sampling |
| `step_size` (plant solver) | `1e-6` | `[1e-7, Tsc/50]` | Power-electronics fixed-step solver. Must satisfy `step_size ≤ Tsc/50` |
| `ramp_time` | `0.5 s` | `[0.1, 1.0]` | ω_ref ramp duration from 0 to target |

## SMC Design

| Parameter | Default | Notes |
|---|---|---|
| `lambda_pd_settling` | `0.01` (10 ms) | PD-type sliding settling time constant; sliding settling 4·λ = 40 ms |
| `Tf_deriv` | `Tsc` | Filtered Derivative time constant; bounds HF gain to `1/Tf` |
| `K1_sta` | auto: `max(200, 1.5·√M·1.5)` where `M = (TL_max + B·ω_max)/J` | Lyapunov requires `K1 > 1.5·√M` |
| `K2_sta` | auto: `max(8000, 1.1·M·1.3)` | Lyapunov requires `K2 > 1.1·M` |

## Current PI

| Parameter | Default | Notes |
|---|---|---|
| `omega_c_inner` | `2000` rad/s | ≈ 5× SMC bandwidth (`1/λ_pd = 100` rad/s); ensures cascade time-scale separation |

PZC computation:
```
Kp_iq = omega_c_inner * Lq;   Ki_iq = omega_c_inner * Rs
Kp_id = omega_c_inner * Ld;   Ki_id = omega_c_inner * Rs
```

Inner PI saturation: `LimitOutput='on'`, `±Vdc/√3`, `AntiWindup='clamping'`.

## Plant Friction (C-CRIT)

| Parameter | Default | Notes |
|---|---|---|
| `B` | `0.008` (≈ 26× small-motor hardware spec) | **MUST be > 0**. SMC needs a dissipation port. If hardware `B << 0.001`, add explicit dissipation Gain block |

## Voltage Source

| Parameter | Notes |
|---|---|
| `Vdc` | Compute from `1.5 · sqrt(3) · ω_e_max · ψ_f` (base/pre_build_grid.md headroom rule). Tight Vdc (1.13× BEMF) causes PI saturation continuously |

## Mode

| Parameter | Default | Notes |
|---|---|---|
| `motor_type` | `'SPMSM'` | Supports `'SPMSM'` and `'IPMSM-mild'` (Lq/Ld < 2). Strong saliency Lq/Ld ≥ 2 → MTPA mid-loop deferred to v1.x |

## Solver

| Parameter | Default | Notes |
|---|---|---|
| `SolverType` | `Fixed-step` | |
| `solver` | `'ode3'` | Bogacki-Shampine. SimPowerSystems standard pairing with powergui Discrete + IGBT |
| `FixedStep` | `Ts_sps` (= `step_size`) | |
| `ZeroCrossControl` | `'DisableAll'` | E-CRIT mandatory: STA's Sign block is discontinuous; ZC ON causes step explosion |

## Scenario (Typical Servo Test)

| Parameter | Default | Notes |
|---|---|---|
| `sim_time` | `1.0 s` | Recommended ≥ `ramp_time + 5·λ_pd + TL_step_transient_observation`. |
| `omega_ref_rpm` | 1000–2000 RPM | Match machine rated speed |
| `TL_step_t` | `0.6 s` | Must be > `ramp_time + 4·λ_pd` (sliding-phase settled before disturbance) |
| `TL_after` | 30–80% of rated torque | Must satisfy `\|TL_after\| ≤ TL_max` |

## Reference Default Plant (Validation Case)

For sanity-checking the build template, a typical SPMSM 1 kW set:

```
Pn = 4
Rs = 0.9585
Ld = 4.987e-3, Lq = 5.513e-3
psi_f = 0.1827
J = 6.329e-4
B = 0.008
Vdc = 500
iq_max = 12, TL_max = 2
omega_ref_rpm = 2000
ramp_time = 0.5
TL_step_t = 0.6, TL_after = 2
```

## Logger

| Parameter | Default | Notes |
|---|---|---|
| `SaveFormat` | `'Array'` | |
| `VariableName` | `'logsout'` | |
| `SampleTime` | `Tsc` | |
| Channel count | `14` | 4 SMC-specific (s / u_sta / e_w / de_w/dt) |
