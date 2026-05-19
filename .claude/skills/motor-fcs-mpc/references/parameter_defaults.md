# Parameter Defaults and Acceptable Ranges

These are recommended starting points for typical PMSM servo applications. Always confirm with user — these are **defaults**, not prescriptions.

## Sampling

| Parameter | Default | Acceptable range | Notes |
|---|---|---|---|
| `Ts` (plant solver) | `1e-6` to `5e-6` | `[1e-7, 1e-5]` | Smaller for tighter switching transients; larger if powergui Discrete struggles |
| `Tsc` (control period) | `2e-5` to `5e-5` | `[1e-5, 1e-4]` | Must satisfy `Tsc/Ts ≥ 10` |

## Cost Function Weights (per Control Objective)

| Objective | `λ_d` | `λ_q` | Ratio |
|---|---|---|---|
| Ripple-priority | 1 | 5–10 | `5:1` to `10:1` |
| Balanced (default) | 1 | 10–30 | `10:1` to `30:1` (common: `1, 20`) |
| Torque-priority | 1 | 50–100 | `50:1` to `100:1` |

Symmetric `1:1` is research-only (q-axis tracking degrades). Outside `[5:1, 100:1]` is atypical — ask user to justify.

## Outer PI

If user supplies `T_eq` and `a` for `pi_design.m`:

| Parameter | Default | Notes |
|---|---|---|
| `T_eq` | `5 · Tsc` | Equivalent inner-loop time constant; 5× control period is a typical Symmetric Optimum assumption |
| `a` | `4` | SO factor; `a=4` gives `ζ_eq ≈ 0.71` |

Hand-tuned (`wn`, `ζ`):

| Parameter | Default | Acceptable range |
|---|---|---|
| `wn` (natural freq) | `2π · 100 Hz` | `[2π·30, 2π·300]` Hz |
| `ζ` (damping) | `0.7` | `[0.5, 1.0]` |

## iq_max (PI Saturation Limit)

Compute from torque budget:

```
iq_max ≥ 1.3 · TL_max / (1.5 · Pn · ψf)
```

Headroom 1.3× is the recommended floor; 1.5× provides comfortable transient margin. Tighter than 1.05× is risky.

## Solver

| Parameter | Default | Notes |
|---|---|---|
| `SolverType` | `Variable-step` | |
| `Solver` | `VariableStepAuto` | Use `ode23tb` for very stiff cases |
| `MaxStep` | `1e-5` | Cap to roughly `Ts` |
| `AbsTol` | `1e-6` | |
| `RelTol` | `1e-4` | |

## Scenario (Typical Servo Test)

| Parameter | Default | Notes |
|---|---|---|
| `StopTime` | `0.6 s` | Allows settling + load step + post-load recovery |
| `ramp_time` | `0.2 s` | `ω_ref` 0 → target ramp duration |
| `ω_ref_target` | 1000–2000 RPM | Match machine rated speed |
| `TL_step_time` | `0.3 s` | Place after `ω_ref` settled |
| `TL_value` | 30–80% of rated torque | Must satisfy `TL_value < 1.5·Pn·ψf·iq_max` |

## Voltage Source

| Parameter | Default | Notes |
|---|---|---|
| `Vdc` | computed from `1.5 · √3 · ω_e_max · ψf` | See base/pre_build_grid.md headroom rule |

## Logging

| Parameter | Default | Notes |
|---|---|---|
| `SaveFormat` | `StructureWithTime` | Compatible with overlay scripts |
| `SampleTime` | `Tsc` | Match control period |
