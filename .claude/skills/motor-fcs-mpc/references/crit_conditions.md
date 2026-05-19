# CRIT Conditions

These six conditions are non-negotiable. Each is a known silent-failure mode that produces plausible-looking simulations with broken physics.

## A-CRIT — Chart Input ZOH (Layered Precision)

Every input port of the FCS-MPC chart must have an upstream `Zero-Order Hold` block with `SampleTime='Tsc'`. This includes inputs already sourced from discrete blocks (PI outputs at Tsc, Constants at infinity rate).

The INHERITED chart's effective trigger rate is set by the **fastest** upstream signal. One un-ZOH'd continuous-rate input poisons the entire chart, breaking the persistent-variable step assumption (e.g., `θ_e = θ_e + Tsc·we` becomes wrong because the actual trigger period is no longer `Tsc`).

**Diagnostic**: if `wm` reaches < 50% of `ω_ref` despite no obvious algorithm bug, first suspect = chart input ZOH missing on at least one port.

## D-CRIT — Outer PI Output Saturation

Required: `LimitOutput='on'`, `UpperSaturationLimit='iq_max'`, `LowerSaturationLimit='-iq_max'`, with `1.5·Pn·ψf·iq_max ≥ 1.3·TL_max` (1.3× headroom; tighter than 1.05× is risky). Without saturation, large speed error drives `iq_ref` unbounded → cost-weight ratio collapses → MPC abandons d-axis tracking → `id` drifts.

Anti-windup is recommended in production but optional in the basic build. Detailed diagnostic (Branch 1 / Branch 2) for `id` steady-state drift is in [outer_pi_saturation.md](outer_pi_saturation.md).

## G-CRIT — Chart Configuration + Dual ZOH

For long algorithm charts (>10 lines + persistent vars + for loop):

- `ch.SampleTime = '-1'`
- `ch.ChartUpdate = 'INHERITED'`
- `ch.Inputs(k).DataType` — leave at Inherit (do NOT set)
- `ch.Inputs(k).Props.Array.Size` — leave at -1 (do NOT set)
- ZOH @ Tsc on every chart input (A-CRIT)
- ZOH @ Tsc on chart output (between chart and downstream Universal Bridge gate input)

Setting DISCRETE+Tsc on a long chart, or force-setting Inputs DataType/Size, causes Stateflow's static analyzer to deadlock on type propagation ("Cannot determine the size and/or data type of the output port").

**Why this works**: the chart itself does not lock the rate; the dual ZOH does. INHERITED chart inherits trigger from its (now Tsc-rate) inputs and emits at the chart's natural rate; the output ZOH locks back to Tsc.

## H-CRIT — Reference Signal Generation

Use `From Workspace` block with an inline matrix expression:

```matlab
add_block('simulink/Sources/From Workspace', [mdl '/wref_src'], ...);
set_param([mdl '/wref_src'], ...
    'VariableName', sprintf('[0 0; %g %g; %g %g]', ramp_time, target_rpm, StopTime, target_rpm), ...
    'Interpolate', 'on', ...
    'SampleTime', '0', ...                                % Continuous; avoids back-inheritance warning
    'OutputAfterFinalValue', 'Holding final value');      % R2024b literal, with -ing
```

Matrix `[t1 v1; t2 v2; ...]` with `Interpolate='on'` linearly interpolates between data points. Single block, fully inlined into `.slx` mask string, satisfies J-CRIT.

**Do NOT use Step + Rate Limiter** unless From Workspace is unavailable. RateLimiter in default Continuous mode escapes its slew limit on the first variable-step solver step, jumping to target instantly while passing all steady-state tests — silent transient failure. If you must use it, set `SampleTimeMode='specified'`, `SampleTime='Tsc'`.

**Do NOT use Signal Editor** — it requires an external `.mat` file in R2024b, cannot be truly inlined, violates J-CRIT.

## J-CRIT — Model Self-Contained (InitFcn Injection)

After `save_system`, the model must be runnable in a fresh MATLAB session by:

1. Double-click `.slx`
2. Press Run
3. Sim succeeds

This requires all parameters (`Rs/Ld/Lq/ψf/J/F/Pn/Vdc/Ts/Tsc/PI gains/iq_max/λ_d/λ_q/id_ref`/scenario timing) to be in the model's `InitFcn` field, generated from the build-script workspace via `sprintf`. `assignin('base', ...)` alone is **not sufficient** — those vars vanish when MATLAB closes.

**Test** in the build script after `save_system`:

```matlab
close_system(mdl, 0);
clear all;             % wipe everything
load_system(mdl_path);
simOut = sim(mdl_name); % must succeed
```

If this fails, the model is not self-contained — fix InitFcn before declaring done.

## K-CRIT — MPC Chart Params via `sprintf` from Plant Source

The MPC prediction model uses `Rs, Ld, Lq, ψf, Pn, Tsc` to compute `id_pred, iq_pred`. These must equal the plant's PMSM parameters (model-match). Correct pattern:

```matlab
chart_body = sprintf([ ...
    'function gate = fcs_mpc(...)\n' ...
    '%%#codegen\n' ...
    'Rs_p   = %g;\n' ...    % literal embedding from build workspace
    'Ld_p   = %g;\n' ...
    'Lq_p   = %g;\n' ...
    'flux_p = %g;\n' ...
    'Pn_p   = %g;\n' ...
    'Tsc_p  = %g;\n' ...
    '... algorithm ...\n' ...
    'end\n'], Rs, Ld, Lq, flux, Pn, Tsc);
ch.Script = chart_body;
```

**Do NOT** put MPC params in an external `.m` file — they will drift from the plant when the user modifies the plant via build script.

**Do NOT** pass MPC params via chart input ports — both deprecated and unable to be runtime-validated against InitFcn.

**Audit**: open the `.slx`, double-click the chart, view the Script. Numeric literals for `Rs/Ld/Lq/ψf` must match `get_param(mdl, 'InitFcn')` digit-for-digit.
