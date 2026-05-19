---
title: R2024b MATLAB / Simulink / SimPowerSystems API Notes
classification: shareable — platform facts (no design content, no machine-specific numbers)
scope: MATLAB R2024b SimPowerSystems + Stateflow API behavior
exclusions: no reference model file names, no machine-specific parameter values, no design decisions
---

# R2024b MATLAB / Simulink / SPS API Notes

Reference for building PMSM control models in R2024b. All facts established by direct API probe or official Simulink/SPS documentation.

## §1 SimPowerSystems library paths (R2024b)

**Key change**: from R2024b onward, SPS blocks have been moved from `powerlib/...` to `sps_lib/...`. `powerlib` still partially works, but some blocks (e.g. Universal Bridge) have a new path.

### Canonical paths

```matlab
% DC / AC sources
'sps_lib/Sources/DC Voltage Source'
'sps_lib/Sources/AC Voltage Source'
'sps_lib/Sources/Controlled Voltage Source'

% Machines
sprintf('sps_lib/Electrical Machines/Permanent Magnet%cSynchronous Machine', 10)
% ⚠️ PMSM block name contains a newline (\n)! Construct with sprintf(...%c..., 10)

% Inverters / power electronics
'sps_lib/Power Electronics/Universal Bridge'
'sps_lib/Power Electronics/Power Electronics Control/PWM Generator (2-Level)'

% Measurements
'sps_lib/Sensors and Measurements/Voltage Measurement'
'sps_lib/Sensors and Measurements/Current Measurement'

% Passives
sprintf('sps_lib/Passives/Three-Phase%cSeries RLC Branch', 10)
'sps_lib/Passives/Series RLC Branch'

% System solver
'sps_lib/powergui'
```

### Fallback probe template

```matlab
% Cross-version safe mode: search by block name
load_system('sps_lib');
h = find_system('sps_lib', 'SearchDepth', Inf, 'Name', 'Universal Bridge');
disp(h);   % returns the actual R2024b path
```

## §2 SimPowerSystems mask fields (R2024b)

**Key change**: SPS mask field names in R2024b are predominantly **CamelCase** (older versions used snake_case).

### Universal Bridge

```matlab
% R2024b correct field names (CamelCase)
Arms                  % '1' / '2' / '3'  (NOT 'arms')
SnubberResistance     % '1e5' typical (NOT 'snubber_resistance')
SnubberCapacitance    % 'inf'
Device                % 'Diodes' / 'Thyristors' / 'IGBT / Diodes' (note spaces in 'IGBT / Diodes')
Ron                   % Ron value (Ω)
Lon                   % '0' typical
ForwardVoltages       % array '[0 0]'
ForwardVoltage        % single value
IGBTparameters        % '[1e-6, 2e-6]' (turn-on, turn-off)
converterType         % 'Rectifier' / 'Inverter'
Measurements          % 'None' / 'Device voltages' / 'Device currents' / ...
```

### PMSM (Permanent Magnet Synchronous Machine)

```matlab
% R2024b correct field names
NbPhases               % '3'
FluxDistribution       % 'Sinusoidal' / 'Trapezoidal'
RotorType              % 'Salient-pole' / 'Round' / ...
MechanicalLoad         % 'Torque Tm' / 'Speed w'
PresetModel            % built-in preset name, or 'No'
MeasurementBus         % 'on' / 'off' — must be 'on' to get bus output
Resistance             % Rs (string; accepts workspace variable name)
dqInductances          % '[Ld Lq]' — note it's an **array**, not two separate fields
Flux                   % ψf
Mechanical             % '[J F PolePairs InitialSpeed]' — 4-element array!
PolePairs              % Pn
RefAngle               % 'Aligned with phase A axis (original Park)' / others
IterativeDiscreteModel % 'Trapezoidal non iterative' / ...
TsBlock                % '-1' to inherit from powergui
TsPowergui             % '0'
```

**⚠️ Common mistake**: writing `stator_inductance_Ld` / `stator_inductance_Lq` / `inertia` / `friction` / `pole_pairs` as separate fields from memory — **R2024b does not have these fields**. Inductances are an array `dqInductances`, and mechanical parameters are packed into the `Mechanical` array.

### DC Voltage Source

```matlab
Amplitude       % voltage value (V)
Measurements    % 'None' default (DC Source has only two fields)
```

### Correct way to probe mask fields

```matlab
% After adding a block, probe DialogParameters to discover actual R2024b field names
new_system('test'); add_block('sps_lib/.../SomeBlock', 'test/b');
dp = get_param('test/b', 'DialogParameters');
disp(fieldnames(dp));
close_system('test', 0);

% Or use the Mask API to see popup options
mo = Simulink.Mask.get('test/b');
for k = 1:numel(mo.Parameters)
    fprintf('%s (%s)\n', mo.Parameters(k).Name, mo.Parameters(k).Type);
    if strcmp(mo.Parameters(k).Type, 'popup')
        fprintf('  options: [%s]\n', strjoin(mo.Parameters(k).TypeOptions, ', '));
    end
end
```

## §3 PMSM measurement bus signal names (R2024b)

When `MeasurementBus = 'on'`, the PMSM outport produces a bus with these signal names:

| Signal | Unit | Physical meaning |
|---|---|---|
| `ias, ibs, ics` | A | Three-phase stator currents |
| `iqs, ids` | A | dq stator currents (PMSM internally transformed; usually prefer `ias/ibs/ics` + your own Clarke/Park) |
| `vqs, vds` | V | dq stator voltages |
| `ha, hb, hc` | bool | Hall sensor signals |
| `w` | rad/s | Rotor mechanical speed (**not `wm`**) |
| `theta` | rad | Rotor mechanical angle (**not `thetam`**; bus signal is literally `theta`) |
| `Te` | N·m | Electromagnetic torque |

**⚠️ Common mistake**: writing `is_abc` / `wm` / `thetam` from memory — **all wrong in R2024b**.

**Bus Selector configuration**:
```matlab
set_param('model/BS_PMSM', 'OutputSignals', 'ias,ibs,ics,w,Te');
```

## §4 Stateflow / MATLAB Function block API

### EMChart vs Chart

Inside a MATLAB Function block, the chart type is `Stateflow.EMChart` (Embedded MATLAB Chart), **not** the general `Stateflow.Chart`.

In R2024b, `sfroot` must be invoked as a function call `sfroot()`. The older property-access syntax `sfroot.find(...)` no longer works.

```matlab
% ✅ Correct (R2024b)
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', 'model/MyFcn');

% ❌ Wrong (R2024b error)
ch = sfroot.find('-isa', 'Stateflow.EMChart', 'Path', 'model/MyFcn');

% ❌ Wrong (returns empty: MATLAB Function is not a general Chart)
ch = sfroot().find('-isa', 'Stateflow.Chart', 'Path', 'model/MyFcn');

% Safe fallback: try both
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', path);
if isempty(ch)
    ch = sfroot().find('-isa', 'Stateflow.Chart', 'Path', path);
end
```

### Setting chart source code

```matlab
ch.Script = body_string;   % full function .m text
```

### Setting chart SampleTime + ChartUpdate

⚠️ For **long-algorithm charts** (>10 lines / persistent vars / for loops), `Tsc + DISCRETE` is **wrong** — it triggers type-propagation deadlock. **Correct setting**:

```matlab
ch.SampleTime  = '-1';         % INHERITED — let the chart follow upstream sample rate
ch.ChartUpdate = 'INHERITED';  % consistent with SampleTime
% Rate-limit the output with a ZOH @ Tsc (see §11), not on the chart itself
```

`Tsc + DISCRETE` works only for **short charts** (<10 lines / no persistent / no for-loops). Long-algorithm charts must use INHERITED + an output-side ZOH.

**⚠️ Order**: set `Script` first (initializes structure), then set `SampleTime` / `ChartUpdate`. Otherwise SampleTime can be overwritten by initialization in some cases.

Full SOP in §11.

## §5 SPS port topology probing

SPS blocks have multiple port types: `Inport`, `Outport`, `LConn`, `RConn`. The first two are signal ports; the last two are physical (electrical) ports.

```matlab
% Probe port layout
ph = get_param(block_path, 'PortHandles');
fprintf('Inport: %d\n', numel(ph.Inport));
fprintf('Outport: %d\n', numel(ph.Outport));
fprintf('LConn: %d\n', numel(ph.LConn));
fprintf('RConn: %d\n', numel(ph.RConn));

% Detailed type + coordinates
pc = get_param(block_path, 'PortConnectivity');
for k = 1:numel(pc)
    fprintf('Port %d: Type=%s Position=%s\n', k, pc(k).Type, mat2str(pc(k).Position));
end
```

### `add_line` and port numbering convention

```matlab
% Signal port (Inport/Outport): "block/N" refers to the Nth Inport or Outport
% MATLAB infers direction from src/dst position (src=Outport, dst=Inport)
add_line(model, 'PMSM/1', 'ZOH/1');   % PMSM Outport 1 → ZOH Inport 1

% Physical port (LConn/RConn): "block/LConnN" or "block/RConnN"
add_line(model, 'Vdc/RConn1', 'UB/RConn1', 'autorouting', 'on');
add_line(model, 'UB/LConn1',  'PMSM/LConn1', 'autorouting', 'on');
```

### ⚠️ PMSM dual-Inport port-count pitfall

`PortConnectivity` may return `Port 1: Type=1 SrcBlock=-1` and `Port 2: Type=1 SrcBlock=(empty)` — both are Type=1 (not LConn/RConn), but one is an Inport (`SrcBlock=-1` means it needs a source) and the other is an Outport (`SrcBlock=""` means it is the source).

**Practical mapping**:
- `Inport[1]` (PMSM) → `Tm` input
- `Outport[1]` (PMSM) → measurement bus output

```matlab
add_line(m, 'TL_Step/1', 'PMSM/1');   % TL_Step Outport 1 → PMSM Inport 1 (Tm)
add_line(m, 'PMSM/1',    'ZOH/1');    % PMSM Outport 1 → ZOH Inport 1
% 'PMSM/1' has different semantics on src vs dst side (src=Outport, dst=Inport); MATLAB infers from position
```

## §6 Solver / powergui combination (minimal working configuration)

```matlab
% powergui configuration
set_param('model/powergui', 'SimulationMode', 'Discrete');
set_param('model/powergui', 'SampleTime', 'Ts');    % workspace variable, typical 1e-6
set_param('model/powergui', 'SolverType', 'Tustin/Backward Euler (TBE)');

% Main solver
set_param('model', 'SolverType', 'Variable-step');
set_param('model', 'Solver', 'VariableStepAuto');    % or 'ode23tb'
set_param('model', 'StopTime', 'StopTime');
```

**Pitfalls**:
- Fixed-step + SimPowerSystems usually conflicts (unless the entire model is strictly discrete); use Variable-step for stability
- `ode15s` / `ode23tb` suits stiff power electronics; `ode45` is very slow with PWM switching

## §7 Sim output unpacking (StructureWithTime format)

When `To Workspace` blocks use `SaveFormat = 'StructureWithTime'`, `sim()` returns a `Simulink.SimulationOutput` object.

```matlab
simOut = sim(model);

% ✅ Correct: all TW variables are wrapped in a struct
t = simOut.varname.time;             % time vector
d = simOut.varname.signals.values;   % data matrix (N × width)

% ❌ Wrong: direct extraction
wm = simOut.get('varname');          % returns a struct; multiplying a struct by a number errors out
```

### List all logged signals

```matlab
names = simOut.who;   % cell array of variable names
for k = 1:numel(names)
    v = simOut.(names{k});
    fprintf('%s: class=%s\n', names{k}, class(v));
end
```

## §8 Common compile / sim error patterns

| Error | Likely cause | Fix |
|---|---|---|
| "No block named 'powerlib/...'" | R2024b moved to sps_lib | Switch to `sps_lib/...` path, or `find_system` to locate |
| "mask has no parameter named 'xxx'" | Mask field name varies across versions | Probe `DialogParameters` to get the actual field name |
| "Stateflow.Chart not found" | MATLAB Function block is an EMChart | Use `Stateflow.EMChart` |
| "Signal X not found in input bus" | Bus Selector OutputSignals string doesn't match actual bus | Probe `InputSignals` or pick from the dialog |
| "Sample time not specified in chart" | Chart SampleTime was overwritten by `set Script` | Re-set `ch.SampleTime='Tsc'` after `Script` |
| "Port already has a signal line connected" | Duplicate `add_line` or unintended branch | `delete_line` first, or use `add_line` with branch semantics |
| "Invalid line specifier" | Wrong port name (LConn vs 1, etc.) | Use `get_param(block, 'PortConnectivity')` to see actual port types |

## §9 Compiled port width probing (before sim)

```matlab
% Enter compile mode to read compiled port widths
feval(model, [], [], [], 'compile');
try
    ph = get_param('model/block', 'PortHandles');
    for k = 1:numel(ph.Outport)
        w  = get_param(ph.Outport(k), 'CompiledPortWidth');
        dt = get_param(ph.Outport(k), 'CompiledPortDataType');
        fprintf('O%d: width=%d type=%s\n', k, w, dt);
    end
catch ME
    fprintf('%s\n', ME.message);
end
feval(model, [], [], [], 'term');
```

## §10 `add_block` cross-version fallback template

```matlab
function blk = add_block_safe(candidates, dest, opts)
    last_err = '';
    for k = 1:numel(candidates)
        try
            args = [{candidates{k}, dest}, opts];
            add_block(args{:});
            blk = dest;
            return;
        catch ME
            last_err = ME.message;
        end
    end
    error('all candidates failed, last: %s', last_err);
end

% Usage (only for generic blocks not in your shared library)
add_block_safe({'powerlib/...', 'sps_lib/...', 'nelib/...'}, ...
               'model/block', {'Amplitude', '300'});
```

**Priority**: try `shared/building_blocks/pmsm_blocks.slx` first; only fall back when the library doesn't carry the block you need.

---

## §11 Stateflow chart configuration SOP (long-algorithm charts)

For MATLAB Function block charts containing a full algorithm (>10 lines / persistent vars / for loops / multiple I/O), the configuration below is required to avoid type-propagation deadlock.

### ✅ Correct configuration

```matlab
% 1. Add the MATLAB Function block
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/CHART_NAME'], ...
    'Position', [...]);

% 2. Set Script (full algorithm function text)
chart_body = sprintf(['function [out1, out2] = my_fn(in1, in2, ...)\n' ...
                     '%%#codegen\n' ...
                     '... algorithm ...\n' ...
                     'end\n']);
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', [model '/CHART_NAME']);
ch.Script = chart_body;

% 3. Set chart mode — INHERITED, NOT DISCRETE
ch.SampleTime  = '-1';          % chart follows upstream sample rate
ch.ChartUpdate = 'INHERITED';

% 4. ⚠️ Critical: don't set DataType / Size on Inputs/Outputs — leave them all Inherit
%    Don't write:
%       ch.Inputs(k).DataType = 'double';       ❌ triggers type-inference conflict
%       ch.Inputs(k).Props.Array.Size = '1';    ❌ triggers type-inference conflict
%    Correct: don't set anything — let Stateflow infer automatically

% 5. ⚠️ Critical: add ZOH @ Tsc upstream of every chart Inport
%    An INHERITED chart's trigger rate is determined by its upstream. Mixed-rate inputs
%    cause the chart to re-trigger between Tsc steps, breaking the step=Tsc assumption
%    of any persistent variables (e.g. an integrated θ_e).
N_inputs = numel(ch.Inputs);
for i = 1:N_inputs
    in_name  = ch.Inputs(i).Name;
    zoh_name = sprintf('ZOH_chart_in_%s', in_name);
    add_block('simulink/Discrete/Zero-Order Hold', [model '/' zoh_name], ...
        'SampleTime', 'Tsc', 'Position', [...]);
    add_line(model, '<upstream_block>/<port>', [zoh_name '/1']);
    add_line(model, [zoh_name '/1'], ['CHART_NAME/' num2str(i)]);
end

% 6. Add ZOH @ Tsc on the output to lock the rate (in lieu of the chart's own Tsc mode)
add_block('simulink/Discrete/Zero-Order Hold', [model '/ZOH_chart_out'], ...
    'SampleTime', 'Tsc', 'Position', [...]);
add_line(model, 'CHART_NAME/1', 'ZOH_chart_out/1');
add_line(model, 'ZOH_chart_out/1', '<downstream_block>/1');
```

**Two-sided ZOH layout**:
```
<upstream_signals>  ──┬→ ZOH_in_1 @ Tsc ──→ chart Inport 1
                      ├→ ZOH_in_2 @ Tsc ──→ chart Inport 2
                      ├→ ... (every input gets its own ZOH)
                      └→ ZOH_in_N @ Tsc ──→ chart Inport N

chart Outport 1 ──→ ZOH_out @ Tsc ──→ <downstream_block>
```

**Why both sides need ZOH**:
- Upstream ZOH: synchronizes all chart inputs to Tsc, prevents fast-rate upstream from re-triggering the chart
- Downstream ZOH: forces the chart output to strictly Tsc rate, prevents downstream from sampling it multiple times
- An INHERITED chart does not lock its own rate; **the rate is set by the ZOH pair on both sides**

### ❌ Wrong configurations (trigger type-propagation deadlock)

| Wrong | Symptom | Fix |
|---|---|---|
| `ch.Inputs(k).DataType = 'double'` (forced) | "Cannot determine block output size and/or type" | Remove; leave Inherit |
| `ch.Inputs(k).Props.Array.Size = '1'` (forced) | Same | Remove; leave -1 auto |
| `ch.SampleTime = 'Tsc' + ChartUpdate = 'DISCRETE'` for a long-algorithm chart | Same | Use `-1` + `INHERITED` + output ZOH |
| All three above | Compounded deadlock | Remove all three; use the correct config |

### Wrong → correct transformation

`Long chart Script + forced types + DISCRETE+Tsc` ≡ **triple over-spec** → Stateflow static analysis cannot solve

`Long chart Script + Inherit + INHERITED+(-1) + output ZOH @ Tsc` ≡ **correct SOP**

### Why "Inherit everywhere" works

A long-algorithm chart fully configured with Inherit (DataType="Inherit: Same as Simulink", Size=-1, SampleTime=-1, ChartUpdate=INHERITED) does not have type-propagation problems even with 50+ lines of cost function + persistent vars + for loops. **The chart's effective trigger rate is determined by the upstream ZOH @ Tsc** (inputs are held to Tsc rate → chart triggers in sync), so the chart itself does not need to lock Tsc.

### Short-chart exception

If the chart Script is ≤10 lines + no persistent vars + no for loops (e.g. simple saturation / abs / sign), `Tsc + DISCRETE + forced types` still works. But for **long-algorithm charts, this is forbidden**.

### Diagnostic flow

On "Cannot determine block output size and/or type":
1. **First suspect**: did you `set_param` any `ch.Inputs(k).DataType` or `Props.Array.Size`? **Remove all**, leave Inherit, recompile
2. **Second suspect**: chart `SampleTime` + `ChartUpdate` set to `DISCRETE+Tsc`? Change to `-1+INHERITED` + add output ZOH @ Tsc, recompile
3. **Third suspect**: chart function signature input count ≠ block Inport count. Check your `add_line` count.

### §11.1 Chart property corruption SOP

**Failure mode**: after `add_block(chart) + ch.Script = body`, doing a large number of subsequent `add_block` / `add_line` operations may leave the chart in a state where the property table reads OK (`ch.SampleTime='-1' / ch.ChartUpdate='INHERITED'` display normally) but Simulink compile sees the SampleTime field as empty → "ChartUpdate set but SampleTime empty".

**Symptoms**:
- The configuration code looks right; `get(ch, 'SampleTime')` returns `'-1'`
- `ch.ChartUpdate` returns `'INHERITED'`
- But `set_param(model, 'SimulationCommand', 'update')` or `sim` reports "ChartUpdate set but SampleTime empty"

**Root cause (suspected)**: race between Stateflow chart internal property storage and the `add_line` graph rebuild; after heavy operations the chart properties need a commit that the API doesn't expose.

**Workaround SOP**:
```matlab
% Phase A: build all other blocks + lines + InitFcn (chart as a placeholder)
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/CHART'], 'Position', [...]);
% ... all other add_block / add_line operations ...

% Phase B: at the end of the build, before save, delete + re-add + re-wire the chart
delete_block([model '/CHART']);
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/CHART'], 'Position', [...]);
ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', [model '/CHART']);
ch.Script = chart_body;
ch.SampleTime  = '-1';
ch.ChartUpdate = 'INHERITED';
% Re-wire all chart input/output lines
add_line(model, ...);   % all chart-related lines
```

**Verification**: `update_diagram` + a short sim (e.g. 0.001 s) succeeding = the chart properties are committed.

### §11.2 Chart vector-output direction rule

When a MATLAB Function chart outputs a vector to an SPS Inport (e.g. Universal Bridge 6-wide pair-adjacent gate), the output **must be a column**:

```matlab
% ✅ Correct (chart output matches UB-expected [6×1])
gate6 = double([sa; 1-sa; sb; 1-sb; sc; 1-sc]);    % column 6×1

% ❌ Wrong (compile reports size mismatch)
gate6 = double([sa, 1-sa, sb, 1-sb, sc, 1-sc]);    % row 1×6
```

**Why column**: SPS blocks (Universal Bridge / PMSM / ...) expect column-vector inputs by default (time × signals; signals form the column dimension). The chart's Inport [-1] auto-size inference uses row and conflicts with the downstream.

**General guideline**: chart output vectors default to `[a; b; c; ...]` column form unless the downstream explicitly expects row.

---

## §12 R2024b `From Workspace` block inline mode SOP

For reference signal generation (ω_ref ramp / step / square wave / multi-segment timing), the `From Workspace` block with an inline matrix expression is the recommended self-contained pattern.

### ✅ Correct usage (`From Workspace` + inline matrix expression)

```matlab
% 1. Add the block
add_block('simulink/Sources/From Workspace', [mdl '/SIG_NAME'], ...
    'Position', [60 80 110 130]);

% 2. Configure mask — VariableName directly takes a matrix expression [t1 v1; t2 v2; ...]
%    Column 1 = time (s), column 2 = signal value
%    The mask field is a string; sprintf to embed workspace vars
set_param([mdl '/SIG_NAME'], ...
    'VariableName', sprintf('[0 0; %g %g; %g %g]', ...
                           ramp_time, target_value, sim_stop_time, target_value), ...
    'Interpolate', 'on', ...
    'SampleTime', '0', ...                              % '0' (Continuous), NOT '-1' — avoids back-inheritance warning
    'OutputAfterFinalValue', 'Holding final value');    % R2024b literal is 'Holding final value' (with -ing)
```

⚠️ **R2024b literal-name notes**:
- `SampleTime='-1'` triggers a Source-block back-inheritance warning on every sim. Use `'0'` (Continuous). `From Workspace` inline matrix is correct in Continuous mode (the data points are linearly interpolated independently of solver step size).
- `OutputAfterFinalValue` R2024b literal is `'Holding final value'` (with -ing). Fuzzy-match accepts `'Hold final value'` but produces a warning; the canonical literal is `'Holding final value'`.

### Matrix format (essential)

```
[ t1  v1
  t2  v2
  t3  v3
  ...      ]
```
- Each row = one (time, value) pair
- Must have 2 columns (time in column 1)
- `Interpolate='on'` → linear interpolation between data points (automatic ramp behavior)
- When sim time exceeds the last row's `t`, behavior follows `OutputAfterFinalValue`:
  - `'Holding final value'` → hold the last value (recommended)
  - `'Setting to zero'` → force zero
  - `'Extrapolation'` → linear extrapolation

### Scenario templates

| Scenario | Matrix expression |
|---|---|
| 0 → V ramp during 0 ~ T, then hold V | `[0 0; T V; sim_end V]` |
| Step: t<T is 0, t≥T is V | `[0 0; T 0; T V; sim_end V]` (repeated time at V = step) |
| Ramp + reverse step | `[0 0; T1 V1; T2 V1; T2 0]` |
| Square wave (period P, 50% duty) | Hand-write multiple rows or generate in a loop |

### Self-contained verification

```matlab
% Close model + clear base workspace + reload + sim
bdclose(mdl); clear all;
load_system([mdl '.slx']);
out = sim(mdl);   % should succeed because the matrix is inside the mask string
```

### ❌ Don't use Signal Editor for inline scenarios

R2024b Signal Editor mask defaults to `FileName=untitled.mat`:
- ⚠️ **Requires an external .mat file** — cannot truly be inline (`FileName=''` / `'inline'` / `SignalSource=Embedded` all fail)
- Datasets in model workspace cannot be attached either
- Deployment must ship the .mat → **violates the self-contained discipline**
- **Signal Editor only fits** lab-style multi-scenario switching (design phase, where managing dataset files is acceptable); **forbidden in a frozen skill template**

### ❌ Don't use `Step + RateLimiter`

```matlab
% Wrong: using RateLim + Step to produce a ramp, but RateLim Continuous mode escapes the slew limit
add_block('simulink/Sources/Step', [mdl '/wref_step'], 'After', '2000');
add_block('simulink/Discrete/Rate Limiter', [mdl '/RateLim'], ...
    'RisingSlewLimit', 'slew_rpm');
% ❌ SampleTimeMode='specified' + SampleTime='Tsc' not set
% → Variable-step solver's first step is too large, RateLim output jumps to the Step target
% → ω_ref 0→2000 RPM jumps in 0.017 s (should take 0.2 s)
```

If you must use this pattern (e.g. pre-R2024a), **always** set:
```matlab
set_param([mdl '/RateLim'], 'SampleTimeMode', 'specified', 'SampleTime', 'Tsc');
```

### Diagnostic flow

If reference signal ω_ref behaves wrong (sudden jump / step lost / ramp duration off):
1. **First suspect**: using RateLim but SampleTime='inherited'. Change to 'specified' + 'Tsc'
2. **Better fix**: switch to `From Workspace` + inline matrix (this section §12), eliminating the entire RateLim Continuous risk surface
3. **Diagnostic sim**: run sim and plot ω_ref against the expected waveform

---

## §13 `Goto` / `From` `TagVisibility` cross-subsystem visibility

`simulink/Signal Routing/Goto` blocks have a `TagVisibility` parameter that determines which `From` blocks can subscribe to the `GotoTag`.

### §13.1 R2024b Goto/From default + visibility matrix

| TagVisibility value | Visible scope | Default |
|---|---|---|
| `'local'` ⭐ default | Same hierarchy level only (within the same Subsystem boundary) | ✅ |
| `'scoped'` | Within the scope defined by a `Goto Tag Visibility` block (requires that block to publish the scope) | — |
| `'global'` | Entire model (penetrates all Subsystem boundaries; library-block-internal Froms can subscribe) | — |

### §13.2 Silent failure mode

⛔ **Critical pitfall**: with default `TagVisibility='local'` + a library-block-internal `From` subscribing to a top-model `Goto`:
- ❌ NO compile error
- ❌ NO sim runtime error
- ❌ The `From` block silently outputs default 0 (unconnected fallback)
- ❌ Downstream computation continues with wrong data, producing plausible but **incorrect** numerical metrics

**Typical scenario**: top model publishes θ_e via a `Goto`; a library block (e.g. `pmsm_blocks/Anti_Park`) subscribes via `From "The"` internally. During library development, when both are at the same hierarchy level, it works. After an external user instantiates the library block in their own model, it silently fails.

### §13.3 Fix

```matlab
% Set 'global' explicitly when adding the Goto block in the top model
add_block('simulink/Signal Routing/Goto', [mdl '/Goto_The'], 'Position', [...]);
set_param([mdl '/Goto_The'], 'GotoTag', 'The', 'TagVisibility', 'global');
```

### §13.4 Self-test SOP

Any build script that uses Goto/From across top-model ↔ library-block boundaries must include a self-test:

```matlab
goto_blks = find_system(mdl, 'BlockType', 'Goto');
for k = 1:numel(goto_blks)
    tag = get_param(goto_blks{k}, 'GotoTag');
    vis = get_param(goto_blks{k}, 'TagVisibility');
    if any(strcmp(tag, {'The', 'omega_e', ...}))   % cross-subsystem tags
        assert(strcmp(vis, 'global'), ...
            'Goto Tag "%s" must be TagVisibility=global, got %s', tag, vis);
    end
end
```

### §13.5 Visual self-check

Numerical metrics alone cannot detect this silent failure. **A user visual review must verify**:
- Whether the physical quantity (e.g. θ_e for Anti_Park) is actually being transmitted
- Indirect indicator: are the abc three-phase currents AC sinusoidal? (If Goto failed → FOC degenerates to lab-frame open-loop → abc DC-locked)

---

## §14 `pmsm_blocks/Anti_Park` library-block port convention

### §14.1 Port topology

```
Anti_Park library-block external interface:
  Inport 1  = Vq      (q-axis voltage command, V)
  Inport 2  = Vd      (d-axis voltage command, V)
  Outport 1 = Ualpha  (αβ frame α-axis voltage, V)
  Outport 2 = Ubeta   (αβ frame β-axis voltage, V)

Anti_Park library-block internal dependency:
  Internal `From` block with GotoTag = "The"
  Subscribes to θ_e (electrical position, rad)
  Source = top-model `Goto "The"` block (must have TagVisibility='global', see §13)
```

### §14.2 Standard wiring

```matlab
% Top model first builds the θ_e chain
add_block('simulink/Math Operations/Gain', [mdl '/Gain_Pn'], 'Position', [...]);
set_param([mdl '/Gain_Pn'], 'Gain', 'Pn');
add_line(mdl, 'BusSel/6', 'Gain_Pn/1', 'autorouting', 'on');   % BusSel/6 = θ_m

add_block('simulink/Signal Routing/Goto', [mdl '/Goto_The'], 'Position', [...]);
set_param([mdl '/Goto_The'], 'GotoTag', 'The', 'TagVisibility', 'global');
add_line(mdl, 'Gain_Pn/1', 'Goto_The/1', 'autorouting', 'on');

% Then connect Anti_Park
add_block('pmsm_blocks/Anti_Park', [mdl '/Anti_Park'], 'Position', [...]);
add_line(mdl, 'Sum_Vq_total/1', 'Anti_Park/1', 'autorouting', 'on');   % port 1 = Vq
add_line(mdl, 'Sum_Vd_total/1', 'Anti_Park/2', 'autorouting', 'on');   % port 2 = Vd
```

⚠️ **Port order**: Vq on port 1, Vd on port 2 (**NOT** swapped). Swapping causes the motor to stop moving completely.

### §14.3 Reuse rule

`Anti_Park` can also be used for current transform dq → αβ (the transform formulas `Uα = Uq·cos(θe) - Vd·sin(θe)*-1` etc. apply equally to `iα/iβ`). The port convention is unchanged — currents also go through Inport 1 = q-axis, Inport 2 = d-axis.

---

## §15 `pmsm_blocks/SVPWM` library-block — sector=7 startup edge case

### §15.1 Library-block internal structure

```
SVPWM library block (look under masks):
  Sector_Calculate subsystem:
    X = sign((sqrt(3)/2)·Vα + (1/2)·Vβ)
    Y = sign(-(1/2)·Vα + (sqrt(3)/2)·Vβ)
    Z = sign(-(sqrt(3)/2)·Vα - (1/2)·Vβ)
    sector = 4·X' + 2·Y' + Z'   (where X'=1 if X<0 else 0, etc.)

  T1T1_Calculate / Tcm_Calculate (two MultiPortSwitch blocks):
    Control port = sector ∈ {1, 2, 3, 4, 5, 6}
    Data ports 1..6 = per-sector T1/T2/Tcm calculation results
    DiagnosticForDefault = 'Error' (default)
```

### §15.2 Failure mode

**At t = 0 transient** (`Vα = Vβ = 0`):
- X = Y = Z = 0
- IEEE 754 `sign(0) = +1` (positive zero treated as non-negative)
- All `X' = Y' = Z' = 1`
- sector = 4·1 + 2·1 + 1 = **7** (out of valid range 1..6)
- MultiPortSwitch defaults to 'Error' → sim startup reports:

```
Error: 'Multiport Switch' block 'mdl/SVPWM_blk/.../T1T1_Calculate' specifies that
the value of the control input (7) cannot match a data port input (1-6).
```

### §15.3 Recommended workaround

Don't modify the shared `pmsm_blocks.slx` library (preserve backward compatibility); modify only the local instance:

```matlab
% Step 1: break the local instance's library link (only affects this instance, not the library source)
set_param([mdl '/SVPWM_blk'], 'LinkStatus', 'inactive');

% Step 2: find all internal MultiPortSwitch blocks and set DiagnosticForDefault = 'None'
ms_blks = find_system([mdl '/SVPWM_blk'], ...
    'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'MultiPortSwitch');
for k = 1:numel(ms_blks)
    set_param(ms_blks{k}, 'DiagnosticForDefault', 'None');
end

% Side effect: when sector=7, MultiPortSwitch uses DataPortForDefault='Last data port' as fallback
% i.e. sector=7 falls back to sector-6 behavior — a 1-2 sample bias at startup, negligible
```

### §15.4 Not recommended

❌ Directly modifying MultiPortSwitch inside the shared `pmsm_blocks.slx` library — affects all users, breaks the library-sharing principle.

❌ Inserting a sgn replacement upstream of SVPWM (`sign(0) → +1` changed to `sign(0) → 0`) — invasive on the library-block input, alters the physical meaning.

❌ Adding a ω_ref soft-start ramp so Vα/Vβ is never zero — treats the symptom, not the cause; floating-point precision can still produce momentary zero vectors.

### §15.5 Self-check

After build + sim:

```matlab
sector_log = ...;   % from sector logger
n_sector7 = sum(sector_log == 7);
n_total   = numel(sector_log);
fprintf('Sector=7 occurrence: %d / %d (%.2f%%)\n', ...
        n_sector7, n_total, 100*n_sector7/n_total);
% Expected: < 0.01% (only 1-2 startup samples)
```

⚠️ If sector=7 occurs > 0.1%, it is not a startup transient — investigate whether the input data is zero for an extended period (e.g. a stalled SMC controller).

---

## Appendix — Scope of this document

✅ This document **includes**:
- R2024b SPS library paths
- Mask field names and types
- Stateflow API names
- Bus signal naming conventions
- Port topology and `add_line` conventions
- Common compile error patterns
- Generic R2024b SOPs (chart configuration, From Workspace inline, Goto/From visibility)
- Port conventions and edge cases for library blocks in `pmsm_blocks.slx`

❌ This document **does NOT include**:
- Reference model file names
- Machine-specific parameter values (Rs / Ld / Lq / Pn etc.)
- Reference model control architectures, topologies, or subsystem boundaries
- MPC / PI / SMC controller gain or weight values
