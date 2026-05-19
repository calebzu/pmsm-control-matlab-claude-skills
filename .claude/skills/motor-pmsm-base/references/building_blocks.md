# Building Blocks Library SOP

## Atomic Block Library

`shared/building_blocks/pmsm_blocks.slx` provides verified atomic blocks (parameter placeholders, shareable across methods):

- **Park / Clarke / Anti-Park / Anti-Clarke** (amplitude-invariant, matches the conventions in [plant_modeling.md](plant_modeling.md))
- **Anti_Park** subsystem (dq → αβ inverse Park, internally reads θ_e via Goto/From)
- **PMSM block** (R2024b SPS wrapper with explicit mask settings — `RefAngle` pre-set to original Park to match `shared/formulas/pmsm_formulas.md §1`; the `sps_lib` bare default is modified Park, 90° offset — see RefAngle CRIT below and F-CRIT 6)
- **SVPWM** (`Discrete SV PWM Generator` block from `powerlib_extras/Discrete Control Blocks/`)
- Common utilities: Mux 3, Demux, Constant, Gain, Sum, Integrator (with init)

## Reuse Discipline

Subagent build scripts must use `add_block` to copy from the shared library — do **not** rebuild blocks from memory:

```matlab
add_block('<project>/shared/building_blocks/pmsm_blocks.slx/PMSM', ...
          [mdl '/PMSM'], 'Position', [...]);
```

This guarantees:
- Frame conventions match
- Mask defaults are battle-tested
- Future library updates propagate to all skills

## G-CRIT — Goto / From `TagVisibility='global'`

Anti_Park internally reads θ_e from the main model via a `From` block tagged `"The"`. The corresponding `Goto` in the main model **must** have `TagVisibility='global'`:

```matlab
% In build script: explicitly set global visibility
add_block('simulink/Signal Routing/Goto', [mdl '/Goto_The'], ...
          'GotoTag', 'The', ...
          'TagVisibility', 'global');     % MUST be 'global'
```

**Why this matters**: the default `'local'` is a silent failure mode. The `From` inside Anti_Park's subsystem cannot see a `local` Goto in the parent — it returns 0. Anti_Park then computes `inv-Park(θ_e=0)` which is just identity in the lab frame. The whole FOC closed-loop becomes silently open-loop. Symptoms: motor stalled, abc currents DC-locked at single angle, iq permanently at ±iq_max.

A self-test in Phase 9 should assert this:

```matlab
goto_blocks = find_system(mdl, 'BlockType', 'Goto', 'GotoTag', 'The');
if ~isempty(goto_blocks)
  assert(strcmp(get_param(goto_blocks{1}, 'TagVisibility'), 'global'), ...
    'G-CRIT: Goto_The TagVisibility must be global');
end
```

## RefAngle CRIT — PMSM `RefAngle` Frame Alignment

The R2024b `sps_lib/.../Permanent Magnet Synchronous Machine` block defaults `RefAngle = '90 degrees behind phase A axis (modified Park)'`. The project's chart Park convention (`d = α·cos(θe) + β·sin(θe)`, per `shared/formulas/pmsm_formulas.md §1`) is the original Park. The two must match — otherwise the plant dq frame is rotated 90° relative to the chart dq, a silent failure mode (see [broken_foc_diagnostics.md](broken_foc_diagnostics.md) F-CRIT 6).

**Required**: copy from `shared/building_blocks/pmsm_blocks.slx/PMSM` (library instance is pre-set), or set explicitly when adding bare from `sps_lib`:

```matlab
set_param([mdl '/PMSM'], 'RefAngle', 'Aligned with phase A axis (original Park)');
```

A self-test in Phase 9 should assert this:

```matlab
pmsm_blocks = find_system(mdl, 'MaskType', 'Permanent Magnet Synchronous Machine');
if ~isempty(pmsm_blocks)
  assert(strcmp(get_param(pmsm_blocks{1}, 'RefAngle'), ...
                'Aligned with phase A axis (original Park)'), ...
    'RefAngle CRIT: PMSM RefAngle must be original Park');
end
```

DTC αβ-frame methods are exempt (no Park transform).

## SVPWM Library Block

- Source: `powerlib_extras/Discrete Control Blocks/Discrete SV PWM Generator` (NOT inside the project's `pmsm_blocks.slx`).
- Configure: αβ-mode, Pattern #1, mask params for switching frequency `Fc` and sample time `Ts`.
- Output: 6-element binary pulse vector `[Sa_up, Sa_dn, Sb_up, Sb_dn, Sc_up, Sc_dn]` directly connected to `Universal_Bridge` inport.
- **Sector=7 startup fix**: at the first one or two samples after startup, sector calculation may produce sector=7 (invalid), causing the motor not to move briefly. Fix:

```matlab
set_param([mdl '/SVPWM/sector'], 'StartSector', '1');
% Or in chart logic: if sector==7, sector = 1; end
```

## Cross-Decoupling Feedforward (FF) — Dimensional Correctness

dq decoupling FF (amplitude-invariant convention):

- `Vd_ff = -ω_e · Lq · iq`     (note **ω_e** in rad/s, NOT θ_e in rad)
- `Vq_ff =  ω_e · (Ld · id + ψ_f)`

**Build script must use a dedicated block** to compute `ω_e = Pn · ω_m`:

```matlab
add_block('simulink/Math Operations/Gain', [mdl '/Gain_Pn_omega'], ...
          'Gain', 'Pn');
% Connect ω_m → Gain_Pn_omega → FF Mux input
```

Do NOT route θ_e directly into the FF Mux. The dimensions will appear superficially valid (same units as ω_e if you forget time), but the physical behavior will be wrong.
