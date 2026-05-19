# Broken-FOC Diagnostics

When you observe motor stuck near 0 RPM, abc currents DC-locked at a single angle, iq permanently at ±iq_max, or Te varying without rotor accelerating — **stop tuning the controller**. These are signatures of underlying topology bugs, not control-law instability. Tune-once-fix-many: walk through these six F-CRIT items in order before reaching for control gains.

## F-CRIT 1 — Goto / From `TagVisibility` (most common)

**Symptom**: Motor stalled or oscillating around 0 RPM. iq saturated at ±iq_max. abc DC-locked at a single phase angle (NOT sinusoidal). Te varies but rotor doesn't accelerate.

**Root cause**: A cross-subsystem `Goto` (typically `Goto_The` carrying θ_e to Anti_Park's internal `From`) has `TagVisibility='local'` (the default). The `From` block inside the subsystem cannot see the `local` `Goto` in the parent and outputs 0. Anti_Park then computes `inv-Park(θ_e=0)` which is just identity in the lab frame. The FOC closed-loop is silently OPEN-LOOP.

**Fix** (single line):

```matlab
set_param([mdl '/Goto_The'], 'TagVisibility', 'global');
```

**Prevention**: All `add_block` calls creating `Goto` blocks must explicitly pass `'TagVisibility', 'global'`. See [building_blocks.md](building_blocks.md) for the standard pattern.

## F-CRIT 2 — FF Dimensional Correctness

**Symptom**: Numerical waveforms appear plausible but physical behavior diverges from theory predictions.

**Root cause**: Cross-decoupling FF Mux receives θ_e (rad) instead of ω_e (rad/s). Same numerical magnitude but wrong physical quantity.

**Fix**: Use a dedicated `Gain_Pn_omega` block (`ω_m → ω_e = Pn · ω_m`); route `ω_e` (NOT θ_e) into the FF Mux. See [building_blocks.md](building_blocks.md).

## F-CRIT 3 — Vdc / BEMF Tight (≤ 1.2×)

**Symptom**: PI inner loop / SMC + PI cascade chronically saturated. Waveforms look like high-frequency chattering but are actually saturation oscillations. Cannot isolate root cause from control law without first relaxing the saturation.

**Fix**: See [pre_build_grid.md](pre_build_grid.md) for the headroom check. Bump Vdc until ratio ≥ 1.5×. DTC αβ hysteresis is exempt.

## F-CRIT 4 — SVPWM sector=7 startup

**Symptom**: Motor doesn't move for first one or two samples after startup, then starts.

**Root cause**: SVPWM sector calculation isn't stable at t=0; can produce sector=7 (invalid).

**Fix**:

```matlab
set_param([mdl '/SVPWM/sector'], 'StartSector', '1');
% Or in chart logic: if sector==7, sector = 1; end
```

## F-CRIT 5 — External Park vs PMSM Internal dq Frame Divergence

**Symptom**: `id_external` (external Park transform of measured abc currents) differs from `id_internal` (PMSM block bus `BusSel/8`) by 20A+ during transients (e.g., t=0.025s). Steady-state values converge.

**Root cause**: External Park transform uses one convention; PMSM block's internal dq may use a different convention (amplitude vs. power invariant). The two outputs disagree during transients when the discrepancy matters most for control feedback.

**Fix**: Control feedback signals must come from PMSM block's internal dq:
- `id_meas = BusSel/8`
- `iq_meas = BusSel/7`

External Park output is for plotting / debugging only, never for closed-loop feedback.

## F-CRIT 6 — PMSM Block `RefAngle` vs Chart Park Frame Mismatch (silent 90° rotation)

**Symptom**: Motor reaches steady-state at the wrong operating point. Concrete fingerprint observed in practice: target `+1000 RPM` but motor settles at `−186 RPM` after load is applied. `abc` currents are sinusoidal but NOT clean — rhythmic distortion rather than pure sinusoid. `id` drifts large (e.g. `−3.87 A` with `id_ref=0`). `iq` oscillates in a narrow band (e.g. `±3 A`) and cannot track `iq_ref`. Crucially, abc is **not** DC-locked (distinguishes from F-CRIT 1), and `iq` is **not** saturated at `±iq_max` (also distinguishes from F-CRIT 1).

**Root cause**: R2024b `sps_lib/.../Permanent Magnet Synchronous Machine` block defaults `RefAngle = '90 degrees behind phase A axis (modified Park)'`. The project's chart Park formula in `shared/formulas/pmsm_formulas.md §1` is the original Park (`d = α·cos(θe) + β·sin(θe)`). The chart's dq frame and the PMSM block's internal dq frame are rotated by 90° relative to each other. The PI controls `iq → iq_ref` in the chart frame, but the plant interprets the same numeric signal as `id_plant ≈ −iq_chart` in the modified frame. Result: the controller's torque-axis demand drives the flux axis on the plant — motor stalls or drifts in the wrong direction.

**Fix** (single line):

```matlab
set_param([mdl '/PMSM'], 'RefAngle', 'Aligned with phase A axis (original Park)');
```

**Prevention**: either copy the PMSM block from `shared/building_blocks/pmsm_blocks.slx/PMSM` (the library instance has `RefAngle` explicitly pre-set to original Park), OR if adding the bare SPS block directly from `sps_lib`, immediately follow with the `set_param` above. DTC αβ-frame methods are exempt (no Park transform). See [building_blocks.md](building_blocks.md).

## When 18+ "control instability" experiments produce identical chattering

If you've run more than ~10 simulation experiments with similar bang-bang or chatter symptoms, varying control gains without convergence — **stop**. Likely root cause is one of F-CRIT 1–6 above (not the control law). Verify all six F-CRIT before more gain tuning. A single topology bug can invalidate dozens of "control law" experiments.
