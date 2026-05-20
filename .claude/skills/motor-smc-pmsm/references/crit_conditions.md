# SMC CRIT Conditions

Eight non-negotiable conditions. Each is a known silent-failure mode.

## A-CRIT — PD-Type Sliding + STA Reaching Law

v1 baseline architecture is **fixed**: PD-type sliding surface + STA reaching law.

- **Sliding surface MUST be PD-type**: `s = e_w + λ · de_w/dt`
  - Use `simulink/Continuous/Transfer Fcn` with `Numerator='[1 0]'`, `Denominator='[Tf 1]'` for filtered derivative (NOT the raw `du/dt` Derivative block — HF gain unbounded → noise amplification)
  - PI-type `s = e + λ·∫e` is forbidden: integrator winds up during startup; anti-windup clamping changes manifold geometry; Lyapunov stability proof voided

- **Reaching law MUST be STA**: `u = K1 · |s|^0.5 · sgn(s) + K2 · ∫sgn(s)`
  - Second-order SMC: `u(t)` is continuous through `s=0` (because `|s|^0.5 · sgn(s) → 0` smoothly), no discontinuous jump on `iq_ref` → minimal chattering
  - Classic sgn: discontinuous, ripple ∝ `K · Tsc`, by-design chattering. NOT v1 baseline.
  - Boundary-layer sat: degenerates to sgn when `|s| >> φ`; STA is preferred.

See [control_law.md](control_law.md) for the full pseudocode.

## B-CRIT — iq Saturation Mandatory

Saturation block at `iq_ref` output BEFORE feeding to current PI. Limits `±iq_max`.

**Reasoning**: STA's transient `K1 · |s|^0.5` term during reaching phase can reach large amplitudes. Without Saturation, the current PI's `LimitOutput=±Vdc/√3` saturation triggers early, AntiWindup clamping kicks in, current tracking degrades, sliding dynamics non-ideal.

**Diagnostic**: if `Scope_iq` shows `iq_meas` clamped at `iq_max` for > 50 ms continuous, first suspect = Saturation block missing or limit wrong (or motor stuck at lab-frame angle — see G-CRIT).

## C-CRIT — Lyapunov STA Gains + B > 0

Lyapunov V̇ < 0 analysis for STA gives:

```
M = (TL_max + B · ω_max) / J     [disturbance bound, rad/s²]
K1_sta > 1.5 · sqrt(M)
K2_sta > 1.1 · M
```

Build script computes `M` and asserts both inequalities at construction time.

**Plant friction `B > 0` (v1 baseline envelope guard, not a theoretical requirement)**: STA finite-time convergence follows from the K1/K2 conditions above, **independent of plant viscous damping** — a `B = 0` pure-integrator speed loop is relative-degree-1 and STA-controllable in principle. The v1 baseline was validated **entirely with `B > 0`**, so the build script asserts `B > 0` to keep within the validated envelope. Default `B = 0.008` (≈ 26× small-motor hardware spec). If your plant has `B ≈ 0`, re-validate the STA gains for that case rather than assuming the controller requires damping.

**`TL_max` is user-supplied** (necessary prior knowledge for matched-disturbance bound). If user does not know, treat as Required Input gap and ask: "What is the maximum expected load torque for this drive?"

**Diagnostic**: if sliding surface `s` does not reach near-zero within `reaching_time = |s(0)| / η` (typically < 100 ms with PD-type + STA), first suspect = K1/K2 too low or B << 0.

## D-CRIT — InitFcn Self-Contained

All workspace parameters MUST be injected via `set_param(mdl, 'InitFcn', ...)` containing ≥ 20 non-empty `varname = value;` lines. User must double-click `.slx`, press Run, and have it work in a fresh MATLAB session with no prior `assignin('base', ...)`.

```matlab
init_lines = {
    sprintf('Pn=%d;', Pn),
    sprintf('Rs=%g;', Rs),
    sprintf('Ld=%g;', Ld),
    sprintf('Lq=%g;', Lq),
    sprintf('psi_f=%g;', psi_f),
    sprintf('Kt=%g;', 1.5*Pn*psi_f),
    sprintf('J=%g;', J),
    sprintf('B=%g;', B),
    sprintf('Vdc=%g;', Vdc),
    sprintf('Vdc_val=%g;', Vdc),       % SVPWM block internal Constants
    sprintf('Tpwm=%g;', Tsc),          % SVPWM block internal Constants
    sprintf('iq_max=%g;', iq_max),
    sprintf('TL_max=%g;', TL_max),
    sprintf('Tsc=%g;', Tsc),
    sprintf('Ts_sps=%g;', step_size),
    sprintf('lambda_pd=%g;', lambda_pd_settling),
    sprintf('K1_sta=%g;', K1_sta),
    sprintf('K2_sta=%g;', K2_sta),
    sprintf('Tf=%g;', Tf_deriv),
    sprintf('Kp_iq=%g;', Kp_iq),
    sprintf('Ki_iq=%g;', Ki_iq),
    sprintf('Kp_id=%g;', Kp_id),
    sprintf('Ki_id=%g;', Ki_id),
    sprintf('wref_rpm=%g;', omega_ref_rpm),
    sprintf('ramp_time=%g;', ramp_time),
    sprintf('sim_time=%g;', sim_time),
    sprintf('TL_step_t=%g;', TL_step_t),
    sprintf('TL_after=%g;', TL_after),
    'omega_ref_data = [0 0; ramp_time wref_rpm; sim_time wref_rpm];',
};
set_param(mdl, 'InitFcn', strjoin(init_lines, newline));
```

## E-CRIT — Solver `ode3` + ZC OFF

`Sign` block in STA outputs ∈ {-1, 0, +1} (discontinuous). With variable-step solver or zero-crossing detection ON:

- Solver detects sign flip at `s=0`
- Adaptive sub-stepping sub-divides time around the flip
- `Sign` flips again → solver detects again → step explosion (sim freezes or errors)

Configuration:

```
SolverType       = 'Fixed-step'
Solver           = 'ode3'
FixedStep        = 'Ts_sps'  (typical 1 μs)
ZeroCrossControl = 'DisableAll'    % MANDATORY
```

**Diagnostic**: if simulation freezes / takes 10× expected time / errors with "max step size reduced to minimum", first suspect = ZC detection ON or variable-step solver.

## F-CRIT — 14-Channel Logger + 4 Scopes

**Logger** channel order:

```
1: wref_rpm  | 2: wm_rad   | 3: iq_ref   | 4: iq_meas
5: id_meas   | 6: Te       | 7: TL       | 8: s         (SMC-specific)
9: u_sta     | 10: e_w     | 11: de_w/dt   (SMC-specific, channels 8–11 = 4 SMC channels)
12: ia       | 13: ib      | 14: ic
```

⭐ **4 SMC-specific channels (8–11)** are mandatory for offline analysis:

- `s`: sliding surface trajectory (reaching phase + sliding-phase chatter)
- `u_sta`: STA controller output (acceleration command rad/s²; should match `Kt/J · iq_meas` post-saturation)
- `e_w`: speed error (validates ω_ref tracking transient + steady)
- `de_w/dt`: filtered derivative (validates `Tf` choice; HF noise rejection)

**Scopes** — 4 mandatory:

- `Scope_wm_RPM`: 2-input (`wm_rad · 30/π`, `wref_rpm`)
- `Scope_iq`: 2-input (`iq_ref` from Saturation, `iq_meas` from BusSel)
- `Scope_s`: 2-input (`s` from Sum_s, `u_sta` from Sum_sta)
- `Scope_Te`: 2-input (`Te_meas`, `TL`)

## G-CRIT — `Goto_The TagVisibility='global'` MANDATORY

Anti_Park's internal `From "The"` subscribes to `θ_e` via Goto Tag `"The"` published from the parent's `Gain_Pn` output. With default `TagVisibility='local'`, the subsystem-internal `From` cannot see the parent's `Goto` → outputs 0 → Anti_Park computes `inv-Park(θ_e=0)` → identity in lab frame → FOC silently OPEN.

**Silent failure signature**: NO compile error, NO sim error. Numerical metrics produce plausible values. But:

- abc currents DC-lock at single phase angle (NOT sinusoidal)
- `id_meas` large persistent (energy goes to d-axis)
- `Te ≈ 0` or unstable

**Build script MUST**:

```matlab
add_block('simulink/Signal Routing/Goto', [mdl '/Goto_The'], 'Position', [1000 745 1050 775]);
set_param([mdl '/Goto_The'], 'GotoTag', 'The', 'TagVisibility', 'global');
add_line(mdl, 'Gain_Pn/1', 'Goto_The/1', 'autorouting', 'on');
```

**Self-test MUST verify**:

```matlab
goto_blks = find_system(mdl, 'BlockType', 'Goto');
goto_The_present = false;
for k = 1:numel(goto_blks)
  if strcmp(get_param(goto_blks{k}, 'GotoTag'), 'The')
    goto_The_present = strcmp(get_param(goto_blks{k}, 'TagVisibility'), 'global');
    break;
  end
end
assert(goto_The_present, 'G-CRIT: Goto_The missing or TagVisibility != global');
```

## H-CRIT — FF Mux Input is `ω_e` (NOT `θ_e`)

Cross-decoupling FF formulas need electrical angular velocity `ω_e (rad/s)`, NOT electrical position `θ_e (rad)`:

```
Vd_ff = -Lq · ω_e · iq_meas
Vq_ff =  Ld · ω_e · id_meas + ψ_f · ω_e
```

Use a dedicated `Gain_Pn_omega` block to compute `ω_e` from `BusSel/4 (ω_m, rad/s) × Pn`. **Independent block** from `Gain_Pn` (which gives `θ_e` for `Goto_The`):

```matlab
add_block('simulink/Math Operations/Gain', [mdl '/Gain_Pn_omega'], 'Gain', 'Pn');
add_line(mdl, 'BusSel/4', 'Gain_Pn_omega/1', 'autorouting', 'on');
% FF Mux Port 1 wired from Gain_Pn_omega/1 (ω_e), NOT from Gain_Pn/1 (θ_e)
```

**Mixing dimensions**: routing `θ_e` into the FF Mux produces `Vq_ff = θ_e · ψ_f` instead of `ω_e · ψ_f`. At `ω_m = 200` rad/s × `Pn = 4` → `ω_e = 800` rad/s vs `θ_e ≤ 6.28` rad (mod 2π). Magnitude error ratio ≈ 127× → BEMF compensation collapses → PI integrator winds up → saturation → tracking degrades.

**Diagnostic**: if `id_meas` oscillates large under load (instead of small near 0) AND `Vq` command shows position-modulated waveform (sawtooth at ω_e frequency), first suspect = FF Mux input dimensional swap.
