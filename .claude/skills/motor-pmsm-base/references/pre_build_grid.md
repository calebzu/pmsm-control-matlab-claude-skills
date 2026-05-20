# Pre-Build Sanity Grid

Run **before** any PMSM control simulation build. Each failure means stop, fix, and re-check — do not start building until all checks pass.

## Checklist

```
□ Goto/From `TagVisibility` set to 'global' for any cross-subsystem signal (e.g., Goto_The for Anti_Park's θ_e)
□ FF (cross-decoupling feedforward) dimensional check: Mux input is ω_e (rad/s, NOT θ_e in rad)
□ Vdc / BEMF headroom ≥ 1.5×  (formula: Vdc_min = 1.5 · √3 · ω_e_max · ψ_f)
□ SVPWM sector=7 startup handled (break SVPWM_blk library link + internal MultiPortSwitch DiagnosticForDefault='None'; see building_blocks.md)
□ Feedback signals use PMSM internal dq (`BusSel/7 (iqs)`, `BusSel/8 (ids)`), NOT external Park transform output
□ Solver: ode23tb (stiff for power electronics) + ZeroCrossControl='DisableAll' (required for SMC sgn / DTC hysteresis)
□ All `add_block` calls pass explicit `Position` (do not rely on `arrangeSystem` defaults)
□ Stateflow chart: `Script` ≥ 20 lines, sample-time `INHERITED`, dual ZOH where method requires
□ Logger has at least the 14 standard channels (see measurement_logger.md)
□ 4 standard Scopes wired (wm_RPM, iq, abc 3-channel, Te 2-channel with TL+B·ω overlay)
```

→ The first line of every build script (`build_template.m` by convention) should be:

```matlab
assert(check_pre_build_sanity_grid(p), 'Pre-build sanity grid FAILED — fix issues before continuing.');
```

## Vdc Headroom Check (Function Template)

```matlab
function check_vdc_headroom(p)
  omega_e_max = p.omega_max_rpm * 2*pi / 60 * p.Pn;
  BEMF_peak   = omega_e_max * p.psi_f;
  Vdc_min     = 1.5 * sqrt(3) * BEMF_peak;
  ratio       = p.Vdc / Vdc_min;
  if ratio < 1.0
    error('Vdc=%g < required %g (1.5x peak BEMF). Tight headroom triggers PI saturation masquerading as control instability.', p.Vdc, Vdc_min);
  elseif ratio < 1.2
    warning('Vdc/BEMF=%gx tight (< 1.2x). Recommend Vdc>=%g.', ratio, Vdc_min);
  end
end
```

DTC αβ hysteresis is exempt (no PI saturation concept; switching table directly selects vectors).

## Why Each Check Matters

| Check | Failure mode if skipped |
|---|---|
| Goto TagVisibility | Anti_Park reads θ_e=0 → degenerates to lab-frame inv-Park → FOC closed-loop is silently OPEN-LOOP |
| FF dimensional | Numerically looks valid but physically wrong (rad confused for rad/s) → control behavior diverges from theory |
| Vdc/BEMF | PI inner loop saturates against control voltage limit → bang-bang waveforms get misdiagnosed as "control law instability" |
| Sector=7 startup | Motor stuck for first few samples, then starts; obscures true startup transient analysis |
| Internal dq feedback | External Park transform output diverges 20A+ from PMSM internal dq during transients (different convention assumptions) |
| ZeroCrossControl | sgn-based reaching laws and hysteresis switching tables thrash zero-crossing detection → solver stalls or excessive simulation time |

If a check fails partway through a build, do not patch downstream — go back, fix at source, and re-run the full build script. Building on top of a partially-broken plant produces unreliable results that look like control-law bugs.
