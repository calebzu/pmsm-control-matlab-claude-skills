---
title: PMSM Building Blocks Manifest
classification: shareable — atomic Simulink blocks for PMSM control studies
placeholder_convention: every mask numeric field is replaced by a workspace variable name (e.g. Vdc_val / Ld / Lq / Pn)
topology: ⚠️ all blocks are **unwired**; layout is arbitrary and carries no design hint
---

# Building Blocks Manifest — `pmsm_blocks.slx`

> **License & trademark notice**: `pmsm_blocks.slx` is a **user-authored** Simulink model
> that references MathWorks Simscape Electrical library blocks (PMSM, Universal Bridge,
> powergui, SVPWM, etc.) **by reference**. It does not embed or redistribute MathWorks
> block implementations — **opening it requires your own licensed MATLAB + Simulink +
> Simscape Electrical installation** (the same way models are shared on MathWorks File
> Exchange). MATLAB® and Simulink® are registered trademarks of The MathWorks, Inc.;
> this is an independent project not affiliated with, endorsed by, or sponsored by MathWorks.

## Usage

```matlab
% Copy a block from the library into your model (recommended — cross-version stable)
add_block('<workspace>/shared/building_blocks/pmsm_blocks.slx/PMSM', ...
          'your_model/PMSM');

% Then inject parameters from your workspace
Rs = ...; Ld = ...;   % etc.

% Or override mask placeholders (mask structural options already set)
```

## Block catalog (14 atomic blocks, no wiring)

| Block name in .slx | Type | Mask configuration (structural) | Workspace placeholders |
|---|---|---|---|
| `powergui` | SPS powergui | SimulationMode=Discrete, SolverType=Tustin/Backward Euler | SampleTime=Ts |
| `DC_Voltage_Source` | SPS DC source | — | Amplitude=Vdc_val |
| `Universal_Bridge` | SPS inverter | Arms=3, Device='IGBT / Diodes', converterType=Inverter, IGBTparameters=[1e-6, 2e-6], SnubberResistance=1e5, SnubberCapacitance=inf, Ron=1e-3 | — |
| `PMSM` | SPS machine | NbPhases=3, FluxDistribution=Sinusoidal, RotorType=Salient-pole, MechanicalLoad='Torque Tm', PresetModel=No, ShowDetailedParameters=on, MeasurementBus=on, MachineConstant='Flux linkage established by magnets (V.s)', RefAngle='Aligned with phase A axis (original Park)', IterativeDiscreteModel='Trapezoidal non iterative' | Resistance=Rs, Inductance=L_unused, dqInductances=[Ld Lq], La=L_leak, Flux=flux, VoltageCst=V_cst_unused, TorqueCst=T_cst_unused, Flat=flat_unused, Mechanical=[J F Pn 0], PolePairs=Pn, TsBlock=-1, TsPowergui=Ts |
| `BusSelector_PMSM` | Signal Routing | OutputSignals='ias,ibs,ics,w,Te' (R2024b PMSM bus signal names) | — |
| `Clark` | SubSystem (3 Fcn blocks) | α=(2/3)·(ia−0.5·ib−0.5·ic), β=(2/3)·(√3/2)·(ib−ic) — amplitude-invariant | — |
| `Plark` | SubSystem (3 Fcn blocks) | d=α·cos(θe)+β·sin(θe), q=−α·sin(θe)+β·cos(θe) — original Park | — |
| `Anti_Park` | SubSystem (2 Fcn + `From "The"` for θ_e — see Port topology) | Vα=cos(θe)·Vd−sin(θe)·Vq, Vβ=sin(θe)·Vd+cos(θe)·Vq — inverse Park (dq → αβ) | — |
| `SVPWM` | SubSystem (5 sub-subsystems: Sector_Calculate / T1T1_Calculate / Tcm_Calculate / XYZ_Calculate / PWM + Repeating Sequence triangle carrier + 2 Constants) | 7-segment SVM, output = 6-bit gate `[Sa+ Sa− Sb+ Sb− Sc+ Sc−]` (pair-adjacent) | Tpwm=Tpwm, Vdc=Vdc_val |
| `ZOH_Tsc` | Zero-Order Hold | — | SampleTime=Tsc |
| `RateLimiter` | Continuous Rate Limiter | — | RisingSlewLimit=slew_rpm, FallingSlewLimit=−slew_rpm |
| `UnitDelay_Tsc` | Discrete Unit Delay | InitialCondition=0 | SampleTime=Tsc |
| `VoltageMeasurement` | SPS sensor | — | — |
| `CurrentMeasurement` | SPS sensor | — | — |

## Port topology (required reading before wiring)

### `Universal_Bridge` (R2024b, Arms=3, IGBT)
- **Inport** (1 port, width=6 double): gate signal `[S1 S2 S3 S4 S5 S6]`
  - 6-bit format **pair-adjacent**: `[Sa_up, Sa_dn, Sb_up, Sb_dn, Sc_up, Sc_dn]`
  - Upper / lower complementary (S2=~S1, S4=~S3, S6=~S5)
- **LConn** (3 ports): AC three-phase output (A/B/C → PMSM stator)
- **RConn** (2 ports): DC input (+/− → DC Voltage Source)

### `PMSM` (Salient-pole + Torque Tm + MeasurementBus=on)
- **Inport[1]**: Tm (load torque input, N·m)
- **Outport[1]**: measurement bus (R2024b signals = `ias, ibs, ics, iqs, ids, vqs, vds, ha, hb, hc, w, theta, Te`; BusSelector picks the subset you need)
- **LConn** (3 ports): stator A/B/C → UB LConn

> ⚠️ **`RefAngle` note**: the catalog row above documents this library's PMSM instance, where `RefAngle` is explicitly pre-set to `'Aligned with phase A axis (original Park)'`. The R2024b `sps_lib` bare default is `'90 degrees behind phase A axis (modified Park)'` — 90° offset from the project Park convention (`shared/formulas/pmsm_formulas.md §1`), a silent failure mode if bare-added without `set_param`. If you must add the bare SPS block instead of copying from this library, follow with `set_param([mdl '/PMSM'], 'RefAngle', 'Aligned with phase A axis (original Park)')`. See the F-CRIT 6 entry in any method skill's `broken_foc_diagnostics.md`.

### `DC_Voltage_Source`
- **LConn[1]** + **RConn[1]**: two physical terminals; polarity follows block orientation (LConn = below/−, RConn = above/+)

### `Clark` / `Plark`
- `Clark`: 3 Inport (`A, B, C`) → 2 Outport (`Alpha, Beta`)
- `Plark`: 3 Inport (`Alpha, Beta, The`) → 2 Outport (`D, Q`)

### `Anti_Park`
- **Inport** (2): `Vq` (port 1), `Vd` (port 2)
- **θ_e via `From` block, `GotoTag = "The"`** — not an Inport. Your build script must publish a matching `Goto` block (tag="The") that exposes the θ_e signal, otherwise compilation fails with "GotoTag 'The' not found"
- **Outport** (2): `Ualpha` (port 1), `Ubeta` (port 2)
- Formula: `Vα = cos(θe)·Vd − sin(θe)·Vq`, `Vβ = sin(θe)·Vd + cos(θe)·Vq`
- Internal Mux1 order: [`From("The")`, `Vd_Inport`, `Vq_Inport`] → Fcn `u[1]/u[2]/u[3]` indices [θ_e, Vd, Vq]
- ⚠️ **Design inconsistency**: `Plark` exposes θ_e as an explicit Inport, while `Anti_Park` uses Goto/From — the two blocks pass θ_e differently and callers must handle each accordingly

### `SVPWM`
- **Inport** (2): `Valpha, Vbeta`
- **Outport** (1, width=6): `pulse` = 6-bit gate signal **pair-adjacent** `[Sa+ Sa− Sb+ Sb− Sc+ Sc−]`, matching `Universal_Bridge` gate convention
- **Internal placeholders**: `Tpwm` (PWM period / triangle carrier period, s) + `Vdc_val` (DC bus voltage, V) — both injected from workspace
- **Internal data flow**: `Valpha/Vbeta` → `XYZ_Calculate` (computes X/Y/Z intermediates) → `Sector_Calculate` (`sign(X)/sign(Y)/sign(Z)` → `N = 4u3 + 2u2 + u1` → sector 1–6) → `T1T1_Calculate` (selects T1/T2 from X/Y/Z and the sector) → `Tcm_Calculate` (per-phase compare values Tcm_a/b/c) → `PWM` (compare against `Repeating Sequence` triangle carrier → 6-bit gate)

## Usage constraints

### ✅ Allowed
- Copy any block from this .slx into your own model
- Add your own MATLAB Function blocks / Constants / Gains / Sums to build the controller logic
- Override mask parameters (but use workspace variable names — don't hard-code numbers)

### ❌ Not allowed
- Don't infer topology from the library's layout or block ordering (the layout is an arbitrary grid)
- Don't assume block-name ordering implies data-flow ordering
- Don't open this .slx as a "complete model" — it has no wiring and reveals no design when opened

## Compliance notes

- All mask numeric values are workspace placeholders (`Vdc_val`, `Rs`, `Ld`, `Lq`, `flux`, `J`, `F`, `Pn`, `slew_rpm`, `Ts`, `Tsc`, `Tpwm`)
- No wiring, no subsystem boundary information exposed
- No controllers included (MPC / PI / SMC design is left to the method skill)
- Block names are generic (`PMSM`, `Universal_Bridge`, `Clark`, `Anti_Park`, `SVPWM`, ...)
- The SVPWM algorithm is standard 7-segment modulation (textbook); only its structural skeleton is included here, no application-specific numeric values
