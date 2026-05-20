# Design Decisions Checklist

For each decision, present the user with options + recommended default, then get explicit confirmation.

| ID | Decision | Recommended | Alternatives |
|---|---|---|---|
| D01 | Switching table | **6-state Sutikno Table 2** (PMSM mandatory) | 8-state Takahashi Table 1 — only for IM-DTC educational comparison; ❌ for PMSM (zero vectors cause flux decay; see [crit_conditions.md §A-CRIT](crit_conditions.md)) |
| D02 | Sector convention | **Convention A**: Sec 1 = `[0°, 60°)`, V1 at α-axis start | Convention B: Sec 1 = `[-30°, 30°)`, V1 at sec-1 mid (rare) |
| D03 | V_k numbering | **Standard**: V1=[100]@0°, V2=[110]@60°, …, V6=[101]@300° | Reference's non-standard permutation (skill canon overrides) |
| D04 | Te feedback source | **αβ cross-product estimator**: `Te = (3/2)·Pn·(ψ_α·iβ − ψ_β·iα)` | `'plant'` reads PMSM bus Te directly (educational; masks estimation errors) |
| D05 | Sector detection | **`atan2(ψ_β, ψ_α) + mod 2π + floor / (π/3) + 1`** | Binary encoding (faster but less readable) |
| D06 | Speed PI design | **SO (Symmetric Optimum) via `pi_design('SO', J, 1, T_eq, a)`** with `a=4`. Pass `Kt=1` (DTC native; plant has no torque constant). | PZC ❌ unusable for B=0 plant (slow disturbance pole); 2nd-order standard form ❌ ignores inner-loop dynamics |
| D07 | T_eq factor | **15 · Tsc** (DTC) | 5·Tsc ❌ FCS-MPC default; gives wm 33% overshoot for DTC. Acceptable range 10–20·Tsc |
| D08 | Anti-windup on Speed PI | **OFF** (v1 baseline) | Clamp / back-calculation (production must add) |
| D09 | Flux integrator | **Forward Euler with persistent state, init `[ψ_f, 0]`** | LPF replacement (drift-compensated; v2+); SOGI / Kalman / sliding-mode observer (out of v1) |
| D10 | Hysteresis bands | **Reverse-calculate from `fs_max`** ([hb_sizing.md](hb_sizing.md)) | Percentage fallback (`HB_T = 0.075·T_max`, `HB_ψ = 0.025·ψ_ref`) only when `fs_max` is unknown |
| D11 | Hysteresis levels | **2-level** (`C_ψ ∈ {0,1}`, `C_T ∈ {0,1}`) | 3-level torque (reverse-braking; out of v1) |
| D12 | Parameter injection | **InitFcn `set_param` + mask field var-name refs** ([crit_conditions.md §E-CRIT](crit_conditions.md)) | Hard-coded literals in mask ❌ drifts from chart |
| D13 | DTC implementation | **Single MATLAB Function chart** (flux integrator + magnitude + sector + 2× hysteresis + table + V_k decoder) | S-functions ×2 + LUT ×3 (legacy reference style; harder to debug) |
| D14 | Chart sampling | **`SampleTime='-1'` INHERITED + 5-input ZOH @ Tsc + 9-output ZOH @ Tsc** ([crit_conditions.md §D-CRIT](crit_conditions.md)) | DISCRETE+Tsc ❌ deadlocks long stateful charts |
| D15 | Chart output gate format | **6×1 column `[Sa; ~Sa; Sb; ~Sb; Sc; ~Sc]`** (pair-adjacent for UB Arms=3) | Row vector ❌ Inport mismatch |
| D16 | Solver | **Fixed-step ode3, FixedStep = step_size = 1 µs** | ode23tb fixed-step (slower); Variable-step ❌ usually conflicts with IGBT switching |
| D17 | Logger | **Mux 19 ch + To Workspace, SampleTime=Tsc, SaveFormat=StructureWithTime** | Outport+`sim()` return (more boilerplate); Scope only ❌ no `.mat` persistence |
| D18 | Scopes | **4 Scopes mandatory** (wm_RPM / Te / mag_psi / αβ XY) ([crit_conditions.md §F-CRIT](crit_conditions.md)) | Logger only ❌ requires external plot scripts; loses one-click visual health check |
| D19 | Voltage measurement wiring | **Phase-to-DC-neg single-ended**: `VoltageMeasurement.LConn1` ↔ `UB.LConn{k}`, `VoltageMeasurement.LConn2` ↔ `DC.LConn1` | Phase-to-phase differential (3 measurements + Δ logic; out of v1) |
| D20 | Chart re-add SOP | **Delete + re-add chart at end of build** | Skip ❌ heavy `add_block` can silently corrupt chart attributes |
| D21 | Layout | **Position bands + `arrangeSystem('FullLayout')` fallback** | Auto-routing only ❌ produces overlapping spaghetti |
