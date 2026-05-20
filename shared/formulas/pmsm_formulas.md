# PMSM Modeling Formulas

A reference set of PMSM (permanent-magnet synchronous motor) modeling and control formulas for `motor-pmsm-base` and method skills (`motor-fcs-mpc`, `motor-dtc-pmsm`, `motor-smc-pmsm`). The document is organized:

- **§0–§7**: PMSM plant equations (dq-frame voltage, flux, torque, mechanical, kinematic)
- **§A**: Outer-loop speed PI design (4 methods + selection decision tree)
- **§B**: DTC controller-side formulas (αβ flux estimator, sector decoding, switching table, hysteresis comparators)
- **§C**: SMC speed-loop control law (PD-type sliding + Super-Twisting, Lyapunov gain conditions, chattering)

**Conventions**:
- **Park transform**: amplitude-invariant (2/3 coefficient), per §1
- **dq frame**: rotor reference frame, d-axis aligned with rotor PM flux ψf, per §0
- **Motor convention**: stator current flowing into the motor is positive
- **Symbols**: `ψf` in docs, `psif` in code; `wm` for mechanical speed (rad/s); `B` for viscous friction
- **Primary form**: s-domain transfer function (time-domain kept only when it clarifies physical meaning)

---

## §0 Three core conventions

### A2 Rotor reference frame
- The dq coordinate system **rotates with the rotor**: `we = pn · wm` (pn = pole pairs, wm = mechanical speed)
- In steady state, `id`, `iq` are DC quantities — necessary condition for zero steady-state PI error

### A3 d-axis aligned with rotor PM flux ψf
- At `θe = 0`, the d-axis aligns with the N-pole flux ψf
- Consequence: in `ψd = Ld·id + ψf`, ψf appears only on the d-axis
- In practice: encoder offset calibration of θe zero point

### A4 Motor convention
- Stator current flowing **into** the motor is positive
- `Te > 0` → rotor acceleration
- BEMF appears with a negative sign in the q-axis voltage equation (`-we·ψf`)

---

## §1 Park & Clarke transforms — amplitude-invariant (2/3)

### Clarke (abc → αβ)
```
α  = (2/3) · (ia - ib/2 - ic/2)
β  = (ib - ic) / √3
i0 = (1/3) · (ia + ib + ic)         [= 0 in balanced 3-phase]
```

### Park (αβ → dq, pure rotation)
```
d =  α·cos(θe) + β·sin(θe)
q = -α·sin(θe) + β·cos(θe)
```

### Inverse Park (dq → αβ)
```
Uα = cos(θe)·Vd - sin(θe)·Vq
Uβ = sin(θe)·Vd + cos(θe)·Vq
```

### Inverse Clarke (αβ → abc)
```
ia = α
ib = -α/2 + (√3/2)·β
ic = -α/2 - (√3/2)·β
```

### Key coefficient
- Clarke outer coefficient **2/3** → amplitude-invariant convention (magnitude preserved)
- This causes the **1.5·pn** factor in §2/§3 voltage and §5 torque
- Power-invariant convention (`√(2/3)` coefficient) drops the 1.5 in Te

---

## §2 d-axis voltage equation

```
s-domain:    id(s) = (Vd(s) + we·Lq·iq(s)) / (Ld·s + Rs)
time-domain: Ld · did/dt = Vd - Rs·id + we·Lq·iq
```

| Symbol | Meaning | Unit |
|---|---|---|
| Vd, id | d-axis voltage, current | V, A |
| Rs | stator resistance | Ω |
| Ld, Lq | d/q-axis inductance | H |
| we | electrical angular speed = pn · wm | rad/s |
| iq | q-axis current (from §3) | A |

### Physical meaning
- `Ld·did/dt`: self-inductance voltage drop from d-axis flux derivative
- `Rs·id`: resistive drop
- `we·Lq·iq`: motional EMF coupling from q-axis to d-axis via rotation (algebraic product of the rotating-frame transform, not new physics)

### Derivation outline
1. abc-frame KVL: `Va = Rs·ia + dψa/dt`
2. Apply Park transform (A1: amplitude-invariant 2/3)
3. Chain rule on θe(t) produces the `-we·ψq` motional EMF term
4. Substitute flux `ψd = Ld·id + ψf` (A5: linear magnetic) with ψf time-invariant (A6)

### Suspended assumptions (resolved later)
- **A1** amplitude-invariant Park (2/3) → §1
- **A2** rotor reference frame → §0
- **A3** d-axis aligned with ψf → §0
- **A4** motor convention → §0
- **A5** linear magnetic circuit → §4
- **A6** ψf time-invariant → §4

---

## §3 q-axis voltage equation

```
s-domain:    iq(s) = (Vq - we·Ld·id - we·ψf) / (Lq·s + Rs)
time-domain: Lq · diq/dt = Vq - Rs·iq - we·Ld·id - we·ψf
```

| Symbol | Meaning | Unit |
|---|---|---|
| Vq, iq | q-axis voltage, current | V, A |
| ψf | permanent-magnet flux linkage | V·s (= Wb) |
| Others | same as §2 | — |

### Physical meaning
- `Rs·iq`: resistive drop
- `we·Ld·id`: motional coupling from d-axis to q-axis (**cross-coupling**, target of FOC feedforward decoupling)
- `we·ψf`: **back-EMF (BEMF)** — the only term originating from PM flux. When `wm > 0`, Vq must first overcome BEMF before iq is produced → physical origin of motor power output

### Duality with §2

| Term | §2 (d-axis) | §3 (q-axis) |
|---|---|---|
| Self-inductance | `+Ld·did/dt` | `+Lq·diq/dt` |
| Resistive | `+Rs·id` | `+Rs·iq` |
| Rotational coupling | `-we·Lq·iq` | `+we·Ld·id` (sign reversed) |
| Flux coupling | — | `+we·ψf` (BEMF) |

### Suspended assumptions
Same A1–A6 as §2.

---

## §4 Flux equations ψd, ψq — linear magnetic circuit

```
ψd = Ld · id + ψf       (d-axis flux = self-induced + PM)
ψq = Lq · iq            (q-axis flux = self-induced only)
```

### Physical meaning
- **ψd**: two superposed sources — `Ld·id` (stator) + `ψf` (constant from PM)
- **ψq**: only stator `Lq·iq` — PM flux projects to zero on q-axis (by A3)

### Suspended assumptions

| ID | Content | Failure condition | TODO |
|---|---|---|---|
| **A5a** | Ld linear (independent of id) | Magnetic saturation | Use lookup `Ld(id, iq)` for MTPA / field-weakening |
| **A5b** | Lq linear (independent of iq) | Same as above | Same |
| **A5c** | ψf constant | VFPM / thermal / aging | Parameter-identification studies relax this |
| **A5d** | No d-q cross-coupling inductance | Asymmetric rotor geometry | Standard IPMSM OK; exotic rotors need re-check |
| **A5e** | No zero-sequence coupling | 3-phase unbalanced | Balanced systems OK |

---

## §5 Electromagnetic torque Te — full IPMSM form

```
Te = 1.5 · pn · [ψf · iq + (Ld - Lq) · id · iq]
       └─PM torque──┘   └──reluctance torque──┘
```

| Symbol | Meaning | Unit |
|---|---|---|
| Te | electromagnetic torque | N·m |
| 1.5 | amplitude-invariant Park 3/2 coefficient | — |
| pn | pole pairs | — |

### Physical meaning
1. **PM torque `1.5·pn·ψf·iq`**: interaction of stator current iq with PM flux — primary source of torque
2. **Reluctance torque `1.5·pn·(Ld-Lq)·id·iq`**: present only when Ld ≠ Lq
   - For `Ld < Lq` (typical IPMSM), choosing `id < 0` (field-weakening injection) makes reluctance torque add to PM torque → MTPA mathematical foundation
   - For SPMSM (`Ld = Lq`), the reluctance term is automatically zero

### Derivation outline (power-balance approach)
1. Input electrical power: `P_in = (3/2)·(Vd·id + Vq·iq)` (3/2 from amplitude-invariant Park)
2. Substitute §2/§3, separate copper loss `Rs·(id² + iq²)` and magnetic-energy rate `Ld·id·did/dt + Lq·iq·diq/dt`
3. Remainder is mechanical power `P_mech = (3/2)·we·[(Ld-Lq)·id·iq + ψf·iq]`
4. From `P_mech = Te·wm` and `we = pn·wm` → solve Te

### Design note
The full IPMSM form is kept as the plant default. SPMSM is recovered automatically when `Ld = Lq`. Plant models should not embed an `id = 0` controller-strategy assumption — MTPA / high-performance IPMSM applications need the reluctance term.

### Suspended assumptions
Depends on §2/§3 A1–A6; no new assumptions.

---

## §6 Mechanical equation — full form with B and TL

```
s-domain:    wm(s) = (Te - TL) / (J·s + B)
time-domain: J · dwm/dt = Te - B · wm - TL
```

| Symbol | Meaning | Unit |
|---|---|---|
| wm | mechanical angular speed | rad/s |
| J | rotor moment of inertia | kg·m² |
| Te | electromagnetic torque (from §5) | N·m |
| B | viscous friction coefficient | N·m·s |
| TL | load torque (external input) | N·m |

### Physical meaning
- `J·dwm/dt` = rotor angular acceleration × inertia
- `B·wm` = viscous friction damping
- `TL` = external load torque (input variable, time-varying allowed)
- Equilibrium: `Te = B·wm + TL`

### Design note
Full form kept (B and TL retained). `B = 0` / `TL = 0` can be activated via parameterization, but the model structure must accommodate disturbance-rejection studies.

---

## §7 Angle relations — pure kinematics

```
s-domain:
  θm(s) = wm(s) / s
  θe(s) = pn · θm(s) = pn · wm(s) / s
  we(s) = s · θe(s) = pn · wm(s)

time-domain:
  dθm/dt = wm
  dθe/dt = we = pn · wm
  θe     = pn · ∫ wm dt
```

| Symbol | Meaning | Unit |
|---|---|---|
| θm | rotor mechanical position | rad (mechanical) |
| θe | electrical position (for Park) | rad (electrical) |
| pn | pole pairs | — |

### Physical meaning
- `θe = pn · θm`: each mechanical revolution covers pn electrical cycles (each pole pair = one N–S–N field cycle)
- Park transform must use θe, not θm

### Implementation notes
- **Angle wrap-around**: `mod(θe, 2π)` for numerical precision (controller must do this; plant model can omit)
- **Encoder calibration**: physical encoder gives θm; θ_offset calibration aligns to d-axis (A3 convention)

---

## Naming conventions

| Quantity | Standard | Alternative |
|---|---|---|
| Mechanical angular speed | **wm** | wr |
| Viscous friction | **B** | Bf |
| Permanent-magnet flux | **ψf** / `psif` | phif |

---

# §A — Outer-Loop Speed PI Design (4 methods)

For a cascaded speed/current control architecture, the outer speed PI converts speed error to `iq_ref`; the inner controller tracks `iq_ref`. From the outer-loop viewpoint, the inner loop is approximated as a first-order lag (time constant `T_eq`), and the mechanical equation contributes the integrator (when B is small).

**Common pitfall**: plugging `wn, ζ` into `Kp = 2ζ·wn·J/Kt, Ki = wn²·J/Kt` while ignoring inner-loop dynamics. §A.3 below gives a decision tree to avoid this.

## §A.1 Speed-loop plant model

```
ωm(s) / iq_ref(s) = Kt / (J·s · (T_eq·s + 1))            (B = 0)
ωm(s) / iq_ref(s) = Kt / ((J·s + B) · (T_eq·s + 1))      (B > 0)
```

| Symbol | Meaning | Unit |
|---|---|---|
| `Kt` | torque constant = `1.5·pn·ψf` (SPMSM) or `1.5·pn·[ψf + (Ld−Lq)·id]` (IPMSM) | N·m/A |
| `J` | rotor inertia | kg·m² |
| `B` | viscous friction | N·m·s |
| `T_eq` | inner-loop equivalent time constant | s (typically ≈ 5·Tsc for digital current loop) |

### Simplifying assumptions
- **A7 (inner-loop simplification)**: iq → iq_ref as first-order lag `1/(T_eq·s+1)`. Failure: iq_ref hits PI saturation / MPC cost weight too small / TL persistently saturates iq / IPMSM cross-coupling `(we·Tsc·Ld/Lq)·id` at high speed
- **A8 (id_ref = 0 simplification)**: Kt takes SPMSM form. For IPMSM MTPA with `id_ref ≠ 0`, Kt must include reluctance contribution

## §A.2 Four methods compared

### Method 1 — Pole-Zero Cancellation (PZC)

**Plant**: `B > 0`, speed loop = `Kt / [(J·s + B)·(T_eq·s + 1)]`

**Idea**: choose PI zero `(τ_i·s + 1)` (`τ_i = Kp/Ki`) to cancel plant pole `(J·s + B)`.
```
Kp / Ki = J / B   →   Ki = Kp · (B/J)
```
**Bandwidth**: open-loop `Kp·Kt / (B · (T_eq·s+1) · s)`, crossover `ωc = Kp·Kt/B`. Choose `ωc ≤ inner BW / 5` (typical 30–100 Hz).

**Pros**: closed loop is first-order, no overshoot, phase margin ≈ 90°.
**Cons**: depends on accurate B (real-world ±20% error common); **fails when B = 0** (no pole to cancel).

### Method 2 — Magnitude Optimum (MO)

**Plant**: first-order + delay `K / [(τ·s + 1)·(T_eq·s+1)]`. **Not applicable to integrator-type plants.**

**Idea**: maximally flat closed-loop magnitude at low frequency → `ζ ≈ 0.707`, overshoot ≈ 4.3%.
**Kessler standard** (when `τ ≫ T_eq`): `Kp = τ/(2·K·T_eq)`, `Ki = 1/(2·K·T_eq)`.

**Note**: not applicable to B=0 speed loops (integrator type). Suits current loops (first-order RL plant) or speed loops with significant B (PZC usually preferred then).

### Method 3 — Symmetrical Optimum (SO) ⭐ recommended default

**Plant**: integrator + delay `K / [s·(T_eq·s + 1)]` (B=0 speed loop matches exactly).

**Kessler standard (B=0)**:
```
Kp  = J / (a · Kt · T_eq)
Ki  = J / (a³ · Kt · T_eq²)
τ_i = Kp / Ki = a² · T_eq
```

**SO factor `a`** controls damping:

| a | ζ_eq | Overshoot | Scenario |
|---|---|---|---|
| 2 | ≈ 0.5 | ~16% | Aggressive |
| 3 | ≈ 0.6 | ~10% | Balanced |
| **4** | **≈ 0.71** | **~7%** | **Default** (Kessler standard, max phase margin ≈ 36°) |
| 6 | ≈ 0.85 | ~3% | Conservative |

**Crossover**: `ωc ≈ 1/(a·T_eq)`. **2% settling time**: `t_s ≈ 4·a·T_eq`.

**Derivation outline**:
1. Plant `G_p(s) = K/(s·(T_eq·s+1))`, `K = Kt/J`
2. PI `G_c(s) = Kp·(τ_i·s+1)/(τ_i·s)`
3. Open loop `L(s) = Kp·K·(τ_i·s+1) / [s²·τ_i·(T_eq·s+1)]`
4. SO condition: at ωc, low-frequency zero `1/τ_i` and high-frequency pole `1/T_eq` are symmetric on log axis → `ωc² = 1/(τ_i·T_eq)`
5. Choose `τ_i = a²·T_eq` → `ωc = 1/(a·T_eq)`
6. `|L(jωc)| = 1` → `Kp = J/(a·Kt·T_eq)`, `Ki = Kp/(a²·T_eq) = J/(a³·Kt·T_eq²)`

**Pros**: closed-form for B=0 plants, only `(J, Kt, T_eq, a)` needed; crossover frequency tied to inner-loop constant.
**Cons**: `a=4` gives ~7% overshoot; for sensitive applications increase `a`.

### Method 4 — IMC / Direct Synthesis

**Plant**: any stable `G_p(s)`.

**Idea**: specify desired closed-loop `H(s) = 1/(λ·s + 1)` → solve `G_c(s) = G_p(s)⁻¹ · H(s)/(1−H(s))`.

**B=0 speed loop (ignoring T_eq)**:
```
Kp = J / (λ · Kt)
Ki = 0   (pure proportional — has steady-state error under step TL)
```
Practical IMC speed loops augment with an integral disturbance model; the formula becomes more complex.

**With T_eq**: `G_p(s) = Kt/[J·s·(T_eq·s+1)]`, target `H(s) = 1/(λ·s+1)²` (double pole) → PI controller + first-order filter.

**Pros**: general, λ directly sets closed-loop bandwidth.
**Cons**: B=0 needs second-order target → formula complex; `λ ≈ 5·T_eq` is rule of thumb.

## §A.3 Selection decision tree

```
Speed-loop PI design
│
├── B > 0 and accurately measured?
│   ├── Yes → Method 1 (PZC)
│   │         Ki/Kp = B/J; choose ωc ≤ inner BW / 5
│   │
│   └── No (B = 0 or B uncertain) → next
│
├── Plant is integrator + delay (B = 0 speed loop)?
│   └── Yes → Method 3 (SO) ⭐ default
│             Kp = J/(a·Kt·T_eq), Ki = J/(a³·Kt·T_eq²)
│             a = 4 standard; T_eq ≈ 5·Tsc
│
└── Need single-parameter bandwidth (IMC)?
    └── Yes → Method 4 (IMC)
              λ ≈ 5·T_eq
```

**Anti-patterns**:
- ❌ Apply PZC to B=0 plant — no pole
- ❌ Apply MO to integrator-type speed loop — formula doesn't fit
- ❌ Plug `wn, ζ` into `Kp = 2ζ·wn·J/Kt, Ki = wn²·J/Kt` ignoring inner loop

## §A.4 Unit conversion (rad/s ↔ RPM)

PI can be implemented in rad/s (theory) or RPM (engineering). SO formula is in rad/s natively:
```
Kp_rad = J / (a · Kt · T_eq)             [A·s/rad]
Ki_rad = J / (a³ · Kt · T_eq²)           [A/rad]
```

Convert to RPM (`ω_ref_rpm − ω_meas_rpm` → `iq_ref [A]`):
```
Kp_rpm = Kp_rad · (π / 30)               [A/RPM]
Ki_rpm = Ki_rad · (π / 30)               [A/(RPM·s)]
```

**Why π/30**: `1 RPM = π/30 rad/s = 0.1047 rad/s`. Input scales by π/30, so gain scales by π/30 to preserve closed-loop response.

---

# §B — Direct Torque Control law (DTC-PMSM)

Controller-side formulas (αβ-frame estimator + sector decoding + switching table + hysteresis comparators) for DTC-PMSM. Plant physics still uses §0–§7.

## §B.1 αβ stator-flux estimator (voltage-model integrator)

```
component form:
  ψ_sα(t) = ψ_sα(0) + ∫₀^t (u_sα - R_s · i_sα) dτ
  ψ_sβ(t) = ψ_sβ(0) + ∫₀^t (u_sβ - R_s · i_sβ) dτ

space-vector compact form (equivalent):
  ψ_s^s(t) = ψ_s^s(0) + ∫₀^t (u_s^s - R_s · i_s^s) dτ
```
where `ψ_s^s = ψ_sα + j·ψ_sβ`, `u_s^s = u_sα + j·u_sβ`, `i_s^s = i_sα + j·i_sβ`.

| Symbol | Meaning | Unit |
|---|---|---|
| ψ_sα, ψ_sβ | αβ-frame stator flux | V·s (Wb) |
| u_sα, u_sβ | αβ-frame stator voltage (reconstructed from V_dc + SVPWM duties) | V |
| i_sα, i_sβ | αβ-frame stator current (from §1 Clarke) | A |
| R_s | stator resistance | Ω |
| ψ_sα(0), ψ_sβ(0) | initial flux (hardware: `ψf, 0` at θe=0; simulation: 0 acceptable) | V·s |

### Physical meaning
- **Controller-side estimator** (not plant model). Reconstructs stator flux from measured αβ voltage + current + known R_s
- Derived by rearranging and integrating αβ-frame KVL `u = R·i + dψ/dt`
- Feeds the downstream pipeline: magnitude/position (§B.3) → sector decode (§B.4) → vector select (§B.5–B.6)

### Derivation outline
1. abc KVL `u_a = R_s·i_a + dψ_a/dt` (similarly for b, c)
2. Apply §1 Clarke (LTI transform preserves equality): `u_α = R_s·i_α + dψ_α/dt`
3. Rearrange and integrate: `ψ_α(t) − ψ_α(0) = ∫(u_α − R_s·i_α) dτ`

### Implementation notes
- **Measurement source**: `u_sα/β` reconstructed from V_dc + SVPWM duties (not directly measured); `i_sα/β` from Clarke block
- **Simulink**: Clarke + Sum + Gain (R_s) + Integrator
- **Discrete**: trapezoidal or forward-Euler integration at sample T_s

### Suspended assumptions

| ID | Content | Failure / TODO |
|---|---|---|
| **A9** | Three-phase stator resistance symmetric (`R_a = R_b = R_c`) | Winding variance / thermal asymmetry → lookup `R_s(temp)` |
| **A10** | Zero-sequence flux `ψ_0 = 0` | Single-phase fault / grounded neutral → augment ψ_0 channel |
| **A11** | **Pure integrator, no DC drift** ⭐ | R_s mismatch + sensor bias → drift, severe at low speed. Remedies: LPF replacing pure integrator (with magnitude-phase comp) / HPF post-processing / closed-loop observer (SOGI / Kalman / SMO). Advanced implementations must address; baseline uses pure integration |

---

## §B.2 αβ electromagnetic torque (cross-product form)

```
Te = (3/2) · Pn · (ψ_sα · i_sβ - ψ_sβ · i_sα)
```

### Physical meaning
- `ψ_sα·i_sβ − ψ_sβ·i_sα` = z-component of αβ-frame flux × current cross product
- DTC computes Te directly from §B.1 ψ_sα/β + Clarke i_sα/β, **without Park, without θe** — core simplification of DTC vs FOC

### Derivation outline
1. From §5 Te(dq), substitute §4 fluxes:
   ```
   ψd·iq − ψq·id = (Ld·id + ψf)·iq − Lq·iq·id = ψf·iq + (Ld − Lq)·id·iq
   Te = (3/2)·Pn·(ψd·iq − ψq·id)             ⋆ dq cross-product form
   ```
2. Cross product is rotation-invariant under amplitude-invariant Park. Algebraic expansion (`sin²+cos²=1`) yields:
   ```
   ψd·iq − ψq·id = ψ_sα·i_sβ − ψ_sβ·i_sα
   ```
3. Substitute back into ⋆ to get §B.2.

### SPMSM degeneration
When `Ld = Lq`: `Te = (3/2)·Pn·ψf·iq` (consistent with §5 SPMSM form).

### Implementation notes
- **Simulink**: 3 Product blocks (`ψ_sα·i_sβ`, `ψ_sβ·i_sα`) + 1 Sub + 1 Gain `(3/2)·Pn`
- **Inputs**: ψ_sα/β from §B.1, i_sα/β from Clarke block
- **No θe needed** — DTC works entirely in stationary frame

### Suspended assumptions
- Depends on §1 amp-invariant Park / §4 A5a–e / §5 IPMSM form
- §B.1 A9/A10 don't affect §B.2 (no R_s in Te formula; zero-sequence doesn't produce torque)
- Estimated `ψ_sα/β` precision depends on A11 drift — Te estimation indirectly affected

---

## §B.3 Stator flux magnitude + position (polar conversion)

```
|ψs| = √(ψ_sα² + ψ_sβ²)
θ_ψ  = atan2(ψ_sβ, ψ_sα)
```

| Symbol | Meaning | Unit |
|---|---|---|
| `|ψs|` | stator flux vector magnitude | V·s |
| θ_ψ | stator flux argument in αβ plane (4-quadrant) | rad ∈ (−π, π] |

### Physical meaning
- Rectangular `(ψ_sα, ψ_sβ)` → polar `(|ψs|, θ_ψ)`. Pure vector geometry, no motor physics
- DTC uses:
  - `|ψs|` → compare to ψ_ref → flux hysteresis (§B.7)
  - `θ_ψ` → sector decoding (§B.4)

### Implementation notes
- **`|ψs|`**: 2 Square blocks + Sum + Sqrt (or `Math Function` block in `magnitude^2` mode)
- **`θ_ψ`**: 1 `atan2` block (Simulink → Math Operations → Trigonometric Function, `atan2` mode)
- **Discrete**: θ_ψ range `(−π, π]`; sector decoder must handle wrap-around (add π offset to get `[0, 2π)`)
- **Numerical**: `ψ_sα/β` near zero (startup / very low speed) → atan2 unstable; needs special handling (delayed hysteresis activation / dedicated low-speed startup)

### Suspended assumptions

| ID | Content | Failure / TODO |
|---|---|---|
| **A12** | `atan2` is **4-quadrant version** | Wrong choice of single-quadrant `atan` → wrong sector |
| **A13** | At DTC startup, `ψ_sα/β` established (non-zero) | Power-up `ψ_sα/β = 0` → atan2 undefined. Remedy: startup delay `t_init` 10–50 ms, or apply initial magnetizing voltage |

---

## §B.4 6-sector decoding (`θ_ψ → S ∈ {I..VI}`)

```
θ_ψ_norm = mod(θ_ψ + 2π, 2π)             // (−π, π] → [0, 2π)
S = floor(θ_ψ_norm / (π/3)) + 1           // ∈ {1..6}
```

### Sector boundaries (Sector I starts at V1, the positive α-axis)

| Sector | Angle range | Vectors |
|---|---|---|
| **I** | 0° — 60° | V1 → V2 |
| **II** | 60° — 120° | V2 → V3 |
| **III** | 120° — 180° | V3 → V4 |
| **IV** | 180° — 240° | V4 → V5 |
| **V** | 240° — 300° | V5 → V6 |
| **VI** | 300° — 360° | V6 → V1 |

### Vector diagram (αβ plane, V1 along positive α-axis)
```
              β (90°)
              ↑
       V3──────|──────V2
      [010]    |    [110]
      (120°)   |    (60°)
         ╲    Sec    ╱
       Sec   II      Sec
        III   |       I
            ╲ | ╱
              ●  V0/V7
   V4 ───────●───────V1     → α (0°)
   [011]    [000]   [100]
   (180°)   [111]   (0°)
            ╱ | ╲
       Sec    |       Sec
        IV  Sec V      VI
         ╱    |    ╲
      (240°)  |   (300°)
       V5─────|─────V6
      [001]   |   [101]
              ↓
```

### Physical meaning
- Discretizes continuous θ_ψ into 6 sector indices
- DTC switching table (§B.6) is indexed by `(C_ψ, C_T, S)` → select V_k
- **Sector = 60° slice where flux vector currently resides** — determines which adjacent vectors push the flux

### Suspended assumption

| ID | Content |
|---|---|
| **A14** | Sector I starts at V1 (positive α-axis). If multiple 6-vector methods coexist in the same project, sector numbering must remain consistent |

---

## §B.5 Eight switching vectors — αβ projection table

| Vector | [Sa Sb Sc] | αβ angle | u_α | u_β | Nature |
|---|---|---|---|---|---|
| **V0** | [0 0 0] | — | 0 | 0 | zero (all lower on) |
| **V1** | [1 0 0] | 0° | (2/3)·V_dc | 0 | non-zero |
| **V2** | [1 1 0] | 60° | (1/3)·V_dc | (√3/3)·V_dc | non-zero |
| **V3** | [0 1 0] | 120° | −(1/3)·V_dc | (√3/3)·V_dc | non-zero |
| **V4** | [0 1 1] | 180° | −(2/3)·V_dc | 0 | non-zero |
| **V5** | [0 0 1] | 240° | −(1/3)·V_dc | −(√3/3)·V_dc | non-zero |
| **V6** | [1 0 1] | 300° | (1/3)·V_dc | −(√3/3)·V_dc | non-zero |
| **V7** | [1 1 1] | — | 0 | 0 | zero (all upper on) |

Compact polar form: `Vk = (2/3)·V_dc · exp(j·(k-1)·π/3)`, `k = 1..6`; `V0 = V7 = 0`.

| Symbol | Meaning | Unit |
|---|---|---|
| Sa, Sb, Sc | upper-switch state (1=on, 0=off; lower complementary) | binary |
| V_dc | DC bus voltage | V |
| (2/3)·V_dc | non-zero vector magnitude (from Clarke 2/3 coefficient) | V |

### Physical meaning
- 8 switch combinations: 6 non-zero (uniformly 60° apart in αβ), 2 zero
- DTC selects **one vector per sample**; the vector magnitude `(2/3)·V_dc` comes from amplitude-invariant Clarke

### Derivation outline
Phase-to-neutral voltages: `V_aN = (2·Sa − Sb − Sc)·V_dc/3` (similarly for b, c). Apply §1 Clarke → αβ projection.

### Suspended assumption

| ID | Content |
|---|---|
| **A15** | Ideal switches (no dead time / no on-resistance / no switching delay). Real IGBT: dead time 2–5 μs, on-drop ~1.5 V → dead-time compensation needed in advanced DTC |

---

## §B.6 Hysteresis switching table — 8-state classical (IM-DTC / general reference)

> ⚠️ **PMSM MUST use the 6-state Sutikno table (§B.6b), NOT this 8-state table.** The zero vectors V0/V7 below cause PMSM stator-flux decay in steady state (a PMSM has no induction-motor slip mechanism to rebuild flux during zero-vector intervals). This 8-state classical table is retained only for induction-motor DTC / general reference.

Input: `(C_ψ, C_T, S)`. `C_ψ ∈ {0, 1}` (flux 1=inc, 0=dec), `C_T ∈ {0, 1}` (torque 1=inc, 0=dec/hold), `S ∈ {I..VI}` (§B.4).

Output: `V_k` ∈ {0..7}.

| C_ψ | C_T | S=I | S=II | S=III | S=IV | S=V | S=VI |
|---|---|---|---|---|---|---|---|
| 1 | 1 | V2 | V3 | V4 | V5 | V6 | V1 |
| 1 | 0 | V7 | V0 | V7 | V0 | V7 | V0 |
| 0 | 1 | V3 | V4 | V5 | V6 | V1 | V2 |
| 0 | 0 | V0 | V7 | V0 | V7 | V0 | V7 |

### Geometric intuition
- **(C_ψ=1, C_T=1)**: increase magnitude + torque → **next forward** (`V_{S+1}`, 60° ahead)
- **(C_ψ=0, C_T=1)**: decrease magnitude + increase torque → **next-next forward** (`V_{S+2}`, 120° ahead)
- **(C_ψ=*, C_T=0)**: hold torque → **zero vector** (V0/V7 alternated to minimize switching transitions)

### Physical meaning
- Core of DTC: compresses "flux + torque dual-loop" into "3-tuple lookup → 1 vector"
- No PI (except optional speed outer loop) / no PWM / no Park
- One vector per sample held the whole period → **switching frequency is variable** (classical DTC limitation)

### Suspended assumptions

| ID | Content |
|---|---|
| **A16** | C_ψ and C_T are **2-level** (1/0). A 3-level torque hysteresis (`C_T ∈ {-1, 0, 1}`) is an advanced option. Baseline uses 2-level |
| **A17** | Variable switching frequency (classical DTC limitation). Remedies: DTC-SVM / model-predictive DTC / advanced switching strategies — record under "Known Limitations" |

---

## §B.6b PMSM 6-state Sutikno table (no zero vectors) ⭐ PMSM default

PMSM **must** use this 6-state table (Sutikno 2011), **not** the 8-state classical table (§B.6). Every cell selects an **active** vector `{V1..V6}` — no V0/V7 — because a PMSM has no slip mechanism to rebuild stator flux during zero-vector intervals (`dψ_s/dt = -R_s·i_s` is decay-only when `u = 0`), so V0/V7 cause steady-state flux collapse.

| (C_ψ, C_T) | S=I | S=II | S=III | S=IV | S=V | S=VI | Effect |
|---|---|---|---|---|---|---|---|
| (1, 1) | V2 | V3 | V4 | V5 | V6 | V1 | flux↑ + Te↑ (forward) |
| (1, 0) | V6 | V1 | V2 | V3 | V4 | V5 | flux↑ + Te↓ (backward) |
| (0, 1) | V3 | V4 | V5 | V6 | V1 | V2 | flux↓ + Te↑ (forward) |
| (0, 0) | V5 | V6 | V1 | V2 | V3 | V4 | flux↓ + Te↓ (backward) |

**Diagnostic**: healthy 6-state operation traces a **circular** αβ flux trajectory of radius `ψ_ref`. A **hexagonal** pattern with an inner pull toward the origin is the classic 8-state-leak signature (V0/V7 collapsing flux).

---

## §B.7 Flux + torque hysteresis comparators

### Flux H_ψ (2-level, baseline)
```
E_ψ = ψ_ref − |ψs|                          // reference − actual
if E_ψ > +HB_ψ:      C_ψ = 1                // actual too small → increase
elif E_ψ < -HB_ψ:    C_ψ = 0                // actual too large → decrease
else:                C_ψ = C_ψ_prev          // dead band: hold previous
```

### Torque H_T (2-level, baseline)
```
E_T = T_ref − Te
if E_T > +HB_T:      C_T = 1
elif E_T < -HB_T:    C_T = 0
else:                C_T = C_T_prev
```

### Torque H_T (3-level, advanced)
```
if E_T > +HB_T:                       C_T = +1     // forward non-zero vector
elif E_T < -HB_T:                     C_T = -1     // reverse non-zero (braking)
elif |E_T| small and prev ∈ {+1,-1}:  C_T = 0      // zero vector
else:                                 C_T = C_T_prev
```
The 3-level form enables rapid reversal/braking. **Not used in baseline.**

### Symbol definitions

| Symbol | Meaning | Unit |
|---|---|---|
| E_ψ | flux magnitude error = ψ_ref − |ψs| | V·s |
| E_T | torque error = T_ref − Te | N·m |
| HB_ψ | flux hysteresis half-band | V·s |
| HB_T | torque hysteresis half-band | N·m |
| C_ψ | flux comparator output (→ §B.6) | {0, 1} |
| C_T | torque comparator output (→ §B.6) | {0, 1} or {-1, 0, 1} |

### Physical meaning
- **Hysteresis = comparator with memory**: within dead band, output holds previous state → prevents chattering near threshold
- DTC uses hysteresis comparators to **replace the PI controller**
- HB half-band trade-off:
  - Too narrow → frequent crossings → high switching frequency → IGBT stress
  - Too wide → large dead band → large flux/torque ripple
  - Typical: `HB_ψ ≈ ±2-5% ψ_ref` / `HB_T ≈ ±5-10% T_max`

### Suspended assumptions

| ID | Content |
|---|---|
| **A18** | `HB_ψ / HB_T` are *Required User Inputs*. Defaults: `HB_ψ ≈ ψ_ref·0.025`, `HB_T ≈ T_max·0.075`. Low-speed regime (ψ drift, A11) → widen HB_ψ |
| **A19** | Baseline uses **2-level** torque comparator + §B.6 table. 3-level requires a bipolar 8-state switching table — advanced option |
| **A20** | Initial state: `C_ψ_prev = 1` (default: build up flux), `C_T_prev = 0` (default: zero vector). Aligns with §B.1 t_init / §B.3 A13 |

---

# §C — Sliding Mode Control law (SMC-PMSM speed-loop, PD-type sliding + STA)

Speed-loop SMC: **PD-type sliding surface (filtered derivative) + Super-Twisting Algorithm (STA) reaching law**. Inner current loop = PI cascade (PZC). STA is a **second-order** sliding-mode algorithm — the switching acts on the *integral* of the control, so the control signal `iq_ref` is continuous through `s = 0`: no chattering by construction (contrast classical `sgn`, ruled out in §C.7). This is the production form for the `motor-smc-pmsm` skill.

## §C.1 Speed error definition

```
e_w = ω_ref − ω_m       [rad/s, internal to control law]
```

| Symbol | Meaning | Unit |
|---|---|---|
| `e_w` | speed tracking error | rad/s |
| `ω_ref` | speed command (interface converts from RPM via ×2π/60) | rad/s |
| `ω_m` | actual mechanical angular speed | rad/s |

### Suspended assumptions
- **A_smc1**: outer-loop view, inner current loop treated as ideal fast loop (`iq ≈ iq_ref`). Consistent with §A.1 A7
- **A_smc1b**: interface uses RPM (×2π/60 → rad/s feeding the control law); scope display in RPM (×60/2π). **Control law internals entirely in rad/s** — prevents RPM domain unit-scaling pitfalls (running error in RPM scales gain by 9.55×)

---

## §C.2 Sliding surface definition (PD-type, filtered derivative)

```
s = e_w + λ · ė_w^f      where   ė_w^f = [ p / (Tf·p + 1) ] · e_w
                         (Simulink Transfer Fcn: Num=[1 0], Den=[Tf 1]; p = d/dt, Laplace s)
```

| Symbol | Meaning | Unit | Range |
|---|---|---|---|
| `s` | sliding surface variable | rad/s | design target: finite-time `s → 0` **and** `ṡ → 0` (2nd-order sliding) |
| `λ` | PD weight (`lambda_pd`) | s | transient phase-lead; default settling-time based (~10 ms) |
| `Tf` | derivative filter constant (`Tf_deriv`) | s | ≈ `Tsc`; bounds derivative HF gain to `1/Tf` |
| `ė_w^f` | filtered speed-error derivative | rad/s² | proper (causal) derivative of `e_w` |

**Symbol choice**: `λ` (not `c`) — avoids conflict with §A.1 plant coefficient.

### Why FILTERED, not pure, derivative (relative-degree key)
A pure-derivative surface `s = e_w + λ·de_w/dt` is **improper**: the control `iq` would appear algebraically in `s` (relative-degree mismatch), so STA cannot act on it. The filter `p/(Tf·p+1)` makes `s` a **proper lead-lag of `e_w`**:
```
s(p) = [ ((Tf + λ)·p + 1) / (Tf·p + 1) ] · e_w(p)
```
Relative degree 0 (proper) → the map `iq → s` keeps the plant's relative degree **1**. **STA requires a relative-degree-1 sliding variable** [Levant 1993; Moreno & Osorio 2012]. DC gain = 1 → `s → 0  ⟺  e_w → 0` at steady state; the `λ` term injects transient phase-lead (damping).

**NO integrator in the surface** → no surface wind-up. STA already supplies the integral (equivalent-control) action through its own `∫sgn(s)` channel (§C.4); a surface integral (PI-type ISMC) is redundant and breaks the STA strict-Lyapunov structure — see §C.7.

### Suspended assumption
- **A_smc2**: Speed system as first-order plant `J·dω_m/dt = Te − B·ω_m − TL` (from A_smc1 with ideal-fast inner loop). Consistent with §A.1 plant model. `B > 0` is the **v1 baseline assumption** (§C.5) — an engineering baseline, not a theoretical requirement; with `B = 0` the plant is a pure integrator that STA still controls in principle (relative degree 1).

---

## §C.3 Sliding-surface dynamics (`ṡ` with plant substituted)

At the sliding-relevant scale the lead-lag DC gain is 1 (`s ≈ e_w`), so:
```
ṡ ≈ ė_w = −(Kt/J)·iq + d ,     d = (B·ω_m + T_L)/J     [rad/s², lumped matched disturbance]
```
with `Kt = (3/2)·P_n·ψ_f` (SPMSM).

This is `ṡ` expressed explicitly in terms of control input `iq` — the starting point for §C.4.

- The control input `iq` appears in `ṡ` (not in `s`) → **relative degree 1** → STA applicable (§C.4).
- `d` lumps the *known* friction `(B·ω_m)/J` and the *unknown* load `T_L/J`. STA rejects the whole of `d` via its integral channel — **no explicit equivalent-control feedforward needed** (contrast classical-sgn §C.7, which must add an `u_eq` term). `|d| ≤ M` bounds the gains (§C.5).

### Derivation outline (4 steps)
1. **Differentiate §C.2** (`s ≈ e_w` at DC): `ṡ ≈ ė_w`
2. **Expand `ė_w`** (with ω_ref piecewise constant → A_smc3): `ė_w = −ω̇_m`
3. **Substitute plant §6**: `ω̇_m = (1/J)·(Te − B·ω_m − T_L)`
4. **Substitute plant §5 SPMSM (id=0 → A_smc4)**: `Te = Kt·iq` → final form

### Suspended assumptions

| ID | Content | Failure / Note |
|---|---|---|
| **A_smc3** | ω_ref piecewise constant (`dω_ref/dt = 0`) | Ramp inputs need ω̇_ref feedforward. Baseline uses step / ramp-to-hold |
| **A_smc4** | SPMSM (`Ld = Lq`, `id_ref = 0`), `Kt = (3/2)·P_n·ψ_f` | IPMSM MTPA → Kt includes reluctance `(3/2)·Pn·[ψf + (Ld−Lq)·id]`. Baseline SPMSM-only |

### Physical meaning
- ṡ = "motion rate" of sliding surface; control target: `ṡ → 0` (and `s → 0`)
- `iq` is the only controllable variable; `d` is the lumped matched disturbance the STA must dominate
- Control design = drive `ṡ` with the STA reaching law and solve for `iq_ref` (see §C.4)
- For Lyapunov design (§C.5), `iq` coefficient `Kt/J` is the control gain `b > 0`

---

## §C.4 Control law — Super-Twisting Algorithm (STA) reaching law

```
u_sta  = K1·|s|^0.5·sgn(s) + K2·∫ sgn(s) dt        [acceleration, rad/s²]
iq_ref = (J/Kt) · u_sta      →  Saturation(±iq_max)     (B-CRIT mandatory)
```

with `K1, K2 > 0` (Lyapunov bounds §C.5), `Kt = (3/2)·P_n·ψ_f`.

### Sign-convention derivation (preserve as build-script comment)
```
ṡ ≈ −(Kt/J)·iq + d            (increasing iq → Te↑ → ω_m↑ → e_w↓ → s↓)
set iq_ref = (J/Kt)·u_sta  →  ṡ ≈ −u_sta + d
standard STA  u = −K1·|σ|^0.5·sgn(σ) − K2·∫sgn(σ)  drives  σ̇ → 0
⇒ with our sign mapping, the positive-gain form above results (u_sta enters with +).
```

### Why STA (second-order sliding)

| Property | Mechanism |
|---|---|
| Continuous `iq_ref` | the discontinuity sits in `u̇_sta` (the `K2·∫sgn` term differentiates to `K2·sgn`); `iq_ref` itself is continuous → **no chattering by construction** |
| Disturbance rejection without FF | `K2·∫sgn(s)` is the equivalent-control estimate of the lumped `d = (B·ω_m + T_L)/J` → rejects matched disturbance, no manual feedforward |
| Finite-time, 2nd-order | converges to `s = ṡ = 0` in finite time (§C.5) |

### Suspended assumption

| ID | Content |
|---|---|
| **A_smc5** | matched disturbance bounded `|d| ≤ M = (TL_max + B·ω_max)/J`. STA rejects matched disturbance up to the gain bound (§C.5). Engineering value: worst-case load in design spec |

### Ruled-out reaching laws
Classical `sgn`, boundary-layer `sat`, PI-type ISMC and the unfiltered pure-derivative PD surface are documented (with reasons) in §C.7 — STA supersedes all of them as the production form.

---

## §C.5 Lyapunov stability + STA gain conditions (C-CRIT)

STA finite-time convergence to the 2nd-order sliding set `s = ṡ = 0` is proven by a **strict (quadratic) Lyapunov function** `V = ζᵀ·P·ζ`, with `ζ = [|s|^0.5·sgn(s),  ∫sgn(s)·dτ]ᵀ`, giving `V̇ ≤ −κ·V^{1/2} < 0` (finite-time) under sufficient gains tied to the disturbance bound [Moreno & Osorio 2012]. Unlike `V = ½s²` for classical 1st-order SMC, this quadratic form is *strict* and certifies convergence of **both** `s` and `ṡ`.

**Disturbance bound**:
```
M = (TL_max + B·ω_max) / J        [rad/s²]
```

**Sufficient gains** (standard practical tuning rule [Shtessel et al. 2014; Levant 1993]; the `K1 ∝ √·`, `K2 ∝ ·` structure is the homogeneity-based selection — cf. [Xiong, Kamal & Jin 2018, eq. 8]):
```
K1 > 1.5 · √M
K2 > 1.1 · M
```

Build script asserts both at construction (C-CRIT). Default auto-computation with empirical margin:
```
K1_sta = max(200 , 1.5·√M · 1.5)
K2_sta = max(8000, 1.1·M  · 1.3)
```

### Honesty note on the bound
The textbook rule's `L` bounds the perturbation **derivative** `|ḋ| ≤ L`; here `M` is a **magnitude** bound used as a practical / conservative proxy (load torque slowly-varying ⇒ `ḋ` scale ~ `M`; the 1.5 / 1.3 margins add buffer). The bound is a **necessary** condition for finite-time reaching, **not** a guarantee of TL-step trough depth — on high-`M` plants pump `K1` to 3–5× and `K2` to 3× the floor (see acceptance criteria / skill `control_law.md`).

### `B > 0` — v1 baseline assumption (not a theoretical requirement)
`B > 0` (default `B = 0.008`) is an **engineering baseline assumption, not a theoretical requirement**. STA finite-time convergence follows from the gain conditions above, **independent of plant viscous damping**; a `B = 0` pure-integrator speed loop (`J·ω̇ = Te − TL`) is relative-degree-1 and STA-controllable in principle. The v1 baseline was developed and validated **entirely with `B > 0`**, so `B = 0` sits **outside the validated envelope** — if your plant has `B ≈ 0`, re-validate the STA gains rather than assuming the controller requires damping.

### Key properties
- **K1** = `|s|^0.5` reaching gain (rad/s²·√s); **K2** = integral robustness gain (rad/s³). Both must dominate `M`.
- Convergence to **both** `s → 0` **and** `ṡ → 0` (2nd-order sliding), unlike classical 1st-order SMC (`s → 0` only).
- Only **matched** disturbances (`T_L` in the input channel) are compensated; unmatched disturbances are not.

---

## §C.6 Discretization + chattering

### Discretized form (ZOH sampling at `Tsc`, typical 50 μs to 1 ms)
```
s[k]      = e_w[k] + λ·ė_w^f[k]                              (ė_w^f from the [1 0]/[Tf 1] filter, sampled)
u_sta[k]  = K1·|s[k]|^0.5·sgn(s[k]) + K2·I[k],   I[k] = I[k−1] + Tsc·sgn(s[k])   (forward Euler integral)
iq_ref[k] = (J/Kt)·u_sta[k]                                  (ZOH-held to (k+1)·Tsc)
```

### Why STA suppresses chattering
Classical `sgn` (§C.7) feeds the discontinuity **directly** into `iq_ref` → discrete limit-cycle of amplitude `O(Tsc)`, with `Δiq ≈ 2·(J·K/Kt)`. STA puts the discontinuity in `u̇_sta` only (the `K2·∫sgn` integral), so the realized `iq_ref` is **continuous** and the discrete sliding accuracy is **`O(Tsc²)`** (Levant 2nd-order accuracy [Levant 1993]) — orders of magnitude below classical-sgn for the same `Tsc`.

### Solver requirement
The `Sign` block is discontinuous → fixed-step `ode3` + `ZeroCrossControl='DisableAll'` (variable-step or ZC-on causes step explosion). See base/`pre_build_grid.md`.

### Expectations
- ✅ Speed converges in finite time; `s, ṡ → 0`
- ✅ TL rejection via the STA integral (`K2·∫sgn`), no manual feedforward
- ✅ `iq_ref` continuous → Te ripple low (no `sgn`-driven oscillation)
- ⚠️ High-`M` plants: TL-step trough may need gain pumping (§C.5)

---

## §C.7 Ruled-out alternatives (why PD-sliding + STA)

These are **not** the production form; documented so the design choice is auditable.

| Alternative | Form | Why ruled out |
|---|---|---|
| **PI-type ISMC** | `s = e_w + λ·∫e_w` | surface integrator → wind-up; redundant with STA's own integral channel; the extra state breaks the STA strict-Lyapunov structure |
| **Classical sgn** | `iq_ref = u_eq + (J·K/Kt)·sgn(s)` | discontinuous control → chattering `ΔTe ≈ 2·J·K`; only 1st-order sliding (`s→0` only); needs explicit `u_eq` feedforward. Pedagogical baseline only |
| **Boundary-layer sat** | `sgn(s) → sat(s/φ)` | trades sliding precision (steady-state error ≈ `φ`) for chattering reduction; STA gets continuity *without* the precision loss |
| **Pure-derivative PD** | `s = e_w + λ·de_w/dt` (unfiltered) | improper / relative-degree mismatch — control appears algebraically in `s`. The filtered derivative (§C.2) is the fix |
| **Zero-order** | `s = e_w` | valid relative-degree-1 STA surface (common in the literature) but no transient phase-lead; PD-type adds damping |

### Sources (AI-self-audited; openable)
- A. Levant, "Sliding order and sliding accuracy in sliding mode control," *Int. J. Control*, 58(6):1247–1263, 1993 — STA origin, relative-degree-1 requirement, `O(Tsc²)` accuracy.
- J. A. Moreno & M. Osorio, "Strict Lyapunov Functions for the Super-Twisting Algorithm," *IEEE Trans. Autom. Control*, 57(4):1035–1040, 2012 — strict quadratic Lyapunov function + sufficient gain conditions.
- Y. Shtessel, C. Edwards, L. Fridman, A. Levant, *Sliding Mode Control and Observation*, Birkhäuser, 2014 — practical tuning `K1 = 1.5·√L`, `K2 = 1.1·L`.
- X. Xiong, S. Kamal, S. Jin, "Adaptive Gains to Super-Twisting Technique for Sliding Mode Design," arXiv:1805.07761, 2018 — STA form (eq. 1), perturbation bound (eq. 3), homogeneity-based gain selection (eq. 8).

---
