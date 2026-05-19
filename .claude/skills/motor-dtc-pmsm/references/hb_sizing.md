# Hysteresis Band (HB) Sizing

`HB_T` and `HB_psi` should be **back-calculated from `fs_max`**, not set as blind percentages of `T_max` / `psi_ref`. Percentage fallback is for use when `fs_max` is unknown.

## HB_T Reverse-Calculation

Torque slew rate upper bound (from §B.2 + plant Vdc):

```
|dTe/dt|_max ≈ 1.5 · Pn · psi_ref · |di_q/dt|_max
            ≈ 1.5 · Pn · psi_ref · (2/3 · Vdc − psi_f · omega_e_max) / Lq
```

Hysteresis flip frequency upper bound (double-edge):

```
fs_T ≈ |dTe/dt|_max / (4 · HB_T)        [Hz]
HB_T ≥ |dTe/dt|_max / (4 · fs_max)
```

## HB_psi Reverse-Calculation

Flux slew rate ≈ active vector edge contribution:

```
|d psi_s / dt|_max ≈ Vdc / (2 · sqrt(3))
HB_psi ≥ Vdc / (8 · sqrt(3) · fs_max)
```

## Algorithm (in your build script)

```matlab
% Pre-chart construction:
omega_e_max = omega_ref_rpm * 2*pi / 60 * Pn;

dTe_dt_max  = 1.5 * Pn * psi_ref * (2/3 * Vdc - psi_f * omega_e_max) / Lq;
dpsi_dt_max = Vdc / (2 * sqrt(3));

HB_T_min   = dTe_dt_max  / (4 * fs_max);
HB_psi_min = dpsi_dt_max / (4 * fs_max);

% Floor against numerical hyper-sensitivity:
HB_T   = max(HB_T_min,   0.01  * T_max);    % at least 1%
HB_psi = max(HB_psi_min, 0.005 * psi_ref);  % at least 0.5%

% If fs_max not supplied, fallback:
%   HB_T   = 0.075 * T_max
%   HB_psi = 0.025 * psi_ref
```

## Trade-off

For ripple-fidelity tuning to match a specific reference baseline, `HB` may go below the formula floor at the cost of higher switching frequency. Always log and verify the actual switching frequency stays within the IGBT thermal window:

```matlab
fs_actual = count(diff(V_k_built) ~= 0) / sim_time;
assert(fs_actual < 3 * fs_max, 'Switching frequency exceeds 3x fs_max — IGBT thermal risk.');
```

The 3× factor accounts for V_k transitions occurring approximately 3× per phase (each phase switches independently).
