# DTC Chart Algorithm

One control period @ Tsc. Embed into the chart Script via `sprintf` from build script (analogous to FCS-MPC K-CRIT pattern).

## Chart Inputs (5)

- `Te_ref` (from outer Speed PI Saturation output)
- `ia, ib` (from Clark transform on phase currents)
- `ua, ub` (from Clark transform on phase voltages)

All ZOH @ Tsc upstream (D-CRIT).

## Chart Outputs (9)

- `gate` (6×1 column) → Universal Bridge gate input
- `Te_meas, mag_psi, ψ_α, ψ_β, sector, V_k, C_ψ, C_T` → logger Mux

All ZOH @ Tsc downstream (D-CRIT).

## Persistent State (4)

- `psi_a` (α-axis stator flux integral, init `psi_alpha_0 = ψ_f`)
- `psi_b` (β-axis stator flux integral, init `psi_beta_0 = 0`)
- `Cpsi_prev` (previous flux hysteresis output, init `1`)
- `CT_prev` (previous torque hysteresis output, init `1`)

## Pseudocode

```
# ---- 1. Stator flux integrator (Forward Euler) ----
# u_s = U_s - Rs * i_s   (back-EMF subtracted)
psi_a = psi_a + Tsc * (ua - Rs * ia)
psi_b = psi_b + Tsc * (ub - Rs * ib)

# ---- 2. Te estimator (αβ cross-product) ----
Te_meas = (3/2) * Pn * (psi_a * ib - psi_b * ia)

# ---- 3. Flux magnitude + angle ----
mag_psi = sqrt(psi_a^2 + psi_b^2)
theta_psi = atan2(psi_b, psi_a)              # [-pi, pi]

# ---- 4. Sector identification (Convention A) ----
sector = floor(mod(theta_psi + 2*pi, 2*pi) / (pi/3)) + 1   # 1..6

# ---- 5. 2-level hysteresis with state memory ----
# Flux hysteresis
E_psi = psi_ref - mag_psi
if      E_psi >  HB_psi/2,   C_psi = 1
elseif  E_psi < -HB_psi/2,   C_psi = 0
else                          C_psi = Cpsi_prev

# Torque hysteresis
E_T = Te_ref - Te_meas
if      E_T >  HB_T/2,    C_T = 1
elseif  E_T < -HB_T/2,    C_T = 0
else                       C_T = CT_prev

# Update persistent state
Cpsi_prev = C_psi
CT_prev   = C_T

# ---- 6. 6-state Sutikno Table 2 lookup ----
# Row index: 4 combinations of (C_psi, C_T), in order (1,1)/(1,0)/(0,1)/(0,0)
row = (1 - C_psi) * 2 + (1 - C_T) + 1     # (1,1)→1, (1,0)→2, (0,1)→3, (0,0)→4
table = int32([
    2 3 4 5 6 1;     % (1,1)
    6 1 2 3 4 5;     % (1,0)
    3 4 5 6 1 2;     % (0,1)
    5 6 1 2 3 4      % (0,0)
])
V_k = table(row, sector)

# ---- 7. V_k → gate (pair-adjacent for Universal Bridge) ----
vector_table = [
    1 0 0;     % V1
    1 1 0;     % V2
    0 1 0;     % V3
    0 1 1;     % V4
    0 0 1;     % V5
    1 0 1      % V6
]
S = vector_table(V_k, :)
gate = [S(1); 1 - S(1); S(2); 1 - S(2); S(3); 1 - S(3)]
```

## Chart Re-Add SOP

At end of build, defensively `delete_block` + re-add the chart (with same `Position`) and re-wire all incoming/outgoing lines. Heavy `add_block` / `add_line` activity earlier in the build can silently corrupt chart attributes; the re-add is a cheap insurance.

## Chart Configuration (D-CRIT)

```matlab
ch = chart_handle;
ch.SampleTime  = '-1';        % INHERITED
ch.ChartUpdate = 'INHERITED';
% Do NOT set ch.Inputs(k).DataType
% Do NOT set ch.Inputs(k).Props.Array.Size
```

## Chart Embedding via sprintf (analogous to FCS-MPC K-CRIT)

```matlab
chart_body = sprintf([ ...
    'function [gate, Te_meas, mag_psi, psi_a_out, psi_b_out, sector, V_k, C_psi, C_T] = dtc_chart(Te_ref, ia, ib, ua, ub)\n' ...
    '%%#codegen\n' ...
    'persistent psi_a psi_b Cpsi_prev CT_prev\n' ...
    'if isempty(psi_a),  psi_a = %g;     end\n' ...    % psi_alpha_0
    'if isempty(psi_b),  psi_b = %g;     end\n' ...    % psi_beta_0
    'if isempty(Cpsi_prev), Cpsi_prev = 1; end\n' ...
    'if isempty(CT_prev),   CT_prev = 1;   end\n' ...
    'Pn = %d; Rs_p = %g; Tsc_p = %g;\n' ...
    'psi_ref_p = %g; HB_psi_p = %g; HB_T_p = %g;\n' ...
    '... algorithm body ...\n' ...
    'end\n'], psi_alpha_0, psi_beta_0, Pn, Rs, Tsc, psi_ref, HB_psi, HB_T);
ch.Script = chart_body;
```

The chart Script must total ≥ 50 lines for Phase 9 self-test 2 to pass (full algorithm inline).

## Audit

After build, open the `.slx`, double-click the chart, view its Script. Numeric literals for `Pn / Rs / Tsc / psi_ref / HB_psi / HB_T` must match `get_param(mdl, 'InitFcn')` digit-for-digit.
