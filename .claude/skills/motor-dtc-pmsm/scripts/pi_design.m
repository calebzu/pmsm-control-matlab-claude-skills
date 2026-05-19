function [Kp_rad, Ki_rad, Kp_rpm, Ki_rpm, info] = pi_design(varargin)
% pi_design — Outer-loop speed PI gain calculator for PMSM DTC drives.
%
% KEY DIFFERENCE FROM motor-fcs-mpc/pi_design.m:
%   FCS-MPC outer PI outputs iq_ref [A]  → plant gain has Kt:  G_p = Kt/[J·s·(T_eq·s+1)]
%   DTC     outer PI outputs Te_ref [N·m] → plant gain NO  Kt:  G_p = 1 /[J·s·(T_eq·s+1)]
%   So SO/PZC formulas drop Kt from numerator. Units of Kp/Ki are N·m-based, not A-based.
%
% Three modes per shared/formulas/pmsm_formulas.md §A:
%
%   1. 'SO'        Symmetrical Optimum, B=0 plant (default; see §A.2 method 3 — DTC adapted)
%                  pi_design('SO', J, T_eq, a)              [4 args, Kt removed]
%                  Kp = J / (a · T_eq)
%                  Ki = J / (a^3 · T_eq^2)
%
%   2. 'PZC'       Pole-Zero Cancellation, B>0 plant (see §A.2 method 1 — DTC adapted)
%                  pi_design('PZC', J, B, omega_c)          [4 args, Kt removed]
%                  Ki/Kp = B/J;   open-loop crossover = omega_c
%                  Kp = omega_c · B
%                  Ki = omega_c · B^2 / J
%
%   3. '2ndorder'  Legacy 2nd-order standard form (NOT SO; emits warning)
%                  pi_design('2ndorder', J, wn, zeta)
%                  Kp = 2·zeta·wn·J
%                  Ki = wn^2 · J
%
% =====================================================================================
% PHASE 4.5 THEORY SANITY CHECK
% =====================================================================================
% Plant (DTC, B=0):    G_p(s) = 1 / [J·s·(T_eq·s + 1)]
% Controller (PI):     G_c(s) = (Kp·s + Ki)/s
% SO design:           Kp = J/(a·T_eq), Ki = J/(a^3·T_eq^2),  τ_i = a^2·T_eq, ωc = 1/(a·T_eq)
%
% --- 3 CLOSED-LOOP TRANSFER FUNCTIONS ---
%   1. Reference tracking:  ω(s)/r(s) = L/(1+L),   L = G_c·G_p
%   2. Disturbance reject:  ω(s)/(-TL)(s) = G_mech / (1+L)
%                                         = -s·(T_eq·s+1) / D(s)
%   3. Sensor noise:        not evaluated for v1 baseline (DTC uses αβ-estimated Te per
%                           §B.2; sensor noise enters via voltage/current measurement,
%                           path is complex; deferred to v1.x sensor-noise study)
%
% Closed-loop characteristic polynomial:
%   D(s) = J·T_eq·s^3 + J·s^2 + Kp·s + Ki
%
% --- 2 SLOWEST POLE & τ_max ---
% Substitute SO Kp/Ki and normalize σ = T_eq·s:
%   σ^3 + σ^2 + σ/a + 1/a^3 = 0
% This dimensionless cubic has roots determined ONLY by a (not by J, T_eq):
%   a=2: σ = {-0.500, -0.250±0.433i}    → |Re|_min = 0.250, τ_max = 4.0 ·T_eq
%   a=3: σ = {-0.333, -0.333,  -0.333}  → |Re|_min = 0.333, τ_max = 3.0 ·T_eq  (triple-real)
%   a=4: σ = {-0.655, -0.250, -0.0955}  → |Re|_min = 0.0955, τ_max = 10.47·T_eq  ⭐ DEFAULT
%   a=6: σ = {-0.799, -0.167, -0.0348}  → |Re|_min = 0.0348, τ_max = 28.75·T_eq
%
% ⚠️ NON-INTUITIVE: larger a → SLOWER τ_max (slow real pole pulls toward origin).
%    Trade-off: small a (2,3) → faster τ_max but lower damping (ζ_eq drops);
%               a=4 is Kessler standard — best phase margin (~36°), accepts τ_max=10.47·T_eq.
%    σ root values verified by MATLAB roots().
%
% --- 3 NUMERICAL INSTANCES (DTC v1 baseline) ---
% Tsc = 50 μs (control sampling), a = 4 default:
%   T_eq =  5·Tsc = 250 μs  →  τ_max =  2.62 ms,  5·τ_max = 13.1 ms
%   T_eq = 10·Tsc = 500 μs  →  τ_max =  5.24 ms,  5·τ_max = 26.2 ms  ⭐ recommended
%   T_eq = 20·Tsc = 1.0 ms  →  τ_max = 10.47 ms,  5·τ_max = 52.4 ms
%
% T_eq for DTC ≠ 5·Tsc (FCS-MPC convention). DTC inner loop is hysteresis + switching
% table, NOT current PI. Effective T_eq depends on hysteresis switching frequency
% (set by HB_T, HB_psi, plant impedance, DC bus). Conservative estimate:
%   T_eq_DTC ≈ 10·Tsc to 20·Tsc  (actual switching freq ≈ 5-20 kHz in simulation)
%
% --- 4 T_window COMPATIBILITY VERDICT ---
% Hard rule: T_window ≥ 5·τ_max (workflow Phase 4.5)
%   T_window = test scenario evaluation window (e.g., "last 20% of sim" / "post-load 0.4s")
%
% v1 baseline scenario plan: 4-pulse-square TL on 1.0 s sim (motor-fcs-mpc convention)
%   T_window = "last 20% of 1.0 s" = 200 ms
%   For T_eq = 500 μs (recommended): 5·τ_max = 26.2 ms ≤ 200 ms  ✓ PASS (margin 7.6×)
%   Absolute ceiling: T_eq < T_window/(5·10.47) = 200/52.4 = 3.82 ms → SO a=4 always PASS
%
% verdict_check() below enforces this rule when 'T_window' is supplied as kwarg.
% =====================================================================================
%
% Inputs:
%   J        — rotor inertia [kg·m^2]
%   For 'SO': T_eq — inner-loop equivalent time constant [s] (typ. 10·Tsc for DTC)
%             a    — SO factor (2/3/4/6; default 4 → ζ_eq≈0.71)
%   For 'PZC': B   — viscous friction [N·m·s]
%              omega_c — desired open-loop crossover [rad/s]
%   For '2ndorder': wn — closed-loop natural frequency [rad/s]
%                   zeta — closed-loop damping ratio
%   Optional kwarg pair: 'T_window', value [s]  — Phase 4.5 sanity check window
%
% Outputs:
%   Kp_rad, Ki_rad — gains for PI in rad/s domain (units: N·m·s/rad, N·m/rad)
%   Kp_rpm, Ki_rpm — gains for PI in RPM domain   (units: N·m/RPM, N·m/(RPM·s))
%   info struct    — diagnostic fields (.method, .omega_c, .t_settle, .tau_max,
%                                       .verdict, .sanity_check_T_window)
%
% Usage example (DTC v1 baseline):
%   J = 5.58e-4; Tsc = 50e-6; T_eq = 10*Tsc; a = 4;
%   [Kp_rad, Ki_rad, Kp_rpm, Ki_rpm, info] = pi_design('SO', J, T_eq, a, ...
%                                                       'T_window', 0.2);
%   % Then in build_template.m: assignin('base', 'Kp_w', Kp_rpm);  % or write to mdl InitFcn
%   %                           assignin('base', 'Ki_w', Ki_rpm);
%   % Simulink: Discrete PID Controller block, fields P='Kp_w', I='Ki_w' (string refs).
%   % info.verdict should report 'OK' with 5·τ_max = 26 ms ≤ T_window 200 ms
%
% ⚠️ ROLE BOUNDARY (this is a MATLAB function FILE, not a Simulink block):
%   - This .m file is an OFFLINE BUILD-TIME HELPER. It is invoked by build_template.m
%     during model construction to compute (Kp, Ki) from (J, T_eq, a). It is NOT
%     embedded in the .slx model in any form.
%   - The PI controller INSIDE the Simulink model is a `Discrete PID Controller` block
%     with P/I fields holding STRING references to workspace variables `Kp_w`/`Ki_w`.
%   - DTC algorithm logic (sector detection, switching table, V_k decoder) MUST use
%     Simulink `MATLAB Function block` (a.k.a. chart) — NOT legacy S-function.
%
% Reference: shared/formulas/pmsm_formulas.md §A (DTC plant/Kt difference noted §A.7).

    if nargin == 0
        help pi_design;
        return;
    end

    method = varargin{1};
    if ~ischar(method) && ~isstring(method)
        error('pi_design:bad_method', ...
              'First argument must be method string: ''SO'', ''PZC'', or ''2ndorder''.');
    end
    method = char(method);

    % --- parse optional 'T_window' kwarg (Phase 4.5 sanity check) ---
    T_window = NaN;
    pos_args = varargin;
    for k = nargin-1:-1:2
        if (ischar(varargin{k}) || isstring(varargin{k})) && strcmpi(varargin{k}, 'T_window')
            T_window = varargin{k+1};
            pos_args(k:k+1) = [];
            break;
        end
    end
    n_pos = numel(pos_args);

    info = struct('method', method, 'sanity_check_T_window', T_window);

    switch lower(method)
        case 'so'
            if n_pos < 3
                error('pi_design:SO_inputs', ...
                      'SO mode needs (J, T_eq); a optional (default 4). Note: NO Kt for DTC.');
            end
            J    = pos_args{2};
            T_eq = pos_args{3};
            if n_pos >= 4; a = pos_args{4}; else; a = 4; end

            Kp_rad = J / (a * T_eq);
            Ki_rad = J / (a^3 * T_eq^2);

            info.J     = J;
            info.T_eq  = T_eq;
            info.a     = a;
            info.zeta_eq = damping_from_a(a);
            info.omega_c = 1 / (a * T_eq);
            info.t_settle = 4 * a * T_eq;
            info.t_rise   = 1.8 / info.omega_c;
            info.tau_max  = tau_max_from_a(a) * T_eq;     % slowest closed-loop pole

        case 'pzc'
            if n_pos < 4
                error('pi_design:PZC_inputs', ...
                      'PZC mode needs (J, B, omega_c). Note: NO Kt for DTC.');
            end
            J = pos_args{2};
            B = pos_args{3};
            omega_c = pos_args{4};
            if B <= 0
                error('pi_design:PZC_B_positive', ...
                      'PZC requires B > 0. For B=0 plant use ''SO'' mode.');
            end

            Kp_rad = omega_c * B;
            Ki_rad = omega_c * B^2 / J;

            info.J = J; info.B = B;
            info.omega_c = omega_c;
            info.t_settle_tracking = 4 / omega_c;
            info.t_settle = info.t_settle_tracking;
            info.disturb_pole = -B / J;
            info.t_disturb_settle = 4 * J / B;
            info.t_rise   = 2.2 / omega_c;
            info.zeta_eq  = 1.0;
            info.tau_max  = max(1/omega_c, J/B);          % slow pole = max of two

        case '2ndorder'
            if n_pos < 3
                error('pi_design:2ndorder_inputs', ...
                      '2ndorder mode needs (J, wn, zeta=0.7). Note: NO Kt for DTC.');
            end
            J = pos_args{2};
            wn = pos_args{3};
            if n_pos >= 4; zeta = pos_args{4}; else; zeta = 0.7; end

            warning('pi_design:legacy_2ndorder', ...
                ['2ndorder mode ignores inner-loop dynamics (T_eq). For B=0 DTC plant ', ...
                 'prefer ''SO'' mode which couples gains to hysteresis-equivalent T_eq. ', ...
                 'See pmsm_formulas.md §A.2 + §A.7.']);

            Kp_rad = 2 * zeta * wn * J;
            Ki_rad = wn^2 * J;

            info.J = J;
            info.wn   = wn;
            info.zeta = zeta;
            info.omega_c = wn;
            info.t_settle = 5 / (zeta * wn);
            info.t_rise   = 1.8 / wn;
            info.zeta_eq  = zeta;
            info.tau_max  = 1 / (zeta * wn);

        otherwise
            error('pi_design:unknown_method', ...
                  'Unknown method ''%s''. Use ''SO'', ''PZC'', or ''2ndorder''.', method);
    end

    Kp_rpm = Kp_rad * pi / 30;
    Ki_rpm = Ki_rad * pi / 30;

    info.verdict = verdict_check(info);

    if nargout == 0
        print_summary(Kp_rad, Ki_rad, Kp_rpm, Ki_rpm, info);
    end
end

function zeta_eq = damping_from_a(a)
    table_a    = [2, 3, 4, 6];
    table_zeta = [0.5, 0.6, 0.71, 0.85];
    if any(a == table_a)
        zeta_eq = table_zeta(a == table_a);
    else
        zeta_eq = interp1(table_a, table_zeta, a, 'linear', 'extrap');
    end
end

function r = tau_max_from_a(a)
    % Slowest-pole multiplier for SO closed-loop char eq sigma^3+sigma^2+sigma/a+1/a^3=0
    % Verified numerically (MATLAB roots()): a=2→4.0, a=3→3.0, a=4→10.47, a=6→28.75
    table_a   = [2, 3, 4, 6];
    table_tau = [4.0, 3.0, 10.472, 28.748];
    if any(a == table_a)
        r = table_tau(a == table_a);
    else
        coeffs = [1, 1, 1/a, 1/a^3];
        rts = roots(coeffs);
        r = 1 / min(abs(real(rts)));
    end
end

function v = verdict_check(info)
    v = 'OK';
    if isfield(info, 'T_eq') && info.t_settle > 50 * info.T_eq
        v = 'too slow (t_s > 50·T_eq; consider smaller a)';
        return;
    end
    if isfield(info, 'T_eq') && info.t_settle < 4 * info.T_eq
        v = 'aggressive (t_s < 4·T_eq; check inner BW headroom)';
        return;
    end
    % PZC low-B/J check (inherited from motor-fcs-mpc design)
    if isfield(info, 'B') && isfield(info, 'omega_c') && info.B > 0
        bj = info.B / info.J;
        if bj < info.omega_c / 5
            if info.t_disturb_settle < 1
                settle_str = sprintf('%.0f ms', info.t_disturb_settle*1000);
            else
                settle_str = sprintf('%.2f s', info.t_disturb_settle);
            end
            v = sprintf(['low-B/J regime (B/J=%.3f rad/s << omega_c=%.0f rad/s, ratio %.4f). ' ...
                         'PZC keeps slow plant pole in disturb path; t_disturb_settle = %s. ' ...
                         'For load-step scenarios with recovery window < %s, prefer SO mode.'], ...
                         bj, info.omega_c, bj/info.omega_c, settle_str, settle_str);
            return;
        end
    end
    % Phase 4.5 Theory Sanity Check
    if isfield(info, 'sanity_check_T_window') && ~isnan(info.sanity_check_T_window) ...
            && isfield(info, 'tau_max')
        T_win = info.sanity_check_T_window;
        if 5 * info.tau_max > T_win
            v = sprintf(['Phase 4.5 FAIL: 5·tau_max = %.1f ms > T_window = %.1f ms. ' ...
                         'Slow closed-loop pole will leak into evaluation window. ' ...
                         'Either lengthen sim (T_window >= %.1f ms) or shrink T_eq ' ...
                         '(< %.2f ms for current a=%g).'], ...
                         5*info.tau_max*1000, T_win*1000, ...
                         5*info.tau_max*1000, ...
                         T_win/(5*tau_max_from_a(info.a))*1000, info.a);
            return;
        end
    end
    if info.omega_c > 1e5
        v = 'extreme bandwidth (omega_c > 100 krad/s; numerical risk)';
    end
end

function print_summary(Kp_rad, Ki_rad, Kp_rpm, Ki_rpm, info)
    fprintf('\n=== pi_design (%s) — DTC variant ===\n', upper(info.method));
    if isfield(info, 'J');    fprintf('  J        = %.3e kg·m^2\n', info.J);   end
    if isfield(info, 'T_eq'); fprintf('  T_eq     = %.3e s (%.0f μs)\n', info.T_eq, info.T_eq*1e6); end
    if isfield(info, 'a');    fprintf('  a        = %g (zeta_eq ≈ %.2f)\n', info.a, info.zeta_eq); end
    if isfield(info, 'B');    fprintf('  B        = %.3e N·m·s\n', info.B);    end
    if isfield(info, 'wn');   fprintf('  wn       = %g rad/s, zeta = %g\n', info.wn, info.zeta); end
    fprintf('\n  Kp_rad   = %.4e N·m·s/rad\n', Kp_rad);
    fprintf('  Ki_rad   = %.4e N·m/rad\n',     Ki_rad);
    fprintf('  Kp_rpm   = %.4e N·m/RPM\n',     Kp_rpm);
    fprintf('  Ki_rpm   = %.4e N·m/(RPM·s)\n', Ki_rpm);
    fprintf('\n  omega_c  = %.1f rad/s   (~%.1f Hz)\n', info.omega_c, info.omega_c/(2*pi));
    if isfield(info, 'tau_max')
        fprintf('  tau_max  = %.2f ms  (slowest closed-loop pole; Phase 4.5)\n', info.tau_max*1000);
        fprintf('  5·tau_max= %.2f ms  (T_window must >= this)\n', 5*info.tau_max*1000);
    end
    if isfield(info, 't_settle_tracking') && isfield(info, 't_disturb_settle')
        fprintf('  t_settle (tracking) = %.2f ms (2%%)\n',  info.t_settle_tracking*1000);
        fprintf('  t_settle (disturb)  = %.2f s  (= 4*J/B; PZC slow plant pole NOT cancelled)\n', info.t_disturb_settle);
    else
        fprintf('  t_settle = %.2f ms (2%%)\n',  info.t_settle*1000);
    end
    fprintf('  t_rise   = %.2f ms\n',        info.t_rise*1000);
    if isfield(info, 'sanity_check_T_window') && ~isnan(info.sanity_check_T_window)
        fprintf('  T_window = %.1f ms (Phase 4.5 sanity check window)\n', info.sanity_check_T_window*1000);
    end
    fprintf('  verdict  = %s\n\n', info.verdict);
end
