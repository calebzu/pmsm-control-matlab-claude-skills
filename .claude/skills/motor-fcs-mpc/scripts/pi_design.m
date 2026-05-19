function [Kp_rad, Ki_rad, Kp_rpm, Ki_rpm, info] = pi_design(varargin)
% pi_design — Outer-loop speed PI gain calculator for PMSM FCS-MPC drives.
%
% Three modes per shared/formulas/pmsm_formulas.md §A:
%
%   1. 'SO'        Symmetrical Optimum, B=0 plant (default; see §A.2 method 3)
%                  pi_design('SO', J, Kt, T_eq, a)
%                  Kp = J / (a · Kt · T_eq)
%                  Ki = J / (a^3 · Kt · T_eq^2)
%
%   2. 'PZC'       Pole-Zero Cancellation, B>0 plant (see §A.2 method 1)
%                  pi_design('PZC', J, Kt, B, omega_c)
%                  Ki/Kp = B/J;   open-loop crossover = omega_c
%                  Kp = omega_c · B / Kt
%                  Ki = Kp · B / J
%
%   3. '2ndorder'  Legacy 2nd-order standard form (NOT SO; emits warning)
%                  pi_design('2ndorder', J, Kt, wn, zeta)
%                  Kp = 2·zeta·wn·J / Kt
%                  Ki = wn^2 · J / Kt
%                  Use only if user specifies wn/zeta directly. For B=0 plant
%                  prefer 'SO' which accounts for inner-loop time constant.
%
% Inputs:
%   J        — rotor inertia [kg·m^2]
%   Kt       — torque constant 1.5·Pn·flux [N·m/A]
%   For 'SO': T_eq — inner-loop equivalent time constant [s] (typ. 5·Tsc)
%             a    — SO factor (2/3/4/6; default 4 → zeta_eq≈0.71)
%   For 'PZC': B   — viscous friction [N·m·s]
%              omega_c — desired open-loop crossover [rad/s]
%   For '2ndorder': wn   — closed-loop natural frequency [rad/s]
%                   zeta — closed-loop damping ratio
%
% Outputs:
%   Kp_rad, Ki_rad — gains for PI in rad/s domain
%   Kp_rpm, Ki_rpm — gains for PI in RPM domain (Kp_rad · pi/30)
%   info struct    — diagnostic fields:
%                    .method, .omega_c [rad/s], .t_settle [s], .t_rise [s],
%                    .verdict {'OK'/'too slow'/'aggressive'/'inner BW conflict'},
%                    .scenario_check (if scenario hints provided via 'TL_period')
%
% Usage example (IPMSM mid-saliency, SO recommendation):
%   J = 5.58e-4; Pn = 5; flux = 5.45e-2; Kt = 1.5*Pn*flux;
%   Tsc = 50e-6; T_eq = 5*Tsc; a = 4;
%   [Kp_rad, Ki_rad, Kp_rpm, Ki_rpm, info] = pi_design('SO', J, Kt, T_eq, a);
%   % Then in build_template.m PARAMETER BLOCK: Kp_w = Kp_rpm; Ki_w = Ki_rpm;
%
% Reference: shared/formulas/pmsm_formulas.md §A (Outer-Loop PI Design).

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

    info = struct('method', method);

    switch lower(method)
        case 'so'
            if nargin < 4
                error('pi_design:SO_inputs', ...
                      'SO mode needs (J, Kt, T_eq); a optional (default 4).');
            end
            J    = varargin{2};
            Kt   = varargin{3};
            T_eq = varargin{4};
            if nargin >= 5; a = varargin{5}; else; a = 4; end

            Kp_rad = J / (a * Kt * T_eq);
            Ki_rad = J / (a^3 * Kt * T_eq^2);

            info.J     = J;
            info.Kt    = Kt;
            info.T_eq  = T_eq;
            info.a     = a;
            info.zeta_eq = damping_from_a(a);
            info.omega_c = 1 / (a * T_eq);                 % rad/s (SO crossover)
            info.t_settle = 4 * a * T_eq;                  % 2% settling, 2nd-order approx
            info.t_rise   = 1.8 / info.omega_c;            % standard 10-90% rise

        case 'pzc'
            if nargin < 5
                error('pi_design:PZC_inputs', ...
                      'PZC mode needs (J, Kt, B, omega_c).');
            end
            J = varargin{2}; Kt = varargin{3};
            B = varargin{4}; omega_c = varargin{5};
            if B <= 0
                error('pi_design:PZC_B_positive', ...
                      'PZC requires B > 0. For B=0 plant use ''SO'' mode.');
            end

            Kp_rad = omega_c * B / Kt;
            Ki_rad = Kp_rad * B / J;

            info.J = J; info.Kt = Kt; info.B = B;
            info.omega_c = omega_c;
            info.t_settle_tracking = 4 / omega_c;          % reference-tracking 2% settling
            info.t_settle = info.t_settle_tracking;        % backward-compat alias
            % Disturbance-rejection settling — driven by the plant pole at -B/J that
            % PZC's pole-zero cancellation does NOT remove from the disturbance path.
            % Detecting this prevents silent failure on low-friction
            % plants where B/J << omega_c (a silent failure mode).
            info.disturb_pole = -B / J;                    % rad/s (closed-loop disturb pole)
            info.t_disturb_settle = 4 * J / B;             % s (4-tau settling for disturb)
            info.t_rise   = 2.2 / omega_c;
            info.zeta_eq  = 1.0;                           % first-order, no overshoot

        case '2ndorder'
            if nargin < 4
                error('pi_design:2ndorder_inputs', ...
                      '2ndorder mode needs (J, Kt, wn, zeta=0.7).');
            end
            J = varargin{2}; Kt = varargin{3};
            wn = varargin{4};
            if nargin >= 5; zeta = varargin{5}; else; zeta = 0.7; end

            warning('pi_design:legacy_2ndorder', ...
                ['2ndorder mode ignores inner-loop dynamics (T_eq). For B=0 plant ', ...
                 'with FCS-MPC inner loop, prefer ''SO'' mode which couples gains ', ...
                 'to the 5·Tsc time constant. See pmsm_formulas.md §A.2.']);

            Kp_rad = 2 * zeta * wn * J / Kt;
            Ki_rad = wn^2 * J / Kt;

            info.J = J; info.Kt = Kt;
            info.wn   = wn;
            info.zeta = zeta;
            info.omega_c = wn;                             % nominal
            info.t_settle = 5 / (zeta * wn);               % 2% settling
            info.t_rise   = 1.8 / wn;
            info.zeta_eq  = zeta;

        otherwise
            error('pi_design:unknown_method', ...
                  'Unknown method ''%s''. Use ''SO'', ''PZC'', or ''2ndorder''.', method);
    end

    % --- RPM-domain conversion (PI input/output in RPM) ---
    %     1 RPM = pi/30 rad/s; gains scale by pi/30.
    Kp_rpm = Kp_rad * pi / 30;
    Ki_rpm = Ki_rad * pi / 30;

    % --- Verdict: cross-check vs scenario sanity ---
    info.verdict = verdict_check(info);

    % --- Print summary unless suppressed ---
    if nargout == 0
        print_summary(Kp_rad, Ki_rad, Kp_rpm, Ki_rpm, info);
    end
end

function zeta_eq = damping_from_a(a)
    % SO factor a → equivalent closed-loop damping (Kessler 1955 / Schröder)
    table_a    = [2, 3, 4, 6];
    table_zeta = [0.5, 0.6, 0.71, 0.85];
    if any(a == table_a)
        zeta_eq = table_zeta(a == table_a);
    else
        zeta_eq = interp1(table_a, table_zeta, a, 'linear', 'extrap');
    end
end

function v = verdict_check(info)
    % Heuristic warnings based on info fields. Not fatal — informational.
    v = 'OK';
    if isfield(info, 'T_eq') && info.t_settle > 50 * info.T_eq
        v = 'too slow (t_s > 50·T_eq; consider smaller a)';
        return;
    end
    if isfield(info, 'T_eq') && info.t_settle < 4 * info.T_eq
        v = 'aggressive (t_s < 4·T_eq; check inner BW headroom)';
        return;
    end
    % PZC low-B/J check (failure analysis).
    % PZC cancels plant pole at -B/J in the FORWARD path, but the same pole reappears
    % in the closed-loop DISTURBANCE-rejection path. When B/J << omega_c, reference
    % tracking is fast (= 4/omega_c) but load-step recovery is slow (= 4·J/B). For
    % typical servo motors with low friction, this is a silent failure mode that
    % passes pi_design's verdict but fails closed-loop sanity at runtime.
    if isfield(info, 'B') && isfield(info, 'omega_c') && info.B > 0
        bj = info.B / info.J;
        if bj < info.omega_c / 5
            % Format t_disturb_settle adaptively (ms for sub-second, s otherwise)
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
    if info.omega_c > 1e5
        v = 'extreme bandwidth (omega_c > 100 krad/s; numerical risk)';
    end
end

function print_summary(Kp_rad, Ki_rad, Kp_rpm, Ki_rpm, info)
    fprintf('\n=== pi_design (%s) ===\n', upper(info.method));
    if isfield(info, 'J');    fprintf('  J        = %.3e kg·m^2\n', info.J);   end
    if isfield(info, 'Kt');   fprintf('  Kt       = %.5f N·m/A\n', info.Kt);   end
    if isfield(info, 'T_eq'); fprintf('  T_eq     = %.3e s (%.0f μs)\n', info.T_eq, info.T_eq*1e6); end
    if isfield(info, 'a');    fprintf('  a        = %g (zeta_eq ≈ %.2f)\n', info.a, info.zeta_eq); end
    if isfield(info, 'B');    fprintf('  B        = %.3e N·m·s\n', info.B);    end
    if isfield(info, 'wn');   fprintf('  wn       = %g rad/s, zeta = %g\n', info.wn, info.zeta); end
    fprintf('\n  Kp_rad   = %.4e A·s/rad\n', Kp_rad);
    fprintf('  Ki_rad   = %.4e A/rad\n',     Ki_rad);
    fprintf('  Kp_rpm   = %.4e A/RPM\n',     Kp_rpm);
    fprintf('  Ki_rpm   = %.4e A/(RPM·s)\n', Ki_rpm);
    fprintf('\n  omega_c  = %.1f rad/s   (~%.1f Hz)\n', info.omega_c, info.omega_c/(2*pi));
    if isfield(info, 't_settle_tracking') && isfield(info, 't_disturb_settle')
        % PZC mode — show both tracking and disturbance settling
        fprintf('  t_settle (tracking) = %.2f ms (2%%)\n',  info.t_settle_tracking*1000);
        fprintf('  t_settle (disturb)  = %.2f s  (= 4*J/B; slow plant pole NOT cancelled in disturb path)\n', info.t_disturb_settle);
    else
        fprintf('  t_settle = %.2f ms (2%%)\n',  info.t_settle*1000);
    end
    fprintf('  t_rise   = %.2f ms\n',        info.t_rise*1000);
    fprintf('  verdict  = %s\n\n', info.verdict);
end
