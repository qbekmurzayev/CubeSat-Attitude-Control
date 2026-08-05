% MAIN_SIMULATION  Driver script for the quaternion-based 3U CubeSat
% attitude dynamics and control simulation. Produces the figures used in
% Documentation/report/sections/06_results.tex and the numeric checks
% used in 07_validation_discussion.tex.

clear; clc; close all;

here = fileparts(mfilename('fullpath'));
addpath(here);
figdir = fullfile(here, '..', 'Figures');
resdir = fullfile(here, '..', 'Results');
if ~exist(figdir, 'dir'); mkdir(figdir); end
if ~exist(resdir, 'dir'); mkdir(resdir); end

p = cubesat_params();

% --- RK4 propagation (primary integrator, Sec. 5.2) ---
[T, X] = rk4_integrator(p);
N = numel(T);

Q = X(:,1:4);
W = X(:,5:7);

% --- Post-processing: recompute controller terms and diagnostics ---
QE      = zeros(N,4);
TAUCMD  = zeros(N,3);
TAUAPP  = zeros(N,3);
EUL     = zeros(N,3);
V       = zeros(N,1);
qnorm_err = zeros(N,1);

for k = 1:N
    q = Q(k,:).'; w = W(k,:).';
    [tau_app, qe, tau_cmd] = pd_controller(q, w, p.qd, p.Kp, p.Kd, p.tau_max);
    QE(k,:) = qe.';
    TAUCMD(k,:) = tau_cmd.';
    TAUAPP(k,:) = tau_app.';
    EUL(k,:) = rad2deg(quat_to_euler321(q));
    % Lyapunov candidate V = attitude potential + rotational kinetic energy.
    % Matrix-gain form: 2*eps_e'*Kp*eps_e/(1+|eta_e|) reduces exactly to the
    % scalar version 2*Kp*(1-|eta_e|) since eps_e'*eps_e = 1 - eta_e^2.
    V(k) = 2*(qe(1:3).'*p.Kp*qe(1:3))/(1 + abs(qe(4))) + 0.5*(w.'*p.J*w);
    qnorm_err(k) = abs(norm(q) - 1);
end

eps_e_norm = sqrt(sum(QE(:,1:3).^2,2));
w_norm     = sqrt(sum(W.^2,2));

% --- Settling time: first time after which eps_e_norm < 0.02 AND
%     w_norm < 0.01 rad/s for the remainder of the simulation ---
settle_mask = (eps_e_norm < 0.02) & (w_norm < 0.01);
settle_idx = find(~settle_mask, 1, 'last');
if isempty(settle_idx)
    t_settle = 0;
elseif settle_idx == N
    t_settle = NaN; % never settles within T
else
    t_settle = T(settle_idx + 1);
end

peak_torque = max(abs(TAUAPP(:)));
n_saturated = sum(any(abs(TAUCMD) >= p.tau_max - 1e-12, 2));
final_eps_e_norm = eps_e_norm(end);
final_w_norm = w_norm(end);
max_qnorm_err = max(qnorm_err);

% --- ode45 cross-check (Sec. 5.2 validation) ---
odefun = @(t,x) local_ode_wrap(x, p);
opts = odeset('RelTol',1e-10,'AbsTol',1e-12);
sol45 = ode45(odefun, [0 p.T], [p.q0; p.w0], opts);
X45 = deval(sol45, T).';
Q45 = X45(:,1:4);
attitude_diff = zeros(N,1);
for k = 1:N
    % angle between RK4 and ode45 quaternions via dot product
    d = abs(Q(k,:) * Q45(k,:).');
    d = min(1, d);
    attitude_diff(k) = rad2deg(2*acos(d));
end
max_attitude_diff_deg = max(attitude_diff);

% =====================================================================
% Figures
% =====================================================================

% 1. Quaternion response
f1 = figure('Position',[100 100 800 500]);
plot(T, Q(:,1), 'r-', T, Q(:,2), 'g-', T, Q(:,3), 'b-', T, Q(:,4), 'k-', 'LineWidth', 1.2);
hold on;
yline(p.qd(1), 'r--'); yline(p.qd(2), 'g--'); yline(p.qd(3), 'b--'); yline(p.qd(4), 'k--');
xlabel('Time (s)'); ylabel('Quaternion component');
legend('q_1','q_2','q_3','q_4 (scalar)','Location','best');
title('Attitude Quaternion Response (solid) vs. Target (dashed)');
grid on;
saveas(f1, fullfile(figdir, 'fig_quaternion_response.png'));

% 2. Angular velocity response
f2 = figure('Position',[100 100 800 500]);
plot(T, rad2deg(W(:,1)), 'r-', T, rad2deg(W(:,2)), 'g-', T, rad2deg(W(:,3)), 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angular velocity (deg/s)');
legend('\omega_x','\omega_y','\omega_z','Location','best');
title('Body Angular Velocity Response');
grid on;
saveas(f2, fullfile(figdir, 'fig_angular_velocity.png'));

% 3. Control torque
f3 = figure('Position',[100 100 800 500]);
plot(T, TAUAPP(:,1)*1e3, 'r-', T, TAUAPP(:,2)*1e3, 'g-', T, TAUAPP(:,3)*1e3, 'b-', 'LineWidth', 1.2);
hold on;
yline(p.tau_max*1e3, 'k--'); yline(-p.tau_max*1e3, 'k--');
xlabel('Time (s)'); ylabel('Applied wheel torque (mN\cdotm)');
legend('\tau_x','\tau_y','\tau_z','\pm\tau_{max}','Location','best');
title('Reaction Wheel Control Torque (saturated)');
grid on;
saveas(f3, fullfile(figdir, 'fig_control_torque.png'));

% 4. Quaternion norm error
f4 = figure('Position',[100 100 800 400]);
semilogy(T, max(qnorm_err, 1e-18), 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('|| q || - 1  (abs)');
title('Quaternion Norm Drift (RK4 + per-step renormalization)');
grid on;
saveas(f4, fullfile(figdir, 'fig_quaternion_norm_error.png'));

% 5. Euler angles (3-2-1)
f5 = figure('Position',[100 100 800 500]);
plot(T, EUL(:,1), 'r-', T, EUL(:,2), 'g-', T, EUL(:,3), 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Euler angle (deg)');
legend('\phi (roll)','\theta (pitch)','\psi (yaw)','Location','best');
title('Equivalent 3-2-1 Euler Angles (post-processed from quaternion)');
grid on;
saveas(f5, fullfile(figdir, 'fig_euler_angles.png'));

% 6. Lyapunov function
f6 = figure('Position',[100 100 800 400]);
semilogy(T, max(V,1e-18), 'm-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('V(t)  (J, log scale)');
title('Lyapunov Function Decay');
grid on;
saveas(f6, fullfile(figdir, 'fig_lyapunov.png'));

% 7. RK4 vs ode45 cross-check
f7 = figure('Position',[100 100 800 400]);
plot(T, attitude_diff, 'k-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Attitude difference (deg)');
title('RK4 vs. ode45 Cross-Check: Attitude Discrepancy');
grid on;
saveas(f7, fullfile(figdir, 'fig_integrator_crosscheck.png'));

close all;

% =====================================================================
% Numeric summary (used directly in Sec. 6-7 text)
% =====================================================================
summary = struct();
summary.t_settle_s = t_settle;
summary.final_eps_e_norm = final_eps_e_norm;
summary.final_w_norm_radps = final_w_norm;
summary.final_w_norm_dps = rad2deg(final_w_norm);
summary.peak_torque_Nm = peak_torque;
summary.peak_torque_mNm = peak_torque*1e3;
summary.n_saturated_samples = n_saturated;
summary.frac_saturated = n_saturated/N;
summary.max_qnorm_err = max_qnorm_err;
summary.max_attitude_diff_vs_ode45_deg = max_attitude_diff_deg;

save(fullfile(resdir, 'simulation_summary.mat'), 'summary', 'T', 'Q', 'W', 'TAUAPP', 'EUL', 'V', 'eps_e_norm');

fid = fopen(fullfile(resdir, 'simulation_summary.txt'), 'w');
fn = fieldnames(summary);
for i = 1:numel(fn)
    fprintf(fid, '%s = %.6g\n', fn{i}, summary.(fn{i}));
end
fclose(fid);

disp(summary);
disp('Simulation complete. Figures written to Figures/, summary written to Results/.');

function xdot = local_ode_wrap(x, p)
    xdot = attitude_dynamics(x, p);
end
