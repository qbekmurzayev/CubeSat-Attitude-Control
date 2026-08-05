% COMPARE_GAIN_DESIGNS  Runs the closed-loop attitude simulation under both
% controller gain designs and prints the performance tradeoff:
%
%   'baseline' : scalar PD gains (the design documented in the report).
%   'per_axis' : gains derived from a common (wn, zeta) target, scaled by each
%                axis inertia, so all three axes share the same damping character.
%
% This is an analysis / portfolio tool. It is NOT required for the main results;
% main_simulation.m always uses the 'baseline' design.

clear; clc;

here = fileparts(mfilename('fullpath'));
addpath(here);

designs = {'baseline', 'per_axis'};

fprintf('\nController gain design comparison (3U CubeSat, identical scenario)\n');
fprintf('%s\n', repmat('=', 1, 64));
fprintf('%-10s  %11s  %14s  %13s\n', 'design', 't_settle[s]', 'final|eps_e|', 'peak tau[mNm]');
fprintf('%s\n', repmat('-', 1, 64));

for i = 1:numel(designs)
    p = cubesat_params(designs{i});     % request this design directly

    [T, X] = rk4_integrator(p);
    Q = X(:,1:4);  W = X(:,5:7);
    N = numel(T);

    eps_e_norm = zeros(N,1);
    peak_torque = 0;
    for k = 1:N
        [tau_app, qe] = pd_controller(Q(k,:).', W(k,:).', p.qd, ...
                                      p.Kp, p.Kd, p.tau_max);
        eps_e_norm(k) = norm(qe(1:3));
        peak_torque   = max(peak_torque, max(abs(tau_app)));
    end
    w_norm = sqrt(sum(W.^2, 2));

    % Settling time: last instant the error leaves the box, then stays inside.
    settle_mask = (eps_e_norm < 0.02) & (w_norm < 0.01);
    idx = find(~settle_mask, 1, 'last');
    if isempty(idx)
        t_settle = 0;
    elseif idx == N
        t_settle = NaN;                 % never settles within the horizon
    else
        t_settle = T(idx + 1);
    end

    fprintf('%-10s  %11.2f  %14.2e  %13.3f\n', ...
        designs{i}, t_settle, eps_e_norm(end), peak_torque*1e3);
end

fprintf('%s\n', repmat('=', 1, 64));
fprintf(['Note: per_axis lowers the z-axis bandwidth to match x/y, so overall\n' ...
         'settling is slightly slower; the gain is a uniformly-tuned design in\n' ...
         'which every gain follows from one (wn, zeta) specification.\n\n']);
