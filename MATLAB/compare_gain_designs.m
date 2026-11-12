% compares the two PD gain designs

clear; clc;

here = fileparts(mfilename('fullpath'));
addpath(here);

designs = {'baseline', 'per_axis'};

fprintf('\nController gain design comparison (3U CubeSat, identical scenario)\n');
fprintf('%s\n', repmat('=', 1, 64));
fprintf('%-10s  %11s  %14s  %13s\n', 'design', 't_settle[s]', 'final|eps_e|', 'peak tau[mNm]');
fprintf('%s\n', repmat('-', 1, 64));

for i = 1:numel(designs)
    p = cubesat_params(designs{i});

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

    settle_mask = (eps_e_norm < 0.02) & (w_norm < 0.01);
    idx = find(~settle_mask, 1, 'last');
    if isempty(idx)
        t_settle = 0;
    elseif idx == N
        t_settle = NaN;
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
