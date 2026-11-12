% TRIAD attitude determination with noisy sensors

clear; clc; close all;
rng(42);

here = fileparts(mfilename('fullpath'));
addpath(here);
figdir = fullfile(here, '..', 'Figures');
resdir = fullfile(here, '..', 'Results');
if ~exist(figdir, 'dir'); mkdir(figdir); end
if ~exist(resdir, 'dir'); mkdir(resdir); end

p = cubesat_params();

% True trajectory (same propagation main_simulation uses)
[T, X] = rk4_integrator(p);
N = numel(T);

% Estimate attitude at every step and measure the error
err_deg = zeros(N,1);
for k = 1:N
    q_true = X(k,1:4).';
    C_true = quat_to_dcm(q_true);

    [s_meas, b_meas] = sensor_model(q_true, p);
    C_est = triad(s_meas, b_meas, p.s_I, p.b_I);

    C_err = C_est * C_true.';
    cos_ang = (trace(C_err) - 1) / 2;
    err_deg(k) = rad2deg(acos(max(-1, min(1, cos_ang))));
end

rms_err  = sqrt(mean(err_deg.^2));
mean_err = mean(err_deg);
max_err  = max(err_deg);

% Figure: determination error over time
f = figure('Position',[100 100 800 400]);
plot(T, err_deg, 'b-', 'LineWidth', 1.0);
hold on;
yline(rms_err, 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Attitude estimation error (deg)');
legend('TRIAD error', sprintf('RMS = %.3f deg', rms_err), 'Location','best');
title('Open-Loop Attitude Determination Error (TRIAD, sun + magnetometer)');
grid on;
saveas(f, fullfile(figdir, 'fig_triad_error.png'));
close(f);

% Numeric summary
det_summary = struct();
det_summary.sigma_sun_deg  = rad2deg(p.sigma_sun);
det_summary.sigma_mag_deg  = rad2deg(p.sigma_mag);
det_summary.mean_err_deg   = mean_err;
det_summary.rms_err_deg    = rms_err;
det_summary.max_err_deg    = max_err;

fid = fopen(fullfile(resdir, 'determination_summary.txt'), 'w');
fn = fieldnames(det_summary);
for i = 1:numel(fn)
    fprintf(fid, '%s = %.6g\n', fn{i}, det_summary.(fn{i}));
end
fclose(fid);

disp(det_summary);
disp('Determination demo complete. Figure -> Figures/fig_triad_error.png');
