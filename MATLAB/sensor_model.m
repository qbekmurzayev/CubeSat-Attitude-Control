function [s_meas, b_meas] = sensor_model(q_true, p)
% SENSOR_MODEL  Synthetic noisy vector-sensor measurements.
% Given the true attitude quaternion q_true, projects the known inertial
% reference directions (sun p.s_I, magnetic field p.b_I) into the body frame,
% adds Gaussian noise, and returns unit measurement vectors.
%
%   [s_meas, b_meas] = sensor_model(q_true, p)
%
% s_meas : measured sun direction in the body frame  (3x1 unit)
% b_meas : measured magnetic-field direction, body frame (3x1 unit)
%
% Uses randn: seed the RNG in the caller (rng(...)) for reproducible results.

C = quat_to_dcm(q_true);             % inertial -> body rotation, C*v_I = v_B

s_body = C * p.s_I;                  % true sun direction, body frame
b_body = C * p.b_I;                  % true field direction, body frame

% Additive Cartesian noise + renormalization ~ angular error of size sigma.
s_meas = s_body + p.sigma_sun * randn(3,1);
b_meas = b_body + p.sigma_mag * randn(3,1);

s_meas = s_meas / norm(s_meas);      % sensors report unit directions
b_meas = b_meas / norm(b_meas);

end
