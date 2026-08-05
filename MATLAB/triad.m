function C_est = triad(s_meas, b_meas, s_I, b_I)
% TRIAD  Deterministic two-vector attitude determination (Black, 1964).
% Reconstructs the body-from-inertial DCM from two measured body-frame unit
% vectors and their known inertial-frame references.
%
%   C_est = triad(s_meas, b_meas, s_I, b_I)
%
% s_meas, b_meas : measured directions in the body frame (3x1 unit)
% s_I,    b_I    : corresponding reference directions, inertial frame (3x1 unit)
% C_est          : estimated DCM, C_est * v_I ~ v_B
%
% IMPORTANT: the FIRST vector is trusted exactly (its direction is preserved in
% the estimate); the second only resolves the remaining rotation. Pass the more
% accurate sensor (here the sun sensor) as the first argument.

% --- Body triad from the measurements ---
r1 = s_meas;                         % trusted axis, carried exactly
r2 = cross(s_meas, b_meas);
r2 = r2 / norm(r2);                  % perpendicular to both measurements
r3 = cross(r1, r2);                  % completes the right-handed set

% --- Reference triad from the known inertial directions (same construction) ---
v1 = s_I;
v2 = cross(s_I, b_I);
v2 = v2 / norm(v2);
v3 = cross(v1, v2);

% --- The rotation between the two orthonormal triads is the attitude ---
Mb = [r1, r2, r3];                   % body triad   (columns)
Mi = [v1, v2, v3];                   % reference triad (columns)
C_est = Mb * Mi.';                   % Mb = C_est * Mi  =>  C_est = Mb * Mi'

end
