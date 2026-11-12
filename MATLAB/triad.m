function C_est = triad(s_meas, b_meas, s_I, b_I)
% TRIAD method

% Body triad from the measurements
r1 = s_meas;
r2 = cross(s_meas, b_meas);
r2 = r2 / norm(r2);
r3 = cross(r1, r2);

v1 = s_I;
v2 = cross(s_I, b_I);
v2 = v2 / norm(v2);
v3 = cross(v1, v2);

Mb = [r1, r2, r3];   % body triad   (columns)
Mi = [v1, v2, v3];
C_est = Mb * Mi.';

end
