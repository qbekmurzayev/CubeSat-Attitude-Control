function [s_meas, b_meas] = sensor_model(q_true, p)
% noisy sun and magnetic field vectors

C = quat_to_dcm(q_true);

s_body = C * p.s_I;
b_body = C * p.b_I;

s_meas = s_body + p.sigma_sun * randn(3,1);
b_meas = b_body + p.sigma_mag * randn(3,1);

s_meas = s_meas / norm(s_meas);
b_meas = b_meas / norm(b_meas);

end
