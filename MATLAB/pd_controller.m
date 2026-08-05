function [tau_applied, qe, tau_cmd] = pd_controller(q, w, qd, Kp, Kd, tau_max)
% PD_CONTROLLER  Quaternion-error PD attitude controller.
% Error quaternion qe = qd^-1 (x) q (report Eq. error_quat_def), expanded
% per Eqs. (error_quat_vec, error_quat_scalar). This ordering is required
% (not qe = q (x) qd^-1) for qe to obey the same kinematics as q itself
% (Eq. error_quat_kinematics), which the Lyapunov proof in Sec. 4.3 relies
% on. q, qd = [eps; eta]; w = body angular velocity (rad/s).

eps  = q(1:3);  eta  = q(4);
epsd = qd(1:3); etad = qd(4);

eps_e = etad*eps - eta*epsd - skew(epsd)*eps;   % Eq. (error_quat_vec)
eta_e = eta*etad + eps.'*epsd;                 % Eq. (error_quat_scalar)
qe = [eps_e; eta_e];

tau_cmd = -Kp*sign_nz(eta_e)*eps_e - Kd*w;      % Eq. (pd_law)

tau_applied = max(-tau_max, min(tau_max, tau_cmd));  % Eq. (torque_saturation)

end

function s = sign_nz(x)
% sign() with sign(0) = +1, matching the report's sgn(eta_e) convention.
if x >= 0
    s = 1;
else
    s = -1;
end
end
