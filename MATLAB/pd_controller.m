function [tau_applied, qe, tau_cmd] = pd_controller(q, w, qd, Kp, Kd, tau_max)
% quaternion error PD controller

eps  = q(1:3);  eta  = q(4);
epsd = qd(1:3); etad = qd(4);

eps_e = etad*eps - eta*epsd - skew(epsd)*eps;   % Eq. (error_quat_vec)
eta_e = eta*etad + eps.'*epsd;
qe = [eps_e; eta_e];

tau_cmd = -Kp*sign_nz(eta_e)*eps_e - Kd*w;   % Eq. (pd_law)

tau_applied = max(-tau_max, min(tau_max, tau_cmd));

end

function s = sign_nz(x)
if x >= 0
    s = 1;
else
    s = -1;
end
end
