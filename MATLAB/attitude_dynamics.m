function [xdot, tau_applied] = attitude_dynamics(x, p)
% state derivative, x = [q; w]

q = x(1:4);
w = x(5:7);

eps = q(1:3); eta = q(4);
Xi = [eta*eye(3) + skew(eps); -eps.'];

qdot = 0.5 * Xi * w;

[tau_applied, ~, ~] = pd_controller(q, w, p.qd, p.Kp, p.Kd, p.tau_max);

wdot = p.Jinv * (tau_applied - cross(w, p.J*w));   % Eq. (euler_equation)

xdot = [qdot; wdot];

end
