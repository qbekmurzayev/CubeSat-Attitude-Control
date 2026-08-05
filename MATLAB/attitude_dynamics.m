function [xdot, tau_applied] = attitude_dynamics(x, p)
% ATTITUDE_DYNAMICS  Closed-loop state derivative for the 7-state vector
% x = [q(4); w(3)]. Implements report Eq. (state_derivative), combining
% the quaternion kinematics (quat_kinematics) and Euler's equation
% (euler_equation), with the PD controller (pd_controller.m) evaluated
% at the current state.

q = x(1:4);
w = x(5:7);

eps = q(1:3); eta = q(4);
Xi = [eta*eye(3) + skew(eps); -eps.'];   % Eq. (quat_kinematics), Xi(q)

qdot = 0.5 * Xi * w;

[tau_applied, ~, ~] = pd_controller(q, w, p.qd, p.Kp, p.Kd, p.tau_max);

wdot = p.Jinv * (tau_applied - cross(w, p.J*w));   % Eq. (euler_equation)

xdot = [qdot; wdot];

end
