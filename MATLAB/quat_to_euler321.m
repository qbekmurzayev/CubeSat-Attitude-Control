function eul = quat_to_euler321(q)
% QUAT_TO_EULER321  3-2-1 (yaw-pitch-roll) Euler angles [phi theta psi]
% (rad) from an attitude quaternion, via the DCM. theta = -asin(C13) is
% the angle that goes singular (report Eq. euler_kinematics) at +-90 deg.

C = quat_to_dcm(q);
theta = -asin(max(-1, min(1, C(1,3))));
phi   = atan2(C(2,3), C(3,3));
psi   = atan2(C(1,2), C(1,1));
eul = [phi, theta, psi];

end
