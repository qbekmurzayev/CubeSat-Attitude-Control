function eul = quat_to_euler321(q)
% 3-2-1 Euler angles from quaternion

C = quat_to_dcm(q);
theta = -asin(max(-1, min(1, C(1,3))));
phi   = atan2(C(2,3), C(3,3));
psi   = atan2(C(1,2), C(1,1));
eul = [phi, theta, psi];

end
