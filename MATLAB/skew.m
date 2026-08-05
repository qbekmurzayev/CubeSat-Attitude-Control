function S = skew(v)
% SKEW  Skew-symmetric cross-product matrix of a 3-vector.
% For any 3-vectors v and u:  skew(v)*u = cross(v, u) = v x u.
% Centralizes the [vx] operator used by the quaternion kinematics
% (attitude_dynamics.m), the error-quaternion product (pd_controller.m),
% and the direction cosine matrix (quat_to_dcm.m).

S = [ 0    -v(3)  v(2);
      v(3)  0    -v(1);
     -v(2)  v(1)  0   ];

end
