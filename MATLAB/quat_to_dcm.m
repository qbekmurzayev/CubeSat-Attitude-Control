function C = quat_to_dcm(q)
% DCM from quaternion q = [eps; eta]

eps = q(1:3);
eta = q(4);
C = (eta^2 - (eps.'*eps))*eye(3) + 2*(eps*eps.') - 2*eta*skew(eps);

end
