function C = quat_to_dcm(q)
% QUAT_TO_DCM  Direction cosine matrix from attitude quaternion.
% q = [eps1; eps2; eps3; eta], implements report Eq. (quat_to_dcm):
%   C(q) = (eta^2 - eps'*eps) I3 + 2*eps*eps' - 2*eta*skew(eps)

eps = q(1:3);
eta = q(4);
C = (eta^2 - (eps.'*eps))*eye(3) + 2*(eps*eps.') - 2*eta*skew(eps);

end
