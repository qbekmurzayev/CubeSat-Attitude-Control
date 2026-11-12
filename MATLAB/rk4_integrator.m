function [T, X] = rk4_integrator(p)
% fixed step RK4 with quaternion normalization

N = round(p.T/p.dt);
T = (0:N).' * p.dt;
X = zeros(N+1, 7);

x = [p.q0; p.w0];
X(1,:) = x.';

for k = 1:N
    k1 = attitude_dynamics(x, p);
    k2 = attitude_dynamics(x + p.dt/2*k1, p);
    k3 = attitude_dynamics(x + p.dt/2*k2, p);
    k4 = attitude_dynamics(x + p.dt*k3, p);

    x = x + (p.dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x(1:4) = x(1:4) / norm(x(1:4));

    X(k+1,:) = x.';
end

end
