function p = cubesat_params(gain_design)
% 3U CubeSat parameters, gains and scenario

% Mass properties (Sec. 3.2-3.3)
p.m = 4.00;
a = 0.10; b = 0.10; c = 0.30;
Jx = (p.m/12)*(b^2+c^2);
Jy = (p.m/12)*(a^2+c^2);
Jz = (p.m/12)*(a^2+b^2);
p.J = diag([Jx, Jy, Jz]);
p.Jinv = inv(p.J);

p.tau_max = 1.0e-3;
p.h_max   = 10.0e-3;

if nargin < 1 || isempty(gain_design)
    gain_design = 'baseline';
end
p.gain_design = gain_design;

switch p.gain_design
    case 'baseline'
        p.Kp = 6.0e-4;   % N m
        p.Kd = 4.4e-3;   % N m s
    case 'per_axis'
        wn   = 0.09487;
        zeta = 0.6957;
        p.Kp = 2 * p.J * wn^2;
        p.Kd = 2 * zeta * wn * p.J;
    otherwise
        error('cubesat_params:gain_design', ...
              'Unknown gain_design "%s" (use ''baseline'' or ''per_axis'').', ...
              p.gain_design);
end

p.q0 = [0; 0; 0; 1];
w0_dps = [10; -15; 8];   % deg/s
p.w0 = w0_dps * (pi/180);   % rad/s

e_hat = [1;1;1]/sqrt(3);
phi_d = deg2rad(120);
p.qd = [e_hat*sin(phi_d/2); cos(phi_d/2)];   % [0.5 0.5 0.5 0.5]

p.T  = 120;   % s, simulation duration
p.dt = 0.01;

p.s_I = [1; 0; 0];
p.b_I = [0.5; 0.5; 0.7071];
p.s_I = p.s_I / norm(p.s_I);
p.b_I = p.b_I / norm(p.b_I);
p.sigma_sun = deg2rad(0.1);
p.sigma_mag = deg2rad(0.5);

end
