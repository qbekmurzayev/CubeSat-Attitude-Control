function p = cubesat_params(gain_design)
% CUBESAT_PARAMS  Spacecraft mass properties, actuator limits, controller
% gains, and simulation scenario definition for the 3U CubeSat report
% (Documentation/report/sections/03_spacecraft_parameters.tex,
%  04_control_design.tex, 05_simulation_methodology.tex).
%
% p = cubesat_params()            uses the 'baseline' controller (report default)
% p = cubesat_params('per_axis')  uses the per-axis controller variant
%
% See the "Controller design variants" section of the README.

% --- Mass properties (Sec. 3.2-3.3) ---
p.m = 4.00;                          % kg, 3U CubeSat Design Spec max mass
a = 0.10; b = 0.10; c = 0.30;        % m, outer envelope dimensions
Jx = (p.m/12)*(b^2+c^2);
Jy = (p.m/12)*(a^2+c^2);
Jz = (p.m/12)*(a^2+b^2);
p.J = diag([Jx, Jy, Jz]);            % kg m^2, principal-axis inertia tensor
p.Jinv = inv(p.J);

% --- Reaction wheel actuator limits (Sec. 3.1, Table wheel_params) ---
p.tau_max = 1.0e-3;                  % N m, max torque per axis
p.h_max   = 10.0e-3;                 % N m s, max momentum storage (not modeled dynamically)

% --- PD controller gains (Sec. 4-5) ---
% Two documented designs are available (see README, "Controller design variants"):
%   'baseline' : scalar gains. This is the design documented in the report and
%                the one the report figures/Results correspond to.
%   'per_axis' : gains derived from a single closed-loop target (wn, zeta) and
%                scaled by each axis inertia, so all three axes share the same
%                damping character. Trades a little z-axis bandwidth for a
%                uniformly-tuned, fully-specified controller (see compare_gain_designs.m).
% Nothing else in the code depends on whether the gains are scalars or matrices.
if nargin < 1 || isempty(gain_design)
    gain_design = 'baseline';        % default: the design documented in the report
end
p.gain_design = gain_design;         % 'baseline' | 'per_axis'

switch p.gain_design
    case 'baseline'
        p.Kp = 6.0e-4;               % N m
        p.Kd = 4.4e-3;               % N m s
    case 'per_axis'
        % Linearized per-axis model near target (small error, omega_d = 0):
        %   J_i*ddtheta_i + Kd_i*dtheta_i + (Kp_i/2)*theta_i = 0
        %   =>  Kp_i = 2*J_i*wn^2 ,  Kd_i = 2*zeta*wn*J_i
        % wn, zeta chosen to reproduce the well-tuned baseline x/y response.
        wn   = 0.09487;              % rad/s, target natural frequency (all axes)
        zeta = 0.6957;               % -, target damping ratio (all axes)
        p.Kp = 2 * p.J * wn^2;       % N m,   = diag([6.0e-4 6.0e-4 1.2e-4])
        p.Kd = 2 * zeta * wn * p.J;  % N m s, = diag([4.4e-3 4.4e-3 8.8e-4])
    otherwise
        error('cubesat_params:gain_design', ...
              'Unknown gain_design "%s" (use ''baseline'' or ''per_axis'').', ...
              p.gain_design);
end

% --- Scenario: initial conditions and target attitude (Sec. 5.5) ---
p.q0 = [0; 0; 0; 1];                 % identity quaternion [eps1 eps2 eps3 eta]
w0_dps = [10; -15; 8];               % deg/s
p.w0 = w0_dps * (pi/180);            % rad/s

e_hat = [1;1;1]/sqrt(3);
phi_d = deg2rad(120);
p.qd = [e_hat*sin(phi_d/2); cos(phi_d/2)];   % [0.5 0.5 0.5 0.5]

p.T  = 120;                          % s, simulation duration
p.dt = 0.01;                         % s, fixed integration step

% --- Attitude determination: sensor model (open-loop, for the TRIAD demo) ---
% Two vector sensors observe known inertial reference directions. Real sun and
% magnetic-field vectors evolve along the orbit; over the short 120 s slew they
% are treated as constant in the inertial frame (a deliberate simplification).
% References must be non-parallel for TRIAD; here they are ~60 deg apart.
p.s_I = [1; 0; 0];                   % sun direction, inertial frame
p.b_I = [0.5; 0.5; 0.7071];          % magnetic-field direction, inertial frame
p.s_I = p.s_I / norm(p.s_I);         % normalize to unit vectors
p.b_I = p.b_I / norm(p.b_I);
p.sigma_sun = deg2rad(0.1);          % rad, 1-sigma sun-sensor noise (fine)
p.sigma_mag = deg2rad(0.5);          % rad, 1-sigma magnetometer noise (coarser)

end
