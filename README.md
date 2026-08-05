# CubeSat Attitude Determination and Control

**Quaternion-based attitude dynamics and PD control simulation for a 3U CubeSat, in MATLAB.**

This project simulates a 3U CubeSat that deploys into a tumble and must slew to and
hold a commanded attitude. It implements the full rigid-body rotational dynamics,
a quaternion attitude representation, and a quaternion-error PD controller, and
propagates the closed loop with a hand-written fixed-step RK4 integrator that is
cross-validated against MATLAB's `ode45`.

It is the software companion to the full theoretical report:
**[Read the report (PDF)](Documentation/report/report.pdf)**
(LaTeX source in [`Documentation/report`](Documentation/report)).

---

## What it does

- **Rigid-body dynamics** — Euler's rotational equation `J*wdot = tau - w x (J*w)`,
  including the gyroscopic coupling that makes the problem nonlinear.
- **Quaternion attitude** — vector-first convention `q = [eps1 eps2 eps3 eta]`,
  with kinematics `qdot = 0.5*Xi(q)*w` and per-step renormalization.
- **PD attitude control** — quaternion-error PD law with anti-unwinding
  (`sgn(eta_e)`) and reaction-wheel torque saturation.
- **Numerical simulation** — fixed-step RK4, validated against `ode45`, with a
  Lyapunov-function decay diagnostic.
- **Attitude determination (open-loop)** — synthetic sun + magnetometer sensors
  with noise, reconstructed via the TRIAD algorithm; estimation error measured
  against truth.

---

## Repository layout

```
CubeSat_Attitude_Dynamics/
├── MATLAB/
│   ├── main_simulation.m     % driver: runs the sim, post-processes, plots, saves
│   ├── cubesat_params.m      % single source of truth: mass, inertia, gains, scenario
│   ├── attitude_dynamics.m   % state derivative xdot = f(x): kinematics + Euler's eqn
│   ├── pd_controller.m       % quaternion-error PD control law + saturation
│   ├── rk4_integrator.m      % fixed-step RK4 with quaternion renormalization
│   ├── skew.m                % skew-symmetric cross-product matrix helper
│   ├── quat_to_dcm.m         % quaternion -> direction cosine matrix
│   ├── quat_to_euler321.m    % quaternion -> 3-2-1 Euler angles (for display only)
│   ├── compare_gain_designs.m % analysis tool: baseline vs per-axis gain tradeoff
│   ├── sensor_model.m        % synthetic noisy sun + magnetometer measurements
│   ├── triad.m               % TRIAD deterministic two-vector attitude determination
│   └── determination_demo.m  % open-loop TRIAD demo: estimation error vs truth
├── Figures/                  % generated plots (PNG)
├── Results/                  % generated numeric summary (.mat and .txt)
└── Documentation/report/     % LaTeX theoretical report
```

**Design principle:** parameters, physics, control, numerics, and presentation each
live in their own file. `attitude_dynamics.m` is stateless (`xdot = f(x)`), which is
exactly why the same function can be driven by both the custom RK4 integrator and
`ode45` — enabling the integrator cross-check.

---

## The model (summary)

| Element | Equation |
|---|---|
| State (7) | `x = [eps1 eps2 eps3 eta, wx wy wz]` |
| Kinematics | `qdot = 0.5 * Xi(q) * w` |
| Dynamics | `wdot = J^-1 (tau - w x (J*w))` |
| Error quaternion | `qe = qd^-1 (x) q` |
| Control law | `tau = -Kp*sgn(eta_e)*eps_e - Kd*w`, saturated to `+/- tau_max` |

Quaternions are used internally throughout; Euler angles are computed only for
human-readable plots, precisely because of the 3-2-1 gimbal-lock singularity at
`theta = +/-90 deg`.

---

## Spacecraft and scenario

| Parameter | Value |
|---|---|
| Form factor / mass | 3U (0.1 x 0.1 x 0.3 m), 4.0 kg |
| Inertia (principal) | `diag([0.0333, 0.0333, 0.00667])` kg·m² |
| Initial tumble | `[10, -15, 8]` deg/s |
| Target attitude | 120° about `[1,1,1]/sqrt(3)` |
| Wheel torque limit | 1 mN·m per axis |
| PD gains `Kp`, `Kd` (baseline) | 6.0e-4 N·m, 4.4e-3 N·m·s (scalar) |
| Simulation | 120 s at 100 Hz (RK4) |

The baseline uses **scalar** PD gains, tuned for a well-damped response
(`zeta ≈ 0.70`) on the two stiff transverse axes. An alternative **per-axis**
design is also provided — see [Controller design variants](#controller-design-variants).

---

## How to run

Requires MATLAB (developed on R2024a; no toolboxes required).

```matlab
cd MATLAB
main_simulation
```

Or from a shell:

```bash
matlab -batch "cd MATLAB; main_simulation"
```

This regenerates every PNG in `Figures/` and writes `Results/simulation_summary.txt`
and `.mat`.

---

## Results

The controller detumbles and slews the spacecraft to the target attitude and holds it.

| Metric | Value (baseline) |
|---|---|
| Settling time (eps error < 0.02 and rate < 0.01 rad/s, and stays) | ≈ 74.1 s |
| Final attitude error `‖eps_e‖` | 9.6e-4 |
| Final rate | 0.010 deg/s |
| Peak wheel torque | 1.0 mN·m (saturated ~4.7% of samples early on) |
| Max quaternion norm drift | 2.2e-16 (machine precision) |
| Max attitude disagreement vs `ode45` | 1.3e-4 deg |

Key figures (`Figures/`):

- `fig_quaternion_response.png` — quaternion tracking to target
- `fig_angular_velocity.png` — detumble of the body rates
- `fig_control_torque.png` — commanded torque with saturation limits
- `fig_euler_angles.png` — equivalent 3-2-1 Euler angles
- `fig_lyapunov.png` — monotonic Lyapunov-function decay (stability evidence)
- `fig_quaternion_norm_error.png` — norm drift held at machine precision
- `fig_integrator_crosscheck.png` — RK4 vs `ode45` agreement

---

## Controller design variants

The controller ships with two documented gain designs, selected by a single
argument to `cubesat_params`:

- **`baseline`** (default) — scalar gains `Kp = 6.0e-4`, `Kd = 4.4e-3`. This is
  the design documented in the report and used by `main_simulation.m`.
- **`per_axis`** — gains derived from a single closed-loop target
  (`wn ≈ 0.095` rad/s, `zeta ≈ 0.70`) scaled by each axis inertia
  (`Kp_i = 2*J_i*wn^2`, `Kd_i = 2*zeta*wn*J_i`), giving
  `Kp = diag([6.0, 6.0, 1.2])e-4`, `Kd = diag([4.4, 4.4, 0.88])e-3`.
  All three axes then share the same damping character.

Because the baseline scalars were tuned on the stiff transverse axes, the
low-inertia z-axis is *overdamped* under the baseline. The `per_axis` design
fixes that inconsistency — but, since it lowers the z-axis bandwidth to match
x/y, overall settling is marginally *slower*. The lesson: overdamped is not the
same as slow; the baseline z-axis was overdamped yet had higher bandwidth. The
`per_axis` design's value is **uniform, fully-specified tuning**, not raw speed.

Run the comparison yourself:

```matlab
compare_gain_designs
```

| Design | Settling time | Final `‖eps_e‖` | Peak torque |
|---|---|---|---|
| `baseline` | 74.1 s | 9.6e-4 | 1.00 mN·m |
| `per_axis` | 78.8 s | 1.5e-3 | 1.00 mN·m |

To run the full simulation under the variant, change one line in
`cubesat_params.m` (`p.gain_design = 'per_axis';`) or call
`cubesat_params('per_axis')`.

---

## Attitude determination (open-loop)

The `determination_demo.m` script closes the "determination" half of the project.
Along the true trajectory it generates noisy **sun sensor** (0.1°) and
**magnetometer** (0.5°) measurements and reconstructs the attitude at every step
with the **TRIAD** algorithm, then compares the estimate to truth.

```matlab
determination_demo
```

| Metric | Value |
|---|---|
| Sun / magnetometer noise (1σ) | 0.1° / 0.5° |
| Mean estimation error | 0.50° |
| RMS estimation error | 0.60° |
| Max estimation error | 2.38° |

The RMS error is set by the *weaker* sensor and the measurement geometry:
`sigma_mag / sin(separation) = 0.5° / sin(60°) ≈ 0.58°`, matching the simulated
0.60°. The error is a stationary noise band (no drift) — the signature of a
memoryless deterministic estimator. See `Figures/fig_triad_error.png`.

This is **open-loop**: the estimate is compared to truth but not fed to the
controller, so the validated control loop is unchanged. Feeding the estimate
back (closed-loop estimation-and-control), optimal weighting (QUEST), and
temporal filtering (EKF) are the natural next steps.

---

## Limitations and future work

Deliberately scoped as an undergraduate control study. Not modeled:

- **Attitude determination** — a deterministic two-vector estimator (TRIAD) runs
  open-loop; it is not yet fed back to the controller, and statistical methods
  (QUEST for optimal weighting, EKF for temporal filtering) are future work.
- **Actuator dynamics** — reaction wheels appear only as a torque saturation limit;
  wheel momentum, spin-up, and magnetorquer desaturation are not simulated.
- **Environment** — no disturbance torques (gravity gradient, aerodynamic, solar).
- **Digital control** — the controller is evaluated in continuous time, not at a
  fixed discrete rate with zero-order hold.
- **Robustness** — a single nominal case; no Monte Carlo over dispersions.

---

## Author

Maxat Bekmurzayev — aerospace engineering project (GNC).

## License

Released under the MIT License — see [LICENSE](LICENSE).
