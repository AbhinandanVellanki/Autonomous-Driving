# Autonomous-Driving

Vehicle modeling and path-tracking control for ground vehicles: a kinematic bicycle model,
a linear dynamic bicycle model with state-feedback lateral control, and a pure-pursuit +
PID controller. Built for CMU 16-665 (Air Mobility / Autonomous Driving module).

The assignment is split across MATLAB (vehicle models + state-feedback tracking) and Python
(pure pursuit) implementations.

## Repository layout

```
Assignment/
├── pepy_model.m              # Kinematic bicycle model (Pepy KBM), as a MATLAB class
├── use_pepy.m                # Drives pepy_model with a steering input, plots the trajectory
├── curved_trajectory.m       # Open-loop reference path generator (straight + constant-curvature arcs)
├── curved_path.m             # Linear lateral-error dynamics + pole-placement tracking, curved road, V_x = 60 m/s
├── lane_change.m             # Linear lateral-error dynamics + pole-placement tracking, lane-change maneuver, V_x = 30 m/s
├── purepursuit 16665.py      # Pure pursuit steering + PID speed control on a sinusoidal path (Python)
├── images/                   # Generated result plots (git-ignored)
├── Assignment.pdf            # Problem statement
└── Solution.pdf               # Written solution/report
```

## Part 1 — Kinematic bicycle model (`pepy_model.m`, `use_pepy.m`)

`pepy_model` implements the Pepy kinematic bicycle model as a MATLAB class: given a constant
speed and a (saturated) steering angle, each `step()` integrates yaw rate
`(v / L) * tan(delta)` and the forward velocity resolved into the global frame to update
`(x, y, psi)`. `use_pepy.m` instantiates the model, drives it with a steering input (square
wave by default, sinusoidal alternative included) over a 50 s simulation, and plots the
resulting trajectory with start/end markers and a heading arrow.

## Part 2 — Linear bicycle model & state-feedback path tracking (`curved_path.m`, `lane_change.m`, `curved_trajectory.m`)

`curved_path.m` and `lane_change.m` both build the standard 4-state linear lateral-error
dynamics model — `[lateral error, lateral error rate, yaw error, yaw error rate]` — driven by
a desired yaw-rate input, parameterized by front/rear cornering stiffness, mass, yaw inertia,
and forward speed. A state-feedback gain `K` is computed with `place()` to set the closed-loop
pole locations, then the controller is simulated against:

- **`curved_path.m`** — straight → positive-curvature arc → straight → negative-curvature arc
  road, `V_x = 60 m/s`.
- **`lane_change.m`** — the same maneuver structure at `V_x = 30 m/s`, representing a lane change.

Both scripts reconstruct the true global path from the lateral/yaw error states and plot
lateral error, yaw error, steering-rate, and the global path (with zoomed views at the
curvature transitions) against the desired reference.

`curved_trajectory.m` is a standalone, controller-free reference-path generator used to sanity
check the same straight/constant-curvature segment composition independently of the feedback
model.

## Part 3 — Pure pursuit + PID speed control (`purepursuit 16665.py`)

A Python simulation of a kinematic-bicycle `Vehicle` tracking a sinusoidal reference course
(`Trajectory` class) using:

- A longitudinal PI controller (`Controller.Longitudinalcontrol`) regulating speed to a target
  velocity.
- A pure-pursuit steering law (`Controller.PurePursuitcontrol`) that selects a look-ahead point
  at distance `L` along the path and derives the steering angle from the chord/triangle geometry
  of the look-ahead segment combined with the Pepy KBM steering relation.

Run it with:

```bash
python "purepursuit 16665.py"
```

This animates the vehicle following the path live with matplotlib, drawing the vehicle outline,
wheels, steering angle, and current look-ahead target each step.

## Setup

```bash
python3 -m venv this_env
source this_env/bin/activate
pip install numpy matplotlib
```

The MATLAB scripts (`pepy_model.m`, `use_pepy.m`, `curved_path.m`, `lane_change.m`,
`curved_trajectory.m`) require MATLAB with the Control System Toolbox (for `place()`).

## Acknowledgments

Starter code for `purepursuit 16665.py` by Rathin Shah (rsshah) and Shruti Gangopadhyay
(sgangopa) for CMU 16-665. The vehicle models and controllers in this repo are the completed
assignment implementation.
