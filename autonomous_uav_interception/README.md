# Autonomous UAV Interception and Pursuit-Evasion

Generic and sanitized Python framework for studying autonomous UAV pursuit,
rendezvous and target-tracking problems in a simulated environment.

The project demonstrates:

- 2-D UAV kinematic models
- Maneuvering target simulation
- Noisy target-position measurements
- Kalman-based target-state estimation
- Pure pursuit guidance
- Predictive pursuit guidance
- Heading and speed control
- Capture-radius based rendezvous logic
- Comparative simulation metrics

> This repository is intended for autonomous-systems, robotics and airspace-safety
> research. "Interception" is modeled as a non-contact rendezvous/tagging problem
> with a configurable safety radius. No weapon, payload, terminal-impact or
> operational engagement logic is included.

---

## Architecture

```text
Maneuvering Target UAV
          |
          v
   Noisy Measurements
          |
          v
  Kalman State Estimator
          |
          v
 Estimated Position / Velocity
          |
          v
 Pure Pursuit or Predictive Guidance
          |
          v
 Heading / Speed Commands
          |
          v
   Pursuer UAV Controller
          |
          v
   UAV Kinematic Model
          |
          +------> relative geometry / capture check
```

---

## Main Components

### 1. UAV Kinematics

A planar point-mass UAV model is used:

```text
x_dot   = V cos(psi)
y_dot   = V sin(psi)
psi_dot = omega
V_dot   = a
```

The model is intentionally generic and is not tied to a specific airframe.

### 2. Maneuvering Target

The target can follow a smooth time-varying heading profile to create a
pursuit-evasion scenario.

### 3. Kalman Target Tracking

A constant-velocity discrete Kalman filter estimates:

```text
[x, y, vx, vy]
```

from noisy position measurements.

### 4. Pure Pursuit Guidance

The pursuer points toward the current estimated target position.

### 5. Predictive Pursuit Guidance

A short look-ahead prediction is generated from the estimated target velocity:

```text
p_pred = p_hat + T_lookahead * v_hat
```

The pursuer then tracks the predicted rendezvous point.

### 6. Low-Level Control

A generic heading-rate controller and speed controller convert guidance
commands into bounded kinematic inputs.

### 7. Rendezvous Criterion

Success is defined by entering a configurable capture/safety radius around the
target. The model does not simulate impact or physical engagement.

---

## Repository Structure

```text
autonomous_uav_interception/
├── README.md
├── requirements.txt
├── .gitignore
├── src/
│   ├── environment/
│   │   └── uav_kinematics.py
│   ├── estimation/
│   │   └── kalman_target_tracker.py
│   ├── guidance/
│   │   └── pursuit_guidance.py
│   ├── control/
│   │   └── uav_controller.py
│   └── simulation/
│       └── pursuit_evasion.py
└── examples/
    ├── run_demo.py
    └── compare_guidance.py
```

---

## Evaluation Metrics

The example simulations report:

- Rendezvous success
- Time to capture-radius entry
- Minimum separation
- Mean target-estimation error
- Mean heading error
- Total control effort

---

## Installation

```bash
pip install -r requirements.txt
```

---

## Run

```bash
python examples/run_demo.py
```

or compare guidance methods:

```bash
python examples/compare_guidance.py
```

---

## Technologies

- Python
- NumPy
- Matplotlib

---

## Research Areas

- Autonomous UAV guidance
- Pursuit-evasion
- Target tracking
- State estimation
- Guidance, navigation and control
- Cooperative and safety-oriented autonomous systems

---

## Public Implementation Notice

This public version is intentionally generic and sanitized. It does not contain
platform-specific parameters, operational mission data, restricted airspace
information, terminal-impact logic, payload logic, or unpublished research
configuration values.

## Status

Research-oriented educational implementation.
