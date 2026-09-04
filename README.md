# Autonomous UAV Interception and Pursuit–Evasion

A simulation-based framework for **autonomous UAV pursuit, target-state estimation, predictive guidance, and closed-loop control** in a maneuvering-target scenario.

The project investigates how an autonomous pursuer UAV can track a maneuvering aerial target using noisy measurements, estimate its motion using a **Kalman filter**, and generate guidance commands using **Pure Pursuit** and **Predictive Pursuit** strategies.

The framework is intended for research and educational studies in **UAV autonomy, Guidance, Navigation and Control (GNC), target tracking, pursuit–evasion, and autonomous rendezvous**.

> **Note:** Interception is modeled as a non-contact autonomous rendezvous/tagging problem. The repository does not contain weapon, payload, terminal-impact, or operational engagement logic.

---

## System Architecture

The framework combines target motion, state estimation, guidance, low-level control, and pursuer dynamics in a closed-loop architecture.

```text
        Maneuvering Target UAV
                 |
                 v
        Noisy Position Sensor
                 |
                 v
        +-------------------+
        |   Kalman Filter   |
        | Target Estimation |
        +-------------------+
                 |
                 v
      Estimated Target State
        [x, y, vx, vy]
                 |
                 v
     +-----------------------+
     |   Guidance Algorithm  |
     |                       |
     |  • Pure Pursuit       |
     |  • Predictive Pursuit |
     +-----------------------+
                 |
                 v
      Heading / Speed Reference
                 |
                 v
     +-----------------------+
     |   UAV Controller      |
     | Heading + Speed Loop  |
     +-----------------------+
                 |
                 v
        Pursuer UAV Model
                 |
                 v
        Relative Geometry
                 |
                 +---------> Rendezvous Check
                 |
                 +---------> Closed-Loop Feedback
```

---

## 1. Pursuer UAV Model

The pursuer is represented using a generic planar point-mass UAV model.

The state variables are

```text
x       horizontal position
y       horizontal position
V       UAV speed
psi     heading angle
```

with simplified kinematics

```text
x_dot   = V cos(psi)
y_dot   = V sin(psi)

psi_dot = omega
V_dot   = a
```

where

```text
omega = commanded heading rate
a     = commanded longitudinal acceleration
```

Heading-rate and acceleration commands are bounded to represent generic vehicle limitations.

The public model is intentionally independent of any specific UAV platform.

---

## 2. Maneuvering Target UAV

A second UAV represents the moving target.

Rather than following a constant heading, the target performs smooth time-varying maneuvers.

A generic heading profile is used to generate a dynamic pursuit–evasion problem.

```text
Target UAV
    |
    +--> Time-varying heading
    |
    +--> Constant nominal speed
    |
    +--> Dynamic target trajectory
```

This allows the guidance algorithms to be evaluated against a target whose future position is not directly known.

---

## 3. Noisy Target Measurements

The pursuer does not directly use the exact target position.

Instead, synthetic measurement noise is added to the target coordinates:

```text
z_k = H x_k + v_k
```

where

```text
z_k = measured target position
x_k = true target state
H   = measurement matrix
v_k = measurement noise
```

The resulting noisy measurements are processed by the state estimator.

---

## 4. Kalman-Based Target State Estimation

A discrete **constant-velocity Kalman filter** is used to estimate the target state.

The estimated state vector is

```text
x_hat = [x, y, vx, vy]^T
```

The prediction stage is

```text
x_hat(k|k-1) = F x_hat(k-1|k-1)
```

```text
P(k|k-1) = F P(k-1|k-1) F^T + Q
```

The measurement update is

```text
K_k = P H^T (H P H^T + R)^-1
```

```text
x_hat(k|k) =
x_hat(k|k-1) + K_k [z_k - H x_hat(k|k-1)]
```

```text
P(k|k) = (I - K_k H) P(k|k-1)
```

The filter therefore provides both estimated target position and velocity for the guidance system.

---

## 5. Pure Pursuit Guidance

The first guidance strategy is **Pure Pursuit**.

The pursuer continuously points toward the current estimated target position.

The commanded heading is

```text
psi_cmd =
atan2(
    y_target_hat - y_pursuer,
    x_target_hat - x_pursuer
)
```

The method is simple and reactive.

```text
Current Target Estimate
          |
          v
    LOS Direction
          |
          v
  Heading Command
```

Because the guidance command is based only on the current estimated position, it does not explicitly account for future target motion.

---

## 6. Predictive Pursuit Guidance

The second strategy uses the estimated target velocity to predict a short-horizon future position.

The predicted target position is

```text
p_pred =
p_target_hat +
T_lookahead * v_target_hat
```

where

```text
T_lookahead = prediction horizon
```

The commanded heading then becomes

```text
psi_cmd =
atan2(
    y_pred - y_pursuer,
    x_pred - x_pursuer
)
```

The architecture becomes

```text
Kalman Filter
     |
     +--> Estimated Position
     |
     +--> Estimated Velocity
              |
              v
       Target Prediction
              |
              v
      Predicted Position
              |
              v
       Pursuit Guidance
```

This allows the pursuer to respond to the estimated motion of the target rather than simply chasing its current position.

---

## 7. Heading and Speed Control

Guidance generates the desired heading.

A bounded proportional heading controller converts the heading error into a heading-rate command.

```text
e_psi = wrap(psi_cmd - psi)
```

```text
omega_cmd = K_psi * e_psi
```

The command is limited according to the generic UAV maneuverability constraint.

A separate speed controller is defined as

```text
a_cmd = K_v (V_ref - V)
```

with bounded acceleration.

Therefore, the complete control hierarchy is

```text
Target Estimate
      |
      v
Guidance Algorithm
      |
      v
Desired Heading
      |
      v
Heading Controller
      |
      +----> Heading Rate
      |
Speed Reference
      |
      v
Speed Controller
      |
      +----> Acceleration
                 |
                 v
             UAV Model
```

---

## 8. Pursuit–Evasion Scenario

The simulation consists of two autonomous aerial agents:

```text
Pursuer UAV
    vs.
Maneuvering Target UAV
```

The target follows a time-varying trajectory while the pursuer attempts to reduce relative separation.

The pursuer receives only noisy target-position measurements.

Therefore, the complete problem combines

```text
Target Motion
     +
Measurement Noise
     +
State Estimation
     +
Target Prediction
     +
Guidance
     +
Vehicle Control
```

within the same closed-loop simulation.

---

## 9. Autonomous Rendezvous Criterion

Interception is represented as a **capture-radius / rendezvous condition**.

Let

```text
d =
|| p_pursuer - p_target ||
```

A successful rendezvous occurs when

```text
d <= R_capture
```

where `R_capture` is a configurable safety radius.

This criterion allows pursuit and guidance performance to be evaluated without modeling physical contact or terminal engagement.

---

## 10. Guidance Comparison

Two guidance approaches are included:

| Method | Target Position | Target Velocity | Prediction |
|---|---:|---:|---:|
| Pure Pursuit | ✓ | — | — |
| Predictive Pursuit | ✓ | ✓ | ✓ |

The comparison allows the influence of target-state prediction on autonomous pursuit performance to be investigated.

---

## 11. Evaluation Metrics

The simulation evaluates several performance indicators.

### Rendezvous Success

Determines whether the pursuer reaches the specified capture radius.

### Capture Time

```text
T_capture
```

measures the elapsed time before the rendezvous condition is satisfied.

### Minimum Separation

```text
d_min =
min ||p_pursuer - p_target||
```

measures the closest approach.

### Target Estimation Error

```text
e_est =
||p_target - p_target_hat||
```

is used to evaluate Kalman-filter tracking performance.

### Heading Tracking Error

```text
e_psi =
psi_cmd - psi
```

evaluates the response of the pursuer to guidance commands.

### Control Effort

A generic control-effort metric is calculated from heading-rate and acceleration commands.

Together, these metrics enable quantitative comparison between different guidance approaches.

---

## 12. Repository Structure

```text
autonomous_uav_interception/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── src/
│   │
│   ├── environment/
│   │   ├── __init__.py
│   │   └── uav_kinematics.py
│   │
│   ├── estimation/
│   │   ├── __init__.py
│   │   └── kalman_target_tracker.py
│   │
│   ├── guidance/
│   │   ├── __init__.py
│   │   └── pursuit_guidance.py
│   │
│   ├── control/
│   │   ├── __init__.py
│   │   └── uav_controller.py
│   │
│   └── simulation/
│       ├── __init__.py
│       └── pursuit_evasion.py
│
└── examples/
    ├── run_demo.py
    └── compare_guidance.py
```

---

## 13. Module Description

| Module | Purpose |
|---|---|
| `uav_kinematics.py` | Generic planar UAV dynamics |
| `kalman_target_tracker.py` | Kalman-based target position and velocity estimation |
| `pursuit_guidance.py` | Pure Pursuit and Predictive Pursuit guidance |
| `uav_controller.py` | Heading and speed control |
| `pursuit_evasion.py` | Integrated closed-loop simulation |
| `run_demo.py` | Main simulation and visualization |
| `compare_guidance.py` | Guidance-method comparison |

---

## 14. Running the Project

Install the dependencies:

```bash
pip install -r requirements.txt
```

Run the main simulation:

```bash
python examples/run_demo.py
```

Compare Pure Pursuit and Predictive Pursuit:

```bash
python examples/compare_guidance.py
```

---

## 15. Example Outputs

The framework can generate:

- Pursuer and target trajectories
- Kalman-estimated target trajectory
- Relative separation versus time
- Target-state estimation error
- Guidance performance metrics
- Pure Pursuit vs. Predictive Pursuit comparison

Example result figures can be added to:

```text
results/
├── pursuit_trajectory.png
├── kalman_tracking.png
├── separation_history.png
└── guidance_comparison.png
```

---

## Technologies

- Python
- NumPy
- Matplotlib
- Kalman Filtering
- Autonomous Guidance
- Closed-Loop Simulation

---

## Research Areas

This project is related to:

- Autonomous UAV Systems
- Guidance, Navigation and Control
- UAV Autonomy
- Target Tracking
- State Estimation
- Kalman Filtering
- Predictive Guidance
- Pursuit–Evasion
- Autonomous Rendezvous
- Multi-Agent Autonomous Systems

---

## Project Motivation

Autonomous aerial systems operating in shared or restricted airspace may require reliable **tracking, prediction, guidance and rendezvous capabilities**.

This project investigates the integration of these components rather than treating target estimation and UAV guidance as isolated problems.

The main architecture can be summarized as:

```text
Sensing
   ↓
Target Tracking
   ↓
State Estimation
   ↓
Motion Prediction
   ↓
Guidance
   ↓
Control
   ↓
Autonomous UAV Motion
```

The modular structure also provides a foundation for future research involving more advanced UAV dynamics, nonlinear estimation, optimization-based guidance, sensor fusion and multi-agent coordination.

---

## Possible Future Extensions

Future research extensions may include:

- 3-D UAV dynamics
- Extended Kalman Filter (EKF)
- Unscented Kalman Filter (UKF)
- Multi-sensor target tracking
- Camera-based target measurements
- Radar-camera sensor fusion
- Model Predictive Control
- Model Predictive Guidance
- Multiple-target tracking
- Multiple-pursuer coordination
- Dynamic obstacle avoidance
- Uncertainty-aware trajectory prediction

---

## Public Implementation Notice

The source code provided in this repository contains **generic and sanitized implementations** developed to demonstrate the underlying autonomous-systems algorithms.

The public version intentionally excludes:

- Platform-specific UAV parameters
- Operational mission configurations
- Restricted airspace information
- Payload or weapon models
- Terminal-impact logic
- Operational engagement logic
- Unpublished experimental datasets
- Sensitive implementation details

The repository should therefore be interpreted as a **research and educational autonomous UAV pursuit, tracking and rendezvous framework**, rather than an operational interception system.

---

## Status

**Research-oriented project / active development**

The current public implementation includes the core simulation, target estimation, guidance, and control architecture. Additional algorithms and experimental comparisons may be incorporated in future versions.

---

## Author

**Mehmet Ateş**

Research interests:

- Autonomous Systems
- UAV Guidance and Control
- Guidance, Navigation and Control (GNC)
- State Estimation
- Target Tracking
- Path Planning
- Reinforcement Learning
- Marine and Aerial Robotics
