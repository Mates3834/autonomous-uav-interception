import numpy as np

from src.environment.uav_kinematics import (
    UAVState,
    propagate,
    position_vector,
)
from src.estimation.kalman_target_tracker import ConstantVelocityKalman
from src.guidance.pursuit_guidance import (
    pure_pursuit_command,
    predictive_pursuit_command,
)
from src.control.uav_controller import HeadingSpeedController


def target_heading_profile(t):
    """
    Smooth generic maneuver profile for the target UAV.
    """
    return (
        np.deg2rad(15.0)
        + np.deg2rad(18.0) * np.sin(0.045 * t)
        + np.deg2rad(7.0) * np.sin(0.11 * t)
    )


def run_simulation(
    method="predictive",
    duration=120.0,
    dt=0.1,
    capture_radius=20.0,
    seed=3,
):
    rng = np.random.default_rng(seed)

    pursuer = UAVState(
        x=0.0,
        y=0.0,
        speed=20.0,
        heading=np.deg2rad(10.0),
    )

    target = UAVState(
        x=450.0,
        y=180.0,
        speed=15.0,
        heading=np.deg2rad(15.0),
    )

    tracker = ConstantVelocityKalman(
        dt=dt,
        process_var=1.5,
        measurement_var=36.0,
    )

    controller = HeadingSpeedController()

    pursuer_log = []
    target_log = []
    measurement_log = []
    estimate_log = []
    predicted_log = []
    separation_log = []
    estimation_error_log = []
    heading_error_log = []
    control_effort_log = []

    success = False
    capture_time = None

    times = np.arange(0.0, duration + dt, dt)

    for t in times:
        target.heading = target_heading_profile(t)
        target = propagate(
            target,
            heading_rate=0.0,
            acceleration=0.0,
            dt=dt,
        )

        true_target_xy = position_vector(target)

        measurement = true_target_xy + rng.normal(0.0, 6.0, size=2)
        estimate = tracker.step(measurement)

        pursuer_xy = position_vector(pursuer)
        estimated_xy = estimate[:2]
        estimated_v = estimate[2:4]

        if method == "pure":
            heading_ref = pure_pursuit_command(
                pursuer_xy,
                estimated_xy,
            )
            predicted = estimated_xy.copy()
        elif method == "predictive":
            heading_ref, predicted = predictive_pursuit_command(
                pursuer_xy,
                estimated_xy,
                estimated_v,
                lookahead_time=1.5,
            )
        else:
            raise ValueError("method must be 'pure' or 'predictive'")

        # Keep a modest speed advantage for rendezvous.
        speed_ref = 21.0

        heading_rate, acceleration, heading_error = controller.compute(
            pursuer,
            heading_ref,
            speed_ref,
        )

        pursuer = propagate(
            pursuer,
            heading_rate,
            acceleration,
            dt,
        )

        separation = np.linalg.norm(
            position_vector(pursuer) - true_target_xy
        )

        pursuer_log.append(position_vector(pursuer))
        target_log.append(true_target_xy)
        measurement_log.append(measurement)
        estimate_log.append(estimated_xy)
        predicted_log.append(predicted)
        separation_log.append(separation)
        estimation_error_log.append(
            np.linalg.norm(estimated_xy - true_target_xy)
        )
        heading_error_log.append(abs(heading_error))
        control_effort_log.append(
            abs(heading_rate) + 0.25 * abs(acceleration)
        )

        if separation <= capture_radius:
            success = True
            capture_time = float(t)
            break

    separation_arr = np.asarray(separation_log)

    metrics = {
        "success": success,
        "capture_time_s": capture_time,
        "minimum_separation_m": float(np.min(separation_arr)),
        "mean_estimation_error_m": float(np.mean(estimation_error_log)),
        "mean_heading_error_deg": float(
            np.degrees(np.mean(heading_error_log))
        ),
        "control_effort": float(np.sum(control_effort_log) * dt),
    }

    return {
        "pursuer": np.asarray(pursuer_log),
        "target": np.asarray(target_log),
        "measurements": np.asarray(measurement_log),
        "estimates": np.asarray(estimate_log),
        "predicted": np.asarray(predicted_log),
        "separation": separation_arr,
        "time": np.arange(len(separation_arr)) * dt,
        "metrics": metrics,
    }
