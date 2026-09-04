import numpy as np


def bearing_to_point(origin_xy, target_xy):
    delta = np.asarray(target_xy, dtype=float) - np.asarray(origin_xy, dtype=float)
    return float(np.arctan2(delta[1], delta[0]))


def pure_pursuit_command(pursuer_xy, estimated_target_xy):
    """Heading command toward the current estimated target position."""
    return bearing_to_point(pursuer_xy, estimated_target_xy)


def predictive_pursuit_command(
    pursuer_xy,
    estimated_target_xy,
    estimated_target_velocity,
    lookahead_time=1.5,
):
    """
    Heading command toward a short-horizon predicted target position.

    This is a generic rendezvous-oriented prediction, not terminal-impact logic.
    """
    target_xy = np.asarray(estimated_target_xy, dtype=float)
    target_v = np.asarray(estimated_target_velocity, dtype=float)

    predicted = target_xy + float(lookahead_time) * target_v
    heading = bearing_to_point(pursuer_xy, predicted)
    return heading, predicted
