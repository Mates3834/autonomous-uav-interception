import numpy as np


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class HeadingSpeedController:
    """Generic bounded proportional heading and speed controller."""

    def __init__(
        self,
        heading_gain=2.5,
        speed_gain=0.8,
        max_heading_rate=np.deg2rad(45.0),
        max_acceleration=4.0,
    ):
        self.heading_gain = float(heading_gain)
        self.speed_gain = float(speed_gain)
        self.max_heading_rate = float(max_heading_rate)
        self.max_acceleration = float(max_acceleration)

    def compute(self, state, heading_ref, speed_ref):
        heading_error = wrap_angle(heading_ref - state.heading)

        heading_rate = np.clip(
            self.heading_gain * heading_error,
            -self.max_heading_rate,
            self.max_heading_rate,
        )

        acceleration = np.clip(
            self.speed_gain * (speed_ref - state.speed),
            -self.max_acceleration,
            self.max_acceleration,
        )

        return float(heading_rate), float(acceleration), float(heading_error)
