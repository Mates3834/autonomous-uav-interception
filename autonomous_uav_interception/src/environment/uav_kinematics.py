from dataclasses import dataclass
import numpy as np


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class UAVState:
    x: float
    y: float
    speed: float
    heading: float


def propagate(state, heading_rate, acceleration, dt,
              min_speed=3.0, max_speed=35.0,
              max_heading_rate=np.deg2rad(45.0)):
    """Propagate a generic planar UAV point-mass model."""
    heading_rate = float(np.clip(
        heading_rate,
        -max_heading_rate,
        max_heading_rate,
    ))
    speed = float(np.clip(
        state.speed + acceleration * dt,
        min_speed,
        max_speed,
    ))
    heading = wrap_angle(state.heading + heading_rate * dt)

    x = state.x + speed * np.cos(heading) * dt
    y = state.y + speed * np.sin(heading) * dt

    return UAVState(x=x, y=y, speed=speed, heading=heading)


def velocity_vector(state):
    return np.array([
        state.speed * np.cos(state.heading),
        state.speed * np.sin(state.heading),
    ])


def position_vector(state):
    return np.array([state.x, state.y])
