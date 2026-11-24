"""Low-speed no-slip Ackermann vehicle, not a validated dynamic plant."""
from dataclasses import dataclass
from math import cos, sin, tan
from .geometry import Pose, Vehicle


@dataclass(frozen=True)
class Command:
    steer: float
    velocity: float


def step(pose: Pose, command: Command, vehicle: Vehicle, dt: float) -> Pose:
    if dt <= 0:
        raise ValueError("dt must be positive")
    if abs(command.steer) > vehicle.max_steer + 1e-9:
        raise ValueError("steering exceeds physical bound")
    yaw_rate = command.velocity * tan(command.steer) / vehicle.wheelbase
    half = pose.yaw + yaw_rate * dt / 2
    return Pose(pose.x + command.velocity * dt * cos(half), pose.y + command.velocity * dt * sin(half), pose.yaw + yaw_rate * dt)
