"""Optional Vehicle-Dynamics *kinematic* plant adapter.

The external model uses CoG positions (state [Xcg,Ycg,yaw,v,steer]),
while ADAS uses the rear axle midpoint. This adapter transforms both ways.
It is not an adapter for the seven-DOF tire-force model and does not
provide force-level brake/drive or real vehicle validation.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Protocol
from .geometry import Pose, Vehicle
from .plant import Command

class ParkingPlant(Protocol):
    def reset(self, pose: Pose) -> None: ...
    def step(self, command: Command, dt: float) -> Pose: ...

@dataclass(frozen=True)
class KinematicAdapterLimits:
    accel_m_s2: float = 1.0
    steering_rate_rad_s: float = math.radians(45.0)

class VehicleDynamicsKinematicAdapter:
    """Bridge external KinematicSingleTrack; optional dependency imported on init."""

    def __init__(self, vehicle: Vehicle = Vehicle(),
                 limits: KinematicAdapterLimits = KinematicAdapterLimits(),
                 external_model=None):
        if external_model is None:
            try:
                from vehicle_model.vehicle_models.kinematic_single_track import KinematicSingleTrack
            except ImportError as exc:
                raise ImportError(
                    "Install CagriCatik/Vehicle-Dynamics separately to enable this adapter"
                ) from exc
            external_model = KinematicSingleTrack()
        if abs(external_model.params.wheelbase - vehicle.wheelbase) > 1e-4:
            raise ValueError("External wheelbase differs from ADAS vehicle; calibrate both")
        if abs(external_model.params.lr - vehicle.rear_axle_from_center) > 1e-4:
            raise ValueError("External CG-to-rear offset differs from ADAS vehicle")
        self.vehicle = vehicle
        self.limits = limits
        self.model = external_model
        self._state = None

    def reset(self, pose: Pose) -> None:
        import numpy as np
        d = self.vehicle.rear_axle_from_center
        self._state = np.array([pose.x + d * math.cos(pose.yaw),
                                pose.y + d * math.sin(pose.yaw), pose.yaw, 0., 0.],
                               dtype=float)

    def step(self, command: Command, dt: float) -> Pose:
        import numpy as np
        if self._state is None:
            raise RuntimeError("Call reset before step")
        if dt <= 0 or abs(command.steer) > self.vehicle.max_steer + 1e-9:
            raise ValueError("invalid timestep or steering")
        speed = float(self._state[3])
        steer = float(self._state[4])
        accel = max(-self.limits.accel_m_s2,
                    min(self.limits.accel_m_s2, (command.velocity-speed)/dt))
        rate = max(-self.limits.steering_rate_rad_s,
                   min(self.limits.steering_rate_rad_s, (command.steer-steer)/dt))
        self._state = self.model.odeintRK4(self._state, [0., dt], np.array([accel,rate]))
        yaw = float(self._state[2])
        d = self.vehicle.rear_axle_from_center
        return Pose(float(self._state[0])-d*math.cos(yaw),
                    float(self._state[1])-d*math.sin(yaw), yaw)
