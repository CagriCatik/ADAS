"""Abstract boundary to reuse Vehicle-Dynamics physics after calibration."""
from typing import Protocol
from .geometry import Pose
from .plant import Command

class ParkingPlant(Protocol):
    def reset(self,pose:Pose)->None: ...
    def step(self,command:Command,dt:float)->Pose: ...

class VehicleDynamicsAdapter:
    def __init__(self):
        raise NotImplementedError("7-DOF mapping, zero-speed behaviour and actuator interfaces not yet validated")
