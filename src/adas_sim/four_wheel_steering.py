"""Low-speed rear-axle steering aligned with Vehicle-Dynamics sign conventions."""
from dataclasses import dataclass
import math
from .geometry import Pose,Vehicle
from .plant import Command

@dataclass(frozen=True)
class RearSteerConfig:
    max_rear_rad:float=math.radians(10)
    low_speed_ratio:float=-.30
    front_rate_rad_s:float=math.radians(45)
    rear_rate_rad_s:float=math.radians(35)

    def rear_target(self,front_rad,enabled=True):
        if not enabled:return 0.
        return max(-self.max_rear_rad,min(self.max_rear_rad,self.low_speed_ratio*front_rad))

def step_4ws(p:Pose,front_rad:float,rear_rad:float,signed_speed:float,vehicle:Vehicle,dt:float)->Pose:
    """Planar rear-axle-midpoint model, including actual rear-wheel steering."""
    if dt<=0 or not all(map(math.isfinite,(p.x,p.y,p.yaw,front_rad,rear_rad,signed_speed))):
        raise ValueError("invalid state/input/dt")
    if abs(front_rad)>vehicle.max_steer+1e-9 or abs(rear_rad)>=math.pi/2:
        raise ValueError("invalid steering angle")
    yaw_rate=signed_speed*math.cos(rear_rad)*(math.tan(front_rad)-math.tan(rear_rad))/vehicle.wheelbase
    heading=p.yaw+yaw_rate*dt/2+rear_rad
    return Pose(p.x+signed_speed*dt*math.cos(heading),
                p.y+signed_speed*dt*math.sin(heading),p.yaw+yaw_rate*dt)

def _towards(value,target,limit):
    return max(value-limit,min(value+limit,target))

def follow_commands(initial:Pose,commands:tuple[Command,...],dt:float,vehicle:Vehicle,
                    four_wheel:bool,cfg:RearSteerConfig=RearSteerConfig()):
    """Paired open-loop 2WS/4WS experiment with matched front inputs."""
    pose=initial;front=rear=0.
    poses=[pose];fronts=[front];rears=[rear]
    for cmd in commands:
        if abs(cmd.steer)>vehicle.max_steer+1e-9:raise ValueError("front steering limit")
        front=_towards(front,cmd.steer,cfg.front_rate_rad_s*dt)
        rear=_towards(rear,cfg.rear_target(front,four_wheel),cfg.rear_rate_rad_s*dt)
        pose=step_4ws(pose,front,rear,cmd.velocity,vehicle,dt)
        poses.append(pose);fronts.append(front);rears.append(rear)
    return tuple(poses),tuple(fronts),tuple(rears)
