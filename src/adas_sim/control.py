"""Low-speed actuator-constrained parking execution. Feedforward baseline, not MPC."""
from dataclasses import dataclass
import math
from .geometry import footprint_collision,wrap_angle
from .plant import Command,step

@dataclass(frozen=True)
class ActuatorLimits:
    steering_rate_rad_s:float=math.radians(65)
    acceleration_m_s2:float=1.
    deceleration_m_s2:float=1.8
    gear_dwell_s:float=.35
    max_speed_m_s:float=1.

@dataclass(frozen=True)
class Execution:
    poses:tuple
    commands:tuple
    dt:float
    collision:bool
    status:str
    position_error_m:float
    yaw_error_deg:float

def approach(value,target,step_size):
    return max(value-step_size,min(value+step_size,target))

def execute(scene,route,limits=ActuatorLimits()):
    """Replay with steering slew, acceleration and gear dwell; report failures."""
    if limits.max_speed_m_s<=0 or min(limits.steering_rate_rad_s,limits.acceleration_m_s2,limits.deceleration_m_s2)<=0:
        raise ValueError("invalid actuator limits")
    if not route.success or not route.commands:
        return Execution((scene.start,),(),route.step_dt,False,"no_plan",0.,0.)
    count=max(1,math.ceil(route.step_dt/.05))
    dt=route.step_dt/count
    pose=scene.start;steer=speed=dwell=0.;gear=0;collision=False
    poses=[pose];controls=[]
    for target_command in route.commands:
        target_gear=1 if target_command.velocity>0 else -1
        if target_gear!=gear:
            if gear!=0 and abs(speed)>.015:desired_speed=0.
            else:
                gear=target_gear;dwell=limits.gear_dwell_s;desired_speed=0.
        else:
            desired_speed=max(-limits.max_speed_m_s,min(limits.max_speed_m_s,target_command.velocity))
        for _ in range(count):
            steer=approach(steer,target_command.steer,limits.steering_rate_rad_s*dt)
            target_speed=0. if dwell>0 else desired_speed
            acceleration=(limits.acceleration_m_s2 if abs(target_speed)>abs(speed) else limits.deceleration_m_s2)*dt
            speed=approach(speed,target_speed,acceleration)
            if speed*target_speed<0:speed=0.
            command=Command(steer,speed)
            pose=step(pose,command,scene.vehicle,dt)
            collision=collision or footprint_collision(pose,scene.vehicle,scene.obstacles)
            poses.append(pose);controls.append(command)
            dwell=max(0.,dwell-dt)
            if collision:break
        if collision:break
    err=math.hypot(pose.x-scene.goal.x,pose.y-scene.goal.y)
    yaw=abs(wrap_angle(pose.yaw-scene.goal.yaw))
    ok=not collision and err<=scene.position_tolerance and yaw<=scene.yaw_tolerance
    return Execution(tuple(poses),tuple(controls),dt,collision,"PASS" if ok else "COLLISION" if collision else "TRACKING_ERROR",err,math.degrees(yaw))
