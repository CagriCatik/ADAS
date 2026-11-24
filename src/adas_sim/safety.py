"""Discrete preview emergency-stop guard; not a safety-certified controller."""
from dataclasses import dataclass
from math import isfinite,ceil
from .geometry import footprint_collision
from .plant import Command,step

@dataclass(frozen=True)
class SafetyDecision:
    allowed:bool
    reason:str
    requested:Command
    output:Command

def evaluate(pose,request,vehicle,obstacles,dt=.1,horizon_s=.8,min_margin_m=.15,localization_ok=True):
    if dt<=0 or horizon_s<=0:raise ValueError("invalid horizon")
    stop=Command(0.,0.)
    if not localization_ok:return SafetyDecision(False,"localization_unavailable",request,stop)
    if not (isfinite(request.steer) and isfinite(request.velocity)) or abs(request.steer)>vehicle.max_steer:
        return SafetyDecision(False,"invalid_command",request,stop)
    if footprint_collision(pose,vehicle,obstacles,margin=min_margin_m):
        return SafetyDecision(False,"already_in_clearance_zone",request,stop)
    step_time=min(dt,.05)
    for _ in range(ceil(horizon_s/step_time)):
        pose=step(pose,request,vehicle,step_time)
        if footprint_collision(pose,vehicle,obstacles,margin=min_margin_m):
            return SafetyDecision(False,"predicted_collision",request,stop)
    return SafetyDecision(True,"clear",request,request)
