"""Ultrasonic survey -> measured bay -> planner -> independent truth check.

World geometry is passed to the synthetic transducer simulation, but NEVER to
the park-gap detector or planner. Independent post-plan validation uses ground
truth and may veto deployment in this research simulator.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import ceil
from .geometry import Pose,Rectangle,Vehicle,footprint_collision
from .ultrasound_12 import scan_12
from .acoustic_schedule import AcousticSchedule,invalidate_interference
from .parking_perception import SurveyFrame,Perception,GapSettings,detect_parallel_gap
from .planner import plan,Plan
from .validation import validate
from .scenarios import Scenario

@dataclass(frozen=True)
class ParkingDecision:
    status:str
    reason:str
    perception:Perception
    survey:tuple[SurveyFrame,...]
    plan:Plan|None
    validation:object|None

def evaluate_parking(obstacles:tuple[Rectangle,...],*,side="left",
                     vehicle:Vehicle=Vehicle(),survey_start=-5.,survey_end=18.,
                     survey_step=.2,gap:GapSettings=GapSettings(),
                     transmit_mode="tdma",noise_std_m=0.,dropout_probability=0.)->ParkingDecision:
    if survey_step<=0 or survey_end<=survey_start:raise ValueError("invalid survey span")
    if transmit_mode not in ("tdma","simultaneous"):raise ValueError("invalid transmit mode")
    schedule=AcousticSchedule()
    frames=[]
    n=ceil((survey_end-survey_start)/survey_step)
    for i in range(n+1):
        x=min(survey_end,survey_start+i*survey_step)
        pose=Pose(x,0.,0.)
        if footprint_collision(pose,vehicle,obstacles):
            p=Perception("NO_SLOT","survey_path_blocked",None)
            return ParkingDecision("NO_PARK",p.reason,p,tuple(frames),None,None)
        raw=scan_12(pose,obstacles,vehicle=vehicle,noise_std_m=noise_std_m,
                    dropout_probability=dropout_probability,seed=i)
        ids=[e.id for e in raw]
        pulses=schedule.tdma(ids) if transmit_mode=="tdma" else schedule.simultaneous(ids)
        echoes=invalidate_interference(raw,pulses,schedule)
        frames.append(SurveyFrame(pose,echoes))
    survey=tuple(frames)
    perceived=detect_parallel_gap(survey,side,vehicle,gap)
    if perceived.slot is None:
        return ParkingDecision("NO_PARK",perceived.reason,perceived,survey,None,None)
    slot=perceived.slot
    # No oracle ground-truth obstacle or goal data enters the planner.
    scene=Scenario("sensor_derived","Measured parking target",
                   survey[-1].pose,slot.goal,slot.perceived_obstacles,vehicle)
    path=plan(scene)
    if not path.success:
        return ParkingDecision("NO_PARK","planner_"+path.reason,perceived,survey,path,None)
    truth=Scenario("independent_validation","Independent simulator truth",
                   scene.start,scene.goal,obstacles,vehicle)
    report=validate(truth,path)
    if report.result!="PASS":
        return ParkingDecision("NO_PARK","independent_collision_or_goal_failure",
                               perceived,survey,path,report)
    return ParkingDecision("PARK_ALLOWED","sensor_verified_slot_and_path",
                           perceived,survey,path,report)
