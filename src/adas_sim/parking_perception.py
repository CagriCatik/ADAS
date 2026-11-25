"""Conservative parallel-bay detection from sequential ultrasonic RANGE observations.

Only range, channel ID/status and transducer pose are consumed; simulator
ground-truth hit coordinates are intentionally ignored. A straight survey
estimates bounded gaps from the side-looking front/rear transducer transitions.
Requires roughly aligned parked vehicles and known nominal car dimensions.
"""
from __future__ import annotations
from dataclasses import dataclass
from statistics import median
from math import sin
from .geometry import Pose,Rectangle,Vehicle
from .ultrasound_12 import Echo

@dataclass(frozen=True)
class SurveyFrame:
    pose: Pose
    echoes: tuple[Echo,...]

@dataclass(frozen=True)
class ParkingSlot:
    side: str
    left_boundary_x: float
    right_boundary_x: float
    certified_length_m: float
    measured_face_y: float
    goal: Pose
    perceived_obstacles: tuple[Rectangle,...]
    supporting_echoes: int
    free_space_checks: int

@dataclass(frozen=True)
class Perception:
    status:str
    reason:str
    slot:ParkingSlot|None

@dataclass(frozen=True)
class GapSettings:
    min_clearance_m:float=.6
    boundary_uncertainty_m:float=.5
    min_occupied_samples:int=6
    min_free_space_checks:int=5
    assumed_parked_width_m:float=1.8
    assumed_parked_length_m:float=4.5
    max_intermittent_samples:int=1

def _runs(frames,channel,side_sign,settings):
    """Contiguous range detections in the survey's travel direction."""
    result=[];active=[];misses=0
    for i,frame in enumerate(frames):
        e=next((item for item in frame.echoes if item.id==channel),None)
        valid=False
        if e and e.status=="DETECTED" and e.range_m is not None:
            lateral=side_sign*(e.origin_y+e.range_m*sin(e.heading_rad)-frame.pose.y)
            valid=1.0<=lateral<=4.1
        if valid:
            active.append((i,e));misses=0
        elif active:
            misses+=1
            if misses>settings.max_intermittent_samples:
                if len(active)>=settings.min_occupied_samples:result.append(active)
                active=[];misses=0
    if len(active)>=settings.min_occupied_samples:result.append(active)
    return result

def detect_parallel_gap(frames:tuple[SurveyFrame,...],side:str,
                        vehicle:Vehicle=Vehicle(),settings:GapSettings=GapSettings())->Perception:
    if side not in ("left","right"):raise ValueError("side must be left or right")
    if len(frames)<10 or any(len(f.echoes)!=12 for f in frames):
        return Perception("NO_SLOT","insufficient_scan_coverage",None)
    if settings.boundary_uncertainty_m<0 or settings.min_clearance_m<0:
        raise ValueError("invalid safety margins")
    if any(abs(f.pose.yaw)>.12 or abs(f.pose.y-frames[0].pose.y)>.3 for f in frames):
        return Perception("NO_SLOT","unsupported_survey_heading",None)
    if any(b.pose.x<=a.pose.x for a,b in zip(frames,frames[1:])):
        return Perception("NO_SLOT","survey_not_monotonic",None)
    invalid=sum(e.status in ("DROPOUT","INTERFERENCE") for f in frames for e in f.echoes)
    if invalid>len(frames)*12*.20:
        return Perception("NO_SLOT","unreliable_acoustic_measurements",None)
    sign=1 if side=="left" else -1
    front="SLF" if sign>0 else "SRF"
    rear="SLR" if sign>0 else "SRR"
    front_runs=_runs(frames,front,sign,settings)
    rear_runs=_runs(frames,rear,sign,settings)
    if len(front_runs)<2 or len(rear_runs)<2:
        return Perception("NO_SLOT","unbounded_or_unobserved_gap",None)
    short=False;unverified=False
    for i in range(min(len(front_runs),len(rear_runs))-1):
        # Use side-facing front transducer transitions for both gap boundaries;
        # the rear side transducer must independently confirm both vehicles.
        left=front_runs[i][-1][1].origin_x
        right=front_runs[i+1][0][1].origin_x
        rear_left=rear_runs[i][-1][1].origin_x
        rear_right=rear_runs[i+1][0][1].origin_x
        if abs(left-rear_left)>1.0 or abs(right-rear_right)>1.0:
            continue
        certified=right-left-2*settings.boundary_uncertainty_m
        if certified<vehicle.length+settings.min_clearance_m:
            short=True;continue
        contacts=[e for _,e in front_runs[i]+rear_runs[i]+front_runs[i+1]+rear_runs[i+1]]
        face=median(e.origin_y+e.range_m*sin(e.heading_rad) for e in contacts if e.range_m is not None)
        inner_left=left+settings.boundary_uncertainty_m+1.
        inner_right=right-settings.boundary_uncertainty_m-1.
        clear=set()
        for frame in frames:
            for e in frame.echoes:
                if e.id not in (front,rear) or e.status!="NO_ECHO":continue
                if sign*sin(e.heading_rad)<.85:continue
                if e.max_range_m*sign*sin(e.heading_rad)<(sign*(face-frame.pose.y)+vehicle.width+settings.min_clearance_m):
                    continue
                if inner_left<=e.origin_x<=inner_right:
                    clear.add((round(frame.pose.x,2),e.id))
        if len(clear)<settings.min_free_space_checks:
            unverified=True;continue
        cx=(left+right)/2
        cy=face+sign*settings.assumed_parked_width_m/2
        goal=Pose(cx-(vehicle.length/2-vehicle.rear_axle_from_center),cy,0.)
        # Build conservative occupied spans from the complete observed runs.
        # Never guess a fixed 4.5m car extent from just one boundary.
        uncertainty=settings.boundary_uncertainty_m
        occupied=((front_runs[i][0][1].origin_x-uncertainty,
                   front_runs[i][-1][1].origin_x+uncertainty),
                  (front_runs[i+1][0][1].origin_x-uncertainty,
                   front_runs[i+1][-1][1].origin_x+uncertainty))
        reconstructed=tuple(Rectangle((lo+hi)/2,cy,hi-lo,
                                      settings.assumed_parked_width_m)
                            for lo,hi in occupied)
        slot=ParkingSlot(side,left,right,certified,face,goal,reconstructed,
                         sum(len(x) for x in (front_runs[i],rear_runs[i],front_runs[i+1],rear_runs[i+1])),
                         len(clear))
        return Perception("SLOT_FOUND","side_ultrasound_bounded_gap",slot)
    return Perception("NO_SLOT",
                      "slot_too_short" if short else "insufficient_free_space_evidence" if unverified else "slot_not_verified",None)
