"""Deterministic synthetic ultrasonic ranging and noisy localization."""
from dataclasses import dataclass
from math import sin,cos,pi,inf,radians
from random import Random
from .geometry import Pose,obstacle_polygon,wrap_angle

@dataclass(frozen=True)
class RangeMeasurement:
    bearing_rad:float
    range_m:float|None
    valid:bool

def _intersect(o,d,a,b):
    sx,sy=b[0]-a[0],b[1]-a[1]
    dx,dy=d
    den=dx*sy-dy*sx
    if abs(den)<1e-10:return None
    qx,qy=a[0]-o[0],a[1]-o[1]
    t=(qx*sy-qy*sx)/den
    u=(qx*dy-qy*dx)/den
    return t if t>=0 and 0<=u<=1 else None

def ultrasound(pose,obstacles,bearings=(-pi/2,-pi/4,0,pi/4,pi/2),max_range_m=5.,noise_std_m=.02,seed=0):
    rng=Random(seed);result=[]
    for bearing in bearings:
        heading=pose.yaw+bearing
        direction=cos(heading),sin(heading)
        hits=[]
        for obstacle in obstacles:
            corners=obstacle_polygon(obstacle)
            for i in range(4):
                d=_intersect((pose.x,pose.y),direction,corners[i],corners[(i+1)%4])
                if d is not None:hits.append(d)
        d=min(hits,default=inf)
        valid=d<=max_range_m
        result.append(RangeMeasurement(bearing,max(0.,d+rng.gauss(0,noise_std_m)) if valid else None,valid))
    return tuple(result)

def noisy_pose(pose,std_position_m=.04,std_yaw_rad=radians(.4),seed=0):
    rng=Random(seed)
    return Pose(pose.x+rng.gauss(0,std_position_m),pose.y+rng.gauss(0,std_position_m),wrap_angle(pose.yaw+rng.gauss(0,std_yaw_rad)))
