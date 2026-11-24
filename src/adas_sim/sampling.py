"""Deterministic, Ackermann-constrained sampling planners.

RRT: nearest tree extension. RRT*: local parent selection and conservative
rewiring (only exactly replayable connections). FMT*: a lazy frontier search on
a sampled motion-primitive graph (FMT-inspired approximation, not canonical FMT*).
"""
from __future__ import annotations
from dataclasses import dataclass
import heapq
import math
import random
from .geometry import Pose, footprint_collision, wrap_angle
from .plant import Command
from .planner import Plan, Settings, _valid_motion, _within_goal, plan
from .scenarios import Scenario

@dataclass(frozen=True)
class SamplingSettings:
    iterations: int = 1200
    seed: int = 42
    goal_bias: float = 0.26
    max_vertices: int = 700

def dist(a: Pose, b: Pose) -> float:
    return math.hypot(a.x-b.x,a.y-b.y)+1.7*abs(wrap_angle(a.yaw-b.yaw))

def controls(s: Scenario, cfg: Settings):
    return tuple(Command(st,g*cfg.speed) for g in (1,-1)
                 for st in (-s.vehicle.max_steer,-s.vehicle.max_steer/2,0,
                            s.vehicle.max_steer/2,s.vehicle.max_steer))

def sample(s: Scenario,rng:random.Random,bias:float) -> Pose:
    if rng.random()<bias:return s.goal
    return Pose(rng.uniform(min(s.start.x,s.goal.x)-5,max(s.start.x,s.goal.x)+5),
                rng.uniform(min(s.start.y,s.goal.y)-6,max(s.start.y,s.goal.y)+6),
                rng.uniform(-math.pi,math.pi))

def prelim(s:Scenario,c:Settings):
    dt=c.primitive_length/c.primitive_steps/c.speed
    if footprint_collision(s.start,s.vehicle,s.obstacles):
        return Plan((s.start,),(),dt,0,False,"start_in_collision")
    if footprint_collision(s.goal,s.vehicle,s.obstacles):
        return Plan((s.start,),(),dt,0,False,"goal_in_collision")
    if _within_goal(s.start,s):
        return Plan((s.start,),(),0,0,True,"already_at_goal")
    return None

def build(nodes, idx, dt, expanded):
    segments=[]
    while idx>0:
        pose,parent,cmd,pts,cost=nodes[idx]
        segments.append((cmd,pts))
        idx=parent
    poses=[nodes[0][0]]; commands=[]
    for cmd,pts in reversed(segments):
        poses.extend(pts);commands.extend([cmd]*len(pts))
    return Plan(tuple(poses),tuple(commands),dt,expanded,True,"goal_reached")

def run_rrt(s:Scenario,c:Settings=Settings(), settings:SamplingSettings=SamplingSettings(),star:bool=False)->Plan:
    if c.speed<=0 or c.primitive_steps<1 or settings.iterations<1:
        raise ValueError("invalid settings")
    prev=prelim(s,c)
    if prev:return prev
    dt=c.primitive_length/c.primitive_steps/c.speed
    rng=random.Random(settings.seed); opts=controls(s,c)
    nodes=[(s.start,None,None,(),0.0)]
    for i in range(settings.iterations):
        target=sample(s,rng,settings.goal_bias)
        order=sorted(range(len(nodes)),key=lambda j:dist(nodes[j][0],target))
        near=order[0]
        def extensions(j,goal):
            out=[]
            for cmd in opts:
                pts=_valid_motion(nodes[j][0],cmd,dt,c.primitive_steps,s)
                if pts:out.append((dist(pts[-1],goal),cmd,pts))
            return sorted(out,key=lambda a:a[0])
        edges=extensions(near,target)
        if not edges:continue
        _,cmd,pts=edges[0];endpoint=pts[-1]
        if any(dist(nodes[j][0],endpoint)<0.16 for j in order[:18]):
            continue
        cost=nodes[near][4]+c.primitive_length
        if star:
            for j in order[1:6]:
                if dist(nodes[j][0],endpoint)>2.4:continue
                for _,alternate,apts in extensions(j,endpoint):
                    # Never connect approximate endpoints: replay must be exact.
                    if dist(apts[-1],endpoint)>1e-6:break
                    ac=nodes[j][4]+c.primitive_length
                    if ac<cost:near,cmd,pts,cost=j,alternate,apts,ac
                    break
        idx=len(nodes);nodes.append((endpoint,near,cmd,pts,cost))
        if _within_goal(endpoint,s):
            return build(nodes,idx,dt,i+1)
    return Plan((s.start,),(),dt,settings.iterations,False,"search_limit")

def run_fmt(s:Scenario,c:Settings=Settings(), settings:SamplingSettings=SamplingSettings())->Plan:
    """Seeded reachable-state sampling with cost-ordered, lazy directed expansion."""
    if c.speed<=0 or c.primitive_steps<1 or settings.max_vertices<2:
        raise ValueError("invalid settings")
    prev=prelim(s,c)
    if prev:return prev
    dt=c.primitive_length/c.primitive_steps/c.speed
    rng=random.Random(settings.seed);opts=controls(s,c)
    samples=[s.start];seen={(0,0,0)}
    for attempt in range(settings.max_vertices*25):
        if len(samples)>=settings.max_vertices:break
        origin=samples[rng.randrange(len(samples))]
        cmd=opts[rng.randrange(len(opts))]
        pts=_valid_motion(origin,cmd,dt,c.primitive_steps,s)
        if not pts:continue
        p=pts[-1]
        if math.hypot(p.x-s.start.x,p.y-s.start.y)>17:continue
        key=(round(p.x/.28),round(p.y/.28),round(wrap_angle(p.yaw)/.12))
        if key not in seen:
            seen.add(key);samples.append(p)
    unseen=set(range(1,len(samples)))
    nodes=[(s.start,None,None,(),0.0)];heap=[(0.0,0)]
    expanded=0
    while heap and expanded<settings.iterations:
        _,i=heapq.heappop(heap);expanded+=1
        pose=nodes[i][0]
        for cmd in opts:
            pts=_valid_motion(pose,cmd,dt,c.primitive_steps,s)
            if not pts:continue
            end=pts[-1]
            cost=nodes[i][4]+c.primitive_length
            if _within_goal(end,s):
                nodes.append((end,i,cmd,pts,cost))
                return build(nodes,len(nodes)-1,dt,expanded)
            near=min(((dist(end,samples[j]),j) for j in unseen),default=None)
            if near is None or near[0]>.30:continue
            unseen.remove(near[1])
            nodes.append((end,i,cmd,pts,cost))
            heapq.heappush(heap,(cost,len(nodes)-1))
    return Plan((s.start,),(),dt,expanded,False,"search_limit" if expanded>=settings.iterations else "no_path")

def choose(s:Scenario,method:str,c:Settings=Settings(),settings:SamplingSettings=SamplingSettings())->Plan:
    if method=="hybrid_astar":return plan(s,c)
    if method=="rrt":return run_rrt(s,c,settings)
    if method=="rrt_star":return run_rrt(s,c,settings,True)
    if method=="fmt_star":return run_fmt(s,c,settings)
    raise ValueError(f"unknown planner {method}")
