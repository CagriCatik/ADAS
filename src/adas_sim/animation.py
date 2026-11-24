"""Render actual planned Ackermann vehicle motion as a 2D top-down GIF."""
from __future__ import annotations
import math
from pathlib import Path
from .geometry import vehicle_polygon, obstacle_polygon
from .planner import Plan
from .scenarios import Scenario

def render_gif(scene: Scenario, route: Plan, destination: str | Path,
               fps: int = 16, speedup: float = 2.0, max_frames: int = 300):
    from PIL import Image, ImageDraw
    if fps < 1 or speedup <= 0 or max_frames < 2:
        raise ValueError("invalid GIF settings")
    bounds=[(p.x,p.y) for p in route.poses]
    for o in scene.obstacles:bounds.extend(obstacle_polygon(o))
    for p in (scene.start,scene.goal):bounds.extend(vehicle_polygon(p,scene.vehicle))
    xmin=min(x for x,y in bounds)-3
    xmax=max(x for x,y in bounds)+3
    ymin=min(y for x,y in bounds)-3
    ymax=max(y for x,y in bounds)+3
    w,h=1000,700
    scale=min((w-80)/(xmax-xmin),(h-130)/(ymax-ymin))
    def xy(t):
        return int(40+(t[0]-xmin)*scale),int(h-50-(t[1]-ymin)*scale)
    def shape(p,margin=0):
        return [xy(q) for q in vehicle_polygon(p,scene.vehicle,margin)]
    base=Image.new("RGB",(w,h),"#eff5fa")
    draw=ImageDraw.Draw(base)
    draw.rectangle((0,0,w,63),fill="#14213d")
    for obstacle in scene.obstacles:
        draw.polygon([xy(q) for q in obstacle_polygon(obstacle)],fill="#94a3b8",outline="#334155",width=3)
    draw.polygon(shape(scene.start),fill="#c7d2fe",outline="#64748b",width=2)
    draw.polygon(shape(scene.goal),fill="#bbf7d0",outline="#15803d",width=3)
    trail=[xy((p.x,p.y)) for p in route.poses]
    if len(trail)>1:draw.line(trail,fill="#a0aec0",width=3)
    indices=[0]
    if len(route.poses)>1:
        stride=max(1,round(speedup/(fps*route.step_dt)))
        indices=list(range(0,len(route.poses),stride))
        if indices[-1]!=len(route.poses)-1:indices.append(len(route.poses)-1)
        if len(indices)>max_frames:
            stride=math.ceil((len(indices)-1)/(max_frames-1))
            indices=indices[::stride]
            if indices[-1]!=len(route.poses)-1:indices.append(len(route.poses)-1)
    frames=[]
    for frame_idx,index in enumerate(indices):
        image=base.copy()
        d=ImageDraw.Draw(image)
        if index>0:d.line(trail[:index+1],fill="#ea580c",width=4)
        pose=route.poses[index]
        d.polygon(shape(pose),fill="#2563eb",outline="#1e3a8a",width=3)
        center=(pose.x+(scene.vehicle.length/2-scene.vehicle.rear_axle_from_center)*math.cos(pose.yaw),
                pose.y+(scene.vehicle.length/2-scene.vehicle.rear_axle_from_center)*math.sin(pose.yaw))
        front=(center[0]+1.5*math.cos(pose.yaw),center[1]+1.5*math.sin(pose.yaw))
        d.line([xy(center),xy(front)],fill="white",width=5)
        for offset in (0,scene.vehicle.wheelbase):
            p=(pose.x+offset*math.cos(pose.yaw),pose.y+offset*math.sin(pose.yaw))
            u,v=xy(p)
            d.ellipse((u-4,v-4,u+4,v+4),fill="#fef08a")
        command=route.commands[min(index,len(route.commands)-1)] if route.commands else None
        steer=math.degrees(command.steer) if command else 0
        speed=command.velocity if command else 0
        d.text((20,22),f"ADAS | {scene.name} | {'PATH FOUND' if route.success else 'NO PATH'}",fill="white")
        d.text((20,h-34),f"time {index*route.step_dt:.1f}s | steer {steer:+.1f}deg | speed {speed:+.1f}m/s | frame {frame_idx+1}/{len(indices)}",fill="#1f2937")
        frames.append(image)
    path=Path(destination)
    path.parent.mkdir(parents=True,exist_ok=True)
    frames[0].save(path,save_all=True,append_images=frames[1:],
                   duration=max(30,round(1000/fps)),loop=0,optimize=True)
