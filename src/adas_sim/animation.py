"""Uniform parking GIF renderer with one camera and shared visual grammar."""
from __future__ import annotations
import math
from pathlib import Path
from .geometry import vehicle_polygon, obstacle_polygon
from .planar_vehicle import planar_parts
from .visualization.common import CAMERA, Camera, PALETTE, image_indices, decorate_pillow

def render_gif(scene, route, destination, fps=None, speedup=None, max_frames=None):
    from PIL import Image, ImageDraw
    from dataclasses import replace
    spec=CAMERA["parking"]
    if fps is not None or speedup is not None or max_frames is not None:
        spec=replace(spec, fps=fps if fps is not None else spec.fps,
                     speedup=speedup if speedup is not None else spec.speedup,
                     max_frames=max_frames if max_frames is not None else spec.max_frames)
    cam=Camera(spec)
    obstacles=[obstacle_polygon(o) for o in scene.obstacles]
    all_points=[pt for poly in obstacles for pt in poly]
    all_points.extend(pt for pose in (scene.start,scene.goal) for pt in vehicle_polygon(pose,scene.vehicle))
    all_points.extend(pt for pose in route.poses for pt in vehicle_polygon(pose,scene.vehicle))
    cam.assert_visible(all_points)
    indices=image_indices(len(route.poses),route.step_dt,spec)
    frames=[]
    full_path=[cam.xy((pose.x,pose.y)) for pose in route.poses]
    for frame_no,idx in enumerate(indices):
        im=Image.new("RGB",(spec.width,spec.height),PALETTE["background"])
        draw=ImageDraw.Draw(im)
        for poly in obstacles:
            draw.polygon([cam.xy(p) for p in poly],fill=PALETTE["obstacle"],outline=PALETTE["obstacle_edge"],width=3)
        draw.polygon([cam.xy(p) for p in vehicle_polygon(scene.start,scene.vehicle)],
                     fill=PALETTE["start"],outline=PALETTE["2WS"],width=2)
        draw.polygon([cam.xy(p) for p in vehicle_polygon(scene.goal,scene.vehicle)],
                     fill=PALETTE["goal"],outline=PALETTE["goal_edge"],width=3)
        if len(full_path)>1:draw.line(full_path,fill=PALETTE["planned"],width=3)
        if idx>0:draw.line(full_path[:idx+1],fill=PALETTE["executed"],width=4)
        pose=route.poses[idx]
        command=route.commands[min(idx,len(route.commands)-1)] if route.commands else None
        front=command.steer if command else 0.
        rear=0.
        body,*wheels=planar_parts(pose,scene.vehicle,front,rear)
        draw.polygon([cam.xy(p) for p in body],fill="#d7e7fa",outline=PALETTE["2WS"],width=3)
        for poly in wheels:
            draw.polygon([cam.xy(p) for p in poly],fill=PALETTE["wheel"],outline="#ffffff",width=2)
        draw.ellipse((cam.xy((pose.x,pose.y))[0]-4,cam.xy((pose.x,pose.y))[1]-4,
                      cam.xy((pose.x,pose.y))[0]+4,cam.xy((pose.x,pose.y))[1]+4),fill=PALETTE["reference"])
        elapsed=idx*route.step_dt
        decorate_pillow(draw,spec,f"ADAS | PARKING | {scene.name}",
                        f"Shared camera  {spec.pixels_per_meter:.0f} px/m  |  {'PATH' if route.success else 'FAILED'}: {route.reason}",
                        f"t={elapsed:5.2f} s   speed={command.velocity if command else 0.:+4.1f} m/s   "
                        f"front={math.degrees(front):+5.1f} deg   rear={math.degrees(rear):+5.1f} deg   "
                        f"frame={frame_no+1}/{len(indices)}")
        frames.append(im)
    path=Path(destination);path.parent.mkdir(parents=True,exist_ok=True)
    frames[0].save(path,save_all=True,append_images=frames[1:],
                   duration=round(1000/spec.fps),loop=0,optimize=True)
    return path
