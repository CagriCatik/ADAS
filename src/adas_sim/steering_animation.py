"""Unified side-by-side planar steering animations, common camera across maneuvers."""
from __future__ import annotations
import math
from pathlib import Path
from .geometry import Vehicle
from .planar_vehicle import planar_parts
from .visualization.common import CAMERA, Camera, PALETTE, image_indices, decorate_pillow

def render_comparison(name,pair,path,fps=None,max_frames=None,dt=.05):
    from PIL import Image,ImageDraw
    from dataclasses import replace
    spec=CAMERA["steering"]
    if fps is not None or max_frames is not None:
        spec=replace(spec,fps=spec.fps if fps is None else fps,
                     max_frames=spec.max_frames if max_frames is None else max_frames)
    v=Vehicle()
    cams={"2WS":Camera(spec,0,2),"4WS":Camera(spec,1,2)}
    for mode in ("2WS","4WS"):
        poses,fronts,rears,_=pair[mode]
        points=(pt for pose,front,rear in zip(poses,fronts,rears)
                for poly in planar_parts(pose,v,front,rear) for pt in poly)
        cams[mode].assert_visible(points)
    indices=image_indices(len(pair["2WS"][0]),dt,spec)
    frames=[]
    for frame_no,k in enumerate(indices):
        im=Image.new("RGB",(spec.width,spec.height),PALETTE["background"])
        d=ImageDraw.Draw(im)
        d.line((spec.width//2,spec.header_height,spec.width//2,spec.height-spec.footer_height),
               fill="#94a3b8",width=2)
        for mode in ("2WS","4WS"):
            cam=cams[mode]
            poses,fronts,rears,metrics=pair[mode]
            path_points=[cam.xy((p.x,p.y)) for p in poses]
            d.line(path_points,fill=PALETTE["planned"],width=3)
            if k>0:d.line(path_points[:k+1],fill=PALETTE[mode],width=4)
            body,*wheels=planar_parts(poses[k],v,fronts[k],rears[k])
            d.polygon([cam.xy(p) for p in body],fill="#d7e7fa",outline=PALETTE[mode],width=4)
            for poly in wheels:
                d.polygon([cam.xy(p) for p in poly],fill=PALETTE["wheel"],outline="#ffffff",width=2)
            radius=metrics["effective_radius_m"]
            d.text((int(cam.panel_index*cam.panel_width)+24,spec.header_height+19),
                   f'{mode} | R_eff = {radius:.2f} m' if radius is not None else f'{mode} | R_eff = inf',
                   fill=PALETTE["text"])
            x,y=cam.xy((poses[k].x,poses[k].y))
            d.ellipse((x-4,y-4,x+4,y+4),fill=PALETTE["reference"])
            d.text((int(cam.panel_index*cam.panel_width)+24,spec.height-spec.footer_height-26),
                   f'Front {math.degrees(fronts[k]):+5.1f} deg | Rear {math.degrees(rears[k]):+5.1f} deg',
                   fill=PALETTE["text"])
        decorate_pillow(d,spec,f"ADAS | 2WS / 4WS | {name}",
                        f"Shared camera: {spec.pixels_per_meter:.0f} px/m  |  Identical front command and velocity",
                        f"t={k*dt:5.2f} s   frame={frame_no+1}/{len(indices)}   "
                        f"synthetic kinematic comparison")
        frames.append(im)
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    frames[0].save(path,save_all=True,append_images=frames[1:],
                   duration=round(1000/spec.fps),loop=0,optimize=True)
    return path
