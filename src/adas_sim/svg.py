"""Uniform SVG scene plot at exactly the parking GIF's world scale."""
from __future__ import annotations
from pathlib import Path
from html import escape
from .geometry import obstacle_polygon,vehicle_polygon
from .visualization.common import CAMERA,Camera,PALETTE

def render(s,p,out):
    spec=CAMERA["parking"];cam=Camera(spec)
    obstacles=[obstacle_polygon(o) for o in s.obstacles]
    body_start=vehicle_polygon(s.start,s.vehicle)
    body_goal=vehicle_polygon(s.goal,s.vehicle)
    polys=obstacles+[body_start,body_goal]
    points=[point for poly in polys for point in poly]
    points += [point for pose in p.poses for point in vehicle_polygon(pose,s.vehicle)]
    cam.assert_visible(points)
    def poly(points,fill,outline):
        coords=" ".join(f"{x},{y}" for x,y in map(cam.xy,points))
        return f'<polygon points="{coords}" fill="{fill}" stroke="{outline}" stroke-width="2"/>'
    rows=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{spec.width}" height="{spec.height}" viewBox="0 0 {spec.width} {spec.height}">',
          f'<rect width="{spec.width}" height="{spec.height}" fill="{PALETTE["background"]}"/>',
          f'<rect width="{spec.width}" height="{spec.header_height}" fill="{PALETTE["header"]}"/>',
          f'<text x="24" y="34" fill="#fff" font-size="20" font-family="sans-serif">ADAS | PARKING | {escape(s.name)}</text>',
          f'<text x="24" y="59" fill="#c7d2fe" font-size="14" font-family="sans-serif">Shared camera: {spec.pixels_per_meter:.0f} px/m | {escape(p.reason)}</text>']
    rows.extend(poly(obs,PALETTE["obstacle"],PALETTE["obstacle_edge"]) for obs in obstacles)
    rows.append(poly(body_start,PALETTE["start"],PALETTE["2WS"]))
    rows.append(poly(body_goal,PALETTE["goal"],PALETTE["goal_edge"]))
    if len(p.poses)>1:
        coords=" ".join(f"{x},{y}" for x,y in (cam.xy((pose.x,pose.y)) for pose in p.poses))
        rows.append(f'<polyline points="{coords}" fill="none" stroke="{PALETTE["executed"]}" stroke-width="4"/>')
    rows.append(f'<rect y="{spec.height-spec.footer_height}" width="{spec.width}" height="{spec.footer_height}" fill="#e2e8f0"/>')
    rows.append(f'<text x="24" y="{spec.height-22}" font-size="15" fill="{PALETTE["text"]}" font-family="sans-serif">Planar 2D | synthetic simulation | fixed-scale scenario family</text>')
    rows.append("</svg>")
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text("\n".join(rows)+"\n",encoding="utf-8")
