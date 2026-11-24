"""Animated, metric-consistent 2D vehicle panels with real front/rear tire steering."""
import math
from pathlib import Path
from .geometry import Vehicle
from .planar_vehicle import planar_parts

def render_comparison(name,pair,path,fps=12,max_frames=85):
    from PIL import Image,ImageDraw
    v=Vehicle();width,height=1380,790;panel=width//2
    polys=[pt for poses,fronts,rears,_ in pair.values()
           for p,f,r in zip(poses,fronts,rears)
           for poly in planar_parts(p,v,f,r) for pt in poly]
    xmin=min(p[0] for p in polys)-2;xmax=max(p[0] for p in polys)+2
    ymin=min(p[1] for p in polys)-2;ymax=max(p[1] for p in polys)+2
    scale=min((panel-80)/(xmax-xmin),(height-165)/(ymax-ymin))
    def xy(p,offset):return int(offset+40+(p[0]-xmin)*scale),int(height-67-(p[1]-ymin)*scale)
    total=len(pair["2WS"][0]);step=max(1,math.ceil(total/max_frames))
    indices=list(range(0,total,step))
    if indices[-1]!=total-1:indices.append(total-1)
    frames=[]
    for k in indices:
        im=Image.new("RGB",(width,height),"#edf3f9");d=ImageDraw.Draw(im)
        d.rectangle((0,0,width,80),fill="#102038")
        d.text((24,16),f"ADAS | 2D PLANAR | {name}",fill="#ffffff")
        d.text((24,42),"Same front steering and speed. Four separate steered tire polygons.",fill="#c7d2fe")
        d.line((panel,80,panel,height),fill="#94a3b8",width=2)
        for index,(mode,color) in enumerate((("2WS","#0d68ca"),("4WS","#c76216"))):
            poses,fronts,rears,metrics=pair[mode];offset=index*panel
            d.text((offset+24,99),f'{mode} | R_eff = {metrics["effective_radius_m"]:.2f} m',fill="#13233e")
            route=[xy((p.x,p.y),offset) for p in poses]
            d.line(route,fill="#c1cbd6",width=3)
            if k>0:d.line(route[:k+1],fill=color,width=4)
            parts=planar_parts(poses[k],v,fronts[k],rears[k])
            d.polygon([xy(p,offset) for p in parts[0]],fill="#d7e7fa",outline=color,width=4)
            for poly in parts[1:]:
                d.polygon([xy(p,offset) for p in poly],fill="#182e49",outline="white",width=2)
            pos=xy((poses[k].x,poses[k].y),offset)
            d.ellipse((pos[0]-4,pos[1]-4,pos[0]+4,pos[1]+4),fill="#e11d48")
            d.text((offset+22,height-43),
                   f"Front {math.degrees(fronts[k]):+5.1f} deg | Rear {math.degrees(rears[k]):+5.1f} deg",fill="#23334c")
        d.text((width-190,18),f"t = {k*.05:.2f} s",fill="white")
        frames.append(im)
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    frames[0].save(path,save_all=True,append_images=frames[1:],
                   duration=95,loop=0,optimize=True)
    return path
