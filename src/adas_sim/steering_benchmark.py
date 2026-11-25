"""Equal-input 2WS / counter-phase 4WS low-speed research benchmark."""
from __future__ import annotations
from pathlib import Path
import csv,json,math
from .geometry import Pose,Vehicle
from .plant import Command
from .four_wheel_steering import follow_commands
from .planar_vehicle import planar_parts

def maneuvers():
    a=math.radians(30);dt=.05
    def steps(*sections):
        return tuple(Command(steer,speed) for steer,speed,seconds in sections
                     for _ in range(round(seconds/dt)))
    return {
        "left_u_turn":steps((a,1.,8.)),
        "right_u_turn":steps((-a,1.,8.)),
        "reverse_arc":steps((a,-.8,7.)),
        "parking_s_bend":steps((a,.7,4.),(-a,-.6,4.),(0.,.5,2.)),
    }

def compare(commands,vehicle=Vehicle(),dt=.05):
    result={}
    for mode,enable in (("2WS",False),("4WS",True)):
        poses,fronts,rears=follow_commands(Pose(0,0,0),commands,dt,vehicle,enable)
        samples=[p for pose,f,r in zip(poses,fronts,rears)
                   for poly in planar_parts(pose,vehicle,f,r) for p in poly]
        xs=[x for x,y in samples];ys=[y for x,y in samples]
        distance=sum(math.hypot(q.x-p.x,q.y-p.y) for p,q in zip(poses,poses[1:]))
        heading=poses[-1].yaw-poses[0].yaw
        metrics={"architecture":mode,"path_length_m":round(distance,3),
                 "heading_change_deg":round(math.degrees(heading),3),
                 "effective_radius_m":round(abs(distance/heading),3) if abs(heading)>1e-9 else None,
                 "swept_width_m":round(max(ys)-min(ys),3),
                 "swept_length_m":round(max(xs)-min(xs),3),
                 "front_steer_max_deg":round(max(map(abs,fronts))*180/math.pi,3),
                 "rear_steer_max_deg":round(max(map(abs,rears))*180/math.pi,3)}
        result[mode]=(poses,fronts,rears,metrics)
    return result

def benchmark(out=Path("docs/assets/steering")):
    from .steering_animation import render_comparison
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    rows=[]
    for name,commands in maneuvers().items():
        pair=compare(commands)
        rows.extend({"maneuver":name,**pair[mode][3]} for mode in ("2WS","4WS"))
        render_comparison(name,pair,out/f"{name}_2ws_vs_4ws.gif",max_frames=18)
        print(name,pair["2WS"][3]["effective_radius_m"],pair["4WS"][3]["effective_radius_m"],flush=True)
    (out/"results.json").write_text(json.dumps(rows,indent=2)+"\n")
    with (out/"results.csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    return rows

if __name__=="__main__":
    benchmark()
