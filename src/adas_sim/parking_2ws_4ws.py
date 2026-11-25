"""Same-goal 2WS versus 4WS parking planning and replay benchmark.

Both configurations have identical vehicle footprint, obstacles, start and
goal pose. 4WS receives its own search, not the 2WS controls. This is a
geometric path-planning comparison, not yet rate-limited closed-loop parking.
"""
from __future__ import annotations
from dataclasses import asdict
from pathlib import Path
import csv
import json
import math
import time

from .geometry import footprint_collision, vehicle_polygon, obstacle_polygon
from .planner import plan
from .planning_4ws import plan_4ws, replay_4ws
from .scenarios import SCENARIOS
from .validation import validate


def summarize(scene, mode, route, replay_ok, planning_time):
    distance = sum(math.hypot(b.x-a.x,b.y-a.y)
                   for a,b in zip(route.poses,route.poses[1:]))
    changes = sum((a.velocity>0)!=(b.velocity>0)
                  for a,b in zip(route.commands,route.commands[1:]))
    end = route.poses[-1]
    position_error = math.hypot(end.x-scene.goal.x,end.y-scene.goal.y)
    heading_error = abs(math.atan2(math.sin(end.yaw-scene.goal.yaw),
                                  math.cos(end.yaw-scene.goal.yaw)))
    accepted = bool(route.success and replay_ok and
                    not any(footprint_collision(p,scene.vehicle,scene.obstacles)
                            for p in route.poses) and
                    position_error<=scene.position_tolerance and
                    heading_error<=scene.yaw_tolerance)
    return {
        "scenario":scene.name,"mode":mode,"success":accepted,
        "reason":route.reason if route.success else route.reason,
        "expanded":route.expanded,"planning_time_s":round(planning_time,5),
        "path_length_m":round(distance,4),"gear_changes":changes,
        "simulated_duration_s":round(len(route.commands)*route.step_dt,3),
        "position_error_m":round(position_error,4),
        "heading_error_deg":round(math.degrees(heading_error),3),
        "replay_valid":bool(replay_ok),
    }


def compare_scene(scene):
    outputs={}
    for mode,planner in (("2WS",plan),("4WS",plan_4ws)):
        start=time.perf_counter()
        route=planner(scene)
        elapsed=time.perf_counter()-start
        if mode=="2WS":
            report=validate(scene,route)
            replay_ok=report.result=="PASS"
        else:
            _,replay_ok=replay_4ws(scene,route)
        result=summarize(scene,mode,route,replay_ok,elapsed)
        outputs[mode]=(route,result)
    return outputs


def run(out="docs/assets/parking_2ws_4ws",scenarios=None,animations=True):
    """Persist all results. Failed runs remain present and labelled."""
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    rows=[]
    for name in (tuple(SCENARIOS) if scenarios is None else scenarios):
        scene=SCENARIOS[name]
        results=compare_scene(scene)
        for mode in ("2WS","4WS"):
            rows.append(results[mode][1])
            print(f'{name:25} {mode:3} {results[mode][1]["success"]} '
                  f'{results[mode][1]["reason"]}',flush=True)
        if animations:
            from .parking_comparison_animation import render_comparison
            render_comparison(scene,results,out/f"{name}_2ws_vs_4ws.gif")
    (out/"results.json").write_text(json.dumps(rows,indent=2)+"\n",encoding="utf-8")
    with (out/"results.csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)
    return rows


if __name__=="__main__":
    run()
