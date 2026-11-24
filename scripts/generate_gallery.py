"""Generate tracked repository GIF gallery from independently validated parking plans."""
from __future__ import annotations
from pathlib import Path
import json
from adas_sim.scenarios import SCENARIOS
from adas_sim.planner import plan
from adas_sim.validation import validate
from adas_sim.animation import render_gif
from adas_sim.svg import render
from adas_sim.control import execute
from adas_sim.planner import Plan

def main():
    folder=Path("docs/assets/parking")
    folder.mkdir(parents=True,exist_ok=True)
    rows=[]
    for name,scene in SCENARIOS.items():
        route=plan(scene)
        result=validate(scene,route)
        if result.result!="PASS":
            raise RuntimeError(f"refusing to publish failed {name}: {result}")
        render_gif(scene,route,folder/f"{name}.gif",fps=8,speedup=4,max_frames=50)
        render(scene,route,folder/f"{name}.svg")
        replay=execute(scene,route)
        dynamic=Plan(replay.poses,replay.commands,replay.dt,0,replay.status=="PASS",replay.status)
        render_gif(scene,dynamic,folder/f"{name}_actuator.gif",fps=8,speedup=4,max_frames=50)
        rows.append({"scenario":name,"planner":"hybrid_astar","result":result.result,
                     "frames":len(route.poses),"path_steps":len(route.commands),"actuator_status":replay.status})
        print(f"{name}: PASS -> {folder/name}.gif",flush=True)
    (folder/"manifest.json").write_text(json.dumps(rows,indent=2)+"\n",encoding="utf-8")

if __name__=="__main__":
    main()
