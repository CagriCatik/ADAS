"""Planner comparisons: equal geometry, scenarios, replay-based acceptance."""
from __future__ import annotations
import argparse
import csv
import json
from dataclasses import asdict
from pathlib import Path
from time import perf_counter
from .sampling import choose, SamplingSettings
from .planner import Settings
from .scenarios import SCENARIOS
from .validation import validate
from .animation import render_gif
from .svg import render

METHODS=("hybrid_astar","rrt","rrt_star","fmt_star")

def benchmark(out:Path,methods=METHODS,scenarios=None,iterations=1200,seed=42,gifs=True):
    out.mkdir(parents=True,exist_ok=True)
    results=[]
    for name in (scenarios or tuple(SCENARIOS)):
        scene=SCENARIOS[name]
        for method in methods:
            tic=perf_counter()
            route=choose(scene,method,Settings(),
                         SamplingSettings(iterations=iterations,seed=seed,max_vertices=min(iterations,700)))
            elapsed=perf_counter()-tic
            report=validate(scene,route)
            row={"algorithm":method,"seed":seed,"iteration_budget":iterations,
                 "wall_time_s":round(elapsed,4),"reason":route.reason,**asdict(report)}
            results.append(row)
            folder=out/name/method
            folder.mkdir(parents=True,exist_ok=True)
            (folder/"result.json").write_text(json.dumps(row,indent=2)+"\n")
            render(scene,route,folder/"trajectory.svg")
            if gifs:render_gif(scene,route,folder/"vehicle.gif")
            print(f"{name:24s} {method:15s} {report.result:4s} {elapsed:6.2f}s "
                  f"expanded={route.expanded} {route.reason}",flush=True)
    (out/"benchmark.json").write_text(json.dumps(results,indent=2)+"\n")
    if results:
        with (out/"benchmark.csv").open("w",newline="") as f:
            writer=csv.DictWriter(f,fieldnames=results[0].keys())
            writer.writeheader()
            writer.writerows(results)
    return results

def main():
    p=argparse.ArgumentParser(description="Deterministic ADAS parking planner benchmark")
    p.add_argument("--algorithm",choices=("all",*METHODS),default="all")
    p.add_argument("--scenario",choices=("all",*SCENARIOS),default="open_bay")
    p.add_argument("--iterations",type=int,default=1200)
    p.add_argument("--seed",type=int,default=42)
    p.add_argument("--out",type=Path,default=Path("output/benchmarks"))
    p.add_argument("--no-gif",action="store_true")
    a=p.parse_args()
    methods=METHODS if a.algorithm=="all" else (a.algorithm,)
    scenarios=tuple(SCENARIOS) if a.scenario=="all" else (a.scenario,)
    benchmark(a.out,methods,scenarios,a.iterations,a.seed,not a.no_gif)

if __name__=="__main__":main()
