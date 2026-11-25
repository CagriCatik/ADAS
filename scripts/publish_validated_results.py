"""Publish validated parking results (Vehicle-Dynamics dashboard, 12-channel ultrasound).

These are deterministic kinematic replay demonstrations, not certified parking
solutions. Only scenarios that pass collision and goal tolerance checks are
published. No planner or control fallback is silently substituted.
"""
from __future__ import annotations

import csv
import json
from dataclasses import asdict
from pathlib import Path

from adas_sim.planner import plan
from adas_sim.scenarios import SCENARIOS
from adas_sim.ultrasound_12 import scan_12
from adas_sim.validation import validate
from adas_sim.visualization.vehicle_dynamics_style import from_plan, render_runs

SCENARIOS_TO_PUBLISH = ("parallel_left", "parallel_right", "perpendicular_reverse")


def generate(destination="docs/assets/validated_results"):
    from PIL import Image

    out = Path(destination)
    out.mkdir(parents=True, exist_ok=True)
    summaries = []
    for name in SCENARIOS_TO_PUBLISH:
        scenario = SCENARIOS[name]
        route = plan(scenario)
        report = validate(scenario, route)
        if report.result != "PASS" or not route.success:
            raise RuntimeError(f"Refusing failed scenario: {name}: {route.reason}, {report}")
        if len(route.poses) != len(route.commands) + 1:
            raise ValueError("Incomplete or inconsistent trajectory")
        sensors = [scan_12(pose, scenario.obstacles, vehicle=scenario.vehicle, seed=i)
                   for i, pose in enumerate(route.poses)]
        if any(len(e) != 12 for e in sensors):
            raise ValueError("Expected 12 independent ultrasonic channels")
        with (out / f"{name}_ultrasound.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s", *[e.id + "_m" for e in sensors[0]]])
            for i, scan in enumerate(sensors):
                w.writerow([round(i * route.step_dt, 4),
                            *[f"{e.range_m:.4f}" if e.range_m is not None else ""
                              for e in scan]])
        run = from_plan(route, label="2WS Hybrid A* (kinematic)")
        gif = out / f"{name}.gif"
        image = out / f"{name}.png"
        title = f"ADAS / 12-USS / {name} / validated simulation"
        render_runs([run], gif, title, scene=scenario, vehicle=scenario.vehicle,
                    family="parking", fps=8, playback=4.0, max_frames=18,
                    ultrasound=True)
        render_runs([run], image, title, scene=scenario, vehicle=scenario.vehicle,
                    family="parking", snapshot=True, fps=8, playback=4.0,
                    max_frames=2, ultrasound=True)
        with Image.open(gif) as f:
            if f.n_frames < 3 or f.size != (1280, 720):
                raise RuntimeError(f"Invalid GIF output: {gif}")
        with Image.open(image) as f:
            f.verify()
        if image.stat().st_size < 20000:
            raise RuntimeError(f"Unexpected PNG output: {image}")
        count = sum(e.status == "DETECTED" for scan in sensors for e in scan)
        summary = {
            "scenario": name, "status": report.result,
            "goal_position_error_m": round(report.final_position_error_m, 4),
            "goal_heading_error_deg": round(report.final_heading_error_deg, 3),
            "path_length_m": round(report.path_length_m, 4),
            "duration_s": round(report.duration_s, 3),
            "gear_changes": report.reversals,
            "ultrasonic_echo_count": count,
            "ultrasonic_samples": len(sensors),
            "gif_frames": Image.open(gif).n_frames,
            "physics": "low_speed_kinematic", "tracking": "open_loop_command_replay",
            "collision_model": "discrete_oriented_rectangle_SAT",
        }
        summaries.append(summary)
        print(f'{name}: {report.result}, {count} echoes, {summary["gif_frames"]} GIF frames', flush=True)

    (out / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n", encoding="utf-8")
    with (out / "summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=summaries[0].keys())
        writer.writeheader()
        writer.writerows(summaries)
    return summaries


if __name__ == "__main__":
    generate()
