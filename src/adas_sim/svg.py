"""Parking SVG snapshots through the same Vehicle-Dynamics-style renderer as GIFs.

No separate white-background plotting logic, camera scaling or drawing palette.
"""
from .visualization.vehicle_dynamics_style import from_plan, render_runs


def render(s, p, out):
    return render_runs(
        [from_plan(p, label="2WS Parking")],
        out,
        f"ADAS Parking | {s.name}",
        scene=s,
        family="parking",
        vehicle=s.vehicle,
        max_frames=2,
        snapshot=True,
    )
