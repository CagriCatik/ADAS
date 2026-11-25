"""Vehicle-Dynamics-style parking GIFs: common overview, close-up, HUD and traces."""
from pathlib import Path
from .visualization.vehicle_dynamics_style import from_plan,render_runs

def render_gif(scene,route,destination,fps=10,speedup=4.0,max_frames=50):
    run=from_plan(route,label="2WS Parking")
    return render_runs([run],destination,f"ADAS Parking | {scene.name}",
                       scene=scene,family="parking",vehicle=scene.vehicle,
                       fps=fps,playback=speedup,max_frames=max_frames)
