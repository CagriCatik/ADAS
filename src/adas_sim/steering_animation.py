"""Vehicle-Dynamics-style paired planar 2WS/4WS dashboard."""
from .geometry import Vehicle
from .visualization.vehicle_dynamics_style import from_steering_history,render_runs

def render_comparison(name,pair,path,fps=10,max_frames=50,dt=.05):
    runs=[]
    for mode in ("2WS","4WS"):
        poses,front,rear,_=pair[mode]
        runs.append(from_steering_history(mode,poses,front,rear,dt))
    return render_runs(runs,path,f"ADAS | 2WS vs 4WS | {name}",
                       family="steering",vehicle=Vehicle(),fps=fps,
                       playback=4.0,max_frames=max_frames)
