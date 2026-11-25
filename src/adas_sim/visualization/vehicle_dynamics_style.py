"""Vehicle-Dynamics-style common 2D planar animation engine.

Visual reference:
CagriCatik/Vehicle-Dynamics/src/vehicle_model/visualization/
  theme.py, plan_view.py, run_animation.py

Implements the SAME layout contract (overview, close-up, HUD, three aligned
telemetry plots), dark palette and true-scale planar polygons. The ADAS
model remains independent of the SevenDOFParams and upstream config loader.
All GIF producers use this renderer, not bespoke PIL layouts.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import math

from ..geometry import Pose, Vehicle, obstacle_polygon, vehicle_polygon
from ..planar_vehicle import planar_parts

BG = "#0b1120"
AXES = "#0f172a"
CARD = "#1e293b"
GRID = "#334155"
WHITE = "#f8fafc"
MUTED = "#94a3b8"
CYAN = "#38bdf8"
ROSE = "#f43f5e"
COLOURS = ("#3987e5", "#d95926", "#199e70", "#c98500")
VIEW_LIMITS = {
    "parking": ((-7.0, 21.0), (-13.0, 13.0)),
    "steering": ((-19.0, 21.0), (-20.0, 20.0)),
}

@dataclass(frozen=True)
class Run:
    label: str
    poses: tuple[Pose, ...]
    front: tuple[float, ...]
    rear: tuple[float, ...]
    velocity: tuple[float, ...]
    dt: float
    status: str = "SIMULATED"

    def __post_init__(self):
        n = len(self.poses)
        if n < 1 or any(len(x) != n for x in (self.front, self.rear, self.velocity)):
            raise ValueError("all history channels must have equal nonzero length")
        if n > 1 and self.dt <= 0:
            raise ValueError("recorded sample interval must be positive")

def from_plan(route, label="2WS", *, four_wheel=False, status=None):
    from ..four_wheel_steering import RearSteerConfig
    rear_config = RearSteerConfig()
    cmds = route.commands
    front = tuple(0.0 if not i or not cmds else cmds[min(i-1, len(cmds)-1)].steer
                  for i in range(len(route.poses)))
    speed = tuple(0.0 if not i or not cmds else cmds[min(i-1, len(cmds)-1)].velocity
                  for i in range(len(route.poses)))
    rear = tuple(rear_config.rear_target(x, four_wheel) for x in front)
    return Run(label, route.poses, front, rear, speed,
               route.step_dt if route.step_dt > 0 else 0.1,
               status or (route.reason if route.success else "FAIL: " + route.reason))

def from_steering_history(label, poses, front, rear, dt=0.05, status="KINEMATIC"):
    speeds = []
    for i in range(len(poses)):
        if i == 0:
            speeds.append(0.0)
        else:
            a, b = poses[i-1], poses[i]
            displacement = math.hypot(b.x-a.x, b.y-a.y) / dt
            sign = 1.0 if (b.x-a.x)*math.cos(a.yaw)+(b.y-a.y)*math.sin(a.yaw) >= 0 else -1.0
            speeds.append(sign * displacement)
    return Run(label, tuple(poses), tuple(front), tuple(rear), tuple(speeds), dt, status)

def _theme():
    import matplotlib as mpl
    mpl.use("Agg")
    from matplotlib import pyplot as plt
    mpl.rcParams.update({
        "figure.facecolor": BG, "figure.edgecolor": BG,
        "axes.facecolor": AXES, "axes.edgecolor": GRID,
        "axes.labelcolor": MUTED, "axes.titlecolor": WHITE,
        "axes.grid": True, "grid.color": GRID, "grid.linestyle": "--",
        "grid.alpha": 0.45, "grid.linewidth": 0.8,
        "xtick.color": MUTED, "ytick.color": MUTED,
        "text.color": WHITE, "font.family": "sans-serif",
        "legend.facecolor": CARD, "legend.edgecolor": GRID,
        "legend.labelcolor": WHITE, "savefig.facecolor": BG,
    })
    return plt

def _axis(ax, title, xlabel=None, ylabel=None):
    ax.set_facecolor(AXES)
    ax.set_title(title, color=WHITE, fontsize=10, weight="bold")
    if xlabel: ax.set_xlabel(xlabel, color=MUTED, fontsize=9)
    if ylabel: ax.set_ylabel(ylabel, color=MUTED, fontsize=9)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.grid(True, color=GRID, alpha=0.45, linestyle="--", linewidth=0.8)

def _indices(runs, fps, playback, max_frames):
    total = max((len(r.poses)-1)*r.dt for r in runs)
    n = min(max_frames, max(2, math.ceil(total*fps/playback)+1))
    return [total*i/(n-1) for i in range(n)]

def _pose_at(run, t):
    i = min(len(run.poses)-1, int(round(t/run.dt))) if run.dt > 0 else 0
    return i

def _prepare_axes(plt, title, runs, family):
    fig = plt.figure(figsize=(16, 9) if family == "parking" else (20, 10))
    outer = fig.add_gridspec(1, 2, width_ratios=[1.75, 1.0])
    left = outer[0].subgridspec(2, 2, height_ratios=[0.9, 1.25],
                                 width_ratios=[1.0, 0.75])
    overview = fig.add_subplot(left[0, :])
    closeup = fig.add_subplot(left[1, 0])
    hud = fig.add_subplot(left[1, 1])
    hud.axis("off")
    trace_grid = outer[1].subgridspec(3, 1)
    traces = [fig.add_subplot(trace_grid[i, 0]) for i in range(3)]
    xlim, ylim = VIEW_LIMITS[family]
    overview.set_xlim(*xlim)
    overview.set_ylim(*ylim)
    overview.set_aspect("equal", adjustable="box")
    _axis(overview, "recorded paths (true scale)", "X [m]", "Y [m]")
    closeup.set_aspect("equal", adjustable="box")
    _axis(closeup, f"close-up: {runs[0].label}", "X [m]", "Y [m]")
    labels = (("vehicle speed", "v [km/h]"), ("front road-wheel steering", "delta_f [deg]"),
              ("rear road-wheel steering", "delta_r [deg]"))
    for ax, (name, unit) in zip(traces, labels):
        _axis(ax, name, "t [s]", unit)
    fig.suptitle(title, fontsize=14, weight="bold")
    fig.tight_layout(rect=(0, 0.03, 1, .96))
    return fig, overview, closeup, hud, traces

def _place_objects(ax, scenario):
    if scenario is None:
        return
    from matplotlib.patches import Polygon
    for obstacle in scenario.obstacles:
        ax.add_patch(Polygon(obstacle_polygon(obstacle), closed=True,
                             fc="#64748b", ec="#cbd5e1", alpha=.7, zorder=1))
    ax.add_patch(Polygon(vehicle_polygon(scenario.goal, scenario.vehicle, margin=0.0),
                         closed=True, fc="#10b981", ec="#34d399",
                         alpha=.22, lw=1.5, zorder=1))

def _artists(ax, vehicle, colour):
    from matplotlib.patches import Polygon
    body = Polygon([(0,0)]*4, closed=True, fc=colour, ec=colour, alpha=.5, zorder=5)
    ax.add_patch(body)
    wheels = []
    for k in range(4):
        wheel = Polygon([(0,0)]*4, closed=True, fc="#1e1e24",
                        ec=CYAN if k < 2 else ROSE, linewidth=1.2, zorder=6)
        ax.add_patch(wheel)
        wheels.append(wheel)
    return body, wheels

def _update_vehicle(art, pose, steer_f, steer_r, vehicle):
    polys = planar_parts(pose, vehicle, steer_f, steer_r,
                         track_m=min(vehicle.width-.2,1.6),
                         tire_length_m=.64,tire_width_m=.225)
    art[0].set_xy(polys[0])
    for wheel, polygon in zip(art[1],polys[1:]):
        wheel.set_xy(polygon)

def render_runs(runs, destination, title, *, scene=None, family="parking",
                vehicle=Vehicle(), fps=10, playback=4.0, max_frames=50,
                snapshot=False, ultrasound=False):
    """Publish recorded, true-scale histories in the Vehicle-Dynamics layout."""
    if not runs or family not in VIEW_LIMITS or fps<=0 or playback<=0 or max_frames<2:
        raise ValueError("invalid renderer inputs")
    from matplotlib.animation import FuncAnimation, PillowWriter
    plt = _theme()
    fig, overview, closeup, hud, traces = _prepare_axes(plt, title, runs, family)
    _place_objects(overview, scene)
    _place_objects(closeup, scene)
    echoes_by_run = None
    if ultrasound and scene is not None:
        from ..ultrasound_12 import scan_12
        echoes_by_run = [scan_12(p, scene.obstacles, vehicle=scene.vehicle, seed=i)
                         for i, p in enumerate(runs[0].poses)]
    xs, ys = VIEW_LIMITS[family]
    for run in runs:
        for pose in run.poses:
            if not (xs[0] <= pose.x <= xs[1] and ys[0] <= pose.y <= ys[1]):
                plt.close(fig)
                raise ValueError("recorded path exceeds fixed family camera")
    all_trails = []
    moving = []
    for k, run in enumerate(runs):
        colour = COLOURS[k%len(COLOURS)]
        overview.plot([p.x for p in run.poses], [p.y for p in run.poses],
                      color=colour, alpha=.25, lw=.8)
        trail, = overview.plot([],[],color=colour,lw=1.8,label=run.label,zorder=4)
        all_trails.append(trail)
        moving.append(_artists(overview,vehicle,colour))
        t = [i*run.dt for i in range(len(run.poses))]
        data = ([abs(v)*3.6 for v in run.velocity],
                [math.degrees(v) for v in run.front],
                [math.degrees(v) for v in run.rear])
        for ax, values in zip(traces,data):
            ax.plot(t,values,color=colour,lw=1.2,label=run.label)
    overview.legend(loc="upper left",fontsize=8)
    detail = _artists(closeup,vehicle,COLOURS[0])
    sectors, echo_dots, echo_lines = [], [], []
    if echoes_by_run is not None:
        from matplotlib.patches import Wedge, Circle
        for channel in echoes_by_run[0]:
            sector = Wedge((0, 0), channel.max_range_m, 0, 1,
                           facecolor=CYAN, edgecolor=CYAN, lw=0.35,
                           alpha=0.07, zorder=2)
            dot = Circle((0, 0), .065, color="#f59e0b", visible=False, zorder=9)
            line, = closeup.plot([], [], color="#f59e0b", lw=1.3, zorder=8)
            closeup.add_patch(sector)
            closeup.add_patch(dot)
            sectors.append(sector)
            echo_dots.append(dot)
            echo_lines.append(line)
    detail_trail, = closeup.plot([p.x for p in runs[0].poses],
                                [p.y for p in runs[0].poses],
                                color=COLOURS[0], lw=.8, alpha=.3)
    cursors = [ax.axvline(0,color=WHITE,lw=1) for ax in traces]
    text = hud.text(0,1,"",transform=hud.transAxes,va="top",
                    family="monospace",fontsize=9,color=WHITE)
    times = _indices(runs,fps,playback,max_frames)

    def update(frame):
        t = times[frame]
        lines = [f"t: {t:6.2f} s", ""]
        for k, run in enumerate(runs):
            i = _pose_at(run,t)
            pose = run.poses[i]
            all_trails[k].set_data([p.x for p in run.poses[:i+1]],
                                   [p.y for p in run.poses[:i+1]])
            _update_vehicle(moving[k],pose,run.front[i],run.rear[i],vehicle)
            lines += [f"{run.label} [{run.status}]",
                      f"  v: {run.velocity[i]*3.6:+6.1f} km/h",
                      f"  df: {math.degrees(run.front[i]):+5.1f} deg",
                      f"  dr: {math.degrees(run.rear[i]):+5.1f} deg",""]
        first = runs[0]
        idx = _pose_at(first,t)
        pose = first.poses[idx]
        _update_vehicle(detail,pose,first.front[idx],first.rear[idx],vehicle)
        if echoes_by_run is not None:
            echoes = echoes_by_run[idx]
            distances = []
            for channel, sector, dot, line in zip(echoes,sectors,echo_dots,echo_lines):
                sector.set_center((channel.origin_x, channel.origin_y))
                sector.set_radius(channel.max_range_m)
                sector.set_theta1(math.degrees(channel.heading_rad-channel.opening_angle_rad/2))
                sector.set_theta2(math.degrees(channel.heading_rad+channel.opening_angle_rad/2))
                detected = channel.status == "DETECTED"
                dot.set_visible(detected)
                if detected:
                    dot.center = (channel.hit_x,channel.hit_y)
                    line.set_data([channel.origin_x,channel.hit_x],
                                  [channel.origin_y,channel.hit_y])
                    distances.append(f"{channel.id}: {channel.range_m:.2f}m")
                else:
                    line.set_data([], [])
                    distances.append(f"{channel.id}: --")
            lines.extend(["ULTRASONIC | "+str(sum(e.status=="DETECTED" for e in echoes))+"/12",
                          *["  ".join(distances[k:k+2]) for k in range(0,12,2)]])
        closeup.set_xlim(pose.x-6.0,pose.x+6.0)
        closeup.set_ylim(pose.y-6.0,pose.y+6.0)
        text.set_text("\n".join(lines))
        for line in cursors: line.set_xdata((t,t))
        return []

    dest=Path(destination)
    dest.parent.mkdir(parents=True,exist_ok=True)
    try:
        if snapshot:
            update(len(times)-1)
            fig.savefig(dest, format=dest.suffix.lstrip(".") or "png", dpi=95)
        else:
            anim=FuncAnimation(fig,update,frames=len(times),interval=1000/fps,blit=False)
            anim.save(dest,writer=PillowWriter(fps=fps),dpi=80)
    finally:
        plt.close(fig)
    return dest
