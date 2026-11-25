"""Vehicle-Dynamics-style functional survey/park animations.

The renderer receives measured sensor decisions; it does not invent a parking
route or display a goal before the survey has authorized parking. Only one
scheduled TDMA transmitter is highlighted per frame to avoid misleading
simultaneous overlapping acoustic beams.
"""
from __future__ import annotations
import math
from pathlib import Path
from .sensor_demo import build_case_payload
from .geometry import Pose,Rectangle,Vehicle,obstacle_polygon,vehicle_polygon
from .planar_vehicle import planar_parts
from .ultrasonic_overlay import UltrasonicOverlay, status_table

BG="#0b1120"
AXES="#0f172a"
GRID="#334155"
WHITE="#f8fafc"
MUTED="#94a3b8"
CYAN="#38bdf8"
AMBER="#f59e0b"
RED="#fb7185"
GREEN="#34d399"

def _theme():
    import matplotlib as mpl
    mpl.use("Agg")
    from matplotlib import pyplot as plt
    plt.rcParams.update({
        "figure.facecolor":BG,"figure.edgecolor":BG,
        "axes.facecolor":AXES,"axes.edgecolor":GRID,
        "axes.labelcolor":MUTED,"axes.titlecolor":WHITE,
        "xtick.color":MUTED,"ytick.color":MUTED,
        "text.color":WHITE,"font.family":"sans-serif",
        "grid.color":GRID,"grid.alpha":.4,"grid.linestyle":"--",
        "savefig.facecolor":BG,
    })
    return plt

def _vehicle(ax,vehicle):
    from matplotlib.patches import Polygon
    body=Polygon([(0,0)]*4,closed=True,fc="#3987e5",ec=CYAN,alpha=.58,zorder=7)
    ax.add_patch(body)
    wheels=[]
    for k in range(4):
        wheel=Polygon([(0,0)]*4,closed=True,fc="#111827",
                      ec=CYAN if k<2 else RED,lw=1.3,zorder=8)
        ax.add_patch(wheel)
        wheels.append(wheel)
    return body,wheels

def _move_vehicle(artists,pose,vehicle,front_steer):
    body,*wheels=planar_parts(pose,vehicle,front_steer,0.,
                             track_m=min(vehicle.width-.2,1.6))
    artists[0].set_xy(body)
    for patch,pts in zip(artists[1],wheels):
        patch.set_xy(pts)

def render_case(name,out="docs/assets/functional_parking",*,max_frames=20):
    from matplotlib.patches import Polygon
    from matplotlib.animation import FuncAnimation,PillowWriter
    from PIL import Image
    plt=_theme()
    data=build_case_payload(name)
    frames=data["frames"]
    survey_count=data["survey_samples"]
    vehicle=Vehicle()
    obstacles=[Rectangle(**o) for o in data["obstacles"]]
    destination=Path(out)
    destination.mkdir(parents=True,exist_ok=True)
    fig=plt.figure(figsize=(16,9))
    layout=fig.add_gridspec(1,2,width_ratios=[1.75,1.0])
    left=layout[0].subgridspec(2,2,height_ratios=[.95,1.3],width_ratios=[1.1,.72])
    overview=fig.add_subplot(left[0,:])
    close=fig.add_subplot(left[1,0])
    hud=fig.add_subplot(left[1,1])
    hud.axis("off")
    traces=layout[1].subgridspec(3,1)
    axes=[fig.add_subplot(traces[i,0]) for i in range(3)]
    plots=((overview,"measured search + conditional parking"),
           (close,"12 sensors / all FoVs + active TX"),
           (axes[0],"nearest valid front echo [m]"),
           (axes[1],"nearest valid rear echo [m]"),
           (axes[2],"valid channels / 12"))
    for ax,title in plots:
        ax.set_facecolor(AXES)
        ax.grid(True,alpha=.35,linestyle="--")
        ax.set_title(title,fontsize=9)
        ax.tick_params(labelsize=8)
    overview.set_xlim(-7,23)
    overview.set_ylim(-9,10)
    overview.set_aspect("equal",adjustable="box")
    overview.set_xlabel("X [m]")
    overview.set_ylabel("Y [m]")
    close.set_aspect("equal",adjustable="box")
    close.set_xlabel("X [m]")
    close.set_ylabel("Y [m]")
    for ax in (overview,close):
        for obstacle in obstacles:
            ax.add_patch(Polygon(obstacle_polygon(obstacle),
                                 fc="#64748b",ec="#cbd5e1",alpha=.75,zorder=2))
    overview.plot([f["pose"]["x"] for f in frames[:survey_count]],
                  [f["pose"]["y"] for f in frames[:survey_count]],
                  color=MUTED,alpha=.4,lw=1.2,ls="--",label="survey route")
    surveytrail,=overview.plot([],[],color=CYAN,lw=2.3,label="measured scan")
    parktrail,=overview.plot([],[],color=GREEN,lw=2.,label="authorized parking")
    car_over=_vehicle(overview,vehicle)
    car_close=_vehicle(close,vehicle)
    ultrasound=UltrasonicOverlay(close)
    target=Polygon([(0,0)]*4,closed=True,fc=GREEN,ec=GREEN,alpha=.2,visible=False,zorder=3)
    overview.add_patch(target)
    limits=[overview.axvline(0,color=GREEN,ls=":",lw=1.2,visible=False) for _ in range(2)]
    hudtext=hud.text(.02,.98,"",va="top",family="monospace",fontsize=7.15,transform=hud.transAxes)
    times=[f["time_s"] for f in frames]
    front=[];rear=[];valid=[]
    for frame in frames:
        fr=[e["range_m"] for e in frame["echoes"] if e["id"].startswith("F") and e["status"]=="DETECTED"]
        rr=[e["range_m"] for e in frame["echoes"] if e["id"].startswith("R") and e["status"]=="DETECTED"]
        front.append(min(fr) if fr else math.nan)
        rear.append(min(rr) if rr else math.nan)
        valid.append(sum(e["status"]=="DETECTED" for e in frame["echoes"]))
    for ax in axes:ax.set_xlim(0,max(times[-1],1))
    axes[0].plot(times,front,color=CYAN,lw=1.6)
    axes[1].plot(times,rear,color=AMBER,lw=1.6)
    axes[2].plot(times,valid,color=GREEN,lw=1.6)
    axes[0].set_ylim(0,5)
    axes[1].set_ylim(0,5)
    axes[2].set_ylim(-.5,12.5)
    axes[2].set_xlabel("t [s]")
    cursors=[ax.axvline(0,color=WHITE,lw=1.,alpha=.8) for ax in axes]
    overview.legend(loc="lower left",fontsize=8,facecolor="#1e293b",labelcolor=WHITE)
    fig.suptitle(f"ADAS / SENSOR-FIRST PARKING / {name}",fontsize=13,weight="bold")
    fig.tight_layout(rect=(0,.02,1,.96))
    count=max(3,min(max_frames,len(frames)))
    indices=sorted(set(round(i*(len(frames)-1)/(count-1)) for i in range(count)))
    indices=sorted(set(indices+[survey_count-1,min(survey_count,len(frames)-1)]))

    def update(frame_idx):
        i=indices[frame_idx]
        frame=frames[i]
        pose=Pose(**frame["pose"])
        _move_vehicle(car_over,pose,vehicle,frame["front_steer_rad"])
        _move_vehicle(car_close,pose,vehicle,frame["front_steer_rad"])
        survey=frames[:min(i+1,survey_count)]
        surveytrail.set_data([f["pose"]["x"] for f in survey],
                             [f["pose"]["y"] for f in survey])
        if i>=survey_count and data["status"]=="PARK_ALLOWED":
            parked=frames[survey_count:i+1]
            parktrail.set_data([f["pose"]["x"] for f in parked],
                               [f["pose"]["y"] for f in parked])
        else:parktrail.set_data([],[])
        slot=data["slot"] if i>=survey_count-1 else None
        target.set_visible(bool(slot))
        if slot:
            target.set_xy(vehicle_polygon(Pose(**slot["goal"]),vehicle,margin=0))
            for limit,x in zip(limits,(slot["left_boundary_x"],slot["right_boundary_x"])):
                limit.set_xdata((x,x));limit.set_visible(True)
        else:
            for limit in limits:limit.set_visible(False)
        close.set_xlim(pose.x-5.3,pose.x+5.3)
        close.set_ylim(pose.y-4.7,pose.y+4.7)
        active=frame["echoes"][i%12]
        active_id=active["id"] if data["tx_mode"]=="tdma" else None
        ultrasound.update(frame["echoes"],active_id=active_id,show_fovs=True)
        state="SEARCHING" if i<survey_count-1 else data["status"]
        colour=GREEN if state=="PARK_ALLOWED" else RED if state=="NO_PARK" else CYAN
        slot_len=f'{slot["certified_length_m"]:.2f} m' if slot else "--"
        lines=[f"STATE: {state}",f"PHASE: {frame['stage']}",
               f"Time: {frame['time_s']:.1f} s",f"Slot: {slot_len}",
               f"TX: {active_id or 'CONFLICT'} | {data['tx_mode'].upper()}",
               f"Hits: {valid[i]}/12","",
               "FRONT                     REAR",
               *status_table(frame["echoes"],active_id),"",
               f"PLAN: {data['path_steps'] if state=='PARK_ALLOWED' else 0} steps",
               f"REASON: {data['reason'] if state!='SEARCHING' else 'survey incomplete'}"]
        hudtext.set_text("\n".join(lines))
        hudtext.set_color(colour)
        for cursor in cursors:
            cursor.set_xdata((frame["time_s"],frame["time_s"]))
        return []

    gif=destination/f"{name}.gif"
    animation=FuncAnimation(fig,update,frames=len(indices),interval=130,blit=False)
    try:
        animation.save(gif,writer=PillowWriter(fps=8),dpi=100)
        update(len(indices)-1)
        fig.savefig(destination/f"{name}.png",dpi=110)
    finally:
        plt.close(fig)
    with Image.open(gif) as image:
        assert image.n_frames>=3 and image.size==(1600,900)
    return data
