"""Matplotlib adapter for a continuously visible twelve-transducer display.

All mounts and weak fields of view remain visible regardless of transmit
schedule. One TDMA sender gets an active pulse; interference never produces
a valid echo line. Renderer reads (but never changes) sensor decisions.
"""
from __future__ import annotations
from dataclasses import dataclass
import math

STATUS_COLOR = {"DETECTED":"#f59e0b","NO_ECHO":"#94a3b8",
                "INTERFERENCE":"#fb7185","DROPOUT":"#fb7185"}
IDS = tuple([*(f"F{i:02}" for i in range(1,5)),
             *(f"R{i:02}" for i in range(1,5)), "SLF", "SRF", "SLR", "SRR"])

@dataclass
class SensorArtists:
    footprint: object
    marker: object
    label: object

class UltrasonicOverlay:
    """12 mount markers, 12 faint fan FoVs, and one highlighted TX/echo."""
    def __init__(self, ax):
        from matplotlib.patches import Wedge,Circle
        self.channels=[]
        for cid in IDS:
            field=Wedge((0,0),1,0,1,facecolor="#38bdf8",edgecolor="#38bdf8",
                        linewidth=.35,alpha=.025,zorder=3)
            dot=Circle((0,0),.095,facecolor="#64748b",edgecolor="#e2e8f0",
                       linewidth=.65,zorder=12)
            label=ax.text(0,0,cid,fontsize=7,color="#e2e8f0",ha="left",va="center",
                          zorder=14,bbox={"facecolor":"#0f172a","alpha":.75,
                                          "edgecolor":"none","pad":.45})
            ax.add_patch(field);ax.add_patch(dot)
            self.channels.append(SensorArtists(field,dot,label))
        self.active_field=Wedge((0,0),1,0,1,facecolor="#38bdf8",edgecolor="#38bdf8",
                                linewidth=1.5,alpha=.23,zorder=5,visible=False)
        ax.add_patch(self.active_field)
        self.echo_point=Circle((0,0),.085,facecolor="#fbbf24",visible=False,zorder=18)
        ax.add_patch(self.echo_point)
        self.echo_line,=ax.plot([],[],color="#fbbf24",lw=2.,zorder=16)

    def update(self, echoes, *, active_id: str | None, show_fovs=True):
        if len(echoes)!=12 or {e["id"] for e in echoes}!=set(IDS):
            raise ValueError("12 distinct sensor states required")
        by_id={e["id"]:e for e in echoes}
        for artist in self.channels:
            e=by_id[artist.label.get_text()]
            x,y=e["origin_x"],e["origin_y"]
            active=active_id==e["id"]
            color=STATUS_COLOR.get(e["status"],"#94a3b8")
            artist.footprint.set_visible(show_fovs)
            artist.footprint.set_center((x,y))
            artist.footprint.set_radius(e["max_range_m"])
            artist.footprint.set_theta1(math.degrees(e["heading_rad"]-e["opening_angle_rad"]/2))
            artist.footprint.set_theta2(math.degrees(e["heading_rad"]+e["opening_angle_rad"]/2))
            artist.marker.center=(x,y)
            artist.marker.set_facecolor(color)
            artist.marker.set_edgecolor("#ffffff" if active else "#64748b")
            artist.marker.set_linewidth(1.8 if active else .55)
            front=e["id"].startswith("F") or e["id"] in ("SLF","SRF")
            artist.label.set_position((x+(.33 if front else -.33),y))
            artist.label.set_ha("left" if front else "right")
            artist.label.set_color("#fff7cc" if active else color)
            artist.label.set_fontweight("bold" if active else "normal")
        active=by_id.get(active_id)
        valid=active is not None and active["status"] in ("DETECTED","NO_ECHO")
        self.active_field.set_visible(valid and show_fovs)
        self.echo_point.set_visible(False)
        self.echo_line.set_data([],[])
        if valid:
            self.active_field.set_center((active["origin_x"],active["origin_y"]))
            self.active_field.set_radius(active["max_range_m"])
            self.active_field.set_theta1(math.degrees(active["heading_rad"]-active["opening_angle_rad"]/2))
            self.active_field.set_theta2(math.degrees(active["heading_rad"]+active["opening_angle_rad"]/2))
            if active["status"]=="DETECTED" and active["hit_x"] is not None:
                self.echo_point.set_visible(True)
                self.echo_point.center=(active["hit_x"],active["hit_y"])
                self.echo_line.set_data((active["origin_x"],active["hit_x"]),
                                        (active["origin_y"],active["hit_y"]))

def status_table(echoes,active_id):
    """Four rows display front, rear and side sensors in parallel columns."""
    by_id={e["id"]:e for e in echoes}
    columns=(("F01","R01","SLF"),("F02","R02","SRF"),
             ("F03","R03","SLR"),("F04","R04","SRR"))
    rows=[]
    for ids in columns:
        parts=[]
        for cid in ids:
            e=by_id[cid]
            active=">" if cid==active_id else " "
            state={"DETECTED":"HIT","NO_ECHO":"CLEAR","INTERFERENCE":"X-TALK",
                   "DROPOUT":"DROP"}.get(e["status"],"UNKNOWN")
            value=f"{e['range_m']:4.2f}" if e["range_m"] is not None else " -- "
            parts.append(f"{active}{cid} {state:6} {value}")
        rows.append(" | ".join(parts))
    return rows
