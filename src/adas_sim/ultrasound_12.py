"""Twelve bumper-mounted ultrasonic channels for planar ADAS parking simulation.

This is an idealized acoustic field-of-view model. Each transducer casts a fan
of rays from its *physical mounting point*, returns the nearest obstacle
intersection and emits a deterministic detection record. Beam width, noise,
dropout and range are configurable. No multipath, acoustic interference,
echo intensity, thermal correction or sensor cycle timing is modelled.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, pi, radians, inf, isfinite
from random import Random
from .geometry import Pose, Vehicle, Rectangle, obstacle_polygon
from .sensors import _intersect


@dataclass(frozen=True)
class UltrasonicChannel:
    id: str
    x_body: float
    y_body: float
    yaw_body: float
    opening_angle_rad: float = radians(60)
    min_range_m: float = 0.20
    max_range_m: float = 4.5

    def __post_init__(self):
        if not (0 < self.opening_angle_rad <= pi) or not (0 <= self.min_range_m < self.max_range_m):
            raise ValueError("invalid ultrasonic FoV or range")


@dataclass(frozen=True)
class Echo:
    id: str
    origin_x: float
    origin_y: float
    heading_rad: float
    opening_angle_rad: float
    min_range_m: float
    max_range_m: float
    range_m: float | None
    hit_x: float | None
    hit_y: float | None
    status: str


def default_channels(vehicle: Vehicle = Vehicle()) -> tuple[UltrasonicChannel, ...]:
    """Six front and six rear sensors, positioned on the body outline.

    Coordinates use the rear-axle midpoint, x forward and y left. Front and
    rear units face outwards and are progressively angled towards the corners.
    These synthetic placements are NOT an OEM bumper mounting specification.
    """
    front = vehicle.length - vehicle.rear_axle_from_center
    rear = -vehicle.rear_axle_from_center
    half_width = vehicle.width / 2
    lateral = (-0.78, -0.48, -0.17, 0.17, 0.48, 0.78)
    headings_front = (-65, -35, -10, 10, 35, 65)
    headings_rear = (245, 215, 190, 170, 145, 115)
    channels = [
        UltrasonicChannel(f"F{i+1:02}", front-.08, half_width*v,
                          radians(a), opening_angle_rad=radians(55))
        for i,(v,a) in enumerate(zip(lateral,headings_front))
    ]
    channels += [
        UltrasonicChannel(f"R{i+1:02}", rear+.08, half_width*v,
                          radians(a), opening_angle_rad=radians(55))
        for i,(v,a) in enumerate(zip(lateral,headings_rear))
    ]
    return tuple(channels)


def mount_world(pose: Pose, channel: UltrasonicChannel):
    c,s = cos(pose.yaw), sin(pose.yaw)
    x = pose.x + c*channel.x_body - s*channel.y_body
    y = pose.y + s*channel.x_body + c*channel.y_body
    return x,y,pose.yaw+channel.yaw_body


def scan_12(pose: Pose, obstacles: tuple[Rectangle, ...],
            channels: tuple[UltrasonicChannel, ...] | None = None,
            *, vehicle: Vehicle = Vehicle(), rays: int = 21,
            noise_std_m: float = 0.0, dropout_probability: float = 0.0,
            seed: int = 0) -> tuple[Echo, ...]:
    if channels is None:
        channels = default_channels(vehicle)
    if rays < 2 or noise_std_m < 0 or not (0 <= dropout_probability <= 1):
        raise ValueError("invalid sensor sampling parameters")
    if len(channels) != 12 or len(set(c.id for c in channels)) != 12:
        raise ValueError("expected twelve distinct ultrasonic channels")
    rng=Random(seed)
    polygons=[obstacle_polygon(o) for o in obstacles]
    echoes=[]
    for channel in channels:
        ox,oy,heading=mount_world(pose,channel)
        best=(inf,None,None)
        for i in range(rays):
            bearing=heading + channel.opening_angle_rad*(i/(rays-1)-.5)
            direction=(cos(bearing),sin(bearing))
            for poly in polygons:
                for k in range(4):
                    distance=_intersect((ox,oy),direction,poly[k],poly[(k+1)%4])
                    if distance is not None and channel.min_range_m <= distance <= channel.max_range_m and distance < best[0]:
                        best=(distance,ox+distance*direction[0],oy+distance*direction[1])
        if rng.random() < dropout_probability:
            status="DROPOUT";measured=None;hx=hy=None
        elif isfinite(best[0]):
            status="DETECTED"
            measured=max(channel.min_range_m,min(channel.max_range_m,best[0]+rng.gauss(0.,noise_std_m)))
            hx,hy=best[1:]
        else:
            status="NO_ECHO";measured=None;hx=hy=None
        echoes.append(Echo(channel.id,ox,oy,heading,channel.opening_angle_rad,
                           channel.min_range_m,channel.max_range_m,measured,hx,hy,status))
    return tuple(echoes)
