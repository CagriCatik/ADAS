"""SE(2) geometry, oriented vehicle footprints and convex SAT collision."""
from __future__ import annotations
from dataclasses import dataclass
import math


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Rectangle:
    x: float
    y: float
    length: float
    width: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Vehicle:
    length: float = 4.5
    width: float = 1.8
    wheelbase: float = 2.75
    max_steer: float = math.radians(35)
    rear_axle_from_center: float = 1.35
    safety_margin: float = 0.10

    def __post_init__(self) -> None:
        if min(self.length, self.width, self.wheelbase, self.rear_axle_from_center) <= 0 or not (0 < self.max_steer < math.pi / 2) or self.safety_margin < 0:
            raise ValueError("Invalid vehicle dimensions or limits")


def polygon(cx: float, cy: float, yaw: float, length: float, width: float) -> tuple[tuple[float, float], ...]:
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple((cx + c * dx - s * dy, cy + s * dx + c * dy) for dx, dy in ((length / 2, width / 2), (length / 2, -width / 2), (-length / 2, -width / 2), (-length / 2, width / 2)))


def vehicle_polygon(pose: Pose, vehicle: Vehicle, margin: float | None = None):
    """Pose refers to the rear-axle midpoint, including physical overhangs."""
    m = vehicle.safety_margin if margin is None else margin
    offset = vehicle.length / 2 - vehicle.rear_axle_from_center
    return polygon(pose.x + offset * math.cos(pose.yaw), pose.y + offset * math.sin(pose.yaw), pose.yaw, vehicle.length + 2 * m, vehicle.width + 2 * m)


def obstacle_polygon(rect: Rectangle):
    return polygon(rect.x, rect.y, rect.yaw, rect.length, rect.width)


def polygons_overlap(a, b) -> bool:
    """Separating axis theorem; touching is considered a collision."""
    for poly in (a, b):
        for i in range(len(poly)):
            p, q = poly[i], poly[(i + 1) % len(poly)]
            ax, ay = -(q[1] - p[1]), q[0] - p[0]
            va = [x * ax + y * ay for x, y in a]
            vb = [x * ax + y * ay for x, y in b]
            if max(va) < min(vb) - 1e-10 or max(vb) < min(va) - 1e-10:
                return False
    return True


def footprint_collision(pose: Pose, vehicle: Vehicle, obstacles: tuple[Rectangle, ...], margin: float | None = None) -> bool:
    body = vehicle_polygon(pose, vehicle, margin)
    return any(polygons_overlap(body, obstacle_polygon(o)) for o in obstacles)
