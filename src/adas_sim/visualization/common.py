"""Centralized rendering specification, coordinate transforms and visual semantics.

The family cameras are intentionally *not* inferred from each scenario. A measured
distance occupies the same number of pixels in every image of the same family.
"""
from __future__ import annotations
from dataclasses import dataclass
import math

PALETTE = {
    "background": "#eef3f9", "header": "#102038", "text": "#1d2d46",
    "muted": "#64748b", "obstacle": "#94a3b8", "obstacle_edge": "#334155",
    "start": "#bfdbfe", "goal": "#bbf7d0", "goal_edge": "#15803d",
    "planned": "#94a3b8", "executed": "#ea580c",
    "2WS": "#2563eb", "4WS": "#d97706",
    "wheel": "#182e49", "collision": "#dc2626", "reference": "#e11d48",
}
PLOT_COLORS = {"hybrid_astar": "#2563eb", "rrt": "#d97706",
               "rrt_star": "#16a34a", "fmt_star": "#dc2626"}

@dataclass(frozen=True)
class RenderSpec:
    width: int
    height: int
    pixels_per_meter: float
    center_x: float
    center_y: float
    fps: int = 10
    speedup: float = 4.0
    max_frames: int = 80
    header_height: int = 74
    footer_height: int = 55
    padding: int = 28

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0 or self.pixels_per_meter <= 0:
            raise ValueError("render geometry must be positive")
        if self.fps <= 0 or self.speedup <= 0 or self.max_frames < 2:
            raise ValueError("invalid frame configuration")
        if self.header_height + self.footer_height + 2 * self.padding >= self.height:
            raise ValueError("no usable plotting area")

PARKING = RenderSpec(1280, 720, 29.0, 6.0, 0.0)
STEERING = RenderSpec(1600, 800, 22.0, 2.5, 0.0)
CAMERA = {"parking": PARKING, "steering": STEERING}

@dataclass(frozen=True)
class Camera:
    spec: RenderSpec
    panel_index: int = 0
    panel_count: int = 1

    def __post_init__(self):
        if self.panel_count < 1 or self.panel_index not in range(self.panel_count):
            raise ValueError("invalid camera panel")

    @property
    def panel_width(self):
        return self.spec.width / self.panel_count

    @property
    def origin_x(self):
        return self.panel_width * (self.panel_index + 0.5)

    @property
    def origin_y(self):
        return (self.spec.header_height + self.spec.height - self.spec.footer_height) / 2

    def xy(self, point):
        x, y = point
        return (round(self.origin_x + (x - self.spec.center_x) * self.spec.pixels_per_meter),
                round(self.origin_y - (y - self.spec.center_y) * self.spec.pixels_per_meter))

    def visible(self, point) -> bool:
        x, y = self.xy(point)
        left = self.panel_index * self.panel_width + self.spec.padding
        right = (self.panel_index + 1) * self.panel_width - self.spec.padding
        return (left <= x <= right and
                self.spec.header_height + self.spec.padding <= y <=
                self.spec.height - self.spec.footer_height - self.spec.padding)

    def assert_visible(self, points):
        outside = [p for p in points if not self.visible(p)]
        if outside:
            raise ValueError("geometry exceeds fixed camera bounds; adjust family camera: "
                             + str(outside[:2]))

def image_indices(n: int, dt: float, spec: RenderSpec):
    """Sample playback indices at common apparent speed; always include last frame."""
    if n < 1:
        raise ValueError("empty trajectory")
    if n == 1:
        return [0]
    if dt <= 0:
        raise ValueError("dt must be positive")
    stride = max(1, round(spec.speedup / (spec.fps * dt)))
    indices = list(range(0, n, stride))
    if indices[-1] != n - 1:
        indices.append(n - 1)
    if len(indices) > spec.max_frames:
        skip = math.ceil((len(indices) - 1) / (spec.max_frames - 1))
        indices = indices[::skip]
        if indices[-1] != n - 1:
            indices.append(n - 1)
    return indices

def decorate_pillow(draw, spec: RenderSpec, title: str, subtitle: str, footer: str):
    draw.rectangle((0, 0, spec.width, spec.header_height), fill=PALETTE["header"])
    draw.text((24, 17), title, fill="#ffffff")
    draw.text((24, 43), subtitle, fill="#c7d2fe")
    draw.rectangle((0, spec.height-spec.footer_height, spec.width, spec.height),
                   fill="#e2e8f0")
    draw.text((24, spec.height-spec.footer_height+16), footer, fill=PALETTE["text"])
