"""Dependency-free deterministic SVG scenario renderer."""
from pathlib import Path
from html import escape
from .geometry import obstacle_polygon, vehicle_polygon
from .scenarios import Scenario
from .planner import Plan


def render(s: Scenario, p: Plan, out: str | Path) -> None:
    polygons = [obstacle_polygon(o) for o in s.obstacles]
    polygons.extend([vehicle_polygon(s.start, s.vehicle), vehicle_polygon(s.goal, s.vehicle)])
    points = [xy for poly in polygons for xy in poly] + [(q.x, q.y) for q in p.poses]
    x0 = min(x for x, y in points) - 2
    x1 = max(x for x, y in points) + 2
    y0 = min(y for x, y in points) - 2
    y1 = max(y for x, y in points) + 2
    scale = 42
    w, h = (x1 - x0) * scale, (y1 - y0) * scale
    def xy(q):
        return f"{(q[0] - x0) * scale:.1f},{(y1 - q[1]) * scale:.1f}"
    def poly(q, fill, stroke):
        return f'<polygon points="{" ".join(xy(z) for z in q)}" fill="{fill}" stroke="{stroke}" stroke-width="2"/>'
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.0f} {h:.0f}">', '<rect width="100%" height="100%" fill="#f8fafc"/>']
    parts.extend(poly(q, "#9ca3af", "#374151") for q in polygons[:-2])
    parts.append(poly(polygons[-2], "#bfdbfe", "#1d4ed8"))
    parts.append(poly(polygons[-1], "#bbf7d0", "#15803d"))
    if len(p.poses) > 1:
        parts.append(f'<polyline points="{" ".join(xy((q.x, q.y)) for q in p.poses)}" fill="none" stroke="#e11d48" stroke-width="2.5"/>')
    parts.append(f'<text x="15" y="24" fill="#111827" font-family="sans-serif" font-size="17">{escape(s.name)} — {"PLANNED" if p.success else p.reason}</text>')
    parts.append("</svg>")
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    Path(out).write_text("\n".join(parts), encoding="utf-8")
