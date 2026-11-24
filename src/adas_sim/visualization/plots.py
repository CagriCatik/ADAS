"""Consistent dependency-free SVG comparison plots with shared metric scales."""
from __future__ import annotations
from html import escape
from pathlib import Path
from .common import PALETTE, PLOT_COLORS

def bar_chart(rows, *, label_key: str, value_key: str, out: str | Path,
              title: str, suffix: str = "", group_key: str | None = None,
              y_max: float | None = None):
    """Render a fixed-size comparable plot; a metric's bounds are selected once."""
    width, height = 1280, 720
    margin_left, margin_right, top, bottom = 135, 48, 115, 135
    base = height - bottom
    if y_max is None:
        vals = [float(r[value_key]) for r in rows if r.get(value_key) is not None]
        y_max = max(vals, default=1) * 1.2 or 1
    if y_max <= 0:
        raise ValueError("invalid plot axis")
    span = (width - margin_left - margin_right) / max(1, len(rows))
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        f'<rect width="100%" height="100%" fill="{PALETTE["background"]}"/>',
        f'<rect width="{width}" height="74" fill="{PALETTE["header"]}"/>',
        f'<text x="24" y="40" font-size="22" fill="#fff" font-family="sans-serif">{escape(title)}</text>',
    ]
    for i in range(5):
        y = base - i / 4 * (base - top)
        parts.append(f'<line x1="{margin_left}" y1="{y:.1f}" x2="{width-margin_right}" y2="{y:.1f}" stroke="#cbd5e1"/>')
        parts.append(f'<text x="{margin_left-15}" y="{y+5:.1f}" text-anchor="end" font-size="15" fill="{PALETTE["text"]}">{y_max * i / 4:.2f}</text>')
    for i, row in enumerate(rows):
        value = row.get(value_key)
        cx = margin_left + (i + .5) * span
        if value is not None:
            val = float(value)
            if val < 0 or val > y_max:
                raise ValueError("value exceeds shared plot scale")
            h = val / y_max * (base - top)
            key = str(row.get(group_key)) if group_key else ""
            color = PLOT_COLORS.get(key, PALETTE.get(key, "#2563eb"))
            parts.append(f'<rect x="{cx-span*.29:.1f}" y="{base-h:.1f}" width="{span*.58:.1f}" height="{h:.1f}" fill="{color}"/>')
            parts.append(f'<text x="{cx:.1f}" y="{base-h-10:.1f}" font-size="14" fill="{PALETTE["text"]}" text-anchor="middle">{val:.2f}{escape(suffix)}</text>')
        label = escape(str(row[label_key]))
        parts.append(f'<text x="{cx:.1f}" y="{base+26:.1f}" font-size="14" fill="{PALETTE["text"]}" text-anchor="middle">{label}</text>')
    parts.append(f'<text x="24" y="{height-27}" font-size="14" fill="{PALETTE["muted"]}">Same metric: shared axis and color convention. Synthetic simulation.</text>')
    parts.append('</svg>')
    p = Path(out); p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text("\n".join(parts) + "\n", encoding="utf-8")
