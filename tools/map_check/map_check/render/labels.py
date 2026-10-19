"""Greedy non-overlapping label placement for the overview, as in prototypes/render_rooms.py."""
from __future__ import annotations

import math
from typing import Mapping

Box = tuple[float, float, float, float]  # x0, y0, x1, y1 in pixels

RADII = (40, 70, 100, 135, 175, 220)  # px from the anchor, tried in order
ANGLES = 16  # per radius: the facing direction, then alternating either side of it
PAD_X, PAD_Y = 6, 4  # px between the text and the box edge
EDGE = 4  # px kept clear at the image edges and under the header
ROBOT_HALF = 18  # px; no label covers the square this far around any anchor


def _clear(a: Box, b: Box) -> bool:
    return a[2] < b[0] or a[0] > b[2] or a[3] < b[1] or a[1] > b[3]


def place_labels(anchors: Mapping[str, tuple[float, float]], sizes: Mapping[str, tuple[int, int]],
                 facing: Mapping[str, float], w: int, h: int,
                 header_px: int = 44) -> dict[str, tuple[Box, tuple[float, float]]]:
    """name -> (label box, anchor). sizes are text sizes; facing is an image angle (atan2 dv, du)."""
    lo = (EDGE, header_px + EDGE, w - EDGE, h - EDGE)
    taken = [(0.0, 0.0, float(w), float(header_px))]
    taken += [(u - ROBOT_HALF, v - ROBOT_HALF, u + ROBOT_HALF, v + ROBOT_HALF)
              for u, v in anchors.values()]

    def free(box):
        return (box[0] >= lo[0] and box[1] >= lo[1] and box[2] <= lo[2] and box[3] <= lo[3]
                and all(_clear(box, t) for t in taken))

    placed = {}
    for name in sorted(anchors, key=lambda n: anchors[n][1]):  # top of the image first
        u, v = anchors[name]
        tw, th = sizes[name]
        bw, bh = tw + 2 * PAD_X, th + 2 * PAD_Y
        base = facing[name]
        box = None
        for r in RADII:
            for k in range(ANGLES):
                ang = base + (k // 2 + 1) * (math.pi / 8) * (1 if k % 2 else -1) if k else base
                cx, cy = u + r * math.cos(ang), v + r * math.sin(ang)
                cand = (cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2)
                if free(cand):
                    box = cand
                    break
            if box:
                break
        if box is None:  # crowded: up and right of the anchor, kept inside the image
            x0 = min(max(u + 20, lo[0]), lo[2] - bw)
            y0 = min(max(v - bh, lo[1]), lo[3] - bh)
            box = (x0, y0, x0 + bw, y0 + bh)
        taken.append(box)
        placed[name] = (box, (u, v))
    return {n: placed[n] for n in anchors}
