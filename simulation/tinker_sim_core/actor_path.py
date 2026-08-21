from __future__ import annotations

import math
from typing import Sequence


def validate_path(path: Sequence[Sequence[float]]) -> list[tuple[float, float]]:
    if len(path) < 2:
        raise ValueError("actor path requires at least two waypoints")
    points = []
    for waypoint in path:
        if len(waypoint) != 2:
            raise ValueError("actor path waypoints must be [x, y] pairs")
        x, y = float(waypoint[0]), float(waypoint[1])
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError("actor path waypoints must be finite")
        points.append((x, y))
    return points


def path_length(path: Sequence[Sequence[float]]) -> float:
    points = validate_path(path)
    return sum(
        math.dist(points[i], points[i + 1]) for i in range(len(points) - 1)
    )


def path_pose_at(path: Sequence[Sequence[float]], distance: float) -> tuple[float, float, float]:
    """(x, y, yaw) at ``distance`` metres along the polyline, clamped to its end."""
    points = validate_path(path)
    if not math.isfinite(distance):
        raise ValueError("actor path distance must be finite")
    distance = max(0.0, float(distance))
    remaining = distance
    for start, end in zip(points, points[1:]):
        segment = math.dist(start, end)
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        if remaining <= segment or (start, end) == (points[-2], points[-1]):
            fraction = min(1.0, remaining / segment) if segment > 0.0 else 1.0
            return (
                start[0] + (end[0] - start[0]) * fraction,
                start[1] + (end[1] - start[1]) * fraction,
                yaw,
            )
        remaining -= segment
    # Unreachable: points has >= 2 elements (validate_path enforces this), so
    # the loop's last iteration always hits the (start, end) == (points[-2],
    # points[-1]) arm above and returns before falling out of the loop.
