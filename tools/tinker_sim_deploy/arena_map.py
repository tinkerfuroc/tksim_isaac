"""Scan-plane occupancy rasterizer for the RoboCup arena import.

Slices the arena's wall and furniture box colliders at the Livox scan
height and rasterizes the result into a ROS-style ``map_server`` PGM/YAML
pair (resolution 0.05, trinary mode). No timestamps or non-deterministic
state are embedded; the output is a pure function of the input geometry.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Mapping

from .arena_world import ArenaLayout, BoxCollider

_OCCUPIED = 0
_FREE = 254


class ArenaMapError(RuntimeError):
    pass


def livox_scan_height(robot_urdf_bytes: bytes) -> float:
    root = ET.fromstring(robot_urdf_bytes)
    for joint in root.iter("joint"):
        if joint.get("name") == "livox_joint":
            origin = joint.find("origin")
            if origin is None or not origin.get("xyz"):
                break
            return float(origin.get("xyz").split()[2])
    raise ArenaMapError("livox_joint origin not found in robot URDF")


def _footprints_at(
    layout: ArenaLayout,
    colliders: Mapping[str, tuple[BoxCollider, ...]],
    scan_height: float,
) -> list[tuple[float, float, float, float, float]]:
    """(cx, cy, half_x, half_y, yaw) rectangles that intersect the scan plane."""
    rects = []
    for wall in layout.walls:
        z_low = wall.center[2] - wall.size[2] / 2.0
        z_high = wall.center[2] + wall.size[2] / 2.0
        if z_low <= scan_height <= z_high:
            rects.append((wall.center[0], wall.center[1],
                          wall.size[0] / 2.0, wall.size[1] / 2.0, wall.yaw))
    for item in layout.furniture:
        for box in colliders.get(item.model_id, ()):
            world_z = item.position[2] + box.center[2]
            if not (world_z - box.size[2] / 2.0 <= scan_height <= world_z + box.size[2] / 2.0):
                continue
            cos_y, sin_y = math.cos(item.yaw), math.sin(item.yaw)
            cx = item.position[0] + cos_y * box.center[0] - sin_y * box.center[1]
            cy = item.position[1] + sin_y * box.center[0] + cos_y * box.center[1]
            rects.append((cx, cy, box.size[0] / 2.0, box.size[1] / 2.0,
                          item.yaw + box.yaw))
    if not rects:
        raise ArenaMapError("no geometry intersects the scan plane")
    return rects


def rasterize(
    layout: ArenaLayout,
    furniture_box_colliders: Mapping[str, tuple[BoxCollider, ...]],
    *,
    scan_height: float,
    resolution: float = 0.05,
    padding_m: float = 0.5,
) -> tuple[bytes, bytes]:
    rects = _footprints_at(layout, furniture_box_colliders, scan_height)
    corner_xs: list[float] = []
    corner_ys: list[float] = []
    for cx, cy, hx, hy, yaw in rects:
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                corner_xs.append(cx + sx * hx * abs(math.cos(yaw)) + sy * hy * abs(math.sin(yaw)))
                corner_ys.append(cy + sx * hx * abs(math.sin(yaw)) + sy * hy * abs(math.cos(yaw)))
    origin_x = math.floor((min(corner_xs) - padding_m) / resolution) * resolution
    origin_y = math.floor((min(corner_ys) - padding_m) / resolution) * resolution
    width = int(math.ceil((max(corner_xs) + padding_m - origin_x) / resolution))
    height = int(math.ceil((max(corner_ys) + padding_m - origin_y) / resolution))

    def cell_occupied(x: float, y: float) -> bool:
        for cx, cy, hx, hy, yaw in rects:
            dx, dy = x - cx, y - cy
            cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
            lx = cos_y * dx - sin_y * dy
            ly = sin_y * dx + cos_y * dy
            if abs(lx) <= hx and abs(ly) <= hy:
                return True
        return False

    rows = bytearray()
    for row in range(height):                      # top row = highest world y
        world_y = origin_y + (height - row - 0.5) * resolution
        for col in range(width):
            world_x = origin_x + (col + 0.5) * resolution
            rows.append(_OCCUPIED if cell_occupied(world_x, world_y) else _FREE)
    pgm = b"P5\n%d %d\n255\n" % (width, height) + bytes(rows)
    map_yaml = (
        "image: map.pgm\n"
        "mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x}, {origin_y}, 0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    ).encode("utf-8")
    return pgm, map_yaml
