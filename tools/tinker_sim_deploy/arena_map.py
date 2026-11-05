"""Scan-band occupancy rasterizer for the RoboCup arena import.

Projects the arena's wall and furniture box colliders that fall inside the
band the 2-D scan is built from, and rasterizes the result into a ROS-style
``map_server`` PGM/YAML pair (resolution 0.05, trinary mode). No timestamps
or non-deterministic state are embedded; the output is a pure function of
the input geometry.

The band is what pointcloud_to_laserscan keeps: every point 0..2 m above
the sensor (livox360 frame), projected into one scan. So a tabletop or shelf
board above the sensor is a scan hit, and a plinth below it is not. That is
also what a SLAM map built from the real robot's scan contains. The sensor
sits at the livox joint height PLUS base_link's rest height above the floor
(0.195 + 0.0775 in sim); slicing at the bare joint height treated base_link
as the floor.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import TYPE_CHECKING, Mapping, Sequence

from .arena_world import ArenaLayout, BoxCollider, FurniturePlacement
from .robot_geometry import base_rest_height

if TYPE_CHECKING:
    import numpy as np

_OCCUPIED = 0
_FREE = 254
# pointcloud_to_laserscan's min_height / max_height in the livox360 frame
# (ros2_ws/src/tinker_sim_bridge/config/pointcloud_to_laserscan.yaml).
SCAN_BAND_MIN_ABOVE_SENSOR_M = 0.0
SCAN_BAND_MAX_ABOVE_SENSOR_M = 2.0


class ArenaMapError(RuntimeError):
    pass


def livox_scan_height(robot_urdf_bytes: bytes) -> float:
    """Height of the lidar above the floor: livox joint z + base_link rest height."""
    root = ET.fromstring(robot_urdf_bytes)
    for joint in root.iter("joint"):
        if joint.get("name") == "livox_joint":
            origin = joint.find("origin")
            if origin is None or not origin.get("xyz"):
                break
            return float(origin.get("xyz").split()[2]) + base_rest_height(robot_urdf_bytes)
    raise ArenaMapError("livox_joint origin not found in robot URDF")


def livox_scan_band(robot_urdf_bytes: bytes) -> tuple[float, float]:
    """(low, high) heights above the floor that the 2-D scan is built from."""
    sensor = livox_scan_height(robot_urdf_bytes)
    return sensor + SCAN_BAND_MIN_ABOVE_SENSOR_M, sensor + SCAN_BAND_MAX_ABOVE_SENSOR_M


def instance_footprints(
    item: FurniturePlacement,
    boxes: Sequence[BoxCollider],
    band: tuple[float, float],
) -> list[tuple[float, float, float, float, float]]:
    """(cx, cy, half_x, half_y, yaw) rectangles of one furniture item's boxes
    whose z-range overlaps the band."""
    low, high = band
    rects = []
    for box in boxes:
        world_z = item.position[2] + box.center[2]
        if not (world_z - box.size[2] / 2.0 <= high and low <= world_z + box.size[2] / 2.0):
            continue
        cos_y, sin_y = math.cos(item.yaw), math.sin(item.yaw)
        cx = item.position[0] + cos_y * box.center[0] - sin_y * box.center[1]
        cy = item.position[1] + sin_y * box.center[0] + cos_y * box.center[1]
        rects.append((cx, cy, box.size[0] / 2.0, box.size[1] / 2.0,
                      item.yaw + box.yaw))
    return rects


def _wall_footprints(
    layout: ArenaLayout, band: tuple[float, float]
) -> list[tuple[float, float, float, float, float]]:
    low, high = band
    rects = []
    for wall in layout.walls:
        z_low = wall.center[2] - wall.size[2] / 2.0
        z_high = wall.center[2] + wall.size[2] / 2.0
        if z_low <= high and low <= z_high:
            rects.append((wall.center[0], wall.center[1],
                          wall.size[0] / 2.0, wall.size[1] / 2.0, wall.yaw))
    return rects


def _footprints_at(
    layout: ArenaLayout,
    colliders: Mapping[str, tuple[BoxCollider, ...]],
    band: tuple[float, float],
) -> list[tuple[float, float, float, float, float]]:
    """(cx, cy, half_x, half_y, yaw) rectangles whose z-range overlaps the band."""
    rects = _wall_footprints(layout, band)
    for item in layout.furniture:
        rects.extend(instance_footprints(item, colliders.get(item.model_id, ()), band))
    if not rects:
        raise ArenaMapError("no geometry intersects the scan band")
    return rects


_SUBCELL_M = 0.005


def _outline_cells(
    outlines: Mapping[str, Sequence[np.ndarray]],
    origin_x: float, origin_y: float, width: int, height: int, resolution: float,
) -> np.ndarray:
    """(height, width) bool, row 0 = lowest world y: the map cells touched by
    any instance's outline. Each instance is rasterized on a 0.005 m sub-grid
    over its bounding window, enclosed hollows are filled, and a cell is
    occupied if any of its sub-cells is."""
    import numpy as np
    from scipy.ndimage import binary_fill_holes

    factor = max(1, int(round(resolution / _SUBCELL_M)))
    sub = resolution / factor
    cells = np.zeros((height, width), dtype=bool)
    for polygons in outlines.values():
        polys = [np.asarray(p, dtype=np.float64) for p in polygons]
        if not polys:
            continue
        allp = np.concatenate(polys)
        lo, hi = allp.min(axis=0), allp.max(axis=0)
        c0 = int(math.floor((lo[0] - origin_x) / resolution))
        c1 = int(math.floor((hi[0] - origin_x) / resolution)) + 1
        r0 = int(math.floor((lo[1] - origin_y) / resolution))
        r1 = int(math.floor((hi[1] - origin_y) / resolution)) + 1
        nx, ny = c1 - c0, r1 - r0
        xs = origin_x + c0 * resolution + (np.arange(nx * factor) + 0.5) * sub
        ys = origin_y + r0 * resolution + (np.arange(ny * factor) + 0.5) * sub
        mask = np.zeros((ny * factor, nx * factor), dtype=bool)
        for poly in polys:
            inside = np.ones(mask.shape, dtype=bool)
            for i in range(len(poly)):
                x0, y0 = poly[i]
                x1, y1 = poly[(i + 1) % len(poly)]
                inside &= (x1 - x0) * (ys[:, None] - y0) - (y1 - y0) * (xs[None, :] - x0) >= 0.0
            mask |= inside
        mask = binary_fill_holes(mask)
        window = mask.reshape(ny, factor, nx, factor).any(axis=(1, 3))
        # crop the window to the grid
        gr0, gr1 = max(r0, 0), min(r1, height)
        gc0, gc1 = max(c0, 0), min(c1, width)
        if gr0 < gr1 and gc0 < gc1:
            cells[gr0:gr1, gc0:gc1] |= window[gr0 - r0:gr1 - r0, gc0 - c0:gc1 - c0]
    return cells


def rasterize(
    layout: ArenaLayout,
    furniture_box_colliders: Mapping[str, tuple[BoxCollider, ...]],
    *,
    scan_band: tuple[float, float] | None = None,
    scan_height: float | None = None,
    resolution: float = 0.05,
    padding_m: float = 0.5,
    furniture_outlines: Mapping[str, Sequence[np.ndarray]] | None = None,
) -> tuple[bytes, bytes]:
    """``scan_band`` (low, high) above the floor; ``scan_height`` alone is a plane slice.

    ``furniture_outlines`` (instance name -> world-XY convex polygons) draws
    each furniture instance from its outline instead of its boxes; the grid
    frame is still computed from walls + boxes, so ``map.yaml`` is the same."""
    if (scan_band is None) == (scan_height is None):
        raise ArenaMapError("pass exactly one of scan_band or scan_height")
    band = scan_band if scan_band is not None else (scan_height, scan_height)
    rects = _footprints_at(layout, furniture_box_colliders, band)
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

    if furniture_outlines is not None:
        rects = _wall_footprints(layout, band)  # boxes are not drawn; cell_occupied sees walls only
        drawn = _outline_cells(furniture_outlines, origin_x, origin_y, width, height, resolution)
    else:
        drawn = None

    rows = bytearray()
    for row in range(height):                      # top row = highest world y
        world_y = origin_y + (height - row - 0.5) * resolution
        for col in range(width):
            world_x = origin_x + (col + 0.5) * resolution
            if cell_occupied(world_x, world_y) or (drawn is not None and drawn[height - row - 1][col]):
                rows.append(_OCCUPIED)
            else:
                rows.append(_FREE)
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
