"""Placement surface records derived from arena furniture placements.

A placement surface is a flat region on a piece of furniture (e.g. a table
top) expressed in the world frame, used downstream to constrain where
objects may be placed. Surfaces are serialized to a canonical
``placement.json`` payload: pure function of the input geometry, no
timestamps or other non-deterministic state.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Sequence

from .arena_world import FurniturePlacement

_MODEL_ID_PREFIX = "rcw26_"


@dataclass(frozen=True)
class PlacementSurface:
    surface_id: str        # "<furniture short name>#<surface name>"
    furniture_id: str
    center_xyz: tuple[float, float, float]   # world frame
    size_xy: tuple[float, float]
    yaw: float
    edge_margin: float


def _short_furniture_id(furniture_id: str) -> str:
    if furniture_id.startswith(_MODEL_ID_PREFIX):
        return furniture_id[len(_MODEL_ID_PREFIX):]
    return furniture_id


def world_surface(
    furniture: FurniturePlacement,
    *,
    surface_name: str,
    local_center: tuple[float, float, float],
    size_xy: tuple[float, float],
    edge_margin: float,
) -> PlacementSurface:
    lx, ly, lz = local_center
    cos_y, sin_y = math.cos(furniture.yaw), math.sin(furniture.yaw)
    wx = furniture.position[0] + cos_y * lx - sin_y * ly
    wy = furniture.position[1] + sin_y * lx + cos_y * ly
    wz = furniture.position[2] + lz
    surface_id = f"{_short_furniture_id(furniture.model_id)}#{surface_name}"
    return PlacementSurface(
        surface_id=surface_id,
        furniture_id=furniture.model_id,
        center_xyz=(wx, wy, wz),
        size_xy=tuple(size_xy),
        yaw=furniture.yaw,
        edge_margin=edge_margin,
    )


def placement_json(arena_id: str, surfaces: Sequence[PlacementSurface]) -> bytes:
    records = [
        {
            "surface_id": surface.surface_id,
            "furniture_id": surface.furniture_id,
            "center_xyz": list(surface.center_xyz),
            "size_xy": list(surface.size_xy),
            "yaw": surface.yaw,
            "edge_margin": surface.edge_margin,
        }
        for surface in sorted(surfaces, key=lambda item: item.surface_id)
    ]
    payload = {
        "schema_version": 1,
        "arena_id": arena_id,
        "surfaces": records,
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
