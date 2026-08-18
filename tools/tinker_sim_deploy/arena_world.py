"""Parse the pinned SOBITS arena world XML into a neutral layout model.

The upstream ``.world.xacro`` files are plain XML (upstream itself parses
them with ElementTree); no xacro processing is performed or supported.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

_POSE_EPSILON = 1e-6


class ArenaWorldError(RuntimeError):
    pass


@dataclass(frozen=True)
class WallBox:
    name: str
    size: tuple[float, float, float]     # metres, x/y/z
    center: tuple[float, float, float]   # world frame
    yaw: float                           # radians


@dataclass(frozen=True)
class FurniturePlacement:
    model_id: str                        # e.g. "rcw26_kitchen_table"
    position: tuple[float, float, float]
    yaw: float
    static: bool


@dataclass(frozen=True)
class BoxCollider:
    size: tuple[float, float, float]
    center: tuple[float, float, float]   # model-local frame
    yaw: float


@dataclass(frozen=True)
class MeshCollider:
    uri: str                             # mesh URI as written in the SDF


@dataclass(frozen=True)
class ArenaLayout:
    walls: tuple[WallBox, ...]
    furniture: tuple[FurniturePlacement, ...]


def _parse_pose(text: str | None, label: str) -> tuple[float, float, float, float]:
    """Return (x, y, z, yaw); fail closed on non-zero roll/pitch."""
    if text is None or not text.strip():
        return (0.0, 0.0, 0.0, 0.0)
    parts = text.split()
    if len(parts) != 6:
        raise ArenaWorldError(f"{label}: pose must have 6 values, got {text!r}")
    x, y, z, roll, pitch, yaw = (float(part) for part in parts)
    if abs(roll) > _POSE_EPSILON or abs(pitch) > _POSE_EPSILON:
        raise ArenaWorldError(
            f"{label}: non-zero roll/pitch is not supported by the importer"
        )
    return (x, y, z, yaw)


def _parse_static(text: str | None) -> bool:
    """Gazebo SDF writes ``<static>`` as either the word form or 0/1; both
    are seen across the real upstream worlds, so both must be recognized.
    """
    if text is None:
        return False
    return text.strip().lower() in ("true", "1")


def parse_world(
    xml_bytes: bytes,
    model_allowlist: frozenset[str],
    model_skiplist: frozenset[str] = frozenset(),
) -> ArenaLayout:
    try:
        root = ET.fromstring(xml_bytes)
    except ET.ParseError as error:
        raise ArenaWorldError(f"world XML is not parseable: {error}") from error
    world = root.find("world")
    if world is None:
        raise ArenaWorldError("world element missing")
    walls: list[WallBox] = []
    furniture: list[FurniturePlacement] = []
    for model in world.findall("model"):
        model_name = model.get("name", "model")
        mx, my, mz, myaw = _parse_pose(model.findtext("pose"), model_name)
        for link in model.findall("link"):
            link_name = link.get("name", "link")
            label = f"{model_name}/{link_name}"
            size_text = link.findtext("collision/geometry/box/size")
            if size_text is None:
                raise ArenaWorldError(f"{label}: wall link without box collision")
            sx, sy, sz = (float(part) for part in size_text.split())
            lx, ly, lz, lyaw = _parse_pose(link.findtext("pose"), label)
            if abs(myaw) > _POSE_EPSILON:
                cos_y, sin_y = math.cos(myaw), math.sin(myaw)
                lx, ly = mx + cos_y * lx - sin_y * ly, my + sin_y * lx + cos_y * ly
            else:
                lx, ly = mx + lx, my + ly
            walls.append(
                WallBox(name=label, size=(sx, sy, sz),
                        center=(lx, ly, mz + lz), yaw=myaw + lyaw)
            )
    for include in world.findall("include"):
        uri = (include.findtext("uri") or "").strip()
        if not uri.startswith("model://"):
            raise ArenaWorldError(f"include with unsupported uri: {uri!r}")
        model_id = uri[len("model://"):]
        if model_id in model_skiplist:
            continue
        if model_id not in model_allowlist:
            raise ArenaWorldError(f"model not on the import allowlist: {model_id}")
        x, y, z, yaw = _parse_pose(include.findtext("pose"), model_id)
        static = _parse_static(include.findtext("static"))
        furniture.append(
            FurniturePlacement(model_id=model_id, position=(x, y, z),
                               yaw=yaw, static=static)
        )
    if not walls:
        raise ArenaWorldError("world contains no wall links")
    return ArenaLayout(walls=tuple(walls), furniture=tuple(furniture))


def parse_model_colliders(sdf_bytes: bytes) -> tuple[BoxCollider | MeshCollider, ...]:
    try:
        root = ET.fromstring(sdf_bytes)
    except ET.ParseError as error:
        raise ArenaWorldError(f"model SDF is not parseable: {error}") from error
    colliders: list[BoxCollider | MeshCollider] = []
    for collision in root.iter("collision"):
        box_size = collision.findtext("geometry/box/size")
        mesh_uri = collision.findtext("geometry/mesh/uri")
        cylinder_radius = collision.findtext("geometry/cylinder/radius")
        cylinder_length = collision.findtext("geometry/cylinder/length")
        label = collision.get("name", "collision")
        if box_size is not None:
            sx, sy, sz = (float(part) for part in box_size.split())
            cx, cy, cz, cyaw = _parse_pose(collision.findtext("pose"), label)
            colliders.append(
                BoxCollider(size=(sx, sy, sz), center=(cx, cy, cz), yaw=cyaw)
            )
        elif cylinder_radius is not None and cylinder_length is not None:
            # No downstream consumer (map rasterization, collider authoring)
            # understands a cylinder primitive; represent it as its
            # axis-aligned bounding box (diameter x diameter x length). This
            # is a conservative over-approximation, not an exact match to
            # the round footprint.
            radius = float(cylinder_radius)
            length = float(cylinder_length)
            cx, cy, cz, cyaw = _parse_pose(collision.findtext("pose"), label)
            colliders.append(
                BoxCollider(
                    size=(2.0 * radius, 2.0 * radius, length),
                    center=(cx, cy, cz),
                    yaw=cyaw,
                )
            )
        elif mesh_uri is not None:
            # Mesh collisions reuse the visual mesh's own <pose> in the real
            # upstream SDFs, including a non-zero roll that corrects the
            # source GLB's Y-up authoring to Z-up (the same roll the
            # <visual> element carries). MeshCollider stores no pose --
            # author_model_colliders applies a convex hull directly to the
            # already visually-corrected mesh prim -- so the collision
            # <pose> is not read here and the box/cylinder roll==0
            # restriction does not apply to it.
            colliders.append(MeshCollider(uri=mesh_uri.strip()))
        else:
            raise ArenaWorldError("collision with unsupported geometry")
    return tuple(colliders)
