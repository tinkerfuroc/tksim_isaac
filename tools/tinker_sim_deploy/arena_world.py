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


# Collision overrides (config/arena-import.json "collision_overrides").
#
# Collider choice is otherwise driven entirely by the upstream model SDF:
# parse_model_colliders turns each <collision> into a BoxCollider (<box>,
# <cylinder>) or a MeshCollider (<mesh>), and author_model_colliders makes a
# MeshCollider a convexHull MeshCollisionAPI on the visual mesh -- which for
# an open bookshelf is one solid block from the floor to the top board, so an
# object spawned on a middle board starts INSIDE the hull and is ejected.
# An override replaces a model's SDF colliders with an explicit, model-local
# box set; the "open_shelf" kind expands a compact board/panel description
# into thin boxes so shelf levels are real, separate surfaces.
_OPEN_SHELF_KIND = "open_shelf"


def _pair(value: object, label: str) -> tuple[float, float]:
    try:
        lo, hi = (float(part) for part in value)  # type: ignore[union-attr]
    except (TypeError, ValueError) as error:
        raise ArenaWorldError(f"{label}: expected [lo, hi], got {value!r}") from error
    if not hi > lo:
        raise ArenaWorldError(f"{label}: expected lo < hi, got {value!r}")
    return lo, hi


def _slab(x: tuple[float, float], y: tuple[float, float], z: tuple[float, float]) -> BoxCollider:
    return BoxCollider(
        size=(x[1] - x[0], y[1] - y[0], z[1] - z[0]),
        center=((x[0] + x[1]) / 2.0, (y[0] + y[1]) / 2.0, (z[0] + z[1]) / 2.0),
        yaw=0.0,
    )


def expand_collision_override(spec: object, label: str = "collision override") -> tuple[BoxCollider, ...]:
    """Expand one ``collision_overrides`` entry into model-local BoxColliders.

    ``kind: "open_shelf"``::

        {"kind": "open_shelf",
         "footprint_x": [x_min, x_max],      # shelf bbox, model-local metres
         "footprint_y": [y_min, y_max],
         "footprint_z": [z_min, z_max],      # panel height (floor .. top)
         "boards_z": [[z_bottom, z_top], ...],  # one thin box per board
         "side_panel_thickness": 0.04,       # optional: boxes at both x edges
         "back_panel": {"side": "+y"|"-y"|"+x"|"-x", "thickness": 0.045}}  # optional

    Boards span the full x/y footprint; side panels are full-height, full-y
    boxes hugging ``footprint_x``'s edges; the back panel is a full-height,
    full-length box on the named side. Order: boards (ascending z), then side
    panels (-x, +x), then the back panel -- ``author_model_colliders`` names
    them ``box_00``.. in that order. Fails closed on a malformed spec.
    """
    if not isinstance(spec, dict):
        raise ArenaWorldError(f"{label}: expected an object, got {spec!r}")
    kind = spec.get("kind")
    if kind != _OPEN_SHELF_KIND:
        raise ArenaWorldError(f"{label}: unsupported kind {kind!r} (supported: {_OPEN_SHELF_KIND!r})")
    fx = _pair(spec.get("footprint_x"), f"{label}.footprint_x")
    fy = _pair(spec.get("footprint_y"), f"{label}.footprint_y")
    fz = _pair(spec.get("footprint_z"), f"{label}.footprint_z")
    boards_raw = spec.get("boards_z")
    if not isinstance(boards_raw, (list, tuple)) or not boards_raw:
        raise ArenaWorldError(f"{label}.boards_z: expected a non-empty list of [z_bottom, z_top]")
    boards = sorted(_pair(item, f"{label}.boards_z[{index}]") for index, item in enumerate(boards_raw))
    for lo, hi in boards:
        if lo < fz[0] - _POSE_EPSILON or hi > fz[1] + _POSE_EPSILON:
            raise ArenaWorldError(f"{label}.boards_z: board [{lo}, {hi}] outside footprint_z {list(fz)}")
    colliders = [_slab(fx, fy, board) for board in boards]
    side = spec.get("side_panel_thickness")
    if side is not None:
        thickness = float(side)
        if not 0.0 < thickness < (fx[1] - fx[0]) / 2.0:
            raise ArenaWorldError(f"{label}.side_panel_thickness: {side!r} not in (0, half width)")
        colliders.append(_slab((fx[0], fx[0] + thickness), fy, fz))
        colliders.append(_slab((fx[1] - thickness, fx[1]), fy, fz))
    back = spec.get("back_panel")
    if back is not None:
        if not isinstance(back, dict):
            raise ArenaWorldError(f"{label}.back_panel: expected an object")
        thickness = float(back.get("thickness", 0.0))
        panel_side = str(back.get("side", ""))
        axis_extent = fy if panel_side.endswith("y") else fx
        if panel_side not in ("+x", "-x", "+y", "-y") or not 0.0 < thickness < (axis_extent[1] - axis_extent[0]):
            raise ArenaWorldError(f"{label}.back_panel: bad side {panel_side!r} / thickness {thickness!r}")
        lo, hi = axis_extent
        span = (hi - thickness, hi) if panel_side.startswith("+") else (lo, lo + thickness)
        if panel_side.endswith("y"):
            colliders.append(_slab(fx, span, fz))
        else:
            colliders.append(_slab(span, fy, fz))
    return tuple(colliders)


def collision_overrides_from_config(config: object) -> dict[str, tuple[BoxCollider, ...]]:
    """``config["collision_overrides"]`` ({model_id: spec}) expanded per model;
    an absent/empty key yields ``{}``. Fails closed on any malformed entry."""
    raw = config.get("collision_overrides", {}) if isinstance(config, dict) else {}
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ArenaWorldError("collision_overrides: expected an object keyed by model_id")
    return {
        str(model_id): expand_collision_override(spec, label=f"collision_overrides[{model_id}]")
        for model_id, spec in raw.items()
        if not str(model_id).startswith("_")  # "_comment" and friends
    }


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
