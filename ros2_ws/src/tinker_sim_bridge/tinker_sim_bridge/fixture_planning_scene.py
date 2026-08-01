"""ROS-free fixture planning-scene adapter (Task 5).

Bridges a scenario ``planning_scene`` declaration (schema version 2) into
concrete :class:`~tinker_sim_bridge.fixture_contract.CollisionObjectSpec`
ADD operations, the declared owned-id tuple, the canonical fixture descriptor,
and the canonical shared fixture-status mapping published by the live node.
This module imports neither ROS nor Isaac Sim and runs under both simulator
CPython 3.12 and system Humble CPython 3.10.

Diagnostic regions enter the collision-body set only when explicitly marked
``enter_collision_bodies: true``; the owned-id set matches exactly the objects
the adapter inserts into the PlanningScene so readback confirmation stays
consistent.
"""
from __future__ import annotations

import hashlib
import math
import struct
from pathlib import Path
from typing import Mapping, Sequence

from .fixture_contract import (
    CollisionObjectSpec,
    FIXTURE_NAMESPACE_PREFIX,
    FIXTURE_OWNER,
    FIXTURE_STATE_READY,
    OBJECT_ADD,
    STATUS_SCHEMA_VERSION,
    TARGET_HANDOFF,
    FixtureContractError,
    canonical_json,
    revision_digest,
    sha256_json,
)

# Explicitly supported, deterministic mesh file formats for fixture fixtures.
# Any other extension is rejected during scenario validation and mesh loading.
SUPPORTED_MESH_EXTENSIONS = (".stl", ".obj")

__all__ = [
    "FIXTURE_OWNER",
    "FIXTURE_STATE_READY",
    "STATUS_SCHEMA_VERSION",
    "SUPPORTED_MESH_EXTENSIONS",
    "TARGET_HANDOFF",
    "canonical_fixture_status",
    "find_project_root",
    "fixture_descriptor",
    "fixture_descriptor_sha256",
    "fixture_owned_ids",
    "fixture_to_specs",
    "load_mesh_asset",
    "parse_mesh_bytes",
]


def _pose_seven(record: Mapping[str, object]) -> tuple[float, float, float, float, float, float, float]:
    pose = record["pose"]
    xyz = pose["xyz"]
    xyzw = pose.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
    return tuple(float(value) for value in (*xyz, *xyzw))


def _record_to_spec(record: Mapping[str, object], frame_id: str) -> CollisionObjectSpec:
    if "primitive" in record:
        return CollisionObjectSpec(
            id=str(record["id"]),
            frame_id=frame_id,
            operation=OBJECT_ADD,
            primitives=(dict(record["primitive"]),),
            primitive_poses=(_pose_seven(record),),
        )
    mesh = record["mesh"]
    return CollisionObjectSpec(
        id=str(record["id"]),
        frame_id=frame_id,
        operation=OBJECT_ADD,
        meshes=(
            {
                "uri": str(mesh["uri"]),
                "sha256": str(mesh["sha256"]),
                "scale": [float(value) for value in mesh.get("scale", [1.0, 1.0, 1.0])],
            },
        ),
        mesh_poses=(_pose_seven(record),),
    )


def fixture_owned_ids(planning_scene: Mapping[str, object]) -> tuple[str, ...]:
    """Return the declared-order owned fixture ids.

    The owned set is exactly the collision-body set: every public object plus
    diagnostic regions explicitly marked ``enter_collision_bodies: true``.
    """
    result: list[str] = []
    for record in planning_scene.get("objects", []):
        result.append(str(record["id"]))
    for record in planning_scene.get("diagnostic_objects", []):
        if record.get("enter_collision_bodies") is True:
            result.append(str(record["id"]))
    return tuple(result)


def fixture_to_specs(planning_scene: Mapping[str, object]) -> tuple[CollisionObjectSpec, ...]:
    """Build the ADD specs for every fixture the adapter owns in the scene."""
    frame_id = str(planning_scene["frame_id"])
    specs: list[CollisionObjectSpec] = []
    for record in planning_scene.get("objects", []):
        specs.append(_record_to_spec(record, frame_id))
    for record in planning_scene.get("diagnostic_objects", []):
        if record.get("enter_collision_bodies") is True:
            specs.append(_record_to_spec(record, frame_id))
    return tuple(specs)


def fixture_descriptor(planning_scene: Mapping[str, object]) -> Mapping[str, object]:
    """Return the canonical fixture descriptor shared with later tasks.

    The descriptor carries the shared identities (owned ids, target source,
    handoff, revision) and the exact owned collision-body geometry that Task 6
    and the hardening reconciler consume.
    """
    specs = fixture_to_specs(planning_scene)
    return {
        "owner": FIXTURE_OWNER,
        "revision": str(planning_scene["revision"]),
        "revision_digest": revision_digest(planning_scene),
        "frame_id": str(planning_scene["frame_id"]),
        "target_source_id": str(planning_scene["target_source_id"]),
        "target_handoff": str(planning_scene["target_handoff"]),
        "owned_ids": list(fixture_owned_ids(planning_scene)),
        "objects": {spec.id: _spec_canonical(spec) for spec in specs},
    }


def _spec_canonical(spec: CollisionObjectSpec) -> Mapping[str, object]:
    return {
        "frame_id": spec.frame_id,
        "primitives": [dict(primitive) for primitive in spec.primitives],
        "primitive_poses": [list(pose) for pose in spec.primitive_poses],
        "meshes": [dict(mesh) for mesh in spec.meshes],
        "mesh_poses": [list(pose) for pose in spec.mesh_poses],
    }


def fixture_descriptor_sha256(planning_scene: Mapping[str, object]) -> str:
    """Return the lowercase SHA-256 of the canonical fixture descriptor bytes."""
    return sha256_json(fixture_descriptor(planning_scene))


def canonical_fixture_status(
    *,
    scenario: str,
    revision: str,
    revision_digest: str,
    sequence: int,
    published_at: float,
    owned_ids: Sequence[str],
    target_source_id: str,
    target_handoff: str,
    descriptor_sha256: str,
    state: str,
) -> Mapping[str, object]:
    """Build the canonical shared fixture-status mapping.

    The status contains exactly ``schema_version``, ``state``, ``scenario``,
    ``owner``, ``revision``, ``revision_digest``, monotonic ``sequence``, finite
    ``published_at``, declared-order ``owned_ids``, ``target_source_id``, scalar
    ``target_handoff``, and ``fixture_descriptor_sha256``.
    """
    return {
        "schema_version": STATUS_SCHEMA_VERSION,
        "state": state,
        "scenario": scenario,
        "owner": FIXTURE_OWNER,
        "revision": revision,
        "revision_digest": revision_digest,
        "sequence": sequence,
        "published_at": published_at,
        "owned_ids": list(owned_ids),
        "target_source_id": target_source_id,
        "target_handoff": target_handoff,
        "fixture_descriptor_sha256": descriptor_sha256,
    }


def serialize_status(status: Mapping[str, object]) -> str:
    """Serialize a canonical status with compact canonical separators."""
    return canonical_json(status).decode("utf-8")


def find_project_root(scenario_path: Path | str) -> Path:
    """Return the source-tree project root that owns a scenario file.

    The scenario file lives under ``simulation/scenarios/``; the project root is
    the nearest ancestor whose ``simulation/scenarios`` directory holds the
    file.  If no such ancestor exists (e.g. a temporary scenario), the scenario
    file's own parent directory is used so mesh assets can resolve beside it.
    """
    resolved = Path(scenario_path).resolve()
    cursor = resolved.parent
    while True:
        if (cursor / "simulation" / "scenarios").is_dir():
            return cursor
        if cursor == cursor.parent:
            return resolved.parent
        cursor = cursor.parent


def _validate_mesh(
    vertices: Sequence[Sequence[float]],
    triangles: Sequence[Sequence[int]],
) -> tuple[tuple[tuple[float, float, float], ...], tuple[tuple[int, int, int], ...]]:
    """Validate and normalize parsed mesh geometry, fail-closed.

    Requires nonempty finite vertices and nonempty triangles, every triangle
    index in range, and nondegenerate (nonzero-area) triangles.
    """
    if not vertices:
        raise FixtureContractError("mesh must contain at least one vertex")
    vertex_count = len(vertices)
    finite_vertices: list[tuple[float, float, float]] = []
    for vertex in vertices:
        if len(vertex) != 3:
            raise FixtureContractError("mesh vertex must contain exactly 3 coordinates")
        values = tuple(float(coord) for coord in vertex)
        if any(not math.isfinite(value) for value in values):
            raise FixtureContractError("mesh vertex must be finite")
        finite_vertices.append(values)
    if not triangles:
        raise FixtureContractError("mesh must contain at least one triangle")
    normalized_triangles: list[tuple[int, int, int]] = []
    for triangle in triangles:
        indices = tuple(int(index) for index in triangle)
        if len(indices) != 3:
            raise FixtureContractError("mesh triangle must contain exactly 3 vertex indices")
        if len(set(indices)) != 3:
            raise FixtureContractError("mesh triangle indices must be distinct")
        if any(index < 0 or index >= vertex_count for index in indices):
            raise FixtureContractError("mesh triangle index out of range")
        a, b, c = (finite_vertices[index] for index in indices)
        edge1 = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        edge2 = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        cross = (
            edge1[1] * edge2[2] - edge1[2] * edge2[1],
            edge1[2] * edge2[0] - edge1[0] * edge2[2],
            edge1[0] * edge2[1] - edge1[1] * edge2[0],
        )
        area2 = math.sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2])
        if not math.isfinite(area2) or area2 == 0.0:
            raise FixtureContractError("mesh triangle must be nondegenerate")
        normalized_triangles.append(indices)
    return tuple(finite_vertices), tuple(normalized_triangles)


def _parse_stl_binary(data: bytes, count: int):
    vertices: list[tuple[float, float, float]] = []
    triangles: list[tuple[int, int, int]] = []
    for triangle_index in range(count):
        base = 84 + triangle_index * 50
        if base + 12 + 36 > len(data):
            raise FixtureContractError("STL binary data is truncated")
        triangle_vertices = []
        for vertex_index in range(3):
            offset = base + 12 + vertex_index * 12
            x, y, z = struct.unpack_from("<fff", data, offset)
            triangle_vertices.append((float(x), float(y), float(z)))
        vertices.extend(triangle_vertices)
        first = triangle_index * 3
        triangles.append((first, first + 1, first + 2))
    return _validate_mesh(vertices, triangles)


def _parse_stl_ascii(data: bytes):
    text = data.decode("utf-8", errors="replace")
    vertices: list[tuple[float, float, float]] = []
    triangles: list[tuple[int, int, int]] = []
    current: list[tuple[float, float, float]] = []
    in_facet = False
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = line.split()
        keyword = parts[0].lower()
        if keyword == "facet":
            if in_facet:
                raise FixtureContractError("STL ASCII has a nested facet")
            in_facet = True
            current = []
        elif keyword == "endfacet":
            if not in_facet:
                raise FixtureContractError("STL ASCII endfacet without a facet")
            if len(current) != 3:
                raise FixtureContractError(
                    "STL ASCII facet must contain exactly 3 vertices"
                )
            base = len(vertices)
            vertices.extend(current)
            triangles.append((base, base + 1, base + 2))
            current = []
            in_facet = False
        elif keyword == "vertex":
            if not in_facet:
                raise FixtureContractError("STL ASCII vertex outside a facet")
            if len(parts) < 4:
                raise FixtureContractError("STL ASCII vertex requires x y z")
            try:
                values = (float(parts[1]), float(parts[2]), float(parts[3]))
            except ValueError as exc:
                raise FixtureContractError("STL ASCII vertex has non-numeric coordinate") from exc
            current.append(values)
        elif keyword == "endsolid":
            break
        # 'solid', 'normal', 'outer', 'loop', 'endloop' are structural keywords;
        # they carry no vertex data and are skipped.
    if in_facet:
        raise FixtureContractError("STL ASCII has an unterminated facet")
    return _validate_mesh(vertices, triangles)


def _parse_stl(data: bytes):
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * count == len(data):
            return _parse_stl_binary(data, count)
    if data[:5].strip().lower() == b"solid" or b"vertex" in data[:4096].lower():
        return _parse_stl_ascii(data)
    raise FixtureContractError("STL data is neither valid binary nor ASCII STL")


def _parse_obj(data: bytes):
    text = data.decode("utf-8", errors="replace")
    vertices: list[tuple[float, float, float]] = []
    raw_triangles: list[tuple[int, int, int]] = []
    for raw_line in text.splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        if parts[0] == "v":
            if len(parts) < 4:
                raise FixtureContractError("OBJ vertex requires x y z")
            try:
                values = (float(parts[1]), float(parts[2]), float(parts[3]))
            except ValueError as exc:
                raise FixtureContractError("OBJ vertex has non-numeric coordinate") from exc
            vertices.append(values)
        elif parts[0] == "f":
            indices: list[int] = []
            for token in parts[1:]:
                vertex_token = token.split("/")[0]
                try:
                    index = int(vertex_token)
                except ValueError as exc:
                    raise FixtureContractError("OBJ face has non-integer vertex index") from exc
                if index < 0:
                    raise FixtureContractError(
                        "OBJ relative (negative) vertex indices are not supported"
                    )
                indices.append(index)
            if len(indices) != 3:
                raise FixtureContractError("OBJ face must have exactly 3 vertices")
            raw_triangles.append(tuple(indices))
    # OBJ faces are 1-indexed; convert to 0-indexed for the canonical mesh form.
    triangles = [
        (index - 1 for index in triangle) for triangle in raw_triangles
    ]
    return _validate_mesh(vertices, [(i0, i1, i2) for i0, i1, i2 in triangles])


def parse_mesh_bytes(
    data: bytes, *, filename: str | Path | None = None
) -> tuple[tuple[tuple[float, float, float], ...], tuple[tuple[int, int, int], ...]]:
    """Parse mesh bytes into (vertices, triangles), fail-closed.

    Supports STL (binary or ASCII) and OBJ.  Returns finite vertices and
    nondegenerate, in-range triangle index tuples; raises
    :class:`FixtureContractError` on malformed, empty, or nonfinite geometry.
    """
    extension = Path(filename or "").suffix.lower()
    if extension == ".stl":
        return _parse_stl(data)
    if extension == ".obj":
        return _parse_obj(data)
    raise FixtureContractError(
        "unsupported mesh format {!r} (supported: {})".format(
            extension or "<none>", ", ".join(SUPPORTED_MESH_EXTENSIONS)
        )
    )


def load_mesh_asset(
    mesh: Mapping[str, object], *, project_root: Path | str
) -> tuple[tuple[tuple[float, float, float], ...], tuple[tuple[int, int, int], ...]]:
    """Resolve, verify, parse, and scale a declared mesh asset, fail-closed.

    The mesh declaration carries ``uri``, ``sha256``, and ``scale``.  The asset
    must exist (resolved against *project_root* unless absolute), its bytes must
    hash to the declared SHA-256, its extension must be a supported format, and
    its parsed geometry must be nonempty, finite, and nondegenerate.  The
    declared positive scale is applied to every vertex.
    """
    uri = str(mesh["uri"])
    declared_digest = str(mesh["sha256"])
    scale = [float(value) for value in mesh.get("scale", [1.0, 1.0, 1.0])]
    if any(not math.isfinite(value) or value <= 0 for value in scale):
        raise FixtureContractError("mesh scale must be finite and positive")
    path = Path(uri)
    if not path.is_absolute():
        path = Path(project_root) / uri
    if not path.is_file():
        raise FixtureContractError("mesh asset not found: {}".format(path))
    data = path.read_bytes()
    actual_digest = hashlib.sha256(data).hexdigest()
    if actual_digest != declared_digest:
        raise FixtureContractError(
            "mesh sha256 mismatch for {}: declared {} actual {}".format(
                path, declared_digest, actual_digest
            )
        )
    vertices, triangles = parse_mesh_bytes(data, filename=path.name)
    if scale != [1.0, 1.0, 1.0]:
        vertices = tuple(
            (vertex[0] * scale[0], vertex[1] * scale[1], vertex[2] * scale[2])
            for vertex in vertices
        )
    return vertices, triangles
