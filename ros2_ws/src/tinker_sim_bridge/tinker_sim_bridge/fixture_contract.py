"""ROS-free atomic fixture planning-scene contract (Task 5).

This module defines the concrete immutable data types and exact typed helpers
that both the live :mod:`fixture_planning_scene_node` and the pure tests
consume.  It imports neither ROS, Isaac Sim, nor simulator-extension packages
and runs under both simulator CPython 3.12 and system Humble CPython 3.10.

The one atomic PlanningScene diff carries every desired ``sim_fixture/*`` object
as an ADD operation and every stale existing ``sim_fixture/*`` id as a REMOVE
operation; other PlanningScene namespaces stay outside the fixture replacement
scope.  ``revision_digest`` computes the deterministic canonical digest over the
scenario ``planning_scene`` declaration, and ``confirm_fixture_revision`` proves
readback/status consistency before the node transitions to ready.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import struct
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from .model_contract import TOUCH_LINKS as MODEL_CONTRACT_TOUCH_LINKS

FIXTURE_NAMESPACE_PREFIX = "sim_fixture/"
FIXTURE_OWNER = "sim_fixture"
TARGET_HANDOFF = "pick_and_place/object_mesh"

# moveit_msgs/msg/CollisionObject operation constants.
OBJECT_ADD = 0
OBJECT_REMOVE = 1

# Canonical shared fixture-status schema.
STATUS_SCHEMA_VERSION = 1
FIXTURE_STATE_READY = "FIXTURE_READY"
FIXTURE_STATE_PENDING = "FIXTURE_PENDING"
FIXTURE_STATE_FAILED = "FIXTURE_FAILED"

_HEX64 = re.compile(r"^(?!0{64}$)[0-9a-f]{64}$")

# The touch-link set is supplied by the validated model contract
# (``model_contract.TOUCH_LINKS``); the fixture adapter never owns task objects.
# The downstream hardening reconciler receives the full SRDF-derived eight-link
# touch set through this single authoritative source.

# Canonical geometry precision: coordinates/dimensions are rounded to this many
# decimal places so the wire float32 mesh representation and the float64
# declaration normalize to the same deterministic signature.
_GEOM_PRECISION = 6
_SOLID_PRIMITIVE_NAME = {1: "box", 2: "sphere", 3: "cylinder"}


class FixtureContractError(ValueError):
    """Typed validation failure for the fixture planning-scene contract."""


def canonical_json(value: Any) -> bytes:
    """Return compact canonical JSON bytes (sorted keys, minimal separators)."""
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")


def sha256_json(value: Any) -> str:
    """Return the lowercase SHA-256 of the canonical JSON bytes of *value*."""
    return hashlib.sha256(canonical_json(value)).hexdigest()


def _quant(value: Any) -> float:
    """Round a finite number to the canonical geometry precision."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FixtureContractError("geometry value must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise FixtureContractError("geometry value must be finite")
    return round(number, _GEOM_PRECISION)


def _float32(value: Any) -> float:
    """Return the float32 wire round-trip of a number.

    ``shape_msgs/Mesh`` vertices serialize as ``float32``; the declaration side
    parses STL/OBJ text as ``float64``.  Quantizing the declaration through the
    same ``float32`` wire representation keeps the canonical geometry descriptor
    byte-identical to the readback for exact signature comparison.
    """
    return float(struct.unpack("<f", struct.pack("<f", float(value)))[0])


def _pose_spec(pose: Sequence[float]) -> list[float]:
    """Canonical ``(x, y, z, qx, qy, qz, qw)`` descriptor for a spec pose."""
    values = [_quant(value) for value in pose]
    if len(values) != 7:
        raise FixtureContractError("spec pose must contain exactly 7 values")
    return values


def _pose_readback(pose) -> list[float]:
    """Canonical ``(x, y, z, qx, qy, qz, qw)`` descriptor for a readback pose."""
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    values = [
        _quant(getattr(position, "x", 0.0)) if position is not None else 0.0,
        _quant(getattr(position, "y", 0.0)) if position is not None else 0.0,
        _quant(getattr(position, "z", 0.0)) if position is not None else 0.0,
        _quant(getattr(orientation, "x", 0.0)) if orientation is not None else 0.0,
        _quant(getattr(orientation, "y", 0.0)) if orientation is not None else 0.0,
        _quant(getattr(orientation, "z", 0.0)) if orientation is not None else 0.0,
        _quant(getattr(orientation, "w", 0.0)) if orientation is not None else 0.0,
    ]
    return values


def spec_geometry(
    spec: "CollisionObjectSpec", *, resolve_mesh=None
) -> Mapping[str, object]:
    """Return the canonical ROS-free geometry descriptor for a desired spec.

    The descriptor covers the object id, frame, primitive type+dimensions and
    poses, and (for mesh fixtures) the resolved vertices+triangles with scale
    applied.  Mesh resolution requires *resolve_mesh* (a callable mapping a
    ``{"uri", "sha256", "scale"}`` declaration to ``(vertices, triangles)``).
    """
    primitives = [
        {
            "type": str(primitive["type"]),
            "dimensions": [_quant(value) for value in primitive["dimensions"]],
        }
        for primitive in spec.primitives
    ]
    meshes: list[dict[str, object]] = []
    for mesh in spec.meshes:
        if resolve_mesh is None:
            raise FixtureContractError(
                "mesh geometry requires a mesh resolver for spec {!r}".format(spec.id)
            )
        vertices, triangles = resolve_mesh(mesh)
        meshes.append(
            {
                "vertices": [
                    [_quant(_float32(coord)) for coord in vertex] for vertex in vertices
                ],
                "triangles": [
                    [int(index) for index in triangle] for triangle in triangles
                ],
            }
        )
    return {
        "id": spec.id,
        "frame_id": spec.frame_id,
        "primitives": primitives,
        "primitive_poses": [_pose_spec(pose) for pose in spec.primitive_poses],
        "meshes": meshes,
        "mesh_poses": [_pose_spec(pose) for pose in spec.mesh_poses],
    }


def readback_geometry(obj) -> Mapping[str, object]:
    """Return the canonical ROS-free geometry descriptor for a readback object.

    *obj* is a ``moveit_msgs/CollisionObject`` message; the function reads only
    attributes (id, header.frame_id, primitives, primitive_poses, meshes,
    mesh_poses) so this module stays import-time ROS-free.
    """
    oid = str(getattr(obj, "id", ""))
    if not oid:
        raise FixtureContractError("readback collision object has an empty id")
    frame_id = str(getattr(getattr(obj, "header", None), "frame_id", ""))
    if not frame_id:
        raise FixtureContractError(
            "readback collision object {!r} has an empty frame_id".format(oid)
        )
    primitives: list[dict[str, object]] = []
    for primitive in getattr(obj, "primitives", ()) or ():
        primitive_type = int(getattr(primitive, "type", -1))
        primitives.append(
            {
                "type": _SOLID_PRIMITIVE_NAME.get(primitive_type, primitive_type),
                "dimensions": [
                    _quant(value)
                    for value in getattr(primitive, "dimensions", ()) or ()
                ],
            }
        )
    meshes: list[dict[str, object]] = []
    for mesh in getattr(obj, "meshes", ()) or ():
        vertices = [
            [
                _quant(getattr(vertex, "x", 0.0)),
                _quant(getattr(vertex, "y", 0.0)),
                _quant(getattr(vertex, "z", 0.0)),
            ]
            for vertex in getattr(mesh, "vertices", ()) or ()
        ]
        triangles = [
            [
                int(index)
                for index in (
                    getattr(triangle, "vertex_indices", ())
                    if getattr(triangle, "vertex_indices", None) is not None
                    else ()
                )
            ]
            for triangle in getattr(mesh, "triangles", ()) or ()
        ]
        meshes.append({"vertices": vertices, "triangles": triangles})
    return {
        "id": oid,
        "frame_id": frame_id,
        "primitives": primitives,
        "primitive_poses": [
            _pose_readback(pose)
            for pose in getattr(obj, "primitive_poses", ()) or ()
        ],
        "meshes": meshes,
        "mesh_poses": [
            _pose_readback(pose) for pose in getattr(obj, "mesh_poses", ()) or ()
        ],
    }


def geometry_signature_sha256(objects: Sequence[Mapping[str, object]]) -> str:
    """Return the SHA-256 of the canonical geometry descriptors (internal digest).

    This is the internal geometry digest used to prove the readback matches the
    declared fixture geometry; it is not part of the published 12-key status
    schema (whose ``fixture_descriptor_sha256`` remains the declaration digest).
    """
    return sha256_json(list(objects))


def _compare_geometry(
    expected_geometry: Mapping[str, Mapping[str, object]],
    observed_geometry: Sequence[Mapping[str, object]],
) -> list[str]:
    """Compare declared fixture geometry against the readback, fail-closed.

    Readback objects are normalized by id (order is not semantic); duplicate
    ``sim_fixture/*`` ids, missing owned objects, and per-object geometry
    mismatches each produce a reason.  Each mismatch reason carries the compact
    canonical SHA-256 of the expected and observed descriptors.
    """
    reasons: list[str] = []
    observed_by_id: dict[str, Mapping[str, object]] = {}
    observed_counts: dict[str, int] = {}
    for obj in observed_geometry:
        oid = str(obj.get("id", ""))
        observed_counts[oid] = observed_counts.get(oid, 0) + 1
        if oid not in observed_by_id:
            observed_by_id[oid] = obj
    duplicates = [
        oid
        for oid, count in observed_counts.items()
        if count > 1 and oid.startswith(FIXTURE_NAMESPACE_PREFIX)
    ]
    if duplicates:
        reasons.append(
            "readback contains duplicate fixture geometry ids: {}".format(
                ", ".join(sorted(duplicates))
            )
        )
    for oid, expected in expected_geometry.items():
        if oid not in observed_by_id:
            reasons.append(
                "readback is missing expected fixture object geometry: {}".format(oid)
            )
            continue
        observed = observed_by_id[oid]
        if observed != expected:
            reasons.append(
                "readback geometry mismatch for {}: expected {} observed {}".format(
                    oid,
                    sha256_json(expected),
                    sha256_json(observed),
                )
            )
    return reasons


@dataclass(frozen=True)
class CollisionObjectSpec:
    """Immutable specification of one collision object in an atomic diff.

    ``primitives`` entries use the ``{"type": "box"|"cylinder"|"sphere",
    "dimensions": [...]}`` canonical form with shape_msgs dimension order
    (box ``[x, y, z]``; cylinder ``[height, radius]``; sphere ``[radius]``).
    ``primitive_poses``/``mesh_poses`` are ``(x, y, z, qx, qy, qz, qw)`` tuples.
    ``meshes`` entries are ``{"uri", "sha256", "scale"}`` mappings.
    """

    id: str
    frame_id: str
    operation: int
    primitives: tuple[Mapping[str, object], ...] = ()
    primitive_poses: tuple[tuple[float, float, float, float, float, float, float], ...] = ()
    meshes: tuple[Mapping[str, object], ...] = ()
    mesh_poses: tuple[tuple[float, float, float, float, float, float, float], ...] = ()


@dataclass(frozen=True)
class PlanningSceneDiffPlan:
    """One atomic, JSON-able PlanningScene replacement diff.

    ``operations`` are the ADD specs for every desired fixture followed by
    REMOVE specs for every stale existing ``sim_fixture/*`` id; ``apply_request``
    is the canonical single diff consumed by the node (``is_diff`` true with a
    ``world.collision_objects`` array).  Foreign-namespace ids never appear.
    """

    added_ids: tuple[str, ...]
    removed_ids: tuple[str, ...]
    operations: tuple[CollisionObjectSpec, ...]
    apply_request: Mapping[str, object]


@dataclass(frozen=True)
class Confirmation:
    """Fail-closed readback/status confirmation for the fixture revision."""

    ready: bool
    reasons: tuple[str, ...]
    expected_revision: str
    expected_digest: str
    observed_revision: object
    observed_digest: object
    expected_owned_ids: tuple[str, ...]
    observed_scene_ids: tuple[str, ...]
    owned_ids_present: bool
    foreign_fixture_ids: tuple[str, ...]
    status_consistent: bool
    geometry_consistent: bool = False
    geometry_reasons: tuple[str, ...] = ()


def revision_digest(planning_scene: Mapping[str, object]) -> str:
    """Return the deterministic canonical digest of a fixture declaration.

    The digest covers the canonical JSON bytes of the full ``planning_scene``
    mapping with any ``revision_digest`` key excluded, so a declared digest can
    agree with its own input without self-reference.
    """
    if not isinstance(planning_scene, Mapping):
        raise FixtureContractError("planning_scene must be a mapping")
    payload = {str(key): value for key, value in planning_scene.items() if key != "revision_digest"}
    return sha256_json(payload)


def parse_required_fixture_owned_ids(value: str) -> tuple[str, ...]:
    """Parse the declared task-owned fixture ids from a launch parameter.

    Accepts a comma-separated string or a JSON array string, trims tokens,
    requires every id to live in the ``sim_fixture/*`` namespace, and rejects
    duplicates and empty inputs.
    """
    if not isinstance(value, str):
        raise FixtureContractError("required fixture owned ids must be a string")
    text = value.strip()
    if not text:
        raise FixtureContractError("required fixture owned ids must not be empty")
    if text.startswith("["):
        try:
            parsed = json.loads(text)
        except ValueError as exc:
            raise FixtureContractError("required fixture owned ids is not a JSON array") from exc
        if not isinstance(parsed, list):
            raise FixtureContractError("required fixture owned ids must parse to an array")
        tokens = [str(item) for item in parsed]
    else:
        tokens = [token.strip() for token in text.split(",") if token.strip()]
    result: list[str] = []
    for token in tokens:
        if not token.startswith(FIXTURE_NAMESPACE_PREFIX):
            raise FixtureContractError(
                "required fixture owned id must be prefixed {!r}: {!r}".format(
                    FIXTURE_NAMESPACE_PREFIX, token
                )
            )
        if token in result:
            raise FixtureContractError("duplicate required fixture owned id: {!r}".format(token))
        result.append(token)
    if not result:
        raise FixtureContractError("required fixture owned ids resolved to no ids")
    return tuple(result)


def _remove_spec(id: str, frame_id: str) -> CollisionObjectSpec:
    return CollisionObjectSpec(id=id, frame_id=frame_id, operation=OBJECT_REMOVE)


def _spec_to_collision_object(spec: CollisionObjectSpec) -> dict[str, object]:
    return {
        "id": spec.id,
        "operation": spec.operation,
        "frame_id": spec.frame_id,
        "primitives": [dict(primitive) for primitive in spec.primitives],
        "primitive_poses": [list(pose) for pose in spec.primitive_poses],
        "meshes": [dict(mesh) for mesh in spec.meshes],
        "mesh_poses": [list(pose) for pose in spec.mesh_poses],
    }


def build_atomic_revision_diff(
    *,
    desired_objects: Sequence[CollisionObjectSpec],
    existing_ids: Sequence[str],
) -> PlanningSceneDiffPlan:
    """Build one atomic fixture replacement diff.

    Every desired object (all of which must live in ``sim_fixture/*``) becomes
    an ADD operation and every stale existing ``sim_fixture/*`` id a REMOVE
    operation.  Foreign-namespace existing ids are preserved untouched.  The
    returned ``apply_request`` is exactly one JSON-able PlanningScene diff.
    """
    desired = list(desired_objects)
    desired_ids: list[str] = []
    for spec in desired:
        if not isinstance(spec, CollisionObjectSpec):
            raise FixtureContractError("desired_objects must contain CollisionObjectSpec instances")
        if not spec.id.startswith(FIXTURE_NAMESPACE_PREFIX):
            raise FixtureContractError(
                "fixture adapter may only create sim_fixture/* objects, got {!r}".format(spec.id)
            )
        if spec.operation != OBJECT_ADD:
            raise FixtureContractError("desired object {!r} must be an ADD operation".format(spec.id))
        desired_ids.append(spec.id)
    if len(desired_ids) != len(set(desired_ids)):
        raise FixtureContractError("desired fixture ids must be unique")

    desired_set = set(desired_ids)
    stale: list[str] = []
    seen: set[str] = set()
    for fixture_id in existing_ids:
        if not isinstance(fixture_id, str):
            raise FixtureContractError("existing_ids must contain strings")
        if fixture_id.startswith(FIXTURE_NAMESPACE_PREFIX) and fixture_id not in desired_set:
            if fixture_id not in seen:
                seen.add(fixture_id)
                stale.append(fixture_id)

    frame_id = desired[0].frame_id if desired else "base_link"
    operations = list(desired) + [_remove_spec(fixture_id, frame_id) for fixture_id in stale]
    apply_request: Mapping[str, object] = {
        "is_diff": True,
        "world": {
            "collision_objects": [_spec_to_collision_object(spec) for spec in operations]
        },
    }
    return PlanningSceneDiffPlan(
        added_ids=tuple(desired_ids),
        removed_ids=tuple(stale),
        operations=tuple(operations),
        apply_request=apply_request,
    )


def confirm_fixture_revision(
    *,
    service_result: bool,
    scene_ids: Sequence[str],
    status: Mapping[str, object],
    expected_revision: str,
    expected_digest: str,
    expected_owned_ids: Sequence[str],
    expected_geometry: Mapping[str, Mapping[str, object]] | None = None,
    observed_geometry: Sequence[Mapping[str, object]] | None = None,
) -> Confirmation:
    """Confirm a fixture readback against the canonical status, fail-closed.

    Proves the apply service succeeded, every expected owned id is present in
    the readback, no unexpected ``sim_fixture/*`` id leaked into the scene, no
    duplicate ``sim_fixture/*`` id is present, and the supplied canonical status
    is internally consistent with the expected revision, digest, owned ids,
    target identity, and heartbeat fields.

    When both *expected_geometry* (id -> canonical descriptor built from the
    desired specs) and *observed_geometry* (readback canonical descriptors) are
    supplied, the exact fixture geometry (frame, poses, primitive type+
    dimensions, mesh vertices+triangles) is compared before ready.
    """
    reasons: list[str] = []
    geometry_reasons: list[str] = []
    observed_scene = tuple(str(fixture_id) for fixture_id in scene_ids)
    expected = tuple(str(fixture_id) for fixture_id in expected_owned_ids)
    expected_set = set(expected)

    if not service_result:
        reasons.append("apply planning scene service did not succeed")

    owned_present = all(fixture_id in observed_scene for fixture_id in expected)
    if not owned_present:
        reasons.append("readback is missing expected fixture owned ids")

    observed_counts: dict[str, int] = {}
    for fixture_id in observed_scene:
        observed_counts[fixture_id] = observed_counts.get(fixture_id, 0) + 1
    duplicate_ids = tuple(
        fixture_id
        for fixture_id, count in observed_counts.items()
        if count > 1 and fixture_id.startswith(FIXTURE_NAMESPACE_PREFIX)
    )
    if duplicate_ids:
        reasons.append(
            "readback contains duplicate sim_fixture ids: {}".format(
                ", ".join(sorted(duplicate_ids))
            )
        )

    foreign = tuple(
        fixture_id
        for fixture_id in observed_scene
        if fixture_id.startswith(FIXTURE_NAMESPACE_PREFIX) and fixture_id not in expected_set
    )
    if foreign:
        reasons.append("readback contains unexpected sim_fixture ids")

    if expected_geometry is not None or observed_geometry is not None:
        if expected_geometry is None or observed_geometry is None:
            geometry_reasons.append(
                "geometry verification requires both expected and observed geometry"
            )
        else:
            geometry_reasons = _compare_geometry(
                expected_geometry, observed_geometry
            )

    status_checks: list[str] = []
    status_value: Mapping[str, object] = status if isinstance(status, Mapping) else {}
    if status_value.get("schema_version") != STATUS_SCHEMA_VERSION:
        status_checks.append("schema_version")
    if status_value.get("state") != FIXTURE_STATE_READY:
        status_checks.append("state")
    if status_value.get("owner") != FIXTURE_OWNER:
        status_checks.append("owner")
    if status_value.get("revision") != expected_revision:
        status_checks.append("revision")
    if status_value.get("revision_digest") != expected_digest:
        status_checks.append("revision_digest")
    if tuple(status_value.get("owned_ids", ())) != expected:
        status_checks.append("owned_ids")
    if status_value.get("target_source_id") not in expected_set:
        status_checks.append("target_source_id")
    if status_value.get("target_handoff") != TARGET_HANDOFF:
        status_checks.append("target_handoff")
    sequence = status_value.get("sequence")
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 1:
        status_checks.append("sequence")
    published_at = status_value.get("published_at")
    if isinstance(published_at, bool) or not isinstance(published_at, (int, float)) or not math.isfinite(float(published_at)):
        status_checks.append("published_at")
    descriptor = status_value.get("fixture_descriptor_sha256")
    if not isinstance(descriptor, str) or not _HEX64.fullmatch(descriptor):
        status_checks.append("fixture_descriptor_sha256")
    if status_checks:
        reasons.append("status inconsistent: {}".format(", ".join(status_checks)))
    reasons.extend(geometry_reasons)

    return Confirmation(
        ready=not reasons,
        reasons=tuple(reasons),
        expected_revision=str(expected_revision),
        expected_digest=str(expected_digest),
        observed_revision=status_value.get("revision"),
        observed_digest=status_value.get("revision_digest"),
        expected_owned_ids=expected,
        observed_scene_ids=observed_scene,
        owned_ids_present=owned_present,
        foreign_fixture_ids=foreign,
        status_consistent=not status_checks,
        geometry_consistent=not geometry_reasons,
        geometry_reasons=tuple(geometry_reasons),
    )


__all__ = [
    "Confirmation",
    "CollisionObjectSpec",
    "FIXTURE_NAMESPACE_PREFIX",
    "FIXTURE_OWNER",
    "FIXTURE_STATE_FAILED",
    "FIXTURE_STATE_PENDING",
    "FIXTURE_STATE_READY",
    "FixtureContractError",
    "MODEL_CONTRACT_TOUCH_LINKS",
    "OBJECT_ADD",
    "OBJECT_REMOVE",
    "STATUS_SCHEMA_VERSION",
    "TARGET_HANDOFF",
    "PlanningSceneDiffPlan",
    "build_atomic_revision_diff",
    "canonical_json",
    "confirm_fixture_revision",
    "geometry_signature_sha256",
    "parse_required_fixture_owned_ids",
    "readback_geometry",
    "revision_digest",
    "sha256_json",
    "spec_geometry",
]
