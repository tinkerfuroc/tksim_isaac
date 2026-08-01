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

from typing import Mapping, Sequence

from .fixture_contract import (
    CollisionObjectSpec,
    FIXTURE_NAMESPACE_PREFIX,
    FIXTURE_OWNER,
    FIXTURE_STATE_READY,
    OBJECT_ADD,
    STATUS_SCHEMA_VERSION,
    TARGET_HANDOFF,
    canonical_json,
    revision_digest,
    sha256_json,
)

__all__ = [
    "FIXTURE_OWNER",
    "FIXTURE_STATE_READY",
    "STATUS_SCHEMA_VERSION",
    "TARGET_HANDOFF",
    "canonical_fixture_status",
    "fixture_descriptor",
    "fixture_descriptor_sha256",
    "fixture_owned_ids",
    "fixture_to_specs",
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


def _validate_namespace(planning_scene: Mapping[str, object]) -> None:
    """Fail closed on a malformed declaration before the node applies anything."""
    if not isinstance(planning_scene, Mapping):
        raise ValueError("planning_scene must be a mapping")
    for key in ("revision", "frame_id", "target_source_id", "target_handoff", "objects"):
        if key not in planning_scene:
            raise ValueError("planning_scene is missing required key {!r}".format(key))
    for record in planning_scene.get("objects", []):
        fixture_id = record.get("id")
        if not str(fixture_id).startswith(FIXTURE_NAMESPACE_PREFIX):
            raise ValueError("fixture id must be prefixed {!r}: {!r}".format(FIXTURE_NAMESPACE_PREFIX, fixture_id))
    if planning_scene.get("target_handoff") != TARGET_HANDOFF:
        raise ValueError("target_handoff must be {!r}".format(TARGET_HANDOFF))
