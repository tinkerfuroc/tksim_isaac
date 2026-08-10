"""Tests for the atomic fixture planning-scene adapter (Task 5).

Pure ROS-free contract tests run under the simulator Python 3.12 venv with
source ``PYTHONPATH``.  Node-level behavior (sequence monotonicity, fail-closed
retry/state, failure/timeout/malformed service paths, exactly-one apply) is
covered further down under the Humble ROS runtime through a local import after
sourcing system ROS.
"""
from __future__ import annotations

import json
import math
import os
import struct
import sys
import time
from hashlib import sha256
from pathlib import Path
from typing import Mapping

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_bridge.fixture_contract import (  # noqa: E402
    CollisionObjectSpec,
    OBJECT_ADD,
    OBJECT_REMOVE,
    TARGET_HANDOFF,
    build_atomic_revision_diff,
    canonical_json,
    confirm_fixture_revision,
    geometry_signature_sha256,
    parse_required_fixture_owned_ids,
    readback_geometry,
    revision_digest,
    spec_geometry,
)
from tinker_sim_bridge.fixture_contract import (  # noqa: E402
    FIXTURE_STATE_FAILED,
    FIXTURE_STATE_PENDING,
)
from tinker_sim_bridge.fixture_planning_scene import (  # noqa: E402
    FIXTURE_OWNER,
    FIXTURE_STATE_READY,
    STATUS_SCHEMA_VERSION,
    SUPPORTED_MESH_EXTENSIONS,
    canonical_fixture_status,
    fixture_descriptor_sha256,
    fixture_owned_ids,
    fixture_to_specs,
    load_mesh_asset,
    parse_mesh_bytes,
)

SCENARIOS = ROOT / "simulation/scenarios"


def load_fixture_scenario(path: Path) -> Mapping[str, object]:
    """Load the ``planning_scene`` fixture declaration from a scenario file."""
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    planning_scene = raw.get("planning_scene")
    if not isinstance(planning_scene, dict):
        raise ValueError(f"{path}: scenario has no planning_scene object")
    return planning_scene


def _record_spec(record: Mapping[str, object], frame_id: str) -> CollisionObjectSpec:
    """Convert one fixture record into an ADD ``CollisionObjectSpec``."""
    pose = record["pose"]
    xyz = pose["xyz"]
    xyzw = pose.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
    pose7 = tuple(float(value) for value in (*xyz, *xyzw))
    if "primitive" in record:
        return CollisionObjectSpec(
            id=str(record["id"]),
            frame_id=frame_id,
            operation=OBJECT_ADD,
            primitives=(dict(record["primitive"]),),
            primitive_poses=(pose7,),
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
                "scale": list(mesh.get("scale", [1.0, 1.0, 1.0])),
            },
        ),
        mesh_poses=(pose7,),
    )


def fixture_to_scene(declaration: Mapping[str, object]) -> tuple[CollisionObjectSpec, ...]:
    """Local spec builder mirroring the adapter's ``fixture_to_specs``."""
    frame_id = str(declaration["frame_id"])
    specs = []
    for record in declaration.get("objects", []):
        specs.append(_record_spec(record, frame_id))
    for record in declaration.get("diagnostic_objects", []):
        if record.get("enter_collision_bodies") is True:
            specs.append(_record_spec(record, frame_id))
    return tuple(specs)


def _tiny_binary_stl() -> bytes:
    """Return a valid one-triangle binary STL byte string (nondegenerate)."""
    header = b"\x00" * 80
    count = struct.pack("<I", 1)
    normal = struct.pack("<fff", 0.0, 0.0, 1.0)
    vertices = b"".join(
        struct.pack("<fff", x, y, z)
        for (x, y, z) in ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    )
    attribute = struct.pack("<H", 0)
    return header + count + normal + vertices + attribute


def _tiny_ascii_stl() -> bytes:
    return (
        b"solid tiny\n"
        b"facet normal 0 0 1\nouter loop\n"
        b"vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\n"
        b"endloop\nendfacet\nendsolid tiny\n"
    )


def _tiny_obj() -> bytes:
    return b"# tiny triangle\no cube\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"


def _ready_status(
    declaration: Mapping[str, object],
    *,
    scenario: str,
    sequence: int,
    published_at: float,
) -> Mapping[str, object]:
    return canonical_fixture_status(
        scenario=scenario,
        revision=str(declaration["revision"]),
        revision_digest=revision_digest(declaration),
        sequence=sequence,
        published_at=published_at,
        owned_ids=fixture_owned_ids(declaration),
        target_source_id=str(declaration["target_source_id"]),
        target_handoff=str(declaration["target_handoff"]),
        descriptor_sha256=fixture_descriptor_sha256(declaration),
        state=FIXTURE_STATE_READY,
    )


# ---------------------------------------------------------------------------
# Scenario fixtures: exact IDs, shared target identity, deterministic digests
# ---------------------------------------------------------------------------


def test_joint_and_pose_share_pedestal_and_public_target_identity() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    pose = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-pose.json")

    assert fixture_owned_ids(joint) == ("sim_fixture/pedestal", "sim_fixture/public_target")
    assert fixture_owned_ids(pose) == ("sim_fixture/pedestal", "sim_fixture/public_target")
    for declaration in (joint, pose):
        assert declaration["frame_id"] == "base_link"
        assert declaration["target_source_id"] == "sim_fixture/public_target"
        assert declaration["target_handoff"] == "pick_and_place/object_mesh"
        assert declaration["target_handoff"] == TARGET_HANDOFF
        assert str(declaration["target_source_id"]) in fixture_owned_ids(declaration)


def test_exact_task_owned_handoff_identity_constant() -> None:
    assert TARGET_HANDOFF == "pick_and_place/object_mesh"


def test_parse_required_fixture_owned_ids_accepts_declared_owned_ids() -> None:
    assert parse_required_fixture_owned_ids(
        "sim_fixture/pedestal, sim_fixture/public_target"
    ) == ("sim_fixture/pedestal", "sim_fixture/public_target")
    assert parse_required_fixture_owned_ids(
        '["sim_fixture/pedestal", "sim_fixture/public_target"]'
    ) == ("sim_fixture/pedestal", "sim_fixture/public_target")
    with pytest.raises(ValueError):
        parse_required_fixture_owned_ids("nav/foreign")
    with pytest.raises(ValueError):
        parse_required_fixture_owned_ids("sim_fixture/a,sim_fixture/a")
    with pytest.raises(ValueError):
        parse_required_fixture_owned_ids("")


def test_revision_digest_is_deterministic_canonical_and_declared() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    first = revision_digest(joint)
    assert first == revision_digest(joint)
    # The declared digest in the scenario equals the recomputed canonical digest.
    assert first == joint["revision_digest"]
    assert isinstance(first, str)
    assert len(first) == 64
    # Canonical compact bytes are contractual for the digest.
    payload = {k: v for k, v in joint.items() if k != "revision_digest"}
    expected = sha256(
        json.dumps(
            payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True
        ).encode("utf-8")
    ).hexdigest()
    assert first == expected
    assert canonical_json(joint) == json.dumps(
        joint, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    # A revision change changes the digest deterministically.
    altered = dict(joint)
    altered["revision"] = str(joint["revision"]) + "-x"
    assert revision_digest(altered) != first


def test_pose_scenario_digest_differs_from_joint() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    pose = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-pose.json")
    assert revision_digest(pose) != revision_digest(joint)
    assert pose["revision_digest"] == revision_digest(pose)


# ---------------------------------------------------------------------------
# Atomic replacement diff: namespace-scoped ADD + REMOVE in one request
# ---------------------------------------------------------------------------


def test_atomic_diff_adds_desired_and_removes_only_stale_fixture_ids() -> None:
    desired = fixture_to_scene(
        load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    )
    existing = [
        "sim_fixture/stale_removed",
        "sim_fixture/pedestal",
        "sim_fixture/public_target",
        "sim_fixture/other_stale",
        "nav/foreign_kept",
    ]
    plan = build_atomic_revision_diff(desired_objects=desired, existing_ids=existing)

    assert plan.added_ids == ("sim_fixture/pedestal", "sim_fixture/public_target")
    assert plan.removed_ids == ("sim_fixture/stale_removed", "sim_fixture/other_stale")
    assert plan.apply_request["is_diff"] is True

    collision_objects = plan.apply_request["world"]["collision_objects"]
    assert isinstance(collision_objects, list)
    ids = [co["id"] for co in collision_objects]
    ops = [co["operation"] for co in collision_objects]
    assert ids == ["sim_fixture/pedestal", "sim_fixture/public_target",
                   "sim_fixture/stale_removed", "sim_fixture/other_stale"]
    assert ops == [OBJECT_ADD, OBJECT_ADD, OBJECT_REMOVE, OBJECT_REMOVE]
    # One atomic request carries both the desired ADDs and the stale REMOVEs.
    assert sum(1 for op in plan.operations if op.operation == OBJECT_ADD) == 2
    assert sum(1 for op in plan.operations if op.operation == OBJECT_REMOVE) == 2
    # No foreign namespace removal and no foreign addition.
    assert "nav/foreign_kept" not in ids
    for spec in plan.operations:
        assert spec.id.startswith("sim_fixture/")
    # The diff is deterministic: identical inputs produce identical bytes.
    again = build_atomic_revision_diff(desired_objects=desired, existing_ids=existing)
    assert canonical_json(plan.apply_request) == canonical_json(again.apply_request)


def test_atomic_diff_no_stale_removes_when_existing_ids_foreign_only() -> None:
    desired = fixture_to_scene(
        load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-pose.json")
    )
    existing = ["nav/obstacle_a", "nav/obstacle_b"]
    plan = build_atomic_revision_diff(desired_objects=desired, existing_ids=existing)
    assert plan.removed_ids == ()
    assert plan.added_ids == ("sim_fixture/pedestal", "sim_fixture/public_target")
    collision_objects = plan.apply_request["world"]["collision_objects"]
    assert all(co["operation"] == OBJECT_ADD for co in collision_objects)


def test_diff_rejects_foreign_desired_object() -> None:
    foreign = CollisionObjectSpec(
        id="nav/not_fixture",
        frame_id="base_link",
        operation=OBJECT_ADD,
        primitives=({"type": "box", "dimensions": [1.0, 1.0, 1.0]},),
        primitive_poses=((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),),
    )
    with pytest.raises(ValueError):
        build_atomic_revision_diff(desired_objects=[foreign], existing_ids=[])


def test_diff_rejects_duplicate_desired_ids() -> None:
    spec = CollisionObjectSpec(
        id="sim_fixture/dupe",
        frame_id="base_link",
        operation=OBJECT_ADD,
        primitives=({"type": "box", "dimensions": [1.0, 1.0, 1.0]},),
        primitive_poses=((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),),
    )
    with pytest.raises(ValueError):
        build_atomic_revision_diff(desired_objects=[spec, spec], existing_ids=[])


def test_local_and_production_spec_builders_agree() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    assert fixture_to_specs(joint) == fixture_to_scene(joint)
    pose = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-pose.json")
    assert fixture_to_specs(pose) == fixture_to_scene(pose)


def test_diagnostic_objects_enter_collision_bodies_only_when_marked() -> None:
    declaration: Mapping[str, object] = {
        "revision": "diag-r1",
        "frame_id": "base_link",
        "target_source_id": "sim_fixture/public_target",
        "target_handoff": "pick_and_place/object_mesh",
        "objects": [
            {
                "id": "sim_fixture/public_target",
                "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
                "pose": {"xyz": [0.5, 0.0, 0.9], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            }
        ],
        "diagnostic_objects": [
            {
                "id": "sim_fixture/diag_excluded",
                "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
                "pose": {"xyz": [1.0, 0.0, 0.5], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                "enter_collision_bodies": False,
            },
            {
                "id": "sim_fixture/diag_included",
                "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
                "pose": {"xyz": [1.5, 0.0, 0.5], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                "enter_collision_bodies": True,
            },
        ],
    }
    specs = fixture_to_scene(declaration)
    assert tuple(spec.id for spec in specs) == (
        "sim_fixture/public_target",
        "sim_fixture/diag_included",
    )
    assert fixture_owned_ids(declaration) == (
        "sim_fixture/public_target",
        "sim_fixture/diag_included",
    )
    assert fixture_to_specs(declaration) == specs


# ---------------------------------------------------------------------------
# Canonical status schema: digest agreement, sequence, finite timestamp
# ---------------------------------------------------------------------------


def test_status_carries_canonical_schema_and_digest_agreement() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    status = _ready_status(joint, scenario="qualification-moveit-plan-joint", sequence=3, published_at=42.5)
    assert set(status) == {
        "schema_version",
        "state",
        "scenario",
        "owner",
        "revision",
        "revision_digest",
        "sequence",
        "published_at",
        "owned_ids",
        "target_source_id",
        "target_handoff",
        "fixture_descriptor_sha256",
    }
    assert status["schema_version"] == STATUS_SCHEMA_VERSION == 1
    assert status["state"] == FIXTURE_STATE_READY == "FIXTURE_READY"
    assert status["owner"] == FIXTURE_OWNER == "sim_fixture"
    assert status["scenario"] == "qualification-moveit-plan-joint"
    assert status["revision"] == joint["revision"]
    assert status["revision_digest"] == revision_digest(joint) == joint["revision_digest"]
    assert tuple(status["owned_ids"]) == fixture_owned_ids(joint)
    assert status["target_source_id"] == "sim_fixture/public_target"
    assert status["target_handoff"] == "pick_and_place/object_mesh"
    assert status["fixture_descriptor_sha256"] == fixture_descriptor_sha256(joint)
    assert len(status["fixture_descriptor_sha256"]) == 64
    assert status["sequence"] == 3
    assert math.isfinite(float(status["published_at"]))
    # Canonical compact bytes of the status are stable across equivalent objects.
    assert canonical_json(status) == canonical_json(dict(status))


def test_status_sequence_is_monotonic_and_published_at_finite() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    first = _ready_status(joint, scenario="qualification-moveit-plan-joint", sequence=1, published_at=1.25)
    second = _ready_status(joint, scenario="qualification-moveit-plan-joint", sequence=2, published_at=1.45)
    assert second["sequence"] > first["sequence"]
    assert second["sequence"] == first["sequence"] + 1
    for status in (first, second):
        assert isinstance(status["sequence"], int)
        assert status["sequence"] >= 1
        assert math.isfinite(float(status["published_at"]))


def test_fixture_descriptor_sha256_is_deterministic() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    pose = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-pose.json")
    assert fixture_descriptor_sha256(joint) == fixture_descriptor_sha256(joint)
    assert fixture_descriptor_sha256(pose) != fixture_descriptor_sha256(joint)


# ---------------------------------------------------------------------------
# confirm_fixture_revision: readback / status consistency, fail-closed
# ---------------------------------------------------------------------------


def test_confirm_accepts_consistent_readback() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    owned = fixture_owned_ids(joint)
    status = _ready_status(joint, scenario="qualification-moveit-plan-joint", sequence=5, published_at=42.5)
    confirmation = confirm_fixture_revision(
        service_result=True,
        scene_ids=["sim_fixture/pedestal", "sim_fixture/public_target", "nav/foreign"],
        status=status,
        expected_revision=str(joint["revision"]),
        expected_digest=revision_digest(joint),
        expected_owned_ids=owned,
    )
    assert confirmation.ready is True
    assert confirmation.reasons == ()
    assert confirmation.owned_ids_present is True
    assert confirmation.foreign_fixture_ids == ()
    assert confirmation.status_consistent is True
    assert confirmation.observed_revision == joint["revision"]
    assert confirmation.observed_digest == revision_digest(joint)
    assert confirmation.expected_owned_ids == owned
    assert confirmation.observed_scene_ids == (
        "sim_fixture/pedestal",
        "sim_fixture/public_target",
        "nav/foreign",
    )


def test_confirm_fails_on_missing_owned_id_and_foreign_fixture_id() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    owned = fixture_owned_ids(joint)
    status = _ready_status(joint, scenario="qualification-moveit-plan-joint", sequence=5, published_at=42.5)
    confirmation = confirm_fixture_revision(
        service_result=True,
        scene_ids=["sim_fixture/public_target", "sim_fixture/rogue"],
        status=status,
        expected_revision=str(joint["revision"]),
        expected_digest=revision_digest(joint),
        expected_owned_ids=owned,
    )
    assert confirmation.ready is False
    assert confirmation.owned_ids_present is False
    assert confirmation.foreign_fixture_ids == ("sim_fixture/rogue",)
    assert any("missing" in reason for reason in confirmation.reasons)
    assert any("unexpected" in reason for reason in confirmation.reasons)


def test_confirm_fails_closed_on_service_failure() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    owned = fixture_owned_ids(joint)
    status = _ready_status(joint, scenario="qualification-moveit-plan-joint", sequence=5, published_at=42.5)
    confirmation = confirm_fixture_revision(
        service_result=False,
        scene_ids=list(owned),
        status=status,
        expected_revision=str(joint["revision"]),
        expected_digest=revision_digest(joint),
        expected_owned_ids=owned,
    )
    assert confirmation.ready is False
    assert any("service" in reason for reason in confirmation.reasons)


def test_confirm_rejects_inconsistent_status() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    owned = fixture_owned_ids(joint)
    status = _ready_status(joint, scenario="qualification-moveit-plan-joint", sequence=5, published_at=42.5)
    bad = dict(status)
    bad["revision"] = "wrong-revision"
    confirmation = confirm_fixture_revision(
        service_result=True,
        scene_ids=list(owned),
        status=bad,
        expected_revision=str(joint["revision"]),
        expected_digest=revision_digest(joint),
        expected_owned_ids=owned,
    )
    assert confirmation.ready is False
    assert confirmation.status_consistent is False
    assert any("status inconsistent" in reason for reason in confirmation.reasons)


def test_confirm_rejects_non_ready_state_and_bad_fields() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    owned = fixture_owned_ids(joint)
    status = canonical_fixture_status(
        scenario="qualification-moveit-plan-joint",
        revision=str(joint["revision"]),
        revision_digest=revision_digest(joint),
        sequence=1,
        published_at=float("nan"),
        owned_ids=owned,
        target_source_id=str(joint["target_source_id"]),
        target_handoff=str(joint["target_handoff"]),
        descriptor_sha256=fixture_descriptor_sha256(joint),
        state="FIXTURE_PENDING",
    )
    confirmation = confirm_fixture_revision(
        service_result=True,
        scene_ids=list(owned),
        status=status,
        expected_revision=str(joint["revision"]),
        expected_digest=revision_digest(joint),
        expected_owned_ids=owned,
    )
    assert confirmation.ready is False
    assert confirmation.status_consistent is False
    assert any("state" in reason for reason in confirmation.reasons)
    assert any("published_at" in reason for reason in confirmation.reasons)


# ---------------------------------------------------------------------------
# Scenario schema strict validation (planning_scene, schema version 2)
# ---------------------------------------------------------------------------


def test_scenario_definition_loads_planning_scene_schema_v2() -> None:
    from tinker_sim_core.scenario import load_named_scenario

    scenario = load_named_scenario(ROOT, "qualification-moveit-plan-pose")
    assert scenario.schema_version == 2
    assert scenario.planning_scene is not None
    assert scenario.planning_scene["revision"]
    assert scenario.planning_scene["frame_id"] == "base_link"
    assert scenario.planning_scene["target_source_id"] == "sim_fixture/public_target"
    assert scenario.planning_scene["target_handoff"] == "pick_and_place/object_mesh"


def test_scenario_validation_rejects_bad_planning_scene() -> None:
    import hashlib
    import tempfile

    from tinker_sim_core.scenario import ScenarioDefinition

    base: Mapping[str, object] = {
        "schema_version": 2,
        "id": "qualification-moveit-plan-joint",
        "world": {"mode": "current"},
        "robot": {"id": "tinker2", "initial_pose": [0.0, 0.0, 0.0]},
        "actors": [],
        "objects": [],
        "events": [{"at_sim_time": 0.0, "event": "spawn_once_while_paused"}],
        "postconditions": [{"name": "ready", "path": "x", "operator": "equals", "value": True}],
    }

    def with_planning_scene(ps: object) -> Mapping[str, object]:
        payload = dict(base)
        payload["planning_scene"] = ps
        return payload

    def load_scenario(payload: Mapping[str, object]) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "scenario.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            ScenarioDefinition.load(path)

    def fresh(ps: Mapping[str, object]) -> Mapping[str, object]:
        """Copy a planning scene and recompute its canonical digest."""
        result = dict(ps)
        result["revision_digest"] = hashlib.sha256(
            json.dumps(
                {k: v for k, v in ps.items() if k != "revision_digest"},
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        return result

    valid_ps: Mapping[str, object] = fresh(
        {
            "revision": "r-1",
            "revision_digest": "",
            "frame_id": "base_link",
            "target_source_id": "sim_fixture/public_target",
            "target_handoff": "pick_and_place/object_mesh",
            "objects": [
                {
                    "id": "sim_fixture/public_target",
                    "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
                    "pose": {"xyz": [0.5, 0.0, 0.9], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                }
            ],
        }
    )
    load_scenario(with_planning_scene(valid_ps))

    # Nonempty revision.
    bad = dict(valid_ps)
    bad["revision"] = ""
    bad = fresh(bad)
    with pytest.raises(ValueError, match="revision"):
        load_scenario(with_planning_scene(bad))
    # Canonical digest input must match (do not recompute: intentionally stale).
    bad = dict(valid_ps)
    bad["revision_digest"] = "0" * 64
    with pytest.raises(ValueError, match="digest"):
        load_scenario(with_planning_scene(bad))
    # frame must be base_link.
    bad = dict(valid_ps)
    bad["frame_id"] = "map"
    bad = fresh(bad)
    with pytest.raises(ValueError, match="base_link"):
        load_scenario(with_planning_scene(bad))
    # target_source_id must be declared and target_handoff exact.
    bad = dict(valid_ps)
    bad["target_source_id"] = "sim_fixture/missing"
    bad = fresh(bad)
    with pytest.raises(ValueError, match="target_source_id"):
        load_scenario(with_planning_scene(bad))
    bad = dict(valid_ps)
    bad["target_handoff"] = "somewhere/else"
    bad = fresh(bad)
    with pytest.raises(ValueError, match="target_handoff"):
        load_scenario(with_planning_scene(bad))
    # Unique sim_fixture/* ids.
    dup = fresh(
        {
            "revision": "r-1",
            "revision_digest": "",
            "frame_id": "base_link",
            "target_source_id": "sim_fixture/a",
            "target_handoff": "pick_and_place/object_mesh",
            "objects": [
                {
                    "id": "sim_fixture/a",
                    "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
                    "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                },
                {
                    "id": "sim_fixture/a",
                    "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
                    "pose": {"xyz": [0.2, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                },
            ],
        }
    )
    with pytest.raises(ValueError, match="unique"):
        load_scenario(with_planning_scene(dup))
    # Non-fixture-prefixed id.
    bad = dict(valid_ps)
    bad["objects"] = [
        {
            "id": "nav/foreign",
            "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
            "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        }
    ]
    bad = fresh(bad)
    with pytest.raises(ValueError, match="sim_fixture"):
        load_scenario(with_planning_scene(bad))
    # Non-positive primitive dimensions.
    bad = dict(valid_ps)
    bad["objects"] = [
        {
            "id": "sim_fixture/public_target",
            "primitive": {"type": "box", "dimensions": [0.0, 0.1, 0.1]},
            "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        }
    ]
    bad = fresh(bad)
    with pytest.raises(ValueError, match="positive"):
        load_scenario(with_planning_scene(bad))
    # Diagnostic must declare enter_collision_bodies explicitly.
    diag = dict(valid_ps)
    diag["diagnostic_objects"] = [
        {
            "id": "sim_fixture/diag",
            "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
            "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        }
    ]
    diag = fresh(diag)
    with pytest.raises(ValueError, match="enter_collision_bodies"):
        load_scenario(with_planning_scene(diag))
    # Hashed mesh asset must exist with matching content (real temp file).
    with tempfile.TemporaryDirectory() as tmp_mesh_dir:
        mesh_abs = Path(tmp_mesh_dir) / "table.stl"
        mesh_abs.write_bytes(_tiny_binary_stl())
        mesh_digest = hashlib.sha256(mesh_abs.read_bytes()).hexdigest()
        mesh = dict(valid_ps)
        mesh["objects"] = [
            {
                "id": "sim_fixture/table",
                "mesh": {"uri": "table.stl", "sha256": mesh_digest},
                "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            }
        ]
        mesh["target_source_id"] = "sim_fixture/table"
        scenario_json = Path(tmp_mesh_dir) / "scenario.json"
        scenario_json.write_text(
            json.dumps(with_planning_scene(fresh(mesh))), encoding="utf-8"
        )
        ScenarioDefinition.load(scenario_json)
        # Content-hash mismatch against the actual file bytes.
        bad_mesh = dict(mesh)
        bad_mesh["objects"] = [
            {
                "id": "sim_fixture/table",
                "mesh": {"uri": "table.stl", "sha256": "a" * 64},
                "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            }
        ]
        scenario_json.write_text(
            json.dumps(with_planning_scene(fresh(bad_mesh))), encoding="utf-8"
        )
        with pytest.raises(ValueError, match="sha256 mismatch"):
            ScenarioDefinition.load(scenario_json)


# ---------------------------------------------------------------------------
# Mesh parsing / asset loading (pure, ROS-free)
# ---------------------------------------------------------------------------


def test_parse_binary_stl() -> None:
    vertices, triangles = parse_mesh_bytes(_tiny_binary_stl(), filename="box.stl")
    assert len(vertices) == 3
    assert triangles == ((0, 1, 2),)
    assert all(all(math.isfinite(coord) for coord in vertex) for vertex in vertices)


def test_parse_ascii_stl() -> None:
    vertices, triangles = parse_mesh_bytes(_tiny_ascii_stl(), filename="box.stl")
    assert len(vertices) == 3
    assert triangles == ((0, 1, 2),)
    assert vertices[1] == (1.0, 0.0, 0.0)
    assert vertices[2] == (0.0, 1.0, 0.0)


def test_parse_obj() -> None:
    vertices, triangles = parse_mesh_bytes(_tiny_obj(), filename="box.obj")
    assert len(vertices) == 3
    assert triangles == ((0, 1, 2),)
    assert vertices[0] == (0.0, 0.0, 0.0)


def test_parse_obj_with_vertex_texcoord_normal_indices() -> None:
    data = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1/1/1 2/2/2 3/3/3\n"
    vertices, triangles = parse_mesh_bytes(data, filename="box.obj")
    assert triangles == ((0, 1, 2),)


def test_parse_mesh_rejects_empty_geometry() -> None:
    with pytest.raises(Exception, match="vertex"):
        parse_mesh_bytes(b"solid empty\nendsolid empty\n", filename="empty.stl")
    with pytest.raises(Exception, match="vertex"):
        parse_mesh_bytes(b"", filename="empty.obj")


def test_parse_mesh_rejects_nonfinite_vertex() -> None:
    data = _tiny_ascii_stl().replace(b"1 0 0", b"nan 0 0")
    with pytest.raises(Exception, match="finite"):
        parse_mesh_bytes(data, filename="bad.stl")


def test_parse_mesh_rejects_degenerate_triangle() -> None:
    data = (
        b"solid deg\nfacet normal 0 0 1\nouter loop\n"
        b"vertex 0 0 0\nvertex 0 0 0\nvertex 0 0 0\n"
        b"endloop\nendfacet\nendsolid deg\n"
    )
    with pytest.raises(Exception, match="nondegenerate"):
        parse_mesh_bytes(data, filename="deg.stl")


def test_parse_ascii_stl_rejects_over_vertex_facet() -> None:
    data = (
        b"solid over\nfacet normal 0 0 1\nouter loop\n"
        b"vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nvertex 1 1 0\n"
        b"endloop\nendfacet\nendsolid over\n"
    )
    with pytest.raises(Exception, match="exactly 3"):
        parse_mesh_bytes(data, filename="over.stl")


def test_parse_ascii_stl_rejects_under_vertex_facet() -> None:
    data = (
        b"solid short\nfacet normal 0 0 1\nouter loop\n"
        b"vertex 0 0 0\nvertex 1 0 0\n"
        b"endloop\nendfacet\nendsolid short\n"
    )
    with pytest.raises(Exception, match="exactly 3"):
        parse_mesh_bytes(data, filename="short.stl")


def test_parse_ascii_stl_rejects_vertex_outside_facet() -> None:
    data = (
        b"solid loose\nvertex 0 0 0\nfacet normal 0 0 1\nouter loop\n"
        b"vertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid loose\n"
    )
    with pytest.raises(Exception, match="outside a facet"):
        parse_mesh_bytes(data, filename="loose.stl")


def test_parse_ascii_stl_rejects_unterminated_facet() -> None:
    data = (
        b"solid unter\nfacet normal 0 0 1\nouter loop\n"
        b"vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\n"
    )
    with pytest.raises(Exception, match="unterminated"):
        parse_mesh_bytes(data, filename="unterminated.stl")


def test_parse_ascii_stl_respects_facet_boundaries() -> None:
    # Two well-formed facets with distinct vertices produce two triangles in
    # declared order, not an arbitrary regrouping of vertex lines.
    data = (
        b"solid two\n"
        b"facet normal 0 0 1\nouter loop\n"
        b"vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\n"
        b"facet normal 0 0 1\nouter loop\n"
        b"vertex 0 0 1\nvertex 1 0 1\nvertex 0 1 1\nendloop\nendfacet\n"
        b"endsolid two\n"
    )
    vertices, triangles = parse_mesh_bytes(data, filename="two.stl")
    assert len(vertices) == 6
    assert triangles == ((0, 1, 2), (3, 4, 5))


def test_parse_mesh_rejects_out_of_range_index() -> None:
    data = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n"
    with pytest.raises(Exception, match="out of range"):
        parse_mesh_bytes(data, filename="bad.obj")


def test_parse_mesh_rejects_unsupported_format() -> None:
    with pytest.raises(Exception, match="unsupported"):
        parse_mesh_bytes(b"garbage", filename="mesh.xyz")


def test_load_mesh_asset_roundtrip(tmp_path: Path) -> None:
    asset = tmp_path / "box.stl"
    asset.write_bytes(_tiny_binary_stl())
    digest = sha256(asset.read_bytes()).hexdigest()
    mesh = {"uri": "box.stl", "sha256": digest, "scale": [1.0, 1.0, 1.0]}
    vertices, triangles = load_mesh_asset(mesh, project_root=tmp_path)
    assert len(vertices) == 3
    assert triangles == ((0, 1, 2),)


def test_load_mesh_asset_scale_applied(tmp_path: Path) -> None:
    asset = tmp_path / "box.stl"
    asset.write_bytes(_tiny_binary_stl())
    digest = sha256(asset.read_bytes()).hexdigest()
    mesh = {"uri": "box.stl", "sha256": digest, "scale": [2.0, 3.0, 4.0]}
    vertices, _ = load_mesh_asset(mesh, project_root=tmp_path)
    assert vertices[1] == (2.0, 0.0, 0.0)
    assert vertices[2] == (0.0, 3.0, 0.0)


def test_load_mesh_asset_hash_mismatch(tmp_path: Path) -> None:
    asset = tmp_path / "box.stl"
    asset.write_bytes(_tiny_binary_stl())
    mesh = {"uri": "box.stl", "sha256": "a" * 64}
    with pytest.raises(Exception, match="sha256 mismatch"):
        load_mesh_asset(mesh, project_root=tmp_path)


def test_load_mesh_asset_missing_file(tmp_path: Path) -> None:
    mesh = {"uri": "missing.stl", "sha256": "a" * 64}
    with pytest.raises(Exception, match="not found"):
        load_mesh_asset(mesh, project_root=tmp_path)


def test_load_mesh_asset_unsupported_format(tmp_path: Path) -> None:
    asset = tmp_path / "box.xyz"
    asset.write_bytes(b"garbage")
    digest = sha256(b"garbage").hexdigest()
    mesh = {"uri": "box.xyz", "sha256": digest}
    with pytest.raises(Exception, match="unsupported"):
        load_mesh_asset(mesh, project_root=tmp_path)


def test_supported_mesh_extensions_documented() -> None:
    assert set(SUPPORTED_MESH_EXTENSIONS) == {".stl", ".obj"}


def test_geometry_signature_sha256_deterministic() -> None:
    first = [
        {
            "id": "sim_fixture/a",
            "frame_id": "base_link",
            "primitives": [{"type": "box", "dimensions": [1.0, 2.0, 3.0]}],
        }
    ]
    assert geometry_signature_sha256(first) == geometry_signature_sha256(list(first))
    second = [
        {
            "id": "sim_fixture/a",
            "frame_id": "base_link",
            "primitives": [{"type": "box", "dimensions": [1.0, 2.0, 4.0]}],
        }
    ]
    assert geometry_signature_sha256(first) != geometry_signature_sha256(second)


def test_touch_links_alias_equals_validated_model_contract() -> None:
    from tinker_sim_bridge.fixture_contract import MODEL_CONTRACT_TOUCH_LINKS
    from tinker_sim_bridge.model_contract import TOUCH_LINKS

    assert MODEL_CONTRACT_TOUCH_LINKS == TOUCH_LINKS
    assert len(MODEL_CONTRACT_TOUCH_LINKS) == 8


def test_real_model_bundle_touch_links_match_exported_fixture_set() -> None:
    """Prove the real/current model-bundle contract's exact eight touch links
    equal the exported fixture/handoff set (must run when artifacts are
    provisioned; explicit skip only when the artifact tree is absent)."""
    import json as _json

    from tinker_sim_bridge.current_artifact import resolve_current_artifact
    from tinker_sim_bridge.fixture_contract import MODEL_CONTRACT_TOUCH_LINKS
    from tinker_sim_bridge.model_bundle import build_manifest

    try:
        resolve_current_artifact(ROOT)
    except Exception as exc:  # noqa: BLE001 - artifacts absent
        pytest.skip("real Tinker 2 artifact is not provisioned: {}".format(exc))
    manifest_path = ROOT / "outputs/ompl-overlay/model-bundle/model-bundle.json"
    if not manifest_path.is_file():
        pytest.skip("real model-bundle manifest is not provisioned: {}".format(manifest_path))
    manifest = _json.loads(manifest_path.read_text(encoding="utf-8"))
    artifacts = manifest["artifacts"]
    paths = {name: entry["path"] for name, entry in artifacts.items()}
    if any(not Path(path).is_file() for path in paths.values()):
        pytest.skip("real model-bundle artifact files are not provisioned")
    rebuilt = build_manifest(
        simulator_full_urdf=paths["simulator_full_urdf"],
        planning_urdf=paths["planning_urdf"],
        planning_srdf=paths["planning_srdf"],
        joint_limits=paths["joint_limits"],
        kinematics=paths["kinematics"],
        prefix=manifest["normalization"]["prefix"],
        mount=manifest["normalization"]["mount"],
    )
    assert tuple(rebuilt["contract"]["touch_links"]) == tuple(MODEL_CONTRACT_TOUCH_LINKS)


def test_scenario_validation_rejects_target_diagnostic_excluded_from_collision() -> None:
    """target_source_id naming a diagnostic with enter_collision_bodies=false is
    rejected at schema validation (it never enters the owned collision set)."""
    import tempfile

    from tinker_sim_core.scenario import ScenarioDefinition

    def with_digest(ps: Mapping[str, object]) -> dict[str, object]:
        ps = dict(ps)
        ps["revision_digest"] = sha256(
            json.dumps(
                {k: v for k, v in ps.items() if k != "revision_digest"},
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        return ps

    payload = {
        "schema_version": 2,
        "id": "qualification-target-diag",
        "world": {"mode": "current"},
        "robot": {"id": "tinker2", "initial_pose": [0.0, 0.0, 0.0]},
        "actors": [],
        "objects": [
            {
                "id": "sim_fixture/a",
                "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
                "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            }
        ],
        "diagnostic_objects": [
            {
                "id": "sim_fixture/diag",
                "enter_collision_bodies": False,
                "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
                "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            }
        ],
        "events": [{"at_sim_time": 0.0, "event": "spawn_once_while_paused"}],
        "postconditions": [{"name": "ready", "path": "x", "operator": "equals", "value": True}],
        "planning_scene": with_digest(
            {
                "revision": "r-1",
                "frame_id": "base_link",
                "target_source_id": "sim_fixture/diag",
                "target_handoff": "pick_and_place/object_mesh",
                "objects": [
                    {
                        "id": "sim_fixture/a",
                        "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
                        "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                    }
                ],
                "diagnostic_objects": [
                    {
                        "id": "sim_fixture/diag",
                        "enter_collision_bodies": False,
                        "primitive": {"type": "box", "dimensions": [0.1, 0.1, 0.1]},
                        "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                    }
                ],
            }
        ),
    }
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "scenario.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        with pytest.raises(ValueError, match="collision-body set"):
            ScenarioDefinition.load(path)
    # A diagnostic marked enter_collision_bodies: true IS a valid target.
    payload["planning_scene"] = with_digest(dict(payload["planning_scene"]))
    payload["planning_scene"]["diagnostic_objects"] = [
        dict(record, enter_collision_bodies=True)
        for record in payload["planning_scene"]["diagnostic_objects"]
    ]
    payload["planning_scene"] = with_digest(payload["planning_scene"])
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "scenario.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        ScenarioDefinition.load(path)


def test_scenario_validation_mesh_supported_format_and_rejection(tmp_path: Path) -> None:
    """Mesh fixtures require an existing supported-format asset whose content
    matches the declared digest; missing/unsupported assets are rejected."""
    from tinker_sim_core.scenario import ScenarioDefinition

    stl = tmp_path / "box.stl"
    stl.write_bytes(_tiny_binary_stl())
    stl_digest = sha256(stl.read_bytes()).hexdigest()
    scenario_path = tmp_path / "scenario.json"
    base_ps = {
        "revision": "r-1",
        "frame_id": "base_link",
        "target_source_id": "sim_fixture/table",
        "target_handoff": "pick_and_place/object_mesh",
    }

    def write(mesh: Mapping[str, object]) -> None:
        ps = dict(base_ps)
        ps["objects"] = [
            {
                "id": "sim_fixture/table",
                "mesh": dict(mesh),
                "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            }
        ]
        ps["revision_digest"] = sha256(
            json.dumps(
                {k: v for k, v in ps.items() if k != "revision_digest"},
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        payload = {
            "schema_version": 2,
            "id": "mesh-scenario",
            "world": {"mode": "current"},
            "robot": {"id": "tinker2", "initial_pose": [0.0, 0.0, 0.0]},
            "actors": [],
            "objects": [],
            "events": [{"at_sim_time": 0.0, "event": "spawn_once_while_paused"}],
            "postconditions": [{"name": "ready", "path": "x", "operator": "equals", "value": True}],
            "planning_scene": ps,
        }
        scenario_path.write_text(json.dumps(payload), encoding="utf-8")
        return scenario_path

    ScenarioDefinition.load(write({"uri": "box.stl", "sha256": stl_digest}))
    with pytest.raises(ValueError, match="not found"):
        ScenarioDefinition.load(write({"uri": "missing.stl", "sha256": "a" * 64}))
    bad_digest = {"uri": "box.stl", "sha256": "a" * 64}
    with pytest.raises(ValueError, match="sha256 mismatch"):
        ScenarioDefinition.load(write(bad_digest))
    xyz = tmp_path / "box.xyz"
    xyz.write_bytes(b"garbage")
    xyz_digest = sha256(b"garbage").hexdigest()
    with pytest.raises(ValueError, match="not supported"):
        ScenarioDefinition.load(write({"uri": "box.xyz", "sha256": xyz_digest}))


# ---------------------------------------------------------------------------
# Humble node behavior (requires ROS runtime; each test skips under the venv)
# ---------------------------------------------------------------------------


def _humble():
    """Importorskip guards kept inside each ROS test so pure tests still run."""
    pytest.importorskip(
        "rclpy", reason="fixture planning scene node tests require the Humble ROS runtime"
    )
    pytest.importorskip(
        "moveit_msgs", reason="fixture planning scene node tests require moveit_msgs"
    )


def _fake_clock(nanoseconds: int = 1_500_000_000):
    class _Now:
        pass

    _Now.nanoseconds = nanoseconds

    class _Clock:
        def now(self) -> _Now:
            return _Now()

    return _Clock()


def _make_node() -> "object":
    from tinker_sim_bridge.fixture_planning_scene_node import FixturePlanningScene

    node = FixturePlanningScene.__new__(FixturePlanningScene)
    node._scenario_id = "qualification-moveit-plan-joint"
    node._revision = "r-1"
    node._revision_digest = "a" * 64
    node._owned_ids = ("sim_fixture/pedestal", "sim_fixture/public_target")
    node._target_source_id = "sim_fixture/public_target"
    node._target_handoff = "pick_and_place/object_mesh"
    node._descriptor_sha256 = "b" * 64
    node._phase = "ready"
    node._state = FIXTURE_STATE_READY
    node._fail_reason = None
    node._sequence = 0
    node._clock = _fake_clock()
    node._last_status = None
    node._load_mesh = None
    node._project_root = None
    node._scene_objects = ()
    published: list[str] = []
    node._publisher = type("Pub", (), {})()
    node._publisher.publish = lambda message: published.append(message.data)
    node._published = published
    logger = type("Logger", (), {})()
    logger.warning = lambda msg: None
    logger.error = lambda msg: None
    node.get_logger = lambda: logger
    return node


def test_ready_node_publishes_monotonic_finite_heartbeat() -> None:
    _humble()
    node = _make_node()
    node._publish_heartbeat()
    node._publish_heartbeat()
    payloads = [json.loads(data) for data in node._published]
    assert len(payloads) == 2
    assert payloads[0]["sequence"] == 1
    assert payloads[1]["sequence"] == 2
    assert payloads[1]["sequence"] > payloads[0]["sequence"]
    for payload in payloads:
        assert payload["schema_version"] == STATUS_SCHEMA_VERSION
        assert payload["state"] == FIXTURE_STATE_READY
        assert payload["owner"] == "sim_fixture"
        assert payload["target_handoff"] == "pick_and_place/object_mesh"
        assert payload["revision_digest"] == node._revision_digest
        assert tuple(payload["owned_ids"]) == node._owned_ids
        assert math.isfinite(float(payload["published_at"]))
        assert len(payload["fixture_descriptor_sha256"]) == 64


def test_ready_service_fails_closed_before_ready_phase() -> None:
    _humble()
    from std_srvs.srv import Trigger

    node = _make_node()
    node._phase = "physics"
    node._state = "FIXTURE_PENDING"
    node._fail_reason = None
    response = Trigger.Response()
    node._on_ready(Trigger.Request(), response)
    assert response.success is False
    payload = json.loads(response.message)
    assert payload["state"] == "FIXTURE_PENDING"

    node._phase = "ready"
    node._state = FIXTURE_STATE_READY
    response = Trigger.Response()
    node._on_ready(Trigger.Request(), response)
    assert response.success is True
    assert json.loads(response.message)["state"] == FIXTURE_STATE_READY


def test_apply_failure_fails_closed_and_never_serves_ready() -> None:
    _humble()
    from tinker_sim_bridge.fixture_planning_scene_node import FixturePlanningScene

    node = _make_node()
    node._phase = "apply"
    node._state = FIXTURE_STATE_PENDING
    node._apply_state = {
        "client": None, "future": None, "error": None, "pending": None,
        "succeeded": False, "result": None,
    }
    node._apply_state["error"] = "service call failed: connection refused"
    node._advance_apply(time.monotonic())
    assert node._phase == "failed"
    assert node._state == "FIXTURE_FAILED"
    assert node._fail_reason is not None

    from std_srvs.srv import Trigger

    response = Trigger.Response()
    node._on_ready(Trigger.Request(), response)
    assert response.success is False


def test_malformed_readback_fails_closed() -> None:
    _humble()
    from tinker_sim_bridge.fixture_planning_scene_node import FixturePlanningScene

    node = _make_node()
    # A get response with no world/collision_objects must be rejected as malformed.
    assert node._get_extract(None) is None


def test_exactly_one_apply_request_carries_atomic_add_and_remove() -> None:
    _humble()

    from tinker_sim_bridge.fixture_planning_scene_node import FixturePlanningScene
    from tinker_sim_bridge.fixture_planning_scene import fixture_to_specs
    from tinker_sim_bridge.fixture_contract import build_atomic_revision_diff

    node = _make_node()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-pose.json")
    node._specs = fixture_to_specs(declaration)
    # One stale sim_fixture id to remove; the foreign id is preserved.
    node._existing_ids = ("sim_fixture/stale_removed", "nav/foreign_kept")
    node._diff_plan = build_atomic_revision_diff(
        desired_objects=node._specs, existing_ids=node._existing_ids
    )
    node._phase = "apply"
    node._apply_state = {
        "client": None, "future": None, "error": None, "pending": None,
        "succeeded": False, "result": None,
    }
    node._service_group = object()
    node.create_client = lambda *a, **k: None

    # The request builder must produce exactly one MoveIt request containing
    # every desired ADD plus the stale REMOVE, with no foreign id.
    from moveit_msgs.srv import ApplyPlanningScene

    client = type("Client", (), {})()
    client.srv_type = ApplyPlanningScene
    request = node._build_apply_request(client)
    assert request.scene.is_diff is True
    collision_objects = request.scene.world.collision_objects
    assert [obj.id for obj in collision_objects] == [
        "sim_fixture/pedestal",
        "sim_fixture/public_target",
        "sim_fixture/stale_removed",
    ]
    assert [obj.operation for obj in collision_objects] == [
        b"\x00", b"\x00", b"\x01",
    ]
    assert all(obj.header.frame_id == "base_link" for obj in collision_objects)
    assert all("nav/" not in obj.id for obj in collision_objects)
    # Exactly one atomic request: a second build is byte-identical.
    again = node._build_apply_request(client)
    assert [obj.id for obj in again.scene.world.collision_objects] == [
        obj.id for obj in collision_objects
    ]


def test_apply_timeout_fails_closed() -> None:
    _humble()

    node = _make_node()
    node._phase = "apply"
    node._state = FIXTURE_STATE_PENDING
    node._apply_state = {
        "client": None, "future": None, "error": None, "pending": None,
        "succeeded": False, "result": None,
    }
    node._start_deadline_s = 0.0
    node._phase_started_at = time.monotonic() - 60.0
    node._advance_apply(time.monotonic())
    assert node._phase == "failed"
    assert node._state == FIXTURE_STATE_FAILED
    assert "timed out" in str(node._fail_reason)


# ---------------------------------------------------------------------------
# ROS-free apply-timeout seam (rclpy import stub; runs under the sim venv)
# ---------------------------------------------------------------------------


def _stub_rclpy(monkeypatch) -> None:
    """Provide importable rclpy stand-ins so the node module can be imported
    without a Humble runtime (the sim venv lacks the python3.12 ``_rclpy`` C
    extension).  Only the symbols bound at module import time are provided;
    the test never calls into rclpy itself."""
    import types

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.ok = lambda *a, **k: True
    rclpy.shutdown = lambda *a, **k: None
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)

    callback_groups = types.ModuleType("rclpy.callback_groups")
    callback_groups.ReentrantCallbackGroup = object
    monkeypatch.setitem(sys.modules, "rclpy.callback_groups", callback_groups)

    node = types.ModuleType("rclpy.node")
    node.Node = object
    monkeypatch.setitem(sys.modules, "rclpy.node", node)

    qos = types.ModuleType("rclpy.qos")
    qos.DurabilityPolicy = type("DurabilityPolicy", (), {})
    qos.QoSProfile = object
    qos.ReliabilityPolicy = object
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos)


def test_apply_request_in_flight_tolerates_startup_safe_delay(monkeypatch) -> None:
    """The ApplyPlanningScene request must not be abandoned by the hard 5.0 s
    request timeout while MoveGroup is still initializing: the planning-scene
    service timeout must tolerate the equivalent startup delay (20.0 s).

    Regression for the live blocked scenario where the apply step failed with
    ``service request timed out after 5.0 s`` while MoveGroup initialized.
    Runs ROS-free under the sim venv via a minimal rclpy import stub.
    """
    _stub_rclpy(monkeypatch)
    try:
        node = _make_node()
        node._phase = "apply"
        node._state = FIXTURE_STATE_PENDING
        node._fail_reason = None
        node._apply_state = {
            "client": None, "future": None, "error": None, "pending": None,
            "succeeded": False, "result": None,
        }
        node._start_deadline_s = 1000.0
        node._phase_started_at = 0.0
        node._diff_plan = object()
        node._service_group = None

        class _Future:
            def done(self) -> bool:
                return False

        class _Client:
            def service_is_ready(self) -> bool:
                return True

            def call_async(self, request):
                return _Future()

        node.create_client = lambda *a, **k: _Client()
        node._build_apply_request = lambda client: object()

        # First tick at t=100.0: the service is ready and the request is sent.
        node._advance_apply(100.0)
        assert node._phase == "apply"
        assert node._apply_state["started_at"] == 100.0

        # Second tick at t=120.0: the same request has been in flight for the
        # full 20.0 s MoveGroup startup window.  A startup-safe planning-scene
        # service timeout (20.0 s) must keep waiting; the hard 5.0 s timeout
        # fails the node here.
        node._advance_apply(120.0)
        assert node._phase == "apply"
        assert node._state == FIXTURE_STATE_PENDING
        assert node._apply_state["pending"] == "request in flight"
        assert node._apply_state["error"] is None
        assert node._fail_reason is None
    finally:
        sys.modules.pop("tinker_sim_bridge.fixture_planning_scene_node", None)


def test_apply_service_timeout_retries_transiently(monkeypatch) -> None:
    """RED: a planning-scene service *timeout* must be transient, not fatal.

    Live joint scenario (2026-08-07T17:17): on the cold start the
    /get_planning_scene response was dropped server-side (move_group
    ``failed to send response ... timeout``), so the fixture node's single
    20 s attempt timed out and the whole Humble stack shut down.  The response
    is lost, not refused — a fresh request succeeds (proven by the warm pose
    scenario).  A timeout must therefore be retried within the per-phase
    120 s budget, not fail the node.
    """
    _stub_rclpy(monkeypatch)
    try:
        node = _make_node()
        node._phase = "apply"
        node._state = FIXTURE_STATE_PENDING
        node._fail_reason = None
        node._apply_state = {
            "client": None, "future": None, "error": None, "pending": None,
            "succeeded": False, "result": None,
        }
        node._start_deadline_s = 120.0
        node._phase_started_at = 0.0
        node._diff_plan = object()
        node._service_group = None

        class _Future:
            def done(self) -> bool:
                return False

        class _Client:
            calls = 0

            def service_is_ready(self) -> bool:
                return True

            def call_async(self, request):
                _Client.calls += 1
                return _Future()

        node.create_client = lambda *a, **k: _Client()
        node._build_apply_request = lambda client: object()

        # First tick: request sent at t=0.
        node._advance_apply(0.0)
        assert node._phase == "apply"
        assert node._apply_state["started_at"] == 0.0

        # Second tick past the 20 s service timeout: the request is abandoned
        # (client reset + error set), but the node must NOT fail — it clears the
        # timeout as transient and stays in the apply phase for a fresh attempt.
        node._advance_apply(100.0)
        assert node._phase == "apply", (
            "a service timeout must retry, not fail: phase=%s" % node._phase
        )
        assert node._state == FIXTURE_STATE_PENDING
        assert node._fail_reason is None
        assert node._apply_state["error"] is None, (
            "timeout error must be cleared for retry: %s" % node._apply_state["error"]
        )

        # Third tick: the fresh client is re-created and a new request issued.
        node._advance_apply(100.05)
        assert node._phase == "apply"
        assert node._apply_state["error"] is None
        assert _Client.calls >= 2, (
            "a fresh request must be re-issued after the timeout (calls=%d)"
            % _Client.calls
        )
    finally:
        sys.modules.pop("tinker_sim_bridge.fixture_planning_scene_node", None)


def test_physics_gate_timeout_fails_closed() -> None:
    _humble()

    node = _make_node()
    node._phase = "physics"
    node._state = FIXTURE_STATE_PENDING
    node._physics_state = {
        "client": None, "future": None, "error": None, "pending": None,
        "succeeded": False, "result": None,
    }
    node._start_deadline_s = 0.0
    node._phase_started_at = time.monotonic() - 60.0
    node._advance_physics(time.monotonic())
    assert node._phase == "failed"
    assert node._state == FIXTURE_STATE_FAILED
    assert "physics" in str(node._fail_reason)


# ---------------------------------------------------------------------------
# Geometry readback confirmation (Humble messages)
# ---------------------------------------------------------------------------


def _confirm_with_geometry(
    declaration: Mapping[str, object],
    objects,
    *,
    mesh_loader=None,
    canonical_frame_id: str | None = None,
) -> "object":
    from tinker_sim_bridge.fixture_contract import confirm_fixture_revision, spec_geometry
    from tinker_sim_bridge.fixture_planning_scene import (
        canonical_fixture_status,
        fixture_descriptor_sha256,
        fixture_owned_ids,
        fixture_to_specs,
    )

    specs = fixture_to_specs(declaration)
    expected_geometry = {
        spec.id: spec_geometry(spec, resolve_mesh=mesh_loader) for spec in specs
    }
    observed_geometry = [
        readback_geometry(obj, canonical_frame_id=canonical_frame_id)
        for obj in objects
    ]
    status = canonical_fixture_status(
        scenario="qualification-geometry",
        revision=str(declaration["revision"]),
        revision_digest=revision_digest(declaration),
        sequence=1,
        published_at=1.0,
        owned_ids=fixture_owned_ids(declaration),
        target_source_id=str(declaration["target_source_id"]),
        target_handoff=str(declaration["target_handoff"]),
        descriptor_sha256=fixture_descriptor_sha256(declaration),
        state=FIXTURE_STATE_READY,
    )
    return confirm_fixture_revision(
        service_result=True,
        scene_ids=[obj.id for obj in objects],
        status=status,
        expected_revision=str(declaration["revision"]),
        expected_digest=revision_digest(declaration),
        expected_owned_ids=fixture_owned_ids(declaration),
        expected_geometry=expected_geometry,
        observed_geometry=observed_geometry,
    )


def _geometry_objects(declaration: Mapping[str, object], *, mesh_loader=None):
    from tinker_sim_bridge.fixture_planning_scene import fixture_to_specs
    from tinker_sim_bridge.fixture_planning_scene_node import _spec_to_collision_object

    specs = fixture_to_specs(declaration)
    return [_spec_to_collision_object(spec, mesh_loader=mesh_loader) for spec in specs]


def test_confirm_geometry_roundtrip_is_ready() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    confirmation = _confirm_with_geometry(declaration, _geometry_objects(declaration))
    assert confirmation.ready, confirmation.reasons
    assert confirmation.geometry_consistent
    assert confirmation.geometry_reasons == ()


def test_confirm_geometry_accepts_moveit_world_object_pose_normalization() -> None:
    _humble()
    from geometry_msgs.msg import Pose

    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    objects = _geometry_objects(declaration)
    for obj in objects:
        shape_pose = obj.primitive_poses[0]
        obj.header.frame_id = "world"
        obj.pose.position.x = shape_pose.position.x
        obj.pose.position.y = shape_pose.position.y
        obj.pose.position.z = shape_pose.position.z
        obj.pose.orientation.x = shape_pose.orientation.x
        obj.pose.orientation.y = shape_pose.orientation.y
        obj.pose.orientation.z = shape_pose.orientation.z
        obj.pose.orientation.w = shape_pose.orientation.w
        local_pose = Pose()
        local_pose.orientation.w = 1.0
        obj.primitive_poses[0] = local_pose

    confirmation = _confirm_with_geometry(
        declaration, objects, canonical_frame_id="base_link"
    )
    assert confirmation.ready, confirmation.reasons


def test_confirm_geometry_wrong_frame_rejected() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    objects = _geometry_objects(declaration)
    for obj in objects:
        obj.header.frame_id = "map"
    confirmation = _confirm_with_geometry(declaration, objects)
    assert not confirmation.ready
    assert any("geometry mismatch" in reason for reason in confirmation.reasons)


def test_confirm_geometry_wrong_pose_rejected() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    objects = _geometry_objects(declaration)
    objects[0].primitive_poses[0].position.x += 0.05
    confirmation = _confirm_with_geometry(declaration, objects)
    assert not confirmation.ready
    assert any("geometry mismatch" in reason for reason in confirmation.reasons)


def test_confirm_geometry_wrong_dimension_rejected() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    objects = _geometry_objects(declaration)
    objects[0].primitives[0].dimensions[0] += 0.1
    confirmation = _confirm_with_geometry(declaration, objects)
    assert not confirmation.ready
    assert any("geometry mismatch" in reason for reason in confirmation.reasons)


def test_confirm_geometry_wrong_type_rejected() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    objects = _geometry_objects(declaration)
    objects[0].primitives[0].type = 2  # sphere instead of box
    confirmation = _confirm_with_geometry(declaration, objects)
    assert not confirmation.ready
    assert any("geometry mismatch" in reason for reason in confirmation.reasons)


def test_confirm_geometry_empty_geometry_rejected() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    objects = _geometry_objects(declaration)
    for obj in objects:
        obj.primitives.clear()
        obj.meshes.clear()
    confirmation = _confirm_with_geometry(declaration, objects)
    assert not confirmation.ready
    assert any("geometry mismatch" in reason for reason in confirmation.reasons)


def test_confirm_geometry_duplicate_readback_id_rejected() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    objects = _geometry_objects(declaration)
    objects.append(_geometry_objects(declaration)[0])
    confirmation = _confirm_with_geometry(declaration, objects)
    assert not confirmation.ready
    assert any("duplicate" in reason for reason in confirmation.reasons)


def test_confirm_geometry_reordered_objects_normalized() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-pose.json")
    objects = list(reversed(_geometry_objects(declaration)))
    confirmation = _confirm_with_geometry(declaration, objects)
    assert confirmation.ready, confirmation.reasons


def test_confirm_geometry_stale_foreign_id_rejected() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    objects = _geometry_objects(declaration)
    stale = CollisionObjectSpec(
        id="sim_fixture/stale",
        frame_id="base_link",
        operation=OBJECT_ADD,
        primitives=({"type": "box", "dimensions": [1.0, 1.0, 1.0]},),
        primitive_poses=((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),),
    )
    from tinker_sim_bridge.fixture_planning_scene_node import _spec_to_collision_object

    objects.append(_spec_to_collision_object(stale))
    confirmation = _confirm_with_geometry(declaration, objects)
    assert not confirmation.ready
    assert any("unexpected sim_fixture" in reason for reason in confirmation.reasons)


def test_confirm_geometry_requires_both_expected_and_observed() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    objects = _geometry_objects(declaration)
    from tinker_sim_bridge.fixture_contract import confirm_fixture_revision
    from tinker_sim_bridge.fixture_planning_scene import (
        canonical_fixture_status,
        fixture_descriptor_sha256,
        fixture_owned_ids,
    )

    status = canonical_fixture_status(
        scenario="s", revision=str(declaration["revision"]),
        revision_digest=revision_digest(declaration), sequence=1, published_at=1.0,
        owned_ids=fixture_owned_ids(declaration),
        target_source_id=str(declaration["target_source_id"]),
        target_handoff=str(declaration["target_handoff"]),
        descriptor_sha256=fixture_descriptor_sha256(declaration),
        state=FIXTURE_STATE_READY,
    )
    confirmation = confirm_fixture_revision(
        service_result=True,
        scene_ids=[obj.id for obj in objects],
        status=status,
        expected_revision=str(declaration["revision"]),
        expected_digest=revision_digest(declaration),
        expected_owned_ids=fixture_owned_ids(declaration),
        observed_geometry=[readback_geometry(obj) for obj in objects],
    )
    assert not confirmation.ready
    assert any("both expected and observed" in reason for reason in confirmation.reasons)


def test_confirm_geometry_mesh_roundtrip_and_vertex_mutation(tmp_path: Path) -> None:
    _humble()
    asset = tmp_path / "box.stl"
    asset.write_bytes(_tiny_binary_stl())
    digest = sha256(asset.read_bytes()).hexdigest()
    declaration = {
        "revision": "r-mesh",
        "frame_id": "base_link",
        "target_source_id": "sim_fixture/table",
        "target_handoff": "pick_and_place/object_mesh",
        "objects": [
            {
                "id": "sim_fixture/table",
                "mesh": {"uri": "box.stl", "sha256": digest, "scale": [1.0, 1.0, 1.0]},
                "pose": {"xyz": [0.5, 0.0, 0.5], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            }
        ],
    }
    loader = lambda mesh: load_mesh_asset(mesh, project_root=tmp_path)
    objects = _geometry_objects(declaration, mesh_loader=loader)
    confirmation = _confirm_with_geometry(declaration, objects, mesh_loader=loader)
    assert confirmation.ready, confirmation.reasons
    assert confirmation.geometry_consistent
    # Mutate a mesh vertex in the readback -> geometry mismatch.
    objects[0].meshes[0].vertices[1].x += 0.05
    confirmation = _confirm_with_geometry(declaration, objects, mesh_loader=loader)
    assert not confirmation.ready
    assert any("geometry mismatch" in reason for reason in confirmation.reasons)


def test_malformed_readback_empty_id_rejected() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    objects = _geometry_objects(declaration)
    objects[0].id = ""
    with pytest.raises(Exception, match="empty id"):
        readback_geometry(objects[0])


def test_malformed_readback_empty_frame_rejected() -> None:
    _humble()
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    objects = _geometry_objects(declaration)
    objects[0].header.frame_id = ""
    with pytest.raises(Exception, match="empty frame"):
        readback_geometry(objects[0])


# ---------------------------------------------------------------------------
# Real constructor wiring (Humble isolated ROS domain/context)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def ros_context():
    """One isolated ROS context shared by the real-constructor tests.

    A single rclpy init/shutdown per module minimizes FastDDS in-process churn;
    each test still creates and destroys its own nodes within the shared
    context.  The context is skipped under the simulator venv (no rclpy).
    """
    _humble()
    import rclpy

    os.environ["ROS_DOMAIN_ID"] = "47"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    context = rclpy.context.Context()
    rclpy.init(context=context)
    yield context
    rclpy.shutdown(context=context)


def test_real_constructor_required_ids_mismatch_rejected(ros_context) -> None:
    _humble()
    from rclpy.parameter import Parameter

    from tinker_sim_bridge.fixture_planning_scene_node import FixturePlanningScene

    with pytest.raises(ValueError, match="required_fixture_owned_ids"):
        FixturePlanningScene(
            node_name="fixture_ctor_mismatch",
            context=ros_context,
            parameter_overrides=[
                Parameter("scenario_file", value=str(SCENARIOS / "qualification-moveit-plan-joint.json")),
                Parameter("required_fixture_owned_ids", value="sim_fixture/wrong"),
            ],
        )


def test_real_constructor_malformed_scenario_rejected(ros_context) -> None:
    _humble()
    from rclpy.parameter import Parameter

    from tinker_sim_bridge.fixture_planning_scene_node import FixturePlanningScene

    with pytest.raises(Exception):
        FixturePlanningScene(
            node_name="fixture_ctor_missing",
            context=ros_context,
            parameter_overrides=[
                Parameter("scenario_file", value="/nonexistent/scenario.json"),
            ],
        )


def test_real_constructor_wiring_and_heartbeat(ros_context) -> None:
    _humble()
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String

    from tinker_sim_bridge.fixture_planning_scene_node import FixturePlanningScene

    context = ros_context
    node = None
    sub_node = None
    executor = None
    try:
        node = FixturePlanningScene(
            node_name="fixture_ctor_wiring",
            context=context,
            parameter_overrides=[
                Parameter("scenario_file", value=str(SCENARIOS / "qualification-moveit-plan-joint.json")),
                Parameter("heartbeat_period", value=0.2),
            ],
        )
        # Scenario load.
        assert node._scenario_id == "qualification-moveit-plan-joint"
        assert node._revision == "2026-08-01-moveit-qualification-joint"
        assert node._owned_ids == ("sim_fixture/pedestal", "sim_fixture/public_target")
        assert node._target_source_id == "sim_fixture/public_target"
        assert node._target_handoff == "pick_and_place/object_mesh"
        assert node._descriptor_sha256
        # Publisher topic + QoS.
        assert node._publisher.topic_name == "/sim/status/planning_scene_fixture"
        qos = node._publisher.qos_profile
        assert qos.reliability == ReliabilityPolicy.RELIABLE
        assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert qos.depth == 1
        # Ready service advertised.
        service_names = dict(node.get_service_names_and_types())
        assert "/sim/ready/fixture" in service_names
        # 5 Hz heartbeat through the real publisher (no physics gate -> PENDING).
        sub_node = rclpy.create_node("fixture_ctor_sub", context=context)
        received: list[Mapping[str, object]] = []
        sub_node.create_subscription(
            String,
            "/sim/status/planning_scene_fixture",
            lambda message: received.append(json.loads(message.data)),
            10,
        )
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.add_node(sub_node)
        end = time.monotonic() + 1.3
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
        executor.shutdown()
        executor = None
        assert len(received) >= 4, "expected ~5 Hz heartbeat, got {}".format(len(received))
        sequences = [payload["sequence"] for payload in received]
        assert sequences == sorted(sequences) and len(set(sequences)) == len(sequences)
        assert all(payload["state"] == "FIXTURE_PENDING" for payload in received)
        assert all(payload["owner"] == "sim_fixture" for payload in received)
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if sub_node is not None:
            sub_node.destroy_node()


def test_real_constructor_full_ready_loop_with_mock_services(ros_context) -> None:
    _humble()
    import rclpy
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    from rclpy.parameter import Parameter
    from std_srvs.srv import Trigger

    from tinker_sim_bridge.fixture_planning_scene_node import (
        FixturePlanningScene,
        _spec_to_collision_object,
    )
    from tinker_sim_bridge.fixture_planning_scene import fixture_to_specs

    scenario_file = SCENARIOS / "qualification-moveit-plan-pose.json"
    declaration = load_fixture_scenario(scenario_file)
    specs = fixture_to_specs(declaration)
    readback_objects = [
        _spec_to_collision_object(spec, mesh_loader=None) for spec in specs
    ]

    context = ros_context
    server = None
    node = None
    executor = None
    try:
        server = rclpy.create_node("fixture_ctor_servers", context=context)
        applied = {"value": False}

        def on_physics(request, response):
            del request
            response.success = True
            return response

        def on_apply(request, response):
            del request
            applied["value"] = True
            response.success = True
            return response

        def on_get(request, response):
            del request
            if applied["value"]:
                response.scene.world.collision_objects.extend(readback_objects)
            return response

        server.create_service(Trigger, "/sim/ready/physics", on_physics)
        server.create_service(ApplyPlanningScene, "/apply_planning_scene", on_apply)
        server.create_service(GetPlanningScene, "/get_planning_scene", on_get)

        node = FixturePlanningScene(
            node_name="fixture_ctor_loop",
            context=context,
            parameter_overrides=[
                Parameter("scenario_file", value=str(scenario_file)),
            ],
        )
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.add_node(server)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and node._state != FIXTURE_STATE_READY:
            executor.spin_once(timeout_sec=0.05)
        executor.shutdown()
        executor = None
        assert node._state == FIXTURE_STATE_READY, (
            "state={} phase={} reason={}".format(node._state, node._phase, node._fail_reason)
        )
        # The ready service now succeeds.
        response = Trigger.Response()
        node._on_ready(Trigger.Request(), response)
        assert response.success is True
        payload = json.loads(response.message)
        assert payload["state"] == FIXTURE_STATE_READY
        assert tuple(payload["owned_ids"]) == node._owned_ids
        assert applied["value"] is True
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if server is not None:
            server.destroy_node()


def test_get_request_requests_world_object_names_and_geometry(ros_context) -> None:
    """The GetPlanningScene request must carry the explicit component bitmask
    (WORLD_OBJECT_NAMES | WORLD_OBJECT_GEOMETRY), never the server-dependent
    components=0 default."""
    _humble()
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene

    from tinker_sim_bridge.fixture_planning_scene_node import _get_planning_scene_request

    client = type("Client", (), {})()
    client.srv_type = GetPlanningScene
    request = _get_planning_scene_request(client)
    expected = (
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    )
    assert request.components.components == expected
    assert request.components.components == (8 | 16)
    assert request.components.components & PlanningSceneComponents.WORLD_OBJECT_NAMES
    assert request.components.components & PlanningSceneComponents.WORLD_OBJECT_GEOMETRY


def test_real_constructor_rejects_nonfinite_or_nonpositive_deadlines(ros_context) -> None:
    _humble()
    from rclpy.parameter import Parameter

    from tinker_sim_bridge.fixture_planning_scene_node import FixturePlanningScene

    base = [
        Parameter("scenario_file", value=str(SCENARIOS / "qualification-moveit-plan-joint.json"))
    ]
    with pytest.raises(ValueError, match="finite positive"):
        FixturePlanningScene(
            node_name="fixture_bad_period",
            context=ros_context,
            parameter_overrides=base + [Parameter("heartbeat_period", value=float("nan"))],
        )
    with pytest.raises(ValueError, match="finite positive"):
        FixturePlanningScene(
            node_name="fixture_inf_period",
            context=ros_context,
            parameter_overrides=base + [Parameter("heartbeat_period", value=float("inf"))],
        )
    with pytest.raises(ValueError, match="finite positive"):
        FixturePlanningScene(
            node_name="fixture_nan_deadline",
            context=ros_context,
            parameter_overrides=base + [Parameter("start_deadline_s", value=float("nan"))],
        )
    with pytest.raises(ValueError, match="finite positive"):
        FixturePlanningScene(
            node_name="fixture_zero_deadline",
            context=ros_context,
            parameter_overrides=base + [Parameter("start_deadline_s", value=0.0)],
        )


def _mesh_scenario_payload(tmp_path: Path, *, mesh: Mapping[str, object], scenario_id: str) -> Path:
    """Write a schema-valid single-mesh-fixture scenario and return its path."""
    payload = {
        "schema_version": 2,
        "id": scenario_id,
        "world": {"mode": "current"},
        "robot": {"id": "tinker2", "initial_pose": [0.0, 0.0, 0.0]},
        "actors": [],
        "objects": [],
        "events": [{"at_sim_time": 0.0, "event": "spawn_once_while_paused"}],
        "postconditions": [{"name": "ready", "path": "x", "operator": "equals", "value": True}],
        "planning_scene": {
            "revision": "r-mesh",
            "frame_id": "base_link",
            "target_source_id": "sim_fixture/table",
            "target_handoff": "pick_and_place/object_mesh",
            "objects": [
                {
                    "id": "sim_fixture/table",
                    "mesh": dict(mesh),
                    "pose": {"xyz": [0.5, 0.0, 0.5], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                }
            ],
        },
    }
    ps = payload["planning_scene"]
    ps["revision_digest"] = sha256(
        json.dumps(
            {k: v for k, v in ps.items() if k != "revision_digest"},
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    scenario_path = tmp_path / "scenario.json"
    scenario_path.write_text(json.dumps(payload), encoding="utf-8")
    return scenario_path


def test_real_constructor_mesh_failures_fail_immediately(ros_context, tmp_path: Path) -> None:
    """Unsupported/missing/hash-mismatched mesh scenarios must fail at
    construction (clear immediate error), not after a misleading apply timeout."""
    _humble()
    from rclpy.parameter import Parameter

    from tinker_sim_bridge.fixture_planning_scene_node import FixturePlanningScene

    # Missing file.
    missing = _mesh_scenario_payload(
        tmp_path,
        mesh={"uri": "missing.stl", "sha256": "a" * 64, "scale": [1.0, 1.0, 1.0]},
        scenario_id="mesh-missing",
    )
    with pytest.raises(ValueError, match="not found"):
        FixturePlanningScene(
            node_name="fixture_mesh_missing",
            context=ros_context,
            parameter_overrides=[Parameter("scenario_file", value=str(missing))],
        )
    # Unsupported format.
    bad = tmp_path / "box.xyz"
    bad.write_bytes(b"garbage")
    xyz = _mesh_scenario_payload(
        tmp_path,
        mesh={"uri": "box.xyz", "sha256": sha256(b"garbage").hexdigest(), "scale": [1.0, 1.0, 1.0]},
        scenario_id="mesh-unsupported",
    )
    with pytest.raises(ValueError, match="not supported"):
        FixturePlanningScene(
            node_name="fixture_mesh_unsupported",
            context=ros_context,
            parameter_overrides=[Parameter("scenario_file", value=str(xyz))],
        )
    # Hash mismatch against the actual file bytes.
    asset = tmp_path / "box.stl"
    asset.write_bytes(_tiny_binary_stl())
    mismatched = _mesh_scenario_payload(
        tmp_path,
        mesh={"uri": "box.stl", "sha256": "a" * 64, "scale": [1.0, 1.0, 1.0]},
        scenario_id="mesh-mismatch",
    )
    with pytest.raises(ValueError, match="sha256 mismatch"):
        FixturePlanningScene(
            node_name="fixture_mesh_mismatch",
            context=ros_context,
            parameter_overrides=[Parameter("scenario_file", value=str(mismatched))],
        )


def test_real_constructor_mesh_ready_loop(ros_context, tmp_path: Path) -> None:
    """A real schema-valid mesh scenario keeps its loader for the node lifetime:
    _build_apply_request emits a non-empty scaled shape_msgs/Mesh and the real
    ready-loop applies, reads back, and confirms it to FIXTURE_READY."""
    _humble()
    import rclpy
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    from rclpy.parameter import Parameter
    from std_srvs.srv import Trigger

    from tinker_sim_bridge.fixture_planning_scene_node import (
        FixturePlanningScene,
        _spec_to_collision_object,
    )
    from tinker_sim_bridge.fixture_planning_scene import fixture_to_specs

    asset = tmp_path / "box.stl"
    asset.write_bytes(_tiny_binary_stl())
    digest = sha256(asset.read_bytes()).hexdigest()
    scenario = _mesh_scenario_payload(
        tmp_path,
        mesh={"uri": "box.stl", "sha256": digest, "scale": [2.0, 3.0, 4.0]},
        scenario_id="mesh-ready-loop",
    )

    context = ros_context
    server = None
    node = None
    executor = None
    applied_request = {}
    try:
        server = rclpy.create_node("fixture_mesh_servers", context=context)
        applied = {"value": False}

        def on_physics(request, response):
            del request
            response.success = True
            return response

        def on_apply(request, response):
            applied["value"] = True
            applied_request["request"] = request
            response.success = True
            return response

        def on_get(request, response):
            del request
            if applied["value"]:
                for spec in node._specs:
                    response.scene.world.collision_objects.append(
                        _spec_to_collision_object(spec, mesh_loader=node._load_mesh)
                    )
            return response

        server.create_service(Trigger, "/sim/ready/physics", on_physics)
        server.create_service(ApplyPlanningScene, "/apply_planning_scene", on_apply)
        server.create_service(GetPlanningScene, "/get_planning_scene", on_get)

        node = FixturePlanningScene(
            node_name="fixture_mesh_loop",
            context=context,
            parameter_overrides=[Parameter("scenario_file", value=str(scenario))],
        )
        # The mesh loader must be preserved (Critical fix) and project root set.
        assert callable(node._load_mesh)
        assert node._project_root is not None

        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.add_node(server)
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and node._state != FIXTURE_STATE_READY:
            executor.spin_once(timeout_sec=0.05)
        executor.shutdown()
        executor = None
        assert node._state == FIXTURE_STATE_READY, (
            "state={} phase={} reason={}".format(node._state, node._phase, node._fail_reason)
        )
        # The apply request carried a real, non-empty, scaled mesh.
        request = applied_request["request"]
        mesh_objects = [
            obj for obj in request.scene.world.collision_objects if obj.meshes
        ]
        assert len(mesh_objects) == 1
        mesh_msg = mesh_objects[0].meshes[0]
        assert len(mesh_msg.vertices) == 3
        assert len(mesh_msg.triangles) == 1
        assert mesh_msg.vertices[0] == _point(0.0, 0.0, 0.0)
        assert mesh_msg.vertices[1] == _point(2.0, 0.0, 0.0)
        assert mesh_msg.vertices[2] == _point(0.0, 3.0, 0.0)
        assert list(mesh_msg.triangles[0].vertex_indices) == [0, 1, 2]
        # Ready service confirms.
        response = Trigger.Response()
        node._on_ready(Trigger.Request(), response)
        assert response.success is True
        payload = json.loads(response.message)
        assert payload["state"] == FIXTURE_STATE_READY
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if server is not None:
            server.destroy_node()


def _point(x: float, y: float, z: float):
    from geometry_msgs.msg import Point

    point = Point()
    point.x, point.y, point.z = x, y, z
    return point


def test_get_extract_foreign_namespace_ignored_for_geometry(ros_context) -> None:
    """A malformed foreign object is preserved by raw id for leak checks but its
    geometry is never parsed, so it cannot block fixture readiness; malformed
    sim_fixture/* objects are still rejected."""
    _humble()
    from moveit_msgs.msg import CollisionObject
    from moveit_msgs.srv import GetPlanningScene

    from tinker_sim_bridge.fixture_planning_scene_node import (
        FixturePlanningScene,
        _spec_to_collision_object,
    )
    from tinker_sim_bridge.fixture_planning_scene import fixture_to_specs

    node = FixturePlanningScene.__new__(FixturePlanningScene)
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    specs = fixture_to_specs(declaration)
    response = GetPlanningScene.Response()
    for spec in specs:
        response.scene.world.collision_objects.append(_spec_to_collision_object(spec))
    # Malformed foreign object: empty frame_id (invalid for geometry, but the
    # fixture does not own it and must not fail because of it).
    foreign = CollisionObject()
    foreign.id = "nav/foreign"
    foreign.header.frame_id = ""
    response.scene.world.collision_objects.append(foreign)

    extracted = node._get_extract(response)
    pairs = dict(extracted)
    assert "nav/foreign" in pairs
    assert pairs["nav/foreign"] is None
    assert len(extracted) == len(specs) + 1
    assert all(
        geometry is not None
        for oid, geometry in extracted
        if oid.startswith("sim_fixture/")
    )

    # A malformed sim_fixture/* object is still rejected (fail closed).
    bad = CollisionObject()
    bad.id = "sim_fixture/bad"
    bad.header.frame_id = ""
    response.scene.world.collision_objects.append(bad)
    with pytest.raises(Exception, match="empty frame"):
        node._get_extract(response)


def test_foreign_namespace_geometry_confirm_and_diff(ros_context) -> None:
    """Foreign ids survive the readback for leak checks; geometry confirmation
    covers only owned sim_fixture/* objects; the atomic diff never removes a
    foreign object."""
    _humble()
    from moveit_msgs.msg import CollisionObject
    from rclpy.parameter import Parameter

    from tinker_sim_bridge.fixture_contract import (
        build_atomic_revision_diff,
        confirm_fixture_revision,
        spec_geometry,
    )
    from tinker_sim_bridge.fixture_planning_scene import (
        canonical_fixture_status,
        fixture_descriptor_sha256,
        fixture_owned_ids,
    )
    from tinker_sim_bridge.fixture_planning_scene_node import (
        FixturePlanningScene,
        _spec_to_collision_object,
    )

    node = FixturePlanningScene(
        node_name="fixture_foreign_confirm",
        context=ros_context,
        parameter_overrides=[
            Parameter("scenario_file", value=str(SCENARIOS / "qualification-moveit-plan-joint.json")),
        ],
    )
    try:
        specs = node._specs
        scene_objects = [
            _spec_to_collision_object(spec, mesh_loader=node._load_mesh) for spec in specs
        ]
        foreign = CollisionObject()
        foreign.id = "nav/foreign"
        foreign.header.frame_id = "map"
        scene_objects.append(foreign)
        extracted = node._get_extract(type("Resp", (), {"scene": type("Sc", (), {"world": type("W", (), {"collision_objects": scene_objects})()})()})())
        scene_ids = tuple(oid for oid, _geometry in extracted)
        observed_geometry = tuple(
            geometry for _oid, geometry in extracted if geometry is not None
        )
        status = canonical_fixture_status(
            scenario=node._scenario_id,
            revision=node._revision,
            revision_digest=node._revision_digest,
            sequence=1,
            published_at=1.0,
            owned_ids=node._owned_ids,
            target_source_id=node._target_source_id,
            target_handoff=node._target_handoff,
            descriptor_sha256=node._descriptor_sha256,
            state=FIXTURE_STATE_READY,
        )
        confirmation = confirm_fixture_revision(
            service_result=True,
            scene_ids=scene_ids,
            status=status,
            expected_revision=node._revision,
            expected_digest=node._revision_digest,
            expected_owned_ids=node._owned_ids,
            expected_geometry={
                spec.id: spec_geometry(spec, resolve_mesh=node._load_mesh) for spec in specs
            },
            observed_geometry=observed_geometry,
        )
        assert confirmation.ready, confirmation.reasons
        assert "nav/foreign" in confirmation.observed_scene_ids
        assert confirmation.foreign_fixture_ids == ()
        # The atomic diff excludes the foreign object from removal.
        diff = build_atomic_revision_diff(desired_objects=specs, existing_ids=scene_ids)
        assert "nav/foreign" not in diff.removed_ids
        assert "nav/foreign" not in [
            obj["id"] for obj in diff.apply_request["world"]["collision_objects"]
        ]
    finally:
        node.destroy_node()
