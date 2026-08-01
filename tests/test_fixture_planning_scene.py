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
    parse_required_fixture_owned_ids,
    revision_digest,
)
from tinker_sim_bridge.fixture_contract import (  # noqa: E402
    FIXTURE_STATE_FAILED,
    FIXTURE_STATE_PENDING,
)
from tinker_sim_bridge.fixture_planning_scene import (  # noqa: E402
    FIXTURE_OWNER,
    FIXTURE_STATE_READY,
    STATUS_SCHEMA_VERSION,
    canonical_fixture_status,
    fixture_descriptor_sha256,
    fixture_owned_ids,
    fixture_to_specs,
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
    blocked = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-blocked.json")

    assert fixture_owned_ids(joint) == ("sim_fixture/pedestal", "sim_fixture/public_target")
    assert fixture_owned_ids(pose) == ("sim_fixture/pedestal", "sim_fixture/public_target")
    assert fixture_owned_ids(blocked) == (
        "sim_fixture/pedestal",
        "sim_fixture/public_target",
        "sim_fixture/plan_blocker",
    )
    for declaration in (joint, pose, blocked):
        assert declaration["frame_id"] == "base_link"
        assert declaration["target_source_id"] == "sim_fixture/public_target"
        assert declaration["target_handoff"] == "pick_and_place/object_mesh"
        assert declaration["target_handoff"] == TARGET_HANDOFF
        assert str(declaration["target_source_id"]) in fixture_owned_ids(declaration)


def test_exact_task_owned_handoff_identity_constant() -> None:
    assert TARGET_HANDOFF == "pick_and_place/object_mesh"
    assert fixture_owned_ids(
        load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-blocked.json")
    )[-1] == "sim_fixture/plan_blocker"


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


def test_blocked_scenario_digest_differs_from_joint() -> None:
    joint = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-joint.json")
    blocked = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-blocked.json")
    assert revision_digest(blocked) != revision_digest(joint)
    assert blocked["revision_digest"] == revision_digest(blocked)


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
    blocked = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-blocked.json")
    assert fixture_to_specs(blocked) == fixture_to_scene(blocked)


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
    blocked = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-blocked.json")
    assert fixture_descriptor_sha256(joint) == fixture_descriptor_sha256(joint)
    assert fixture_descriptor_sha256(blocked) != fixture_descriptor_sha256(joint)


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

    scenario = load_named_scenario(ROOT, "qualification-moveit-plan-blocked")
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
    # Hashed absolute/declared mesh asset.
    mesh = dict(valid_ps)
    mesh["objects"] = [
        {
            "id": "sim_fixture/table",
            "mesh": {"uri": "simulation/assets/table.stl", "sha256": "a" * 64},
            "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        }
    ]
    mesh["target_source_id"] = "sim_fixture/table"
    mesh = fresh(mesh)
    load_scenario(with_planning_scene(mesh))
    bad_mesh = dict(mesh)
    bad_mesh["objects"] = [
        {
            "id": "sim_fixture/table",
            "mesh": {"uri": "simulation/assets/table.stl", "sha256": "not-a-digest"},
            "pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        }
    ]
    bad_mesh = fresh(bad_mesh)
    with pytest.raises(ValueError, match="sha256"):
        load_scenario(with_planning_scene(bad_mesh))


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
    declaration = load_fixture_scenario(SCENARIOS / "qualification-moveit-plan-blocked.json")
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
        "sim_fixture/plan_blocker",
        "sim_fixture/stale_removed",
    ]
    assert [obj.operation for obj in collision_objects] == [
        b"\x00", b"\x00", b"\x00", b"\x01",
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
