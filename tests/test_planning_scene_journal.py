"""Task 3: pure PlanningScene journal contract tests.

Runs under simulator CPython 3.12 (ROS-free).  Exercises the ROS-free
``validation/planning_scene_journal`` pure seam:

- the model-contract loader anchored to the committed
  ``integration/ompl-overlay-contract.json`` (never a duplicate hard-coded tuple
  as authority) plus contract-drift fixtures;
- transactional validation: a rejected record/event leaves ``_records``,
  ``_last_scene``, JSONL bytes, and final JSON bytes unchanged;
- durable canonical append-only ``planning-scene.jsonl`` and atomic final
  ``planning-scene.json`` with no temp residue;
- graph-evidence validation (type/QoS/source/cardinality/remap/payload) over a
  Task-4-supplied projection and retention of the validated evidence;
- positive/negative event ordering, duplicate/forbidden events, and
  attach/detach/cleanup ownership semantics.

The ``MODEL_TOUCH_LINKS`` / ``EXPECTED_ATTACH_LINK`` / ``TARGET_HANDOFF``
constants are derived from ``load_model_touch_contract()`` reading the committed
overlay contract; the literal eight-link assertion below is a contract-drift
cross-check, not the authority used to build the journal.
"""
from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from planning_scene_journal import (  # noqa: E402
    DIGEST,
    POSITIVE_ORDER,
    RECORDER_NODE,
    PlanningSceneJournal,
    load_model_touch_contract,
    validate_graph_evidence,
)

# Anchor the contract to the committed overlay contract (never a caller-invented
# duplicate tuple).  The loader itself fails closed on drift.
_MODEL_CONTRACT = load_model_touch_contract()
MODEL_TOUCH_LINKS = tuple(_MODEL_CONTRACT["touch_links"])
EXPECTED_ATTACH_LINK = str(_MODEL_CONTRACT["link_tcp"])
TARGET_HANDOFF = str(_MODEL_CONTRACT["target_handoff"])


def _fixture_digest(label: str, value: object) -> str:
    digest = hashlib.sha256(
        f"qualification-journal:{label}:{value}".encode("utf-8")
    ).hexdigest()
    assert DIGEST.fullmatch(digest)
    return digest


def _scene(seq, stamp, *, world=(), attached=(), source="fixture"):
    return {
        "scene_sequence": seq,
        "scene_timestamp": stamp,
        "frame_index": seq,
        "timestamp": stamp,
        "owned_ids": list(world),
        "attached_ids": list(attached),
        "attached_links": {name: EXPECTED_ATTACH_LINK for name in attached},
        "touch_links": {name: list(MODEL_TOUCH_LINKS) for name in attached},
        "fixture_revision": "qualification-v1",
        "scene_revision_digest": _fixture_digest("scene", seq),
        "acm_digest": _fixture_digest("acm", "qualification-v1"),
        "robot_state_digest": _fixture_digest("robot-state", seq),
        "source": source,
    }


def _journal(*, required_event_order=(), forbidden_events=(), jsonl_path=None):
    return PlanningSceneJournal(
        fixture_revision="qualification-v1",
        task_namespace="pick_and_place/",
        target_object_id=TARGET_HANDOFF,
        expected_attach_link=EXPECTED_ATTACH_LINK,
        expected_touch_links=MODEL_TOUCH_LINKS,
        required_event_order=required_event_order,
        forbidden_events=forbidden_events,
        jsonl_path=jsonl_path,
    )


def _valid_graph():
    """Build a fully valid Task-4-supplied graph projection."""
    return {
        "node_name": "/tinker_integrated_gate_executor",
        "namespace": "/",
        "remap_table": {},
        "topics": {
            "/planning_scene": {
                "type": "moveit_msgs/msg/PlanningScene",
                "requested_qos": {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1},
                "offered_qos": {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1},
                "publishers": [{"node": "/moveit_planning_scene_monitor", "node_namespace": "/"}],
                "subscribers": [{"node": "/tinker_integrated_gate_executor", "node_namespace": "/"}],
            },
            "/monitored_planning_scene": {
                "type": "moveit_msgs/msg/PlanningScene",
                "requested_qos": {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1},
                "offered_qos": {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1},
                "publishers": [{"node": "/moveit_planning_scene_monitor", "node_namespace": "/"}],
                "subscribers": [{"node": "/tinker_integrated_gate_executor", "node_namespace": "/"}],
            },
            "/sim/status/planning_scene_fixture": {
                "type": "std_msgs/msg/String",
                "requested_qos": {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1},
                "offered_qos": {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1},
                "publishers": [{"node": "/fixture_planning_scene", "node_namespace": "/"}],
                "subscribers": [{"node": "/tinker_integrated_gate_executor", "node_namespace": "/"}],
                "payload": _fixture_status_payload(),
            },
        },
        "services": {
            "/get_planning_scene": {
                "type": "moveit_msgs/srv/GetPlanningScene",
                "requested_qos": {"reliability": "RELIABLE", "durability": "VOLATILE"},
                "offered_qos": {"reliability": "RELIABLE", "durability": "VOLATILE"},
                "servers": [{"node": "/moveit_planning_scene_monitor", "node_namespace": "/"}],
                "clients": [{"node": "/tinker_integrated_gate_executor", "node_namespace": "/"}],
            },
            "/apply_planning_scene": {
                "type": "moveit_msgs/srv/ApplyPlanningScene",
                "requested_qos": {"reliability": "RELIABLE", "durability": "VOLATILE"},
                "offered_qos": {"reliability": "RELIABLE", "durability": "VOLATILE"},
                "servers": [{"node": "/moveit_planning_scene_monitor", "node_namespace": "/"}],
                "clients": [{"node": "/tinker_integrated_gate_executor", "node_namespace": "/"}],
            },
        },
    }


def _fixture_status_payload():
    status = {
        "schema_version": 1,
        "state": "FIXTURE_READY",
        "scenario": "qualification-pick-deliver-place",
        "owner": "sim_fixture",
        "revision": "qualification-v1",
        "revision_digest": "1" * 64,
        "sequence": 3,
        "published_at": 7.5,
        "owned_ids": ["sim_fixture/table"],
        "target_source_id": "sim_fixture/public_target",
        "target_handoff": TARGET_HANDOFF,
        "fixture_descriptor_sha256": "2" * 64,
    }
    return json.dumps(status, sort_keys=True, separators=(",", ":"))


def _mutated_contract(mutator):
    path = ROOT / "integration" / "ompl-overlay-contract.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    mutator(data)
    return data


# --------------------------------------------------------------------------- #
# Brief Step 1 cases (anchored to the loaded contract)
# --------------------------------------------------------------------------- #


def test_journal_fixture_digests_are_deterministic_nonzero_lowercase_sha256():
    scene = _scene(7, 7.0, world=("sim_fixture/table",))
    for field in ("scene_revision_digest", "acm_digest", "robot_state_digest"):
        assert DIGEST.fullmatch(scene[field])
        assert scene[field] != "0" * 64
    assert scene["scene_revision_digest"] == _scene(7, 7.0, world=("sim_fixture/table",))["scene_revision_digest"]


def test_sequence_and_timestamp_regression_is_rejected():
    journal = _journal()
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    with pytest.raises(ValueError, match="monotonic"):
        journal.record_diff("before-pick", _scene(1, 1.1, world=("sim_fixture/table",)))
    with pytest.raises(ValueError, match="monotonic"):
        journal.record_diff("before-pick", _scene(2, 1.0, world=("sim_fixture/table",)))


def test_sim_fixture_objects_survive_task_cleanup():
    journal = _journal()
    before = _scene(1, 1.0, world=("sim_fixture/table", "pick_and_place/object_mesh"))
    after = _scene(2, 2.0, world=("sim_fixture/table",))
    journal.record_diff("fixture-ready", before)
    journal.record_diff("task-cleanup", after)
    journal.assert_transition(before, after, expected="task-cleanup")
    assert "sim_fixture/table" in journal.finalize("diagnostic-pass")["records"][-1]["owned_ids"]


def test_task_cannot_remove_foreign_object():
    journal = _journal()
    before = _scene(1, 1.0, world=("sim_fixture/table", "other_node/keep"))
    after = _scene(2, 2.0, world=("other_node/keep",))
    journal.record_diff("fixture-ready", before)
    with pytest.raises(PermissionError, match="sim_fixture/table"):
        journal.record_diff("task-cleanup", after)


def test_scene_attach_is_diagnostic_without_physics_fields():
    journal = _journal()
    before = _scene(1, 1.0, world=("pick_and_place/object_mesh",))
    after = _scene(2, 2.0, attached=("pick_and_place/object_mesh",))
    journal.record_diff("before-pick", before)
    journal.assert_transition(before, after, expected="scene-attach")
    record = journal.record_diff("scene-attach", after)
    assert "physical_bilateral_contact" not in record
    assert "contacts" not in record


def test_positive_transition_is_world_to_attached_to_world():
    journal = _journal()
    fixture = _scene(1, 1.0, world=("pick_and_place/object_mesh", "sim_fixture/table"))
    attached = _scene(2, 2.0, world=("sim_fixture/table",),
                      attached=("pick_and_place/object_mesh",))
    released = _scene(3, 3.0, world=("sim_fixture/table", "pick_and_place/object_mesh"))
    journal.record_diff("fixture-ready", fixture)
    journal.record_diff("scene-attach", attached)
    journal.record_diff("scene-detach", released)
    journal.assert_transition(fixture, attached, expected="scene-attach")
    journal.assert_transition(attached, released, expected="scene-detach")
    assert journal.finalize("diagnostic-pass")["events"] == [
        "fixture-ready", "scene-attach", "scene-detach"
    ]


def test_duplicate_semantic_target_is_rejected():
    journal = _journal()
    duplicate = _scene(
        1, 1.0,
        world=("pick_and_place/object_mesh",),
        attached=("pick_and_place/object_mesh",),
    )
    with pytest.raises(ValueError, match="both world and attached"):
        journal.record_diff("invalid", duplicate)


def test_model_bundle_touch_contract_is_complete_eight_link_set():
    assert MODEL_TOUCH_LINKS == (
        "xarm_gripper_base_link", "left_outer_knuckle", "left_finger",
        "left_inner_knuckle", "right_inner_knuckle", "right_outer_knuckle",
        "right_finger", "link_tcp",
    )
    assert len(MODEL_TOUCH_LINKS) == 8


def test_touch_link_permutation_is_rejected():
    journal = _journal()
    permuted = _scene(1, 1.0, attached=("pick_and_place/object_mesh",))
    permuted["touch_links"]["pick_and_place/object_mesh"] = list(reversed(MODEL_TOUCH_LINKS))
    with pytest.raises(ValueError, match="touch links"):
        journal.record_diff("scene-attach", permuted)


def test_wrong_attach_link_or_touch_contract_is_rejected():
    journal = _journal()
    wrong_link = _scene(1, 1.0, attached=("pick_and_place/object_mesh",))
    wrong_link["attached_links"]["pick_and_place/object_mesh"] = "wrong_tcp"
    with pytest.raises(ValueError, match="attach link"):
        journal.record_diff("scene-attach", wrong_link)

    wrong_touch = _scene(1, 1.0, attached=("pick_and_place/object_mesh",))
    wrong_touch["touch_links"]["pick_and_place/object_mesh"] = ["left_finger"]
    with pytest.raises(ValueError, match="touch links"):
        journal.record_diff("scene-attach", wrong_touch)


def test_task_cleanup_cannot_remove_unknown_foreign_object():
    journal = _journal()
    before = _scene(1, 1.0, world=("other_node/keep", "pick_and_place/temp"))
    after = _scene(2, 2.0, world=())
    journal.record_diff("before-pick", before)
    with pytest.raises(PermissionError, match="other_node/keep"):
        journal.record_diff("task-cleanup", after)


def test_snapshot_uses_new_journal_identity_without_fabricating_scene_update():
    journal = _journal()
    first = journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    second = journal.snapshot("before-pick", frame_index=2, timestamp=2.0)
    assert second["journal_sequence"] == first["journal_sequence"] + 1
    assert second["scene_sequence"] == first["scene_sequence"]
    assert second["scene_timestamp"] == first["scene_timestamp"]
    assert second["scene_revision_digest"] == first["scene_revision_digest"]
    assert second["frame_index"] == 2
    assert second["timestamp"] == 2.0


def test_scene_identity_is_diagnostic_and_frame_timestamp_is_the_join_key():
    journal = _journal()
    scene = _scene(7, 70.0, world=("sim_fixture/table",))
    record = journal.record_diff("fixture-ready", scene)
    assert (record["frame_index"], record["timestamp"]) == (7, 70.0)
    assert (record["scene_sequence"], record["scene_timestamp"]) == (7, 70.0)
    scene["scene_sequence"] = 8
    scene["scene_timestamp"] = 80.0
    scene["frame_index"] = 9
    scene["timestamp"] = 90.0
    updated = journal.record_diff("scene-update", scene)
    assert (updated["frame_index"], updated["timestamp"]) == (9, 90.0)
    assert (updated["scene_sequence"], updated["scene_timestamp"]) == (8, 80.0)


def test_task_cleanup_allows_only_task_owned_prefix():
    journal = _journal()
    before = _scene(
        1, 1.0,
        world=("sim_fixture/table", "pick_and_place/object_mesh", "other_node/keep"),
    )
    after = _scene(2, 2.0, world=("sim_fixture/table", "other_node/keep"))
    journal.record_diff("fixture-ready", before)
    journal.record_diff("task-cleanup", after)
    assert "pick_and_place/object_mesh" not in journal.finalize("diagnostic-pass")["records"][-1]["owned_ids"]


def test_scene_identity_and_frame_join_key_are_recorded_separately():
    journal = _journal()
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    scene["scene_sequence"] = 2
    scene["scene_timestamp"] = 2.0
    scene["frame_index"] = 1
    scene["timestamp"] = 1.0
    record = journal.record_diff("fixture-ready", scene)
    assert (record["frame_index"], record["timestamp"]) == (1, 1.0)
    assert (record["scene_sequence"], record["scene_timestamp"]) == (2, 2.0)


def test_finalize_rejects_missing_or_out_of_order_required_events():
    journal = _journal(required_event_order=POSITIVE_ORDER)
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    journal.snapshot("before-pick", frame_index=2, timestamp=2.0)
    with pytest.raises(ValueError, match="required event order"):
        journal.finalize("diagnostic-pass")


def test_scene_journal_finalization_is_never_a_physical_verdict():
    journal = _journal()
    journal.record_diff(
        "scene-attach",
        _scene(1, 1.0, attached=("pick_and_place/object_mesh",)),
    )
    verdict = journal.finalize("diagnostic-pass")
    assert verdict["authority"] == "physics_truth"
    assert "physical_grasp_verified" not in verdict
    assert "contacts" not in verdict


# --------------------------------------------------------------------------- #
# Model-contract loader
# --------------------------------------------------------------------------- #


def test_loader_reads_committed_contract():
    contract = load_model_touch_contract()
    assert contract["link_tcp"] == EXPECTED_ATTACH_LINK == "link_tcp"
    assert tuple(contract["touch_links"]) == MODEL_TOUCH_LINKS
    assert contract["target_handoff"] == TARGET_HANDOFF == "pick_and_place/object_mesh"


def test_loader_fails_on_missing_contract(tmp_path):
    with pytest.raises(ValueError, match="contract"):
        load_model_touch_contract(tmp_path / "missing.json")


def test_loader_fails_on_malformed_json(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(ValueError, match="malformed"):
        load_model_touch_contract(path)


def test_loader_fails_on_permuted_touch_links(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(_mutated_contract(lambda d: d["model_bundle"]["semantic_contract"].update(
        touch_links=list(reversed(MODEL_TOUCH_LINKS))
    ))), encoding="utf-8")
    with pytest.raises(ValueError, match="order"):
        load_model_touch_contract(path)


def test_loader_fails_on_seven_link_set(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(_mutated_contract(lambda d: d["model_bundle"]["semantic_contract"].update(
        touch_links=list(MODEL_TOUCH_LINKS[:-1])
    ))), encoding="utf-8")
    with pytest.raises(ValueError, match="eight-link"):
        load_model_touch_contract(path)


def test_loader_fails_on_duplicate_touch_links(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(_mutated_contract(lambda d: d["model_bundle"]["semantic_contract"].update(
        touch_links=[MODEL_TOUCH_LINKS[0]] + list(MODEL_TOUCH_LINKS[:-1])
    ))), encoding="utf-8")
    with pytest.raises(ValueError, match="unique"):
        load_model_touch_contract(path)


def test_loader_fails_on_missing_link_tcp(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(_mutated_contract(lambda d: d["model_bundle"]["semantic_contract"].update(
        tcp_link="wrong_tcp"
    ))), encoding="utf-8")
    with pytest.raises(ValueError, match="tcp_link"):
        load_model_touch_contract(path)


def test_loader_fails_on_wrong_handoff(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text(json.dumps(_mutated_contract(lambda d: d["fixture_contract"].update(
        target_handoff="other/thing"
    ))), encoding="utf-8")
    with pytest.raises(ValueError, match="handoff"):
        load_model_touch_contract(path)


def test_loader_fails_on_non_object_contract(tmp_path):
    path = tmp_path / "contract.json"
    path.write_text("[1, 2, 3]", encoding="utf-8")
    with pytest.raises(ValueError, match="object"):
        load_model_touch_contract(path)


# --------------------------------------------------------------------------- #
# Digest validation (each field independently; unchanged state/files)
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("field", ["scene_revision_digest", "acm_digest", "robot_state_digest"])
@pytest.mark.parametrize(
    "value",
    [None, "", "0" * 64, "A" * 64, "xyz", "1" * 63, "1" * 65, 123456],
)
def test_digest_invalid_forms_rejected_without_mutation(field, value, tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(jsonl_path=jsonl)
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    scene[field] = value
    with pytest.raises(ValueError, match="digest"):
        journal.record_diff("fixture-ready", scene)
    assert journal._records == []
    assert journal._last_scene is None
    assert not jsonl.exists()


@pytest.mark.parametrize("field", ["scene_revision_digest", "acm_digest", "robot_state_digest"])
@pytest.mark.parametrize(
    "value",
    [None, "", "0" * 64, "A" * 64, "xyz"],
)
def test_append_validates_digests_directly(field, value, tmp_path):
    """_append independently validates the three digests before any mutation."""
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(jsonl_path=jsonl)
    poisoned = _scene(1, 1.0, world=("sim_fixture/table",))
    poisoned[field] = value
    with pytest.raises(ValueError, match="digest"):
        journal._append("fixture-ready", poisoned, frame_index=1, timestamp=1.0)
    assert journal._records == []
    assert journal._last_scene is None
    assert not jsonl.exists()


# --------------------------------------------------------------------------- #
# Numeric validation (bool / negative / nonfinite)
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "field,value",
    [
        ("scene_sequence", True),
        ("scene_sequence", -1),
        ("frame_index", False),
        ("frame_index", -2),
        ("scene_timestamp", math.inf),
        ("scene_timestamp", math.nan),
        ("scene_timestamp", -1.0),
        ("timestamp", math.inf),
        ("timestamp", float("nan")),
        ("timestamp", -0.5),
    ],
)
def test_numeric_validation_rejects_bool_negative_nonfinite(field, value):
    journal = _journal()
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    scene[field] = value
    with pytest.raises(ValueError):
        journal.record_diff("fixture-ready", scene)
    assert journal._records == []


def test_duplicate_world_ids_rejected():
    journal = _journal()
    scene = _scene(1, 1.0, world=("sim_fixture/table", "sim_fixture/table"))
    with pytest.raises(ValueError, match="duplicate"):
        journal.record_diff("fixture-ready", scene)
    assert journal._records == []


def test_duplicate_attached_ids_rejected():
    journal = _journal()
    scene = _scene(1, 1.0, attached=("pick_and_place/object_mesh", "pick_and_place/object_mesh"))
    with pytest.raises(ValueError, match="duplicate"):
        journal.record_diff("scene-attach", scene)
    assert journal._records == []


def test_nonempty_source_and_fixture_revision_required():
    journal = _journal()
    for field in ("source", "fixture_revision"):
        scene = _scene(1, 1.0, world=("sim_fixture/table",))
        scene[field] = ""
        with pytest.raises(ValueError, match=field):
            journal.record_diff("fixture-ready", scene)
        assert journal._records == []


def test_fixture_revision_mismatch_rejected():
    journal = _journal()
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    scene["fixture_revision"] = "other-revision"
    with pytest.raises(ValueError, match="revision mismatch"):
        journal.record_diff("fixture-ready", scene)


def test_missing_required_fields_rejected():
    journal = _journal()
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    del scene["acm_digest"]
    with pytest.raises(ValueError, match="missing required"):
        journal.record_diff("fixture-ready", scene)


def test_non_mapping_scene_rejected():
    journal = _journal()
    with pytest.raises(TypeError, match="mappings"):
        journal.record_diff("fixture-ready", [1, 2, 3])


# --------------------------------------------------------------------------- #
# Recursive physics-key leakage
# --------------------------------------------------------------------------- #


def test_top_level_physics_key_rejected():
    journal = _journal()
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    scene["contacts"] = []
    with pytest.raises(ValueError, match="physics"):
        journal.record_diff("fixture-ready", scene)
    assert journal._records == []


def test_nested_physics_key_rejected():
    journal = _journal()
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    scene["touch_links"] = {"pick_and_place/object_mesh": [{"pose": 1.0}]}
    with pytest.raises(ValueError, match="physics"):
        journal.record_diff("scene-attach", scene)
    assert journal._records == []


def test_physics_key_in_attached_links_rejected():
    journal = _journal()
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    scene["attached_links"] = {"pick_and_place/object_mesh": {"force": 5.0}}
    with pytest.raises(ValueError, match="physics"):
        journal.record_diff("fixture-ready", scene)
    assert journal._records == []


@pytest.mark.parametrize(
    "key",
    ["contact", "contacts", "force", "forces", "object_pose", "evaluator_metric", "verdict", "physical_grasp_verified"],
)
def test_physics_key_variants_rejected(key):
    journal = _journal()
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    scene[key] = {"anything": 1}
    with pytest.raises(ValueError, match="physics"):
        journal.record_diff("fixture-ready", scene)


# --------------------------------------------------------------------------- #
# Transactional rollback
# --------------------------------------------------------------------------- #


def test_rejected_record_diff_rolls_back_last_scene(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(jsonl_path=jsonl)
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    last_before = journal._last_scene
    bad = _scene(2, 1.0, world=("sim_fixture/table",))  # non-monotonic timestamp
    with pytest.raises(ValueError, match="monotonic"):
        journal.record_diff("before-pick", bad)
    assert journal._last_scene is last_before
    assert len(journal._records) == 1
    assert jsonl.read_text(encoding="utf-8").count("\n") == 1


def test_forbidden_event_rejected_without_mutation(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(forbidden_events=("before-release",), jsonl_path=jsonl)
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    with pytest.raises(ValueError, match="forbidden"):
        journal.snapshot("before-release", frame_index=2, timestamp=2.0)
    assert len(journal._records) == 1
    assert jsonl.read_text(encoding="utf-8").count("\n") == 1


def test_snapshot_before_first_scene_rejected():
    journal = _journal()
    with pytest.raises(RuntimeError, match="snapshot"):
        journal.snapshot("before-pick", frame_index=1, timestamp=1.0)


def test_empty_event_rejected():
    journal = _journal()
    with pytest.raises(ValueError, match="forbidden or empty"):
        journal.record_diff("", _scene(1, 1.0, world=("sim_fixture/table",)))


def test_unknown_transition_rejected():
    journal = _journal()
    with pytest.raises(ValueError, match="unknown"):
        journal.assert_transition(_scene(1, 1.0), _scene(2, 2.0), expected="teleport")


# --------------------------------------------------------------------------- #
# Graph-evidence validation
# --------------------------------------------------------------------------- #


def test_valid_graph_evidence_passes_and_is_retained():
    graph = _valid_graph()
    validated = validate_graph_evidence(graph)
    assert validated["node_name"] == "/tinker_integrated_gate_executor"
    assert validated["namespace"] == "/"
    assert validated["remap_table"] == {}
    assert set(validated["topics"]) == {
        "/planning_scene", "/monitored_planning_scene", "/sim/status/planning_scene_fixture",
    }
    assert set(validated["services"]) == {"/get_planning_scene", "/apply_planning_scene"}
    fixture = validated["topics"]["/sim/status/planning_scene_fixture"]
    assert fixture["publishers"] == [{"node": "/fixture_planning_scene", "node_namespace": "/"}]
    assert fixture["payload"] == _fixture_status_payload()  # exact canonical string retained
    assert fixture["payload_parsed"]["target_handoff"] == TARGET_HANDOFF


def test_graph_missing_topic_rejected():
    graph = _valid_graph()
    del graph["topics"]["/planning_scene"]
    with pytest.raises(ValueError, match="topic"):
        validate_graph_evidence(graph)


def test_graph_wrong_topic_type_rejected():
    graph = _valid_graph()
    graph["topics"]["/planning_scene"]["type"] = "wrong_msgs/msg/Wrong"
    with pytest.raises(ValueError, match="type"):
        validate_graph_evidence(graph)


def test_graph_wrong_requested_qos_rejected():
    graph = _valid_graph()
    graph["topics"]["/planning_scene"]["requested_qos"]["durability"] = "VOLATILE"
    with pytest.raises(ValueError, match="QoS"):
        validate_graph_evidence(graph)


def test_graph_wrong_offered_qos_rejected():
    graph = _valid_graph()
    graph["topics"]["/monitored_planning_scene"]["offered_qos"]["depth"] = 50
    with pytest.raises(ValueError, match="QoS"):
        validate_graph_evidence(graph)


def test_graph_missing_service_rejected():
    graph = _valid_graph()
    del graph["services"]["/apply_planning_scene"]
    with pytest.raises(ValueError, match="service"):
        validate_graph_evidence(graph)


def test_graph_wrong_service_type_rejected():
    graph = _valid_graph()
    graph["services"]["/get_planning_scene"]["type"] = "wrong_msgs/srv/Wrong"
    with pytest.raises(ValueError, match="type"):
        validate_graph_evidence(graph)


def test_graph_wrong_service_qos_rejected():
    graph = _valid_graph()
    graph["services"]["/get_planning_scene"]["requested_qos"]["durability"] = "TRANSIENT_LOCAL"
    with pytest.raises(ValueError, match="QoS"):
        validate_graph_evidence(graph)


def test_graph_fixture_wrong_publisher_cardinality_rejected():
    graph = _valid_graph()
    graph["topics"]["/sim/status/planning_scene_fixture"]["publishers"].append(
        {"node": "/other", "node_namespace": "/"}
    )
    with pytest.raises(ValueError, match="publisher"):
        validate_graph_evidence(graph)


def test_graph_fixture_wrong_publisher_source_rejected():
    graph = _valid_graph()
    graph["topics"]["/sim/status/planning_scene_fixture"]["publishers"] = [
        {"node": "/not_fixture", "node_namespace": "/"}
    ]
    with pytest.raises(ValueError, match="fixture_planning_scene"):
        validate_graph_evidence(graph)


def test_graph_missing_publisher_metadata_rejected():
    graph = _valid_graph()
    graph["topics"]["/planning_scene"]["publishers"] = []
    with pytest.raises(ValueError, match="endpoint"):
        validate_graph_evidence(graph)


def test_graph_missing_subscriber_metadata_rejected():
    graph = _valid_graph()
    graph["topics"]["/planning_scene"]["subscribers"] = []
    with pytest.raises(ValueError, match="endpoint"):
        validate_graph_evidence(graph)


def test_graph_payload_only_claims_rejected():
    graph = _valid_graph()
    # A payload-only claim without real endpoint metadata must fail.
    graph["topics"]["/planning_scene"]["publishers"] = [
        {"node": "", "node_namespace": ""}
    ]
    with pytest.raises(ValueError, match="endpoint"):
        validate_graph_evidence(graph)


def test_graph_wrong_node_name_rejected():
    graph = _valid_graph()
    graph["node_name"] = "/wrong_node"
    with pytest.raises(ValueError, match="node_name"):
        validate_graph_evidence(graph)


def test_graph_wrong_namespace_rejected():
    graph = _valid_graph()
    graph["namespace"] = "/xarm"
    with pytest.raises(ValueError, match="namespace"):
        validate_graph_evidence(graph)


def test_graph_wrong_remap_rejected():
    graph = _valid_graph()
    graph["remap_table"] = {"/planning_scene": "/ps"}
    with pytest.raises(ValueError, match="remap"):
        validate_graph_evidence(graph)


def test_graph_non_mapping_rejected():
    with pytest.raises(TypeError, match="mapping"):
        validate_graph_evidence([1, 2, 3])


def test_graph_payload_wrong_handoff_rejected():
    graph = _valid_graph()
    status = json.loads(graph["topics"]["/sim/status/planning_scene_fixture"]["payload"])
    status["target_handoff"] = "other/thing"
    graph["topics"]["/sim/status/planning_scene_fixture"]["payload"] = json.dumps(
        status, sort_keys=True, separators=(",", ":")
    )
    with pytest.raises(ValueError, match="handoff"):
        validate_graph_evidence(graph)


def test_graph_payload_not_canonical_encoding_rejected():
    graph = _valid_graph()
    status = json.loads(graph["topics"]["/sim/status/planning_scene_fixture"]["payload"])
    graph["topics"]["/sim/status/planning_scene_fixture"]["payload"] = json.dumps(
        status, sort_keys=True, indent=2
    )
    with pytest.raises(ValueError, match="canonical"):
        validate_graph_evidence(graph)


def test_graph_payload_extra_field_rejected():
    graph = _valid_graph()
    status = json.loads(graph["topics"]["/sim/status/planning_scene_fixture"]["payload"])
    status["extra"] = "boom"
    graph["topics"]["/sim/status/planning_scene_fixture"]["payload"] = json.dumps(
        status, sort_keys=True, separators=(",", ":")
    )
    with pytest.raises(ValueError, match="field set"):
        validate_graph_evidence(graph)


def test_graph_payload_not_object_rejected():
    graph = _valid_graph()
    graph["topics"]["/sim/status/planning_scene_fixture"]["payload"] = "[1, 2]"
    with pytest.raises(ValueError, match="object"):
        validate_graph_evidence(graph)


def test_graph_payload_missing_rejected():
    graph = _valid_graph()
    del graph["topics"]["/sim/status/planning_scene_fixture"]["payload"]
    with pytest.raises(ValueError, match="payload"):
        validate_graph_evidence(graph)


def test_payload_never_substitutes_for_graph_ownership():
    # Even a perfectly valid payload must fail when the publisher source is wrong.
    graph = _valid_graph()
    graph["topics"]["/sim/status/planning_scene_fixture"]["publishers"] = [
        {"node": "/imposter", "node_namespace": "/"}
    ]
    with pytest.raises(ValueError, match="fixture_planning_scene"):
        validate_graph_evidence(graph)


def test_fixture_payload_cross_checks_bridge_canonical_encoding():
    canonical = pytest.importorskip("tinker_sim_bridge.fixture_contract").canonical_json
    from planning_scene_journal import _canonical_json_bytes

    status = json.loads(_fixture_status_payload())
    assert canonical(status).decode("utf-8") == _fixture_status_payload()
    assert _canonical_json_bytes(status).decode("utf-8") == _fixture_status_payload()


# --------------------------------------------------------------------------- #
# Durability: JSONL + final JSON, atomicity, no temp residue
# --------------------------------------------------------------------------- #


def test_valid_digests_write_canonical_jsonl(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(jsonl_path=jsonl)
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    assert len(journal._records) == 1
    lines = [line for line in jsonl.read_text(encoding="utf-8").splitlines() if line]
    assert len(lines) == 1
    record = json.loads(lines[0])
    assert record["scene_sequence"] == 1
    assert record["frame_index"] == 1
    assert record["journal_sequence"] == 1
    assert lines[0] == json.dumps(record, sort_keys=True, separators=(",", ":"))


def test_jsonl_durability_canonical_records_in_order(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(jsonl_path=jsonl)
    for seq in range(1, 4):
        journal.record_diff(f"event-{seq}", _scene(seq, float(seq), world=("sim_fixture/table",)))
    lines = [line for line in jsonl.read_text(encoding="utf-8").splitlines() if line]
    assert len(lines) == 3
    for record, line in zip(journal._records, lines):
        assert line == json.dumps(record, sort_keys=True, separators=(",", ":"))
        assert json.loads(line) == record


def test_finalize_atomic_write_no_temp_residue(tmp_path):
    journal = _journal(jsonl_path=tmp_path / "planning-scene.jsonl")
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    final_path = tmp_path / "planning-scene.json"
    final = journal.finalize("diagnostic-pass", json_path=final_path)
    assert final_path.exists()
    written = json.loads(final_path.read_text(encoding="utf-8"))
    assert written == final
    assert written["schema_version"] == 1
    assert written["status"] == "diagnostic-pass"
    assert written["authority"] == "physics_truth"
    leftovers = [p for p in tmp_path.iterdir() if ".tmp" in p.name]
    assert leftovers == []


def test_finalize_with_graph_retains_evidence(tmp_path):
    journal = _journal()
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    final_path = tmp_path / "planning-scene.json"
    final = journal.finalize("diagnostic-pass", graph=_valid_graph(), json_path=final_path)
    assert final["graph"]["node_name"] == "/tinker_integrated_gate_executor"
    assert final["graph"]["topics"]["/sim/status/planning_scene_fixture"]["payload"] == _fixture_status_payload()
    assert final["graph"]["topics"]["/sim/status/planning_scene_fixture"]["payload_parsed"]["target_handoff"] == TARGET_HANDOFF
    written = json.loads(final_path.read_text(encoding="utf-8"))
    assert written == final


def test_finalize_rejects_bad_graph_without_writing(tmp_path):
    journal = _journal()
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    final_path = tmp_path / "planning-scene.json"
    bad = _valid_graph()
    bad["topics"]["/planning_scene"]["type"] = "wrong/msg/Wrong"
    with pytest.raises(ValueError, match="type"):
        journal.finalize("diagnostic-pass", graph=bad, json_path=final_path)
    assert not final_path.exists()


def test_failed_finalize_never_replaces_existing_artifact(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    final_path = tmp_path / "planning-scene.json"
    journal = _journal(jsonl_path=jsonl)
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    journal.finalize("diagnostic-pass", json_path=final_path)
    first_bytes = final_path.read_bytes()

    journal2 = _journal(required_event_order=POSITIVE_ORDER)
    journal2.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    with pytest.raises(ValueError, match="required event order"):
        journal2.finalize("diagnostic-pass", json_path=final_path)
    assert final_path.read_bytes() == first_bytes
    leftovers = [p for p in tmp_path.iterdir() if ".tmp" in p.name]
    assert leftovers == []


def test_rejected_finalize_does_not_change_jsonl(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(required_event_order=POSITIVE_ORDER, jsonl_path=jsonl)
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    before_bytes = jsonl.read_bytes()
    with pytest.raises(ValueError, match="required event order"):
        journal.finalize("diagnostic-pass")
    assert jsonl.read_bytes() == before_bytes


# --------------------------------------------------------------------------- #
# Event ordering, duplicates, forbidden events, ownership
# --------------------------------------------------------------------------- #


def test_positive_order_exact_once(tmp_path):
    journal = _journal(required_event_order=POSITIVE_ORDER, jsonl_path=tmp_path / "p.jsonl")
    world = ("sim_fixture/table", "pick_and_place/object_mesh")
    attached_world = ("sim_fixture/table",)
    target = ("pick_and_place/object_mesh",)
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=world))
    journal.record_diff("before-pick", _scene(2, 2.0, world=world))
    for seq, event in enumerate(("scene-attach", "lift-complete", "transport", "before-release"), start=3):
        journal.record_diff(event, _scene(seq, float(seq), world=attached_world, attached=target))
    for seq, event in enumerate(("scene-detach", "released-settled", "teardown"), start=7):
        journal.record_diff(event, _scene(seq, float(seq), world=world))
    final = journal.finalize("diagnostic-pass", json_path=tmp_path / "planning-scene.json")
    assert final["events"] == list(POSITIVE_ORDER)
    assert final["records"][-1]["event"] == "teardown"
    assert final["records"][2]["attached_ids"] == ["pick_and_place/object_mesh"]
    assert final["records"][6]["owned_ids"] == ["sim_fixture/table", "pick_and_place/object_mesh"]


def test_duplicate_required_event_rejected():
    journal = _journal(required_event_order=("fixture-ready", "fixture-ready"))
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    with pytest.raises(ValueError, match="required event order"):
        journal.finalize("diagnostic-pass")


def test_teardown_not_final_rejected():
    journal = _journal(required_event_order=POSITIVE_ORDER)
    world = ("sim_fixture/table", "pick_and_place/object_mesh")
    attached_world = ("sim_fixture/table",)
    target = ("pick_and_place/object_mesh",)
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=world))
    journal.record_diff("before-pick", _scene(2, 2.0, world=world))
    for seq, event in enumerate(("scene-attach", "lift-complete", "transport", "before-release"), start=3):
        journal.record_diff(event, _scene(seq, float(seq), world=attached_world, attached=target))
    for seq, event in enumerate(("scene-detach", "released-settled", "teardown"), start=7):
        journal.record_diff(event, _scene(seq, float(seq), world=world))
    journal.snapshot("after-teardown", frame_index=100, timestamp=100.0)
    with pytest.raises(ValueError, match="required event order"):
        journal.finalize("diagnostic-pass")


def test_negative_prefix_without_teardown_is_allowed():
    prefix = ("fixture-ready", "before-pick", "scene-attach", "lift-complete", "transport")
    journal = _journal(required_event_order=prefix)
    world = ("sim_fixture/table", "pick_and_place/object_mesh")
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=world))
    journal.record_diff("before-pick", _scene(2, 2.0, world=world))
    for seq, event in enumerate(("scene-attach", "lift-complete", "transport"), start=3):
        journal.record_diff(event, _scene(seq, float(seq), world=("sim_fixture/table",), attached=("pick_and_place/object_mesh",)))
    final = journal.finalize("diagnostic-pass")
    assert final["events"] == list(prefix)


def test_out_of_order_required_events_rejected():
    journal = _journal(required_event_order=("fixture-ready", "before-pick", "scene-attach"))
    journal.record_diff("before-pick", _scene(1, 1.0, world=("pick_and_place/object_mesh",)))
    journal.record_diff("scene-attach", _scene(2, 2.0, world=(), attached=("pick_and_place/object_mesh",)))
    with pytest.raises(ValueError, match="required event order"):
        journal.finalize("diagnostic-pass")


def test_forbidden_event_fails_at_record_time():
    journal = _journal(forbidden_events=("scene-detach",))
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    with pytest.raises(ValueError, match="forbidden"):
        journal.record_diff("scene-detach", _scene(2, 2.0, world=("sim_fixture/table",)))
    assert [r["event"] for r in journal._records] == ["fixture-ready"]


def test_detach_requires_target_back_in_world():
    journal = _journal()
    attached = _scene(1, 1.0, attached=("pick_and_place/object_mesh",))
    gone = _scene(2, 2.0, world=("sim_fixture/table",))
    with pytest.raises(ValueError, match="world"):
        journal.assert_transition(attached, gone, expected="scene-detach")


def test_attach_requires_target_actually_attached():
    journal = _journal()
    before = _scene(1, 1.0, world=("pick_and_place/object_mesh",))
    after = _scene(2, 2.0, world=("pick_and_place/object_mesh",))
    with pytest.raises(ValueError, match="scene-attach"):
        journal.assert_transition(before, after, expected="scene-attach")


def test_attached_object_without_link_mapping_rejected():
    journal = _journal()
    scene = _scene(1, 1.0, attached=("pick_and_place/object_mesh",))
    del scene["attached_links"]["pick_and_place/object_mesh"]
    with pytest.raises(ValueError, match="no attach link"):
        journal.record_diff("scene-attach", scene)
    assert journal._records == []


# --------------------------------------------------------------------------- #
# Fix round 1 (F1.1): event labels must be true of the recorded scene
# --------------------------------------------------------------------------- #


def test_public_attach_requires_target_actually_attached():
    journal = _journal()
    journal.record_diff("before-pick", _scene(1, 1.0, world=("pick_and_place/object_mesh",)))
    bad = _scene(2, 2.0, world=("pick_and_place/object_mesh",))
    with pytest.raises(ValueError, match="scene-attach"):
        journal.record_diff("scene-attach", bad)
    assert [r["event"] for r in journal._records] == ["before-pick"]


def test_public_attach_rejects_target_still_in_world():
    journal = _journal()
    journal.record_diff("before-pick", _scene(1, 1.0, world=("pick_and_place/object_mesh", "sim_fixture/table")))
    bad = _scene(2, 2.0, world=("pick_and_place/object_mesh", "sim_fixture/table"), attached=("pick_and_place/object_mesh",))
    with pytest.raises(ValueError, match="both world and attached"):
        journal.record_diff("scene-attach", bad)
    assert len(journal._records) == 1


def test_public_attach_rejects_already_attached_target():
    journal = _journal()
    journal.record_diff("scene-attach", _scene(1, 1.0, attached=("pick_and_place/object_mesh",)))
    again = _scene(2, 2.0, world=(), attached=("pick_and_place/object_mesh",))
    with pytest.raises(ValueError, match="scene-attach"):
        journal.record_diff("scene-attach", again)
    assert len(journal._records) == 1


def test_public_detach_requires_prior_attach():
    journal = _journal()
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("pick_and_place/object_mesh",)))
    detach = _scene(2, 2.0, world=("pick_and_place/object_mesh",))
    with pytest.raises(ValueError, match="scene-detach"):
        journal.record_diff("scene-detach", detach)
    assert len(journal._records) == 1


def test_public_detach_requires_target_in_world():
    journal = _journal()
    journal.record_diff("scene-attach", _scene(1, 1.0, attached=("pick_and_place/object_mesh",)))
    vanished = _scene(2, 2.0, world=())
    with pytest.raises(ValueError, match="scene-detach"):
        journal.record_diff("scene-detach", vanished)
    assert len(journal._records) == 1


@pytest.mark.parametrize("event", ["scene-attach", "scene-detach", "task-cleanup"])
def test_snapshot_rejects_transition_labels(event):
    journal = _journal()
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    with pytest.raises(ValueError, match="requires a PlanningScene diff"):
        journal.snapshot(event, frame_index=2, timestamp=2.0)
    assert len(journal._records) == 1


# --------------------------------------------------------------------------- #
# Fix round 1 (F1.2): returned evidence must not alias stored journal state
# --------------------------------------------------------------------------- #


def test_returned_record_mutation_is_isolated(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(jsonl_path=jsonl)
    r = journal.record_diff(
        "fixture-ready",
        _scene(1, 1.0, world=("sim_fixture/table", "pick_and_place/object_mesh")),
    )
    r["owned_ids"].append("injected/object")
    r["attached_ids"].append("injected/attached")
    r["attached_links"]["injected/attached"] = "link_tcp"
    r["touch_links"]["injected/attached"] = list(MODEL_TOUCH_LINKS)
    stored = journal._records[0]
    assert stored["owned_ids"] == ["sim_fixture/table", "pick_and_place/object_mesh"]
    assert stored["attached_ids"] == []
    assert "injected/attached" not in stored["attached_links"]
    assert "injected/attached" not in stored["touch_links"]
    assert jsonl.read_text(encoding="utf-8") == json.dumps(stored, sort_keys=True, separators=(",", ":")) + "\n"
    final_path = tmp_path / "planning-scene.json"
    final = journal.finalize("diagnostic-pass", json_path=final_path)
    assert "injected/object" not in final["records"][0]["owned_ids"]
    written = json.loads(final_path.read_text(encoding="utf-8"))
    assert "injected/object" not in written["records"][0]["owned_ids"]


def test_snapshot_result_mutation_is_isolated(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(jsonl_path=jsonl)
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    s = journal.snapshot("before-pick", frame_index=2, timestamp=2.0)
    s["owned_ids"].append("injected/object")
    assert journal._records[1]["owned_ids"] == ["sim_fixture/table"]
    assert jsonl.read_text(encoding="utf-8").count("\n") == 2


def test_finalize_records_are_deep_copied(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(jsonl_path=jsonl)
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    final = journal.finalize("diagnostic-pass")
    stored_owned = list(journal._records[0]["owned_ids"])
    final["records"][0]["owned_ids"].append("injected/object")
    final["records"][0]["attached_links"]["injected"] = "link_tcp"
    assert journal._records[0]["owned_ids"] == stored_owned
    assert "injected" not in journal._records[0]["attached_links"]


# --------------------------------------------------------------------------- #
# Fix round 1 (F1.3): finalization rejects empty/extra/duplicate/spurious
# --------------------------------------------------------------------------- #


def test_finalize_rejects_empty_journal():
    journal = _journal()
    with pytest.raises(ValueError, match="empty"):
        journal.finalize("diagnostic-pass")


def test_finalize_rejects_extra_event():
    journal = _journal(required_event_order=("fixture-ready", "before-pick"))
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    journal.record_diff("before-pick", _scene(2, 2.0, world=("sim_fixture/table",)))
    journal.record_diff("spurious-event", _scene(3, 3.0, world=("sim_fixture/table",)))
    with pytest.raises(ValueError, match="required event order"):
        journal.finalize("diagnostic-pass")


def test_finalize_rejects_spurious_negative_teardown():
    prefix = ("fixture-ready", "before-pick", "transport")
    journal = _journal(required_event_order=prefix)
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    journal.record_diff("before-pick", _scene(2, 2.0, world=("sim_fixture/table",)))
    journal.record_diff("teardown", _scene(3, 3.0, world=("sim_fixture/table",)))
    with pytest.raises(ValueError, match="required event order"):
        journal.finalize("diagnostic-pass")


# --------------------------------------------------------------------------- #
# Fix round 1 (F1.4): object disappearance cannot bypass ownership by relabeling
# --------------------------------------------------------------------------- #


def test_fixture_disappearance_under_ordinary_label_rejected():
    journal = _journal()
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table", "sim_fixture/shelf")))
    after = _scene(2, 2.0, world=("sim_fixture/table",))
    with pytest.raises(PermissionError, match="sim_fixture/shelf"):
        journal.record_diff("before-pick", after)
    assert len(journal._records) == 1


def test_other_node_disappearance_under_ordinary_label_rejected():
    journal = _journal()
    journal.record_diff("fixture-ready", _scene(1, 1.0, world=("other_node/keep", "pick_and_place/temp")))
    after = _scene(2, 2.0, world=("pick_and_place/temp",))
    with pytest.raises(PermissionError, match="other_node/keep"):
        journal.record_diff("scene-update", after)
    assert len(journal._records) == 1


def test_teardown_may_remove_fixture_objects():
    journal = _journal()
    before = _scene(1, 1.0, world=("sim_fixture/table", "sim_fixture/shelf"))
    after = _scene(2, 2.0, world=())
    journal.record_diff("fixture-ready", before)
    journal.record_diff("teardown", after)
    assert journal.finalize("diagnostic-pass")["records"][-1]["owned_ids"] == []


def test_teardown_cannot_remove_other_node_objects():
    journal = _journal()
    before = _scene(1, 1.0, world=("sim_fixture/table", "other_node/keep"))
    after = _scene(2, 2.0, world=("sim_fixture/table",))
    journal.record_diff("fixture-ready", before)
    with pytest.raises(PermissionError, match="other_node/keep"):
        journal.record_diff("teardown", after)
    assert len(journal._records) == 1


# --------------------------------------------------------------------------- #
# Fix round 1 (F1.5): no append onto stale JSONL
# --------------------------------------------------------------------------- #


def test_existing_nonempty_jsonl_rejected(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    jsonl.write_text('{"stale": true}\n', encoding="utf-8")
    before = jsonl.read_bytes()
    journal = _journal(jsonl_path=jsonl)
    with pytest.raises(ValueError, match="already contains"):
        journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    assert journal._records == []
    assert jsonl.read_bytes() == before


def test_zero_length_jsonl_accepted(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    jsonl.write_text("", encoding="utf-8")
    journal = _journal(jsonl_path=jsonl)
    record = journal.record_diff("fixture-ready", _scene(1, 1.0, world=("sim_fixture/table",)))
    assert record["journal_sequence"] == 1
    assert jsonl.read_text(encoding="utf-8").count("\n") == 1


# --------------------------------------------------------------------------- #
# Fix round 1 (F1.6): close graph and payload strictness gaps
# --------------------------------------------------------------------------- #


def _set_payload(graph, **changes):
    status = json.loads(graph["topics"]["/sim/status/planning_scene_fixture"]["payload"])
    status.update(changes)
    graph["topics"]["/sim/status/planning_scene_fixture"]["payload"] = json.dumps(
        status, sort_keys=True, separators=(",", ":")
    )


def test_graph_extra_topic_rejected():
    graph = _valid_graph()
    graph["topics"]["/extra/topic"] = {
        "type": "std_msgs/msg/String",
        "requested_qos": {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1},
        "offered_qos": {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1},
        "publishers": [{"node": "/x"}],
        "subscribers": [{"node": "/tinker_integrated_gate_executor"}],
    }
    with pytest.raises(ValueError, match="topics"):
        validate_graph_evidence(graph)


def test_graph_extra_service_rejected():
    graph = _valid_graph()
    graph["services"]["/extra/service"] = {
        "type": "std_srvs/srv/Trigger",
        "requested_qos": {"reliability": "RELIABLE", "durability": "VOLATILE"},
        "offered_qos": {"reliability": "RELIABLE", "durability": "VOLATILE"},
        "servers": [{"node": "/x"}],
        "clients": [{"node": "/tinker_integrated_gate_executor"}],
    }
    with pytest.raises(ValueError, match="services"):
        validate_graph_evidence(graph)


def test_graph_recorder_absent_from_subscribers_rejected():
    graph = _valid_graph()
    graph["topics"]["/planning_scene"]["subscribers"] = [{"node": "/someone_else", "node_namespace": "/"}]
    with pytest.raises(ValueError, match="subscribe"):
        validate_graph_evidence(graph)


def test_graph_recorder_absent_from_clients_rejected():
    graph = _valid_graph()
    graph["services"]["/get_planning_scene"]["clients"] = [{"node": "/someone_else", "node_namespace": "/"}]
    with pytest.raises(ValueError, match="called by"):
        validate_graph_evidence(graph)


def test_graph_boolean_depth_rejected():
    graph = _valid_graph()
    graph["topics"]["/planning_scene"]["requested_qos"]["depth"] = True
    with pytest.raises(ValueError, match="QoS"):
        validate_graph_evidence(graph)


def test_graph_extra_qos_key_rejected():
    graph = _valid_graph()
    graph["topics"]["/planning_scene"]["requested_qos"]["history"] = "KEEP_ALL"
    with pytest.raises(ValueError, match="QoS"):
        validate_graph_evidence(graph)


def test_graph_payload_string_published_at_rejected():
    graph = _valid_graph()
    _set_payload(graph, published_at="7.5")
    with pytest.raises(ValueError, match="published_at"):
        validate_graph_evidence(graph)


def test_graph_payload_boolean_sequence_rejected():
    graph = _valid_graph()
    _set_payload(graph, sequence=True)
    with pytest.raises(ValueError, match="sequence"):
        validate_graph_evidence(graph)


def test_graph_payload_duplicate_owned_ids_rejected():
    graph = _valid_graph()
    _set_payload(graph, owned_ids=["sim_fixture/a", "sim_fixture/a"])
    with pytest.raises(ValueError, match="duplicate"):
        validate_graph_evidence(graph)


def test_graph_payload_non_fixture_owned_id_rejected():
    graph = _valid_graph()
    _set_payload(graph, owned_ids=["other_node/thing"])
    with pytest.raises(ValueError, match="sim_fixture"):
        validate_graph_evidence(graph)


def test_graph_input_mutation_after_validation_is_isolated():
    graph = _valid_graph()
    validated = validate_graph_evidence(graph)
    graph["topics"]["/planning_scene"]["requested_qos"]["depth"] = 99
    graph["topics"]["/sim/status/planning_scene_fixture"]["payload"] = "[]"
    graph["topics"]["/planning_scene"]["subscribers"][0]["node"] = "/someone_else"
    assert validated["topics"]["/planning_scene"]["requested_qos"]["depth"] == 1
    assert validated["topics"]["/sim/status/planning_scene_fixture"]["payload"] == _fixture_status_payload()
    assert validated["topics"]["/planning_scene"]["subscribers"][0]["node"] == RECORDER_NODE


def test_validated_graph_output_mutation_does_not_affect_module_constants():
    validated = validate_graph_evidence(_valid_graph())
    validated["topics"]["/planning_scene"]["requested_qos"]["depth"] = 50
    validated["topics"]["/planning_scene"]["subscribers"][0]["node"] = "/someone_else"
    again = validate_graph_evidence(_valid_graph())
    assert again["topics"]["/planning_scene"]["requested_qos"]["depth"] == 1
    assert again["topics"]["/planning_scene"]["subscribers"][0]["node"] == RECORDER_NODE


# --------------------------------------------------------------------------- #
# Fix round 1 (F1.7): strict nested scene types and _append() defense in depth
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "field,value",
    [
        ("scene_sequence", True),
        ("scene_sequence", -1),
        ("frame_index", False),
        ("frame_index", -3),
        ("scene_timestamp", math.inf),
        ("scene_timestamp", math.nan),
        ("scene_timestamp", -1.0),
        ("timestamp", math.inf),
        ("timestamp", float("nan")),
        ("timestamp", -0.5),
    ],
)
def test_append_direct_rejects_invalid_numerics(field, value, tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(jsonl_path=jsonl)
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    scene[field] = value
    with pytest.raises(ValueError):
        journal._append("fixture-ready", scene, frame_index=1, timestamp=1.0)
    assert journal._records == []
    assert journal._last_scene is None
    assert not jsonl.exists()


def test_append_direct_rejects_malformed_touch_links(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(jsonl_path=jsonl)
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    scene["touch_links"] = {"pick_and_place/object_mesh": 42}
    with pytest.raises(ValueError, match="touch_links"):
        journal._append("fixture-ready", scene, frame_index=1, timestamp=1.0)
    assert journal._records == []
    assert not jsonl.exists()


def test_append_direct_rejects_malformed_attached_links(tmp_path):
    jsonl = tmp_path / "planning-scene.jsonl"
    journal = _journal(jsonl_path=jsonl)
    scene = _scene(1, 1.0, world=("sim_fixture/table",))
    scene["attached_links"] = {"pick_and_place/object_mesh": 7}
    with pytest.raises(ValueError, match="attached_links"):
        journal._append("fixture-ready", scene, frame_index=1, timestamp=1.0)
    assert journal._records == []
    assert not jsonl.exists()
