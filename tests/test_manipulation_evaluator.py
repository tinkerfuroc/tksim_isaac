from __future__ import annotations

import ast
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge"))

from truth_evaluator import TruthEvaluatorCore, TruthFrame  # noqa: E402


def frame(
    timestamp: float,
    object_xyz: list[float] | None,
    tcp_xyz: list[float],
    contacts: list[dict],
    tcp_quaternion: list[float] | None = None,
    object_quaternion: list[float] | None = None,
    scenario: str = "qualification-retention",
    task: str = "retain-cube",
    object_id: str = "qualification_cube",
) -> dict:
    return {
        "schema_version": 1,
        "timestamp": timestamp,
        "scenario": scenario,
        "task": task,
        "robot": {
            "base_pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            "tcp_pose": {
                "xyz": tcp_xyz,
                "quaternion_xyzw": tcp_quaternion or [0.0, 0.0, 0.0, 1.0],
            },
            "base_twist": {},
            "joint_names": [f"joint_{index}" for index in range(7)],
            "joint_positions": [0.0] * 7,
            "joint_velocities": [0.0] * 7,
            "joint_efforts": [0.0] * 7,
            "safety_stop": False,
        },
        "object": (
            {
                "id": object_id,
                "class_name": "dynamic_cube",
                "pose": {
                    "xyz": object_xyz,
                    "quaternion_xyzw": object_quaternion or [0.0, 0.0, 0.0, 1.0],
                },
                "twist": {"linear": [0.0, 0.0, 0.0], "angular": [0.0, 0.0, 0.0]},
            }
            if object_xyz is not None
            else None
        ),
        "contacts": contacts,
    }


CONTACTS = [
    {"body_a": "left_finger_link", "body_b": "qualification_cube", "normal_force": 1.0},
    {"body_a": "right_finger_link", "body_b": "qualification_cube", "normal_force": 1.0},
]


class ManipulationEvaluatorTest(unittest.TestCase):
    def test_object_free_truth_still_evaluates_robot_and_task(self) -> None:
        result = TruthEvaluatorCore().process(frame(0.0, None, [0.0, 0.0, 0.0], []))
        self.assertFalse(result["metrics"]["object_present"])
        self.assertFalse(result["metrics"]["retained"])
        self.assertEqual(result["task_truth"]["state"], "no-object")
        self.assertFalse(result["task_truth"]["postcondition_satisfied"])

    def test_timestamp_rollback_starts_a_new_valid_epoch(self) -> None:
        evaluator = TruthEvaluatorCore()
        first = evaluator.process(frame(10.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], CONTACTS))
        self.assertFalse(first["metrics"]["retained"])
        evaluator.process(frame(11.0, [0.85, 0.0, 0.9], [0.85, 0.0, 0.9], CONTACTS))
        retained = evaluator.process(
            frame(12.0, [0.85, 0.0, 0.9], [0.85, 0.0, 0.9], CONTACTS)
        )
        self.assertTrue(retained["metrics"]["retained"])
        rolled_back = evaluator.process(
            frame(0.0, [0.85, 0.0, 0.9], [0.85, 0.0, 0.9], CONTACTS)
        )
        self.assertFalse(rolled_back["metrics"]["retained"])
        self.assertAlmostEqual(rolled_back["metrics"]["lift_m"], 0.0)
        self.assertAlmostEqual(rolled_back["metrics"]["stable_window_s"], 0.0)

    def test_scenario_change_resets_retention_baselines(self) -> None:
        evaluator = TruthEvaluatorCore()
        evaluator.process(frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], CONTACTS))
        changed = evaluator.process(
            frame(
                1.0,
                [0.85, 0.0, 0.9],
                [0.85, 0.0, 0.9],
                CONTACTS,
                scenario="different-scenario",
            )
        )
        self.assertAlmostEqual(changed["metrics"]["lift_m"], 0.0)
        self.assertAlmostEqual(changed["metrics"]["translation_m"], 0.0)
        self.assertFalse(changed["metrics"]["retained"])

    def test_task_change_resets_retention_baselines(self) -> None:
        evaluator = TruthEvaluatorCore()
        evaluator.process(frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], CONTACTS))
        changed = evaluator.process(
            frame(
                1.0,
                [0.85, 0.0, 0.9],
                [0.85, 0.0, 0.9],
                CONTACTS,
                task="different-task",
            )
        )
        self.assertAlmostEqual(changed["metrics"]["lift_m"], 0.0)
        self.assertAlmostEqual(changed["metrics"]["translation_m"], 0.0)
        self.assertFalse(changed["metrics"]["retained"])

    def test_object_id_change_resets_retention_baselines(self) -> None:
        evaluator = TruthEvaluatorCore()
        evaluator.process(frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], CONTACTS))
        changed = evaluator.process(
            frame(
                1.0,
                [0.85, 0.0, 0.9],
                [0.85, 0.0, 0.9],
                CONTACTS,
                object_id="replacement_cube",
            )
        )
        self.assertAlmostEqual(changed["metrics"]["lift_m"], 0.0)
        self.assertAlmostEqual(changed["metrics"]["translation_m"], 0.0)
        self.assertFalse(changed["metrics"]["retained"])

    def test_disappearance_clears_state_before_respawn(self) -> None:
        evaluator = TruthEvaluatorCore()
        evaluator.process(frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], CONTACTS))
        missing = evaluator.process(frame(1.0, None, [0.85, 0.0, 0.9], []))
        self.assertFalse(missing["metrics"]["object_present"])
        respawned = evaluator.process(
            frame(2.0, [0.85, 0.0, 0.9], [0.85, 0.0, 0.9], CONTACTS)
        )
        self.assertAlmostEqual(respawned["metrics"]["lift_m"], 0.0)
        self.assertAlmostEqual(respawned["metrics"]["translation_m"], 0.0)
        self.assertAlmostEqual(respawned["metrics"]["stable_window_s"], 0.0)
        self.assertFalse(respawned["metrics"]["retained"])

    def test_lifecycle_change_cannot_carry_retained_state(self) -> None:
        evaluator = TruthEvaluatorCore()
        evaluator.process(frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], CONTACTS))
        lifted = evaluator.process(frame(1.0, [0.85, 0.0, 0.9], [0.85, 0.0, 0.9], CONTACTS))
        self.assertFalse(lifted["metrics"]["retained"])
        retained = evaluator.process(frame(2.0, [0.85, 0.0, 0.9], [0.85, 0.0, 0.9], CONTACTS))
        self.assertTrue(retained["metrics"]["retained"])
        changed = evaluator.process(
            frame(
                3.0,
                [0.85, 0.0, 0.9],
                [0.85, 0.0, 0.9],
                CONTACTS,
                task="new-task",
            )
        )
        self.assertFalse(changed["metrics"]["retained"])
        self.assertEqual(changed["metrics"]["stable_window_s"], 0.0)

    def test_retention_requires_physical_bilateral_contact_and_stable_window(self) -> None:
        evaluator = TruthEvaluatorCore()
        first = evaluator.process(frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], CONTACTS))
        self.assertFalse(first["metrics"]["retained"])
        lifted = evaluator.process(frame(1.0, [0.85, 0.0, 0.9], [0.85, 0.0, 0.9], CONTACTS))
        self.assertFalse(lifted["metrics"]["retained"])
        complete = evaluator.process(frame(2.0, [0.85, 0.0, 0.9], [0.85, 0.0, 0.9], CONTACTS))
        self.assertTrue(complete["metrics"]["bilateral_contact"])
        self.assertAlmostEqual(complete["metrics"]["stable_window_s"], 1.0)
        self.assertTrue(complete["metrics"]["retained"])
        self.assertNotIn("action_success", json.dumps(complete, sort_keys=True))

    def test_unilateral_contact_cannot_claim_retention(self) -> None:
        evaluator = TruthEvaluatorCore()
        unilateral = [CONTACTS[0]]
        for timestamp in (0.0, 1.0, 2.0):
            result = evaluator.process(frame(timestamp, [0.85, 0.0, 0.9], [0.85, 0.0, 0.9], unilateral))
        self.assertFalse(result["metrics"]["bilateral_contact"])
        self.assertFalse(result["metrics"]["retained"])

    def test_retention_relative_transform_is_in_tcp_frame(self) -> None:
        evaluator = TruthEvaluatorCore()
        identity = [0.0, 0.0, 0.0, 1.0]
        quarter_turn = [0.0, 0.0, 2**-0.5, 2**-0.5]
        half_turn = [0.0, 0.0, 1.0, 0.0]
        evaluator.process(frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], CONTACTS))
        rotated = evaluator.process(
            frame(
                1.0,
                [0.65, 0.20, 0.9],
                [0.65, 0.20, 0.9],
                CONTACTS,
                quarter_turn,
                quarter_turn,
            )
        )
        complete = evaluator.process(
            frame(
                2.0,
                [0.45, 0.20, 0.9],
                [0.45, 0.20, 0.9],
                CONTACTS,
                half_turn,
                half_turn,
            )
        )
        self.assertAlmostEqual(rotated["metrics"]["drift_m"], 0.0, places=9)
        self.assertAlmostEqual(rotated["metrics"]["drift_deg"], 0.0, places=6)
        self.assertAlmostEqual(complete["metrics"]["drift_m"], 0.0, places=9)
        self.assertAlmostEqual(complete["metrics"]["drift_deg"], 0.0, places=6)
        self.assertTrue(complete["metrics"]["retained"])

    def test_jsonl_is_append_only_and_contains_raw_frame_and_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "attempt" / "evaluator.jsonl"
            from truth_evaluator import JsonlWriter

            writer = JsonlWriter(path)
            record = TruthEvaluatorCore().process(frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], []))
            writer.write(record)
            writer.close()
            line = json.loads(path.read_text(encoding="utf-8"))
            self.assertIn("frame", line)
            self.assertIn("metrics", line)
            self.assertEqual(line["evaluator_version"], 1)

    def test_consumes_raw_schema_v2_with_command_targets(self) -> None:
        payload = frame(0.0, None, [0.0, 0.0, 0.0], [])
        payload["schema_version"] = 2
        payload["frame_index"] = 42
        payload["command_targets"] = {
            "joint_names": payload["robot"]["joint_names"],
            "positions": [0.1] * 7,
            "gripper": 0.02,
        }

        result = TruthEvaluatorCore().process(payload)

        self.assertEqual(result["evaluator_version"], 1)
        self.assertEqual(result["frame"]["schema_version"], 2)
        self.assertEqual(result["frame"]["command_targets"]["gripper"], 0.02)
        self.assertEqual(result["task_truth"]["state"], "no-object")

    def test_rejects_unknown_version_and_nonfinite_values(self) -> None:
        evaluator = TruthEvaluatorCore()
        invalid = frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], [])
        invalid["schema_version"] = 3
        with self.assertRaises(ValueError):
            evaluator.process(invalid)
        invalid = frame(0.0, [float("nan"), 0.0, 0.8], [0.65, 0.0, 0.8], [])
        with self.assertRaises(ValueError):
            evaluator.process(invalid)

    def test_objects_plural_parses_declared_and_spawned_entities(self) -> None:
        # Regression for Task #11: backend.py's truth_state() puts declared
        # objects first and spawned entities (e.g. bench_soup_..., a
        # spawned-via-/spawn_entity object) after in the plural "objects"
        # array, while "object" (singular) stays objects[0] for back-compat.
        # TruthFrame must parse the full plural list, not just the singular.
        declared = frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], [])["object"]
        spawned = {
            "id": "bench_soup_100_anygrasp_agw_a1",
            "class_name": "spawned_object",
            "pose": {"xyz": [1.2, 0.3, 0.5], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            "twist": {"linear": [0.0, 0.0, 0.0], "angular": [0.0, 0.0, 0.0]},
        }
        payload = frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], [])
        payload["objects"] = [declared, spawned]

        parsed = TruthFrame.from_mapping(payload)

        self.assertEqual(len(parsed.objects), 2)
        self.assertEqual(parsed.objects[0]["id"], declared["id"])
        self.assertEqual(parsed.objects[1]["id"], "bench_soup_100_anygrasp_agw_a1")
        self.assertEqual(parsed.objects[1]["pose"]["position"], (1.2, 0.3, 0.5))
        # Back-compat: the singular alias still exists and equals objects[0].
        self.assertIsNotNone(parsed.object)
        self.assertEqual(parsed.object["id"], declared["id"])

    def test_legacy_singular_object_still_yields_exactly_one_entry(self) -> None:
        payload = frame(0.0, [0.65, 0.0, 0.8], [0.65, 0.0, 0.8], [])
        self.assertNotIn("objects", payload)

        parsed = TruthFrame.from_mapping(payload)

        self.assertEqual(len(parsed.objects), 1)
        self.assertEqual(parsed.objects[0]["id"], payload["object"]["id"])
        self.assertEqual(parsed.object["id"], payload["object"]["id"])

    def test_no_object_yields_empty_objects_tuple(self) -> None:
        payload = frame(0.0, None, [0.0, 0.0, 0.0], [])
        self.assertNotIn("objects", payload)

        parsed = TruthFrame.from_mapping(payload)

        self.assertEqual(parsed.objects, ())
        self.assertIsNone(parsed.object)

    def test_evaluator_node_accepts_separate_raw_jsonl_path_parameter(self) -> None:
        source = (
            ROOT
            / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/truth_evaluator.py"
        ).read_text(encoding="utf-8")
        # The evaluator owns raw-truth persistence and must accept the attempt's
        # physics_truth.jsonl path separately from the evaluated record path.
        self.assertIn('self.declare_parameter("raw_jsonl_path", "")', source)
        self.assertIn('self.declare_parameter("jsonl_path", "")', source)


def _node_writes(node: ast.AST, writer_attr: str) -> bool:
    """True when ``node`` contains a ``<writer_attr>.write(...)`` or ``.close()`` call."""
    for call in ast.walk(node):
        if not isinstance(call, ast.Call):
            continue
        func = call.func
        if not isinstance(func, ast.Attribute):
            continue
        if func.attr not in {"write", "close"}:
            continue
        value = func.value
        if isinstance(value, ast.Attribute) and value.attr == writer_attr:
            return True
    return False


def _has_for_loop_publishing(node: ast.AST, iter_attr: str, pub_attr: str) -> bool:
    """True when ``node`` contains a ``for x in <...>.<iter_attr>:`` loop whose
    body calls ``self.<pub_attr>.publish(...)`` -- i.e. one publish call per
    iterated item, mirroring the existing "for contact in frame.contacts"
    pattern rather than a single publish gated by an "if" check.
    """
    for loop in ast.walk(node):
        if not isinstance(loop, ast.For):
            continue
        iter_node = loop.iter
        if not (isinstance(iter_node, ast.Attribute) and iter_node.attr == iter_attr):
            continue
        for call in ast.walk(loop):
            if not isinstance(call, ast.Call):
                continue
            func = call.func
            if (
                isinstance(func, ast.Attribute)
                and func.attr == "publish"
                and isinstance(func.value, ast.Attribute)
                and func.value.attr == pub_attr
            ):
                return True
    return False


def test_on_truth_publishes_one_object_truth_per_object_in_frame() -> None:
    # Regression for Task #11: contacts are iterated ("for contact in
    # frame.contacts") and one ContactTruth is published per contact, but
    # objects were published from a single "if frame.object is not None"
    # block -- at most one ObjectTruth per frame, so spawned entities past
    # objects[0] never reached /sim/truth/object_state. _on_truth must loop
    # over frame.objects the same way it loops over frame.contacts.
    source = (
        ROOT
        / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/truth_evaluator.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    node_class = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.ClassDef) and node.name == "TruthEvaluatorNode"
    )
    on_truth = next(
        node
        for node in ast.walk(node_class)
        if isinstance(node, ast.FunctionDef) and node.name == "_on_truth"
    )
    assert _has_for_loop_publishing(on_truth, "objects", "_object_pub"), (
        "_on_truth must loop over frame.objects publishing one ObjectTruth "
        "per entry, not just frame.object"
    )


def test_evaluator_persists_raw_and_evaluated_from_same_callback() -> None:
    source = (
        ROOT
        / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/truth_evaluator.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    node_class = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.ClassDef) and node.name == "TruthEvaluatorNode"
    )
    on_truth = next(
        node
        for node in ast.walk(node_class)
        if isinstance(node, ast.FunctionDef) and node.name == "_on_truth"
    )
    close = next(
        node
        for node in ast.walk(node_class)
        if isinstance(node, ast.FunctionDef) and node.name == "close"
    )
    # The validated raw payload and the evaluated record are persisted together
    # in the same received-truth callback, and both writers are closed.
    assert _node_writes(on_truth, "_raw_writer"), "raw payload not written in _on_truth"
    assert _node_writes(on_truth, "_writer"), "evaluated record not written in _on_truth"
    assert _node_writes(close, "_raw_writer"), "raw writer not closed"
    assert _node_writes(close, "_writer"), "evaluated writer not closed"


if __name__ == "__main__":
    unittest.main()
