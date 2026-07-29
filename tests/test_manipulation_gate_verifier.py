from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from validation.manipulation_gate_verifier import SAFETY_TOPIC, verify  # noqa: E402


CONFIG = {
    "schema_version": 1,
    "physics": {"hz": 120.0},
    "test_only_allow_sparse_frames": True,
    "thresholds": {
        "retention_hold_s": 1.0,
        "contact_force_n": 1.0,
    },
    "scenarios": {
        "free-space-fjt": "qualification-free-space",
        "safety-stop": "qualification-free-space",
        "free-gripper": "qualification-free-gripper",
        "obstructed-gripper": "qualification-obstructed-gripper",
        "arm-collision": "qualification-arm-collision",
        "retention": "qualification-retention",
    },
}


def robot(positions=None, velocities=None, safety=False, gripper=None, target=None, tcp_xyz=None):
    return {
        "joint_names": [f"joint_{index}" for index in range(7)],
        "joint_positions": positions or [0.0] * 7,
        "joint_velocities": velocities or [0.0] * 7,
        "joint_efforts": [0.0] * 7,
        "safety_stop": safety,
        "tcp_pose": {"xyz": tcp_xyz or [0.65, 0.0, 0.83], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        **({"drive_joint": gripper} if gripper is not None else {}),
        **({"commanded_positions": target} if target is not None else {}),
    }


def truth(timestamp, scenario, *, positions=None, velocities=None, safety=False, gripper=None, target=None, object_xyz=None, contacts=None, object_velocity=None, tcp_xyz=None):
    if object_xyz is None and scenario in {"qualification-obstructed-gripper", "qualification-retention"}:
        object_xyz = [0.65, 0.0, 0.80]
    obj = None
    if object_xyz is not None:
        obj = {
            "id": "qualification_cube",
            "pose": {"xyz": object_xyz, "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            "twist": {"linear": object_velocity or [0.0, 0.0, 0.0]},
        }
    result = {
        "schema_version": 1,
        "timestamp": timestamp,
        "scenario": scenario,
        "task": "test-task",
        "robot": robot(positions, velocities, safety, gripper, target, tcp_xyz),
        "object": obj,
        "contacts": contacts or [],
        "actuator_limits": {"drive_joint": 20.0},
    }
    if target is not None:
        result["command_targets"] = {
            "joint_names": [f"joint{index}" for index in range(1, 8)],
            "joint_positions": list(target),
        }
    return result


def action(endpoint="/xarm7_traj_controller/follow_joint_trajectory", **values):
    return {"endpoint": endpoint, **values}


def contact(left, right, force=1.0, object_id="qualification_cube"):
    values = []
    if left:
        values.append({"body_a": "left_finger_link", "body_b": object_id, "normal_force": force})
    if right:
        values.append({"body_a": "right_finger_link", "body_b": object_id, "normal_force": force})
    return values


class ManipulationGateVerifierTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.config = self.root / "config.json"
        self.config.write_text(json.dumps(CONFIG), encoding="utf-8")

    def tearDown(self):
        self.tmp.cleanup()

    def write_attempt(self, gate, frames, execution, *, evaluator=None):
        attempt = self.root / gate
        attempt.mkdir()
        for index, frame in enumerate(frames):
            frame.setdefault("frame_index", index)
        execution = list(execution)
        if not any(record.get("event") == "gate_started" for record in execution):
            execution.insert(0, {"event": "gate_started", "simulated_timestamp": frames[0]["timestamp"]})
        if not any(record.get("event") == "terminal" for record in execution):
            execution.append({"event": "terminal", "simulated_timestamp": frames[-1]["timestamp"]})
        (attempt / "physics_truth.jsonl").write_text("\n".join(json.dumps(frame) for frame in frames) + "\n", encoding="utf-8")
        (attempt / "gate-execution.jsonl").write_text("\n".join(json.dumps(record) for record in execution) + "\n", encoding="utf-8")
        if evaluator is not None:
            (attempt / "evaluator.jsonl").write_text("\n".join(json.dumps(record) for record in evaluator) + "\n", encoding="utf-8")
        return attempt

    @staticmethod
    def safety_event(active):
        return {
            "event": "safety_asserted" if active else "safety_cleared",
            "action_endpoint": "/sim/safety/operator",
            "simulated_timestamp": 0.1 if active else 0.7,
            "result": {"active": active},
        }

    def test_free_fjt_passes_from_raw_tracking(self):
        target = [0.05] * 7
        support = [{"body_a": "rear_left_wheel_joint", "body_b": "/World/defaultGroundPlane", "normal_force": 20.0}]
        frames = [truth(0.0, "qualification-free-space", positions=[0.0] * 7, target=target, contacts=support), truth(1.0, "qualification-free-space", positions=target, target=target, contacts=support)]
        attempt = self.write_attempt("fjt", frames, [action(accepted=True), action(success=True, expected_final_positions=target)])
        verdict = verify("free-space-fjt", attempt, self.config)
        self.assertEqual(verdict["status"], "verified-pass")
        self.assertEqual(verdict["checks"][1]["name"], "action_accepted_success")

    def test_action_success_cannot_override_raw_fjt_failure(self):
        target = [0.1] * 7
        frames = [truth(0.0, "qualification-free-space", positions=[0.0] * 7, target=target), truth(1.0, "qualification-free-space", positions=[0.3] * 7, target=target)]
        attempt = self.write_attempt("fjt-fail", frames, [action(accepted=True), action(success=True, expected_final_positions=target)])
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "verified-fail")

    def test_free_gripper_passes_and_requires_both_actions(self):
        frames = [truth(0.0, "qualification-free-gripper", gripper=0.0), truth(1.0, "qualification-free-gripper", gripper=0.83)]
        execution = [action("/xarm_gripper/gripper_action", phase="close", accepted=True, success=True, reached_goal=True, stalled=False, max_effort=20.0), action("/xarm_gripper/gripper_action", phase="open", accepted=True, success=True, reached_goal=True, stalled=False, max_effort=20.0)]
        attempt = self.write_attempt("free-gripper", frames, execution)
        self.assertEqual(verify("free-gripper", attempt, self.config)["status"], "verified-pass")

    def test_obstructed_gripper_positive_requires_exact_contacts(self):
        frames = [truth(0.0, "qualification-obstructed-gripper", gripper=0.0), truth(1.0, "qualification-obstructed-gripper", gripper=0.80, contacts=contact(True, True))]
        execution = [action(phase="grasp", accepted=True, success=True), action("/xarm_gripper/gripper_action", phase="close", accepted=True, success=True, reached_goal=False, stalled=True, max_effort=20.0)]
        attempt = self.write_attempt("obstructed-pass", frames, execution)
        self.assertEqual(verify("obstructed-gripper", attempt, self.config)["status"], "verified-pass")

    def test_safety_positive_requires_stop_clear_and_target_freeze(self):
        target = [0.1] * 7
        frames = [
            truth(0.0, "qualification-free-space", velocities=[0.1] * 7, target=target),
            truth(0.1, "qualification-free-space", velocities=[0.0] * 7, safety=True, target=target),
            truth(0.6, "qualification-free-space", velocities=[0.0] * 7, safety=True, target=target),
            truth(0.7, "qualification-free-space", velocities=[0.0] * 7, target=target),
            truth(1.7, "qualification-free-space", velocities=[0.0] * 7, target=target),
        ]
        frames[1]["command_gateway"] = {
            "last_command_error": "command ignored while safety stop is active"
        }
        attempt = self.write_attempt("safety-pass", frames, [action(accepted=True), action(success=False), self.safety_event(True), self.safety_event(False)])
        self.assertEqual(verify("safety-stop", attempt, self.config)["status"], "verified-pass")

    def test_collision_positive_requires_exact_arm_contact(self):
        pairs = [{"body_a": "link7", "body_b": "/World/Scenario/qualification_arm_obstacle", "normal_force": 1.0}]
        frames = [
            truth(0.0, "qualification-arm-collision", velocities=[0.2] * 7, target=[0.3] * 7),
            truth(0.1, "qualification-arm-collision", velocities=[0.0] * 7, safety=True, target=[0.3] * 7, contacts=pairs),
        ]
        attempt = self.write_attempt("collision-pass", frames, [action(accepted=True), action(success=False)])
        self.assertEqual(verify("arm-collision", attempt, self.config)["status"], "verified-pass")

    def test_retention_positive_recomputes_motion_and_hold(self):
        frames = [
            truth(0.0, "qualification-retention", object_xyz=[0.65, 0.0, 0.80], tcp_xyz=[0.65, 0.0, 0.83]),
            truth(1.0, "qualification-retention", object_xyz=[0.85, 0.0, 0.91], tcp_xyz=[0.85, 0.0, 0.94], contacts=contact(True, True), object_velocity=[0.2, 0.0, 0.0]),
            truth(2.0, "qualification-retention", object_xyz=[0.85, 0.0, 0.91], tcp_xyz=[0.85, 0.0, 0.94], contacts=contact(True, True)),
            truth(3.0, "qualification-retention", object_xyz=[0.85, 0.0, 0.91], tcp_xyz=[0.85, 0.0, 0.94], contacts=contact(True, True)),
        ]
        attempt = self.write_attempt("retention-pass", frames, [action(phase="grasp", accepted=True, success=True), action("/xarm_gripper/gripper_action", phase="close", accepted=True, success=True, reached_goal=False, stalled=True), action(phase="lift", accepted=True, success=True)])
        self.assertEqual(verify("retention", attempt, self.config)["status"], "verified-pass")

    def test_false_stall_without_exact_bilateral_contacts_fails(self):
        frames = [truth(0.0, "qualification-obstructed-gripper", gripper=0.0), truth(1.0, "qualification-obstructed-gripper", gripper=0.8)]
        execution = [action("/xarm_gripper/gripper_action", phase="close", accepted=True, success=True, reached_goal=False, stalled=True, max_effort=20.0)]
        attempt = self.write_attempt("obstructed", frames, execution)
        self.assertEqual(verify("obstructed-gripper", attempt, self.config)["status"], "verified-fail")

    def test_retention_result_without_object_motion_fails(self):
        frames = [truth(0.0, "qualification-retention", object_xyz=[0.65, 0.0, 0.8], contacts=contact(True, True)), truth(1.0, "qualification-retention", object_xyz=[0.65, 0.0, 0.8], contacts=contact(True, True)), truth(2.0, "qualification-retention", object_xyz=[0.65, 0.0, 0.8], contacts=contact(True, True))]
        attempt = self.write_attempt("retention", frames, [action(phase="grasp", accepted=True, success=True), action("/xarm_gripper/gripper_action", phase="close", accepted=True, success=True), action(phase="lift", accepted=True, success=True)])
        self.assertEqual(verify("retention", attempt, self.config)["status"], "verified-fail")

    def test_collision_pass_through_fails(self):
        frames = [truth(0.0, "qualification-arm-collision", velocities=[0.2] * 7, target=[0.3] * 7), truth(1.0, "qualification-arm-collision", velocities=[0.2] * 7, safety=True, target=[0.4] * 7)]
        execution = [action(accepted=True), action(success=False)]
        attempt = self.write_attempt("collision", frames, execution)
        self.assertEqual(verify("arm-collision", attempt, self.config)["status"], "verified-fail")

    def test_safety_resume_and_motion_fails(self):
        target = [0.1] * 7
        frames = [truth(0.0, "qualification-free-space", velocities=[0.1] * 7, target=target), truth(0.1, "qualification-free-space", velocities=[0.0] * 7, safety=True, target=target), truth(0.6, "qualification-free-space", velocities=[0.0] * 7, safety=True, target=[0.2] * 7), truth(0.7, "qualification-free-space", velocities=[0.0] * 7, safety=False, target=[0.3] * 7), truth(1.7, "qualification-free-space", velocities=[0.1] * 7, target=[0.3] * 7)]
        attempt = self.write_attempt("safety", frames, [action(accepted=True), action(success=False), self.safety_event(True), self.safety_event(False)])
        self.assertEqual(verify("safety-stop", attempt, self.config)["status"], "verified-fail")

    def test_nan_is_evidence_invalid(self):
        frames = [truth(0.0, "qualification-free-space")]
        attempt = self.write_attempt("nan", frames, [action(accepted=True, success=True, expected_final_positions=[0.0] * 7)])
        text = (attempt / "physics_truth.jsonl").read_text(encoding="utf-8").replace("0.0", "NaN", 1)
        (attempt / "physics_truth.jsonl").write_text(text, encoding="utf-8")
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "evidence-invalid")

    def test_missing_and_time_regression_are_evidence_invalid(self):
        attempt = self.root / "missing"
        attempt.mkdir()
        (attempt / "gate-execution.jsonl").write_text(json.dumps(action(accepted=True, success=True)) + "\n", encoding="utf-8")
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "evidence-invalid")
        frames = [truth(1.0, "qualification-free-space"), truth(0.5, "qualification-free-space")]
        regression = self.write_attempt("regression", frames, [action(accepted=True), action(success=True, expected_final_positions=[0.0] * 7)])
        self.assertEqual(verify("free-space-fjt", regression, self.config)["status"], "evidence-invalid")

    def test_startup_reset_prefix_is_ignored_but_gate_event_regression_is_invalid(self):
        target = [0.0] * 7
        frames = [
            truth(4.0, "qualification-free-space", target=target),
            truth(0.0, "qualification-free-space", target=target),
            truth(1.0, "qualification-free-space", target=target),
        ]
        execution = [
            {"gate": "free-space-fjt", "event": "gate_started", "simulated_timestamp": 0.0},
            action(accepted=True),
            action(success=True, expected_final_positions=target),
        ]
        attempt = self.write_attempt("startup-reset", frames, execution)
        (attempt / "gate-window.json").write_text(json.dumps({"gate": "free-space-fjt", "attempt_id": "startup-reset", "raw_start_index": 1}) + "\n", encoding="utf-8")
        self.assertEqual(
            verify("free-space-fjt", attempt, self.config)["status"],
            "verified-pass",
        )
        execution[1] = {
            "gate": "free-space-fjt",
            "event": "later",
            "simulated_timestamp": -0.1,
            "endpoint": "/xarm7_traj_controller/follow_joint_trajectory",
            "accepted": True,
        }
        attempt = self.write_attempt("gate-reset", frames, execution)
        self.assertEqual(
            verify("free-space-fjt", attempt, self.config)["status"],
            "evidence-invalid",
        )

    def test_gate_identity_and_gateway_error_are_evidence_invalid(self):
        frames = [truth(0.0, "qualification-free-space", target=[0.0] * 7), truth(1.0, "qualification-free-space", target=[0.0] * 7)]
        attempt = self.write_attempt("identity", frames, [{"gate": "retention", "endpoint": "action", "accepted": True, "success": True}])
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "evidence-invalid")
        attempt = self.write_attempt("gateway", frames, [{"endpoint": "action", "accepted": True, "success": True, "command_gateway": {"status": "error"}}])
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "evidence-invalid")

    def test_external_execution_can_never_pass(self):
        frames = [truth(0.0, "qualification-free-space", target=[0.0] * 7)]
        attempt = self.write_attempt("external", frames, [{"status": "executed-unverified", "endpoint": "external"}])
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "verified-fail")

    def test_verdict_omits_source_hashes_and_nan_safe_metrics(self):
        target = [0.0] * 7
        frames = [truth(0.0, "qualification-free-space", target=target), truth(1.0, "qualification-free-space", target=target)]
        attempt = self.write_attempt("atomic", frames, [action(accepted=True), action(success=True, expected_final_positions=target)])
        verdict = verify("free-space-fjt", attempt, self.config)
        self.assertNotIn("source_hashes", verdict)
        json.dumps(verdict, allow_nan=False)
        self.assertTrue(math.isfinite(verdict["metrics"]["fjt_error"]["rms_error_rad"]))

    def test_exact_endpoint_is_required(self):
        target = [0.0] * 7
        frames = [truth(0.0, "qualification-free-space", target=target), truth(0.01, "qualification-free-space", target=target)]
        attempt = self.write_attempt("wrong-endpoint", frames, [action("/xarm7_traj_controller/follow_joint_trajectory_extra", accepted=True), action("/xarm7_traj_controller/follow_joint_trajectory_extra", success=True, expected_final_positions=target)])
        verdict = verify("free-space-fjt", attempt, self.config)
        self.assertEqual(verdict["status"], "verified-fail")
        self.assertFalse(next(check for check in verdict["checks"] if check["name"] == "action_endpoint")["passed"])

    def test_missing_safety_or_contacts_is_evidence_invalid(self):
        frames = [truth(0.0, "qualification-free-space", target=[0.0] * 7), truth(0.01, "qualification-free-space", target=[0.0] * 7)]
        frames[0]["robot"].pop("safety_stop")
        attempt = self.write_attempt("missing-safety", frames, [action(accepted=True), action(success=True, expected_final_positions=[0.0] * 7)])
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "evidence-invalid")
        frames = [truth(0.0, "qualification-free-space", target=[0.0] * 7), truth(0.01, "qualification-free-space", target=[0.0] * 7)]
        frames[0].pop("contacts")
        frames[0]["robot"].pop("safety_stop")
        frames[0]["safety_stop"] = False
        attempt = self.write_attempt("missing-contacts", frames, [action(accepted=True), action(success=True, expected_final_positions=[0.0] * 7)])
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "evidence-invalid")

    def test_safety_requires_ordered_operator_journal_events(self):
        target = [0.1] * 7
        frames = [truth(0.0, "qualification-free-space", target=target), truth(0.1, "qualification-free-space", safety=True, target=target), truth(0.6, "qualification-free-space", safety=True, target=target), truth(0.7, "qualification-free-space", target=target), truth(1.7, "qualification-free-space", target=target)]
        attempt = self.write_attempt("missing-safety-events", frames, [action(accepted=True), action(success=False)])
        self.assertEqual(verify("safety-stop", attempt, self.config)["status"], "verified-fail")
        check = next(check for check in verify("safety-stop", attempt, self.config)["checks"] if check["name"] == "safety_operator_journal")
        self.assertFalse(check["passed"])

    def test_stop_window_requires_command_targets(self):
        target = [0.1] * 7
        frames = [truth(0.0, "qualification-free-space", target=target), truth(0.1, "qualification-free-space", safety=True, target=target), truth(0.6, "qualification-free-space", safety=True), truth(0.7, "qualification-free-space", target=target), truth(1.7, "qualification-free-space", target=target)]
        attempt = self.write_attempt("missing-target-window", frames, [action(accepted=True), action(success=False), self.safety_event(True), self.safety_event(False)])
        self.assertEqual(verify("safety-stop", attempt, self.config)["status"], "evidence-invalid")

    def test_obstructed_and_retention_require_cube_state(self):
        obstructed_frames = [truth(0.0, "qualification-obstructed-gripper", gripper=0.0), truth(0.01, "qualification-obstructed-gripper", gripper=0.8, contacts=contact(True, True))]
        obstructed_frames[0]["object"] = None
        obstructed = self.write_attempt("missing-cube-obstructed", obstructed_frames, [action(phase="grasp", accepted=True, success=True), action("/xarm_gripper/gripper_action", phase="close", accepted=True, success=True, reached_goal=False, stalled=True)])
        self.assertEqual(verify("obstructed-gripper", obstructed, self.config)["status"], "evidence-invalid")
        retention_frames = [truth(0.0, "qualification-retention", object_xyz=[0.65, 0.0, 0.8], contacts=contact(True, True)), truth(1.0, "qualification-retention", object_xyz=[0.85, 0.0, 0.91], contacts=contact(True, True)), truth(2.0, "qualification-retention", object_xyz=[0.85, 0.0, 0.91], contacts=contact(True, True))]
        retention_frames[1]["object"] = None
        retention = self.write_attempt("missing-cube-retention", retention_frames, [action(phase="grasp", accepted=True, success=True), action("/xarm_gripper/gripper_action", phase="close", accepted=True, success=True), action(phase="lift", accepted=True, success=True)])
        self.assertEqual(verify("retention", retention, self.config)["status"], "evidence-invalid")

    def test_production_timestamp_gap_requires_physics_rate(self):
        target = [0.0] * 7
        frames = [truth(0.0, "qualification-free-space", target=target), truth(0.1, "qualification-free-space", target=target)]
        production_config = dict(CONFIG)
        production_config.pop("test_only_allow_sparse_frames")
        self.config.write_text(json.dumps(production_config), encoding="utf-8")
        attempt = self.write_attempt("timestamp-gap", frames, [action(accepted=True), action(success=True, expected_final_positions=target)])
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "evidence-invalid")

    def test_post_boundary_timestamp_reset_is_rejected(self):
        target = [0.0] * 7
        frames = [truth(0.0, "qualification-free-space", target=target), truth(0.01, "qualification-free-space", target=target), truth(0.005, "qualification-free-space", target=target)]
        attempt = self.write_attempt("post-boundary-reset", frames, [action(accepted=True), action(success=True, expected_final_positions=target)])
        (attempt / "gate-window.json").write_text(json.dumps({"gate": "free-space-fjt", "attempt_id": "post-boundary-reset", "raw_start_index": 0}) + "\n", encoding="utf-8")
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "evidence-invalid")

    def test_production_manifest_requires_explicit_gate_window(self):
        target = [0.0] * 7
        frames = [truth(0.0, "qualification-free-space", target=target), truth(0.01, "qualification-free-space", target=target)]
        attempt = self.write_attempt("manifest-window", frames, [action(accepted=True), action(success=True, expected_final_positions=target)])
        (attempt / "manifest.json").write_text(json.dumps({"attempt_id": "manifest-window"}) + "\n", encoding="utf-8")
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "evidence-invalid")

    def test_fjt_post_terminal_success_is_excluded_from_selected_truth(self):
        target = [0.1] * 7
        frames = [
            truth(0.0, "qualification-free-space", positions=[0.0] * 7, target=target),
            truth(0.5, "qualification-free-space", positions=[0.0] * 7, target=target),
            truth(0.6, "qualification-free-space", positions=target, target=target),
        ]
        execution = [
            {"event": "gate_started", "simulated_timestamp": 0.0},
            action(accepted=True),
            action(success=True, expected_final_positions=target),
            {"event": "terminal", "simulated_timestamp": 0.5},
        ]
        attempt = self.write_attempt("fjt-post-terminal", frames, execution)
        verdict = verify("free-space-fjt", attempt, self.config)
        self.assertEqual(verdict["status"], "verified-fail")
        self.assertEqual(verdict["metrics"]["fjt_error"]["target_frames"], 2)

    def test_safety_only_after_terminal_is_not_a_safety_stop(self):
        target = [0.1] * 7
        frames = [
            truth(0.0, "qualification-free-space", velocities=[0.1] * 7, target=target),
            truth(0.5, "qualification-free-space", velocities=[0.1] * 7, target=target),
            truth(0.6, "qualification-free-space", velocities=[0.0] * 7, safety=True, target=target),
        ]
        execution = [
            {"event": "gate_started", "simulated_timestamp": 0.0},
            action(accepted=True),
            action(success=False),
            {"event": "safety_asserted", "action_endpoint": SAFETY_TOPIC, "simulated_timestamp": 0.1, "result": {"active": True}},
            {"event": "safety_cleared", "action_endpoint": SAFETY_TOPIC, "simulated_timestamp": 0.4, "result": {"active": False}},
            {"event": "terminal", "simulated_timestamp": 0.5},
        ]
        attempt = self.write_attempt("safety-post-terminal", frames, execution)
        self.assertEqual(verify("safety-stop", attempt, self.config)["status"], "evidence-invalid")

    def test_collision_only_after_terminal_is_not_an_effective_stop(self):
        pairs = [{"body_a": "link7", "body_b": "/World/Scenario/qualification_arm_obstacle", "normal_force": 1.0}]
        frames = [
            truth(0.0, "qualification-arm-collision", velocities=[0.2] * 7, target=[0.3] * 7),
            truth(0.5, "qualification-arm-collision", velocities=[0.2] * 7, target=[0.3] * 7),
            truth(0.6, "qualification-arm-collision", velocities=[0.0] * 7, safety=True, target=[0.3] * 7, contacts=pairs),
        ]
        execution = [
            {"event": "gate_started", "simulated_timestamp": 0.0},
            action(accepted=True),
            action(success=False),
            {"event": "terminal", "simulated_timestamp": 0.5},
        ]
        attempt = self.write_attempt("collision-post-terminal", frames, execution)
        self.assertEqual(verify("arm-collision", attempt, self.config)["status"], "evidence-invalid")

    def test_retention_only_after_terminal_is_not_a_lift(self):
        frames = [
            truth(0.0, "qualification-retention", object_xyz=[0.65, 0.0, 0.80]),
            truth(0.5, "qualification-retention", object_xyz=[0.65, 0.0, 0.80], contacts=contact(True, True)),
            truth(0.6, "qualification-retention", object_xyz=[0.85, 0.0, 0.91], contacts=contact(True, True)),
        ]
        execution = [
            {"event": "gate_started", "simulated_timestamp": 0.0},
            action(phase="grasp", accepted=True, success=True),
            action("/xarm_gripper/gripper_action", phase="close", accepted=True, success=True, reached_goal=False, stalled=True),
            action(phase="lift", accepted=True, success=True),
            {"event": "terminal", "simulated_timestamp": 0.5},
        ]
        attempt = self.write_attempt("retention-post-terminal", frames, execution)
        verdict = verify("retention", attempt, self.config)
        self.assertEqual(verdict["status"], "verified-fail")
        self.assertLess(verdict["metrics"]["retention_physics_truth"]["lift_m"], 0.10)

    def test_execution_window_requires_one_start_and_a_raw_overlap(self):
        target = [0.0] * 7
        frames = [truth(2.0, "qualification-free-space", target=target), truth(2.01, "qualification-free-space", target=target)]
        attempt = self.write_attempt("window-invalid", frames, [
            {"event": "gate_started", "simulated_timestamp": 0.0},
            {"event": "terminal", "simulated_timestamp": 1.0},
        ])
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "evidence-invalid")

        attempt = self.write_attempt("window-duplicate-start", frames, [
            {"event": "gate_started", "simulated_timestamp": 0.0},
            {"event": "gate_started", "simulated_timestamp": 0.1},
            {"event": "terminal", "simulated_timestamp": 1.0},
        ])
        self.assertEqual(verify("free-space-fjt", attempt, self.config)["status"], "evidence-invalid")


if __name__ == "__main__":
    unittest.main()
