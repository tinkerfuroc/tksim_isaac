from __future__ import annotations

import json
import inspect
import sys
import tempfile
import unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from validation.manipulation_gate_executor import (  # noqa: E402
    ACTION_RESULT_TIMEOUT_S,
    FJT_GOAL_TIME_TOLERANCE_S,
    FREE_SPACE_OUTBOUND_S,
    FREE_SPACE_RETURN_S,
    GRASP_MOTION_S,
    LIFT_MOTION_S,
    GATES,
    GRIPPER_ACTION,
    JOINT_NAMES,
    Q_GRASP,
    Q_LIFT,
    Q_OUTBOUND,
    SAFETY_ACTION_RESULT_TIMEOUT_S,
    SAFETY_CLEAR_TRUTH_TIMEOUT_S,
    SAFETY_CANCEL_RESPONSE_TIMEOUT_S,
    SAFETY_HOLD_AFTER_COMPLIANCE_S,
    SAFETY_POST_CLEAR_SETTLE_S,
    SAFETY_TOPIC,
    TRAJECTORY_ACTION,
    VISUAL_CHECKPOINT_EVENTS,
    EventJournal,
    build_gate_actions,
    build_free_space_fixture,
    build_lift_fixture,
    build_safety_fixture,
    gripper_goal,
    load_config,
    main,
    parse_args,
)
import validation.manipulation_gate_executor as gate_executor  # noqa: E402


class ManipulationGateExecutorTest(unittest.TestCase):
    def test_import_does_not_import_ros(self) -> None:
        with patch.dict(sys.modules, {"rclpy": None, "control_msgs": None, "trajectory_msgs": None}):
            self.assertEqual(JOINT_NAMES, tuple(f"joint{i}" for i in range(1, 8)))
            self.assertEqual(build_free_space_fixture((0.0,) * 7).points[-2].positions, Q_OUTBOUND)

    def test_free_space_fixture_is_exact_and_zero_velocity(self) -> None:
        fixture = build_free_space_fixture((0.1,) * 7)
        self.assertEqual(fixture.joint_names, JOINT_NAMES)
        self.assertEqual(
            [point.time_from_start_s for point in fixture.points],
            [0.0, FREE_SPACE_OUTBOUND_S, FREE_SPACE_RETURN_S],
        )
        self.assertEqual(fixture.points[1].positions, Q_OUTBOUND)
        self.assertEqual(fixture.points[0].positions, fixture.points[2].positions)
        self.assertTrue(all(value == 0.0 for point in fixture.points for value in point.velocities))
        self.assertEqual(fixture.start_offset_s, 0.5)

    def test_safety_fixture_is_stretched_to_eight_seconds(self) -> None:
        fixture = build_safety_fixture((0.0,) * 7)
        self.assertEqual([point.time_from_start_s for point in fixture.points], [0.0, 4.0, 8.0])
        self.assertEqual(fixture.points[1].positions, Q_OUTBOUND)

    def test_fake_ros_goal_builder_preserves_full_joint_contract(self) -> None:
        class FakeDuration:
            def __init__(self, *, seconds: float) -> None:
                self.seconds = seconds

            def to_msg(self) -> tuple[str, float]:
                return ("duration", self.seconds)

        class FakePoint:
            def __init__(self) -> None:
                self.positions = []
                self.velocities = []
                self.time_from_start = None

        class FakeGoal:
            def __init__(self) -> None:
                self.trajectory = type("Trajectory", (), {"joint_names": [], "header": type("Header", (), {})(), "points": []})()

        class FakeAction:
            Goal = FakeGoal

        with patch.dict(gate_executor._ROS_IMPORTS, {"Duration": FakeDuration}):
            message = gate_executor._goal_spec_to_message(
                build_free_space_fixture((0.0,) * 7), FakeAction, FakePoint, ("stamp", 1.0)
            )
        self.assertEqual(message.trajectory.joint_names, list(JOINT_NAMES))
        self.assertEqual(message.trajectory.header.stamp, ("stamp", 1.0))
        self.assertEqual(message.trajectory.points[1].positions, list(Q_OUTBOUND))
        self.assertTrue(all(not point.velocities or point.velocities == [0.0] * 7 for point in message.trajectory.points))
        self.assertEqual(message.trajectory.points[-1].time_from_start, ("duration", FREE_SPACE_RETURN_S))
        self.assertEqual(message.goal_time_tolerance, ("duration", FJT_GOAL_TIME_TOLERANCE_S))

    def test_free_space_timing_remains_nontrivial_and_tolerance_is_bounded(self) -> None:
        fixture = build_free_space_fixture((0.0,) * 7)
        self.assertEqual(
            [fixture.points[1].time_from_start_s, fixture.points[2].time_from_start_s],
            [FREE_SPACE_OUTBOUND_S, FREE_SPACE_RETURN_S],
        )
        self.assertGreater(FREE_SPACE_OUTBOUND_S, 0.0)
        self.assertGreater(FREE_SPACE_RETURN_S, FREE_SPACE_OUTBOUND_S)
        self.assertGreater(FJT_GOAL_TIME_TOLERANCE_S, 0.508333)
        self.assertLessEqual(FJT_GOAL_TIME_TOLERANCE_S, 1.0)

    def test_grasp_and_lift_fixtures_are_frozen(self) -> None:
        grasp = build_gate_actions("obstructed-gripper", (0.0,) * 7)[0].goal
        lift = build_lift_fixture()
        self.assertEqual(grasp["points"][-1]["time_from_start_s"], GRASP_MOTION_S)
        self.assertEqual(lift.points[-1].time_from_start_s, LIFT_MOTION_S)
        self.assertGreaterEqual(GRASP_MOTION_S, 8.0)
        self.assertGreaterEqual(LIFT_MOTION_S, 8.0)
        self.assertEqual(tuple(grasp["points"][-1]["positions"]), Q_GRASP)
        self.assertEqual(lift.points[0].positions, Q_GRASP)
        self.assertEqual(lift.points[-1].positions, Q_LIFT)

    def test_gate_sequences_use_only_approved_endpoints(self) -> None:
        for gate in GATES:
            endpoints = [action.endpoint for action in build_gate_actions(gate, (0.0,) * 7)]
            self.assertTrue(set(endpoints) <= {TRAJECTORY_ACTION, GRIPPER_ACTION})
            self.assertNotIn("/isaac_joint_commands", endpoints)
        safety_actions = build_gate_actions("safety-stop", (0.0,) * 7)
        self.assertEqual(safety_actions[0].endpoint, TRAJECTORY_ACTION)
        self.assertEqual(SAFETY_TOPIC, "/sim/safety/operator")

    def test_gripper_goal_is_fixed_at_twenty_newtons(self) -> None:
        self.assertEqual(gripper_goal(0.83), {"position": 0.83, "max_effort": 20.0})
        self.assertEqual(gripper_goal(0.0), {"position": 0.0, "max_effort": 20.0})
        with self.assertRaises(ValueError):
            gripper_goal(0.86)

    def test_event_journal_is_append_only_and_final_summary_is_atomic(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            journal = EventJournal(Path(directory), "free-space-fjt")
            first = journal.record("gate_started", simulated_timestamp=2.5, action_endpoint=TRAJECTORY_ACTION, goals={"x": 1})
            second = journal.record("action_goal_response", accepted=True, result={"status": 1})
            summary = journal.finalize(success=True)
            lines = (Path(directory) / "gate-execution.jsonl").read_text(encoding="utf-8").splitlines()
            self.assertEqual([json.loads(line)["sequence"] for line in lines], [1, 2])
            self.assertEqual(summary["event_count"], 2)
            final = json.loads((Path(directory) / "gate-execution.json").read_text(encoding="utf-8"))
            self.assertEqual(final["events"], [first, second])
            self.assertFalse(list(Path(directory).glob("*.tmp")))

    def test_visual_checkpoint_requests_reference_execution_sequences(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            journal = EventJournal(Path(directory), "free-space-fjt")
            journal.record("start", simulated_timestamp=1.0)
            journal.record("outbound-apex", simulated_timestamp=3.0)
            journal.record("return-arrival", simulated_timestamp=5.0)
            journal.record("terminal", simulated_timestamp=5.1)
            summary = journal.finalize(success=True)
            requests = [
                json.loads(line)
                for line in (Path(directory) / "visual-capture-requests.jsonl").read_text(encoding="utf-8").splitlines()
            ]
            self.assertEqual([request["event"] for request in requests], list(VISUAL_CHECKPOINT_EVENTS["free-space-fjt"]))
            self.assertEqual(
                [request["source_execution_event_sequence"] for request in requests],
                [1, 2, 3, 4],
            )
            self.assertEqual(summary["requested_visual_events"], list(VISUAL_CHECKPOINT_EVENTS["free-space-fjt"]))
            self.assertEqual(summary["visual_capture_requests"], requests)

    def test_trajectory_phase_is_retained_in_journal_records(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            journal = EventJournal(Path(directory), "retention")
            record = journal.record("action_goal_sent", action_endpoint=TRAJECTORY_ACTION, phase="lift")
            self.assertEqual(record["phase"], "lift")
            journal.finalize(success=True)
            stored = json.loads((Path(directory) / "gate-execution.jsonl").read_text().splitlines()[0])
            self.assertEqual(stored["phase"], "lift")

    def test_checkpoint_journals_before_waiting_one_physics_frame(self) -> None:
        executor = object.__new__(gate_executor.RosGateExecutor)
        events = []
        waited = []
        executor.clock = 4.0
        executor.config = {"physics": {"hz": 120.0}}
        executor.journal = SimpleNamespace(record=lambda event, **kwargs: events.append((event, kwargs)))
        executor._spin_until = lambda predicate, timeout_s: True
        executor.wait_clock = lambda target, timeout_s=30.0: waited.append(target)
        executor._checkpoint("velocity-compliant", predicate=lambda: True)
        self.assertEqual(events[0][0], "velocity-compliant")
        self.assertAlmostEqual(waited[0], 4.0 + 1.0 / 120.0)

    def test_cancel_response_is_recorded_and_requires_matching_goal(self) -> None:
        class FakeFuture:
            def __init__(self, value: object, *, done: bool = True) -> None:
                self.value = value
                self._done = done

            def done(self) -> bool:
                return self._done

            def result(self) -> object:
                if not self._done:
                    raise AssertionError("unresolved future result was read")
                return self.value

        class FakeRclpy:
            def __init__(self) -> None:
                self.timeouts: list[float] = []

            def spin_until_future_complete(self, node: object, future: FakeFuture, *, timeout_sec: float) -> None:
                del node, future
                self.timeouts.append(timeout_sec)

        goal_uuid = list(range(16))
        goal_id = SimpleNamespace(uuid=goal_uuid)
        journal = SimpleNamespace(records=[])
        journal.record = lambda event, **kwargs: journal.records.append({"event": event, **kwargs})
        response = SimpleNamespace(
            return_code=0,
            goals_canceling=[SimpleNamespace(goal_id=SimpleNamespace(uuid=goal_uuid))],
        )
        cancel_future = FakeFuture(response)
        handle = SimpleNamespace(goal_id=goal_id, cancel_goal_async=lambda: cancel_future)
        rclpy = FakeRclpy()
        executor = object.__new__(gate_executor.RosGateExecutor)
        executor.ros = {"rclpy": rclpy}
        executor.node = object()
        executor.clock = 12.5
        executor.journal = journal

        self.assertTrue(executor._cancel_trajectory_goal(handle))
        self.assertEqual(rclpy.timeouts, [SAFETY_CANCEL_RESPONSE_TIMEOUT_S])
        record = journal.records[-1]
        self.assertEqual(record["event"], "action_cancel_requested")
        self.assertTrue(record["accepted"])
        self.assertTrue(record["canceled"])
        self.assertEqual(record["result"]["return_code"], 0)
        self.assertTrue(record["result"]["return_code_valid"])
        self.assertTrue(record["result"]["goal_id_match"])
        self.assertEqual(record["result"]["goals_canceling_count"], 1)

        rejected_journal = SimpleNamespace(records=[])
        rejected_journal.record = lambda event, **kwargs: rejected_journal.records.append({"event": event, **kwargs})
        rejected_response = SimpleNamespace(return_code=1, goals_canceling=[])
        rejected_future = FakeFuture(rejected_response)
        rejected_handle = SimpleNamespace(goal_id=goal_id, cancel_goal_async=lambda: rejected_future)
        executor.journal = rejected_journal
        self.assertFalse(executor._cancel_trajectory_goal(rejected_handle))
        rejected_record = rejected_journal.records[-1]
        self.assertFalse(rejected_record["accepted"])
        self.assertFalse(rejected_record["canceled"])
        self.assertEqual(rejected_record["result"]["return_code"], 1)
        self.assertIn("return_code=1", rejected_record["error"])

    def test_safety_stop_finishes_when_canceled_action_result_stays_unresolved(self) -> None:
        class FakeFuture:
            def __init__(self, value: object = None, *, done: bool = True) -> None:
                self.value = value
                self._done = done

            def done(self) -> bool:
                return self._done

            def result(self) -> object:
                if not self._done:
                    raise AssertionError("unresolved action result was read")
                return self.value

        class FakeRclpy:
            def __init__(self) -> None:
                self.timeouts: list[float] = []

            def spin_until_future_complete(self, node: object, future: FakeFuture, *, timeout_sec: float) -> None:
                del node, future
                self.timeouts.append(timeout_sec)

        events: list[dict[str, object]] = []
        journal = SimpleNamespace(records=events)
        journal.record = lambda event, **kwargs: events.append({"event": event, **kwargs})
        goal_uuid = list(range(16))
        goal_id = SimpleNamespace(uuid=goal_uuid)
        cancel_response = SimpleNamespace(
            return_code=0,
            goals_canceling=[SimpleNamespace(goal_id=SimpleNamespace(uuid=goal_uuid))],
        )
        cancel_future = FakeFuture(cancel_response)
        result_future = FakeFuture(done=False)
        handle = SimpleNamespace(
            goal_id=goal_id,
            cancel_goal_async=lambda: cancel_future,
        )
        rclpy = FakeRclpy()
        executor = object.__new__(gate_executor.RosGateExecutor)
        executor.ros = {"rclpy": rclpy}
        executor.node = object()
        executor.clock = 10.0
        executor.config = {"physics": {"hz": 120.0}}
        executor.journal = journal
        executor.effective_safety = False

        def send_trajectory(spec: object, *, phase: str) -> tuple[object, FakeFuture]:
            del spec, phase
            return handle, result_future

        def publish_safety(active: bool) -> None:
            executor.effective_safety = active
            events.append({"event": "safety", "active": active})

        executor._send_trajectory = send_trajectory
        executor._publish_safety = publish_safety
        executor._checkpoint = lambda event, **kwargs: events.append({"event": event, **kwargs})
        executor._wait_for_truth_safety_stop = lambda expected, timeout_s: events.append(
            {"event": "raw-clear", "expected": expected, "timeout_s": timeout_s}
        )
        wait_targets: list[tuple[float, float]] = []

        def wait_clock(target: float, timeout_s: float = 30.0) -> None:
            wait_targets.append((target, timeout_s))
            executor.clock = target

        executor.wait_clock = wait_clock
        executor._spin_until = lambda predicate, timeout_s: True

        executor._run_trajectory(
            build_safety_fixture((0.0,) * 7),
            phase="safety",
            safety_at_s=1.5,
            checkpoint_events=(("moving", 0.0),),
        )

        self.assertEqual(rclpy.timeouts, [SAFETY_CANCEL_RESPONSE_TIMEOUT_S, SAFETY_ACTION_RESULT_TIMEOUT_S])
        cancel_record = next(item for item in events if item["event"] == "action_cancel_requested")
        self.assertTrue(cancel_record["canceled"])
        self.assertEqual(cancel_record["result"]["return_code"], 0)
        result_record = next(item for item in events if item["event"] == "action_result")
        self.assertTrue(result_record["canceled"])
        self.assertFalse(result_record["result"]["resolved"])
        self.assertNotIn("success", result_record["result"])
        self.assertIn("unresolved after cancellation", result_record["error"])
        self.assertEqual(
            [item["event"] for item in events if item["event"] in {"effective-stop", "velocity-compliant", "raw-clear", "post-clear"}],
            ["effective-stop", "velocity-compliant", "raw-clear", "post-clear"],
        )
        self.assertEqual(
            [target for target, _timeout in wait_targets],
            [12.0, 12.0 + SAFETY_HOLD_AFTER_COMPLIANCE_S, 12.0 + SAFETY_HOLD_AFTER_COMPLIANCE_S + SAFETY_POST_CLEAR_SETTLE_S],
        )
        self.assertTrue(all(timeout == SAFETY_CLEAR_TRUTH_TIMEOUT_S for _target, timeout in wait_targets[1:]))
        raw_clear = next(item for item in events if item["event"] == "raw-clear")
        self.assertFalse(raw_clear["expected"])
        self.assertEqual(raw_clear["timeout_s"], SAFETY_CLEAR_TRUTH_TIMEOUT_S)
        self.assertNotEqual(rclpy.timeouts[-1], ACTION_RESULT_TIMEOUT_S)

    def test_safety_timed_wait_is_bounded_and_fails_closed(self) -> None:
        executor = object.__new__(gate_executor.RosGateExecutor)
        executor.clock = 3.0
        waits: list[tuple[float, float]] = []

        def wait_clock(target: float, timeout_s: float = 30.0) -> None:
            waits.append((target, timeout_s))
            raise RuntimeError("simulated clock stalled")

        executor.wait_clock = wait_clock
        with self.assertRaisesRegex(RuntimeError, "simulated clock stalled"):
            executor._wait_simulated_duration(SAFETY_POST_CLEAR_SETTLE_S, timeout_s=SAFETY_CLEAR_TRUTH_TIMEOUT_S)
        self.assertEqual(waits, [(3.0 + SAFETY_POST_CLEAR_SETTLE_S, SAFETY_CLEAR_TRUTH_TIMEOUT_S)])

    def test_safety_clear_waits_for_evaluator_owned_raw_truth(self) -> None:
        executor = object.__new__(gate_executor.RosGateExecutor)
        executor.clock = 10.0
        observed = []
        truth_values = iter((True, True, False))

        def latest_truth_safety_stop() -> bool:
            observed.append("truth")
            return next(truth_values)

        executor._truth_safety_stop = latest_truth_safety_stop
        executor._spin_until = lambda predicate, timeout_s: (
            observed.append(timeout_s) or any(predicate() for _ in range(3))
        )

        executor._wait_for_truth_safety_stop(False, SAFETY_CLEAR_TRUTH_TIMEOUT_S)

        self.assertEqual(observed[-1], "truth")
        self.assertEqual(observed.count("truth"), 3)
        self.assertIn(SAFETY_CLEAR_TRUTH_TIMEOUT_S, observed)

    def test_safety_clear_truth_polling_fails_closed_after_bounded_timeout(self) -> None:
        executor = object.__new__(gate_executor.RosGateExecutor)
        executor._truth_safety_stop = lambda: (_ for _ in ()).throw(RuntimeError("incomplete"))
        executor._spin_until = lambda predicate, timeout_s: (
            self.assertFalse(predicate()) or False
        )

        with self.assertRaisesRegex(RuntimeError, "raw physics truth safety_stop"):
            executor._wait_for_truth_safety_stop(False, 0.25)

    def test_truth_safety_stop_accepts_nested_robot_schema(self) -> None:
        executor = object.__new__(gate_executor.RosGateExecutor)
        executor._latest_physics_truth = lambda: {"robot": {"safety_stop": False}}
        self.assertIs(executor._truth_safety_stop(), False)

    def test_contact_checkpoint_predicate_uses_bilateral_qualification_cube_contacts(self) -> None:
        executor = object.__new__(gate_executor.RosGateExecutor)
        with tempfile.TemporaryDirectory() as directory:
            executor.physics_truth_path = Path(directory) / "physics_truth.jsonl"
            executor.clock = 10.0
            executor.config = {"physics": {"hz": 120.0}}
            record = {
                "timestamp": 10.0,
                "frame_index": 12,
                "contacts": [
                    {"body_a": "left_finger_link", "body_b": "qualification_cube"},
                    {"body_a": "right_finger_link", "body_b": "qualification_cube"},
                ],
                "objects": [{"id": "qualification_cube", "pose": {"xyz": [0.1, 0.2, 0.3]}}],
            }
            executor.physics_truth_path.write_text(json.dumps(record) + "\n", encoding="utf-8")
            self.assertEqual(executor._truth_contact_sides(), (True, True))
            self.assertEqual(executor._truth_cube_pose(), (0.1, 0.2, 0.3))
            record["contacts"].pop()
            executor.physics_truth_path.write_text(json.dumps(record) + "\n", encoding="utf-8")
            self.assertEqual(executor._truth_contact_sides(), (True, False))

    def test_physics_truth_predicates_fail_closed_for_malformed_or_stale_file(self) -> None:
        executor = object.__new__(gate_executor.RosGateExecutor)
        with tempfile.TemporaryDirectory() as directory:
            executor.physics_truth_path = Path(directory) / "physics_truth.jsonl"
            executor.clock = 10.0
            executor.config = {"physics": {"hz": 120.0}}
            executor.physics_truth_path.write_text('{"timestamp": 10.0,\n', encoding="utf-8")
            with self.assertRaises(RuntimeError):
                executor._truth_contact_sides()
            executor.physics_truth_path.write_text(
                json.dumps({"timestamp": 10.0, "frame_index": 1, "contacts": []})
                + "\n"
                + '{"timestamp": 10.0, "frame_index": 2',
                encoding="utf-8",
            )
            with self.assertRaises(RuntimeError):
                executor._truth_contact_sides()
            executor.physics_truth_path.write_text(
                json.dumps({"timestamp": 9.0, "frame_index": 1, "contacts": []}) + "\n",
                encoding="utf-8",
            )
            with self.assertRaises(RuntimeError):
                executor._truth_contact_sides()

    def test_ros_executor_has_no_raw_truth_subscription(self) -> None:
        source = inspect.getsource(gate_executor.RosGateExecutor.__init__)
        self.assertNotIn("/sim/internal/physics_truth", source)
        self.assertNotIn('ros["String"]', source)

    def test_all_visual_checkpoint_contracts_have_four_exact_events(self) -> None:
        self.assertEqual(
            VISUAL_CHECKPOINT_EVENTS["safety-stop"],
            ("moving", "effective-stop", "velocity-compliant", "post-clear"),
        )
        for gate in GATES:
            self.assertEqual(len(VISUAL_CHECKPOINT_EVENTS[gate]), 4)
            self.assertEqual(len(set(VISUAL_CHECKPOINT_EVENTS[gate])), 4)

    def test_cli_arguments_are_explicit(self) -> None:
        args = parse_args(["--gate", "retention", "--attempt-dir", "/tmp/a", "--config", "/tmp/c.json"])
        self.assertEqual(args.gate, "retention")
        self.assertEqual(args.attempt_dir, Path("/tmp/a"))
        self.assertEqual(args.config, Path("/tmp/c.json"))

    def test_config_requires_schema_v2(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps({"schema_version": 2, "gates": list(GATES)}), encoding="utf-8")
            self.assertEqual(load_config(path)["schema_version"], 2)
            path.write_text(json.dumps({"schema_version": 1}), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_config(path)

    def test_ros_startup_failure_is_recorded_and_returns_nonzero(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt_dir = Path(directory) / "attempt"
            config = Path(directory) / "config.json"
            config.write_text(json.dumps({"schema_version": 2, "gates": list(GATES)}), encoding="utf-8")
            with patch("validation.manipulation_gate_executor._load_ros", side_effect=ImportError("ROS unavailable")):
                self.assertNotEqual(
                    main(["--gate", "free-space-fjt", "--attempt-dir", str(attempt_dir), "--config", str(config)]),
                    0,
                )
            summary = json.loads((attempt_dir / "gate-execution.json").read_text(encoding="utf-8"))
            self.assertFalse(summary["success"])
            self.assertEqual(summary["events"][-1]["event"], "executor_error")


if __name__ == "__main__":
    unittest.main()
