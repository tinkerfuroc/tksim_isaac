from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge"
CORE = ROOT / "simulation/tinker_sim_core/command_mux.py"
ISAAC_GATEWAY = ROOT / "simulation/tinker_sim_isaac/ros_gateway.py"
ISAAC_BACKEND = ROOT / "simulation/tinker_sim_isaac/backend.py"


def _source(name: str) -> str:
    return (BRIDGE / name).read_text(encoding="utf-8")


def _tracker_type():
    tree = ast.parse(_source("safety_supervisor.py"))
    tracker = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "SafetySourceTracker"
    )
    namespace: dict[str, object] = {}
    exec(
        compile(
            ast.Module(body=[tracker], type_ignores=[]),
            "safety_supervisor.py",
            "exec",
        ),
        namespace,
    )
    return namespace["SafetySourceTracker"]


def test_effective_stop_has_one_transient_local_publisher() -> None:
    safety = _source("safety_supervisor.py")
    assert safety.count('"/sim/hardware/safety_stop"') == 1
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in safety
    assert 'REQUIRED_SOURCES = ("xarm", "collision")' in safety
    assert 'OPTIONAL_SOURCES = ("operator",)' in safety
    assert "name: None for name in self.REQUIRED_SOURCES" in safety
    assert "self._sources.update({name: False for name in self.OPTIONAL_SOURCES})" in safety
    assert "SafetySourceTracker" in safety
    assert "required_source_deadline_s" in safety
    assert "tracker.requires_stop(time.monotonic())" in safety
    assert "ClockType.STEADY_TIME" in safety
    assert "self._publish(True)" in safety
    assert "self._publish_effective()" in safety
    assert "supervisor liveness heartbeat" in safety

    for name in ("command_gateway.py", "xarm_facade.py", "gripper_facade.py"):
        source = _source(name)
        assert 'create_publisher(\n            Bool, "/sim/hardware/safety_stop"' not in source


def test_command_epoch_and_snapshot_boundary_are_explicit_and_shared() -> None:
    core = CORE.read_text(encoding="utf-8")
    gateway = _source("command_gateway.py")
    isaac_gateway = ISAAC_GATEWAY.read_text(encoding="utf-8")
    backend = ISAAC_BACKEND.read_text(encoding="utf-8")

    assert "COMMAND_FRAME_PREFIX = \"tinker_command_epoch:\"" in core
    assert "encode_command_frame" in gateway
    assert "decode_command_frame" in isaac_gateway
    assert "epoch != self._command_epoch" in isaac_gateway
    assert "begin_command_snapshot" in isaac_gateway
    assert "self._velocity_targets.zero_()" in backend
    assert "self._effort_targets.zero_()" in backend


def test_stopped_command_packets_are_blocked_before_parse_without_error_logging() -> None:
    gateway = _source("command_gateway.py")
    tree = ast.parse(gateway)
    accept = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "_accept"
    )
    assert isinstance(accept.body[0], ast.If)
    stopped = accept.body[0]
    assert isinstance(stopped.test, ast.Attribute)
    assert stopped.test.attr == "_safety_active"
    assert any(isinstance(node, ast.Return) for node in stopped.body)
    assert any(
        isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Subscript)
            and isinstance(target.value, ast.Attribute)
            and target.value.attr == "_rejected"
            for target in node.targets
        )
        for node in stopped.body
    )
    assert "blocked by safety stop" in gateway
    assert "self.get_logger().error(f\"rejected {source} joint command: {error}\")" in gateway
    # The parse/ownership-projection step (renamed from command_from_sequences
    # to _owned_command when inbound parsing moved onto the simulation
    # thread) must stay behind the safety-stop gate.
    assert gateway.index("if self._safety_active:") < gateway.index("self._owned_command(")


def test_safety_clear_requires_a_fresh_packet_after_mux_stop() -> None:
    gateway = _source("command_gateway.py")
    tree = ast.parse(gateway)
    safety_stop = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "_safety_stop"
    )
    calls = [
        node
        for node in ast.walk(safety_stop)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "stop"
    ]
    assert len(calls) == 1
    assert isinstance(calls[0].func.value, ast.Attribute)
    assert calls[0].func.value.attr == "_mux"
    assert "_advance_command_epoch" in gateway
    assert "encode_command_epoch" in gateway
    accept = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "_accept"
    )
    assert isinstance(accept.body[0], ast.If)
    assert any(isinstance(node, ast.Call) for node in ast.walk(accept.body[1]))


def test_safety_reconciles_controller_only_after_list_and_switch_results() -> None:
    safety = _source("safety_supervisor.py")
    assert "from controller_manager_msgs.srv import ListControllers, SwitchController" in safety
    assert '"/controller_manager/list_controllers"' in safety
    assert "ListControllers.Request()" in safety
    assert "future.add_done_callback(self._controllers_listed)" in safety
    assert "future.add_done_callback" in safety
    assert "_controller_was_active" in safety
    assert "self._controller_was_active: bool | None = None" in safety
    assert "self._stop_episode_recorded" in safety
    assert "or self._restore_pending" in safety
    assert "elif not active:" in safety
    assert "if response is None or not response.ok" in safety
    assert "controller_management_ready" in safety
    assert "if not self._management_ready:" in safety
    assert "self._publish_effective()" in safety
    assert "self._controller_active = None" in safety
    assert "_controller_stopped" not in safety


def test_safety_sources_request_latched_state() -> None:
    safety = _source("safety_supervisor.py")
    assert "durability=DurabilityPolicy.TRANSIENT_LOCAL" in safety
    assert "depth=1" in safety
    assert "source_qos" in safety


def test_required_source_starts_stopped_until_first_sample() -> None:
    tracker = _tracker_type()(1.0)
    assert tracker.requires_stop(0.0)


def test_required_source_expires_after_deadline() -> None:
    tracker = _tracker_type()(1.0)
    tracker.update(False, 10.0)
    assert not tracker.requires_stop(10.99)
    assert tracker.requires_stop(11.0)


def test_required_source_refresh_restarts_deadline() -> None:
    tracker = _tracker_type()(1.0)
    tracker.update(False, 10.0)
    tracker.update(False, 10.75)
    assert not tracker.requires_stop(11.70)
    assert tracker.requires_stop(11.75)


def test_required_source_recovers_only_from_fresh_explicit_false() -> None:
    tracker = _tracker_type()(1.0)
    tracker.update(False, 10.0)
    assert tracker.requires_stop(11.0)
    tracker.update(True, 11.1)
    assert tracker.requires_stop(11.2)
    tracker.update(False, 11.3)
    assert not tracker.requires_stop(11.3)


def test_safety_does_not_clear_until_all_sources_are_initialized() -> None:
    safety = _source("safety_supervisor.py")
    source_callback = safety[safety.index("    def _source"): safety.index("    def _publish")]
    assert "_source_trackers[name].update" in source_callback
    assert "self._reconcile()" in source_callback
    assert "self._sources[name] = bool(message.data)" in source_callback


def test_operator_source_is_optional_and_defaults_clear() -> None:
    safety = _source("safety_supervisor.py")
    assert 'OPTIONAL_SOURCES = ("operator",)' in safety
    assert "self._sources.update({name: False for name in self.OPTIONAL_SOURCES})" in safety
    assert "REQUIRED_SOURCES" in safety


def test_controller_restoration_is_recorded_per_stop_episode() -> None:
    safety = _source("safety_supervisor.py")
    assert "self._controller_was_active = self._controller_active is True" in safety
    assert "self._restore_pending = self._restore_pending or self._controller_was_active is True" in safety
    assert "self._controller_was_active = None" in safety
    assert "self._stop_episode_recorded = False" in safety
    assert "if self._desired_stop and not self._stop_episode_recorded:" in safety
    assert "elif active:" in safety
    assert "activation which is still in" in safety
    assert "while a stop transition may be racing it" in safety


def test_safety_restoration_waits_for_confirmed_active_state() -> None:
    safety = _source("safety_supervisor.py")
    restoration = safety[safety.index("elif self._restore_pending"): safety.index("def _request_switch")]
    assert "if active:" in restoration
    assert "self._restore_pending = False" in restoration
    assert "self._publish_effective()" in restoration
    assert "self._request_switch(activate=True)" in restoration


def test_whole_robot_launch_starts_the_safety_supervisor() -> None:
    launch = (ROOT / "ros2_ws/src/tinker_sim_bridge/launch/whole_robot.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'package="tinker_sim_bridge"' in launch
    assert 'executable="safety_supervisor"' in launch
    assert 'parameters=[{"use_sim_time": True}]' in launch


def test_launches_start_safety_immediately_and_gate_spawner_chains() -> None:
    for name in ("manipulation.launch.py", "whole_robot.launch.py"):
        launch = (ROOT / "ros2_ws/src/tinker_sim_bridge/launch" / name).read_text(
            encoding="utf-8"
        )
        assert "TimerAction" not in launch
        assert "joint_state_spawner = Node(" in launch
        assert "xarm_traj_spawner = Node(" in launch
        assert "safety_supervisor = Node(" in launch
        assert "RegisterEventHandler(" in launch
        assert "OnProcessExit(" in launch
        assert "target_action=joint_state_spawner" in launch
        assert "target_action=xarm_traj_spawner" in launch
        assert "event.returncode == 0" in launch
        if name == "manipulation.launch.py":
            assert "controller_ready_setter" not in launch
            assert '"--ready-node"' in launch
            assert '"--ready-parameter"' in launch
            assert '"--ready-timeout",' in launch
        else:
            assert "controller_ready_setter" in launch
        assert "controller_management_ready" in launch
        assert "EmitEvent" in launch
        assert "Shutdown(" in launch

        launch_body = launch[launch.index("def _resolve"):]
        immediate_actions = launch_body.split("    return [", 1)[1].split("\n    ]", 1)[0]
        assert immediate_actions.index("safety_supervisor") < immediate_actions.index("joint_state_spawner")
        assert "\n        joint_state_spawner," in immediate_actions
        assert "\n        xarm_traj_spawner," not in immediate_actions
        assert "\n        safety_supervisor," in immediate_actions
        if name == "manipulation.launch.py":
            assert "\n        scenario_runner," not in immediate_actions
            assert "[scenario_runner]" in launch


def test_hung_spawner_leaves_immediate_latched_stop_in_place() -> None:
    for name in ("manipulation.launch.py", "whole_robot.launch.py"):
        launch = (ROOT / "ros2_ws/src/tinker_sim_bridge/launch" / name).read_text(
            encoding="utf-8"
        )
        launch_body = launch[launch.index("def _resolve"):]
        immediate_actions = launch_body.split("    return [", 1)[1].split("\n    ]", 1)[0]
        assert "safety_supervisor," in immediate_actions
        assert "controller_management_ready" in launch
        assert "on_exit=[safety_supervisor" not in launch


def test_gripper_uses_reentrant_callbacks_and_multithreaded_executor() -> None:
    gripper = _source("gripper_facade.py")
    assert "ReentrantCallbackGroup" in gripper
    assert "callback_group=callbacks" in gripper
    assert "MultiThreadedExecutor(num_threads=4)" in gripper
    assert "executor.spin()" in gripper
    assert "rclpy.spin(node)" not in gripper


def test_gripper_checks_stop_before_emitting_command() -> None:
    tree = ast.parse(_source("gripper_facade.py"))
    execute = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "_execute"
    )
    publish_lines = [
        node.lineno
        for node in ast.walk(execute)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "publish"
    ]
    stop_check_lines = [
        node.lineno
        for node in ast.walk(execute)
        if isinstance(node, ast.Name) and node.id == "stopped"
    ]
    assert publish_lines and stop_check_lines
    assert min(stop_check_lines) < min(publish_lines)
    assert "goal_handle.abort()" in _source("gripper_facade.py")


def test_gripper_retires_with_measured_zero_effort_hold() -> None:
    gripper = _source("gripper_facade.py")
    assert "_active_goal_reserved" in gripper
    assert "_release_goal" in gripper
    assert "_publish_hold(position)" in gripper
    assert "command.effort = [0.0]" in gripper
    assert "contact_received_at >= start_wall" in gripper
    assert "contact_max_age_s" in gripper


def test_installed_bridge_contains_manipulation_launch() -> None:
    setup = (ROOT / "ros2_ws/src/tinker_sim_bridge/setup.py").read_text(
        encoding="utf-8"
    )
    assert '"launch/manipulation.launch.py"' in setup
