from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "ros2_ws/src/tinker_sim_bridge"


def test_runner_preserves_logical_ids_and_stable_namespace_contract() -> None:
    source = (BRIDGE / "tinker_sim_bridge/scenario_runner.py").read_text(encoding="utf-8")
    assert 'request.name = str(operation.payload["name"])' in source
    assert 'request.entity_namespace = str(operation.payload["entity_namespace"])' in source
    assert '"logical_id": str(operation.payload["logical_id"])' in source
    assert '"prim_path": str(operation.payload["prim_path"])' in source


def test_manipulation_launch_uses_installed_guard_profile_and_evaluator_path() -> None:
    launch = (BRIDGE / "launch/manipulation.launch.py").read_text(encoding="utf-8")
    assert '"profile": "manipulation"' in launch
    assert '"jsonl_path": evaluator_jsonl' in launch
    assert '"project_root": str(root)' not in launch
    assert "attempt_dir.mkdir" not in launch
    assert '"qualification": qualification' not in launch
    assert '"attempt_dir": str(attempt_dir or "")' not in launch


def test_whole_robot_launch_has_the_same_safety_supervisor_contract() -> None:
    launch = (BRIDGE / "launch/whole_robot.launch.py").read_text(encoding="utf-8")
    assert 'executable="safety_supervisor"' in launch
    assert 'parameters=[{"use_sim_time": True}]' in launch


def test_manipulation_controller_chain_and_immediate_safety() -> None:
    launch = (BRIDGE / "launch/manipulation.launch.py").read_text(encoding="utf-8")
    assert "TimerAction" not in launch
    assert "joint_state_spawner = Node(" in launch
    assert "xarm_traj_spawner = Node(" in launch
    assert "target_action=joint_state_spawner" in launch
    assert "target_action=xarm_traj_spawner" in launch
    assert "event.returncode == 0" in launch
    assert "controller_ready_setter" not in launch
    assert "controller_management_ready" in launch
    assert "EmitEvent" in launch
    assert "Shutdown(" in launch
    launch_body = launch[launch.index("def _resolve"):]
    immediate_actions = launch_body.split("    return [", 1)[1].split("\n    ]", 1)[0]
    assert immediate_actions.index("safety_supervisor") < immediate_actions.index("joint_state_spawner")
    assert "\n        joint_state_spawner," in immediate_actions
    assert "\n        xarm_traj_spawner," not in immediate_actions
    assert "\n        safety_supervisor," in immediate_actions
    assert "\n        scenario_runner," not in immediate_actions
    assert "[scenario_runner]" in launch
    assert '"--ready-node"' in launch
    assert '"--ready-parameter"' in launch
    assert '"--ready-timeout",' in launch


def test_manipulation_launch_shuts_down_on_scenario_runner_failure() -> None:
    launch = (BRIDGE / "launch/manipulation.launch.py").read_text(encoding="utf-8")
    assert "target_action=scenario_runner" in launch
    assert 'event, "scenario runner", []' in launch
    assert 'DeclareLaunchArgument("reset_attempts", default_value="3")' in launch
    assert 'DeclareLaunchArgument("reset_retry_delay", default_value="0.5")' in launch
    assert '"--reset-attempts", reset_attempts' in launch
    assert '"--reset-retry-delay", reset_retry_delay' in launch


def test_whole_robot_controller_chain_and_immediate_supervisor() -> None:
    launch = (BRIDGE / "launch/whole_robot.launch.py").read_text(encoding="utf-8")
    assert "TimerAction" not in launch
    assert "joint_state_spawner = Node(" in launch
    assert "xarm_traj_spawner = Node(" in launch
    assert "target_action=joint_state_spawner" in launch
    assert "target_action=xarm_traj_spawner" in launch
    assert "event.returncode == 0" in launch
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


def test_safety_controller_reactivation_requires_prior_active_state() -> None:
    safety = (
        ROOT
        / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/safety_supervisor.py"
    ).read_text(encoding="utf-8")
    assert '"/controller_manager/list_controllers"' in safety
    assert "self._controller_was_active = active" in safety
    assert "self._controller_was_active" in safety
    assert "elif not active:" in safety
    assert "controller_management_ready" in safety
    assert "self._restore_pending = self._restore_pending or" in safety


def test_humble_wrapper_uses_installed_manipulation_launch() -> None:
    wrapper = (ROOT / "scripts/launch-humble").read_text(encoding="utf-8")
    assert 'launch_file="manipulation.launch.py"' in wrapper
    assert 'ros2 launch "${project_root}/ros2_ws/src/tinker_sim_bridge/launch/' not in wrapper


def test_arm_collision_uses_dedicated_static_obstacle() -> None:
    scenario = (ROOT / "simulation/scenarios/qualification-arm-collision.json").read_text(
        encoding="utf-8"
    )
    asset = (ROOT / "simulation/assets/primitives/obstacle.usda").read_text(encoding="utf-8")
    assert "simulation/assets/primitives/obstacle.usda" in scenario
    assert "task-object.usda" not in scenario
    assert "PhysicsCollisionAPI" in asset
    assert "PhysicsRigidBodyAPI" not in asset
