from __future__ import annotations

import json
import launch.logging
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 8))


def _topic_control_description(urdf: str) -> str:
    root = ET.fromstring(urdf)
    for existing in list(root.findall("ros2_control")):
        root.remove(existing)
    control = ET.SubElement(root, "ros2_control", name="TinkerTopicSystem", type="system")
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "topic_based_ros2_control/TopicBasedSystem"
    for name, value in (
        ("joint_commands_topic", "/sim/controller/ros2_control_commands"),
        ("joint_states_topic", "/isaac_joint_states"),
        ("trigger_joint_command_threshold", "-1"),
    ):
        ET.SubElement(hardware, "param", name=name).text = value
    for name in ARM_JOINTS:
        joint = ET.SubElement(control, "joint", name=name)
        ET.SubElement(joint, "command_interface", name="position")
        ET.SubElement(joint, "command_interface", name="velocity")
        ET.SubElement(joint, "state_interface", name="position")
        ET.SubElement(joint, "state_interface", name="velocity")
        ET.SubElement(joint, "state_interface", name="effort")
    return ET.tostring(root, encoding="unicode")


def _bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _process_exit_actions(event, label: str, success_actions):
    """Gate launch progression on a successful process exit."""
    if event.returncode == 0:
        return success_actions
    launch.logging.get_logger("tinker_sim.manipulation_launch").error(
        f"{label} exited with return code {event.returncode}; shutting down"
    )
    return [
        EmitEvent(
            event=Shutdown(
                reason=f"{label} failed with return code {event.returncode}"
            )
        )
    ]


def _resolve(context):
    root = Path(LaunchConfiguration("project_root").perform(context)).resolve()
    # Resolve the workspace argument here so launch rejects an invalid path
    # early, while keeping the Tinker source tree read-only at runtime.
    workspace = Path(LaunchConfiguration("tinker_workspace").perform(context)).resolve()
    if not workspace.is_dir():
        raise RuntimeError(f"tinker workspace not found: {workspace}")
    scenario = LaunchConfiguration("scenario").perform(context)
    if not scenario or "/" in scenario or "\\" in scenario or scenario in {".", ".."}:
        raise RuntimeError(f"unsafe scenario id: {scenario!r}")
    seed = LaunchConfiguration("seed").perform(context)
    reset_attempts = LaunchConfiguration("reset_attempts").perform(context)
    reset_retry_delay = LaunchConfiguration("reset_retry_delay").perform(context)
    qualification = _bool(LaunchConfiguration("qualification").perform(context))
    attempt_value = LaunchConfiguration("attempt_dir").perform(context).strip()
    attempt_dir = Path(attempt_value).expanduser().resolve() if attempt_value else None

    scenario_file = root / "simulation/scenarios" / f"{scenario}.json"
    if not scenario_file.is_file():
        raise RuntimeError(f"scenario not found: {scenario_file}")
    current = json.loads(
        (root / "artifacts/robot/tinker2/current.json").read_text(encoding="utf-8")
    )
    manifest_path = Path(current["manifest"])
    if not manifest_path.is_absolute():
        manifest_path = root / manifest_path
    artifact = manifest_path.parent
    robot_description = _topic_control_description(
        (artifact / "robot.urdf").read_text(encoding="utf-8")
    )
    share = Path(FindPackageShare("tinker_sim_bridge").perform(context))

    scenario_arguments = [
        "--root", str(root),
        "--scenario", scenario,
        "--seed", seed,
        "--reset-attempts", reset_attempts,
        "--reset-retry-delay", reset_retry_delay,
    ]
    if attempt_dir is not None:
        scenario_arguments.extend(["--report", str(attempt_dir / "scenario-runner.json")])

    evaluator_jsonl = ""
    if attempt_dir is not None:
        evaluator_jsonl = str(attempt_dir / "evaluator.jsonl")
    elif os.environ.get("TINKER_SIM_EVALUATOR_JSONL"):
        evaluator_jsonl = os.environ["TINKER_SIM_EVALUATOR_JSONL"]
    safety_parameters = [{"use_sim_time": True}]
    evaluator_parameters = [
        {
            "use_sim_time": True,
            "scenario": scenario,
            "task": scenario,
            "jsonl_path": evaluator_jsonl,
        }
    ]
    python_env = {"PYTHONPATH": str(root / "simulation") + os.pathsep + os.environ.get("PYTHONPATH", "")}
    joint_state_spawner = Node(
        package="tinker_sim_bridge",
        executable="controller_reconciler",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    xarm_traj_spawner = Node(
        package="tinker_sim_bridge",
        executable="controller_reconciler",
        arguments=[
            "xarm7_traj_controller",
            "--controller-manager",
            "/controller_manager",
            "--ready-node",
            "/tinker_sim_safety_supervisor",
            "--ready-parameter",
            "controller_management_ready",
            "--ready-value",
            "true",
            "--ready-timeout",
            "15.0",
        ],
        output="screen",
    )
    safety_supervisor = Node(
        package="tinker_sim_bridge",
        executable="safety_supervisor",
        output="screen",
        parameters=safety_parameters,
        additional_env=python_env,
    )
    scenario_runner = Node(
        package="tinker_sim_bridge",
        executable="scenario_runner",
        # The runner blocks on the standard simulation_interfaces services
        # before reset, spawn, and play operations.
        arguments=scenario_arguments,
        output="screen",
        parameters=[{"use_sim_time": True}],
        additional_env=python_env,
    )
    return [
        # The latched stop is live before controller setup starts. Controller
        # switching remains disabled until the successful controller handoff.
        safety_supervisor,
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": True},
                str(share / "config/controllers.yaml"),
            ],
        ),
        joint_state_spawner,
        Node(
            package="tinker_sim_bridge",
            executable="command_gateway",
            output="screen",
            parameters=[str(share / "config/command_gateway.yaml"), {"use_sim_time": True}],
            additional_env=python_env,
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=lambda event, _context: _process_exit_actions(
                    event, "joint_state_broadcaster reconciliation", [xarm_traj_spawner]
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=xarm_traj_spawner,
                on_exit=lambda event, _context: _process_exit_actions(
                    event, "xarm7 trajectory reconciliation", [scenario_runner]
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=scenario_runner,
                on_exit=lambda event, _context: _process_exit_actions(
                    event, "scenario runner", []
                ),
            )
        ),
        Node(
            package="tinker_sim_bridge",
            executable="xarm_facade",
            output="screen",
            parameters=[{"use_sim_time": True}],
            additional_env=python_env,
        ),
        Node(
            package="tinker_sim_bridge",
            executable="gripper_facade",
            output="screen",
            parameters=[{"use_sim_time": True}],
            additional_env=python_env,
        ),
        Node(
            package="tinker_sim_bridge",
            executable="pan_tilt_facade",
            output="screen",
            parameters=[{"use_sim_time": True}],
            additional_env=python_env,
        ),
        Node(
            package="tinker_sim_bridge",
            executable="contract_guard",
            output="screen",
            parameters=[{"use_sim_time": True, "profile": "manipulation"}],
            additional_env=python_env,
        ),
        Node(
            package="tinker_sim_bridge",
            executable="truth_evaluator",
            output="screen",
            parameters=evaluator_parameters,
            additional_env=python_env,
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": True, "robot_description": robot_description}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=os.environ.get("TINKER_SIM_ROOT", "/home/tinker/tinker-sim/6.0.1"),
            ),
            DeclareLaunchArgument(
                "tinker_workspace",
                default_value=os.environ.get("TINKER_WS", "/home/tinker/tk25_ws"),
            ),
            DeclareLaunchArgument("scenario", default_value="pick-deliver-place"),
            DeclareLaunchArgument("seed", default_value="0"),
            DeclareLaunchArgument("reset_attempts", default_value="3"),
            DeclareLaunchArgument("reset_retry_delay", default_value="0.5"),
            DeclareLaunchArgument("qualification", default_value="false"),
            DeclareLaunchArgument(
                "attempt_dir", default_value=os.environ.get("TINKER_SIM_ATTEMPT_DIR", "")
            ),
            OpaqueFunction(function=_resolve),
        ]
    )
