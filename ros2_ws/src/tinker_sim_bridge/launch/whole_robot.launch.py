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
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 8))


def _process_exit_actions(event, label: str, success_actions):
    """Gate controller setup progression on a successful process exit."""
    if event.returncode == 0:
        return success_actions
    launch.logging.get_logger("tinker_sim.whole_robot_launch").error(
        f"{label} exited with return code {event.returncode}; shutting down"
    )
    return [
        EmitEvent(
            event=Shutdown(
                reason=f"{label} failed with return code {event.returncode}"
            )
        )
    ]


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


def _resolve(context):
    root = Path(LaunchConfiguration("project_root").perform(context)).resolve()
    current = json.loads(
        (root / "artifacts/robot/tinker2/current.json").read_text(encoding="utf-8")
    )
    manifest_path = Path(current["manifest"])
    if not manifest_path.is_absolute():
        manifest_path = root / manifest_path
    robot_description = _topic_control_description(
        (manifest_path.parent / "robot.urdf").read_text(encoding="utf-8")
    )
    share = Path(FindPackageShare("tinker_sim_bridge").perform(context))
    joint_state_spawner = Node(
        package="tinker_sim_bridge",
        executable="controller_reconciler",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    xarm_traj_spawner = Node(
        package="tinker_sim_bridge",
        executable="controller_reconciler",
        arguments=["xarm7_traj_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    safety_supervisor = Node(
        package="tinker_sim_bridge",
        executable="safety_supervisor",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    controller_ready_setter = ExecuteProcess(
        cmd=[
            "ros2",
            "param",
            "set",
            "/tinker_sim_safety_supervisor",
            "controller_management_ready",
            "true",
            "--no-daemon",
        ],
        output="screen",
    )
    return [
        # Keep the effective stop active while controller setup is pending.
        safety_supervisor,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/navigation.launch.py")),
            launch_arguments={
                "project_root": str(root),
                "tinker_workspace": LaunchConfiguration("tinker_workspace"),
                "qualification": LaunchConfiguration("qualification"),
            }.items(),
        ),
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
                    event, "xarm7 trajectory reconciliation", [controller_ready_setter]
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_ready_setter,
                on_exit=lambda event, _context: _process_exit_actions(
                    event, "controller-management readiness", []
                ),
            )
        ),
        Node(
            package="tinker_sim_bridge",
            executable="gripper_facade",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="tinker_sim_bridge",
            executable="xarm_facade",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="tinker_sim_bridge",
            executable="pan_tilt_facade",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=os.environ.get(
                    "TINKER_SIM_ROOT", "/home/tinker/tinker-sim/6.0.1"
                ),
            ),
            DeclareLaunchArgument(
                "tinker_workspace",
                default_value=os.environ.get("TINKER_WS", "/home/tinker/tk25_ws"),
            ),
            DeclareLaunchArgument("qualification", default_value="false"),
            OpaqueFunction(function=_resolve),
        ]
    )
