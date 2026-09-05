from __future__ import annotations

import launch.logging
import os
import sys
from pathlib import Path

_project_root = Path(os.environ["TINKER_SIM_ROOT"]) if os.environ.get("TINKER_SIM_ROOT") else Path(__file__).resolve().parents[4]
_tools = _project_root / "tools"
if _tools.is_dir() and str(_tools) not in sys.path:
    sys.path.insert(0, str(_tools))

from tinker_sim_deploy.runtime import resolve_current_artifact, topic_control_description

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


def _resolve(context):
    root = Path(LaunchConfiguration("project_root").perform(context)).resolve()
    resolved_artifact = resolve_current_artifact(root)
    robot_description = topic_control_description(resolved_artifact.robot_urdf)
    share = Path(FindPackageShare("tinker_sim_bridge").perform(context))
    joint_state_spawner = Node(
        package="tinker_sim_bridge",
        executable="controller_reconciler",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            # Isaac startup exceeds the 5s x 3 default budget on slower hosts.
            "--service-timeout",
            "15.0",
        ],
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
                    "TINKER_SIM_ROOT", str(_project_root)
                ),
            ),
            DeclareLaunchArgument(
                "tinker_workspace",
                default_value=os.environ.get("TINKER_WS", ""),
            ),
            DeclareLaunchArgument("qualification", default_value="false"),
            OpaqueFunction(function=_resolve),
        ]
    )
