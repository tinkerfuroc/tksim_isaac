from __future__ import annotations

import json
import os
import sys
from pathlib import Path

_project_root = Path(os.environ["TINKER_SIM_ROOT"]) if os.environ.get("TINKER_SIM_ROOT") else Path(__file__).resolve().parents[4]
_tools = _project_root / "tools"
if _tools.is_dir() and str(_tools) not in sys.path:
    sys.path.insert(0, str(_tools))

from tinker_sim_deploy.runtime import resolve_current_artifact

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve(context):
    root = Path(LaunchConfiguration("project_root").perform(context)).resolve()
    workspace_value = LaunchConfiguration("tinker_workspace").perform(context).strip()
    if not workspace_value:
        raise RuntimeError("tinker_workspace is required; set TINKER_WS or pass tinker_workspace:=...")
    workspace = Path(workspace_value).expanduser().resolve()
    if not workspace.is_dir():
        raise RuntimeError(f"tinker workspace not found: {workspace}")
    qualification = LaunchConfiguration("qualification").perform(context).lower() in {"1", "true", "yes"}
    resolved_artifact = resolve_current_artifact(root)
    artifact = resolved_artifact.artifact_dir
    if qualification:
        source_lock = resolved_artifact.source_lock
        for record in source_lock["files"]:
            path = workspace / record["path"]
            if not path.is_file():
                raise RuntimeError(f"qualification blocked by missing Tinker source: {path}")
    calibration = root / "simulation/calibration/tinker2-missing.json"
    calibration_raw = json.loads(calibration.read_text(encoding="utf-8"))
    if qualification and calibration_raw.get("status") != "calibrated":
        raise RuntimeError("qualification blocked: synchronized Tinker 2 navigation calibration is missing")
    bridge_share = Path(FindPackageShare("tinker_sim_bridge").perform(context))
    nav_share = Path(FindPackageShare("navigation_bringup").perform(context))
    env = {"PYTHONPATH": str(root / "simulation") + os.pathsep + os.environ.get("PYTHONPATH", "")}
    return [
        Node(
            package="tinker_sim_bridge", executable="base_facade", output="screen",
            parameters=[str(bridge_share / "config/base_facade.yaml"), {"calibration": str(calibration)}], additional_env=env,
        ),
        Node(
            package="tinker_sim_bridge",
            executable="command_gateway",
            output="screen",
            parameters=[str(bridge_share / "config/command_gateway.yaml")],
            additional_env=env,
        ),
        Node(package="tinker_sim_bridge", executable="contract_guard", output="screen"),
        Node(
            package="tinker_sim_bridge", executable="initial_pose", output="screen",
            parameters=[{"use_sim_time": True, "x": 0.0, "y": 0.0, "yaw": 0.0}],
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher", output="screen",
            parameters=[{"use_sim_time": True, "robot_description": (artifact / "robot.urdf").read_text(encoding="utf-8")}],
        ),
        # Match the physical Livox driver contract.  The robot URDF's visual
        # sensor link is named livox_frame, while hardware messages and Nav2
        # intentionally use the separate livox360 frame.
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="livox360_static_tf",
            arguments=[
                "--x", "0.12", "--y", "0.0", "--z", "0.25",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "base_link", "--child-frame-id", "livox360",
            ],
            output="screen",
        ),
        Node(
            package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node", name="pointcloud_to_laserscan",
            output="screen", parameters=[str(bridge_share / "config/pointcloud_to_laserscan.yaml")],
            remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
        ),
        # Reuse the existing localization and controller implementations, but
        # launch them as independent processes.  The monolithic bringup always
        # starts RViz and its composed activation can block AMCL's initial-pose
        # callback on a headless server.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_share / "launch/localization_no_ekf_launch.py")),
            launch_arguments={
                "use_sim_time": "True", "map": str(artifact / "map.yaml"),
                "params_file": str(workspace / "src/tk26_navigation/src/navigation_bringup/params/nav2_dwb_params.yaml"),
                "autostart": "True", "use_composition": "False", "use_respawn": "False",
            }.items(),
        ),
        # The hardware localization launch does not propagate use_sim_time to
        # robot_localization.  Launch the same EKF parameters here with the
        # simulation clock explicitly enabled; mixing wall and sim epochs
        # makes odom->base numerically diverge.
        Node(
            package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
            parameters=[str(nav_share / "params/ekf.yaml"), {"use_sim_time": True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_share / "launch/navigation_dwb_launch.py")),
            launch_arguments={
                "use_sim_time": "True",
                "params_file": str(workspace / "src/tk26_navigation/src/navigation_bringup/params/nav2_dwb_params.yaml"),
                "autostart": "True", "use_composition": "False", "use_respawn": "False",
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("project_root", default_value=os.environ.get("TINKER_SIM_ROOT", str(_project_root))),
        DeclareLaunchArgument("tinker_workspace", default_value=os.environ.get("TINKER_WS", "")),

        DeclareLaunchArgument("qualification", default_value="false"),
        SetEnvironmentVariable("ROBOT_NAME", "tinker2"),
        OpaqueFunction(function=_resolve),
    ])
