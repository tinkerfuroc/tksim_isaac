from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve(context):
    root = Path(LaunchConfiguration("project_root").perform(context)).resolve()
    scenario = LaunchConfiguration("scenario").perform(context)
    scenario_file = root / "simulation/scenarios" / f"{scenario}.json"
    if not scenario_file.is_file():
        raise RuntimeError(f"scenario not found: {scenario_file}")
    return [
        Node(
            package="tinker_sim_bridge",
            executable="audio_fixtures",
            output="screen",
            parameters=[
                {"use_sim_time": True, "scenario_file": str(scenario_file)}
            ],
        )
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
                "scenario", default_value="reception-seat-assignment"
            ),
            OpaqueFunction(function=_resolve),
        ]
    )
