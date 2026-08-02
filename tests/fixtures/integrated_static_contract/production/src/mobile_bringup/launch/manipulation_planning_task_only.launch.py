"""Fixture planning/task-only launch for the integrated OMPL static contract."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("execution_profile", default_value="sim_ompl"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("safety_required", default_value="true"),
            DeclareLaunchArgument("fixture_revision_required", default_value="true"),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                parameters=[
                    {
                        "execution_profile": "sim_ompl",
                        "use_sim_time": True,
                        "safety_required": True,
                        "fixture_revision_required": True,
                        "use_cumotion_goalset": False,
                        "use_cumotion_object_attachment": False,
                        "use_cumotion_straight_approach": False,
                        "esdf_freshness_wait_enabled": False,
                    }
                ],
            ),
        ]
    )
