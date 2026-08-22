from __future__ import annotations

import json
import os
import sys
from pathlib import Path

_project_root = Path(os.environ["TINKER_SIM_ROOT"]) if os.environ.get("TINKER_SIM_ROOT") else Path(__file__).resolve().parents[4]
_tools = _project_root / "tools"
if _tools.is_dir() and str(_tools) not in sys.path:
    sys.path.insert(0, str(_tools))

from tinker_sim_deploy.runtime import (
    resolve_arena_map_yaml,
    resolve_current_artifact,
    scenario_arena_id,
    topic_control_description,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# GPSR needs cameras, arm, and Nav2 running together. manipulation.launch.py
# and navigation.launch.py both unconditionally start same-named singleton
# nodes (command_gateway, safety_supervisor, contract_guard,
# robot_state_publisher) with divergent parameters, so this composite is an
# explicit de-duplicating merge of both sources rather than an
# IncludeLaunchDescription of either -- including both verbatim would
# collide on those singletons.


def _process_exit_actions(event, label: str, success_actions):
    """Gate launch progression on a successful process exit."""
    if event.returncode == 0:
        return success_actions
    import launch.logging

    launch.logging.get_logger("tinker_sim.gpsr_launch").error(
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
    workspace_value = LaunchConfiguration("tinker_workspace").perform(context).strip()
    if not workspace_value:
        raise RuntimeError("tinker_workspace is required; set TINKER_WS or pass tinker_workspace:=...")
    workspace = Path(workspace_value).expanduser().resolve()
    if not workspace.is_dir():
        raise RuntimeError(f"tinker workspace not found: {workspace}")

    scenario = LaunchConfiguration("scenario").perform(context)
    if not scenario or "/" in scenario or "\\" in scenario or scenario in {".", ".."}:
        raise RuntimeError(f"unsafe scenario id: {scenario!r}")
    seed = LaunchConfiguration("seed").perform(context)

    scenario_file = root / "simulation/scenarios" / f"{scenario}.json"
    if not scenario_file.is_file():
        raise RuntimeError(f"scenario not found: {scenario_file}")

    # Seed AMCL from the scenario's own spawn pose rather than a hardcoded
    # (0, 0, 0) -- scenarios such as gpsr-rcw2026 spawn the robot away from
    # the map origin, and an unseeded AMCL can start inside occupied space.
    # Fall back to (0, 0, 0) if the scenario doesn't specify one.
    initial_pose_xyz = (0.0, 0.0, 0.0)
    scenario_data = {}
    try:
        scenario_data = json.loads(scenario_file.read_text(encoding="utf-8"))
        robot_initial_pose = scenario_data.get("robot", {}).get("initial_pose")
        if isinstance(robot_initial_pose, list) and len(robot_initial_pose) >= 3:
            initial_pose_xyz = tuple(float(v) for v in robot_initial_pose[:3])
    except (json.JSONDecodeError, OSError, TypeError, ValueError):
        pass

    resolved_artifact = resolve_current_artifact(root)
    artifact = resolved_artifact.artifact_dir
    # Exactly one robot_state_publisher. manipulation's
    # topic_control_description() form is a superset (control-topic
    # augmented) of navigation's raw URDF read, so it is used here.
    robot_description = topic_control_description(resolved_artifact.robot_urdf)

    calibration = root / "simulation/calibration/tinker2-missing.json"
    # AMCL must localize on the arena the simulator raycasts its lidar
    # against.  The robot artifact's colocated map.yaml is the hardware arena
    # (no occupied cell in common with rcw2026), so a scenario that names an
    # arena selects that arena artifact's map unless map_yaml:= overrides it.
    map_yaml_value = LaunchConfiguration("map_yaml").perform(context).strip()
    arena_id = scenario_arena_id(scenario_data)
    if map_yaml_value:
        map_yaml = Path(map_yaml_value).expanduser().resolve()
    elif arena_id:
        map_yaml = resolve_arena_map_yaml(root, arena_id)
    else:
        map_yaml = artifact / "map.yaml"
    if not map_yaml.is_file():
        raise RuntimeError(f"navigation map does not exist: {map_yaml}")

    bridge_share = Path(FindPackageShare("tinker_sim_bridge").perform(context))
    nav_share = Path(FindPackageShare("navigation_bringup").perform(context))

    safety_source_deadline_s = float(
        LaunchConfiguration("safety_source_deadline_s").perform(context)
    )

    # Which arm controller profile ros2_control loads. Defaults to the shipped
    # controllers.yaml; pass an absolute path to override. config/
    # controllers.sim-clock.yaml disables path-tolerance enforcement for hosts
    # where the sim's real-time factor is low enough that a MoveIt trajectory
    # (parameterised for real-speed execution) aborts mid-flight with
    # PATH_TOLERANCE_VIOLATED before the arm can converge.
    controllers_value = LaunchConfiguration("controllers_file").perform(context).strip()
    controllers_file = (
        str(Path(controllers_value).expanduser().resolve())
        if controllers_value
        else str(bridge_share / "config/controllers.yaml")
    )
    if not Path(controllers_file).is_file():
        raise RuntimeError(f"controllers file not found: {controllers_file}")

    python_env = {"PYTHONPATH": str(root / "simulation") + os.pathsep + os.environ.get("PYTHONPATH", "")}

    joint_state_spawner = Node(
        package="tinker_sim_bridge",
        executable="controller_reconciler",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            # Isaac manipulation-core startup exceeds the 5s x 3 default budget
            # on slower hosts (measured ~26s to controller_manager readiness).
            "--service-timeout",
            "15.0",
        ],
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

    # One safety_supervisor for the whole composite. manage_controllers is
    # left at its default True -- the composite starts ros2_control_node, so
    # controllers exist to manage -- and required_sources covers both the
    # arm's collision-avoidance need and navigation's xarm-independent case.
    safety_supervisor = Node(
        package="tinker_sim_bridge",
        executable="safety_supervisor",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "required_source_deadline_s": safety_source_deadline_s,
                "required_sources": ["xarm", "collision"],
            }
        ],
        additional_env=python_env,
    )

    scenario_runner = Node(
        package="tinker_sim_bridge",
        executable="scenario_runner",
        arguments=[
            "--root", str(root),
            "--scenario", scenario,
            "--seed", seed,
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        additional_env=python_env,
    )

    audio_fixtures = Node(
        package="tinker_sim_bridge",
        executable="audio_fixtures",
        output="screen",
        parameters=[
            {"use_sim_time": True, "scenario_file": str(scenario_file)}
        ],
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
                controllers_file,
            ],
        ),
        joint_state_spawner,
        # One command_gateway, shared by the arm and base command paths.
        Node(
            package="tinker_sim_bridge",
            executable="command_gateway",
            output="screen",
            parameters=[str(bridge_share / "config/command_gateway.yaml"), {"use_sim_time": True}],
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
        # One contract_guard, using manipulation's "manipulation" profile --
        # the arm contract is the stricter of the two.
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
            parameters=[
                {
                    "use_sim_time": True,
                    "scenario": scenario,
                    "task": scenario,
                }
            ],
            additional_env=python_env,
        ),
        audio_fixtures,
        # One robot_state_publisher, using manipulation's control-topic
        # augmented description.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": True, "robot_description": robot_description}],
        ),
        Node(
            package="tinker_sim_bridge",
            executable="base_facade",
            output="screen",
            parameters=[str(bridge_share / "config/base_facade.yaml"), {"calibration": str(calibration)}],
            additional_env=python_env,
        ),
        Node(
            package="tinker_sim_bridge",
            executable="initial_pose",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "x": initial_pose_xyz[0],
                    "y": initial_pose_xyz[1],
                    "yaw": initial_pose_xyz[2],
                }
            ],
        ),
        # Make `world` reachable for the manipulation stack.
        #
        # pick_and_place.cpp::transform_chain_ready() rejects EVERY joint-move
        # goal unless both `world -> base_link` and `base_link -> link_tcp`
        # resolve. The robot URDF supplies a fixed `world -> base_link` joint,
        # which is correct for a fixed-base manipulation-only sim -- but this
        # composite also runs navigation, which publishes a dynamic
        # `odom -> base_link`. A frame cannot have two parents: tf2 keeps
        # `odom -> base_link` and silently drops the `world` edge, so `world`
        # becomes unreachable and manipulation fails with the unhelpful
        # "tf chain lookup unavailable".
        #
        # Attach `world` ABOVE the tree root instead of re-parenting anything:
        # `map` has no parent, while `odom` is already owned by AMCL. That
        # yields world -> map -> odom -> base_link -> ... -> link_tcp with no
        # frame gaining a second parent. The real robot expresses the same
        # intent as `fix_odom_to_world` (world -> odom, NOT world -> base_link)
        # in tinker_urdf/src/mobile_manipulator.urdf.xacro.
        #
        # Measured live 2026-08-20: without this edge
        # can_transform(base_link <- world) == 0 and pick_and_place rejected
        # every goal; with it the transform resolves and goals are accepted.
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="world_static_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "world", "--child-frame-id", "map",
            ],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        # Match the physical Livox driver contract. The robot URDF's visual
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
        # launch them as independent processes. The monolithic bringup always
        # starts RViz and its composed activation can block AMCL's
        # initial-pose callback on a headless server.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_share / "launch/localization_no_ekf_launch.py")),
            launch_arguments={
                "use_sim_time": "True", "map": str(map_yaml),
                "params_file": str(workspace / "src/tk26_navigation/src/navigation_bringup/params/nav2_dwb_params.yaml"),
                "autostart": "True", "use_composition": "False", "use_respawn": "False",
            }.items(),
        ),
        # The hardware localization launch does not propagate use_sim_time to
        # robot_localization. Launch the same EKF parameters here with the
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
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=os.environ.get("TINKER_SIM_ROOT", str(_project_root)),
            ),
            DeclareLaunchArgument(
                "tinker_workspace",
                default_value=os.environ.get("TINKER_WS", ""),
            ),
            DeclareLaunchArgument("scenario", default_value="gpsr-rcw2026"),
            DeclareLaunchArgument("map_yaml", default_value=""),
            DeclareLaunchArgument("seed", default_value="0"),
            DeclareLaunchArgument(
                "safety_source_deadline_s", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value="",
                description=(
                    "Absolute path to the ros2_control controller profile. "
                    "Empty uses config/controllers.yaml. Use "
                    "config/controllers.sim-clock.yaml on hosts where the "
                    "sim's real-time factor makes the arm abort with "
                    "PATH_TOLERANCE_VIOLATED."
                ),
            ),
            SetEnvironmentVariable("ROBOT_NAME", "tinker2"),
            OpaqueFunction(function=_resolve),
        ]
    )
