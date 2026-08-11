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


def _planning_overlay_actions(context, root: Path, workspace: Path):
    """Return the integrated staging overlay include, or ``None`` when the
    legacy non-overlay path is requested.

    The staging overlay composes the Task 3/4/5 providers into the first
    integrated OMPL/readiness boundary; it is the only path that installs the
    staged production planning/task overlay.
    """
    share = Path(FindPackageShare("tinker_sim_bridge").perform(context))
    project_root_text = str(root)
    workspace_text = str(workspace)
    integrated = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / "launch" / "integrated_ompl_manipulation.launch.py")
        ),
        launch_arguments={
            "project_root": project_root_text,
            "tinker_workspace": workspace_text,
            "scenario": LaunchConfiguration("scenario"),
            "seed": LaunchConfiguration("seed"),
            "reset_attempts": LaunchConfiguration("reset_attempts"),
            "reset_retry_delay": LaunchConfiguration("reset_retry_delay"),
            "qualification": LaunchConfiguration("qualification"),
            "model_bundle_manifest": LaunchConfiguration("model_bundle_manifest"),
            "provider_manifest_path": LaunchConfiguration("provider_manifest_path"),
            "attempt_dir": LaunchConfiguration("attempt_dir"),
        }.items(),
    )
    return [integrated]


def _resolve(context):
    root = Path(LaunchConfiguration("project_root").perform(context)).resolve()
    # Resolve the workspace argument here so launch rejects an invalid path
    # early, while keeping the Tinker source tree read-only at runtime.
    workspace_value = LaunchConfiguration("tinker_workspace").perform(context).strip()
    if not workspace_value:
        raise RuntimeError("tinker_workspace is required; set TINKER_WS or pass tinker_workspace:=...")
    workspace = Path(workspace_value).expanduser().resolve()
    if not workspace.is_dir():
        raise RuntimeError(f"tinker workspace not found: {workspace}")
    if _bool(LaunchConfiguration("planning_overlay").perform(context)):
        return _planning_overlay_actions(context, root, workspace)
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
    resolved_artifact = resolve_current_artifact(root)
    artifact = resolved_artifact.artifact_dir
    robot_description = topic_control_description(resolved_artifact.robot_urdf)
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
    raw_jsonl_path = ""
    if attempt_dir is not None:
        raw_jsonl_path = str(attempt_dir / "physics_truth.jsonl")
    elif os.environ.get("TINKER_SIM_TRUTH_JSONL"):
        raw_jsonl_path = os.environ["TINKER_SIM_TRUTH_JSONL"]
    safety_parameters = [{"use_sim_time": True}]
    evaluator_parameters = [
        {
            "use_sim_time": True,
            "scenario": scenario,
            "task": scenario,
            "jsonl_path": evaluator_jsonl,
            "raw_jsonl_path": raw_jsonl_path,
        }
    ]
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
                default_value=os.environ.get("TINKER_SIM_ROOT", str(_project_root)),
            ),
            DeclareLaunchArgument(
                "tinker_workspace",
                default_value=os.environ.get("TINKER_WS", ""),
            ),
            DeclareLaunchArgument("scenario", default_value="pick-deliver-place"),
            DeclareLaunchArgument("seed", default_value="0"),
            DeclareLaunchArgument("reset_attempts", default_value="3"),
            DeclareLaunchArgument("reset_retry_delay", default_value="0.5"),
            DeclareLaunchArgument("qualification", default_value="false"),
            DeclareLaunchArgument(
                "attempt_dir", default_value=os.environ.get("TINKER_SIM_ATTEMPT_DIR", "")
            ),
            DeclareLaunchArgument("planning_overlay", default_value="false"),
            DeclareLaunchArgument(
                "model_bundle_manifest",
                default_value=os.environ.get("TINKER_SIM_MODEL_BUNDLE_MANIFEST", ""),
            ),
            DeclareLaunchArgument(
                "provider_manifest_path",
                default_value=os.environ.get("TINKER_SIM_PROVIDER_MANIFEST", ""),
            ),
            OpaqueFunction(function=_resolve),
        ]
    )
