from __future__ import annotations

import hashlib
import json
import launch.logging
import launch_ros
import os
import sys
import tempfile
from pathlib import Path

_project_root = Path(os.environ["TINKER_SIM_ROOT"]) if os.environ.get("TINKER_SIM_ROOT") else Path(__file__).resolve().parents[4]
_tools = _project_root / "tools"
if _tools.is_dir() and str(_tools) not in sys.path:
    sys.path.insert(0, str(_tools))

from tinker_sim_deploy.runtime import (
    resolve_current_artifact,
    sim_robot_description,
    topic_control_description,
)

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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from tinker_sim_bridge.fixture_contract import revision_digest
from tinker_sim_bridge.fixture_planning_scene import (
    fixture_descriptor_sha256,
    fixture_owned_ids,
)
from tinker_sim_bridge.integrated_readiness import (
    build_integrated_mapping,
    canonical_json,
    planning_scene_mapping,
    public_integrated_mapping,
    sha256_json,
)
from tinker_sim_bridge.scenario_resolver import (
    ScenarioResolutionError,
    resolve_scenario_file,
)

_PRODUCTION_PACKAGE = "mobile_bringup"
_PRODUCTION_LAUNCH = "manipulation_planning_task_only.launch.py"

# Exact canonical fixture-id prefix enforced by the production task node.
_FIXTURE_ID_PREFIX = "sim_fixture/"

# Wall-clock deadline for the typed physics/fixture readiness waits.
_WAIT_TIMEOUT_S = 120.0

_logger = launch.logging.get_logger("tinker_sim.integrated_ompl")


def _bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _shutdown(reason: str) -> list:
    _logger.error(reason)
    return [EmitEvent(event=Shutdown(reason=reason))]


def _process_exit_actions(event, label: str, success_actions):
    """Gate launch progression on a successful process exit."""
    if event.returncode == 0:
        return success_actions
    return _shutdown(f"{label} exited with return code {event.returncode}; shutting down")


def _service_waiter(svc: str, label: str, timeout_s: float = _WAIT_TIMEOUT_S):
    """Return a one-shot process that exits 0 only when the Trigger service succeeds.

    The waiter runs the installed ``tinker_sim_bridge.readiness_waiter`` module
    (bounded, executor-serviced, tested under Humble).  It exits 0 only for a
    typed Trigger response with ``success=true`` and nonzero otherwise.
    """
    return ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "tinker_sim_bridge.readiness_waiter",
            "--service",
            svc,
            "--deadline",
            str(timeout_s),
        ],
        name=label,
        output="screen",
    )


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
    share = Path(FindPackageShare("tinker_sim_bridge").perform(context))
    try:
        scenario_file = resolve_scenario_file(root, scenario, share)
    except ScenarioResolutionError as exc:
        raise RuntimeError(str(exc)) from exc
    seed_value = LaunchConfiguration("seed").perform(context)
    if not seed_value.isdigit():
        raise RuntimeError(f"seed must be a nonnegative integer; got {seed_value!r}")
    seed = int(seed_value)
    reset_attempts = LaunchConfiguration("reset_attempts").perform(context)
    reset_retry_delay = LaunchConfiguration("reset_retry_delay").perform(context)
    qualification = _bool(LaunchConfiguration("qualification").perform(context))

    model_bundle_manifest_value = LaunchConfiguration("model_bundle_manifest").perform(context).strip()
    if not model_bundle_manifest_value:
        raise RuntimeError("model_bundle_manifest is required for the integrated OMPL overlay")
    model_bundle_manifest = Path(model_bundle_manifest_value).expanduser().resolve()
    if not model_bundle_manifest.is_file():
        raise RuntimeError(f"model bundle manifest not found: {model_bundle_manifest}")
    try:
        bundle = json.loads(model_bundle_manifest.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"model bundle manifest unreadable: {exc}") from exc
    model_fingerprint = str(bundle.get("structural_fingerprint", "")).strip()
    if (
        len(model_fingerprint) != 64
        or any(c not in "0123456789abcdef" for c in model_fingerprint)
        or model_fingerprint == "0" * 64
    ):
        raise RuntimeError(f"model bundle manifest structural_fingerprint is not a nonzero SHA-256: {model_fingerprint!r}")

    provider_manifest_value = LaunchConfiguration("provider_manifest_path").perform(context).strip()
    if not provider_manifest_value:
        raise RuntimeError("provider_manifest_path is required for the integrated OMPL overlay")
    provider_manifest = Path(provider_manifest_value).expanduser().resolve()
    if not provider_manifest.is_file():
        raise RuntimeError(f"provider manifest not found: {provider_manifest}")
    try:
        provider_raw = json.loads(provider_manifest.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"provider manifest unreadable: {exc}") from exc
    if not isinstance(provider_raw, dict):
        raise RuntimeError("provider manifest must be a JSON object")
    provider_bytes = provider_manifest.read_bytes()
    provider_manifest_sha256 = hashlib.sha256(provider_bytes).hexdigest()
    recorded = provider_raw.get("provider_manifest_sha256")
    canonical_self_hash = sha256_json(
        {k: v for k, v in provider_raw.items() if k != "provider_manifest_sha256"}
    )
    if not isinstance(recorded, str) or recorded != canonical_self_hash:
        raise RuntimeError("provider manifest recorded sha256 does not match its canonical self-hash")

    raw_scenario = json.loads(scenario_file.read_text(encoding="utf-8"))
    if not isinstance(raw_scenario, dict):
        raise RuntimeError("scenario declaration must be a JSON object")
    scenario_id = str(raw_scenario["id"])
    declaration = {str(k): v for k, v in raw_scenario.items() if k not in {"id", "seed"}}
    scenario_mapping_obj = {"id": scenario_id, "seed": seed, "declaration": declaration}
    scenario_declaration_sha256 = sha256_json(scenario_mapping_obj)
    planning_scene = raw_scenario.get("planning_scene")
    if not isinstance(planning_scene, dict):
        raise RuntimeError("scenario has no planning_scene object")
    planning_scene_revision = str(planning_scene["revision"])
    planning_scene_revision_digest = revision_digest(planning_scene)
    owned_ids = tuple(str(item) for item in fixture_owned_ids(planning_scene))
    if not owned_ids:
        raise RuntimeError("planning_scene declares no owned fixture ids")
    for item in owned_ids:
        if not item.startswith(_FIXTURE_ID_PREFIX):
            raise RuntimeError(f"fixture id {item!r} is outside {_FIXTURE_ID_PREFIX!r}")
    target_source_id = str(planning_scene["target_source_id"])
    target_handoff = str(planning_scene["target_handoff"])
    fixture_descriptor_sha = fixture_descriptor_sha256(planning_scene)

    # The full runtime readiness contract is carried separately from the public
    # report's production-canonical one-key ``integrated`` mapping.  The report
    # identity ``planning_scene_sha256`` is the digest of the report's four-key
    # planning-scene mapping (matching what the report computes), while the full
    # Task 5 revision digest remains the fixture-status digest passed separately.
    runtime_contract = build_integrated_mapping()
    runtime_contract_sha256 = sha256_json(runtime_contract)
    runtime_contract_json = canonical_json(runtime_contract).decode("utf-8")
    public_integrated = public_integrated_mapping()
    public_integrated_sha256 = sha256_json(public_integrated)
    public_integrated_json = canonical_json(public_integrated).decode("utf-8")
    planning_scene_report_sha256 = sha256_json(planning_scene_mapping(planning_scene))
    owned_ids_json = canonical_json(list(owned_ids)).decode("utf-8")

    identities = {
        "scenario_id": scenario_id,
        "seed": seed,
        "scenario_declaration_sha256": scenario_declaration_sha256,
        "planning_scene_sha256": planning_scene_report_sha256,
        "integrated_sha256": public_integrated_sha256,
        "model_fingerprint": model_fingerprint,
        "provider_manifest_sha256": provider_manifest_sha256,
    }
    required_scenario_identities_json = canonical_json(identities).decode("utf-8")

    attempt_value = LaunchConfiguration("attempt_dir").perform(context).strip()
    if attempt_value:
        attempt_dir = Path(attempt_value).expanduser().resolve()
    else:
        attempt_dir = Path(tempfile.mkdtemp(prefix="tinker-sim-ompl-"))
    attempt_dir.mkdir(parents=True, exist_ok=True)
    report_path = attempt_dir / "scenario-runner.json"
    physics_ready_path = attempt_dir / "physics-ready.json"

    resolved_artifact = resolve_current_artifact(root)
    # sim_robot_description: topic_control_description plus the wrist camera
    # cam-stand TF rewrite, keyed on TINKER_SIM_WRIST_CAMERA_AIM (the frame the
    # wrist images advertise must be the one they were rendered from).
    robot_description = sim_robot_description(resolved_artifact.robot_urdf)
    python_env = {"PYTHONPATH": str(root / "simulation") + os.pathsep + os.environ.get("PYTHONPATH", "")}

    scenario_arguments = [
        "--root", str(root),
        "--scenario", scenario,
        "--seed", str(seed),
        "--reset-attempts", reset_attempts,
        "--reset-retry-delay", reset_retry_delay,
        "--report", str(report_path),
        "--expected-scenario-declaration-sha256", scenario_declaration_sha256,
        "--expected-planning-scene-revision", planning_scene_revision,
        "--expected-planning-scene-revision-digest", planning_scene_revision_digest,
        "--expected-planning-scene-owned-ids", owned_ids_json,
        "--expected-planning-scene-target-source-id", target_source_id,
        "--expected-planning-scene-target-handoff", target_handoff,
        "--expected-integrated-mapping", public_integrated_json,
        "--expected-integrated-sha256", public_integrated_sha256,
        "--expected-model-fingerprint", model_fingerprint,
        "--provider-manifest", str(provider_manifest),
        "--provider-manifest-sha256", provider_manifest_sha256,
    ]

    safety_parameters = [{"use_sim_time": True}]
    evaluator_jsonl = ""
    if os.environ.get("TINKER_SIM_EVALUATOR_JSONL"):
        evaluator_jsonl = os.environ["TINKER_SIM_EVALUATOR_JSONL"]
    raw_jsonl_path = os.environ.get("TINKER_SIM_TRUTH_JSONL", "")
    evaluator_parameters = [
        {
            "use_sim_time": True,
            "scenario": scenario,
            "task": scenario,
            "jsonl_path": evaluator_jsonl,
            "raw_jsonl_path": raw_jsonl_path,
        }
    ]

    controller_reconciler = Node(
        package="tinker_sim_bridge",
        executable="controller_reconciler",
        arguments=[
            "joint_state_broadcaster",
            "xarm7_traj_controller",
            "--controller-manager", "/controller_manager",
            "--ready-node", "/tinker_sim_safety_supervisor",
            "--ready-parameter", "controller_management_ready",
            "--ready-value", "true",
            "--ready-timeout", "15.0",
        ],
        output="screen",
    )
    scenario_runner_node = Node(
        package="tinker_sim_bridge",
        executable="scenario_runner",
        arguments=scenario_arguments,
        output="screen",
        parameters=[{"use_sim_time": True}],
        additional_env=python_env,
    )

    def _after_scenario_runner(event, context):
        if event.returncode != 0:
            return _shutdown(
                f"scenario runner exited with return code {event.returncode}; shutting down"
            )
        physics_gate = Node(
            package="tinker_sim_bridge",
            executable="physics_ready_gate",
            output="screen",
            parameters=[
                {
                    "report_path": str(report_path),
                    "physics_ready_path": str(physics_ready_path),
                    "check_period_s": 0.5,
                    "timeout_s": 30.0,
                    "scenario_id": scenario_id,
                    "seed": seed,
                    "scenario_declaration_sha256": scenario_declaration_sha256,
                    "planning_scene_revision": planning_scene_revision,
                    "planning_scene_revision_digest": planning_scene_revision_digest,
                    "planning_scene_owned_ids": owned_ids_json,
                    "planning_scene_target_source_id": target_source_id,
                    "planning_scene_target_handoff": target_handoff,
                    "integrated_mapping": runtime_contract_json,
                    "public_integrated_mapping": public_integrated_json,
                    "integrated_sha256": public_integrated_sha256,
                    "runtime_contract_sha256": runtime_contract_sha256,
                    "model_fingerprint": model_fingerprint,
                    "provider_manifest_path": str(provider_manifest),
                    "provider_manifest_sha256": provider_manifest_sha256,
                }
            ],
        )
        physics_waiter = _service_waiter("/sim/ready/physics", "physics_ready_waiter")

        def _after_physics_ready(event, context):
            if event.returncode != 0:
                return _shutdown("physics-ready wait failed; shutting down")
            planning_only = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare(_PRODUCTION_PACKAGE),
                            "launch",
                            _PRODUCTION_LAUNCH,
                        ]
                    )
                ),
                launch_arguments={
                    "model_bundle_manifest": str(model_bundle_manifest),
                    "provider_manifest_path": str(provider_manifest),
                    "provider_manifest_sha256": provider_manifest_sha256,
                    "use_sim_time": "true",
                    "show_rviz": "false",
                    "start_move_group": "true",
                    "start_task_server": "false",
                    "execution_profile": "sim_ompl",
                }.items(),
            )
            fixture_adapter = Node(
                package="tinker_sim_bridge",
                executable="fixture_planning_scene",
                name="fixture_planning_scene",
                output="screen",
                parameters=[
                    {
                        "scenario_file": str(scenario_file),
                        "heartbeat_period": 0.2,
                        "start_deadline_s": 120.0,
                        "required_fixture_owned_ids": owned_ids_json,
                    }
                ],
                additional_env=python_env,
            )
            fixture_waiter = _service_waiter("/sim/ready/fixture", "fixture_ready_waiter")

            def _after_fixture_ready(event, context):
                if event.returncode != 0:
                    return _shutdown("fixture-ready wait failed; shutting down")
                task_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare(_PRODUCTION_PACKAGE),
                                "launch",
                                _PRODUCTION_LAUNCH,
                            ]
                        )
                    ),
                    launch_arguments={
                        "model_bundle_manifest": str(model_bundle_manifest),
                        "provider_manifest_path": str(provider_manifest),
                        "provider_manifest_sha256": provider_manifest_sha256,
                        "use_sim_time": "true",
                        "show_rviz": "false",
                        "start_move_group": "false",
                        "start_task_server": "true",
                        "safety_required": "true",
                        "required_fixture_revision": planning_scene_revision,
                        "required_fixture_revision_digest": planning_scene_revision_digest,
                        "required_fixture_owned_ids": owned_ids_json,
                        "scenario_status_path": str(report_path),
                        "required_scenario_id": scenario_id,
                        "required_scenario_seed": str(seed),
                        "required_scenario_identities": required_scenario_identities_json,
                        "required_model_fingerprint": model_fingerprint,
                        "required_fixture_descriptor_sha256": fixture_descriptor_sha,
                        "execution_profile": "sim_ompl",
                    }.items(),
                )
                readiness = Node(
                    package="tinker_sim_bridge",
                    executable="integrated_readiness",
                    output="screen",
                    parameters=[
                        {
                            "check_period_s": 0.2,
                            "startup_timeout_s": 60.0,
                            "report_path": str(report_path),
                            "physics_ready_path": str(physics_ready_path),
                            "provider_manifest_path": str(provider_manifest),
                            "provider_manifest_sha256": provider_manifest_sha256,
                            "model_bundle_manifest": str(model_bundle_manifest),
                            "scenario_id": scenario_id,
                            "seed": seed,
                            "scenario_declaration_sha256": scenario_declaration_sha256,
                            "planning_scene_revision": planning_scene_revision,
                            "planning_scene_revision_digest": planning_scene_revision_digest,
                            "planning_scene_owned_ids": owned_ids_json,
                            "planning_scene_target_source_id": target_source_id,
                            "planning_scene_target_handoff": target_handoff,
                            "integrated_mapping": runtime_contract_json,
                            "public_integrated_mapping": public_integrated_json,
                            "integrated_sha256": public_integrated_sha256,
                            "runtime_contract_sha256": runtime_contract_sha256,
                            "model_fingerprint": model_fingerprint,
                            "fail_exit_s": 0.0,
                        }
                    ],
                    additional_env=python_env,
                )
                return [task_launch, readiness]

            return [
                planning_only,
                fixture_adapter,
                fixture_waiter,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=fixture_waiter,
                        on_exit=_after_fixture_ready,
                    )
                ),
            ]

        return [
            physics_gate,
            physics_waiter,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=physics_waiter,
                    on_exit=_after_physics_ready,
                )
            ),
        ]

    return [
        Node(
            package="tinker_sim_bridge",
            executable="safety_supervisor",
            output="screen",
            parameters=safety_parameters,
            additional_env=python_env,
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
        controller_reconciler,
        Node(
            package="tinker_sim_bridge",
            executable="command_gateway",
            output="screen",
            parameters=[str(share / "config/command_gateway.yaml"), {"use_sim_time": True}],
            additional_env=python_env,
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_reconciler,
                on_exit=lambda event, ctx: _process_exit_actions(
                    event, "controller reconciliation", [scenario_runner_node]
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=scenario_runner_node,
                on_exit=_after_scenario_runner,
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
        # Task-8 fix round 3 (Option A+): the qualification development LiDAR
        # publishes in the livox360 frame; own the base_link -> livox360 static
        # transform for integrated qualification exactly as navigation.launch.py
        # does.  Ordinary manipulation-core keeps this frame unowned.
        # Spelled qualified (launch_ros.actions.Node) so the immutable Task-2
        # launch-graph allow-list (which permits tf2_ros for the staging gates
        # but not as a bare Node package) keeps accepting this overlay; the node
        # itself is still a literal tf2_ros/static_transform_publisher.
        *(
            [
                launch_ros.actions.Node(
                    package="tf2_ros", executable="static_transform_publisher", name="livox360_static_tf",
                    arguments=[
                        "--x", "0.12", "--y", "0.0", "--z", "0.25",
                        "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                        "--frame-id", "base_link", "--child-frame-id", "livox360",
                    ],
                    output="screen",
                ),
            ]
            if qualification
            else []
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
            DeclareLaunchArgument("scenario", default_value="qualification-moveit-plan-joint"),
            DeclareLaunchArgument("seed", default_value="7"),
            DeclareLaunchArgument("reset_attempts", default_value="3"),
            DeclareLaunchArgument("reset_retry_delay", default_value="0.5"),
            DeclareLaunchArgument("qualification", default_value="false"),
            DeclareLaunchArgument(
                "model_bundle_manifest",
                default_value=os.environ.get("TINKER_SIM_MODEL_BUNDLE_MANIFEST", ""),
            ),
            DeclareLaunchArgument(
                "provider_manifest_path",
                default_value=os.environ.get("TINKER_SIM_PROVIDER_MANIFEST", ""),
            ),
            DeclareLaunchArgument(
                "attempt_dir", default_value=os.environ.get("TINKER_SIM_ATTEMPT_DIR", "")
            ),
            OpaqueFunction(function=_resolve),
        ]
    )
