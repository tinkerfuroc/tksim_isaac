"""Integrated OMPL qualification Gate-C plan-only executor (Task 4).

This module is ROS-lazy: importing it under the simulator CPython 3.12 venv
never imports ``rclpy`` or any generated ROS message type.  All generated-message
imports happen inside :func:`_load_ros` or the goal-builder call paths, which the
Humble suite exercises under sourced ROS Humble Python 3.10.

Pure helpers (importable everywhere):

- endpoint/type/cardinality/QoS contract constants;
- ``expected_physics_ready_report`` / ``validate_physics_ready_snapshot``
  reconciled with the real canonical multi-operation public report (one-key
  ``integrated`` mapping, scenario-declaration-bound fixture descriptor digest);
- ``evaluate_executor_readiness`` with the config-authoritative operator
  freshness threshold and the genuine positive-ready baseline;
- ``stage_c_dispatch`` validating the three Stage-C plan-only scenarios and
  returning a ROS-free dispatch spec;
- ``build_journal_graph_projection`` requiring an explicit observed-graph input
  (never fabricated publisher/server identities) and normalizing it for the
  Task-3 ``planning_scene_journal.validate_graph_evidence`` schema.

The live :class:`IntegratedGateExecutor` (Humble-only) constructs a valid
isolated rclpy node, subscribes to the real ``moveit_msgs/msg/PlanningScene``
topics, owns a :class:`~planning_scene_journal.PlanningSceneJournal`, gates
every goal on live readiness, dispatches the three Stage-C scenarios with
plan-only semantics, writes the Task-4 artifact set, and finalizes the journal.
It never calls ``/execute_trajectory`` in Gate C and never publishes
``/isaac_joint_commands``.  Task 7 later correlates physical truth; Task 4
records diagnostic scene consistency only and never supplies physical
contact/force/object-pose/verdict fields.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import re
import sys
import tempfile
import time
import uuid as _uuid
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.fixture_contract import (  # noqa: E402
    geometry_signature_sha256,
    readback_geometry,
    spec_geometry,
)
from tinker_sim_bridge.fixture_planning_scene import (  # noqa: E402
    fixture_descriptor_sha256,
    fixture_owned_ids,
    fixture_to_specs,
    load_mesh_asset,
)
from tinker_sim_bridge.integrated_readiness import (  # noqa: E402
    build_canonical_report,
    public_integrated_mapping,
    sha256_json,
)

REPORT_REVISION = "integrated-manipulation-v1"
FINAL_SIMULATION_STATE = "STATE_PLAYING"
PHYSICS_READY_BOUNDARY = "PHYSICS_READY"
SIMULATION_STATE_PLAYING = 1
INTEGRATED_EXECUTION_PROFILE = "sim_ompl"
RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"

# rclpy node names are unqualified base names; the qualification identity is
# the fully qualified name ``/tinker_integrated_gate_executor`` (namespace
# ``/`` + basename).  ``use_global_arguments=False`` keeps launch/global remaps
# from changing the qualification identity.
NODE_BASENAME = "tinker_integrated_gate_executor"
OPERATOR_NODE = "/tinker_integrated_gate_executor"
OPERATOR_NODE_NAMESPACE = "/"
FIXTURE_PUBLISHER_NODE = "/fixture_planning_scene"
SAFETY_SUPERVISOR_NODE = "/tinker_sim_safety_supervisor"
CONTROLLER_MANAGER_NODE = "/controller_manager"
GRIPPER_FACADE_NODE = "/tinker_sim_gripper_facade"
PICK_AND_PLACE_NODE = "/pick_and_place"
PHYSICS_READY_GATE_NODE = "/tinker_sim_physics_ready_gate"
MOVE_GROUP_NODE = "/move_group"
PLANNING_SCENE_TOPIC = "/planning_scene"
MONITORED_PLANNING_SCENE_TOPIC = "/monitored_planning_scene"
FIXTURE_TOPIC = "/sim/status/planning_scene_fixture"
JOINT_STATES_TOPIC = "/joint_states"
OPERATOR_TOPIC = "/sim/safety/operator"
SAFETY_STOP_TOPIC = "/sim/hardware/safety_stop"
ISAAC_COMMAND_TOPIC = "/isaac_joint_commands"

#: Stage C is exactly these three plan-only scenarios.
STAGE_C_SCENARIOS: tuple[str, ...] = (
    "qualification-moveit-plan-joint",
    "qualification-moveit-plan-pose",
    "qualification-moveit-plan-blocked",
)

#: Canonical seven-joint outbound target for the Stage-C joint scenario.
Q_OUTBOUND: tuple[float, ...] = (0.20, -0.20, 0.15, 0.30, -0.15, 0.20, 0.15)

#: Task-4 Gate-C explicit journal contract (scenario JSON does not yet carry
#: journal fields).  This is a Stage-C-only derivation; later D/E tasks extend
#: their own explicit contracts.
GATE_C_REQUIRED_EVENT_ORDER: tuple[str, ...] = ("fixture-ready", "teardown")
GATE_C_FORBIDDEN_EVENTS: tuple[str, ...] = (
    "before-pick",
    "scene-attach",
    "lift-complete",
    "transport",
    "before-release",
    "scene-detach",
    "released-settled",
    "task-cleanup",
)
#: The eight Task-4 forbidden manipulation events apply unchanged to every D
#: journal; no attach/detach/release event is ever emitted in Gate D.
D_FORBIDDEN_EVENTS: tuple[str, ...] = GATE_C_FORBIDDEN_EVENTS

#: Stage D is exactly these six scenarios.
STAGE_D_SCENARIOS: tuple[str, ...] = (
    "qualification-moveit-execute-joint",
    "qualification-moveit-execute-pose",
    "qualification-moveit-cartesian-retreat",
    "qualification-moveit-gripper",
    "qualification-moveit-cancel",
    "qualification-moveit-safety",
)

#: D handler kind per exact scenario id.
STAGE_D_KIND: Mapping[str, str] = {
    "qualification-moveit-execute-joint": "execute-joint",
    "qualification-moveit-execute-pose": "execute-pose",
    "qualification-moveit-cartesian-retreat": "retreat",
    "qualification-moveit-gripper": "gripper",
    "qualification-moveit-cancel": "cancel",
    "qualification-moveit-safety": "safety",
}

#: Exact declared polarity per D scenario.
STAGE_D_EXPECTED_POLARITY: Mapping[str, str] = {
    "qualification-moveit-execute-joint": "positive",
    "qualification-moveit-execute-pose": "positive",
    "qualification-moveit-cartesian-retreat": "positive",
    "qualification-moveit-gripper": "positive",
    "qualification-moveit-cancel": "cancel",
    "qualification-moveit-safety": "safety",
}

#: Exact declared ``expected_physical`` list per D scenario.
STAGE_D_EXPECTED_PHYSICAL: Mapping[str, tuple[str, ...]] = {
    "qualification-moveit-execute-joint": ("joint_execution_tracks", "terminal_success"),
    "qualification-moveit-execute-pose": ("pose_execution_reaches_tcp", "terminal_success"),
    "qualification-moveit-cartesian-retreat": ("cartesian_retreat_collision_aware", "terminal_success"),
    "qualification-moveit-gripper": ("gripper_travel_predicates", "terminal_success"),
    "qualification-moveit-cancel": ("execute_goal_canceled", "quiescent_after_cancel", "no_later_stage"),
    "qualification-moveit-safety": ("safety_effective_stop", "target_frozen", "no_auto_resume"),
}

#: Scenario-specific D journal diagnostic event order (Task 3 graph projection
#: stays unchanged; these labels are diagnostics, not physical truth).
STAGE_D_REQUIRED_EVENT_ORDER: Mapping[str, tuple[str, ...]] = {
    "execute-joint": ("fixture-ready", "execution-start", "execution-terminal", "teardown"),
    "execute-pose": ("fixture-ready", "execution-start", "execution-terminal", "teardown"),
    "retreat": ("fixture-ready", "retreat-start", "retreat-terminal", "teardown"),
    "gripper": ("fixture-ready", "gripper-open-terminal", "gripper-close-terminal", "teardown"),
    "cancel": ("fixture-ready", "execution-start", "cancel-requested", "quiescent", "teardown"),
    "safety": ("fixture-ready", "execution-start", "effective-stop", "operator-clear", "quiescent", "teardown"),
}

#: F2.2: journal diagnostic event order for ``run_gripper_sequence(open_first=False)``
#: (close→open).  The public ``open_first`` contract advertises both orders, so a
#: close-first attempt must use this order rather than the open-first default.
GRIPPER_CLOSE_FIRST_EVENT_ORDER: tuple[str, ...] = (
    "fixture-ready",
    "gripper-close-terminal",
    "gripper-open-terminal",
    "teardown",
)

#: ExecuteTrajectory / FJT split-path endpoints.
EXECUTE_TRAJECTORY_ENDPOINT = "/execute_trajectory"
FJT_ENDPOINT = "/xarm7_traj_controller/follow_joint_trajectory"
FJT_STATUS_TOPIC = "/xarm7_traj_controller/follow_joint_trajectory/_action/status"
#: F5.3: the real ExecuteTrajectory action-status topic.  The driver requires
#: observable terminal evidence for the exact preassigned execute goal UUID
#: before acceptance-timeout cleanup completes.
EXECUTE_STATUS_TOPIC = "/execute_trajectory/_action/status"
GRIPPER_ENDPOINT = "/xarm_gripper/gripper_action"
CARTESIAN_MOVE_ENDPOINT = "/cartesian_move_action"

#: ``moveit_msgs/action/ExecuteTrajectory`` terminal action status ints
#: (``action_msgs/msg/GoalStatus``).  Unknown/malformed statuses never pass.
EXECUTE_STATUS_EXECUTING = 2
EXECUTE_STATUS_SUCCEEDED = 4
EXECUTE_STATUS_CANCELED = 5
EXECUTE_STATUS_ABORTED = 6
_EXECUTE_STATUS_NAMES: Mapping[int, str] = {
    EXECUTE_STATUS_SUCCEEDED: "succeeded",
    EXECUTE_STATUS_CANCELED: "canceled",
    EXECUTE_STATUS_ABORTED: "aborted",
}

#: Native gripper contract mirrors current production behavior; Task 5
#: qualification constants (config/scenario files are unchanged).
GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSE_POSITION = 0.85
GRIPPER_MAX_EFFORT = 10.0

#: Cartesian retreat geometry: top-down grasp, exactly ``RETREAT_DISTANCE_M``
#: along the declared ``base_link`` axis, preserving orientation
#: (``object_center_z=0.64``, ``grasp_tcp_z=0.72`` contract).
RETREAT_AXIS = "+z"
RETREAT_DISTANCE_M = 0.10

#: Bound on the cached FJT status entries retained by the executor.
FJT_STATUS_CACHE_LIMIT = 64

#: Bound on the cached ExecuteTrajectory action-status entries (F5.3).  Mirrors
#: the FJT bound so a live action server cannot grow memory without bound.
EXECUTE_STATUS_CACHE_LIMIT = 64

#: F5.4: bounded grace for a valid-but-non-advancing join key.  A file-tail
#: truth read may observe the same (frame_index, timestamp) twice when two
#: journal snapshots land inside one truth frame; wait this long for the next
#: advancing frame before failing closed with ``no-join-key``.  A genuinely
#: stalled or missing truth stream still fails closed after the window.
JOIN_KEY_RETRY_S = 0.1

TASK_NAMESPACE = "pick_and_place/"
TARGET_OBJECT_ID = "pick_and_place/object_mesh"

#: Task 6 fixed-target geometry (mirrors ``simulation/qualification/
#: integrated-ompl.json`` ``geometry_contract``; asserted by the qualification
#: config tests).  The executor never reads the config file itself — the
#: geometry is pinned here so the fixed-target Pick/Place controls are
#: deterministic and ROS-free.
E_GRASP_TCP_XYZ: tuple[float, float, float] = (0.65, 0.0, 0.72)
E_OBJECT_ROOT_XYZ: tuple[float, float, float] = (0.65, 0.0, 0.60)
E_PLACE_TARGET_POINT: Mapping[str, object] = {
    "frame_id": "base_link",
    "xyz": [0.85, 0.0, 0.72],
}
E_PLACE_ORIENTATION_XYZW: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

#: Receipt-window FJT correlation window (executor constant only; the config
#: files are out of scope and are never edited for Task 6).
E_FJT_CORRELATION_TIMEOUT_S = 2.0
#: Fresh-TCP trigger speed and lift tolerance used by the E predicates
#: (``task-6-brief.md`` online-trigger formulas).
E_TRIGGER_TCP_SPEED_M_S = 0.01
E_LIFT_Z_TOLERANCE_M = 0.01
E_NORMAL_STATE_SAMPLES = 2
#: F2.1: the E scenarios that observe the production ``post_grasp_lift_m``
#: runtime parameter (those whose transport trigger requires the lift-complete
#: latch to be physically reachable).  Approach/blocked/malformed controls do
#: not transport and never require the seam.
_E_TRANSPORT_KINDS: frozenset[str] = frozenset(
    {"positive", "occupied-place", "cancel-transport", "safety-transport"}
)

#: ``tinker_arm_msgs/action/Pick`` / ``Place`` ``Result.status`` codes
#: (distinct from the action-client ``action_msgs/msg/GoalStatus`` enum).
PICK_PLACE_RESULT_SUCCESS = 0
PICK_PLACE_RESULT_INVALID_GOAL = 1
PICK_PLACE_RESULT_PLANNING_FAILED = 2
PICK_PLACE_RESULT_EXECUTION_FAILED = 3
PICK_PLACE_RESULT_CANCELED = 4
PICK_PLACE_RESULT_SAFETY_STOP = 5
PICK_PLACE_RESULT_SCENE_INCONSISTENT = 6
PICK_PLACE_RESULT_POSTCONDITION_FAILED = 7
PICK_PLACE_RESULT_TIMEOUT = 8
PICK_PLACE_RESULT_INTERNAL_ERROR = 9
PICK_PLACE_RESULT_NAMES: Mapping[int, str] = {
    PICK_PLACE_RESULT_SUCCESS: "success",
    PICK_PLACE_RESULT_INVALID_GOAL: "invalid_goal",
    PICK_PLACE_RESULT_PLANNING_FAILED: "planning_failed",
    PICK_PLACE_RESULT_EXECUTION_FAILED: "execution_failed",
    PICK_PLACE_RESULT_CANCELED: "canceled",
    PICK_PLACE_RESULT_SAFETY_STOP: "safety_stop",
    PICK_PLACE_RESULT_SCENE_INCONSISTENT: "scene_inconsistent",
    PICK_PLACE_RESULT_POSTCONDITION_FAILED: "postcondition_failed",
    PICK_PLACE_RESULT_TIMEOUT: "timeout",
    PICK_PLACE_RESULT_INTERNAL_ERROR: "internal_error",
}

#: Stage E is exactly these eight fixed-target Pick/Place scenarios.
STAGE_E_SCENARIOS: tuple[str, ...] = (
    "qualification-pick-place-positive",
    "qualification-pick-place-blocked-approach",
    "qualification-pick-place-unreachable-grasp",
    "qualification-pick-place-malformed-back",
    "qualification-pick-place-cancel-approach",
    "qualification-pick-place-cancel-transport",
    "qualification-pick-place-safety-transport",
    "qualification-pick-place-occupied-place",
)

#: E handler kind per exact scenario id.
STAGE_E_KIND: Mapping[str, str] = {
    "qualification-pick-place-positive": "positive",
    "qualification-pick-place-blocked-approach": "blocked-approach",
    "qualification-pick-place-unreachable-grasp": "unreachable-grasp",
    "qualification-pick-place-malformed-back": "malformed-back",
    "qualification-pick-place-cancel-approach": "cancel-approach",
    "qualification-pick-place-cancel-transport": "cancel-transport",
    "qualification-pick-place-safety-transport": "safety-transport",
    "qualification-pick-place-occupied-place": "occupied-place",
}

#: Exact declared polarity per E scenario.
STAGE_E_EXPECTED_POLARITY: Mapping[str, str] = {
    "qualification-pick-place-positive": "positive",
    "qualification-pick-place-blocked-approach": "negative",
    "qualification-pick-place-unreachable-grasp": "negative",
    "qualification-pick-place-malformed-back": "negative",
    "qualification-pick-place-cancel-approach": "negative",
    "qualification-pick-place-cancel-transport": "negative",
    "qualification-pick-place-safety-transport": "negative",
    "qualification-pick-place-occupied-place": "negative",
}

#: Exact declared ``expected_physical`` list per E scenario.
STAGE_E_EXPECTED_PHYSICAL: Mapping[str, tuple[str, ...]] = {
    "qualification-pick-place-positive": (
        "bilateral_contact",
        "lift",
        "transport",
        "bounded_tcp_object_drift",
        "release_in_place_region",
        "settled_speed",
        "no_arm_obstacle_contact",
    ),
    "qualification-pick-place-blocked-approach": (),
    "qualification-pick-place-unreachable-grasp": (),
    "qualification-pick-place-malformed-back": (),
    "qualification-pick-place-cancel-approach": (),
    "qualification-pick-place-cancel-transport": (),
    "qualification-pick-place-safety-transport": (),
    "qualification-pick-place-occupied-place": (),
}

#: Exact declared ``expected_negative`` contract (required/forbidden) per
#: negative E scenario; the positive scenario carries ``None``.
STAGE_E_EXPECTED_NEGATIVE: Mapping[str, Mapping[str, tuple[str, ...]]] = {
    "qualification-pick-place-blocked-approach": {
        "required": (
            "pick_terminal_non_success",
            "contact_absent",
            "scene_attach_absent",
            "lift_m_lt:0.02",
        ),
        "forbidden": ("gripper_close", "scene_attach", "release", "place_goal_sent"),
    },
    "qualification-pick-place-unreachable-grasp": {
        "required": (
            "pick_terminal_non_success",
            "contact_absent",
            "scene_attach_absent",
            "approach_tcp_delta_lt:0.02",
        ),
        "forbidden": ("gripper_close", "scene_attach", "lift", "release"),
    },
    "qualification-pick-place-malformed-back": {
        "required": ("goal_rejected_pre_send", "no_planning_scene_mutation"),
        "forbidden": ("pick_goal_sent", "move_group_goal_sent", "scene_attach", "contact"),
    },
    "qualification-pick-place-cancel-approach": {
        "required": (
            "cancel_trigger_after_approach_start",
            "contact_absent",
            "scene_attach_absent",
            "release_absent",
        ),
        "forbidden": ("gripper_close", "scene_attach", "lift_complete", "place_goal_sent"),
    },
    "qualification-pick-place-cancel-transport": {
        "required": (
            "cancel_trigger_after_lift",
            "contact_present_before_cancel",
            "scene_attached_before_cancel",
            "release_absent",
            "no_post_cancel_stage",
        ),
        "forbidden": ("gripper_open", "scene_detach", "place_goal_sent", "post_clear_resume"),
    },
    "qualification-pick-place-safety-transport": {
        "required": (
            "safety_observed_during_transport",
            "controller_terminal_non_success",
            "velocity_below_stop_limit",
            "release_absent",
            "no_post_clear_resume",
        ),
        "forbidden": ("gripper_open", "scene_detach", "new_goal_after_clear"),
    },
    "qualification-pick-place-occupied-place": {
        "required": (
            "pick_physical_retained",
            "place_terminal_non_success",
            "release_absent",
            "scene_attached_after_place_failure",
        ),
        "forbidden": ("scene_detach", "target_region_settled", "gripper_open"),
    },
}

#: Scenario-specific E journal diagnostic event order (Task 3 graph projection
#: stays unchanged; these labels are diagnostics, not physical truth).  The
#: positive order exactly equals ``planning_scene_journal.POSITIVE_ORDER``.
STAGE_E_REQUIRED_EVENT_ORDER: Mapping[str, tuple[str, ...]] = {
    "positive": (
        "fixture-ready",
        "before-pick",
        "scene-attach",
        "lift-complete",
        "transport",
        "before-release",
        "scene-detach",
        "released-settled",
        "teardown",
    ),
    "blocked-approach": ("fixture-ready", "before-pick", "pick-terminal", "teardown"),
    "unreachable-grasp": ("fixture-ready", "before-pick", "pick-terminal", "teardown"),
    "malformed-back": ("fixture-ready", "teardown"),
    "cancel-approach": (
        "fixture-ready",
        "before-pick",
        "approach-start",
        "cancel-requested",
        "quiescent",
        "teardown",
    ),
    "cancel-transport": (
        "fixture-ready",
        "before-pick",
        "scene-attach",
        "lift-complete",
        "transport",
        "cancel-requested",
        "quiescent",
        "teardown",
    ),
    "safety-transport": (
        "fixture-ready",
        "before-pick",
        "scene-attach",
        "lift-complete",
        "transport",
        "effective-stop",
        "operator-clear",
        "quiescent",
        "teardown",
    ),
    "occupied-place": (
        "fixture-ready",
        "before-pick",
        "scene-attach",
        "lift-complete",
        "transport",
        "place-goal-accepted",
        "cancel-requested",
        "quiescent",
        "teardown",
    ),
}

#: Scenario-specific E forbidden journal events.
STAGE_E_FORBIDDEN_EVENTS: Mapping[str, tuple[str, ...]] = {
    "positive": (),
    "blocked-approach": (
        "scene-attach",
        "lift-complete",
        "transport",
        "before-release",
        "scene-detach",
        "released-settled",
        "task-cleanup",
    ),
    "unreachable-grasp": (
        "scene-attach",
        "lift-complete",
        "transport",
        "before-release",
        "scene-detach",
        "released-settled",
        "task-cleanup",
    ),
    "malformed-back": (
        "before-pick",
        "scene-attach",
        "lift-complete",
        "transport",
        "before-release",
        "scene-detach",
        "released-settled",
        "task-cleanup",
    ),
    "cancel-approach": (
        "scene-attach",
        "lift-complete",
        "transport",
        "before-release",
        "scene-detach",
        "released-settled",
        "task-cleanup",
    ),
    "cancel-transport": (
        "before-release",
        "scene-detach",
        "released-settled",
        "task-cleanup",
    ),
    "safety-transport": (
        "before-release",
        "scene-detach",
        "released-settled",
        "task-cleanup",
    ),
    "occupied-place": (
        "before-release",
        "scene-detach",
        "released-settled",
        "task-cleanup",
    ),
}

#: Exact declared trigger_timeout_s per E scenario (None for positive).
STAGE_E_TRIGGER_TIMEOUT_S: Mapping[str, float | None] = {
    "qualification-pick-place-positive": None,
    "qualification-pick-place-blocked-approach": 10.0,
    "qualification-pick-place-unreachable-grasp": 10.0,
    "qualification-pick-place-malformed-back": 5.0,
    "qualification-pick-place-cancel-approach": 10.0,
    "qualification-pick-place-cancel-transport": 15.0,
    "qualification-pick-place-safety-transport": 15.0,
    "qualification-pick-place-occupied-place": 15.0,
}

#: Exact declared six-value malformed back vector (``malformed-back`` scenario).
E_MALFORMED_BACK_POSITIONS: tuple[float, ...] = (0.2, -0.2, 0.15, 0.3, -0.15, 0.2)

#: Action endpoints and their exact generated types (one server each).
REQUIRED_ACTIONS: Mapping[str, str] = {
    "/move_action": "moveit_msgs/action/MoveGroup",
    "/execute_trajectory": "moveit_msgs/action/ExecuteTrajectory",
    "/xarm7_traj_controller/follow_joint_trajectory": "control_msgs/action/FollowJointTrajectory",
    "/xarm_gripper/gripper_action": "control_msgs/action/GripperCommand",
    "/pickup_action": "tinker_arm_msgs/action/Pick",
    "/place_action": "tinker_arm_msgs/action/Place",
    "/cartesian_move_action": "tinker_arm_msgs/action/CartesianMove",
    "/joint_move_action": "tinker_arm_msgs/action/JointMove",
    "/fold_action": "tinker_arm_msgs/action/Fold",
}

#: Service endpoints and their exact generated types (one server each).
REQUIRED_SERVICES: Mapping[str, str] = {
    "/controller_manager/list_controllers": "controller_manager_msgs/srv/ListControllers",
    "/controller_manager/load_controller": "controller_manager_msgs/srv/LoadController",
    "/controller_manager/configure_controller": "controller_manager_msgs/srv/ConfigureController",
    "/controller_manager/switch_controller": "controller_manager_msgs/srv/SwitchController",
    "/get_planning_scene": "moveit_msgs/srv/GetPlanningScene",
    "/apply_planning_scene": "moveit_msgs/srv/ApplyPlanningScene",
    "/check_state_validity": "moveit_msgs/srv/GetStateValidity",
    "/compute_cartesian_path": "moveit_msgs/srv/GetCartesianPath",
    "/arm_joint_service": "tinker_arm_msgs/srv/ArmJointService",
    "/sim/ready/physics": "std_srvs/srv/Trigger",
    "/sim/ready/fixture": "std_srvs/srv/Trigger",
}

#: Required topic graph contract (type/source/cardinality/QoS).  This is a graph
#: contract, not a fixture convenience: a valid payload can never mask graph
#: metadata and vice versa.
REQUIRED_TOPICS: Mapping[str, Mapping[str, object]] = {
    JOINT_STATES_TOPIC: {
        "type": "sensor_msgs/msg/JointState", "publisher_count": 1,
        "source_node": CONTROLLER_MANAGER_NODE,
        "qos": {"reliability": "reliable", "durability": "volatile", "depth": 10},
    },
    FIXTURE_TOPIC: {
        "type": "std_msgs/msg/String", "publisher_count": 1,
        "source_node": FIXTURE_PUBLISHER_NODE,
        "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
    },
    OPERATOR_TOPIC: {
        "type": "std_msgs/msg/Bool", "publisher_count": 1,
        "source_node": OPERATOR_NODE,
        "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
        "allowlist": [False, True],
    },
    SAFETY_STOP_TOPIC: {
        "type": "std_msgs/msg/Bool", "publisher_count": 1,
        "source_node": SAFETY_SUPERVISOR_NODE,
        "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
    },
}

_REQUIRED_ACTIONS = REQUIRED_ACTIONS
_REQUIRED_SERVICES = REQUIRED_SERVICES

#: Observed-graph provider for every required endpoint.  The
#: ``follow_joint_trajectory`` bridge identity is the logical
#: ``controller_resource:xarm7_traj_controller``; the observed graph node is
#: ``/controller_manager`` and is the value asserted here.
_REQUIRED_ENDPOINT_SOURCES: Mapping[str, str] = {
    "/move_action": MOVE_GROUP_NODE,
    "/execute_trajectory": MOVE_GROUP_NODE,
    "/xarm7_traj_controller/follow_joint_trajectory": CONTROLLER_MANAGER_NODE,
    "/xarm_gripper/gripper_action": GRIPPER_FACADE_NODE,
    "/pickup_action": PICK_AND_PLACE_NODE,
    "/place_action": PICK_AND_PLACE_NODE,
    "/cartesian_move_action": PICK_AND_PLACE_NODE,
    "/joint_move_action": PICK_AND_PLACE_NODE,
    "/fold_action": PICK_AND_PLACE_NODE,
    "/controller_manager/list_controllers": CONTROLLER_MANAGER_NODE,
    "/controller_manager/load_controller": CONTROLLER_MANAGER_NODE,
    "/controller_manager/configure_controller": CONTROLLER_MANAGER_NODE,
    "/controller_manager/switch_controller": CONTROLLER_MANAGER_NODE,
    "/get_planning_scene": MOVE_GROUP_NODE,
    "/apply_planning_scene": MOVE_GROUP_NODE,
    "/check_state_validity": MOVE_GROUP_NODE,
    "/compute_cartesian_path": MOVE_GROUP_NODE,
    "/arm_joint_service": PICK_AND_PLACE_NODE,
    "/sim/ready/physics": PHYSICS_READY_GATE_NODE,
    "/sim/ready/fixture": FIXTURE_PUBLISHER_NODE,
}

_REQUIRED_JOINTS: tuple[str, ...] = tuple(f"joint{index}" for index in range(1, 8)) + (
    "drive_joint",
)

DIGEST = re.compile(r"^(?!0{64}$)[0-9a-f]{64}$")
REPORT_KEYS = frozenset(
    {
        "schema_version", "report_revision", "scenario", "planning_scene",
        "integrated", "identities", "operations", "final_simulation_state",
    }
)
IDENTITY_KEYS = frozenset(
    {
        "scenario_id", "seed", "scenario_declaration_sha256",
        "planning_scene_sha256", "integrated_sha256", "model_fingerprint",
        "provider_manifest_sha256",
    }
)
#: The unique final ``PHYSICS_READY`` operation carries exactly this field set
#: (the report identities merged into the accepted set-simulation-state result).
OPERATION_KEYS = frozenset(
    {
        "operation", "accepted", "state", "boundary", "scenario_id", "seed",
        "scenario_declaration_sha256", "planning_scene_sha256", "integrated_sha256",
        "model_fingerprint", "provider_manifest_sha256",
    }
)
#: Optional fields an earlier accepted standard-operation record may carry
#: (``state`` / ``boundary`` on set-simulation-state, ``logical_id`` /
#: ``prim_path`` on spawn_entity).  ``operation`` and ``accepted`` are required.
EARLIER_OPERATION_OPTIONAL_FIELDS = frozenset(
    {"state", "boundary", "logical_id", "prim_path"}
)
#: Exact canonical fixture-status field set (matches bridge canonical status).
FIXTURE_STATUS_KEYS = frozenset(
    {
        "schema_version", "state", "scenario", "owner", "revision",
        "revision_digest", "sequence", "published_at", "owned_ids",
        "target_source_id", "target_handoff", "fixture_descriptor_sha256",
    }
)
FIXTURE_OWNER = "sim_fixture"
FIXTURE_TARGET_HANDOFF = "pick_and_place/object_mesh"

#: Journal graph projection QoS uses exact uppercase enum strings (Task 3
#: schema); readiness-snapshot QoS uses the existing lowercase representation.
#: The two PlanningScene topics mirror the stock MoveIt2 Humble publisher's
#: plain depth-100 ``rclcpp::QoS`` (RELIABLE + VOLATILE); the fixture status
#: topic stays RELIABLE/TRANSIENT_LOCAL/depth 1 (F2.3).
JOURNAL_PLANNING_SCENE_TOPIC_QOS: Mapping[str, object] = {
    "reliability": "RELIABLE",
    "durability": "VOLATILE",
    "depth": 100,
}
JOURNAL_FIXTURE_TOPIC_QOS: Mapping[str, object] = {
    "reliability": "RELIABLE",
    "durability": "TRANSIENT_LOCAL",
    "depth": 1,
}
#: Backward-compatible alias retained for the fixture-status topic claim.
JOURNAL_TOPIC_QOS: Mapping[str, object] = dict(JOURNAL_FIXTURE_TOPIC_QOS)
JOURNAL_SERVICE_QOS: Mapping[str, object] = {
    "reliability": "RELIABLE",
    "durability": "VOLATILE",
}

#: MoveIt planning-stage non-success codes valid for the blocked diagnostic
#: (F2.4).  Only codes that unambiguously represent planner/IK non-success after
#: a valid plan-only request (PLANNING_FAILED=-1, INVALID_MOTION_PLAN=-2,
#: NO_IK_SOLUTION=5).  Request-level/configuration/timeout/transport errors are
#: never a blocked pass.
MOVEIT_SUCCESS_CODE = 1
MOVEIT_PLANNING_NON_SUCCESS_CODES: frozenset[int] = frozenset({-1, -2, 5})

#: Complete expected malformed-message exception set contained at the
#: PlanningScene callback boundary (F3.2): attribute/type/value and
#: serialization failures that arise from wrong-shaped input.  Process-control
#: exceptions (KeyboardInterrupt/SystemExit) and unrelated fatal errors are
#: never swallowed.
_SCENE_CALLBACK_EXCEPTIONS: tuple[type[Exception], ...] = (
    AttributeError,
    IndexError,
    KeyError,
    TypeError,
    ValueError,
)

#: Journal/evidence artifact names written per attempt.
ARTIFACT_JSONL_FILES: tuple[str, ...] = (
    "integrated-execution.jsonl",
    "moveit-plans.jsonl",
    "controller-results.jsonl",
    "visual-capture-requests.jsonl",
)


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def _finite_vector(values: Sequence[float], *, length: int, name: str) -> list[float]:
    converted = [float(value) for value in values]
    if len(converted) != length or not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{name} must contain exactly {length} finite values")
    return converted


def _validate_quaternion(quaternion) -> None:
    values = tuple(
        float(value)
        for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose quaternion must be finite")
    norm = math.sqrt(sum(value ** 2 for value in values))
    if abs(norm - 1.0) > 1.0e-3:
        raise ValueError("pose quaternion must be normalized within 1e-3")


def _as_mapping(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _fresh(value: object, limit: object) -> bool:
    try:
        age = float(value)
        return math.isfinite(age) and 0.0 <= age <= float(limit)
    except (TypeError, ValueError):
        return False


def _finite_sequence(value: object, *, length: int) -> bool:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != length:
        return False
    try:
        return all(math.isfinite(float(item)) for item in value)
    except (TypeError, ValueError):
        return False


def _finite_number(value: object) -> bool:
    return type(value) in (int, float) and math.isfinite(float(value))


def _ordered_string_ids(value: object) -> bool:
    return (
        isinstance(value, Sequence)
        and not isinstance(value, (str, bytes))
        and all(isinstance(item, str) for item in value)
        and len(value) == len(set(value))
    )


def _valid_digest(value: object) -> bool:
    return isinstance(value, str) and DIGEST.fullmatch(value) is not None


def _strict_int(value: object) -> bool:
    return type(value) is int


def _endpoint_failures(
    observed: object, required: Mapping[str, str], *, kind: str
) -> list[str]:
    endpoints = _as_mapping(observed)
    failures: list[str] = []
    for name, expected_type in required.items():
        endpoint = _as_mapping(endpoints.get(name))
        expected_source = _REQUIRED_ENDPOINT_SOURCES.get(name)
        if not (
            endpoint.get("type") == expected_type
            and endpoint.get("ready") is True
            and endpoint.get("server_count") == 1
            and endpoint.get("source_node") == expected_source
        ):
            failures.append(
                f"{kind} {name} is not exactly-one ready {expected_type} "
                f"owned by {expected_source}"
            )
    return failures


def _topic_failures(observed: object) -> list[str]:
    topics = _as_mapping(observed)
    failures: list[str] = []
    for name, expected in REQUIRED_TOPICS.items():
        topic = _as_mapping(topics.get(name))
        if (
            topic.get("type") != expected["type"]
            or topic.get("publisher_count") != expected["publisher_count"]
            or topic.get("source_node") != expected["source_node"]
            or topic.get("qos") != expected["qos"]
        ):
            failures.append(f"topic {name} has wrong type, cardinality, source, or QoS")
        if name == OPERATOR_TOPIC and topic.get("allowlist") != [False, True]:
            failures.append(f"topic {name} has an invalid Boolean allowlist")
    return failures


def _scenario_fixture_digest(scenario: Mapping[str, object]) -> str | None:
    """Return the real fixture descriptor digest over the scenario declaration."""
    declaration = _as_mapping(scenario.get("planning_scene_declaration"))
    if not declaration:
        return None
    return fixture_descriptor_sha256(declaration)


def _scenario_fixture_ids(scenario: Mapping[str, object]) -> list[str]:
    """Return declared-order owned fixture ids from the full declaration."""
    declaration = _as_mapping(scenario.get("planning_scene_declaration"))
    if declaration:
        return list(fixture_owned_ids(declaration))
    return list(_as_mapping(scenario.get("planning_scene")).get("owned_ids", ()))


def _resolve_declared_mesh(mesh: Mapping[str, object]):
    """Resolve a declared mesh asset to (vertices, triangles) through the bridge."""
    return load_mesh_asset(mesh, project_root=ROOT)


def expected_fixture_geometry_digest(
    declaration: Mapping[str, object], *, resolve_mesh=None
) -> str:
    """Return the deterministic fixture-owned geometry projection digest (F3.3).

    The projection is the declared-order owned collision-body geometry in the
    same canonical descriptor form the bridge's ``readback_geometry`` produces
    for a received MoveIt ``CollisionObject``: frame, primitive type +
    dimensions + poses (and, for mesh fixtures, resolved vertices + triangles).
    The digest is ``geometry_signature_sha256`` over exactly those ordered
    descriptors, so it binds the exact declared IDs, order, frame, geometry, and
    pose — never unrelated full-scene serialization (robot state / ACM).

    Mesh fixtures require *resolve_mesh* (a callable mapping a
    ``{"uri", "sha256", "scale"}`` declaration to ``(vertices, triangles)``);
    the executor supplies the bridge asset resolver by default.
    """
    if resolve_mesh is None:
        resolve_mesh = _resolve_declared_mesh
    specs = fixture_to_specs(declaration)
    descriptors = [spec_geometry(spec, resolve_mesh=resolve_mesh) for spec in specs]
    return geometry_signature_sha256(descriptors)


def expected_physics_ready_report(
    *,
    scenario_mapping: Mapping[str, object],
    planning_scene: Mapping[str, object],
    integrated: Mapping[str, object],
    expected_identities: Mapping[str, object],
) -> dict[str, object]:
    """Build the expected real-shape multi-operation public report.

    ``planning_scene`` must be the full planning-scene declaration (so the
    bridge can derive the four-key public mapping and its digest).  The report
    carries the one-key public ``integrated`` mapping; the full ``integrated``
    mapping passed in is asserted to carry ``execution_profile == "sim_ompl"``
    and remains bound by the scenario declaration SHA-256.
    """
    identities = dict(expected_identities)
    if set(identities) != IDENTITY_KEYS:
        raise ValueError("expected PHYSICS_READY identities must be complete")
    if identities["scenario_id"] != str(scenario_mapping.get("id")):
        raise ValueError("expected identity scenario_id does not match scenario mapping")
    if int(identities["seed"]) != int(scenario_mapping.get("seed")):
        raise ValueError("expected identity seed does not match scenario mapping")
    if dict(integrated).get("execution_profile") != INTEGRATED_EXECUTION_PROFILE:
        raise ValueError("scenario integrated execution_profile must be sim_ompl")
    public_integrated = public_integrated_mapping()
    report = build_canonical_report(
        scenario_id=identities["scenario_id"],
        seed=identities["seed"],
        declaration=dict(_as_mapping(scenario_mapping.get("declaration"))),
        planning_scene=planning_scene,
        integrated=public_integrated,
        operations=[
            {"operation": "reset_spawned", "accepted": True},
            {
                "operation": "set_simulation_state",
                "accepted": True,
                "state": SIMULATION_STATE_PLAYING,
                "boundary": PHYSICS_READY_BOUNDARY,
            },
        ],
        model_fingerprint=identities["model_fingerprint"],
        provider_manifest_sha256=identities["provider_manifest_sha256"],
    )
    report = copy.deepcopy(dict(report))
    if report["identities"] != identities:
        raise ValueError("expected PHYSICS_READY report identities do not match")
    if report["integrated"] != public_integrated:
        raise ValueError(
            "expected PHYSICS_READY integrated mapping is not the public one-key mapping"
        )
    return report


def _validate_expected_report_structure(expected_report: Mapping[str, object]) -> None:
    """Reject boolean/non-integer numerics in the expected report positions.

    Boolean values in ``schema_version``, ``identities.seed``, or any
    operation ``state`` are structurally invalid; they fail closed before the
    observed-byte comparison so a caller passing a mutated expected report gets
    the specific strict-integer failure.
    """
    if not isinstance(expected_report, Mapping):
        raise ValueError("expected report must be a mapping")
    if not _strict_int(expected_report.get("schema_version")):
        raise ValueError("PHYSICS_READY schema_version must be a strict integer")
    identities = _as_mapping(expected_report.get("identities"))
    if not _strict_int(identities.get("seed")):
        raise ValueError("PHYSICS_READY identity seed must be a strict integer")
    operations = expected_report.get("operations")
    if not isinstance(operations, Sequence) or isinstance(operations, (str, bytes)):
        raise ValueError("PHYSICS_READY report has no operations")
    for operation in operations:
        if not isinstance(operation, Mapping):
            raise ValueError("PHYSICS_READY operations must be objects")
        if "state" in operation and not _strict_int(operation.get("state")):
            raise ValueError("PHYSICS_READY operation state must be a strict integer")


def validate_physics_ready_snapshot(
    snapshot: Mapping[str, object],
    scenario: Mapping[str, object],
    *,
    expected_report: Mapping[str, object] | None = None,
) -> None:
    """Validate the exact real-shape multi-operation physics-ready report.

    Keeps exact canonical byte/schema/revision checks and the exact top-level
    ``REPORT_KEYS``; compares ``scenario``/``planning_scene``/``integrated``/
    ``identities`` against the corrected expected contract; requires a non-empty
    operation list with exactly one final ``PHYSICS_READY`` operation carrying
    the exact ``OPERATION_KEYS`` and report identities; every earlier operation
    is an accepted standard-operation record with only the known optional fields.
    """
    if expected_report is not None:
        _validate_expected_report_structure(expected_report)
    state = _as_mapping(snapshot.get("scenario"))
    report_bytes = snapshot.get("scenario_report_bytes")
    if not isinstance(report_bytes, (bytes, bytearray)):
        raise ValueError("PHYSICS_READY exact report bytes are unavailable")
    exact_bytes = bytes(report_bytes)
    try:
        report = json.loads(exact_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("PHYSICS_READY report bytes are not valid UTF-8 JSON") from error
    if not isinstance(report, dict) or set(report) != REPORT_KEYS:
        raise ValueError("PHYSICS_READY report has the wrong canonical top-level schema")
    if not _strict_int(report.get("schema_version")):
        raise ValueError("PHYSICS_READY schema_version must be a strict integer")

    expected = (
        dict(expected_report)
        if expected_report is not None
        else expected_physics_ready_report(
            scenario_mapping=_as_mapping(scenario.get("scenario_mapping")),
            planning_scene=_as_mapping(scenario.get("planning_scene_declaration"))
            or _as_mapping(scenario.get("planning_scene")),
            integrated=_as_mapping(scenario.get("integrated")),
            expected_identities=_as_mapping(scenario.get("identities")),
        )
    )
    for key in ("scenario", "planning_scene", "integrated", "identities"):
        if report.get(key) != expected.get(key):
            raise ValueError(
                f"PHYSICS_READY report {key} does not match the "
                "scenario-specific expected contract"
            )
    canonical_bytes = json.dumps(
        report, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    if canonical_bytes != exact_bytes:
        raise ValueError("PHYSICS_READY report bytes are valid JSON but not the canonical serialization")
    if report["report_revision"] != REPORT_REVISION:
        raise ValueError("PHYSICS_READY report revision mismatch")

    identities = report["identities"]
    if not isinstance(identities, dict) or set(identities) != IDENTITY_KEYS:
        raise ValueError("PHYSICS_READY identity keys are not exact")
    if identities["scenario_id"] != scenario.get("id") or int(identities["seed"]) != int(
        scenario.get("seed")
    ):
        raise ValueError("PHYSICS_READY scenario identity mismatch")
    if not _strict_int(identities.get("seed")):
        raise ValueError("PHYSICS_READY identity seed must be a strict integer")
    if not _strict_int(_as_mapping(report.get("scenario")).get("seed")):
        raise ValueError("PHYSICS_READY scenario seed must be a strict integer")
    expected_identities = {
        "scenario_declaration_sha256": scenario.get("scenario_declaration_sha256"),
        "planning_scene_sha256": scenario.get("planning_scene_sha256"),
        "integrated_sha256": scenario.get("integrated_sha256"),
        "model_fingerprint": scenario.get("model_fingerprint"),
        "provider_manifest_sha256": scenario.get("provider_manifest_sha256"),
    }
    if any(identities[key] != value for key, value in expected_identities.items()):
        raise ValueError("PHYSICS_READY identity digests do not match the expected scenario")
    if identities["model_fingerprint"] != _as_mapping(snapshot.get("model")).get("fingerprint"):
        raise ValueError("PHYSICS_READY model fingerprint does not match the observed model")
    if identities["provider_manifest_sha256"] != snapshot.get("provider_manifest_sha256"):
        raise ValueError("PHYSICS_READY provider manifest digest does not match the observed manifest")
    for key in IDENTITY_KEYS - {"scenario_id", "seed"}:
        if not isinstance(identities[key], str) or DIGEST.fullmatch(identities[key]) is None:
            raise ValueError(f"PHYSICS_READY {key} is not a nonzero lowercase digest")
    for mapping_key, digest_key in (
        ("planning_scene", "planning_scene_sha256"),
        ("integrated", "integrated_sha256"),
    ):
        if identities[digest_key] != sha256_json(report[mapping_key]):
            raise ValueError(f"PHYSICS_READY {digest_key} is not the canonical mapping digest")
    if report["final_simulation_state"] != FINAL_SIMULATION_STATE:
        raise ValueError("PHYSICS_READY final simulation state is not STATE_PLAYING")

    operations = report["operations"]
    if not isinstance(operations, list) or not operations:
        raise ValueError("PHYSICS_READY report has no operations")
    # The real report is genuinely multi-operation: at least one accepted
    # standard-operation record (reset/spawn and related) precedes the unique
    # final PHYSICS_READY operation.  A fabricated single-operation report is
    # rejected, not compared against.
    if len(operations) < 2:
        raise ValueError(
            "PHYSICS_READY report must contain accepted standard-operation records "
            "before the final operation"
        )
    final = operations[-1]
    if not isinstance(final, dict) or set(final) != OPERATION_KEYS:
        raise ValueError("PHYSICS_READY final operation schema is not exact")
    physics_ready = [
        operation
        for operation in operations
        if isinstance(operation, dict) and operation.get("boundary") == PHYSICS_READY_BOUNDARY
    ]
    if len(physics_ready) != 1 or final is not physics_ready[0]:
        raise ValueError("PHYSICS_READY operation is not unique and final")
    for operation in operations[:-1]:
        if not isinstance(operation, dict):
            raise ValueError("PHYSICS_READY earlier operations must be objects")
        if not isinstance(operation.get("operation"), str) or not operation.get("operation"):
            raise ValueError("PHYSICS_READY earlier operations must carry an operation string")
        if operation.get("accepted") is not True:
            raise ValueError("PHYSICS_READY earlier operations must be accepted")
        if not set(operation) <= ({"operation", "accepted"} | EARLIER_OPERATION_OPTIONAL_FIELDS):
            raise ValueError("PHYSICS_READY earlier operations carry unknown fields")
    if (
        not _strict_int(final.get("state"))
        or final["state"] != SIMULATION_STATE_PLAYING
        or final["accepted"] is not True
        or not _strict_int(final.get("seed"))
    ):
        raise ValueError("PHYSICS_READY final operation is not accepted with integer state=1")
    for key in IDENTITY_KEYS:
        if final[key] != identities[key]:
            raise ValueError(f"PHYSICS_READY final operation {key} mismatch")

    expected_external_digest = state.get("scenario_report_sha256")
    if expected_external_digest != hashlib.sha256(exact_bytes).hexdigest():
        raise ValueError("PHYSICS_READY external report digest does not match exact report bytes")


def evaluate_executor_readiness(
    snapshot: Mapping[str, object],
    config: Mapping[str, object],
    scenario: Mapping[str, object],
) -> dict[str, object]:
    """Evaluate the genuine ready baseline; every negative test mutates exactly
    one contract and checks the specific failure reason."""
    thresholds = _as_mapping(config.get("thresholds"))
    integrated = _as_mapping(scenario.get("integrated"))
    declaration = _as_mapping(scenario.get("planning_scene_declaration"))
    expected_fixture = declaration or _as_mapping(scenario.get("planning_scene"))
    expected_ids = _scenario_fixture_ids(scenario)
    scenario_fixture_digest = _scenario_fixture_digest(scenario)
    reasons: list[str] = []

    if config.get("execution_profile") != INTEGRATED_EXECUTION_PROFILE:
        reasons.append("execution_profile must be sim_ompl")
    if integrated.get("execution_profile") != INTEGRATED_EXECUTION_PROFILE:
        reasons.append("scenario execution_profile must be sim_ompl")

    try:
        validate_physics_ready_snapshot(snapshot, scenario)
    except ValueError as error:
        reasons.append(str(error))
    if _as_mapping(snapshot.get("model")).get("fingerprint_match") is not True:
        reasons.append("robot-model fingerprint mismatch")

    tf = _as_mapping(snapshot.get("tf"))
    if tf.get("complete") is not True or not _fresh(tf.get("age_s"), thresholds.get("tf_fresh_s")):
        reasons.append("required TF chain is incomplete or stale")

    joint_state = _as_mapping(snapshot.get("joint_state"))
    names = joint_state.get("names")
    positions = joint_state.get("positions")
    velocities = joint_state.get("velocities")
    missing = [
        name
        for name in _REQUIRED_JOINTS
        if not isinstance(names, Sequence) or isinstance(names, (str, bytes)) or name not in names
    ]
    joint_ok = (
        list(names) == list(_REQUIRED_JOINTS)
        if isinstance(names, Sequence) and not isinstance(names, (str, bytes))
        else False
    )
    joint_ok = joint_ok and _finite_sequence(
        positions, length=len(_REQUIRED_JOINTS)
    ) and _finite_sequence(velocities, length=len(_REQUIRED_JOINTS))
    joint_ok = joint_ok and (
        type(joint_state.get("header_stamp_ns")) is int
        and joint_state.get("header_stamp_ns") > 0
        and _fresh(joint_state.get("age_s"), thresholds.get("joint_state_fresh_s"))
        and joint_state.get("publisher_count") == 1
        and joint_state.get("source_node") == CONTROLLER_MANAGER_NODE
        and joint_state.get("logical_controller") == "joint_state_broadcaster"
    )
    if not joint_ok:
        suffix = f"; missing {missing}" if missing else ""
        reasons.append(
            f"/joint_states joint state is incomplete, non-finite, stale, "
            f"unstamped, or wrongly owned{suffix}"
        )

    controllers = _as_mapping(snapshot.get("controllers"))
    controller_records = _as_mapping(controllers.get("logical_controllers"))
    if not (
        controllers.get("manager_healthy") is True
        and controllers.get("manager_source_node") == CONTROLLER_MANAGER_NODE
        and controllers.get("manager_publisher_count") == 1
        and set(controller_records) == {"joint_state_broadcaster", "xarm7_traj_controller"}
        and controller_records.get("joint_state_broadcaster") == {
            "state": "active", "source_node": CONTROLLER_MANAGER_NODE, "cardinality": 1
        }
        and controller_records.get("xarm7_traj_controller") == {
            "state": "active", "source_node": CONTROLLER_MANAGER_NODE, "cardinality": 1
        }
    ):
        reasons.append("controller manager or required logical controllers are unhealthy")

    topics = _as_mapping(snapshot.get("topics"))
    operator = _as_mapping(topics.get(OPERATOR_TOPIC))
    # F1.5: the configured operator freshness threshold is the authority.  If the
    # config has no dedicated ``operator_fresh_s``, the documented fallback is
    # ``thresholds.fixture_fresh_s`` (current value 0.25).  A snapshot-supplied
    # threshold is never trusted as authority.
    operator_fresh_limit = thresholds.get("operator_fresh_s", thresholds.get("fixture_fresh_s"))
    if not (
        operator.get("type") == "std_msgs/msg/Bool"
        and operator.get("publisher_count") == 1
        and operator.get("source_node") == OPERATOR_NODE
        and operator.get("qos") == REQUIRED_TOPICS[OPERATOR_TOPIC]["qos"]
        and operator.get("received") is True
        and operator.get("received_value") is False
        and type(operator.get("received_timestamp_ns")) is int
        and operator.get("received_timestamp_ns") > 0
        and _fresh(operator.get("received_age_s"), operator_fresh_limit)
    ):
        reasons.append(
            "operator safety sample is missing, asserted, stale, invalid, or graph-mismatched"
        )

    safety = _as_mapping(snapshot.get("safety"))
    raw_safety_topic = _as_mapping(topics.get(SAFETY_STOP_TOPIC))
    if not (
        safety.get("stop") is False
        and raw_safety_topic.get("data") is False
        and _fresh(safety.get("age_s"), thresholds.get("joint_state_fresh_s"))
        and type(safety.get("sample_count")) is int
        and safety.get("sample_count") >= 2
        and safety.get("type") == "std_msgs/msg/Bool"
        and safety.get("publisher_count") == 1
        and safety.get("source_node") == SAFETY_SUPERVISOR_NODE
    ):
        reasons.append(
            "/sim/hardware/safety_stop safety heartbeat is not fresh, explicit, "
            "typed, or singly owned"
        )

    reasons.extend(_endpoint_failures(snapshot.get("actions"), _REQUIRED_ACTIONS, kind="action"))
    reasons.extend(_endpoint_failures(snapshot.get("services"), _REQUIRED_SERVICES, kind="service"))
    reasons.extend(_topic_failures(snapshot.get("topics")))

    fixture = _as_mapping(snapshot.get("fixture"))
    fixture_topic = _as_mapping(topics.get(FIXTURE_TOPIC))
    try:
        parsed_fixture_payload = json.loads(str(fixture_topic.get("payload", "")))
        fixture_payload = parsed_fixture_payload if isinstance(parsed_fixture_payload, Mapping) else {}
    except (TypeError, ValueError):
        fixture_payload = {}
    payload_ids = fixture_payload.get("owned_ids")
    observed_ids = fixture.get("owned_ids")
    target_id = fixture.get("target_source_id")
    fixture_payload_ok = (
        set(fixture_payload) == FIXTURE_STATUS_KEYS
        and set(fixture) >= FIXTURE_STATUS_KEYS
        and fixture_payload.get("schema_version") == 1
        and fixture_payload.get("state") == "FIXTURE_READY"
        and fixture_payload.get("owner") == FIXTURE_OWNER
        and fixture_payload.get("scenario") == scenario.get("id")
        and fixture_payload.get("revision") == fixture.get("revision")
        and _valid_digest(fixture_payload.get("revision_digest"))
        and fixture_payload.get("revision_digest") == fixture.get("revision_digest")
        and _strict_int(fixture_payload.get("sequence"))
        and _strict_int(fixture.get("sequence"))
        and _strict_int(fixture.get("previous_sequence"))
        and fixture_payload.get("sequence") == fixture.get("sequence")
        and fixture.get("sequence") > fixture.get("previous_sequence") >= 1
        and _finite_number(fixture_payload.get("published_at"))
        and _finite_number(fixture.get("published_at"))
        and abs(float(fixture_payload.get("published_at")) - float(fixture.get("published_at"))) <= 1.0e-6
        and _ordered_string_ids(payload_ids)
        and _ordered_string_ids(observed_ids)
        and _ordered_string_ids(expected_ids)
        and payload_ids == observed_ids == list(expected_ids)
        and isinstance(target_id, str)
        and payload_ids.count(target_id) == 1
        and fixture_payload.get("target_source_id") == target_id
        and fixture_payload.get("target_handoff") == FIXTURE_TARGET_HANDOFF
        and _valid_digest(fixture_payload.get("fixture_descriptor_sha256"))
        and fixture_payload.get("fixture_descriptor_sha256") == fixture.get("fixture_descriptor_sha256")
        and (scenario_fixture_digest is not None
             and fixture.get("fixture_descriptor_sha256") == scenario_fixture_digest)
    )
    if set(fixture_payload) != FIXTURE_STATUS_KEYS:
        reasons.append("fixture payload has extra or missing keys")

    fixture_ok = (
        fixture.get("schema_version") == 1
        and fixture.get("state") == "FIXTURE_READY"
        and fixture.get("owner") == FIXTURE_OWNER
        and fixture.get("scenario") == scenario.get("id")
        and fixture.get("revision") == expected_fixture.get("revision")
        and _valid_digest(fixture.get("revision_digest"))
        and fixture.get("revision_digest") == expected_fixture.get("revision_digest")
        and fixture.get("target_source_id") == expected_fixture.get("target_source_id")
        and fixture.get("target_handoff") == FIXTURE_TARGET_HANDOFF
        and _valid_digest(fixture.get("fixture_descriptor_sha256"))
        and (scenario_fixture_digest is not None
             and fixture.get("fixture_descriptor_sha256") == scenario_fixture_digest)
        and fixture.get("fixture_descriptor_sha256") == fixture_payload.get("fixture_descriptor_sha256")
        and _ordered_string_ids(fixture.get("owned_ids"))
        and _ordered_string_ids(expected_ids)
        and not any(item.startswith("pick_and_place/") for item in fixture.get("owned_ids", ()))
        and list(fixture.get("owned_ids")) == list(expected_ids)
        and _strict_int(fixture.get("sequence"))
        and _strict_int(fixture.get("previous_sequence"))
        and fixture.get("sequence") > fixture.get("previous_sequence") >= 1
        and _strict_int(fixture.get("sample_count"))
        and fixture.get("sample_count") >= 2
        and _fresh(fixture.get("age_s"), thresholds.get("fixture_fresh_s"))
        and fixture_topic.get("type") == "std_msgs/msg/String"
        and fixture_topic.get("publisher_count") == 1
        and fixture_topic.get("source_node") == FIXTURE_PUBLISHER_NODE
        and fixture_topic.get("qos") == REQUIRED_TOPICS[FIXTURE_TOPIC]["qos"]
        and fixture_payload_ok
    )
    if not fixture_ok:
        reasons.append("fixture heartbeat/revision/digest/ownership/sequence does not match")

    scene = _as_mapping(snapshot.get("planning_scene"))
    scene_owned_ids = list(scene.get("owned_ids", ()))
    attached_ids = list(scene.get("attached_ids", ()))
    source_id = expected_fixture.get("target_source_id")
    if not (
        scene_owned_ids == list(expected_ids)
        and source_id in scene_owned_ids
        and source_id not in attached_ids
        and len(scene_owned_ids) == len(set(scene_owned_ids))
        and not (set(scene_owned_ids) & set(attached_ids))
    ):
        reasons.append("PlanningScene does not contain the exact world-only fixture target contract")

    if snapshot.get("robot_in_collision", True):
        reasons.append("robot starts in collision")
    return {"ready": not reasons, "reasons": reasons}


# ---------------------------------------------------------------------------
# Observed graph validation / journal projection
# ---------------------------------------------------------------------------

def _validate_endpoint_entries(label: str, endpoints: object) -> list[dict[str, str]]:
    """Validate real endpoint metadata (never payload-only claims)."""
    if not isinstance(endpoints, (list, tuple)) or not endpoints:
        raise ValueError(f"{label} must have real endpoint metadata")
    normalized: list[dict[str, str]] = []
    for endpoint in endpoints:
        if isinstance(endpoint, Mapping):
            node = endpoint.get("node")
            if not isinstance(node, str) or not node:
                raise ValueError(f"{label} has an endpoint without a real node")
            node_namespace = endpoint.get("node_namespace")
            normalized.append(
                {
                    "node": node,
                    "node_namespace": str(node_namespace) if node_namespace is not None else "",
                }
            )
        else:
            raise ValueError(f"{label} has malformed endpoint metadata")
    return normalized


def _qos_exact(qos: object, expected: Mapping[str, object]) -> bool:
    if not isinstance(qos, Mapping) or set(qos) != set(expected):
        return False
    for key, expected_value in expected.items():
        value = qos.get(key)
        if key == "depth":
            if isinstance(value, bool) or not isinstance(value, int) or value != expected_value:
                return False
        elif value != expected_value:
            return False
    return True


def _validate_observed_graph(
    observed_graph: object,
) -> dict[str, object]:
    """Validate an observed graph and return the exact projection shape.

    The observed graph must be a mapping with the exact recorder identity, the
    exact journal topic/service interface sets, exact types/QoS, real endpoint
    metadata, the recorder among every required topic subscriber and service
    client, exactly one ``/fixture_planning_scene`` publisher, and no extra
    interfaces.  The fixture topic entry does not carry a payload; the payload
    is injected by :func:`build_journal_graph_projection`.
    """
    if not isinstance(observed_graph, Mapping):
        raise ValueError("observed graph must be a mapping")
    if observed_graph.get("node_name") != OPERATOR_NODE:
        raise ValueError("observed graph node_name must be /tinker_integrated_gate_executor")
    if observed_graph.get("namespace") != OPERATOR_NODE_NAMESPACE:
        raise ValueError("observed graph namespace must be /")
    remap_table = observed_graph.get("remap_table")
    if not isinstance(remap_table, Mapping) or len(remap_table) != 0:
        raise ValueError("observed graph remap_table must be empty")
    topics = observed_graph.get("topics")
    services = observed_graph.get("services")
    if not isinstance(topics, Mapping) or not isinstance(services, Mapping):
        raise ValueError("observed graph must include topics and services mappings")
    expected_topic_keys = {
        PLANNING_SCENE_TOPIC,
        MONITORED_PLANNING_SCENE_TOPIC,
        FIXTURE_TOPIC,
    }
    expected_service_keys = {"/get_planning_scene", "/apply_planning_scene"}
    if set(topics) != expected_topic_keys:
        raise ValueError(
            "observed graph topics must be exactly "
            f"{sorted(expected_topic_keys)}"
        )
    if set(services) != expected_service_keys:
        raise ValueError(
            "observed graph services must be exactly "
            f"{sorted(expected_service_keys)}"
        )

    normalized_topics: dict[str, dict[str, object]] = {}
    for name, expected_type in (
        (PLANNING_SCENE_TOPIC, "moveit_msgs/msg/PlanningScene"),
        (MONITORED_PLANNING_SCENE_TOPIC, "moveit_msgs/msg/PlanningScene"),
    ):
        entry = _as_mapping(topics.get(name))
        if entry.get("type") != expected_type:
            raise ValueError(f"observed topic {name} has wrong type {entry.get('type')!r}")
        if not _qos_exact(entry.get("requested_qos"), JOURNAL_PLANNING_SCENE_TOPIC_QOS):
            raise ValueError(f"observed topic {name} requested QoS must be RELIABLE/VOLATILE/depth 100")
        if not _qos_exact(entry.get("offered_qos"), JOURNAL_PLANNING_SCENE_TOPIC_QOS):
            raise ValueError(f"observed topic {name} offered QoS must be RELIABLE/VOLATILE/depth 100")
        publishers = _validate_endpoint_entries(f"observed topic {name} publishers", entry.get("publishers"))
        subscribers = _validate_endpoint_entries(f"observed topic {name} subscribers", entry.get("subscribers"))
        if not any(endpoint["node"] == OPERATOR_NODE for endpoint in subscribers):
            raise ValueError(f"observed topic {name} must be subscribed by {OPERATOR_NODE}")
        normalized_topics[name] = {
            "type": expected_type,
            "requested_qos": dict(JOURNAL_PLANNING_SCENE_TOPIC_QOS),
            "offered_qos": dict(JOURNAL_PLANNING_SCENE_TOPIC_QOS),
            "publishers": publishers,
            "subscribers": subscribers,
        }

    fixture_entry = _as_mapping(topics.get(FIXTURE_TOPIC))
    if fixture_entry.get("type") != "std_msgs/msg/String":
        raise ValueError(f"observed topic {FIXTURE_TOPIC} has wrong type")
    if not _qos_exact(fixture_entry.get("requested_qos"), JOURNAL_FIXTURE_TOPIC_QOS):
        raise ValueError(f"observed topic {FIXTURE_TOPIC} requested QoS must be RELIABLE/TRANSIENT_LOCAL/depth 1")
    if not _qos_exact(fixture_entry.get("offered_qos"), JOURNAL_FIXTURE_TOPIC_QOS):
        raise ValueError(f"observed topic {FIXTURE_TOPIC} offered QoS must be RELIABLE/TRANSIENT_LOCAL/depth 1")
    fixture_publishers = _validate_endpoint_entries(
        f"observed topic {FIXTURE_TOPIC} publishers", fixture_entry.get("publishers")
    )
    fixture_subscribers = _validate_endpoint_entries(
        f"observed topic {FIXTURE_TOPIC} subscribers", fixture_entry.get("subscribers")
    )
    if len(fixture_publishers) != 1:
        raise ValueError(f"observed topic {FIXTURE_TOPIC} must have exactly one publisher")
    if fixture_publishers[0]["node"] != FIXTURE_PUBLISHER_NODE:
        raise ValueError(f"observed topic {FIXTURE_TOPIC} publisher must be {FIXTURE_PUBLISHER_NODE}")
    if not any(endpoint["node"] == OPERATOR_NODE for endpoint in fixture_subscribers):
        raise ValueError(f"observed topic {FIXTURE_TOPIC} must be subscribed by {OPERATOR_NODE}")
    normalized_topics[FIXTURE_TOPIC] = {
        "type": "std_msgs/msg/String",
        "requested_qos": dict(JOURNAL_FIXTURE_TOPIC_QOS),
        "offered_qos": dict(JOURNAL_FIXTURE_TOPIC_QOS),
        "publishers": fixture_publishers,
        "subscribers": fixture_subscribers,
    }

    normalized_services: dict[str, dict[str, object]] = {}
    for name, expected_type in (
        ("/get_planning_scene", "moveit_msgs/srv/GetPlanningScene"),
        ("/apply_planning_scene", "moveit_msgs/srv/ApplyPlanningScene"),
    ):
        entry = _as_mapping(services.get(name))
        if entry.get("type") != expected_type:
            raise ValueError(f"observed service {name} has wrong type {entry.get('type')!r}")
        if not _qos_exact(entry.get("requested_qos"), JOURNAL_SERVICE_QOS):
            raise ValueError(f"observed service {name} requested QoS must be RELIABLE/VOLATILE")
        if not _qos_exact(entry.get("offered_qos"), JOURNAL_SERVICE_QOS):
            raise ValueError(f"observed service {name} offered QoS must be RELIABLE/VOLATILE")
        servers = _validate_endpoint_entries(f"observed service {name} servers", entry.get("servers"))
        clients = _validate_endpoint_entries(f"observed service {name} clients", entry.get("clients"))
        if not any(endpoint["node"] == OPERATOR_NODE for endpoint in clients):
            raise ValueError(f"observed service {name} must be called by {OPERATOR_NODE}")
        normalized_services[name] = {
            "type": expected_type,
            "requested_qos": dict(JOURNAL_SERVICE_QOS),
            "offered_qos": dict(JOURNAL_SERVICE_QOS),
            "servers": servers,
            "clients": clients,
        }

    return {
        "node_name": OPERATOR_NODE,
        "namespace": OPERATOR_NODE_NAMESPACE,
        "remap_table": {},
        "topics": normalized_topics,
        "services": normalized_services,
    }


def build_journal_graph_projection(
    *,
    fixture_payload: str,
    observed_graph: Mapping[str, object],
) -> dict[str, object]:
    """Build the Task-3 graph projection from an explicit observed graph.

    ``observed_graph`` must represent the active attempt's real endpoint
    observations (never fabricated constants): the recorder identity
    ``node_name="/tinker_integrated_gate_executor"``, ``namespace="/"``,
    ``remap_table={}``; the exact topic/service key sets, types and uppercase
    QoS; the recorder among all required subscribers/clients; exactly one
    ``/fixture_planning_scene`` fixture publisher.  The exact canonical compact
    fixture payload string is injected as ``payload`` (parsed data separate).
    Fails closed on missing/extra interfaces, wrong type/QoS/source/cardinality,
    or an absent recorder subscriber/client.
    """
    projection = _validate_observed_graph(observed_graph)
    projection = copy.deepcopy(projection)
    # Inject the exact canonical compact fixture payload into the fixture topic
    # entry; parsed data is retained separately by the journal validator.
    projection["topics"][FIXTURE_TOPIC]["payload"] = str(fixture_payload)
    return projection


def stage_c_dispatch(
    scenario_id: str,
    *,
    scenario: Mapping[str, object],
) -> dict[str, object]:
    """Validate a Stage-C plan-only scenario and return a ROS-free dispatch spec.

    Requires the exact scenario id, ``integrated.stage == "C"``,
    ``integrated.execution_profile == "sim_ompl"``, and
    ``integrated.acceptance.polarity == "plan-only"``.  Returns a spec with the
    goal ``kind`` (joint/pose/blocked), the expected diagnostic polarity
    (success/non-success), the seven-joint ``Q_OUTBOUND`` for joint, and the
    declared target-object pose for pose/blocked.
    """
    if not isinstance(scenario_id, str) or not scenario_id:
        raise ValueError("scenario_id must be a nonempty string")
    if scenario.get("id") != scenario_id:
        raise ValueError("scenario_id does not match the executor scenario mapping")
    integrated = _as_mapping(scenario.get("integrated"))
    if integrated.get("stage") != "C":
        raise ValueError("scenario integrated.stage must be C for Gate C")
    if integrated.get("execution_profile") != INTEGRATED_EXECUTION_PROFILE:
        raise ValueError("scenario integrated.execution_profile must be sim_ompl")
    acceptance = _as_mapping(integrated.get("acceptance"))
    if acceptance.get("polarity") != "plan-only":
        raise ValueError("scenario integrated.acceptance.polarity must be plan-only")
    if scenario_id not in STAGE_C_SCENARIOS:
        raise ValueError(f"scenario {scenario_id} is not one of the Stage-C plan-only scenarios")

    declaration = _as_mapping(scenario.get("planning_scene_declaration"))
    if not declaration:
        declaration = _as_mapping(scenario.get("planning_scene"))
    objects = declaration.get("objects")
    if not isinstance(objects, (list, tuple)):
        raise ValueError("scenario planning_scene has no objects list")
    target_source_id = declaration.get("target_source_id")
    target = next(
        (record for record in objects if isinstance(record, Mapping) and record.get("id") == target_source_id),
        None,
    )
    if target is None:
        raise ValueError("scenario declaration has no target object matching target_source_id")
    pose = _as_mapping(target.get("pose"))
    xyz = pose.get("xyz")
    quaternion = pose.get("quaternion_xyzw")
    if not (
        isinstance(xyz, (list, tuple))
        and len(xyz) == 3
        and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in xyz)
    ):
        raise ValueError("scenario target pose xyz must be three finite values")
    if not (
        isinstance(quaternion, (list, tuple))
        and len(quaternion) == 4
        and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in quaternion)
    ):
        raise ValueError("scenario target pose quaternion_xyzw must be four finite values")

    kind = {
        "qualification-moveit-plan-joint": "joint",
        "qualification-moveit-plan-pose": "pose",
        "qualification-moveit-plan-blocked": "blocked",
    }[scenario_id]
    return {
        "scenario_id": scenario_id,
        "kind": kind,
        "expectation": "non-success" if kind == "blocked" else "success",
        "joints": list(Q_OUTBOUND) if kind == "joint" else None,
        "target_pose": {
            "xyz": [float(value) for value in xyz],
            "quaternion_xyzw": [float(value) for value in quaternion],
        },
    }


def _uuid_bytes_from_container(candidate: object) -> bytes | None:
    """Strictly convert a UUID container/iterable to exactly 16 bytes, else None.

    Accepts bytes, bytearray, ``array('B')``, memoryview, a numpy ``uint8[16]``
    array, or a list/tuple of byte integers.  Rejects bools, bare ints, strings
    (a string is treated as UUID text only by the top-level caller, never as a
    16-byte container here), wrong-length buffers, invalid element ranges/types,
    and any conversion exception.
    """
    if candidate is None or isinstance(candidate, bool):
        return None
    if isinstance(candidate, (bytes, bytearray)):
        converted = bytes(candidate)
    elif isinstance(candidate, (str, int)):
        return None
    else:
        try:
            converted = bytes(candidate)
        except (TypeError, ValueError):
            return None
    if len(converted) != 16:
        return None
    return converted


def _normalize_goal_uuid(raw: object) -> str | None:
    """Normalize a goal-handle UUID to lowercase 16-byte hex, else ``None``.

    Humble reality: rclpy ``ClientGoalHandle.goal_id`` is a
    ``unique_identifier_msgs/msg/UUID`` message whose ``.uuid`` field is a numpy
    ``uint8[16]`` array (a strict 16-byte buffer).  This helper preserves
    bytes/bytearray/string behavior and additionally accepts any UUID
    message/object whose ``.uuid`` (or, optionally, ``.bytes``) is a strict
    16-byte iterable/buffer: numpy ``uint8[16]``, ``array('B')``, list/tuple of
    byte integers, or memoryview.  Conversion goes through ``bytes(candidate)``
    and requires exactly 16 bytes.  Bools, malformed lengths, invalid element
    ranges/types, missing values, and any exception normalize to ``None`` so a
    split-path contract can never pass on an invalid identity.
    """
    if isinstance(raw, bool):
        return None
    try:
        if isinstance(raw, (bytes, bytearray)):
            return _uuid.UUID(bytes=bytes(raw)).hex
        if isinstance(raw, str):
            return _uuid.UUID(str(raw)).hex
        candidate = getattr(raw, "uuid", None)
        if candidate is None:
            candidate = getattr(raw, "bytes", None)
        if candidate is None:
            # A bare 16-byte container (numpy uint8[16], array('B'), memoryview,
            # or a list/tuple of byte integers) is itself the payload; a
            # malformed message without a usable payload then fails the strict
            # container conversion and returns None.
            candidate = raw
        converted = _uuid_bytes_from_container(candidate)
        if converted is None:
            return None
        return _uuid.UUID(bytes=converted).hex
    except (ValueError, AttributeError, TypeError):
        return None


def _valid_goal_uuid(value: object) -> bool:
    return isinstance(value, str) and len(value) == 32 and all(
        character in "0123456789abcdef" for character in value
    )


def _execute_status_name(status: object) -> str:
    """Map an ``action_msgs/msg/GoalStatus`` int to the canonical string.

    Only the three ExecuteTrajectory terminal statuses are accepted:
    SUCCEEDED=4, CANCELED=5, ABORTED=6.  Unknown/malformed statuses raise so a
    caller can never pass on an unapproved result.
    """
    if isinstance(status, bool) or not isinstance(status, int):
        raise ValueError(f"execute status must be an integer, found {status!r}")
    name = _EXECUTE_STATUS_NAMES.get(status)
    if name is None:
        raise ValueError(f"unknown ExecuteTrajectory terminal status: {status}")
    return name


def _arm_velocity_within_limit(frames: object, limit: object) -> bool:
    """True when every measured frame's arm-joint velocities are bounded.

    Each *frame* is a sequence of absolute arm-joint (``joint1..joint7``)
    velocities in rad/s.  *limit* is the configured
    ``safety_stop_velocity_rad_s``.  A frame with non-finite velocity, the wrong
    arity, or a magnitude above the limit fails the effective-stop predicate.
    """
    try:
        velocity_limit = float(limit)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(velocity_limit) or velocity_limit < 0.0:
        return False
    if not isinstance(frames, Sequence) or isinstance(frames, (str, bytes)):
        return False
    for frame in frames:
        if not isinstance(frame, Sequence) or isinstance(frame, (str, bytes)):
            return False
        if len(frame) != 7:
            return False
        for value in frame:
            try:
                magnitude = abs(float(value))
            except (TypeError, ValueError):
                return False
            if not math.isfinite(magnitude) or magnitude > velocity_limit:
                return False
    return True


def derive_retreat_target_pose(
    source_pose: Mapping[str, object],
    *,
    distance_m: object,
    axis: str,
) -> dict[str, object]:
    """Derive the deterministic ``base_link`` retreat target from a TCP pose.

    Moves the observed TCP pose exactly *distance_m* along the declared
    *axis* (``+z`` for the top-down grasp geometry) preserving orientation.
    Fails closed on missing/wrong-frame/non-finite/invalid-quaternion input so
    a missing or stale provider can never produce a goal.
    """
    xyz = source_pose.get("xyz")
    quaternion = source_pose.get("quaternion_xyzw")
    frame_id = source_pose.get("frame_id")
    if frame_id not in (None, "base_link"):
        raise ValueError("retreat source pose must use base_link")
    if not (
        isinstance(xyz, (list, tuple))
        and len(xyz) == 3
        and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in xyz)
    ):
        raise ValueError("retreat source pose xyz must be three finite values")
    if not (
        isinstance(quaternion, (list, tuple))
        and len(quaternion) == 4
        and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in quaternion)
    ):
        raise ValueError("retreat source pose quaternion_xyzw must be four finite values")
    norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
    if abs(norm - 1.0) > 1.0e-3:
        raise ValueError("retreat source quaternion must be normalized within 1e-3")
    try:
        distance = float(distance_m)
    except (TypeError, ValueError):
        raise ValueError("retreat distance must be a finite number")
    if not math.isfinite(distance) or distance < 0.0:
        raise ValueError("retreat distance must be a finite non-negative number")
    if axis not in ("+x", "-x", "+y", "-y", "+z", "-z"):
        raise ValueError(f"retreat axis must be one of +x/-x/+y/-y/+z/-z, found {axis!r}")
    offset = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0),
              "+z": (0, 0, 1), "-z": (0, 0, -1)}[axis]
    return {
        "frame_id": "base_link",
        "xyz": [
            float(xyz[0]) + distance * offset[0],
            float(xyz[1]) + distance * offset[1],
            float(xyz[2]) + distance * offset[2],
        ],
        "quaternion_xyzw": [float(value) for value in quaternion],
    }


def _pick_place_result_name(status: object) -> str:
    """Map a Pick/Place ``Result.status`` int to the canonical string.

    ``tinker_arm_msgs`` Pick/Place ``Result.status``: 0=success, 1=invalid_goal,
    2=planning_failed, 3=execution_failed, 4=canceled, 5=safety_stop,
    6=scene_inconsistent, 7=postcondition_failed, 8=timeout, 9=internal_error.
    Unknown/malformed statuses raise so a caller can never pass on an unapproved
    result.  This is a distinct status domain from the action-client
    ``action_msgs/msg/GoalStatus`` (``_execute_status_name``).
    """
    if isinstance(status, bool) or not isinstance(status, int):
        raise ValueError(f"Pick/Place result status must be an integer, found {status!r}")
    name = PICK_PLACE_RESULT_NAMES.get(status)
    if name is None:
        raise ValueError(f"unknown Pick/Place result status: {status}")
    return name


def stage_e_dispatch(
    scenario_id: str,
    *,
    scenario: Mapping[str, object],
) -> dict[str, object]:
    """Validate a Stage-E scenario and return a ROS-free dispatch spec.

    Requires the exact scenario id, ``integrated.stage == "E"``,
    ``integrated.execution_profile == "sim_ompl"``, the exact declared polarity
    (``positive`` / ``negative``), the exact configured ``expected_physical``
    list, the exact ``expected_negative`` contract (for negatives), the exact
    ``forbidden_endpoints == ["/isaac_joint_commands"]``, the exact declared
    ``trigger_timeout_s``, and the scenario-specific fixed-target geometry /
    back-position declarations.  Unknown, C/D-stage, malformed, or mutated
    scenarios fail closed before any goal is created or sent.  Returns the E
    handler ``kind``, polarity, expected physical/negative contracts, trigger
    timeout, the pinned fixed-target geometry, and (for malformed-back) the
    declared six-value back vector.
    """
    if not isinstance(scenario_id, str) or not scenario_id:
        raise ValueError("scenario_id must be a nonempty string")
    if scenario.get("id") != scenario_id:
        raise ValueError("scenario_id does not match the executor scenario mapping")
    integrated = _as_mapping(scenario.get("integrated"))
    if integrated.get("stage") != "E":
        raise ValueError("scenario integrated.stage must be E for Gate E")
    if integrated.get("execution_profile") != INTEGRATED_EXECUTION_PROFILE:
        raise ValueError("scenario integrated.execution_profile must be sim_ompl")
    if scenario_id not in STAGE_E_SCENARIOS:
        raise ValueError(f"scenario {scenario_id} is not one of the Stage-E scenarios")

    kind = STAGE_E_KIND[scenario_id]
    expected_polarity = STAGE_E_EXPECTED_POLARITY[scenario_id]
    acceptance = _as_mapping(integrated.get("acceptance"))
    declared_polarity = acceptance.get("polarity")
    if declared_polarity != expected_polarity:
        raise ValueError(
            f"scenario integrated.acceptance.polarity must be {expected_polarity!r}, "
            f"found {declared_polarity!r}"
        )
    declared_physical = integrated.get("expected_physical")
    expected_physical = list(STAGE_E_EXPECTED_PHYSICAL[scenario_id])
    if not (
        isinstance(declared_physical, (list, tuple))
        and not isinstance(declared_physical, (str, bytes))
        and list(declared_physical) == expected_physical
    ):
        raise ValueError(
            f"scenario integrated.expected_physical must be exactly {expected_physical}, "
            f"found {declared_physical!r}"
        )
    forbidden = integrated.get("forbidden_endpoints")
    if not (
        isinstance(forbidden, (list, tuple))
        and not isinstance(forbidden, (str, bytes))
        and list(forbidden) == [ISAAC_COMMAND_TOPIC]
    ):
        raise ValueError(
            f"scenario integrated.forbidden_endpoints must be exactly "
            f"[{ISAAC_COMMAND_TOPIC}], found {forbidden!r}"
        )
    declared_timeout = integrated.get("trigger_timeout_s")
    expected_timeout = STAGE_E_TRIGGER_TIMEOUT_S[scenario_id]
    if kind == "positive":
        if declared_timeout is not None:
            raise ValueError(
                f"scenario integrated.trigger_timeout_s must be None for positive, "
                f"found {declared_timeout!r}"
            )
    else:
        if not _finite_number(declared_timeout) or float(declared_timeout) != float(expected_timeout):
            raise ValueError(
                f"scenario integrated.trigger_timeout_s must be exactly "
                f"{expected_timeout}, found {declared_timeout!r}"
            )
    expected_negative: Mapping[str, object] | None = None
    if kind != "positive":
        declared_negative = integrated.get("expected_negative")
        if not isinstance(declared_negative, Mapping):
            raise ValueError("scenario integrated.expected_negative must be a mapping")
        expected_negative = STAGE_E_EXPECTED_NEGATIVE[scenario_id]
        for sub_key in ("required", "forbidden"):
            declared_sub = declared_negative.get(sub_key)
            expected_sub = list(expected_negative[sub_key])
            if not (
                isinstance(declared_sub, (list, tuple))
                and not isinstance(declared_sub, (str, bytes))
                and list(declared_sub) == expected_sub
            ):
                raise ValueError(
                    f"scenario integrated.expected_negative.{sub_key} must be exactly "
                    f"{expected_sub}, found {declared_sub!r}"
                )

    goal = _as_mapping(integrated.get("goal"))
    if goal.get("target_object_id") != TARGET_OBJECT_ID:
        raise ValueError(
            f"scenario integrated.goal.target_object_id must be {TARGET_OBJECT_ID!r}"
        )
    if goal.get("place_region") != "place-region":
        raise ValueError("scenario integrated.goal.place_region must be 'place-region'")

    back_positions: object = None
    if kind == "malformed-back":
        declared_back = integrated.get("back_positions")
        if not (
            isinstance(declared_back, (list, tuple))
            and not isinstance(declared_back, (str, bytes))
            and len(declared_back) == 6
            and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in declared_back)
            and [float(value) for value in declared_back] == list(E_MALFORMED_BACK_POSITIONS)
        ):
            raise ValueError(
                f"scenario integrated.back_positions must be exactly "
                f"{list(E_MALFORMED_BACK_POSITIONS)}, found {declared_back!r}"
            )
        back_positions = [float(value) for value in declared_back]
    if kind == "blocked-approach":
        if goal.get("approach") != "top-down":
            raise ValueError(
                f"scenario integrated.goal.approach must be 'top-down', found {goal.get('approach')!r}"
            )
        declared_tcp = goal.get("target_tcp_xyz")
        if not (
            isinstance(declared_tcp, (list, tuple))
            and not isinstance(declared_tcp, (str, bytes))
            and len(declared_tcp) == 3
            and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in declared_tcp)
            and [float(value) for value in declared_tcp] == list(E_GRASP_TCP_XYZ)
        ):
            raise ValueError(
                f"scenario integrated.goal.target_tcp_xyz must be exactly "
                f"{list(E_GRASP_TCP_XYZ)}, found {declared_tcp!r}"
            )

    return {
        "scenario_id": scenario_id,
        "kind": kind,
        "polarity": expected_polarity,
        "expected_physical": expected_physical,
        "expected_negative": (
            {key: list(values) for key, values in expected_negative.items()}
            if expected_negative is not None
            else None
        ),
        "trigger_timeout_s": None if kind == "positive" else float(declared_timeout),
        "forbidden_endpoints": [ISAAC_COMMAND_TOPIC],
        "geometry": {
            "grasp_tcp_xyz": list(E_GRASP_TCP_XYZ),
            "object_root_xyz": list(E_OBJECT_ROOT_XYZ),
            "place_target_point": dict(E_PLACE_TARGET_POINT),
            "place_orientation_xyzw": list(E_PLACE_ORIENTATION_XYZW),
        },
        "back_positions": back_positions,
    }


def _e_stage_event_order(scenario: Mapping[str, object]) -> tuple[str, ...]:
    """Return the scenario-specific E journal event order, else Gate C's."""
    integrated = _as_mapping(scenario.get("integrated"))
    if integrated.get("stage") != "E":
        return GATE_C_REQUIRED_EVENT_ORDER
    kind = STAGE_E_KIND.get(str(scenario.get("id", "")))
    if kind is None:
        return GATE_C_REQUIRED_EVENT_ORDER
    return STAGE_E_REQUIRED_EVENT_ORDER.get(kind, GATE_C_REQUIRED_EVENT_ORDER)


def _e_forbidden_events(scenario: Mapping[str, object]) -> tuple[str, ...]:
    """Return the scenario-specific E forbidden journal event set."""
    integrated = _as_mapping(scenario.get("integrated"))
    if integrated.get("stage") != "E":
        return GATE_C_FORBIDDEN_EVENTS
    kind = STAGE_E_KIND.get(str(scenario.get("id", "")))
    if kind is None:
        return GATE_C_FORBIDDEN_EVENTS
    return STAGE_E_FORBIDDEN_EVENTS.get(kind, GATE_C_FORBIDDEN_EVENTS)


def _first_fjt_goal_after_acceptance(
    entries: Sequence[Mapping[str, object]], *, base: Mapping[str, object]
) -> Mapping[str, object] | None:
    """First receipt-window FJT EXECUTING(2) entry received after *base*.

    Receipt-window correlation only: the observed FJT ``goal_uuid`` is recorded
    as evidence and is never claimed to equal the Pick/Place internal
    ``ExecuteTrajectory`` goal UUID (that UUID is private to production).
    """
    base_seq = int(base.get("fjt_seq", 0))
    for entry in entries:
        if int(entry.get("seq", 0)) > base_seq and int(entry.get("status", -1)) == 2:
            return entry
    return None


def _next_fjt_goal(
    entries: Sequence[Mapping[str, object]], after_seq: object
) -> Mapping[str, object] | None:
    """First later fresh FJT EXECUTING(2) entry with ``seq`` > *after_seq*."""
    for entry in entries:
        if int(entry.get("seq", 0)) > int(after_seq) and int(entry.get("status", -1)) == 2:
            return entry
    return None


def _fjt_receipt_delta_s(entry: Mapping[str, object], boundary_mono: object) -> float | None:
    """Seconds between the FJT status receipt and *boundary_mono* (or ``None``).

    F1.6: the receipt-time correlation window uses the FJT status topic
    ``received_mono`` timestamp versus the task-goal acceptance/latch baseline.
    A non-finite or negative delta (received before the boundary) is ``None``.
    """
    received = entry.get("received_mono")
    if received is None:
        return None
    try:
        delta = float(received) - float(boundary_mono)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(delta) or delta < 0.0:
        return None
    return delta


def _fjt_within_receipt_window(
    entry: Mapping[str, object], boundary_mono: object, window_s: object
) -> bool:
    """True when the FJT receipt is a fresh positive delta inside *window_s*."""
    try:
        window = float(window_s)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(window) or window < 0.0:
        return False
    delta = _fjt_receipt_delta_s(entry, boundary_mono)
    if delta is None:
        return False
    return delta <= window


def _post_grasp_lift_m_observation(
    sample: object,
    *,
    object_lift_m: float,
    fresh_limit_s: object,
) -> tuple[float, dict[str, object]] | str:
    """Validate one fresh observed ``post_grasp_lift_m`` runtime-parameter sample.

    F2.1: the E transport scenarios observe the production ``pick_and_place``
    runtime parameter ``post_grasp_lift_m`` (production default 0.08 m) through
    an injected provider.  The committed qualification requires ``object_lift_m``
    >= 0.10 m, so the observed lift value must be finite and ``>= object_lift_m``;
    anything missing/stale/non-finite/below-threshold returns a stable reason
    string.  On acceptance returns ``(value_m, meta)`` where *meta* carries the
    provider identity, receipt time, observed value, and the enforced threshold.
    """
    if not isinstance(sample, Mapping):
        return "missing"
    identity = sample.get("identity")
    if not (isinstance(identity, (int, str)) and str(identity)):
        return "missing"
    if not _fresh(sample.get("age_s"), fresh_limit_s):
        return "stale"
    value_m = sample.get("value_m")
    if isinstance(value_m, bool):
        return "non-finite"
    try:
        value_m = float(value_m)
    except (TypeError, ValueError):
        return "non-finite"
    if not math.isfinite(value_m):
        return "non-finite"
    if value_m < float(object_lift_m):
        return f"below-object-lift: observed {value_m} < required {float(object_lift_m)}"
    return value_m, {
        "identity": str(identity),
        "received_mono": float(time.monotonic()),
        "value_m": value_m,
        "object_lift_m": float(object_lift_m),
    }


def _tcp_z_from_samples(
    samples: Sequence[Mapping[str, object]],
) -> float | None:
    """Newest fresh TCP ``xyz[2]`` (``tcp_z_m``), ``None`` when no sample."""
    if not samples:
        return None
    newest = samples[-1]
    xyz = newest.get("xyz")
    if not (isinstance(xyz, (list, tuple)) and len(xyz) == 3):
        return None
    try:
        return float(xyz[2])
    except (TypeError, ValueError):
        return None


def _tcp_speed_from_samples(
    samples: Sequence[Mapping[str, object]],
) -> float | None:
    """Translational ``tcp_speed_m_s = |Δxyz| / Δt`` over the two newest samples.

    ``None`` when fewer than two samples or the two newest samples are
    non-finite / non-increasing in time (so the trigger cannot fire).
    """
    if len(samples) < 2:
        return None
    second = samples[-2]
    first = samples[-1]
    for sample in (first, second):
        if not isinstance(sample, Mapping):
            return None
    a = second.get("xyz")
    b = first.get("xyz")
    if not (
        isinstance(a, (list, tuple)) and len(a) == 3 and
        isinstance(b, (list, tuple)) and len(b) == 3
    ):
        return None
    try:
        ax, ay, az = (float(value) for value in a)
        bx, by, bz = (float(value) for value in b)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (ax, ay, az, bx, by, bz)):
        return None
    delta_t = float(first.get("received_mono", 0.0)) - float(second.get("received_mono", 0.0))
    if not math.isfinite(delta_t) or delta_t <= 0.0:
        return None
    return math.sqrt((bx - ax) ** 2 + (by - ay) ** 2 + (bz - az) ** 2) / delta_t


def stage_d_dispatch(
    scenario_id: str,
    *,
    scenario: Mapping[str, object],
) -> dict[str, object]:
    """Validate a Stage-D scenario and return a ROS-free dispatch spec.

    Requires the exact scenario id, ``integrated.stage == "D"``,
    ``integrated.execution_profile == "sim_ompl"``, the exact declared polarity
    (``positive`` / ``cancel`` / ``safety``), the exact configured
    ``expected_physical`` list for that scenario, and
    ``forbidden_endpoints == ["/isaac_joint_commands"]``.  Unknown, C/E-stage,
    malformed, or mutated scenarios fail closed before any goal is created or
    sent.  Returns the D handler ``kind``, the declared polarity/expected
    physical list, the seven-joint ``Q_OUTBOUND`` for execute-joint, and the
    declared ``sim_fixture/public_target`` pose for execute-pose.
    """
    if not isinstance(scenario_id, str) or not scenario_id:
        raise ValueError("scenario_id must be a nonempty string")
    if scenario.get("id") != scenario_id:
        raise ValueError("scenario_id does not match the executor scenario mapping")
    integrated = _as_mapping(scenario.get("integrated"))
    if integrated.get("stage") != "D":
        raise ValueError("scenario integrated.stage must be D for Gate D")
    if integrated.get("execution_profile") != INTEGRATED_EXECUTION_PROFILE:
        raise ValueError("scenario integrated.execution_profile must be sim_ompl")
    if scenario_id not in STAGE_D_SCENARIOS:
        raise ValueError(f"scenario {scenario_id} is not one of the Stage-D scenarios")

    kind = STAGE_D_KIND[scenario_id]
    expected_polarity = STAGE_D_EXPECTED_POLARITY[scenario_id]
    acceptance = _as_mapping(integrated.get("acceptance"))
    declared_polarity = acceptance.get("polarity")
    if declared_polarity != expected_polarity:
        raise ValueError(
            f"scenario integrated.acceptance.polarity must be {expected_polarity!r}, "
            f"found {declared_polarity!r}"
        )
    declared_physical = integrated.get("expected_physical")
    expected_physical = list(STAGE_D_EXPECTED_PHYSICAL[scenario_id])
    if not (
        isinstance(declared_physical, (list, tuple))
        and not isinstance(declared_physical, (str, bytes))
        and list(declared_physical) == expected_physical
    ):
        raise ValueError(
            f"scenario integrated.expected_physical must be exactly {expected_physical}, "
            f"found {declared_physical!r}"
        )
    forbidden = integrated.get("forbidden_endpoints")
    if not (
        isinstance(forbidden, (list, tuple))
        and not isinstance(forbidden, (str, bytes))
        and list(forbidden) == [ISAAC_COMMAND_TOPIC]
    ):
        raise ValueError(
            f"scenario integrated.forbidden_endpoints must be exactly "
            f"[{ISAAC_COMMAND_TOPIC}], found {forbidden!r}"
        )

    target_pose: dict[str, object] | None = None
    if kind == "execute-pose":
        declaration = _as_mapping(
            scenario.get("planning_scene_declaration") or scenario.get("planning_scene")
        )
        objects = declaration.get("objects")
        if not isinstance(objects, (list, tuple)):
            raise ValueError("scenario planning_scene has no objects list")
        target_source_id = declaration.get("target_source_id")
        target = next(
            (
                record
                for record in objects
                if isinstance(record, Mapping) and record.get("id") == target_source_id
            ),
            None,
        )
        if target is None:
            raise ValueError("scenario declaration has no target object matching target_source_id")
        pose = _as_mapping(target.get("pose"))
        xyz = pose.get("xyz")
        quaternion = pose.get("quaternion_xyzw")
        if not (
            isinstance(xyz, (list, tuple))
            and len(xyz) == 3
            and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in xyz)
        ):
            raise ValueError("scenario target pose xyz must be three finite values")
        if not (
            isinstance(quaternion, (list, tuple))
            and len(quaternion) == 4
            and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in quaternion)
        ):
            raise ValueError("scenario target pose quaternion_xyzw must be four finite values")
        norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
        if abs(norm - 1.0) > 1.0e-3:
            raise ValueError("scenario target pose quaternion must be normalized within 1e-3")
        target_pose = {
            "xyz": [float(value) for value in xyz],
            "quaternion_xyzw": [float(value) for value in quaternion],
        }

    return {
        "scenario_id": scenario_id,
        "kind": kind,
        "polarity": expected_polarity,
        "expected_physical": expected_physical,
        "joints": list(Q_OUTBOUND) if kind == "execute-joint" else None,
        "target_pose": target_pose,
        "forbidden_endpoints": [ISAAC_COMMAND_TOPIC],
    }


def _d_stage_event_order(scenario: Mapping[str, object]) -> tuple[str, ...]:
    """Return the scenario-specific D journal event order, else Gate C's."""
    integrated = _as_mapping(scenario.get("integrated"))
    if integrated.get("stage") != "D":
        return GATE_C_REQUIRED_EVENT_ORDER
    kind = STAGE_D_KIND.get(str(scenario.get("id", "")))
    if kind is None:
        return GATE_C_REQUIRED_EVENT_ORDER
    return STAGE_D_REQUIRED_EVENT_ORDER.get(kind, GATE_C_REQUIRED_EVENT_ORDER)


# ---------------------------------------------------------------------------
# ROS-lazy goal builders (import generated messages only at call time)
# ---------------------------------------------------------------------------

def build_joint_move_group_goal(
    joints: Sequence[float], *, plan_only: bool
):
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint

    values = _finite_vector(joints, length=7, name="joint goal")
    goal = MoveGroup.Goal()
    goal.request.group_name = "xarm7"
    goal.request.pipeline_id = "ompl"
    goal.request.num_planning_attempts = 3
    goal.request.allowed_planning_time = 3.0
    constraints = Constraints()
    constraints.joint_constraints = [
        JointConstraint(
            joint_name=f"joint{i}",
            position=value,
            tolerance_above=0.01,
            tolerance_below=0.01,
            weight=1.0,
        )
        for i, value in enumerate(values, start=1)
    ]
    goal.request.goal_constraints = [constraints]
    goal.planning_options.plan_only = bool(plan_only)
    goal.planning_options.replan = False
    # F2.6: pin the exact MoveGroup planning contract explicitly; never rely on
    # the moveit_msgs PlanningOptions.look_around default.
    goal.planning_options.look_around = False
    return goal


def build_pose_move_group_goal(pose, *, plan_only: bool):
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import BoundingVolume, Constraints, OrientationConstraint, PositionConstraint
    from shape_msgs.msg import SolidPrimitive

    if pose.header.frame_id != "base_link":
        raise ValueError("pose goal must use base_link")
    _validate_quaternion(pose.pose.orientation)
    primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])
    position = PositionConstraint(
        header=pose.header,
        link_name="link_tcp",
        constraint_region=BoundingVolume(
            primitives=[primitive], primitive_poses=[pose.pose]
        ),
        weight=1.0,
    )
    orientation = OrientationConstraint(
        header=pose.header,
        link_name="link_tcp",
        orientation=pose.pose.orientation,
        absolute_x_axis_tolerance=0.05,
        absolute_y_axis_tolerance=0.05,
        absolute_z_axis_tolerance=0.05,
        weight=1.0,
    )
    goal = MoveGroup.Goal()
    goal.request.group_name = "xarm7"
    goal.request.pipeline_id = "ompl"
    goal.request.num_planning_attempts = 3
    goal.request.allowed_planning_time = 3.0
    goal.request.goal_constraints = [
        Constraints(
            position_constraints=[position], orientation_constraints=[orientation]
        )
    ]
    goal.planning_options.plan_only = bool(plan_only)
    goal.planning_options.replan = False
    # F2.6: pin the exact MoveGroup planning contract explicitly; never rely on
    # the moveit_msgs PlanningOptions.look_around default.
    goal.planning_options.look_around = False
    return goal


def deterministic_cube_cloud(*, frame_id="base_link"):
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    offsets = (-0.04, -0.02, 0.0, 0.02, 0.04)
    points = [
        (0.65 + dx, 0.0 + dy, 0.60 + dz)
        for dx in offsets for dy in offsets for dz in offsets
    ]
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    header = Header(frame_id=frame_id)
    header.stamp.sec = 7
    cloud = point_cloud2.create_cloud(header, fields, points)
    cloud.height = 1
    cloud.width = 125
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 1500
    cloud.is_dense = True
    return cloud


def build_pick_goal(
    *,
    target_pose,
    candidate_poses: Sequence,
    env_points,
    object_points,
    back_positions: Sequence[float],
    use_mesh: bool,
    stay: bool,
    two_stage_plan: bool = False,
):
    from tinker_arm_msgs.action import Pick

    back = _finite_vector(back_positions, length=7, name="back_positions")
    if not candidate_poses or candidate_poses[0] != target_pose:
        raise ValueError("candidate_poses must be non-empty and start with target_pose")
    _validate_quaternion(target_pose.orientation)
    goal = Pick.Goal()
    goal.target_pose = target_pose
    goal.candidate_poses = list(candidate_poses)
    goal.env_points = env_points
    goal.object_points = object_points
    goal.back_positions = back
    goal.two_stage_plan = bool(two_stage_plan)
    goal.use_mesh = bool(use_mesh)
    goal.stay = bool(stay)
    return goal


def build_place_goal(
    *,
    target_point,
    orientation,
    env_points,
    back_positions: Sequence[float],
):
    """Build a Place goal.

    Qualification-only constraint (Task 4 plan): ``target_point.header.frame_id``
    must be ``base_link``.  The production ``Place`` server accepts an arbitrary
    frame and TF-transforms it, but Task 4 deliberately restricts to ``base_link``
    to keep the deterministic qualification geometry frame-local; later execute
    gates may broaden this.
    """
    from tinker_arm_msgs.action import Place

    if target_point.header.frame_id != "base_link":
        raise ValueError("place target must use base_link")
    _validate_quaternion(orientation.orientation)
    goal = Place.Goal()
    goal.target_point = target_point
    goal.orientation = orientation
    goal.env_points = env_points
    goal.back_positions = _finite_vector(back_positions, length=7, name="back_positions")
    return goal


def build_execute_trajectory_goal(planned_trajectory):
    """Construct exactly one ``moveit_msgs/action/ExecuteTrajectory.Goal``.

    The returned ``planned_trajectory`` is assigned directly to
    ``goal.trajectory`` without mutation/replanning/round-trip reconstruction;
    the caller records the canonical ROS-serialized trajectory digest before and
    after assignment and requires them to match.  A ``None`` or empty trajectory
    fails closed.
    """
    from moveit_msgs.action import ExecuteTrajectory

    if planned_trajectory is None:
        raise ValueError("ExecuteTrajectory goal requires a non-empty planned trajectory")
    points = getattr(
        getattr(planned_trajectory, "joint_trajectory", None), "points", None
    )
    if not isinstance(points, (list, tuple)) or not points:
        raise ValueError("ExecuteTrajectory goal requires a non-empty planned trajectory")
    goal = ExecuteTrajectory.Goal()
    goal.trajectory = planned_trajectory
    return goal


def build_gripper_goal(position, *, max_effort):
    """Construct one native ``control_msgs/action/GripperCommand`` goal.

    Task-5 qualification constants: open position ``0.0``, close position
    ``0.85``, max effort ``10.0``.  A non-finite position/effort fails closed.
    """
    from control_msgs.action import GripperCommand

    try:
        position_value = float(position)
        effort_value = float(max_effort)
    except (TypeError, ValueError):
        raise ValueError("gripper position and max effort must be finite numbers")
    if not math.isfinite(position_value) or not math.isfinite(effort_value):
        raise ValueError("gripper position and max effort must be finite numbers")
    goal = GripperCommand.Goal()
    goal.command.position = position_value
    goal.command.max_effort = effort_value
    return goal


def build_cartesian_move_goal(target_pose, *, env_points=None):
    """Construct one ``tinker_arm_msgs/action/CartesianMove`` goal.

    ``target_pose`` is a ``geometry_msgs/msg/Pose`` in ``base_link``.  The goal
    carries the collision-aware production path (``env_points`` PointCloud2 when
    supplied).  A non-``base_link`` header (when a PoseStamped is supplied) or a
    non-finite target fails closed.
    """
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import PointCloud2
    from tinker_arm_msgs.action import CartesianMove

    if target_pose is None:
        raise ValueError("CartesianMove goal requires a target pose")
    pose = getattr(target_pose, "pose", target_pose)
    if getattr(pose, "position", None) is None:
        raise ValueError("CartesianMove goal requires a real Pose target")
    _validate_quaternion(pose.orientation)
    header_frame = getattr(target_pose, "header", None)
    if header_frame is not None and getattr(header_frame, "frame_id", "base_link") != "base_link":
        raise ValueError("cartesian retreat target must use base_link")
    goal = CartesianMove.Goal()
    if isinstance(target_pose, Pose):
        goal.target_pose = target_pose
    else:
        goal.target_pose = pose
    # The generated CartesianMove goal requires a PointCloud2 (never None) for
    # the collision-aware env_points field; an empty cloud is the neutral
    # default when no environment cloud is supplied.
    goal.env_points = env_points if env_points is not None else PointCloud2()
    return goal


# ---------------------------------------------------------------------------
# Live ROS-lazy executor (imported/instantiated only under sourced Humble)
# ---------------------------------------------------------------------------

_ROS_IMPORTS: dict[str, Any] = {}


def _load_ros() -> dict[str, Any]:
    """Import ROS only at live execution time (Humble CPython 3.10)."""
    if _ROS_IMPORTS:
        return _ROS_IMPORTS
    # F1.1: require or set the fast-DDS RMW before importing rclpy.
    configured_rmw = os.environ.get("RMW_IMPLEMENTATION")
    if configured_rmw is not None and configured_rmw != RMW_IMPLEMENTATION:
        raise RuntimeError(
            f"IntegratedGateExecutor requires RMW_IMPLEMENTATION={RMW_IMPLEMENTATION}; "
            f"found {configured_rmw}"
        )
    if configured_rmw is None:
        os.environ["RMW_IMPLEMENTATION"] = RMW_IMPLEMENTATION
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from controller_manager_msgs.srv import (
        ConfigureController,
        ListControllers,
        LoadController,
        SwitchController,
    )
    from moveit_msgs.action import ExecuteTrajectory, MoveGroup
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetStateValidity
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from tinker_arm_msgs.action import CartesianMove, Fold, JointMove, Pick, Place
    from tinker_arm_msgs.srv import ArmJointService

    if rclpy.get_rmw_implementation_identifier() != RMW_IMPLEMENTATION:
        raise RuntimeError(
            "rclpy loaded RMW "
            f"{rclpy.get_rmw_implementation_identifier()!r}; expected {RMW_IMPLEMENTATION}"
        )
    _ROS_IMPORTS.update(locals())
    return _ROS_IMPORTS


def _operator_qos(ros: Mapping[str, Any]) -> Any:
    return ros["QoSProfile"](
        depth=1,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].TRANSIENT_LOCAL,
    )


def _planning_scene_qos(ros: Mapping[str, Any]) -> Any:
    """Stock MoveIt2 Humble PlanningScene contract: RELIABLE/VOLATILE/depth 100."""
    return ros["QoSProfile"](
        depth=100,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].VOLATILE,
    )


def _fixture_qos(ros: Mapping[str, Any]) -> Any:
    """Fixture/safety/operator status contract: RELIABLE/TRANSIENT_LOCAL/depth 1."""
    return ros["QoSProfile"](
        depth=1,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].TRANSIENT_LOCAL,
    )


def _joint_state_qos(ros: Mapping[str, Any]) -> Any:
    return ros["QoSProfile"](
        depth=10,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].VOLATILE,
    )


def _fjt_status_qos(ros: Mapping[str, Any]) -> Any:
    """Stock Humble action status QoS (``rcl_action/default_qos.h``):
    RELIABLE / TRANSIENT_LOCAL / depth 1."""
    return ros["QoSProfile"](
        depth=1,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].TRANSIENT_LOCAL,
    )


def _atomic_write_json(value: object, path: Path) -> None:
    """Write *value* canonically through temp-file + fsync + replace + dir fsync.

    Mirrors the strongest repository durability pattern (Task 3 journal) so a
    pass claim is never exposed before the parent directory entry is durable.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        try:
            dir_fd = os.open(path.parent, os.O_RDONLY)
        except OSError:
            pass
        else:
            try:
                os.fsync(dir_fd)
            finally:
                os.close(dir_fd)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class IntegratedGateExecutor:
    """Live Gate-C plan-only executor node.

    Runs under sourced Humble Python 3.10 only.  Creates a private
    ``rclpy.context.Context`` per executor, initializes it with the exact
    requested ``ros_domain_id``, and creates the node
    ``/tinker_integrated_gate_executor`` (basename ``tinker_integrated_gate_executor``,
    namespace ``/``, ``use_global_arguments=False``).  It creates typed
    action/service clients for every required endpoint, the operator publisher
    ``/sim/safety/operator`` (``std_msgs/msg/Bool``, reliable/transient-local/
    depth 1), real ``moveit_msgs/msg/PlanningScene`` subscriptions, an owned
    ``PlanningSceneJournal``, and the Gate C plan-only flow.

    Providers (Task 7/orchestration supplies them later; Task 4 defines the
    contracts and tests them):

    - ``join_key_provider`` -> exact raw/evaluator ``(frame_index, timestamp)``;
    - ``readiness_snapshot_provider`` -> the complete observed readiness
      snapshot evaluated by :func:`evaluate_executor_readiness`;
    - ``graph_observation_provider`` -> the observed graph for journal
      finalization.

    Gate C never sends ``/execute_trajectory`` and never publishes
    ``/isaac_joint_commands``.  Plan-only evidence records remain
    ``diagnostic_only = true`` and never claim execution.
    """

    def __init__(
        self,
        *,
        scenario: Mapping[str, object],
        attempt_dir: Path,
        config: Mapping[str, object],
        ros_domain_id: int | str,
        journal: Any = None,
        join_key_provider: Callable[[], tuple[int, float]] | None = None,
        readiness_snapshot_provider: Callable[[], Mapping[str, object]] | None = None,
        graph_observation_provider: Callable[[], Mapping[str, object]] | None = None,
    ) -> None:
        domain_id = self._validate_domain(ros_domain_id)
        if not isinstance(scenario, Mapping):
            raise ValueError("scenario must be a mapping")
        if not isinstance(config, Mapping):
            raise ValueError("config must be a mapping")
        self.scenario = scenario
        self.config = config
        self.attempt_dir = Path(attempt_dir).resolve()
        self.attempt_dir.mkdir(parents=True, exist_ok=True)
        self._reject_stale_attempt_evidence()

        declaration = _as_mapping(
            scenario.get("planning_scene_declaration") or scenario.get("planning_scene")
        )
        self.fixture_revision = str(declaration.get("revision", ""))
        if not self.fixture_revision:
            raise ValueError("scenario planning_scene has no revision")

        # Providers.
        self.join_key_provider = join_key_provider
        self.readiness_snapshot_provider = readiness_snapshot_provider
        self.graph_observation_provider = graph_observation_provider

        self.ros = _load_ros()
        self._context_initialized = False
        self.context = self.ros["Context"]()
        self._init_rclpy(domain_id)
        self._context_initialized = True
        self.node = self.ros["Node"](
            NODE_BASENAME,
            namespace=OPERATOR_NODE_NAMESPACE,
            cli_args=[],
            context=self.context,
            use_global_arguments=False,
        )
        self._spinner = self.ros["SingleThreadedExecutor"](context=self.context)
        self._spinner.add_node(self.node)
        self.operator_publisher = self.node.create_publisher(
            self.ros["Bool"], OPERATOR_TOPIC, _operator_qos(self.ros)
        )
        # F1.1: private collection of every real owned ActionClient, kept apart
        # from the mutable public map that tests may replace with fakes.  Must
        # exist before ``_create_clients`` so the constructor can track each
        # real client and a partial-constructor failure can destroy them.
        self._owned_action_clients: list[Any] = []
        try:
            self._create_clients()
            self._create_subscriptions()
        except Exception:
            # F1.1: partial constructor failure must destroy the already-created
            # action clients before node/context cleanup so no waitable leaks.
            self._destroy_owned_action_clients()
            try:
                self.node.destroy_node()
            except Exception:
                pass
            try:
                self.ros["rclpy"].shutdown(context=self.context)
            except Exception:
                pass
            self._context_initialized = False
            raise

        # Journal ownership (F1.3): default to a real PlanningSceneJournal.
        # Task 6 branches the construction for stage E (per-scenario exact
        # event order + per-scenario forbidden events; never the Gate C/D set
        # and never POSITIVE_ORDER for a negative scenario).
        if journal is not None:
            self.journal = journal
        else:
            integrated = _as_mapping(self.scenario.get("integrated"))
            if integrated.get("stage") == "E":
                self.journal = self._build_e_journal(
                    required_event_order=_e_stage_event_order(self.scenario),
                    forbidden_events=_e_forbidden_events(self.scenario),
                )
            else:
                self.journal = self._build_d_journal(
                    required_event_order=_d_stage_event_order(self.scenario)
                )

        self._latest_planning_scene: dict[str, object] | None = None
        self._planning_scene_invalid = False
        self._scene_invalid_sequence: int | None = None
        self._fixture_payload: str | None = None
        self._fixture_payload_invalid = False
        self._latest_joint_state: Any = None
        self._latest_safety_stop: Any = None
        self._joint_velocity_frames: list[dict[str, object]] = []
        self._fjt_status_cache: list[dict[str, object]] = []
        self._fjt_receipt_sequence = 0
        self._last_fjt_discovery_error: str | None = None
        # F5.3: real ExecuteTrajectory action-status cache (terminal evidence for
        # acceptance-timeout exact-goal cleanup).  Kept outside the journal graph
        # projection exactly like the FJT status cache.
        self._execute_status_cache: list[dict[str, object]] = []
        self._execute_receipt_sequence = 0
        self._joint_receipt_sequence = 0
        self._scene_sequence = 0
        self._last_join_key: tuple[int, float] | None = None
        # Task 6: bounded per-attempt TCP sample deque fed by the injected
        # ``current_tcp_pose_provider`` (never a TF listener embedded here).
        self._tcp_pose_samples: list[dict[str, object]] = []
        self._last_tcp_pose_provider: Callable[[], Mapping[str, object]] | None = None
        # F1.7/F1.9/F1.10: strictly per-attempt Gate-E trigger/latch state.  Reset
        # at the start of every public E entry point so a reused executor can
        # never carry a previous attempt's sample/latch/goal evidence into the
        # next attempt.
        self._e_active_goal_handle: Any = None
        self._e_goal_state: dict[str, object] = {
            "pick_sent": False,
            "pick_goal_id": None,
            "place_sent": False,
            "place_goal_id": None,
        }
        self._e_native_gripper_count_provider: Callable[[], Mapping[str, object]] | None = None
        self._e_native_gripper_count_baseline: int | None = None
        # F2.1: strictly per-attempt observation of the production
        # ``pick_and_place`` runtime parameter ``post_grasp_lift_m``.  The
        # injected provider is a live-observable seam (never a tf/action
        # dependency); the accepted observation is persisted into E artifacts.
        self._e_post_grasp_lift_m_provider: Callable[[], Mapping[str, object]] | None = None
        self._e_post_grasp_lift_m_observed: Mapping[str, object] | None = None

    # -- construction helpers ----------------------------------------------

    @staticmethod
    def _validate_domain(ros_domain_id: int | str) -> int:
        if isinstance(ros_domain_id, bool):
            raise ValueError("ROS_DOMAIN_ID must be an integer, not a boolean")
        try:
            domain_id = int(ros_domain_id)
        except (TypeError, ValueError):
            raise ValueError("ROS_DOMAIN_ID must be an integer in [0, 232]")
        if domain_id < 0 or domain_id > 232:
            raise ValueError("ROS_DOMAIN_ID must be an integer in [0, 232]")
        return domain_id

    def _init_rclpy(self, domain_id: int) -> None:
        self.ros["rclpy"].init(args=[], context=self.context, domain_id=domain_id)

    def _reject_stale_attempt_evidence(self) -> None:
        jsonl = self.attempt_dir / "planning-scene.jsonl"
        if jsonl.exists() and jsonl.stat().st_size > 0:
            raise ValueError(f"planning-scene.jsonl already contains records: {jsonl}")
        for name in ARTIFACT_JSONL_FILES:
            path = self.attempt_dir / name
            if path.exists() and path.stat().st_size > 0:
                raise ValueError(f"{name} already contains records: {path}")

    def _build_d_journal(self, *, required_event_order: Sequence[str]):
        """F2.2: construct a fresh D ``PlanningSceneJournal`` with the given order."""
        from planning_scene_journal import PlanningSceneJournal, load_model_touch_contract

        contract = load_model_touch_contract()
        return PlanningSceneJournal(
            fixture_revision=self.fixture_revision,
            task_namespace=TASK_NAMESPACE,
            target_object_id=TARGET_OBJECT_ID,
            expected_attach_link=contract["link_tcp"],
            expected_touch_links=contract["touch_links"],
            required_event_order=required_event_order,
            forbidden_events=D_FORBIDDEN_EVENTS,
            jsonl_path=self.attempt_dir / "planning-scene.jsonl",
        )

    def _build_e_journal(
        self, *, required_event_order: Sequence[str], forbidden_events: Sequence[str]
    ):
        """Task 6: construct a fresh E ``PlanningSceneJournal``.

        The E journal carries the scenario-specific exact event order and the
        scenario-specific forbidden set (never ``D_FORBIDDEN_EVENTS`` for a
        positive/transport scenario, which would reject the manipulation events
        the E journal must record).
        """
        from planning_scene_journal import PlanningSceneJournal, load_model_touch_contract

        contract = load_model_touch_contract()
        return PlanningSceneJournal(
            fixture_revision=self.fixture_revision,
            task_namespace=TASK_NAMESPACE,
            target_object_id=TARGET_OBJECT_ID,
            expected_attach_link=contract["link_tcp"],
            expected_touch_links=contract["touch_links"],
            required_event_order=tuple(required_event_order),
            forbidden_events=tuple(forbidden_events),
            jsonl_path=self.attempt_dir / "planning-scene.jsonl",
        )

    def _rebuild_gripper_journal_close_first(self) -> str:
        """F2.2: select the close-first gripper journal contract before any record.

        ``run_gripper_sequence(open_first=False)`` needs a journal whose required
        event order matches ``fixture-ready → gripper-close-terminal →
        gripper-open-terminal → teardown``.  The journal is only rebuilt while it
        is still fresh (zero durable records and an empty/absent jsonl path); an
        in-progress journal is never mutated — the attempt fails closed.
        """
        journal = getattr(self, "journal", None)
        if journal is not None and getattr(journal, "record_count", 0) > 0:
            return "refused: journal already holds records"
        jsonl_path = getattr(journal, "jsonl_path", None) if journal is not None else None
        if jsonl_path is not None:
            path = Path(jsonl_path)
            if path.exists() and path.stat().st_size > 0:
                return "refused: planning-scene.jsonl already holds records"
        self.journal = self._build_d_journal(
            required_event_order=GRIPPER_CLOSE_FIRST_EVENT_ORDER
        )
        return "rebuilt"

    def _create_clients(self) -> None:
        ros = self.ros
        self._action_clients: dict[str, Any] = {}
        for name, action_type in REQUIRED_ACTIONS.items():
            action_class_name = action_type.split("/")[-1]
            action_class = ros.get(action_class_name)
            if action_class is None:
                raise RuntimeError(
                    f"missing imported action class {action_class_name} for {name}"
                )
            client = ros["ActionClient"](self.node, action_class, name)
            self._action_clients[name] = client
            self._owned_action_clients.append(client)
        self._service_clients: dict[str, Any] = {}
        for name, service_type in REQUIRED_SERVICES.items():
            message_type = _service_type_to_ros(service_type, ros)
            self._service_clients[name] = self.node.create_client(message_type, name)
        if len(self._action_clients) != len(REQUIRED_ACTIONS):
            raise RuntimeError("not all required action clients were created")
        if len(self._service_clients) != len(REQUIRED_SERVICES):
            raise RuntimeError("not all required service clients were created")

    def _create_subscriptions(self) -> None:
        ros = self.ros
        self.node.create_subscription(
            ros["PlanningScene"],
            PLANNING_SCENE_TOPIC,
            self._make_scene_callback(PLANNING_SCENE_TOPIC),
            _planning_scene_qos(ros),
        )
        self.node.create_subscription(
            ros["PlanningScene"],
            MONITORED_PLANNING_SCENE_TOPIC,
            self._make_scene_callback(MONITORED_PLANNING_SCENE_TOPIC),
            _planning_scene_qos(ros),
        )
        self.node.create_subscription(
            ros["String"],
            FIXTURE_TOPIC,
            self._on_fixture_payload,
            _fixture_qos(ros),
        )
        self.node.create_subscription(
            ros["JointState"],
            JOINT_STATES_TOPIC,
            self._on_joint_state,
            _joint_state_qos(ros),
        )
        self.node.create_subscription(
            ros["Bool"],
            SAFETY_STOP_TOPIC,
            self._on_safety_stop,
            _fixture_qos(ros),
        )
        # D-stage FJT status observation.  ROS 2 actions do NOT publish an
        # ``_action/goal`` topic; the goal travels over the ``send_goal``
        # service.  Only the real ``_action/status`` subscription exists, with
        # the stock Humble action status QoS (RELIABLE/TRANSIENT_LOCAL/depth 1).
        # This subscription stays outside the Task-3 three-topic/two-service
        # journal graph projection.
        self.node.create_subscription(
            ros["GoalStatusArray"],
            FJT_STATUS_TOPIC,
            self._on_fjt_status,
            _fjt_status_qos(ros),
        )
        # F5.3: real ExecuteTrajectory action-status subscription for the
        # acceptance-timeout exact-goal cleanup terminal-evidence requirement.
        # Same stock Humble action-status QoS; stays outside the journal graph
        # projection like the FJT subscription.
        self.node.create_subscription(
            ros["GoalStatusArray"],
            EXECUTE_STATUS_TOPIC,
            self._on_execute_status,
            _fjt_status_qos(ros),
        )

    def _make_scene_callback(self, source: str):
        def callback(message: Any) -> None:
            try:
                normalized = self._normalize_planning_scene(message, source=source)
            except _SCENE_CALLBACK_EXCEPTIONS:
                # F3.2: a transient malformed message latches fail-closed but
                # never erases the last valid cached scene; a later valid
                # callback clears the latch.
                self._planning_scene_invalid = True
                self._scene_invalid_sequence = self._scene_sequence
            else:
                self._latest_planning_scene = normalized
                self._planning_scene_invalid = False
                self._scene_invalid_sequence = None

        return callback

    def _on_fixture_payload(self, message: Any) -> None:
        payload = str(getattr(message, "data", ""))
        try:
            self._validate_canonical_fixture_payload(payload)
        except ValueError:
            self._fixture_payload_invalid = True
            return
        self._fixture_payload_invalid = False
        self._fixture_payload = payload

    def _on_joint_state(self, message: Any) -> None:
        self._latest_joint_state = message
        frame = self._arm_velocity_frame(message)
        if frame is not None:
            self._joint_receipt_sequence += 1
            self._joint_velocity_frames.append(
                {
                    "seq": self._joint_receipt_sequence,
                    "received_mono": float(time.monotonic()),
                    "velocities": frame,
                    "positions": self._arm_position_frame(message),
                }
            )
            limit = int(self._thresholds().get("safety_stop_frames", 5))
            if limit < 1:
                limit = 1
            if len(self._joint_velocity_frames) > limit:
                del self._joint_velocity_frames[: len(self._joint_velocity_frames) - limit]

    @staticmethod
    def _arm_velocity_frame(message: Any) -> list[float] | None:
        """Extract the seven arm-joint absolute velocities from a JointState."""
        names = list(getattr(message, "name", ()))
        velocities = list(getattr(message, "velocity", ()))
        by_name = dict(zip(names, velocities))
        try:
            return [float(by_name[f"joint{index}"]) for index in range(1, 8)]
        except (KeyError, TypeError, ValueError):
            return None

    @staticmethod
    def _arm_position_frame(message: Any) -> list[float]:
        """Extract the seven arm-joint positions from a JointState (0s absent)."""
        names = list(getattr(message, "name", ()))
        positions = list(getattr(message, "position", ()))
        by_name = dict(zip(names, positions))
        try:
            return [float(by_name[f"joint{index}"]) for index in range(1, 8)]
        except (KeyError, TypeError, ValueError):
            return [0.0] * 7

    def _on_safety_stop(self, message: Any) -> None:
        self._latest_safety_stop = message

    def _on_fjt_status(self, message: Any) -> None:
        """Cache bounded, well-formed ``(goal_uuid, status)`` FJT status entries.

        Only well-formed entries with a valid lowercase 16-byte goal UUID and a
        strict integer status are retained; malformed entries are dropped.  The
        cache is bounded (the newest ``FJT_STATUS_CACHE_LIMIT`` entries) so a
        live controller cannot grow memory without bound.
        """
        status_list = getattr(message, "status_list", None)
        if not isinstance(status_list, (list, tuple)):
            return
        for status_entry in status_list:
            goal_info = getattr(status_entry, "goal_info", None)
            goal_id = getattr(goal_info, "goal_id", None)
            goal_uuid = _normalize_goal_uuid(goal_id)
            status = getattr(status_entry, "status", None)
            if goal_uuid is None or isinstance(status, bool) or not isinstance(status, int):
                continue
            self._fjt_receipt_sequence += 1
            # F5.1: every cache entry carries the exact observed status-topic
            # source (never a generic prose label).
            self._fjt_status_cache.append(
                {
                    "goal_uuid": goal_uuid,
                    "status": int(status),
                    "received_mono": float(time.monotonic()),
                    "seq": self._fjt_receipt_sequence,
                    "source": FJT_STATUS_TOPIC,
                }
            )
        if len(self._fjt_status_cache) > FJT_STATUS_CACHE_LIMIT:
            del self._fjt_status_cache[: len(self._fjt_status_cache) - FJT_STATUS_CACHE_LIMIT]

    def _on_execute_status(self, message: Any) -> None:
        """Cache bounded, well-formed ``(goal_uuid, status)`` execute entries.

        F5.3: mirrors ``_on_fjt_status`` but reads the real ExecuteTrajectory
        action-status topic.  The driver uses these entries as observable
        terminal evidence for the exact preassigned execute goal UUID during
        acceptance-timeout cleanup.  Malformed entries are dropped and the cache
        is bounded (newest ``EXECUTE_STATUS_CACHE_LIMIT`` entries).
        """
        status_list = getattr(message, "status_list", None)
        if not isinstance(status_list, (list, tuple)):
            return
        for status_entry in status_list:
            goal_info = getattr(status_entry, "goal_info", None)
            goal_id = getattr(goal_info, "goal_id", None)
            goal_uuid = _normalize_goal_uuid(goal_id)
            status = getattr(status_entry, "status", None)
            if goal_uuid is None or isinstance(status, bool) or not isinstance(status, int):
                continue
            self._execute_receipt_sequence += 1
            self._execute_status_cache.append(
                {
                    "goal_uuid": goal_uuid,
                    "status": int(status),
                    "received_mono": float(time.monotonic()),
                    "seq": self._execute_receipt_sequence,
                    "source": EXECUTE_STATUS_TOPIC,
                }
            )
        if len(self._execute_status_cache) > EXECUTE_STATUS_CACHE_LIMIT:
            del self._execute_status_cache[: len(self._execute_status_cache) - EXECUTE_STATUS_CACHE_LIMIT]

    def _execute_status_entries(self) -> list[dict[str, object]]:
        return list(self._execute_status_cache)

    def _wait_for_execute_status(
        self,
        goal_uuid: str,
        target_statuses: Sequence[int],
        timeout_s: object,
    ) -> dict[str, object] | None:
        """Bounded wait for a real execute-status entry for *goal_uuid*.

        F5.3: used only by the driver's acceptance-timeout cleanup to require
        observable terminal evidence for the exact preassigned execute goal UUID
        before teardown.  Entries are captured once as an immutable copy so a
        later status emission cannot race the caller.  Returns ``None`` on
        timeout or malformed input.
        """
        if not (isinstance(goal_uuid, str) and goal_uuid):
            return None
        try:
            wanted = set(int(status) for status in target_statuses)
        except (TypeError, ValueError):
            return None
        captured: dict[str, object] | None = None

        def _seen() -> bool:
            nonlocal captured
            for entry in reversed(self._execute_status_cache):
                if entry.get("goal_uuid") != goal_uuid:
                    continue
                if int(entry.get("status", -1)) in wanted:
                    captured = dict(entry)
                    return True
            return False

        if not self._wait_for(_seen, timeout_s):
            return None
        return captured

    def _fjt_status_entries(self) -> list[dict[str, object]]:
        return list(self._fjt_status_cache)

    def _newest_fjt_status(self) -> dict[str, object] | None:
        return self._fjt_status_cache[-1] if self._fjt_status_cache else None

    def _seed_fjt_status(self, goal_uuid: str, status: int, *, seq: int | None = None) -> None:
        """Seed one FJT status-topic entry (test/offline path)."""
        self._fjt_receipt_sequence += 1
        # F5.1: the seeded entry carries the exact observed status-topic source
        # so provider/validator joins are identical for real and offline paths.
        self._fjt_status_cache.append(
            {
                "goal_uuid": goal_uuid,
                "status": int(status),
                "received_mono": float(time.monotonic()),
                "seq": seq if seq is not None else self._fjt_receipt_sequence,
                "source": FJT_STATUS_TOPIC,
            }
        )
        if len(self._fjt_status_cache) > FJT_STATUS_CACHE_LIMIT:
            del self._fjt_status_cache[: len(self._fjt_status_cache) - FJT_STATUS_CACHE_LIMIT]

    def _seed_joint_frame(
        self, velocities: Sequence[float], *, positions: Sequence[float] | None = None
    ) -> None:
        """Seed one joint-state velocity frame (test/offline path)."""
        self._joint_receipt_sequence += 1
        self._joint_velocity_frames.append(
            {
                "seq": self._joint_receipt_sequence,
                "received_mono": float(time.monotonic()),
                "velocities": [float(value) for value in velocities],
                "positions": [float(value) for value in positions] if positions is not None else [0.0] * 7,
            }
        )
        limit = int(self._thresholds().get("safety_stop_frames", 5))
        if limit < 1:
            limit = 1
        if len(self._joint_velocity_frames) > limit:
            del self._joint_velocity_frames[: len(self._joint_velocity_frames) - limit]

    # -- windowed, fresh, bounded evidence helpers (F1.4) --------------------

    def _d_baseline(self) -> dict[str, object]:
        """Capture the current stream receipt positions at execution start.

        F4.2: in addition to the FJT/joint receipt sequences and start
        monotonic, the baseline snapshots the deterministic set of controller
        FJT goal UUIDs already known before the transaction.  A later unique-new
        discovery therefore ignores UUIDs present here even if an action-status
        heartbeat republishes them after the baseline.
        """
        return {
            "fjt_seq": self._fjt_receipt_sequence,
            "joint_seq": self._joint_receipt_sequence,
            "start_mono": float(time.monotonic()),
            "known_fjt_goal_uuids": self._snapshot_known_fjt_goal_uuids(),
        }

    def _snapshot_known_fjt_goal_uuids(self) -> tuple[str, ...]:
        """Return the sorted distinct FJT goal UUIDs known at snapshot time."""
        return tuple(sorted({
            str(entry.get("goal_uuid"))
            for entry in self._fjt_status_cache
            if isinstance(entry, Mapping)
            and isinstance(entry.get("goal_uuid"), str)
            and entry.get("goal_uuid")
        }))

    def _known_fjt_goal_uuids(self, baseline: Mapping[str, object]) -> frozenset[str]:
        """Return the baseline-known controller FJT goal UUIDs as a set."""
        raw = baseline.get("known_fjt_goal_uuids", ())
        if isinstance(raw, (list, tuple, set, frozenset)):
            return frozenset(str(value) for value in raw if value)
        return frozenset()

    def _new_fjt_goal_uuids(self, baseline: Mapping[str, object]) -> list[str]:
        """Distinct valid controller FJT goal UUIDs first seen after *baseline*.

        Only entries received after the baseline receipt sequence and carrying a
        valid 16-byte UUID that is NOT in the baseline-known set are considered.
        """
        known = self._known_fjt_goal_uuids(baseline)
        seen: set[str] = set()
        for entry in self._fresh_fjt_entries(baseline):
            goal_uuid = entry.get("goal_uuid")
            if _valid_goal_uuid(goal_uuid) and goal_uuid not in known:
                seen.add(str(goal_uuid))
        return sorted(seen)

    def _discover_new_fjt_goal(
        self, baseline: Mapping[str, object]
    ) -> tuple[str | None, str | None]:
        """Discover exactly one distinct new valid controller FJT goal UUID.

        Returns ``(goal_uuid, error)``.  Fails closed on zero new UUIDs (the
        transaction's controller goal has not yet appeared) and on multiple
        distinct new UUIDs (ambiguous transaction).  UUIDs already known at
        *baseline* are ignored even if republished after the baseline.
        """
        new_uuids = self._new_fjt_goal_uuids(baseline)
        if not new_uuids:
            return None, "no new controller FJT goal UUID was observed after the baseline"
        if len(new_uuids) > 1:
            return None, f"multiple new controller FJT goal UUIDs were observed: {new_uuids}"
        return new_uuids[0], None

    def _fresh_fjt_entries(
        self, baseline: Mapping[str, object], goal_uuid: object | None = None
    ) -> list[dict[str, object]]:
        """FJT status entries received after *baseline* (optionally for a goal)."""
        base_seq = int(baseline.get("fjt_seq", 0))
        entries = [
            entry for entry in self._fjt_status_cache
            if isinstance(entry, Mapping) and int(entry.get("seq", 0)) > base_seq
        ]
        if goal_uuid is not None:
            entries = [entry for entry in entries if entry.get("goal_uuid") == goal_uuid]
        return entries

    def _latest_fresh_joint_frame(
        self, baseline: Mapping[str, object]
    ) -> Mapping[str, object] | None:
        """Newest joint-state frame received after *baseline*, else None."""
        base_seq = int(baseline.get("joint_seq", 0))
        for frame in reversed(self._joint_velocity_frames):
            if isinstance(frame, Mapping) and int(frame.get("seq", 0)) > base_seq:
                return frame
        return None

    def _wait_for(self, predicate: Callable[[], bool], timeout_s: object) -> bool:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if predicate():
                return True
            self._spin_once()
        return False

    def _wait_for_fjt_status(
        self,
        goal_uuid: str | None,
        target_statuses: Sequence[int],
        timeout_s: object,
        *,
        baseline: Mapping[str, object],
    ) -> dict[str, object] | None:
        """Bounded wait for a fresh joined FJT entry in *target_statuses*.

        F2.6: the matching entry is captured once inside the predicate so cache
        trimming cannot produce an inconsistent second lookup (the pre-fix
        ``and _seen() or None`` idiom could return None if the bounded cache
        trimmed between the predicate check and the re-scan).

        F4.2: when *goal_uuid* is a real UUID the wait filters exactly on that
        controller goal.  When ``None`` the wait performs unique-new-controller-
        goal discovery: it requires exactly one distinct new valid FJT goal UUID
        (not present in the baseline-known set) in the current window and waits
        for that goal to reach one of *target_statuses*.  Multiple distinct new
        UUIDs fail closed immediately (returns None).  UUIDs already known at
        baseline are ignored even if republished after the baseline.
        """
        wanted = set(int(status) for status in target_statuses)
        captured: dict[str, object] | None = None
        discovery_error: list[str] = []

        def _seen() -> bool:
            nonlocal captured
            if goal_uuid is None:
                discovered, error = self._discover_new_fjt_goal(baseline)
                if discovered is None:
                    if error is not None and "multiple" in error:
                        # Ambiguous transaction: fail closed immediately.
                        discovery_error.append(error)
                        return True
                    return False
                for entry in reversed(self._fresh_fjt_entries(baseline, discovered)):
                    if int(entry.get("status", -1)) in wanted:
                        # F5.1: capture an immutable copy of the exact entry so
                        # a later status emission/cache trim cannot change it.
                        captured = dict(entry)
                        return True
                return False
            for entry in reversed(self._fresh_fjt_entries(baseline, goal_uuid)):
                if int(entry.get("status", -1)) in wanted:
                    # F5.1: immutable single capture (see above).
                    captured = dict(entry)
                    return True
            return False

        if not self._wait_for(_seen, timeout_s):
            return None
        if discovery_error:
            self._last_fjt_discovery_error = "; ".join(discovery_error)
            return None
        return captured

    def _wait_for_fjt_executing(
        self,
        goal_uuid: str | None,
        timeout_s: object,
        *,
        baseline: Mapping[str, object],
    ) -> bool:
        """Bounded wait for the joined FJT goal to reach EXECUTING (2)."""
        return self._wait_for_fjt_status(goal_uuid, (2,), timeout_s, baseline=baseline) is not None

    def _wait_for_motion_trigger(
        self,
        timeout_s: object,
        *,
        baseline: Mapping[str, object],
        threshold: object,
    ) -> bool:
        """Bounded wait for a fresh current-attempt joint frame proving motion.

        At least one fresh frame (received after *baseline*) must have some
        arm-joint absolute velocity above *threshold*; a transaction that never
        started moving cannot be interrupted.
        """
        try:
            limit = float(threshold)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(limit) or limit < 0.0:
            return False

        def _moving() -> bool:
            # Any fresh current-attempt frame proving arm motion satisfies the
            # trigger; a later stopped frame does not retroactively erase that a
            # moving transaction was observed.
            base_seq = int(baseline.get("joint_seq", 0))
            for frame in reversed(self._joint_velocity_frames):
                if not isinstance(frame, Mapping) or int(frame.get("seq", 0)) <= base_seq:
                    break
                velocities = frame.get("velocities")
                if not isinstance(velocities, Sequence) or isinstance(velocities, (str, bytes)):
                    continue
                if any(
                    isinstance(value, (int, float))
                    and math.isfinite(float(value))
                    and abs(float(value)) > limit
                    for value in velocities
                ):
                    return True
            return False

        return self._wait_for(_moving, timeout_s)

    def _wait_for_stopped_frames(
        self,
        count: object,
        timeout_s: object,
        *,
        baseline: Mapping[str, object],
        velocity_limit: object,
    ) -> list[Mapping[str, object]]:
        """Bounded wait for *count* consecutive fresh bounded joint frames.

        Every frame must be received after *baseline* and carry all seven
        arm-joint absolute velocities at or below *velocity_limit*.  Frames are
        consecutive in the cache (which trims to ``safety_stop_frames``).
        """
        try:
            required = int(count)
            limit = float(velocity_limit)
        except (TypeError, ValueError):
            return []
        if required < 1 or not math.isfinite(limit) or limit < 0.0:
            return []
        base_seq = int(baseline.get("joint_seq", 0))

        def _bounded_run() -> list[Mapping[str, object]]:
            run: list[Mapping[str, object]] = []
            for frame in reversed(self._joint_velocity_frames):
                if not isinstance(frame, Mapping) or int(frame.get("seq", 0)) <= base_seq:
                    break
                velocities = frame.get("velocities")
                if not isinstance(velocities, Sequence) or isinstance(velocities, (str, bytes)):
                    break
                if len(velocities) != 7:
                    break
                if not all(
                    isinstance(value, (int, float)) and math.isfinite(float(value)) and abs(float(value)) <= limit
                    for value in velocities
                ):
                    break
                run.append(frame)
            return run

        def _ready() -> bool:
            return len(_bounded_run()) >= required

        self._wait_for(_ready, timeout_s)
        return _bounded_run()

    def _wait_for_post_clear_stability(
        self,
        timeout_s: object,
        *,
        baseline: Mapping[str, object],
        known_goal_id: str,
        velocity_limit: object,
        creep_limit: object,
    ) -> dict[str, object]:
        """Bounded post-clear stability observation (F1.4).

        Within the bounded window: no new action/controller goal UUID appears,
        every fresh joint frame has all velocities bounded, and every arm-joint
        position remains within *creep_limit* of the clear-time baseline.  The
        return carries the stability result plus the measured max creep.

        F4.4: *known_goal_id* is the distinct controller FJT goal UUID (never
        the ExecuteTrajectory UUID); the no-fresh-goal check allows only that
        controller goal's terminal status.
        """
        try:
            limit = float(velocity_limit)
            creep = float(creep_limit)
        except (TypeError, ValueError):
            return {"stable": False, "reason": "non-finite thresholds"}
        if not math.isfinite(limit) or limit < 0.0 or not math.isfinite(creep) or creep < 0.0:
            return {"stable": False, "reason": "non-finite thresholds"}
        base_seq = int(baseline.get("joint_seq", 0))
        clear_positions = list(baseline.get("clear_positions") or [])
        if len(clear_positions) != 7:
            return {"stable": False, "reason": "no clear-time position baseline"}
        max_creep = 0.0
        seen_uuid: str | None = None
        terminal = False
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            # No fresh goal UUID beyond the terminal controller FJT goal.
            fresh = self._fresh_fjt_entries(baseline)
            if fresh:
                for entry in reversed(fresh):
                    uuid = entry.get("goal_uuid")
                    status = int(entry.get("status", -1))
                    if uuid != known_goal_id:
                        terminal = True
                        seen_uuid = str(uuid)
                        break
                    if status not in (EXECUTE_STATUS_SUCCEEDED, EXECUTE_STATUS_CANCELED, EXECUTE_STATUS_ABORTED):
                        terminal = True
                        seen_uuid = str(uuid)
                        break
                    break  # only newest matters for the no-fresh-goal check
            frame = self._latest_fresh_joint_frame(baseline)
            if frame is not None:
                velocities = frame.get("velocities")
                positions = frame.get("positions")
                if not isinstance(velocities, Sequence) or len(velocities) != 7:
                    terminal = True
                    seen_uuid = "malformed-velocity-frame"
                    break
                if not all(
                    isinstance(value, (int, float)) and math.isfinite(float(value)) and abs(float(value)) <= limit
                    for value in velocities
                ):
                    terminal = True
                    seen_uuid = "unbounded-velocity"
                    break
                if isinstance(positions, Sequence) and len(positions) == 7:
                    for index, value in enumerate(positions):
                        try:
                            delta = abs(float(value) - float(clear_positions[index]))
                        except (TypeError, ValueError):
                            terminal = True
                            seen_uuid = "non-finite-position"
                            break
                        if delta > max_creep:
                            max_creep = delta
                        if delta > creep:
                            terminal = True
                            seen_uuid = "position-creep"
                            break
                    if terminal:
                        break
            self._spin_once()
        if terminal:
            return {"stable": False, "reason": f"fresh goal or unbounded state: {seen_uuid}", "max_creep": round(max_creep, 6)}
        return {"stable": True, "reason": None, "max_creep": round(max_creep, 6)}

    def _normalize_planning_scene(self, message: Any, *, source: str) -> dict[str, object]:
        ros = self.ros
        self._scene_sequence += 1
        owned_ids = [str(collision_object.id) for collision_object in message.world.collision_objects]
        attached = list(message.robot_state.attached_collision_objects)
        attached_ids = [str(attached_object.object.id) for attached_object in attached]
        attached_links = {
            str(attached_object.object.id): str(attached_object.link_name)
            for attached_object in attached
        }
        touch_links = {
            str(attached_object.object.id): [str(link) for link in attached_object.touch_links]
            for attached_object in attached
        }
        fixture_digest, fixture_geometry = self._fixture_geometry_projection(message)
        return {
            "scene_sequence": self._scene_sequence,
            "scene_timestamp": float(time.monotonic()),
            "owned_ids": owned_ids,
            "attached_ids": attached_ids,
            "attached_links": attached_links,
            "touch_links": touch_links,
            "fixture_revision": self.fixture_revision,
            "fixture_geometry_digest": fixture_digest,
            "fixture_geometry": fixture_geometry,
            "scene_revision_digest": self._digest(ros["serialize_message"](message)),
            "acm_digest": self._digest(ros["serialize_message"](message.allowed_collision_matrix)),
            "robot_state_digest": self._digest(ros["serialize_message"](message.robot_state)),
            "source": source,
        }

    def _fixture_declaration(self) -> Mapping[str, object]:
        return _as_mapping(
            self.scenario.get("planning_scene_declaration")
            or self.scenario.get("planning_scene")
        )

    def _expected_fixture_geometry_digest(self) -> str | None:
        declaration = self._fixture_declaration()
        if not declaration:
            return None
        try:
            return expected_fixture_geometry_digest(declaration)
        except Exception:
            return None

    def _fixture_geometry_projection(
        self, message: Any
    ) -> tuple[str, list[dict[str, object]]]:
        """F3.3: canonical fixture-owned geometry projection of the received scene.

        Extracts, in declared owned-ID order, the canonical bridge geometry
        descriptor for every scenario-owned collision object present in the
        message.  A missing or geometry-less owned object yields an empty
        descriptor so the digest cannot accidentally match; the projection never
        includes foreign objects or unrelated full-scene serialization (robot
        state / ACM).  Returns ``(digest, ordered_descriptors)``.
        """
        expected_ids = list(fixture_owned_ids(self._fixture_declaration()))
        by_id: dict[str, dict[str, object]] = {}
        for collision_object in message.world.collision_objects:
            object_id = str(getattr(collision_object, "id", ""))
            if object_id not in expected_ids:
                continue
            try:
                by_id[object_id] = dict(readback_geometry(collision_object))
            except Exception:
                # A geometry-less/malformed owned object can never match the
                # declared projection; record an empty descriptor so the digest
                # differs instead of failing normalization.
                by_id[object_id] = {
                    "id": object_id,
                    "frame_id": "",
                    "primitives": [],
                    "primitive_poses": [],
                    "meshes": [],
                    "mesh_poses": [],
                }
        ordered = [by_id[object_id] for object_id in expected_ids if object_id in by_id]
        return geometry_signature_sha256(ordered), ordered

    @staticmethod
    def _digest(data: bytes) -> str:
        return hashlib.sha256(bytes(data)).hexdigest()

    @staticmethod
    def _validate_canonical_fixture_payload(payload: str) -> dict[str, object]:
        if not isinstance(payload, str) or not payload:
            raise ValueError("fixture payload must be a nonempty canonical compact JSON string")
        try:
            parsed = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError("fixture payload must be parseable JSON") from exc
        if not isinstance(parsed, dict):
            raise ValueError("fixture payload must be a JSON object")
        canonical = json.dumps(parsed, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
        if canonical != payload:
            raise ValueError("fixture payload must be the canonical compact fixture-status encoding")
        if set(parsed) != FIXTURE_STATUS_KEYS:
            raise ValueError("fixture payload must be the exact canonical fixture-status field set")
        return parsed

    # -- operator publisher -------------------------------------------------

    def publish_operator(self, value: bool) -> None:
        if value not in (False, True):
            raise ValueError("operator payload allowlist is [False, True]")
        message = self.ros["Bool"]()
        message.data = bool(value)
        self.operator_publisher.publish(message)

    # -- providers / journal scene ------------------------------------------

    def _join_key(self) -> tuple[int, float] | None:
        if self.join_key_provider is None:
            return None
        # F5.4: a valid-shaped but non-advancing key is a file-tail read race:
        # the truth stream advances continuously, but two journal snapshots
        # landing inside one truth frame observe the same (frame_index,
        # timestamp).  Wait a bounded time for the next advancing frame before
        # failing closed, so a live always-advancing truth never emits a
        # spurious ``no-join-key``.  Malformed keys, a missing provider, or a
        # genuinely stalled truth stream still fail closed after the window.
        deadline = time.monotonic() + JOIN_KEY_RETRY_S
        while True:
            try:
                key = self.join_key_provider()
            except Exception:
                key = None
            if not isinstance(key, (tuple, list)) or len(key) != 2:
                return None
            frame_index = key[0]
            timestamp = key[1]
            if isinstance(frame_index, bool) or not isinstance(frame_index, int) or frame_index < 0:
                return None
            if isinstance(timestamp, bool):
                return None
            try:
                timestamp = float(timestamp)
            except (TypeError, ValueError):
                return None
            if not math.isfinite(timestamp) or timestamp < 0.0:
                return None
            if self._last_join_key is not None:
                previous_frame, previous_timestamp = self._last_join_key
                if frame_index <= previous_frame or timestamp <= previous_timestamp:
                    if time.monotonic() >= deadline:
                        return None
                    time.sleep(0.002)
                    continue
            self._last_join_key = (frame_index, timestamp)
            return (frame_index, timestamp)

    def _journal_scene(self, join: tuple[int, float]) -> dict[str, object] | None:
        diagnostic = self._latest_planning_scene
        if diagnostic is None:
            return None
        frame_index, timestamp = join
        return {**dict(diagnostic), "frame_index": frame_index, "timestamp": timestamp}

    def _readiness(self) -> dict[str, object] | None:
        if self.readiness_snapshot_provider is None:
            return None
        try:
            snapshot = self.readiness_snapshot_provider()
        except Exception:
            return {"ready": False, "reasons": ["readiness_snapshot_provider raised"]}
        if not isinstance(snapshot, Mapping):
            return {"ready": False, "reasons": ["readiness_snapshot_provider returned a non-mapping"]}
        return evaluate_executor_readiness(snapshot, self.config, self.scenario)

    def _graph_observation(self) -> Mapping[str, object] | None:
        if self.graph_observation_provider is None:
            return None
        try:
            graph = self.graph_observation_provider()
        except Exception:
            return None
        return graph if isinstance(graph, Mapping) else None

    # -- goal construction ---------------------------------------------------

    def _pose_stamped_from_spec(self, spec: Mapping[str, object]):
        from geometry_msgs.msg import PoseStamped

        pose = _as_mapping(spec.get("target_pose"))
        xyz = pose.get("xyz")
        quaternion = pose.get("quaternion_xyzw")
        stamped = PoseStamped()
        stamped.header.frame_id = "base_link"
        stamped.pose.position.x = float(xyz[0])
        stamped.pose.position.y = float(xyz[1])
        stamped.pose.position.z = float(xyz[2])
        stamped.pose.orientation.x = float(quaternion[0])
        stamped.pose.orientation.y = float(quaternion[1])
        stamped.pose.orientation.z = float(quaternion[2])
        stamped.pose.orientation.w = float(quaternion[3])
        return stamped

    def _build_goal(self, spec: Mapping[str, object], joints: Sequence[float] | None):
        kind = spec.get("kind")
        if kind == "joint":
            target = list(joints) if joints is not None else list(spec.get("joints", Q_OUTBOUND))
            return build_joint_move_group_goal(target, plan_only=True)
        pose = self._pose_stamped_from_spec(spec)
        return build_pose_move_group_goal(pose, plan_only=True)

    # -- plan-only transaction -----------------------------------------------

    def _spin_once(self) -> None:
        self._spinner.spin_once(timeout_sec=0.05)

    def _wait_for_server(self, client: Any, timeout_s: float) -> bool:
        try:
            return bool(client.wait_for_server(timeout_sec=float(timeout_s)))
        except Exception:
            return False

    def _thresholds(self) -> Mapping[str, object]:
        return _as_mapping(self.config.get("thresholds"))

    def _threshold_timeout(self, key: str, default: float) -> float:
        """Return a finite positive threshold timeout; fail closed on malformed.

        F5.2: an explicit finite positive scenario override is authoritative;
        missing, boolean, non-finite, zero, negative, or otherwise malformed
        overrides fail closed (raising ValueError, which the run methods convert
        to ``evidence-invalid``).  The production defaults for the FJT and
        motion-trigger waits are exactly 10.0 seconds.
        """
        raw = self._thresholds().get(key, default)
        if isinstance(raw, bool):
            raise ValueError(f"{key} must be a finite positive number, got bool")
        try:
            value = float(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{key} must be a finite positive number, got {raw!r}") from exc
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{key} must be a finite positive number, got {raw!r}")
        return value

    def _send_plan_only_goal(
        self,
        scenario_id: str,
        goal: Any,
        spec: Mapping[str, object],
    ) -> dict[str, object]:
        """Send exactly one plan-only goal with bounded, correctly cancelled waits.

        F2.2/F2.8: every exceptional completion (server wait, send future,
        acceptance, result future, cancellation) is converted into a finite
        canonical diagnostic outcome with a stable reason code.  An unaccepted or
        indeterminate send future can never pass, and the evidence states that
        canceling a client future is not proof of server-side cancellation.
        """
        thresholds = self._thresholds()
        client = self._action_clients["/move_action"]
        server_timeout_s = float(thresholds.get("action_server_wait_s", 5.0))
        if not self._wait_for_server(client, server_timeout_s):
            return {
                "scenario_id": scenario_id,
                "status": "action-server-unavailable",
                "reason_code": "action-server-unavailable",
                "diagnostic_only": True,
                "error": "/move_action server was not available before send",
            }

        accept_timeout_s = float(thresholds.get("goal_accept_timeout_s", 5.0))
        send_future = client.send_goal_async(goal)
        accept_deadline = time.monotonic() + accept_timeout_s
        while not send_future.done() and time.monotonic() < accept_deadline:
            self._spin_once()
        if not send_future.done():
            # F2.8: canceling the client future is a client-side no-op and is not
            # proof of server-side cancellation; do not claim a cancel.
            try:
                send_future.cancel()
            except Exception:
                pass
            return {
                "scenario_id": scenario_id,
                "status": "goal-accept-timeout",
                "reason_code": "goal-accept-timeout",
                "diagnostic_only": True,
                "error": (
                    "goal acceptance timed out before a goal handle existed; "
                    "canceling the client send future is not proof of server-side cancellation"
                ),
                "send_future_cancelled": True,
            }
        try:
            goal_handle = send_future.result()
        except Exception as exc:  # F2.2: an exceptional send completion fails closed.
            return {
                "scenario_id": scenario_id,
                "status": "goal-send-exception",
                "reason_code": "goal-send-exception",
                "diagnostic_only": True,
                "error": f"send_goal future raised: {exc}",
            }
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            return {
                "scenario_id": scenario_id,
                "status": "goal-rejected",
                "reason_code": "goal-rejected",
                "diagnostic_only": True,
                "error": "send_goal returned no accepted goal handle",
            }

        result_timeout_s = float(thresholds.get("plan_result_timeout_s", 10.0))
        result_future = goal_handle.get_result_async()
        result_deadline = time.monotonic() + result_timeout_s
        while not result_future.done() and time.monotonic() < result_deadline:
            self._spin_once()
        if not result_future.done():
            cancel_response = self._cancel_goal(goal_handle)
            return {
                "scenario_id": scenario_id,
                "status": "timeout",
                "reason_code": "result-timeout",
                "diagnostic_only": True,
                "error": "planning result timed out",
                "cancel_response": cancel_response,
            }
        try:
            result = result_future.result()
        except Exception as exc:
            return {
                "scenario_id": scenario_id,
                "status": "malformed-result",
                "reason_code": "malformed-result",
                "diagnostic_only": True,
                "error": f"result future raised: {exc}",
            }
        if result is None or getattr(result, "result", None) is None:
            return {
                "scenario_id": scenario_id,
                "status": "malformed-result",
                "reason_code": "malformed-result",
                "diagnostic_only": True,
                "error": "result future returned no MoveGroup result",
            }
        return self._classify_plan_only_result(scenario_id, result, spec)

    def _cancel_goal(self, goal_handle: Any) -> str:
        cancel_timeout_s = float(self._thresholds().get("cancel_timeout_s", 3.0))
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception:
            return "cancel-failed"
        cancel_deadline = time.monotonic() + cancel_timeout_s
        while not cancel_future.done() and time.monotonic() < cancel_deadline:
            self._spin_once()
        return "completed" if cancel_future.done() else "timed-out"

    def _cancel_execute_goal(
        self,
        goal_handle: Any,
        *,
        expected_goal_uuid: str,
        timeout_s: object,
    ) -> dict[str, object]:
        """F1.2: call ``cancel_goal_async()`` once and require the accepted shape.

        Humble ``CancelGoal.Response`` carries ``return_code`` and
        ``goals_canceling`` (a list of ``GoalInfo``).  Acceptance requires
        ``return_code == ERROR_NONE`` and ``goals_canceling`` containing exactly
        the ExecuteTrajectory goal UUID.  Rejected/unknown/empty/extra/
        malformed/exceptional/timed-out responses fail closed.
        """
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as exc:
            return {
                "response": "failed", "return_code": None, "goals_canceling": [],
                "error": f"cancel_goal_async raised: {exc}",
            }
        cancel_deadline = time.monotonic() + float(timeout_s)
        while not cancel_future.done() and time.monotonic() < cancel_deadline:
            self._spin_once()
        if not cancel_future.done():
            return {
                "response": "timed-out", "return_code": None, "goals_canceling": [],
                "error": "cancel response future did not resolve within the bounded wait",
            }
        try:
            response = cancel_future.result()
        except Exception as exc:
            return {
                "response": "failed", "return_code": None, "goals_canceling": [],
                "error": f"cancel response future raised: {exc}",
            }
        return_code = getattr(response, "return_code", None)
        goals_canceling = [
            normalized
            for normalized in (
                _normalize_goal_uuid(getattr(goal_info, "goal_id", None))
                for goal_info in getattr(response, "goals_canceling", [])
            )
            if normalized is not None
        ]
        if not _strict_int(return_code):
            return {
                "response": "unknown", "return_code": return_code,
                "goals_canceling": goals_canceling,
                "error": "cancel response return_code is not a strict integer",
            }
        if return_code != 0:
            return {
                "response": "rejected", "return_code": return_code,
                "goals_canceling": goals_canceling,
                "error": f"cancel response return_code {return_code} != ERROR_NONE (0)",
            }
        if goals_canceling != [expected_goal_uuid]:
            return {
                "response": "rejected", "return_code": return_code,
                "goals_canceling": goals_canceling,
                "error": f"cancel response goals_canceling {goals_canceling} != [{expected_goal_uuid}]",
            }
        return {
            "response": "accepted", "return_code": return_code,
            "goals_canceling": goals_canceling, "error": None,
        }

    def _cleanup_execute_goal(self, goal_handle: Any, *, timeout_s: object) -> dict[str, object]:
        """F1.5: bounded cleanup of an accepted ExecuteTrajectory goal.

        Attempts cancellation on the exact handle and waits bounded for the
        result.  The cleanup outcome is recorded without ever claiming cancel
        success unless the exact F1.2 cancel-response contract was met.
        """
        if goal_handle is None:
            return {"cleanup": "none", "cleanup_status": "no-handle"}
        goal_uuid = self._normalize_goal_id(goal_handle) or ""
        cancel = self._cancel_execute_goal(
            goal_handle, expected_goal_uuid=goal_uuid, timeout_s=timeout_s
        )
        result_status = None
        try:
            result_future = goal_handle.get_result_async()
            result_deadline = time.monotonic() + float(timeout_s)
            while not result_future.done() and time.monotonic() < result_deadline:
                self._spin_once()
            if result_future.done():
                result_status = getattr(result_future.result(), "status", None)
        except Exception:
            result_status = None
        return {
            "cleanup": cancel.get("response"),
            "cleanup_return_code": cancel.get("return_code"),
            "cleanup_goals_canceling": list(cancel.get("goals_canceling") or []),
            "cleanup_result_status": result_status,
            "cleanup_error": cancel.get("error"),
        }

    def _wait_execute_result_status(
        self, goal_handle: Any, timeout_s: object
    ) -> tuple[object, str | None]:
        """Bounded wait for the ExecuteTrajectory result; return (status, string)."""
        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            return None, f"get_result_async raised: {exc}"
        deadline = time.monotonic() + float(timeout_s)
        while not result_future.done() and time.monotonic() < deadline:
            self._spin_once()
        if not result_future.done():
            return None, "result future did not resolve within the bounded wait"
        try:
            response = result_future.result()
        except Exception as exc:
            return None, f"result future raised: {exc}"
        status = getattr(response, "status", None)
        try:
            return status, _execute_status_name(status)
        except ValueError:
            return status, None

    def _classify_plan_only_result(
        self,
        scenario_id: str,
        result: Any,
        spec: Mapping[str, object],
    ) -> dict[str, object]:
        result_object = getattr(result, "result", None)
        error_code = getattr(result_object, "error_code", None)
        error_value = getattr(error_code, "val", None) if error_code is not None else None
        if not _strict_int(error_value):
            return {
                "scenario_id": scenario_id,
                "status": "malformed-result",
                "reason_code": "malformed-result",
                "diagnostic_only": True,
                "error": "MoveGroup result error_code.val is not a strict integer",
            }
        planned = getattr(result_object, "planned_trajectory", None)
        points = (
            getattr(getattr(planned, "joint_trajectory", None), "points", None)
            if planned is not None
            else None
        )
        nonempty_plan = isinstance(points, (list, tuple)) and len(points) > 0
        trajectory_digest = (
            self._digest(self.ros["serialize_message"](planned)) if planned is not None else None
        )
        expectation = spec.get("expectation")
        success = bool(error_value == MOVEIT_SUCCESS_CODE)
        if expectation == "non-success":
            # F2.4/F3.4: the blocked scenario only passes on an explicit
            # planning-stage non-success after a valid request AND an empty
            # planned trajectory; unknown/request-level codes never pass, and an
            # allowlisted code with a contradictory non-empty trajectory is an
            # explicit contradiction, never a pass.
            if error_value == MOVEIT_SUCCESS_CODE:
                classification = "unexpected-success"
                passed = False
            elif error_value in MOVEIT_PLANNING_NON_SUCCESS_CODES:
                if nonempty_plan:
                    classification = "contradictory-nonempty-trajectory"
                    passed = False
                else:
                    classification = "planning-non-success"
                    passed = True
            else:
                classification = "request-level-or-unknown"
                passed = False
        else:
            passed = success and nonempty_plan
            classification = (
                "success"
                if passed
                else "success-with-empty-plan"
                if success
                else "non-success"
            )
        return {
            "scenario_id": scenario_id,
            "status": "diagnostic-pass" if passed else "diagnostic-fail",
            "diagnostic_only": True,
            "error_code": error_value,
            "error_code_classification": classification,
            "nonempty_plan": nonempty_plan,
            "trajectory_digest": trajectory_digest,
            "expectation": expectation,
        }

    # -- Gate C entry point ---------------------------------------------------

    def run_gate_c_plan_only(
        self, scenario_id: str, *, joints: Sequence[float] | None = None
    ) -> dict[str, object]:
        """Run exactly one plan-only Gate C scenario through ``/move_action``.

        Fail-dominant (F2.1): the authoritative final status is computed after
        the plan outcome *and* every required evidence finalization step.  Any
        readiness, journal event, graph projection, journal finalization, artifact
        serialization/write, or required-artifact-existence failure makes the
        public return and ``integrated-execution.json`` status ``evidence-invalid``;
        no artifact retains a pass claim for that attempt.  The raw planner
        outcome is preserved separately as ``planner_status``.

        Exceptional completion (F2.2): server wait, goal construction/serialization,
        ``send_goal_async``, send-future spin/result, goal acceptance, result-future
        spin/result, cancellation, provider calls, and artifact finalization are all
        converted into finite canonical diagnostic records with zero physical claim
        and exact zero-command/controller flags.  Once ``fixture-ready`` exists the
        executor always attempts teardown journal completion and failed finalization.
        No expected runtime/DDS/action failure escapes the public API.
        """
        fixture_ready_recorded = False
        try:
            try:
                spec = stage_c_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid(
                    scenario_id, "scenario-rejected", [str(exc)]
                )

            if self.join_key_provider is None:
                return self._evidence_invalid(
                    scenario_id,
                    "no-join-key",
                    ["join_key_provider is required before sending any goal"],
                )
            readiness = self._readiness()
            if readiness is None:
                return self._evidence_invalid(
                    scenario_id,
                    "readiness-unavailable",
                    ["readiness_snapshot_provider is required before sending any goal"],
                )
            if not readiness["ready"]:
                return self._evidence_invalid(
                    scenario_id, "readiness-failed", list(readiness["reasons"])
                )

            # F2.5: bounded self-spin to obtain a current fixture scene.
            acquire_error = self._acquire_scene(scenario_id)
            if acquire_error is not None:
                return acquire_error

            join = self._join_key()
            if join is None:
                return self._evidence_invalid(
                    scenario_id,
                    "no-join-key",
                    ["join_key_provider returned no valid strictly-increasing key"],
                )
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid(
                    scenario_id,
                    "no-planning-scene",
                    ["no valid PlanningScene cached before fixture-ready"],
                )
            # F2.6: fixture-ready must match the declared fixture contract.
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid(
                    scenario_id, "fixture-scene-mismatch", [scene_error]
                )

            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid(
                    scenario_id, "journal-fixture-ready-rejected", [str(exc)]
                )
            fixture_ready_recorded = True

            # F2.7: `before` visual request is durably flushed before the goal send.
            self._append_visual_request("before", scenario_id, spec)

            try:
                goal = self._build_goal(spec, joints=joints)
                goal_digest = self._digest(self.ros["serialize_message"](goal))
            except Exception as exc:
                goal = None
                goal_digest = None
                outcome = {
                    "scenario_id": scenario_id,
                    "status": "goal-construction-exception",
                    "reason_code": "goal-construction-exception",
                    "diagnostic_only": True,
                    "error": f"goal construction/serialization raised: {exc}",
                }
            else:
                outcome = self._send_plan_only_goal(scenario_id, goal, spec)

            teardown_status = "not-recorded"
            later_join = self._join_key()
            if later_join is None:
                teardown_status = "no-join-key"
            else:
                try:
                    self.journal.snapshot(
                        "teardown", frame_index=later_join[0], timestamp=later_join[1]
                    )
                    teardown_status = "recorded"
                except (ValueError, TypeError) as exc:
                    teardown_status = f"rejected: {exc}"

            # F2.7: `after` visual request only in the post-transaction phase.
            self._append_visual_request("after", scenario_id, spec)

            # F2.1: authoritative fail-dominant final status after the plan outcome
            # and every evidence finalization step.
            planner_status = outcome.get("status")
            final_status = (
                "diagnostic-pass"
                if planner_status == "diagnostic-pass"
                else "diagnostic-fail"
                if planner_status == "diagnostic-fail"
                else "evidence-invalid"
            )
            graph_status = "unavailable"
            journal_finalize_error: str | None = None
            projection = None
            try:
                graph = self._graph_observation()
                if graph is None:
                    raise ValueError("observed graph evidence is unavailable")
                projection = build_journal_graph_projection(
                    fixture_payload=self._fixture_payload_for_graph(),
                    observed_graph=graph,
                )
                # F3.1: validate the graph BEFORE any artifact write (no durable
                # output yet), so a pass is never provisionally persisted before
                # the graph evidence is known to be valid.
                self.journal.finalize(final_status, graph=projection, json_path=None)
                graph_status = "validated"
            except Exception as exc:
                journal_finalize_error = str(exc)
                graph_status = f"invalid: {exc}"
                final_status = "evidence-invalid"
                # F2.1: always produce planning-scene.json as a canonical failure
                # artifact when journal finalization cannot validate the graph.
                if self.journal.record_count > 0:
                    self._finalize_failure_artifact(journal_finalize_error, graph_status)

            record = {
                **outcome,
                "planner_status": planner_status,
                "teardown": teardown_status,
                "graph": graph_status,
                "goal_digest": goal_digest,
                "diagnostic_only": True,
                "execute_trajectory_goal_sent": False,
                "isaac_joint_commands_published": False,
            }
            # F2.1: the fail-dominant status is authoritative in the public record.
            record["status"] = final_status
            if final_status == "evidence-invalid" and record.get("reason_code") is None:
                record["reason_code"] = (
                    "graph-evidence-invalid"
                    if journal_finalize_error is not None
                    else "evidence-invalid"
                )

            # F3.1: transactional finalization/write order.  All non-journal
            # artifacts are made durable first; the successful final journal
            # artifact (planning-scene.json) is deferred until every other
            # required artifact is durable.  If ANY required write fails after a
            # provisional planner/journal pass, every already-created
            # status-bearing artifact is downgraded to evidence-invalid (atomic
            # summaries rewritten, corrective ``row_kind="final"`` rows appended
            # to the JSONL lifecycle streams, and planning-scene.json written as
            # a failure artifact), so no persisted artifact retains a pass.
            try:
                self._write_artifacts(scenario_id, spec, goal, record, readiness, graph_status)
                if graph_status == "validated":
                    self.journal.finalize(
                        final_status,
                        graph=projection,
                        json_path=self.attempt_dir / "planning-scene.json",
                    )
            except Exception as exc:
                downgraded_from = record.get("status")
                record["status"] = "evidence-invalid"
                record["reason_code"] = "artifact-write-failed"
                record["artifact_error"] = str(exc)
                self._downgrade_persisted_evidence(
                    scenario_id,
                    record,
                    readiness,
                    graph_status,
                    planner_status,
                    downgraded_from=downgraded_from,
                    goal_kind=spec.get("kind"),
                )
            return record
        except Exception as exc:  # F2.2: no expected runtime failure escapes the API.
            if fixture_ready_recorded:
                return self._evidence_invalid_after_fixture_ready(
                    scenario_id, exc, spec, readiness
                )
            return self._evidence_invalid(
                scenario_id, "unexpected-exception", [str(exc)]
            )

    # -- Gate D ---------------------------------------------------------------
    #
    # D-stage scenarios reuse Task 4's required attempt paths and transactional
    # fail-dominant mechanics, with a separate D-stage record shape (never
    # changing Gate-C bytes).  Every D attempt stays ``diagnostic_only=true``
    # and ``isaac_joint_commands_published=false``; physical verdicts remain
    # verifier-owned.

    def _evidence_invalid_d(
        self,
        scenario_id: str,
        reason_code: str,
        reasons: Sequence[str],
        *,
        handler: str | None = None,
    ) -> dict[str, object]:
        """D-stage evidence-invalid record with the D schema and durable rows.

        F2.3: the D handler label is carried when known (scene-acquisition
        failures route through ``_acquire_scene(d_handler=...)``) so the record
        and the durable ``event=gate-d`` row identify the D handler, never a
        Gate-C shape.
        """
        record: dict[str, object] = {
            "scenario_id": scenario_id,
            "stage": "D",
            "diagnostic_only": True,
            "physical_verdict": None,
            "status": "evidence-invalid",
            "reason_code": reason_code,
            "reasons": list(reasons),
            "execute_trajectory_goal_sent": False,
            "controller_goal_sent": False,
            "isaac_joint_commands_published": False,
        }
        if handler is not None:
            record["handler"] = handler
        try:
            row: dict[str, object] = {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "event": "gate-d",
                "stage": "D",
                "status": "evidence-invalid",
                "reason_code": reason_code,
                "reasons": list(reasons),
                "diagnostic_only": True,
                "execute_trajectory_goal_sent": False,
                "controller_goal_sent": False,
                "isaac_joint_commands_published": False,
                "timestamp": float(time.monotonic()),
            }
            if handler is not None:
                row["handler"] = handler
            self._append_jsonl(
                self.attempt_dir / "integrated-execution.jsonl",
                row,
            )
        except Exception:
            pass
        try:
            self._write_json_atomic(
                self.attempt_dir / "integrated-execution.json",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "stage": "D",
                    "diagnostic_only": True,
                    "physical_verdict": None,
                    "status": "evidence-invalid",
                    "reason_code": reason_code,
                    "reasons": list(reasons),
                    "execute_trajectory_goal_sent": False,
                    "controller_goal_sent": False,
                    "isaac_joint_commands_published": False,
                },
            )
        except Exception:
            pass
        # Fail-dominant D journal: when the attempt recorded fixture-ready (the
        # journal holds records), emit the canonical failure planning-scene.json.
        # A pre-fixture-ready failure leaves the journal empty; finalize_failure
        # rejects that and the helper swallows it, matching Gate C's no-scene
        # behavior.
        self._d_journal_failure(reason=reason_code, graph_diagnosis="D diagnostic journal")
        return record

    def _d_journal_failure(self, *, reason: str, graph_diagnosis: str) -> str:
        """Write the canonical D failure planning-scene.json, returning its status."""
        try:
            self.journal.finalize_failure(
                reason=reason,
                graph_diagnosis=graph_diagnosis,
                json_path=self.attempt_dir / "planning-scene.json",
            )
            return "written"
        except Exception as exc:
            return f"failed: {exc}"

    def _normalize_goal_id(self, goal_handle: Any) -> str | None:
        return _normalize_goal_uuid(getattr(goal_handle, "goal_id", None))

    def _build_d_goal(self, spec: Mapping[str, object], joints: Sequence[float] | None):
        kind = spec.get("kind")
        if kind == "execute-joint":
            target = (
                list(joints)
                if joints is not None
                else list(spec.get("joints", Q_OUTBOUND))
            )
            return build_joint_move_group_goal(target, plan_only=True)
        pose = self._pose_stamped_from_spec(spec)
        return build_pose_move_group_goal(pose, plan_only=True)

    def _send_plan_only_retaining_handle(
        self,
        scenario_id: str,
        goal: Any,
        spec: Mapping[str, object],
    ) -> dict[str, object]:
        """Send exactly one plan-only MoveGroup goal retaining handle/UUID/plan.

        Mirrors the Gate-C ``_send_plan_only_goal`` bounded lifecycle but keeps
        the accepted goal handle, its normalized lowercase planning UUID, and the
        complete generated ``planned_trajectory`` message for the split path.
        """
        thresholds = self._thresholds()
        client = self._action_clients["/move_action"]
        server_timeout_s = float(thresholds.get("action_server_wait_s", 5.0))
        if not self._wait_for_server(client, server_timeout_s):
            return {
                "scenario_id": scenario_id,
                "status": "action-server-unavailable",
                "reason_code": "action-server-unavailable",
                "diagnostic_only": True,
                "error": "/move_action server was not available before send",
            }
        accept_timeout_s = float(thresholds.get("goal_accept_timeout_s", 5.0))
        send_future = client.send_goal_async(goal)
        accept_deadline = time.monotonic() + accept_timeout_s
        while not send_future.done() and time.monotonic() < accept_deadline:
            self._spin_once()
        if not send_future.done():
            try:
                send_future.cancel()
            except Exception:
                pass
            return {
                "scenario_id": scenario_id,
                "status": "goal-accept-timeout",
                "reason_code": "goal-accept-timeout",
                "diagnostic_only": True,
                "error": "goal acceptance timed out before a goal handle existed",
            }
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            return {
                "scenario_id": scenario_id,
                "status": "goal-send-exception",
                "reason_code": "goal-send-exception",
                "diagnostic_only": True,
                "error": f"send_goal future raised: {exc}",
            }
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            return {
                "scenario_id": scenario_id,
                "status": "goal-rejected",
                "reason_code": "goal-rejected",
                "diagnostic_only": True,
                "error": "send_goal returned no accepted goal handle",
            }
        planning_goal_id = self._normalize_goal_id(goal_handle)
        result_timeout_s = float(thresholds.get("plan_result_timeout_s", 10.0))
        result_future = goal_handle.get_result_async()
        result_deadline = time.monotonic() + result_timeout_s
        while not result_future.done() and time.monotonic() < result_deadline:
            self._spin_once()
        if not result_future.done():
            cancel_response = self._cancel_goal(goal_handle)
            return {
                "scenario_id": scenario_id,
                "status": "timeout",
                "reason_code": "result-timeout",
                "diagnostic_only": True,
                "error": "planning result timed out",
                "cancel_response": cancel_response,
                "planning_goal_id": planning_goal_id,
                "goal_handle": goal_handle,
            }
        try:
            result = result_future.result()
        except Exception as exc:
            return {
                "scenario_id": scenario_id,
                "status": "malformed-result",
                "reason_code": "malformed-result",
                "diagnostic_only": True,
                "error": f"result future raised: {exc}",
                "planning_goal_id": planning_goal_id,
                "goal_handle": goal_handle,
            }
        if result is None or getattr(result, "result", None) is None:
            return {
                "scenario_id": scenario_id,
                "status": "malformed-result",
                "reason_code": "malformed-result",
                "diagnostic_only": True,
                "error": "result future returned no MoveGroup result",
                "planning_goal_id": planning_goal_id,
                "goal_handle": goal_handle,
            }
        planned = getattr(getattr(result, "result", None), "planned_trajectory", None)
        outcome = self._classify_plan_only_result(scenario_id, result, spec)
        outcome = dict(outcome)
        outcome["planning_goal_id"] = planning_goal_id
        outcome["goal_handle"] = goal_handle
        outcome["planned_trajectory"] = planned
        return outcome

    def _send_execute_trajectory(
        self,
        scenario_id: str,
        goal: Any,
        *,
        result_timeout_s: float,
        cancel_timeout_s: float,
        allow_cancel: bool = False,
    ) -> dict[str, object]:
        """Send exactly one ExecuteTrajectory goal and wait bounded for result.

        Returns the distinct execution UUID, the terminal action status int and
        string, and the accepted goal handle.  ``allow_cancel`` permits the
        caller to cancel the ExecuteTrajectory handle instead of timing out; a
        cancellation is never proof of controller-side quiescence by itself.
        """
        thresholds = self._thresholds()
        client = self._action_clients["/execute_trajectory"]
        server_timeout_s = float(thresholds.get("action_server_wait_s", 5.0))
        if not self._wait_for_server(client, server_timeout_s):
            return {
                "scenario_id": scenario_id,
                "status": "execute-server-unavailable",
                "reason_code": "execute-server-unavailable",
                "diagnostic_only": True,
                "error": "/execute_trajectory server was not available before send",
            }
        accept_timeout_s = float(thresholds.get("goal_accept_timeout_s", 5.0))
        send_future = client.send_goal_async(goal)
        accept_deadline = time.monotonic() + accept_timeout_s
        while not send_future.done() and time.monotonic() < accept_deadline:
            self._spin_once()
        if not send_future.done():
            try:
                send_future.cancel()
            except Exception:
                pass
            return {
                "scenario_id": scenario_id,
                "status": "execute-goal-accept-timeout",
                "reason_code": "execute-goal-accept-timeout",
                "diagnostic_only": True,
                "error": "execute goal acceptance timed out before a goal handle existed",
            }
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            return {
                "scenario_id": scenario_id,
                "status": "execute-goal-send-exception",
                "reason_code": "execute-goal-send-exception",
                "diagnostic_only": True,
                "error": f"execute send_goal future raised: {exc}",
            }
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            return {
                "scenario_id": scenario_id,
                "status": "execute-goal-rejected",
                "reason_code": "execute-goal-rejected",
                "diagnostic_only": True,
                "error": "execute send_goal returned no accepted goal handle",
            }
        execute_goal_id = self._normalize_goal_id(goal_handle)
        result_future = goal_handle.get_result_async()
        result_deadline = time.monotonic() + float(result_timeout_s)
        while not result_future.done() and time.monotonic() < result_deadline:
            self._spin_once()
        if not result_future.done():
            if allow_cancel:
                cancel_response = self._cancel_goal(goal_handle)
                return {
                    "scenario_id": scenario_id,
                    "status": "execute-cancellation-issued",
                    "reason_code": "execute-cancellation-issued",
                    "diagnostic_only": True,
                    "execute_goal_id": execute_goal_id,
                    "goal_handle": goal_handle,
                    "cancel_response": cancel_response,
                }
            return {
                "scenario_id": scenario_id,
                "status": "execute-result-timeout",
                "reason_code": "execute-result-timeout",
                "diagnostic_only": True,
                "execute_goal_id": execute_goal_id,
                "goal_handle": goal_handle,
            }
        try:
            result = result_future.result()
        except Exception as exc:
            return {
                "scenario_id": scenario_id,
                "status": "execute-malformed-result",
                "reason_code": "execute-malformed-result",
                "diagnostic_only": True,
                "error": f"execute result future raised: {exc}",
                "execute_goal_id": execute_goal_id,
                "goal_handle": goal_handle,
            }
        action_status = getattr(result, "status", None)
        try:
            status_string = _execute_status_name(action_status)
        except ValueError:
            status_string = None
        return {
            "scenario_id": scenario_id,
            "status": "execute-terminal",
            "reason_code": "execute-terminal",
            "execute_goal_id": execute_goal_id,
            "goal_handle": goal_handle,
            "execute_result_status": action_status,
            "execute_result_status_string": status_string,
            "diagnostic_only": True,
        }

    def _bind_and_call_fjt_provider(
        self, provider: Any, entry: Mapping[str, object]
    ) -> object:
        """Bind *provider* to the single-captured *entry* and call it.

        F5.1: the driver's production FJT provider exposes a ``bind(entry)``
        method so a run method can bind it to the exact terminal status entry
        captured by ``_wait_for_fjt_status`` before calling it.  Offline
        providers without ``bind`` are called directly; the validator then
        checks their evidence against the captured entry, never against a
        re-read of the mutable cache.  Binding is best-effort: a provider that
        does not support late binding is still called so its evidence is checked
        against the captured entry (fail-closed on mismatch).
        """
        bind = getattr(provider, "bind", None)
        if callable(bind):
            bind(dict(entry))
        return provider()

    def _validate_fjt_evidence(
        self,
        provider_evidence: object,
        *,
        expected_trajectory_digest: str | None,
        baseline: Mapping[str, object] | None = None,
        expected_fjt_goal_uuid: str | None = None,
        expected_fjt_entry: Mapping[str, object] | None = None,
    ) -> tuple[bool, str | None]:
        """Validate injected FJT transaction evidence and join it to status.

        The provider must return real observed controller-transaction evidence
        (endpoint exactly ``/xarm7_traj_controller/follow_joint_trajectory``,
        normalized FJT goal UUID, canonical trajectory digest equal to the
        unchanged ExecuteTrajectory digest, a finite timestamp/sequence, and a
        real observation source).

        F4.3: when *expected_fjt_goal_uuid* is supplied the provider UUID must
        equal that discovered controller goal UUID (never the ExecuteTrajectory
        UUID).

        F5.1: when *expected_fjt_entry* is supplied (the exact entry captured by
        ``_wait_for_fjt_status``), the provider UUID/status/sequence/timestamp
        must equal that captured entry EXACTLY and the validator MUST NOT
        re-query ``_fjt_status_cache`` for the transaction.  A second status
        emission for the same UUID between capture and validation therefore can
        never switch the transaction or race it to ``evidence-invalid``; any
        mismatch fails closed.  When *expected_fjt_entry* is None the legacy
        cache-join path is used (the newest joined cache entry must carry the
        provider status/sequence/timestamp exactly).  Missing, stale,
        mismatched, extra, malformed, or provider-exception evidence makes the
        attempt ``evidence-invalid``.
        """
        if not isinstance(provider_evidence, Mapping):
            return False, "fjt_transaction_provider returned a non-mapping"
        if provider_evidence.get("endpoint") != FJT_ENDPOINT:
            return False, f"fjt evidence endpoint must be {FJT_ENDPOINT}"
        goal_uuid = provider_evidence.get("goal_uuid")
        if not _valid_goal_uuid(goal_uuid):
            return False, "fjt evidence goal_uuid is not a valid 16-byte hex UUID"
        if expected_fjt_goal_uuid is not None and goal_uuid != expected_fjt_goal_uuid:
            return False, (
                "fjt evidence goal_uuid does not equal the discovered controller "
                "goal UUID"
            )
        digest = provider_evidence.get("trajectory_digest")
        if not _valid_digest(digest):
            return False, "fjt evidence trajectory digest must be a valid nonzero digest"
        if expected_trajectory_digest is not None and digest != expected_trajectory_digest:
            return False, "fjt evidence trajectory digest does not match the ExecuteTrajectory trajectory"
        source = provider_evidence.get("source")
        if not isinstance(source, str) or not source:
            return False, "fjt evidence source must identify real controller introspection"
        sequence = provider_evidence.get("sequence")
        timestamp = provider_evidence.get("timestamp")
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            return False, "fjt evidence sequence must be a non-negative integer"
        if not _finite_number(timestamp):
            return False, "fjt evidence timestamp must be finite"
        provider_status = provider_evidence.get("status")
        if isinstance(provider_status, bool) or not isinstance(provider_status, int):
            return False, "fjt evidence status must be an integer"
        # F5.1: single-capture path — validate against the exact captured entry,
        # never the mutable cache.
        if expected_fjt_entry is not None:
            if goal_uuid != expected_fjt_entry.get("goal_uuid"):
                return False, (
                    "fjt evidence goal_uuid does not equal the captured controller "
                    "goal UUID"
                )
            if provider_status != expected_fjt_entry.get("status"):
                return False, (
                    "fjt evidence status does not join the captured status entry"
                )
            if sequence != expected_fjt_entry.get("seq"):
                return False, (
                    "fjt evidence sequence does not join the captured status entry"
                )
            if timestamp != expected_fjt_entry.get("received_mono"):
                return False, (
                    "fjt evidence timestamp does not join the captured status entry"
                )
            return True, None
        candidates = self._fjt_status_cache
        if baseline is not None:
            candidates = self._fresh_fjt_entries(baseline)
        goal_candidates = [
            entry for entry in candidates if entry.get("goal_uuid") == goal_uuid
        ]
        if not goal_candidates:
            return False, (
                "fjt evidence goal_uuid does not join to any status entry in the "
                "current window"
            )
        newest = goal_candidates[-1]
        if newest.get("status") != provider_status:
            return False, "fjt evidence status does not join to the newest joined status entry"
        if newest.get("seq") != sequence:
            return False, "fjt evidence sequence does not join to the newest joined status entry"
        if newest.get("received_mono") != timestamp:
            return False, "fjt evidence timestamp does not join to the newest joined status entry"
        return True, None

    def _journal_snapshot_d(self, event: str) -> str:
        """Append one D diagnostic journal snapshot, returning the join status."""
        later_join = self._join_key()
        if later_join is None:
            return "no-join-key"
        try:
            self.journal.snapshot(
                event, frame_index=later_join[0], timestamp=later_join[1]
            )
            return "recorded"
        except (ValueError, TypeError) as exc:
            return f"rejected: {exc}"

    def run_execute_sequence(
        self,
        scenario_id: str,
        *,
        joints: Sequence[float] | None = None,
        fjt_transaction_provider: Callable[[], Mapping[str, object]] | None = None,
    ) -> dict[str, object]:
        """Run one Stage-D split-path joint/pose execution.

        Mandatory MoveGroup plan-only -> ExecuteTrajectory boundary: exactly one
        plan-only ``/move_action`` goal, a required non-empty ``planned_trajectory``,
        exactly one ``/execute_trajectory`` goal whose trajectory is byte-identical
        to the planned one, distinct valid 16-byte plan/execute UUIDs, and FJT
        transaction evidence joining to the real status topic.
        """
        start_wall = time.monotonic()
        fixture_ready_recorded = False
        event_log: list[str] = []
        try:
            try:
                spec = stage_d_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "scenario-rejected", [str(exc)])
            if spec["kind"] not in ("execute-joint", "execute-pose"):
                return self._evidence_invalid_d(
                    scenario_id, "wrong-handler", [f"D handler is {spec['kind']!r}, not execute"]
                )
            if fjt_transaction_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-fjt-provider",
                    ["fjt_transaction_provider is required before sending any goal"],
                )
            if self.join_key_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-join-key",
                    ["join_key_provider is required before sending any goal"],
                )
            readiness = self._readiness()
            if readiness is None:
                return self._evidence_invalid_d(
                    scenario_id, "readiness-unavailable",
                    ["readiness_snapshot_provider is required before sending any goal"],
                )
            if not readiness["ready"]:
                return self._evidence_invalid_d(scenario_id, "readiness-failed", list(readiness["reasons"]))
            acquire_error = self._acquire_scene(scenario_id, d_handler=spec.get("kind"))
            if acquire_error is not None:
                return acquire_error
            join = self._join_key()
            if join is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-join-key", ["join_key_provider returned no valid key"]
                )
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid_d(scenario_id, "no-planning-scene", [])
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid_d(scenario_id, "fixture-scene-mismatch", [scene_error])
            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid_d(scenario_id, "journal-fixture-ready-rejected", [str(exc)])
            fixture_ready_recorded = True
            event_log.append("fixture-ready")

            try:
                goal = self._build_d_goal(spec, joints=joints)
            except Exception as exc:
                return self._evidence_invalid_d(
                    scenario_id, "goal-construction-exception", [str(exc)]
                )
            plan_outcome = self._send_plan_only_retaining_handle(scenario_id, goal, spec)
            planner_status = plan_outcome.get("status")
            planning_goal_id = plan_outcome.get("planning_goal_id")
            planned = plan_outcome.get("planned_trajectory")
            if planner_status != "diagnostic-pass":
                final_status = (
                    "diagnostic-fail"
                    if planner_status == "diagnostic-fail"
                    else "evidence-invalid"
                )
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, final_status,
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                )
            if planned is None:
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "diagnostic-fail",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                )
            # F1.4: capture the FJT/status and joint-state stream positions at
            # execution start so every later observation is windowed, fresh, and
            # bounded to the current attempt.
            baseline = self._d_baseline()
            # F1.8/Md5: D visual capture before the first D goal, with real
            # chronology (never a retroactive request).
            self._append_visual_request("before", scenario_id, spec, kind="gate-d-diagnostic")
            event_log.append("execution-start")
            snap = self._journal_snapshot_d("execution-start")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"execution-start journal snapshot rejected: {snap}",
                    journal_issues=[snap],
                )

            planned_digest_before = self._digest(self.ros["serialize_message"](planned))
            try:
                exec_goal = build_execute_trajectory_goal(planned)
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"execute goal construction raised: {exc}",
                )
            executed_digest_after = self._digest(
                self.ros["serialize_message"](exec_goal.trajectory)
            )
            if executed_digest_after != planned_digest_before:
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error="ExecuteTrajectory trajectory digest differs from the planned trajectory",
                )
            execute_timeout_s = float(self._thresholds().get("execute_timeout_s", 120.0))
            cancel_timeout_s = float(self._thresholds().get("cancel_timeout_s", 10.0))
            exec_outcome = self._send_execute_trajectory(
                scenario_id, exec_goal,
                result_timeout_s=execute_timeout_s,
                cancel_timeout_s=cancel_timeout_s,
            )
            execute_goal_id = exec_outcome.get("execute_goal_id")
            execute_result_status = exec_outcome.get("execute_result_status")
            execute_handle = exec_outcome.get("goal_handle")
            if (
                not _valid_goal_uuid(planning_goal_id)
                or not _valid_goal_uuid(execute_goal_id)
                or planning_goal_id == execute_goal_id
            ):
                # F2.4: an accepted ExecuteTrajectory handle must be cleaned up
                # (bounded cancel) before rejecting its UUID evidence, so an
                # accepted goal is never left running on an identity failure.
                uuid_cleanup = None
                if execute_handle is not None and getattr(execute_handle, "accepted", False):
                    uuid_cleanup = self._cleanup_execute_goal(
                        execute_handle, timeout_s=cancel_timeout_s
                    )
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_outcome=exec_outcome,
                    execute_error="plan/execute UUIDs must both be valid and distinct",
                    cleanup=uuid_cleanup,
                )
            if execute_result_status != EXECUTE_STATUS_SUCCEEDED:
                cleanup = self._cleanup_execute_goal(execute_handle, timeout_s=cancel_timeout_s)
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "diagnostic-fail",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_outcome=exec_outcome,
                    execute_error="ExecuteTrajectory did not terminate SUCCEEDED",
                    cleanup=cleanup,
                )
            # F1.4/M3: bounded spin for the joined fresh FJT status entry so a
            # live status-topic arrival cannot race the result future.
            # F4.3: the controller FJT goal UUID is distinct from the
            # ExecuteTrajectory UUID (MoveIt forwards to the controller, which
            # creates its own FJT action goal).  Discover the unique new
            # controller goal UUID in the current window; never key FJT status
            # on ``execute_goal_id``.
            fjt_wait_s = self._threshold_timeout("fjt_wait_timeout_s", 10.0)
            joined = self._wait_for_fjt_status(
                None, (execute_result_status,), fjt_wait_s, baseline=baseline
            )
            if joined is None:
                discovery_reason = (
                    f"no new controller FJT goal reached the executed terminal status "
                    f"within the bounded wait"
                )
                if self._last_fjt_discovery_error:
                    discovery_reason = f"{discovery_reason}; {self._last_fjt_discovery_error}"
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_outcome=exec_outcome,
                    execute_error=discovery_reason,
                )
            fjt_goal_id = str(joined.get("goal_uuid"))
            try:
                # F5.1: bind the provider to the exact captured terminal entry
                # before calling it; a second status emission for the same UUID
                # cannot switch the transaction.
                provider_evidence = self._bind_and_call_fjt_provider(
                    fjt_transaction_provider, joined
                )
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_outcome=exec_outcome,
                    execute_error=f"fjt_transaction_provider raised: {exc}",
                )
            fjt_ok, fjt_reason = self._validate_fjt_evidence(
                provider_evidence,
                expected_trajectory_digest=executed_digest_after,
                baseline=baseline,
                expected_fjt_goal_uuid=fjt_goal_id,
                expected_fjt_entry=joined,
            )
            if not fjt_ok:
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_outcome=exec_outcome,
                    execute_error=fjt_reason or "fjt evidence invalid",
                )
            self._append_visual_request("after", scenario_id, spec, kind="gate-d-diagnostic")
            event_log.append("execution-terminal")
            snap = self._journal_snapshot_d("execution-terminal")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_outcome=exec_outcome,
                    execute_error=f"execution-terminal journal snapshot rejected: {snap}",
                    journal_issues=[snap],
                )
            event_log.append("teardown")
            snap = self._journal_snapshot_d("teardown")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_outcome=exec_outcome,
                    execute_error=f"teardown journal snapshot rejected: {snap}",
                    journal_issues=[snap],
                )

            return self._finalize_d_attempt(
                scenario_id, spec, plan_outcome, planner_status, "diagnostic-pass",
                readiness, start_wall, event_log=event_log,
                planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                execute_goal_id=execute_goal_id,
                execute_outcome=exec_outcome,
                fjt_evidence=provider_evidence,
                fjt_goal_id=fjt_goal_id,
                trajectory_digest=executed_digest_after,
                controller_endpoint=FJT_ENDPOINT,
            )
        except Exception as exc:
            if fixture_ready_recorded:
                return self._evidence_invalid_d(
                    scenario_id, "unexpected-exception", [str(exc)]
                )
            return self._evidence_invalid_d(
                scenario_id, "unexpected-exception", [str(exc)]
            )

    def run_cancel_sequence(
        self,
        scenario_id: str,
        *,
        long_motion_provider: Callable[[], Mapping[str, object]] | None = None,
        fjt_transaction_provider: Callable[[], Mapping[str, object]] | None = None,
        planning_goal_id: str | None = None,
        execute_goal_id: str | None = None,
        timeout_s: float | None = None,
        execute_goal_handle: Any = None,
        fjt_goal_id: str | None = None,
        transaction_baseline: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        """Run the Stage-D cancellation contract.

        Requires an explicit validated long-motion target (via
        *long_motion_provider* or the supplied plan/execute goal ids); missing
        target evidence fails closed before goal send.  When the orchestrator or
        focused test supplies the live ``/execute_trajectory``
        ``ClientGoalHandle`` via *execute_goal_handle*, ``cancel_goal_async()``
        is called on exactly that handle (never the completed MoveGroup planning
        handle).  Requires terminal CANCELED (5), requires controller
        quiescence, and never sends a later stage.

        F4.4: the controller FJT transaction is keyed on the distinct
        controller goal UUID (*fjt_goal_id*) and windowed to the supplied
        pre-send *transaction_baseline* when the live driver presends a long
        motion.  FJT status is never keyed on *execute_goal_id*.  A direct
        offline path may supply an explicit seeded *fjt_goal_id* (distinct from
        *execute_goal_id*); missing *fjt_goal_id* fails closed before cancel.
        """
        start_wall = time.monotonic()
        fixture_ready_recorded = False
        event_log: list[str] = []
        try:
            try:
                spec = stage_d_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "scenario-rejected", [str(exc)])
            if spec["kind"] != "cancel":
                return self._evidence_invalid_d(
                    scenario_id, "wrong-handler", [f"D handler is {spec['kind']!r}, not cancel"]
                )
            if (
                long_motion_provider is None
                and not (_valid_goal_uuid(planning_goal_id) and _valid_goal_uuid(execute_goal_id))
            ):
                return self._evidence_invalid_d(
                    scenario_id, "no-cancel-target",
                    ["cancel requires an explicit validated long-motion target/provider"],
                )
            if fjt_transaction_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-fjt-provider",
                    ["fjt_transaction_provider is required before sending any goal"],
                )
            if self.join_key_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-join-key",
                    ["join_key_provider is required before sending any goal"],
                )
            readiness = self._readiness()
            if readiness is None or not readiness["ready"]:
                return self._evidence_invalid_d(
                    scenario_id,
                    "readiness-unavailable" if readiness is None else "readiness-failed",
                    [] if readiness is None else list(readiness["reasons"]),
                )
            acquire_error = self._acquire_scene(scenario_id, d_handler=spec.get("kind"))
            if acquire_error is not None:
                return acquire_error
            join = self._join_key()
            if join is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid_d(scenario_id, "no-planning-scene", [])
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid_d(scenario_id, "fixture-scene-mismatch", [scene_error])
            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid_d(scenario_id, "journal-fixture-ready-rejected", [str(exc)])
            fixture_ready_recorded = True
            event_log.append("fixture-ready")

            if long_motion_provider is not None:
                try:
                    target_evidence = long_motion_provider()
                except Exception as exc:
                    return self._evidence_invalid_d(
                        scenario_id, "cancel-target-provider-raised", [str(exc)]
                    )
                if not isinstance(target_evidence, Mapping):
                    return self._evidence_invalid_d(
                        scenario_id, "cancel-target-invalid", ["long_motion_provider returned a non-mapping"]
                    )
                supplied_plan = _normalize_goal_uuid(target_evidence.get("planning_goal_id"))
                supplied_exec = _normalize_goal_uuid(target_evidence.get("execute_goal_id"))
                if not (_valid_goal_uuid(supplied_plan) and _valid_goal_uuid(supplied_exec)):
                    return self._evidence_invalid_d(
                        scenario_id, "cancel-target-invalid",
                        ["long_motion_provider must supply valid plan and execute UUIDs"],
                    )
                planning_goal_id = supplied_plan
                execute_goal_id = supplied_exec
            if planning_goal_id == execute_goal_id:
                return self._evidence_invalid_d(
                    scenario_id, "cancel-target-invalid", ["plan and execute UUIDs must differ"]
                )

            # F4.4: the distinct controller FJT goal UUID is mandatory.  The live
            # driver path supplies it (discovered from the pre-send transaction);
            # a direct offline path must supply an explicit seeded controller UUID.
            # FJT status is never keyed on ``execute_goal_id``.
            if not _valid_goal_uuid(fjt_goal_id):
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="cancel requires the distinct controller FJT goal UUID",
                    goals_canceling=[execute_goal_id],
                )
            if fjt_goal_id == execute_goal_id:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="controller FJT goal UUID must differ from the ExecuteTrajectory UUID",
                    goals_canceling=[execute_goal_id],
                )

            # F1.4: window the current attempt.  F4.4: the live presend path
            # supplies the pre-send *transaction_baseline* (captured before the
            # ExecuteTrajectory goal was sent); a direct offline path captures a
            # fresh baseline from the current stream position.
            baseline = (
                dict(transaction_baseline)
                if transaction_baseline is not None
                else self._d_baseline()
            )
            # F1.8/Md5: visual capture before the first D goal.
            self._append_visual_request("before", scenario_id, spec, kind="gate-d-diagnostic")
            event_log.append("execution-start")
            snap = self._journal_snapshot_d("execution-start")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"execution-start journal snapshot rejected: {snap}",
                    journal_issues=[snap],
                )

            # F1.2: a cancel pass is impossible without the exact live
            # ExecuteTrajectory ClientGoalHandle.  Raw UUID kwargs or provider
            # strings never substitute for a handle, and the completed MoveGroup
            # planning handle is never canceled.
            if execute_goal_handle is None:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="cancel requires the exact live ExecuteTrajectory goal handle",
                    goals_canceling=[execute_goal_id],
                )
            handle_uuid = self._normalize_goal_id(execute_goal_handle)
            if handle_uuid != execute_goal_id:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=(
                        f"cancel handle goal_id {handle_uuid!r} does not equal the recorded "
                        f"execute_goal_id {execute_goal_id!r}"
                    ),
                    goals_canceling=[execute_goal_id],
                )
            if planning_goal_id == execute_goal_id:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="plan and execute UUIDs must differ",
                    goals_canceling=[execute_goal_id],
                )

            # F1.4/M4: the transaction must have actually started — FJT
            # EXECUTING(2) joined and at least one fresh current-attempt joint
            # frame proves arm motion above threshold.  A transaction that never
            # started moving cannot be interrupted.
            fjt_wait_s = self._threshold_timeout("fjt_wait_timeout_s", 10.0)
            motion_wait_s = self._threshold_timeout("motion_trigger_timeout_s", 10.0)
            motion_limit = float(self._thresholds().get("cancel_motion_velocity_rad_s", 0.005))
            if not self._wait_for_fjt_executing(fjt_goal_id, fjt_wait_s, baseline=baseline):
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="cancel motion trigger: FJT goal never reached EXECUTING within the bounded wait",
                    goals_canceling=[execute_goal_id],
                )
            if not self._wait_for_motion_trigger(
                motion_wait_s, baseline=baseline, threshold=motion_limit
            ):
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="cancel motion trigger: no fresh joint-state frame proved arm motion",
                    goals_canceling=[execute_goal_id],
                )

            # F1.2: call ``cancel_goal_async()`` exactly once on the exact
            # ExecuteTrajectory handle and require the accepted response shape
            # (return_code == ERROR_NONE and goals_canceling == [execute_goal_id]).
            cancel_timeout_s = float(
                timeout_s if timeout_s is not None
                else self._thresholds().get("cancel_timeout_s", 10.0)
            )
            cancel_response = self._cancel_execute_goal(
                execute_goal_handle, expected_goal_uuid=execute_goal_id, timeout_s=cancel_timeout_s
            )
            if cancel_response.get("response") != "accepted":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=(
                        cancel_response.get("error")
                        or f"cancel response was {cancel_response.get('response')!r}, not accepted"
                    ),
                    goals_canceling=[execute_goal_id],
                    cancel_response=cancel_response,
                )
            goals_canceling = list(cancel_response.get("goals_canceling") or [execute_goal_id])
            event_log.append("cancel-requested")
            snap = self._journal_snapshot_d("cancel-requested")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"cancel-requested journal snapshot rejected: {snap}",
                    journal_issues=[snap], goals_canceling=goals_canceling,
                    cancel_response=cancel_response,
                )

            # F1.2: require the ExecuteTrajectory action result terminal CANCELED (5).
            action_status, action_status_string = self._wait_execute_result_status(
                execute_goal_handle, cancel_timeout_s
            )
            if action_status != EXECUTE_STATUS_CANCELED:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=(
                        f"ExecuteTrajectory terminal status was {action_status_string!r} "
                        f"({action_status!r}); cancellation requires CANCELED (5)"
                    ),
                    goals_canceling=goals_canceling,
                    cancel_response=cancel_response,
                    execute_outcome={
                        "execute_result_status": action_status,
                        "execute_result_status_string": action_status_string,
                    },
                )
            # F1.2: require the joined FJT controller goal to reach CANCELED (5)
            # within a bounded wait (windowed to the current attempt).
            fjt_terminal = self._wait_for_fjt_status(
                fjt_goal_id, (EXECUTE_STATUS_CANCELED,), fjt_wait_s, baseline=baseline
            )
            if fjt_terminal is None:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="joined FJT controller goal never reached CANCELED within the bounded wait",
                    goals_canceling=goals_canceling,
                    cancel_response=cancel_response,
                    execute_outcome={
                        "execute_result_status": action_status,
                        "execute_result_status_string": action_status_string,
                    },
                )
            # F1.4: bounded quiescence — the canceled goal's newest fresh status
            # is terminal (no longer active).  Historical pre-terminal entries
            # (e.g. the EXECUTING trigger) are not treated as active.
            quiescent = self._wait_for(
                lambda: (lambda entries: bool(entries) and int(entries[-1].get("status", -1)) in (
                    EXECUTE_STATUS_SUCCEEDED, EXECUTE_STATUS_CANCELED, EXECUTE_STATUS_ABORTED
                ))(self._fresh_fjt_entries(baseline, fjt_goal_id)),
                fjt_wait_s,
            )
            if not quiescent:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="FJT controller goal was still active after the cancel result",
                    goals_canceling=goals_canceling,
                    cancel_response=cancel_response,
                    execute_outcome={
                        "execute_result_status": action_status,
                        "execute_result_status_string": action_status_string,
                    },
                )
            try:
                # F5.1: bind the provider to the exact captured CANCELED terminal
                # entry before calling it (single-capture evidence transaction).
                provider_evidence = self._bind_and_call_fjt_provider(
                    fjt_transaction_provider, fjt_terminal
                )
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"fjt_transaction_provider raised: {exc}",
                    goals_canceling=goals_canceling,
                    cancel_response=cancel_response,
                    execute_outcome={
                        "execute_result_status": action_status,
                        "execute_result_status_string": action_status_string,
                    },
                )
            fjt_ok, fjt_reason = self._validate_fjt_evidence(
                provider_evidence, expected_trajectory_digest=None, baseline=baseline,
                expected_fjt_goal_uuid=fjt_goal_id,
                expected_fjt_entry=fjt_terminal,
            )
            if not fjt_ok:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=fjt_reason or "fjt evidence invalid",
                    goals_canceling=goals_canceling,
                    cancel_response=cancel_response,
                    execute_outcome={
                        "execute_result_status": action_status,
                        "execute_result_status_string": action_status_string,
                    },
                )
            self._append_visual_request("after", scenario_id, spec, kind="gate-d-diagnostic")
            event_log.append("quiescent")
            snap = self._journal_snapshot_d("quiescent")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"quiescent journal snapshot rejected: {snap}",
                    journal_issues=[snap], goals_canceling=goals_canceling,
                    cancel_response=cancel_response,
                    execute_outcome={
                        "execute_result_status": action_status,
                        "execute_result_status_string": action_status_string,
                    },
                )
            event_log.append("teardown")
            snap = self._journal_snapshot_d("teardown")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"teardown journal snapshot rejected: {snap}",
                    journal_issues=[snap], goals_canceling=goals_canceling,
                    cancel_response=cancel_response,
                    execute_outcome={
                        "execute_result_status": action_status,
                        "execute_result_status_string": action_status_string,
                    },
                )
            return self._finalize_d_attempt(
                scenario_id, spec, None, None, "diagnostic-pass",
                readiness, start_wall, event_log=event_log,
                planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                execute_goal_id=execute_goal_id,
                fjt_goal_id=fjt_goal_id,
                goals_canceling=goals_canceling,
                terminal_status="canceled",
                fjt_evidence=provider_evidence,
                controller_goal_sent=True,
                controller_endpoint=FJT_ENDPOINT,
                plan_applicable=False,
                cancel_response=cancel_response,
                execute_outcome={
                    "execute_result_status": action_status,
                    "execute_result_status_string": action_status_string,
                },
            )
        except Exception as exc:
            return self._evidence_invalid_d(
                scenario_id, "unexpected-exception", [str(exc)]
            )

    def run_safety_sequence(
        self,
        scenario_id: str,
        *,
        long_motion_provider: Callable[[], Mapping[str, object]] | None = None,
        fjt_transaction_provider: Callable[[], Mapping[str, object]] | None = None,
        stop_timeout_s: float | None = None,
        fjt_goal_id: str | None = None,
        transaction_baseline: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        """Run the Stage-D safety interruption contract.

        Uses the executor's real ``/sim/safety/operator`` publisher and the
        ``/sim/hardware/safety_stop``/``/joint_states`` callbacks: publishes
        operator ``True``, waits bounded for safety-stop ``True``, requires the
        old ExecuteTrajectory terminal status ABORTED (6), requires
        ``safety_stop_frames`` consecutive fresh joint-state velocity frames
        with every arm-joint absolute velocity bounded, publishes operator
        ``False`` after the effective-stop predicate, and sends no replacement/
        resume goal.

        F4.4: the controller FJT transaction is keyed on the distinct
        controller goal UUID (*fjt_goal_id*) and windowed to the supplied
        pre-send *transaction_baseline* when the live driver presends a long
        motion.  FJT status is never keyed on *execute_goal_id*.  A direct
        offline path may supply an explicit seeded *fjt_goal_id* (distinct from
        *execute_goal_id*); missing *fjt_goal_id* fails closed before publish.
        """
        start_wall = time.monotonic()
        fixture_ready_recorded = False
        event_log: list[str] = []
        try:
            try:
                spec = stage_d_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "scenario-rejected", [str(exc)])
            if spec["kind"] != "safety":
                return self._evidence_invalid_d(
                    scenario_id, "wrong-handler", [f"D handler is {spec['kind']!r}, not safety"]
                )
            if long_motion_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-safety-target",
                    ["safety requires an explicit validated long-motion target/provider"],
                )
            if fjt_transaction_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-fjt-provider",
                    ["fjt_transaction_provider is required before sending any goal"],
                )
            if self.join_key_provider is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            readiness = self._readiness()
            if readiness is None or not readiness["ready"]:
                return self._evidence_invalid_d(
                    scenario_id,
                    "readiness-unavailable" if readiness is None else "readiness-failed",
                    [] if readiness is None else list(readiness["reasons"]),
                )
            acquire_error = self._acquire_scene(scenario_id, d_handler=spec.get("kind"))
            if acquire_error is not None:
                return acquire_error
            join = self._join_key()
            if join is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid_d(scenario_id, "no-planning-scene", [])
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid_d(scenario_id, "fixture-scene-mismatch", [scene_error])
            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid_d(scenario_id, "journal-fixture-ready-rejected", [str(exc)])
            fixture_ready_recorded = True
            event_log.append("fixture-ready")

            try:
                target_evidence = long_motion_provider()
            except Exception as exc:
                return self._evidence_invalid_d(scenario_id, "safety-target-provider-raised", [str(exc)])
            if not isinstance(target_evidence, Mapping):
                return self._evidence_invalid_d(scenario_id, "safety-target-invalid", [])
            planning_goal_id = _normalize_goal_uuid(target_evidence.get("planning_goal_id"))
            execute_goal_id = _normalize_goal_uuid(target_evidence.get("execute_goal_id"))
            if not (_valid_goal_uuid(planning_goal_id) and _valid_goal_uuid(execute_goal_id)):
                return self._evidence_invalid_d(
                    scenario_id, "safety-target-invalid",
                    ["safety target must supply valid plan and execute UUIDs"],
                )
            if planning_goal_id == execute_goal_id:
                return self._evidence_invalid_d(
                    scenario_id, "safety-target-invalid", ["plan and execute UUIDs must differ"]
                )

            # F4.4: the distinct controller FJT goal UUID is mandatory.  The live
            # driver path supplies it (discovered from the pre-send transaction);
            # a direct offline path must supply an explicit seeded controller UUID.
            # FJT status is never keyed on ``execute_goal_id``.
            if not _valid_goal_uuid(fjt_goal_id):
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="safety requires the distinct controller FJT goal UUID",
                )
            if fjt_goal_id == execute_goal_id:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="controller FJT goal UUID must differ from the ExecuteTrajectory UUID",
                )

            # F1.4: window the current attempt.  F4.4: the live presend path
            # supplies the pre-send *transaction_baseline* (captured before the
            # ExecuteTrajectory goal was sent); a direct offline path captures a
            # fresh baseline from the current stream position.
            baseline = (
                dict(transaction_baseline)
                if transaction_baseline is not None
                else self._d_baseline()
            )
            # F1.8/Md5: visual capture before the first D goal.
            self._append_visual_request("before", scenario_id, spec, kind="gate-d-diagnostic")
            event_log.append("execution-start")
            snap = self._journal_snapshot_d("execution-start")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"execution-start journal snapshot rejected: {snap}",
                    journal_issues=[snap],
                )

            # F1.4/F1.3: the safety interruption must target a transaction that
            # actually started — FJT EXECUTING(2) joined and a fresh joint frame
            # proves arm motion above threshold.
            fjt_wait_s = self._threshold_timeout("fjt_wait_timeout_s", 10.0)
            motion_wait_s = self._threshold_timeout("motion_trigger_timeout_s", 10.0)
            motion_limit = float(self._thresholds().get("safety_motion_velocity_rad_s", 0.005))
            if not self._wait_for_fjt_executing(fjt_goal_id, fjt_wait_s, baseline=baseline):
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="safety motion trigger: FJT goal never reached EXECUTING within the bounded wait",
                )
            if not self._wait_for_motion_trigger(
                motion_wait_s, baseline=baseline, threshold=motion_limit
            ):
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="safety motion trigger: no fresh joint-state frame proved arm motion",
                )

            # 3. publish operator True.
            try:
                self.publish_operator(True)
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"operator publish failed: {exc}",
                )
            # 4. wait bounded for safety-stop True.
            safety_stop_seen = self._wait_for_safety_stop(stop_timeout_s)
            if not safety_stop_seen:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="safety-stop True was not observed within the bounded wait",
                )
            event_log.append("effective-stop")
            snap = self._journal_snapshot_d("effective-stop")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"effective-stop journal snapshot rejected: {snap}",
                    journal_issues=[snap],
                )

            # F1.3: the old ExecuteTrajectory/FJT transaction must reach ABORTED
            # (6) after the safety assertion.  The provider evidence and its join
            # are mandatory; a provider exception/mismatch/stale-cache fails
            # closed (never swallowed, never an unrelated cached ABORTED goal).
            aborted_entry = self._wait_for_fjt_status(
                fjt_goal_id, (EXECUTE_STATUS_ABORTED,), fjt_wait_s, baseline=baseline
            )
            if aborted_entry is None:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="joined FJT controller goal never reached ABORTED after the safety assertion",
                )
            try:
                # F5.1: bind the provider to the exact captured ABORTED terminal
                # entry before calling it (single-capture evidence transaction).
                provider_evidence = self._bind_and_call_fjt_provider(
                    fjt_transaction_provider, aborted_entry
                )
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"safety fjt_transaction_provider raised: {exc}",
                )
            fjt_ok, fjt_reason = self._validate_fjt_evidence(
                provider_evidence, expected_trajectory_digest=None, baseline=baseline,
                expected_fjt_goal_uuid=fjt_goal_id,
                expected_fjt_entry=aborted_entry,
            )
            if not fjt_ok:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=fjt_reason or "safety fjt evidence invalid",
                    fjt_evidence=provider_evidence,
                )
            provider_status = provider_evidence.get("status")
            if provider_status != EXECUTE_STATUS_ABORTED:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=(
                        f"safety provider status {provider_status!r} is not ABORTED (6); "
                        "safety terminal evidence must join the joined fresh ABORTED entry"
                    ),
                    fjt_evidence=provider_evidence,
                )

            # 6. safety_stop_frames consecutive fresh post-stop joint-state
            # velocity frames, each received after the safety assertion.
            stop_baseline = self._d_baseline()
            velocity_limit = float(self._thresholds().get("safety_stop_velocity_rad_s", 0.02))
            required_frames = int(self._thresholds().get("safety_stop_frames", 5))
            stop_wait_s = float(self._thresholds().get("safety_stop_frames_wait_s", 1.0))
            stopped = self._wait_for_stopped_frames(
                required_frames, stop_wait_s,
                baseline=stop_baseline, velocity_limit=velocity_limit,
            )
            if len(stopped) < required_frames:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=(
                        f"safety required {required_frames} consecutive fresh bounded "
                        f"joint-state frames; observed {len(stopped)}"
                    ),
                    fjt_evidence=provider_evidence,
                )

            # 7. publish operator False only after the effective-stop predicate.
            try:
                self.publish_operator(False)
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"operator clear publish failed: {exc}",
                    fjt_evidence=provider_evidence,
                )
            event_log.append("operator-clear")
            snap = self._journal_snapshot_d("operator-clear")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"operator-clear journal snapshot rejected: {snap}",
                    journal_issues=[snap], fjt_evidence=provider_evidence,
                )

            # F1.4/M2: bounded post-clear stability — no fresh goal UUID, all
            # velocities bounded, and every arm-joint position within
            # ``safety_position_creep_rad`` of the clear-time baseline.
            clear_baseline = self._d_baseline()
            latest_clear = self._latest_fresh_joint_frame(clear_baseline)
            if latest_clear is None:
                latest_clear = self._latest_fresh_joint_frame(baseline)
            clear_positions = list(latest_clear.get("positions") or []) if latest_clear is not None else []
            if len(clear_positions) != 7:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="safety could not capture a clear-time joint-position baseline",
                    fjt_evidence=provider_evidence,
                )
            clear_baseline = dict(clear_baseline)
            clear_baseline["clear_positions"] = clear_positions
            creep_limit = float(self._thresholds().get("safety_position_creep_rad", 0.005))
            stability_wait_s = float(self._thresholds().get("safety_stability_wait_s", 1.0))
            stability = self._wait_for_post_clear_stability(
                stability_wait_s,
                baseline=clear_baseline,
                known_goal_id=fjt_goal_id,
                velocity_limit=velocity_limit,
                creep_limit=creep_limit,
            )
            if not stability.get("stable"):
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"post-clear stability not met: {stability.get('reason')}",
                    fjt_evidence=provider_evidence,
                )
            self._append_visual_request("after", scenario_id, spec, kind="gate-d-diagnostic")
            event_log.append("quiescent")
            snap = self._journal_snapshot_d("quiescent")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"quiescent journal snapshot rejected: {snap}",
                    journal_issues=[snap], fjt_evidence=provider_evidence,
                )
            event_log.append("teardown")
            snap = self._journal_snapshot_d("teardown")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"teardown journal snapshot rejected: {snap}",
                    journal_issues=[snap], fjt_evidence=provider_evidence,
                )
            return self._finalize_d_attempt(
                scenario_id, spec, None, None, "diagnostic-pass",
                readiness, start_wall, event_log=event_log,
                planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                execute_goal_id=execute_goal_id,
                fjt_goal_id=fjt_goal_id,
                terminal_status="aborted",
                fjt_evidence=provider_evidence,
                controller_goal_sent=True,
                controller_endpoint=FJT_ENDPOINT,
                plan_applicable=False,
                env_cloud_evidence=None,
            )
        except Exception as exc:
            return self._evidence_invalid_d(
                scenario_id, "unexpected-exception", [str(exc)]
            )

    def _wait_for_safety_stop(self, stop_timeout_s: float | None) -> bool:
        """Bounded spin until the newest safety-stop sample reads True."""
        timeout_s = float(
            stop_timeout_s if stop_timeout_s is not None
            else self._thresholds().get("safety_stop_wait_s", 0.25)
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            sample = self._latest_safety_stop
            if sample is not None and bool(getattr(sample, "data", False)) is True:
                return True
            self._spin_once()
        return False

    def run_cartesian_retreat(
        self,
        scenario_id: str,
        *,
        current_tcp_pose_provider: Callable[[], Mapping[str, object]] | None = None,
        environment_cloud_provider: Callable[[], object] | None = None,
    ) -> dict[str, object]:
        """Run the Stage-D Cartesian retreat contract.

        Uses an injected ``current_tcp_pose_provider`` (never a TF listener
        embedded in this task) returning a fresh finite normalized ``base_link``
        TCP pose with observation identity/age.  F1.7: an explicit fresh
        ``environment_cloud_provider`` must return a real non-empty finite
        ``base_link`` ``sensor_msgs/msg/PointCloud2`` observation; that exact
        cloud is passed into ``CartesianMove.Goal.env_points`` and only then is
        ``collision_checking`` recorded true.  Derives the deterministic ``+Z``
        ``RETREAT_DISTANCE_M`` target preserving orientation and sends one
        collision-aware ``/cartesian_move_action`` goal.
        """
        start_wall = time.monotonic()
        fixture_ready_recorded = False
        event_log: list[str] = []
        try:
            try:
                spec = stage_d_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "scenario-rejected", [str(exc)])
            if spec["kind"] != "retreat":
                return self._evidence_invalid_d(
                    scenario_id, "wrong-handler", [f"D handler is {spec['kind']!r}, not retreat"]
                )
            if current_tcp_pose_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-tcp-pose-provider",
                    ["current_tcp_pose_provider is required before sending a retreat goal"],
                )
            if environment_cloud_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-environment-cloud-provider",
                    ["environment_cloud_provider is required before sending a collision-aware retreat goal"],
                )
            if self.join_key_provider is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            readiness = self._readiness()
            if readiness is None or not readiness["ready"]:
                return self._evidence_invalid_d(
                    scenario_id,
                    "readiness-unavailable" if readiness is None else "readiness-failed",
                    [] if readiness is None else list(readiness["reasons"]),
                )
            acquire_error = self._acquire_scene(scenario_id, d_handler=spec.get("kind"))
            if acquire_error is not None:
                return acquire_error
            join = self._join_key()
            if join is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid_d(scenario_id, "no-planning-scene", [])
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid_d(scenario_id, "fixture-scene-mismatch", [scene_error])
            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid_d(scenario_id, "journal-fixture-ready-rejected", [str(exc)])
            fixture_ready_recorded = True
            event_log.append("fixture-ready")

            try:
                source = current_tcp_pose_provider()
            except Exception as exc:
                return self._evidence_invalid_d(scenario_id, "tcp-pose-provider-raised", [str(exc)])
            if not isinstance(source, Mapping):
                return self._evidence_invalid_d(scenario_id, "tcp-pose-invalid", ["provider returned a non-mapping"])
            identity = source.get("identity")
            age = source.get("age_s")
            if not (isinstance(identity, (int, str)) and str(identity)):
                return self._evidence_invalid_d(scenario_id, "tcp-pose-invalid", ["provider must supply observation identity"])
            if not _fresh(age, self._thresholds().get("tf_fresh_s", 0.25)):
                return self._evidence_invalid_d(scenario_id, "tcp-pose-stale", ["provider pose is stale"])
            try:
                target = derive_retreat_target_pose(
                    source, distance_m=RETREAT_DISTANCE_M, axis=RETREAT_AXIS
                )
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "retreat-derivation-failed", [str(exc)])

            # F1.7: require a real observed non-empty environment cloud.  A
            # missing, empty, malformed, stale, wrong-frame, provider-exception,
            # or serialization-failure cloud fails closed before goal send; no
            # cloud is ever fabricated in live code.
            try:
                cloud = environment_cloud_provider()
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"environment_cloud_provider raised: {exc}",
                    plan_applicable=False, controller_goal_sent=False,
                    controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                )
            try:
                env_cloud_evidence = self._env_cloud_evidence(cloud)
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"environment cloud invalid: {exc}",
                    plan_applicable=False, controller_goal_sent=False,
                    controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                )

            # F1.8/Md5: visual capture before the first D goal.
            self._append_visual_request("before", scenario_id, spec, kind="gate-d-diagnostic")
            event_log.append("retreat-start")
            snap = self._journal_snapshot_d("retreat-start")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"retreat-start journal snapshot rejected: {snap}",
                    journal_issues=[snap], plan_applicable=False,
                    controller_goal_sent=False, controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                )
            from geometry_msgs.msg import Pose

            target_pose = Pose()
            target_pose.position.x = float(target["xyz"][0])
            target_pose.position.y = float(target["xyz"][1])
            target_pose.position.z = float(target["xyz"][2])
            target_pose.orientation.x = float(target["quaternion_xyzw"][0])
            target_pose.orientation.y = float(target["quaternion_xyzw"][1])
            target_pose.orientation.z = float(target["quaternion_xyzw"][2])
            target_pose.orientation.w = float(target["quaternion_xyzw"][3])
            try:
                cartesian_goal = build_cartesian_move_goal(target_pose, env_points=cloud)
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"cartesian goal construction failed: {exc}",
                    plan_applicable=False, controller_goal_sent=False,
                    controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                    env_cloud_evidence=env_cloud_evidence,
                )

            client = self._action_clients["/cartesian_move_action"]
            server_timeout_s = float(self._thresholds().get("action_server_wait_s", 5.0))
            if not self._wait_for_server(client, server_timeout_s):
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error="cartesian server unavailable before send",
                    plan_applicable=False, controller_goal_sent=False,
                    controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                    env_cloud_evidence=env_cloud_evidence,
                )
            accept_timeout_s = float(self._thresholds().get("goal_accept_timeout_s", 5.0))
            send_future = client.send_goal_async(cartesian_goal)
            accept_deadline = time.monotonic() + accept_timeout_s
            while not send_future.done() and time.monotonic() < accept_deadline:
                self._spin_once()
            if not send_future.done():
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error="cartesian goal acceptance timed out",
                    plan_applicable=False, controller_goal_sent=False,
                    controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                    env_cloud_evidence=env_cloud_evidence,
                )
            try:
                goal_handle = send_future.result()
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"cartesian send future raised: {exc}",
                    plan_applicable=False, controller_goal_sent=False,
                    controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                    env_cloud_evidence=env_cloud_evidence,
                )
            if goal_handle is None or not getattr(goal_handle, "accepted", False):
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error="cartesian goal was rejected",
                    plan_applicable=False, controller_goal_sent=False,
                    controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                    env_cloud_evidence=env_cloud_evidence,
                )
            retreat_goal_id = self._normalize_goal_id(goal_handle)
            result_timeout_s = float(self._thresholds().get("execute_timeout_s", 120.0))
            result_future = goal_handle.get_result_async()
            result_deadline = time.monotonic() + result_timeout_s
            while not result_future.done() and time.monotonic() < result_deadline:
                self._spin_once()
            if not result_future.done():
                # F1.5: an accepted goal must be cleaned up on timeout.
                cleanup = self._cleanup_execute_goal(goal_handle, timeout_s=result_timeout_s)
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error="cartesian result timed out",
                    plan_applicable=False, controller_goal_sent=False,
                    controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                    env_cloud_evidence=env_cloud_evidence,
                    cleanup=cleanup,
                )
            try:
                result = result_future.result()
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"cartesian result future raised: {exc}",
                    plan_applicable=False, controller_goal_sent=False,
                    controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                    env_cloud_evidence=env_cloud_evidence,
                )
            action_status = getattr(result, "status", None)
            try:
                status_string = _execute_status_name(action_status)
            except ValueError:
                status_string = None
            if action_status != EXECUTE_STATUS_SUCCEEDED:
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"cartesian action did not succeed: {status_string} ({action_status})",
                    plan_applicable=False, controller_goal_sent=False,
                    controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                    env_cloud_evidence=env_cloud_evidence,
                )

            self._append_visual_request("after", scenario_id, spec, kind="gate-d-diagnostic")
            event_log.append("retreat-terminal")
            snap = self._journal_snapshot_d("retreat-terminal")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"retreat-terminal journal snapshot rejected: {snap}",
                    journal_issues=[snap], plan_applicable=False,
                    controller_goal_sent=False, controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                    env_cloud_evidence=env_cloud_evidence,
                )
            event_log.append("teardown")
            snap = self._journal_snapshot_d("teardown")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"teardown journal snapshot rejected: {snap}",
                    journal_issues=[snap], plan_applicable=False,
                    controller_goal_sent=False, controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                    env_cloud_evidence=env_cloud_evidence,
                )
            return self._finalize_d_attempt(
                scenario_id, spec, None, None, "diagnostic-pass",
                readiness, start_wall, event_log=event_log,
                planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                plan_applicable=False,
                controller_goal_sent=False,
                controller_endpoint=CARTESIAN_MOVE_ENDPOINT,
                env_cloud_evidence=env_cloud_evidence,
                retreat_source=source,
                retreat_target=target,
                retreat_goal_id=retreat_goal_id,
            )
        except Exception as exc:
            return self._evidence_invalid_d(scenario_id, "unexpected-exception", [str(exc)])

    def _env_cloud_evidence(self, cloud: object) -> dict[str, object]:
        """F1.7: validate an observed PointCloud2 and return its evidence dict.

        Requires a real non-empty finite ``base_link`` PointCloud2 whose
        serialization succeeds.  Raises ``ValueError`` on missing/empty/
        malformed/stale/wrong-frame/unsupported serialization.
        """
        if cloud is None:
            raise ValueError("environment cloud provider returned None")
        header = getattr(cloud, "header", None)
        frame_id = getattr(header, "frame_id", None)
        if frame_id != "base_link":
            raise ValueError(f"environment cloud frame_id must be base_link, got {frame_id!r}")
        width = getattr(cloud, "width", 0)
        height = getattr(cloud, "height", 0)
        try:
            width = int(width)
            height = int(height)
        except (TypeError, ValueError):
            raise ValueError("environment cloud width/height must be integers")
        if width < 1 or height < 1:
            raise ValueError(f"environment cloud must be non-empty (width={width}, height={height})")
        data = getattr(cloud, "data", None)
        if data is None:
            raise ValueError("environment cloud data must be a non-empty byte payload")
        if isinstance(data, (bytes, bytearray)):
            data_bytes = bytes(data)
        elif hasattr(data, "tobytes"):
            data_bytes = bytes(data.tobytes())
        else:
            data_bytes = bytes(data)
        if len(data_bytes) < 1:
            raise ValueError("environment cloud data must be a non-empty byte payload")
        point_step = getattr(cloud, "point_step", 0)
        row_step = getattr(cloud, "row_step", 0)
        try:
            point_step = int(point_step)
            row_step = int(row_step)
        except (TypeError, ValueError):
            raise ValueError("environment cloud point_step/row_step must be integers")
        if point_step < 1 or row_step < 1:
            raise ValueError("environment cloud point_step/row_step must be positive")
        # F2.5: the buffer must be structurally self-consistent — every row is
        # at least width*point_step bytes wide (allowing valid row padding) and
        # the byte payload is exactly row_step*height bytes (rejecting both
        # truncated and oversized buffers, so a mid-stream trailing-scan frame
        # is never misread as a complete cloud).
        if row_step < width * point_step:
            raise ValueError(
                f"environment cloud row_step {row_step} must be >= width*point_step "
                f"({width}*{point_step}={width * point_step})"
            )
        if len(data_bytes) != row_step * height:
            raise ValueError(
                f"environment cloud data length {len(data_bytes)} must equal "
                f"row_step*height ({row_step}*{height}={row_step * height})"
            )
        # F2.5: when point fields are advertised, they must expose a usable
        # x/y/z FLOAT32 layout; an unadvertised field list (or an absent list)
        # is consumed as opaque bytes for the digest/structural evidence only.
        point_layout: str | dict[str, object]
        fields = getattr(cloud, "fields", None)
        if fields is None or len(fields) == 0:
            point_layout = "opaque-bytes"
        else:
            from sensor_msgs.msg import PointField

            x_offset = y_offset = z_offset = None
            layout_ok = True
            for field in fields:
                name = getattr(field, "name", None)
                if name not in ("x", "y", "z"):
                    continue
                dtype = getattr(field, "datatype", None)
                count = int(getattr(field, "count", 0))
                if int(dtype) != int(PointField.FLOAT32) or count != 1:
                    layout_ok = False
                offset = getattr(field, "offset", None)
                if name == "x":
                    x_offset = offset
                elif name == "y":
                    y_offset = offset
                else:  # z
                    z_offset = offset
            if not (layout_ok and x_offset is not None and y_offset is not None and z_offset is not None):
                raise ValueError(
                    "environment cloud fields must expose a usable x/y/z FLOAT32 layout"
                )
            point_layout = {
                "x_offset": x_offset,
                "y_offset": y_offset,
                "z_offset": z_offset,
                "datatype": "float32",
                "count": 1,
            }
        try:
            serialized = self.ros["serialize_message"](cloud)
        except Exception as exc:
            raise ValueError(f"environment cloud serialization failed: {exc}")
        return {
            "digest": self._digest(serialized),
            "source": "observed-environment-cloud",
            "frame_id": frame_id,
            "width": width,
            "height": height,
            "points": int(width * height),
            "bytes": len(data_bytes),
            "point_step": point_step,
            "row_step": row_step,
            "point_layout": point_layout,
        }

    def run_gripper_sequence(
        self,
        scenario_id: str,
        *,
        open_first: bool = True,
        gripper_open_position: float = GRIPPER_OPEN_POSITION,
        gripper_close_position: float = GRIPPER_CLOSE_POSITION,
        gripper_max_effort: float = GRIPPER_MAX_EFFORT,
    ) -> dict[str, object]:
        """Run the Stage-D native gripper contract.

        Sends open then close (or the reversed order) ``GripperCommand`` goals
        sequentially to ``/xarm_gripper/gripper_action`` with separate bounded
        acceptance/result deadlines; requires action success for each.  Routes
        through the fail-dominant D finalization so every attempt writes the
        complete authoritative artifact set (F1.6/M2).
        """
        start_wall = time.monotonic()
        fixture_ready_recorded = False
        event_log: list[str] = []
        try:
            try:
                spec = stage_d_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "scenario-rejected", [str(exc)])
            if spec["kind"] != "gripper":
                return self._evidence_invalid_d(
                    scenario_id, "wrong-handler", [f"D handler is {spec['kind']!r}, not gripper"]
                )
            if self.join_key_provider is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            readiness = self._readiness()
            if readiness is None or not readiness["ready"]:
                return self._evidence_invalid_d(
                    scenario_id,
                    "readiness-unavailable" if readiness is None else "readiness-failed",
                    [] if readiness is None else list(readiness["reasons"]),
                )
            acquire_error = self._acquire_scene(scenario_id, d_handler=spec.get("kind"))
            if acquire_error is not None:
                return acquire_error
            join = self._join_key()
            if join is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid_d(scenario_id, "no-planning-scene", [])
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid_d(scenario_id, "fixture-scene-mismatch", [scene_error])
            # F2.2: a close-first attempt needs the close-first journal contract.
            # The journal is rebuilt only while still fresh (zero records); an
            # in-progress journal is never mutated and the attempt fails closed.
            if not open_first:
                rebuild = self._rebuild_gripper_journal_close_first()
                if rebuild != "rebuilt":
                    return self._evidence_invalid_d(
                        scenario_id, "journal-order-rebuild-refused", [rebuild]
                    )
            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid_d(scenario_id, "journal-fixture-ready-rejected", [str(exc)])
            fixture_ready_recorded = True
            event_log.append("fixture-ready")

            commands = (
                [("open", gripper_open_position), ("close", gripper_close_position)]
                if open_first
                else [("close", gripper_close_position), ("open", gripper_open_position)]
            )
            # F1.8/Md5: visual capture before the first D goal.
            self._append_visual_request("before", scenario_id, spec, kind="gate-d-diagnostic")
            client = self._action_clients["/xarm_gripper/gripper_action"]
            server_timeout_s = float(self._thresholds().get("action_server_wait_s", 5.0))
            if not self._wait_for_server(client, server_timeout_s):
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error="gripper server unavailable before send",
                    plan_applicable=False, controller_goal_sent=False,
                    controller_endpoint=GRIPPER_ENDPOINT,
                    gripper_command_records=[], native_action=True, open_first=open_first,
                )
            accept_timeout_s = float(self._thresholds().get("goal_accept_timeout_s", 5.0))
            result_timeout_s = float(self._thresholds().get("execute_timeout_s", 120.0))
            command_records: list[dict[str, object]] = []
            goal_uuids: list[object] = []
            for name, position in commands:
                try:
                    gripper_goal = build_gripper_goal(position, max_effort=gripper_max_effort)
                except Exception as exc:
                    return self._finalize_d_attempt(
                        scenario_id, spec, None, None, "evidence-invalid",
                        readiness, start_wall, event_log=event_log,
                        planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                        execute_error=f"gripper goal construction failed: {exc}",
                        plan_applicable=False, controller_goal_sent=False,
                        controller_endpoint=GRIPPER_ENDPOINT,
                        gripper_command_records=command_records, native_action=True,
                        open_first=open_first,
                    )
                send_future = client.send_goal_async(gripper_goal)
                accept_deadline = time.monotonic() + accept_timeout_s
                while not send_future.done() and time.monotonic() < accept_deadline:
                    self._spin_once()
                if not send_future.done():
                    return self._finalize_d_attempt(
                        scenario_id, spec, None, None, "evidence-invalid",
                        readiness, start_wall, event_log=event_log,
                        planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                        execute_error="gripper goal acceptance timed out",
                        plan_applicable=False, controller_goal_sent=False,
                        controller_endpoint=GRIPPER_ENDPOINT,
                        gripper_command_records=command_records, native_action=True,
                        open_first=open_first,
                    )
                try:
                    goal_handle = send_future.result()
                except Exception as exc:
                    return self._finalize_d_attempt(
                        scenario_id, spec, None, None, "evidence-invalid",
                        readiness, start_wall, event_log=event_log,
                        planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                        execute_error=f"gripper send future raised: {exc}",
                        plan_applicable=False, controller_goal_sent=False,
                        controller_endpoint=GRIPPER_ENDPOINT,
                        gripper_command_records=command_records, native_action=True,
                        open_first=open_first,
                    )
                if goal_handle is None or not getattr(goal_handle, "accepted", False):
                    return self._finalize_d_attempt(
                        scenario_id, spec, None, None, "evidence-invalid",
                        readiness, start_wall, event_log=event_log,
                        planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                        execute_error="gripper goal was rejected",
                        plan_applicable=False, controller_goal_sent=False,
                        controller_endpoint=GRIPPER_ENDPOINT,
                        gripper_command_records=command_records, native_action=True,
                        open_first=open_first,
                    )
                gripper_goal_id = self._normalize_goal_id(goal_handle)
                goal_uuids.append(gripper_goal_id)
                result_future = goal_handle.get_result_async()
                result_deadline = time.monotonic() + result_timeout_s
                while not result_future.done() and time.monotonic() < result_deadline:
                    self._spin_once()
                if not result_future.done():
                    # F1.5: an accepted goal must be cleaned up on timeout.
                    cleanup = self._cleanup_execute_goal(goal_handle, timeout_s=result_timeout_s)
                    return self._finalize_d_attempt(
                        scenario_id, spec, None, None, "evidence-invalid",
                        readiness, start_wall, event_log=event_log,
                        planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                        execute_error=f"gripper {name} result timed out",
                        plan_applicable=False, controller_goal_sent=False,
                        controller_endpoint=GRIPPER_ENDPOINT,
                        gripper_command_records=command_records, native_action=True,
                        open_first=open_first, cleanup=cleanup,
                    )
                try:
                    result = result_future.result()
                except Exception as exc:
                    return self._finalize_d_attempt(
                        scenario_id, spec, None, None, "evidence-invalid",
                        readiness, start_wall, event_log=event_log,
                        planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                        execute_error=f"gripper result future raised: {exc}",
                        plan_applicable=False, controller_goal_sent=False,
                        controller_endpoint=GRIPPER_ENDPOINT,
                        gripper_command_records=command_records, native_action=True,
                        open_first=open_first,
                    )
                action_status = getattr(result, "status", None)
                try:
                    status_string = _execute_status_name(action_status)
                except ValueError:
                    status_string = None
                if action_status != EXECUTE_STATUS_SUCCEEDED:
                    return self._finalize_d_attempt(
                        scenario_id, spec, None, None, "evidence-invalid",
                        readiness, start_wall, event_log=event_log,
                        planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                        execute_error=f"gripper {name} did not succeed: {status_string} ({action_status})",
                        plan_applicable=False, controller_goal_sent=False,
                        controller_endpoint=GRIPPER_ENDPOINT,
                        gripper_command_records=command_records, native_action=True,
                        open_first=open_first,
                    )
                command_records.append(
                    {
                        "command": name,
                        "position": position,
                        "max_effort": gripper_max_effort,
                        "goal_id": gripper_goal_id,
                        "status": action_status,
                        "status_string": status_string,
                    }
                )
                event_log.append(f"gripper-{name}-terminal")
                snap = self._journal_snapshot_d(f"gripper-{name}-terminal")
                if snap != "recorded":
                    return self._finalize_d_attempt(
                        scenario_id, spec, None, None, "evidence-invalid",
                        readiness, start_wall, event_log=event_log,
                        planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                        execute_error=f"gripper-{name}-terminal journal snapshot rejected: {snap}",
                        journal_issues=[snap], plan_applicable=False,
                        controller_goal_sent=False, controller_endpoint=GRIPPER_ENDPOINT,
                        gripper_command_records=command_records, native_action=True,
                        open_first=open_first,
                    )

            self._append_visual_request("after", scenario_id, spec, kind="gate-d-diagnostic")
            event_log.append("teardown")
            snap = self._journal_snapshot_d("teardown")
            if snap != "recorded":
                return self._finalize_d_attempt(
                    scenario_id, spec, None, None, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"teardown journal snapshot rejected: {snap}",
                    journal_issues=[snap], plan_applicable=False,
                    controller_goal_sent=False, controller_endpoint=GRIPPER_ENDPOINT,
                    gripper_command_records=command_records, native_action=True,
                    open_first=open_first,
                )
            return self._finalize_d_attempt(
                scenario_id, spec, None, None, "diagnostic-pass",
                readiness, start_wall, event_log=event_log,
                planning_goal_id=None, fixture_ready_recorded=fixture_ready_recorded,
                plan_applicable=False,
                controller_goal_sent=False,
                controller_endpoint=GRIPPER_ENDPOINT,
                gripper_command_records=command_records,
                native_action=True,
                open_first=open_first,
            )
        except Exception as exc:
            return self._evidence_invalid_d(scenario_id, "unexpected-exception", [str(exc)])

    # -- Task 6 / Gate E fixed-target Pick and Place -------------------------

    def _e_reset_attempt_state(self) -> None:
        """F1.7: reset every per-attempt Gate-E sample/latch/goal state.

        Called at the start of every public E entry point so a reused executor
        can never satisfy a trigger with a previous attempt's TCP sample, FJT/
        joint receipt, native gripper count, or accepted goal handle.
        """
        self._tcp_pose_samples.clear()
        self._last_tcp_pose_provider = None
        self._e_active_goal_handle = None
        self._e_goal_state = {
            "pick_sent": False,
            "pick_goal_id": None,
            "place_sent": False,
            "place_goal_id": None,
        }
        self._e_native_gripper_count_provider = None
        self._e_native_gripper_count_baseline = None
        self._e_post_grasp_lift_m_provider = None
        self._e_post_grasp_lift_m_observed = None
        # F3.1/F3.3: per-attempt observation seams.  ``_e_lift_latch_mono`` is the
        # monotonic instant the Gate-E lift-complete latch fired (a test barrier
        # waits on it before injecting the transport FJT, replacing fixed wall-
        # clock timer races).  ``_e_observed_fjt_trigger`` is the first captured
        # FJT-based trigger (approach/transport/place) so the unexpected-
        # exception path can derive controller traffic ONLY from actual observed
        # FJT evidence (F3.3), never from task-goal cleanup.
        self._e_lift_latch_mono = None
        self._e_observed_fjt_trigger = None

    def _e_native_gripper_count(self) -> int | None:
        """Fresh receipt-sequenced native gripper action-goal count, else None.

        F1.10: the injected provider reports the native gripper action-server
        goal count (a live-observable seam, not the fake-only ``sent_goals``
        attribute).  Missing/stale/non-finite/provider-error evidence returns
        ``None`` so cancel-approach fails closed.
        """
        provider = self._e_native_gripper_count_provider
        if provider is None:
            return None
        try:
            sample = provider()
        except Exception:
            return None
        if not isinstance(sample, Mapping):
            return None
        count = sample.get("count")
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            return None
        age = sample.get("age_s")
        if not _fresh(age, self._thresholds().get("tf_fresh_s", 0.25)):
            return None
        return int(count)

    def _e_unexpected_exception(
        self, scenario_id: str, exc: Exception, *, spec: Mapping[str, object]
    ) -> dict[str, object]:
        """F1.9/F2.6: fail-closed unexpected-exception evidence with cleanup.

        Any exception escaping an E runner is contained into a complete Gate-E
        record.  If an action goal was accepted, bounded cleanup/cancel is
        attempted on the exact handle before the record is finalized; the
        cleanup outcome is recorded without ever claiming cancel success.

        F2.6: the accepted-goal truth (pick/place goal-sent flags, goal IDs,
        goals sent, cleanup outcome) is derived BEFORE any durable write, so
        ``_evidence_invalid_e`` persists the truthful rows into
        ``integrated-execution.jsonl``/``.json``, ``moveit-plans.jsonl``,
        ``controller-results.jsonl`` and the goal artifact.  No durable row may
        claim no goal was sent when one was accepted.

        F3.3: ``controller_goal_sent`` is derived ONLY from actual observed FJT
        evidence (``_e_observed_fjt_trigger``).  Accepting/canceling a task goal
        or attempting task-goal cleanup never proves a controller goal was sent;
        when no goal was accepted, cleanup is ``None`` so it cannot imply
        traffic.
        """
        cleanup: Mapping[str, object] | None = None
        goal_handle = self._e_active_goal_handle
        if goal_handle is not None:
            try:
                cleanup = self._cleanup_execute_goal(
                    goal_handle,
                    timeout_s=self._thresholds().get("cancel_timeout_s", 3.0),
                )
            except Exception as clean_exc:  # pragma: no cover - defensive
                cleanup = {"cleanup": "exception", "cleanup_error": str(clean_exc)}
            self._e_active_goal_handle = None
        goal_state = dict(self._e_goal_state)
        pick_goal_sent = bool(goal_state.get("pick_sent"))
        place_goal_sent = bool(goal_state.get("place_sent"))
        place_goal_accepted = bool(goal_state.get("place_sent"))
        goals_sent = int(pick_goal_sent) + int(place_goal_sent)
        observed_fjt = self._e_observed_fjt_trigger
        controller_goal_sent = bool(
            observed_fjt is not None and observed_fjt.get("goal_uuid") is not None
        )
        controller_goal_uuid = (
            observed_fjt.get("goal_uuid") if observed_fjt is not None else None
        )
        return self._evidence_invalid_e(
            scenario_id,
            "unexpected-exception",
            [f"{type(exc).__name__}: {exc}"],
            handler=spec.get("kind"),
            spec=spec,
            pick_goal_sent=pick_goal_sent,
            place_goal_sent=place_goal_sent,
            place_goal_accepted=place_goal_accepted,
            goals_sent=goals_sent,
            pick_goal_id=goal_state.get("pick_goal_id"),
            place_goal_id=goal_state.get("place_goal_id"),
            cleanup=cleanup,
            controller_goal_sent=controller_goal_sent,
            controller_goal_uuid=controller_goal_uuid,
        )

    def run_pick_place_sequence(
        self,
        scenario_id: str,
        *,
        current_tcp_pose_provider: Callable[[], Mapping[str, object]] | None = None,
        native_gripper_goal_count_provider: Callable[[], Mapping[str, object]] | None = None,
        post_grasp_lift_m_provider: Callable[[], Mapping[str, object]] | None = None,
    ) -> dict[str, object]:
        """Run one Gate-E fixed-target Pick/Place sequence (positive or negative).

        The scenario is validated fail-closed through :func:`stage_e_dispatch`
        before any goal is created or sent.  The positive scenario runs the
        production two-goal Pick then Place; every negative runs its isolated
        short journal and never infers a physical verdict.  Per-attempt E state
        is reset first (F1.7) and any escaping exception is contained fail-closed
        with accepted-goal cleanup (F1.9).  F2.1: the E transport scenarios also
        require a fresh observed ``post_grasp_lift_m`` runtime-parameter
        provider before any Pick traffic.
        """
        self._e_reset_attempt_state()
        spec: Mapping[str, object] = {}
        try:
            try:
                spec = stage_e_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_e(scenario_id, "scenario-rejected", [str(exc)])
            self._e_native_gripper_count_provider = native_gripper_goal_count_provider
            self._e_post_grasp_lift_m_provider = post_grasp_lift_m_provider
            if spec["kind"] == "positive":
                return self._run_e_positive(
                    spec,
                    current_tcp_pose_provider=current_tcp_pose_provider,
                    native_gripper_goal_count_provider=native_gripper_goal_count_provider,
                )
            return self._run_e_negative(
                spec,
                current_tcp_pose_provider=current_tcp_pose_provider,
            )
        except Exception as exc:
            return self._e_unexpected_exception(scenario_id, exc, spec=spec)

    def run_pick_place_positive(
        self,
        *,
        current_tcp_pose_provider: Callable[[], Mapping[str, object]] | None = None,
        native_gripper_goal_count_provider: Callable[[], Mapping[str, object]] | None = None,
        post_grasp_lift_m_provider: Callable[[], Mapping[str, object]] | None = None,
    ) -> dict[str, object]:
        """Run the Gate-E positive fixed-target Pick then Place sequence."""
        return self.run_pick_place_sequence(
            "qualification-pick-place-positive",
            current_tcp_pose_provider=current_tcp_pose_provider,
            native_gripper_goal_count_provider=native_gripper_goal_count_provider,
            post_grasp_lift_m_provider=post_grasp_lift_m_provider,
        )

    def run_pick_place_negative(
        self,
        scenario_id: str,
        *,
        current_tcp_pose_provider: Callable[[], Mapping[str, object]] | None = None,
        native_gripper_goal_count_provider: Callable[[], Mapping[str, object]] | None = None,
        post_grasp_lift_m_provider: Callable[[], Mapping[str, object]] | None = None,
    ) -> dict[str, object]:
        """Run one isolated Gate-E negative Pick/Place control.

        The positive scenario is never dispatched through the negative path; an
        unknown or non-E identity fails closed through :func:`stage_e_dispatch`
        before any goal is created or sent.  Per-attempt E state is reset first
        and any escaping exception is contained fail-closed (F1.7/F1.9).
        """
        self._e_reset_attempt_state()
        spec: Mapping[str, object] = {}
        try:
            try:
                spec = stage_e_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_e(scenario_id, "scenario-rejected", [str(exc)])
            if spec["kind"] == "positive":
                return self._evidence_invalid_e(
                    scenario_id,
                    "positive-not-dispatched",
                    ["run_pick_place_negative does not dispatch the positive scenario"],
                    handler=spec["kind"],
                )
            self._e_native_gripper_count_provider = native_gripper_goal_count_provider
            self._e_post_grasp_lift_m_provider = post_grasp_lift_m_provider
            return self._run_e_negative(
                spec,
                current_tcp_pose_provider=current_tcp_pose_provider,
            )
        except Exception as exc:
            return self._e_unexpected_exception(scenario_id, exc, spec=spec)

    # -- Gate E TCP pose sampling (injected provider; never a TF listener) ---

    def _e_record_tcp_sample(
        self, current_tcp_pose_provider: Callable[[], Mapping[str, object]] | None
    ) -> None:
        """Record one fresh finite TCP sample from the injected provider."""
        if current_tcp_pose_provider is None:
            return
        try:
            sample = current_tcp_pose_provider()
        except Exception:
            return
        if not isinstance(sample, Mapping):
            return
        xyz = sample.get("xyz")
        if not (isinstance(xyz, (list, tuple)) and len(xyz) == 3):
            return
        try:
            x, y, z = (float(value) for value in xyz)
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in (x, y, z)):
            return
        identity = sample.get("identity")
        if not (isinstance(identity, (int, str)) and str(identity)):
            return
        age = sample.get("age_s")
        if not _fresh(age, self._thresholds().get("tf_fresh_s", 0.25)):
            return
        self._tcp_pose_samples.append(
            {
                "xyz": [x, y, z],
                "received_mono": float(time.monotonic()),
                "identity": str(identity),
            }
        )
        if len(self._tcp_pose_samples) > 8:
            del self._tcp_pose_samples[: len(self._tcp_pose_samples) - 8]

    def _e_tcp_z_m(self) -> float | None:
        return _tcp_z_from_samples(self._tcp_pose_samples)

    def _e_tcp_speed_m_s(self) -> float | None:
        return _tcp_speed_from_samples(self._tcp_pose_samples)

    def _e_trigger_speed_m_s(self) -> float:
        return float(self._thresholds().get("tcp_trigger_speed_m_s", E_TRIGGER_TCP_SPEED_M_S))

    def _e_settled_speed_m_s(self) -> float:
        return float(self._thresholds().get("settled_speed_m_s", 0.02))

    def _e_safety_stop_velocity_rad_s(self) -> float:
        return float(self._thresholds().get("safety_stop_velocity_rad_s", 0.02))

    def _e_object_lift_m(self) -> float:
        return float(self._thresholds().get("object_lift_m", 0.1))

    def _e_lift_z_tolerance_m(self) -> float:
        return float(self._thresholds().get("lift_tolerance_m", E_LIFT_Z_TOLERANCE_M))

    def _e_normal_state_samples(self) -> int:
        count = int(self._thresholds().get("normal_state_samples", E_NORMAL_STATE_SAMPLES))
        return count if count > 0 else E_NORMAL_STATE_SAMPLES

    def _e_normal_state_sample_count(
        self, baseline: Mapping[str, object]
    ) -> int:
        """Consecutive fresh joint frames whose arm velocity is settled-low."""
        base_seq = int(baseline.get("joint_seq", 0))
        limit = self._e_settled_speed_m_s()
        count = 0
        for frame in reversed(self._joint_velocity_frames):
            if not isinstance(frame, Mapping) or int(frame.get("seq", 0)) <= base_seq:
                break
            velocities = frame.get("velocities")
            if not isinstance(velocities, Sequence) or isinstance(velocities, (str, bytes)):
                break
            try:
                max_velocity = max(abs(float(value)) for value in velocities)
            except (TypeError, ValueError):
                break
            if not math.isfinite(max_velocity) or max_velocity > limit:
                break
            count += 1
        return count

    def _e_max_abs_velocity_rad_s(self, baseline: Mapping[str, object]) -> float | None:
        """Newest fresh joint frame's maximum absolute arm velocity, else None."""
        base_seq = int(baseline.get("joint_seq", 0))
        for frame in reversed(self._joint_velocity_frames):
            if not isinstance(frame, Mapping) or int(frame.get("seq", 0)) <= base_seq:
                break
            velocities = frame.get("velocities")
            if not isinstance(velocities, Sequence) or isinstance(velocities, (str, bytes)):
                continue
            try:
                return max(abs(float(value)) for value in velocities)
            except (TypeError, ValueError):
                continue
        return None

    def _e_target_attached(self) -> bool:
        scene = self._latest_planning_scene
        if scene is None:
            return False
        return TARGET_OBJECT_ID in list(scene.get("attached_ids", []))

    def _e_first_fjt_after_acceptance(
        self, baseline: Mapping[str, object]
    ) -> Mapping[str, object] | None:
        return _first_fjt_goal_after_acceptance(
            self._fresh_fjt_entries(baseline), base=baseline
        )

    def _e_next_fjt(self, after_seq: object) -> Mapping[str, object] | None:
        return _next_fjt_goal(self._fresh_fjt_entries(self._d_baseline()), after_seq=after_seq)

    # -- Gate E goal construction / send ------------------------------------

    def _e_pick_goal(self, spec: Mapping[str, object], *, back_positions: object = None):
        from geometry_msgs.msg import Pose

        geometry = _as_mapping(spec.get("geometry"))
        grasp = geometry.get("grasp_tcp_xyz")
        pose = Pose()
        pose.position.x = float(grasp[0])
        pose.position.y = float(grasp[1])
        pose.position.z = float(grasp[2])
        pose.orientation.w = 1.0
        cloud = deterministic_cube_cloud(frame_id="base_link")
        return build_pick_goal(
            target_pose=pose,
            candidate_poses=[pose],
            env_points=cloud,
            object_points=cloud,
            back_positions=(
                Q_OUTBOUND if back_positions is None else list(back_positions)
            ),
            use_mesh=True,
            stay=False,
        )

    def _e_place_goal(self, spec: Mapping[str, object]):
        from geometry_msgs.msg import PointStamped, Pose

        geometry = _as_mapping(spec.get("geometry"))
        target = _as_mapping(geometry.get("place_target_point"))
        orientation = geometry.get("place_orientation_xyzw")
        point = PointStamped()
        point.header.frame_id = str(target.get("frame_id", "base_link"))
        xyz = target.get("xyz")
        point.point.x = float(xyz[0])
        point.point.y = float(xyz[1])
        point.point.z = float(xyz[2])
        pose = Pose()
        pose.orientation.x = float(orientation[0])
        pose.orientation.y = float(orientation[1])
        pose.orientation.z = float(orientation[2])
        pose.orientation.w = float(orientation[3])
        return build_place_goal(
            target_point=point,
            orientation=pose,
            env_points=deterministic_cube_cloud(frame_id="base_link"),
            back_positions=Q_OUTBOUND,
        )

    def _send_e_action_goal(self, endpoint: str, goal: Any) -> dict[str, object]:
        """Send exactly one Pick/Place goal; return a finite accepted/rejected outcome."""
        thresholds = self._thresholds()
        client = self._action_clients[endpoint]
        server_timeout_s = float(thresholds.get("action_server_wait_s", 5.0))
        if not self._wait_for_server(client, server_timeout_s):
            return {"status": "action-server-unavailable", "reason_code": "action-server-unavailable"}
        accept_timeout_s = float(thresholds.get("goal_accept_timeout_s", 5.0))
        send_future = client.send_goal_async(goal)
        accept_deadline = time.monotonic() + accept_timeout_s
        while not send_future.done() and time.monotonic() < accept_deadline:
            self._spin_once()
        if not send_future.done():
            try:
                send_future.cancel()
            except Exception:
                pass
            return {"status": "goal-accept-timeout", "reason_code": "goal-accept-timeout"}
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            return {"status": "goal-send-exception", "reason_code": "goal-send-exception", "error": str(exc)}
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            return {"status": "goal-rejected", "reason_code": "goal-rejected"}
        goal_id = self._normalize_goal_id(goal_handle)
        # F1.9: track the active accepted goal so an unexpected exception can
        # perform bounded cleanup and record truthful pick/place goal state.
        self._e_active_goal_handle = goal_handle
        if endpoint == "/pickup_action":
            self._e_goal_state["pick_sent"] = True
            self._e_goal_state["pick_goal_id"] = goal_id
        elif endpoint == "/place_action":
            self._e_goal_state["place_sent"] = True
            self._e_goal_state["place_goal_id"] = goal_id
        return {
            "status": "accepted",
            "goal_handle": goal_handle,
            "goal_id": goal_id,
            "endpoint": endpoint,
        }

    def _e_wait_action_result(self, goal_handle: Any) -> dict[str, object]:
        """Bounded wait for a Pick/Place goal result (never a pass by itself)."""
        result_timeout_s = float(self._thresholds().get("execute_timeout_s", 120.0))
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + result_timeout_s
        while not result_future.done() and time.monotonic() < deadline:
            self._spin_once()
        if not result_future.done():
            return {"status": "result-timeout", "result": None}
        try:
            response = result_future.result()
        except Exception as exc:
            return {"status": "result-exception", "result": None, "error": str(exc)}
        return {
            "status": "done",
            "result": getattr(response, "result", None),
            "terminal_status": _execute_status_name(getattr(response, "status", None)),
        }

    def _e_wait_interrupted_result(self, goal_handle: Any) -> dict[str, object]:
        """F1.4: bounded await of an interrupted Pick/Place result terminal.

        Records both status domains from the actual goal handle/result: the
        action-client ``GoalStatus`` (as ``terminal_status``) and the Pick/Place
        ``Result.status`` (as ``result_status``).  A timeout or exception is
        returned as-is so the caller fails closed.
        """
        outcome = self._e_wait_action_result(goal_handle)
        result = outcome.get("result")
        result_status = getattr(result, "status", None) if result is not None else None
        return {
            "status": outcome.get("status"),
            "result_status": result_status,
            "terminal_status": outcome.get("terminal_status"),
            "result_status_string": (
                _pick_place_result_name(result_status) if result_status is not None else None
            ),
        }

    # -- Gate E journal event helpers ---------------------------------------

    def _e_record_snapshot(self, event: str) -> str:
        later_join = self._join_key()
        if later_join is None:
            return "no-join-key"
        try:
            self.journal.snapshot(event, frame_index=later_join[0], timestamp=later_join[1])
            return "recorded"
        except (ValueError, TypeError) as exc:
            return f"rejected: {exc}"

    def _e_record_diff_from_current(self, event: str) -> str:
        scene = self._latest_planning_scene
        if scene is None:
            return "no-scene"
        join = self._join_key()
        if join is None:
            return "no-join-key"
        journal_scene = {**dict(scene), "frame_index": join[0], "timestamp": join[1]}
        try:
            self.journal.record_diff(event, journal_scene)
            return "recorded"
        except (ValueError, TypeError, PermissionError) as exc:
            return f"rejected: {exc}"

    # -- Gate E runner shared preamble --------------------------------------

    def _e_prepare(self, spec: Mapping[str, object], *, current_tcp_pose_provider) -> dict[str, object] | None:
        """Shared E preamble.  Returns ``None`` on success or an evidence record."""
        scenario_id = spec["scenario_id"]
        # F1.12: malformed-back never moves and never samples TCP — it rejects
        # before any motion-only provider requirement, so the TCP provider is
        # required for every E kind except malformed-back.
        if current_tcp_pose_provider is None and spec["kind"] != "malformed-back":
            return self._evidence_invalid_e(
                scenario_id, "no-tcp-pose-provider",
                ["current_tcp_pose_provider is required before sending a Pick/Place goal"],
                handler=spec["kind"],
            )
        if self.join_key_provider is None:
            return self._evidence_invalid_e(scenario_id, "no-join-key", [], handler=spec["kind"])
        readiness = self._readiness()
        if readiness is None:
            return self._evidence_invalid_e(
                scenario_id, "readiness-unavailable",
                ["readiness_snapshot_provider is required before sending any goal"],
                handler=spec["kind"],
            )
        if not readiness["ready"]:
            return self._evidence_invalid_e(
                scenario_id, "readiness-failed", list(readiness["reasons"]), handler=spec["kind"]
            )
        # F1.10: cancel-approach requires a fresh native gripper action-goal
        # count provider at baseline (zero native gripper goals before any goal);
        # missing/stale/provider-error evidence fails closed.
        gripper_count_baseline: int | None = None
        if spec["kind"] == "cancel-approach":
            if self._e_native_gripper_count_provider is None:
                return self._evidence_invalid_e(
                    scenario_id, "no-native-gripper-provider",
                    ["native_gripper_goal_count_provider is required for cancel-approach"],
                    handler=spec["kind"],
                )
            gripper_count_baseline = self._e_native_gripper_count()
            if gripper_count_baseline is None:
                return self._evidence_invalid_e(
                    scenario_id, "native-gripper-provider-unavailable",
                    ["native gripper count provider returned no fresh finite count at baseline"],
                    handler=spec["kind"],
                )
            self._e_native_gripper_count_baseline = gripper_count_baseline
        # F2.1: the E transport scenarios (positive, occupied-place,
        # cancel-transport, safety-transport) require a fresh observation of the
        # production ``pick_and_place`` runtime parameter ``post_grasp_lift_m``
        # BEFORE any Pick traffic.  The committed qualification requires
        # ``object_lift_m=0.10``; the production default ``post_grasp_lift_m``
        # is 0.08, which makes the observed lift peak 0.80 m < the 0.81 m latch,
        # so Gate E must fail immediately with a stable readiness reason and
        # zero action traffic (never a 15 s transport timeout).  The later live
        # orchestrator must launch ``pick_and_place`` with
        # ``post_grasp_lift_m:=0.10`` and read back the value before Gate E.
        if spec["kind"] in _E_TRANSPORT_KINDS:
            if self._e_post_grasp_lift_m_provider is None:
                return self._evidence_invalid_e(
                    scenario_id, "no-post-grasp-lift-m-provider",
                    ["post_grasp_lift_m_provider is required for E transport scenarios"],
                    handler=spec["kind"],
                )
            try:
                lift_sample = self._e_post_grasp_lift_m_provider()
            except Exception as exc:
                return self._evidence_invalid_e(
                    scenario_id, "post-grasp-lift-m-provider-unavailable",
                    [f"post_grasp_lift_m provider raised: {exc}"],
                    handler=spec["kind"],
                )
            lift_result = _post_grasp_lift_m_observation(
                lift_sample,
                object_lift_m=self._e_object_lift_m(),
                fresh_limit_s=self._thresholds().get("tf_fresh_s", 0.25),
            )
            if isinstance(lift_result, str):
                reason_map = {
                    "missing": "post-grasp-lift-m-provider-missing",
                    "stale": "post-grasp-lift-m-provider-stale",
                    "non-finite": "post-grasp-lift-m-provider-non-finite",
                    "below-object-lift": "post-grasp-lift-m-below-object-lift",
                }
                prefix = lift_result.split(":")[0]
                return self._evidence_invalid_e(
                    scenario_id, reason_map.get(prefix, "post-grasp-lift-m-provider-invalid"),
                    [f"post_grasp_lift_m observation rejected: {lift_result}"],
                    handler=spec["kind"],
                )
            lift_value, lift_meta = lift_result
            self._e_post_grasp_lift_m_observed = lift_meta
        acquire_error = self._acquire_scene(scenario_id, e_handler=spec["kind"])
        if acquire_error is not None:
            return acquire_error
        join = self._join_key()
        if join is None:
            return self._evidence_invalid_e(
                scenario_id, "no-join-key",
                ["join_key_provider returned no valid key"], handler=spec["kind"],
            )
        scene = self._journal_scene(join)
        if scene is None:
            return self._evidence_invalid_e(scenario_id, "no-planning-scene", [], handler=spec["kind"])
        scene_error = self._fixture_scene_error(scene, allow_e_target=True)
        if scene_error is not None:
            return self._evidence_invalid_e(
                scenario_id, "fixture-scene-mismatch", [scene_error], handler=spec["kind"]
            )
        try:
            self.journal.record_diff("fixture-ready", scene)
        except (ValueError, TypeError, PermissionError) as exc:
            return self._evidence_invalid_e(
                scenario_id, "journal-fixture-ready-rejected", [str(exc)], handler=spec["kind"]
            )
        self._last_tcp_pose_provider = current_tcp_pose_provider
        return {"readiness": readiness, "scene": scene}

    # -- Gate E negative sequences ------------------------------------------

    def _run_e_negative(
        self,
        spec: Mapping[str, object],
        *,
        current_tcp_pose_provider: Callable[[], Mapping[str, object]] | None = None,
    ) -> dict[str, object]:
        scenario_id = spec["scenario_id"]
        kind = spec["kind"]
        start_wall = time.monotonic()
        event_log: list[str] = []
        fixture_ready_recorded = False
        prepared = self._e_prepare(spec, current_tcp_pose_provider=current_tcp_pose_provider)
        if prepared is None:
            return self._evidence_invalid_e(
                scenario_id, "prepare-failed", ["E preamble produced no record"],
                handler=kind,
            )
        if prepared.get("status") == "evidence-invalid":
            return prepared
        fixture_ready_recorded = True
        event_log.append("fixture-ready")
        readiness = prepared["readiness"]

        if kind == "malformed-back":
            return self._run_e_malformed_back(
                scenario_id, spec, readiness, start_wall,
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
            )
        if kind in ("blocked-approach", "unreachable-grasp"):
            return self._run_e_blocked_or_unreachable(
                scenario_id, spec, readiness, start_wall, kind,
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
            )
        if kind in ("cancel-approach", "cancel-transport"):
            return self._run_e_cancel(
                scenario_id, spec, readiness, start_wall, kind,
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                current_tcp_pose_provider=current_tcp_pose_provider,
            )
        if kind == "safety-transport":
            return self._run_e_safety_transport(
                scenario_id, spec, readiness, start_wall,
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                current_tcp_pose_provider=current_tcp_pose_provider,
            )
        if kind == "occupied-place":
            return self._run_e_occupied_place(
                scenario_id, spec, readiness, start_wall,
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                current_tcp_pose_provider=current_tcp_pose_provider,
            )
        return self._finalize_e_attempt(
            scenario_id, spec, readiness, start_wall, "evidence-invalid",
            event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
            task_error=f"unsupported E kind: {kind}", pick_goal_sent=False,
            place_goal_sent=False, place_goal_accepted=False, goals_sent=0,
        )

    def _run_e_malformed_back(
        self, scenario_id, spec, readiness, start_wall, *, event_log, fixture_ready_recorded,
    ) -> dict[str, object]:
        """malformed-back: builder rejects before any action traffic."""
        from geometry_msgs.msg import Pose

        geometry = _as_mapping(spec.get("geometry"))
        grasp = geometry.get("grasp_tcp_xyz")
        pose = Pose()
        pose.position.x = float(grasp[0])
        pose.position.y = float(grasp[1])
        pose.position.z = float(grasp[2])
        pose.orientation.w = 1.0
        cloud = deterministic_cube_cloud(frame_id="base_link")
        rejected: str | None = None
        try:
            build_pick_goal(
                target_pose=pose,
                candidate_poses=[pose],
                env_points=cloud,
                object_points=cloud,
                back_positions=list(spec["back_positions"]),
                use_mesh=True,
                stay=False,
            )
        except ValueError as exc:
            rejected = str(exc)
        if rejected is None or "7 finite" not in rejected:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error="malformed back_positions were not rejected by the Pick builder",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        self._append_visual_request("before", scenario_id, spec, kind="gate-e-diagnostic")
        teardown_status = self._e_record_snapshot("teardown")
        if teardown_status != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"malformed-back teardown snapshot rejected: {teardown_status}",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        event_log.append("teardown")
        return self._finalize_e_attempt(
            scenario_id, spec, readiness, start_wall, "diagnostic-pass",
            event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
            pick_goal_sent=False, place_goal_sent=False,
            place_goal_accepted=False, goals_sent=0,
        )

    def _run_e_blocked_or_unreachable(
        self, scenario_id, spec, readiness, start_wall, kind, *, event_log, fixture_ready_recorded,
    ) -> dict[str, object]:
        """blocked-approach / unreachable-grasp: Pick terminal non-success, no Place."""
        before_pick = self._e_record_snapshot("before-pick")
        if before_pick != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"before-pick snapshot rejected: {before_pick}",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        event_log.append("before-pick")
        self._append_visual_request("before-pick", scenario_id, spec, kind="gate-e-diagnostic")
        try:
            goal = self._e_pick_goal(spec)
        except Exception as exc:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick goal construction failed: {exc}",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        outcome = self._send_e_action_goal("/pickup_action", goal)
        if outcome["status"] != "accepted":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick goal not accepted: {outcome.get('reason_code')}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=outcome.get("goal_id"),
            )
        pick_goal_id = outcome.get("goal_id")
        result_outcome = self._e_wait_action_result(outcome["goal_handle"])
        result = result_outcome.get("result")
        result_status = getattr(result, "status", None) if result is not None else None
        if result_outcome["status"] != "done" or result is None:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick result unavailable: {result_outcome.get('status')}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, terminal_status=result_outcome.get("terminal_status"),
            )
        if result_status == PICK_PLACE_RESULT_SUCCESS:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error="blocked/unreachable Pick must not succeed",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, task_result_status=result_status,
                terminal_status=result_outcome.get("terminal_status"),
            )
        terminal_status = result_outcome.get("terminal_status")
        # F2.7: production ``complete_pick`` aborts any Pick non-success other
        # than (canceled while canceling), so blocked-approach/unreachable-grasp
        # must record an action-client ABORTED terminal together with a
        # non-success, non-canceled task result.  A contradictory pair (e.g. a
        # SUCCEEDED GoalStatus delivering a failure Result, or a canceled/safety
        # Result) is rejected rather than diagnostic-passed.
        if terminal_status != "aborted":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=(
                    f"blocked/unreachable Pick terminal must be aborted for a "
                    f"non-success result, got terminal={terminal_status} "
                    f"result={result_status}"
                ),
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, task_result_status=result_status,
                terminal_status=terminal_status,
            )
        if result_status in (PICK_PLACE_RESULT_CANCELED, PICK_PLACE_RESULT_SAFETY_STOP):
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=(
                    f"blocked/unreachable Pick must not be canceled/safety-stopped, "
                    f"got result={result_status}"
                ),
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, task_result_status=result_status,
                terminal_status=terminal_status,
            )
        pick_terminal = self._e_record_snapshot("pick-terminal")
        if pick_terminal != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick-terminal snapshot rejected: {pick_terminal}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, task_result_status=result_status,
                terminal_status=result_outcome.get("terminal_status"),
            )
        event_log.append("pick-terminal")
        teardown_status = self._e_record_snapshot("teardown")
        if teardown_status != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"teardown snapshot rejected: {teardown_status}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, task_result_status=result_status,
                terminal_status=result_outcome.get("terminal_status"),
            )
        event_log.append("teardown")
        return self._finalize_e_attempt(
            scenario_id, spec, readiness, start_wall, "diagnostic-pass",
            event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
            pick_goal_sent=True, place_goal_sent=False,
            place_goal_accepted=False, goals_sent=1,
            pick_goal_id=pick_goal_id, task_result_status=result_status,
            terminal_status=result_outcome.get("terminal_status"),
        )

    def _run_e_cancel(
        self, scenario_id, spec, readiness, start_wall, kind, *,
        event_log, fixture_ready_recorded, current_tcp_pose_provider,
    ) -> dict[str, object]:
        """cancel-approach / cancel-transport: cancel the exact Pick handle."""
        before_pick = self._e_record_snapshot("before-pick")
        if before_pick != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"before-pick snapshot rejected: {before_pick}",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        event_log.append("before-pick")
        self._append_visual_request("before-pick", scenario_id, spec, kind="gate-e-diagnostic")
        try:
            goal = self._e_pick_goal(spec)
        except Exception as exc:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick goal construction failed: {exc}",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        outcome = self._send_e_action_goal("/pickup_action", goal)
        if outcome["status"] != "accepted":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick goal not accepted: {outcome.get('reason_code')}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=outcome.get("goal_id"),
            )
        pick_goal_id = outcome.get("goal_id")
        baseline = self._d_baseline()
        if kind == "cancel-approach":
            trigger = self._e_wait_approach_started(baseline, spec)
            trigger_event = "approach-start"
        else:
            trigger = self._e_wait_transport_started(baseline, spec)
            trigger_event = "transport"
        if trigger is None:
            # F2.3: when a cancel trigger never fires (e.g. the native gripper
            # count increased after acceptance), the exact accepted Pick is
            # cleaned up before the evidence-invalid finalization; no
            # attachment/later goal ever occurs and the cleanup outcome is
            # recorded truthfully in the durable artifacts.
            cleanup: dict[str, object] = {}
            if self._e_active_goal_handle is not None:
                try:
                    cleanup = self._cleanup_execute_goal(
                        self._e_active_goal_handle,
                        timeout_s=self._thresholds().get("cancel_timeout_s", 3.0),
                    )
                except Exception as clean_exc:  # pragma: no cover - defensive
                    cleanup = {"cleanup": "exception", "cleanup_error": str(clean_exc)}
                self._e_active_goal_handle = None
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"{kind} trigger timed out before observable evidence",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, cleanup=cleanup,
            )
        if trigger_event == "transport":
            attach_status = self._e_record_diff_from_current("scene-attach")
            if attach_status != "recorded":
                return self._finalize_e_attempt(
                    scenario_id, spec, readiness, start_wall, "evidence-invalid",
                    event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                    task_error=f"scene-attach diff rejected: {attach_status}",
                    pick_goal_sent=True, place_goal_sent=False,
                    place_goal_accepted=False, goals_sent=1,
                    pick_goal_id=pick_goal_id, trigger=trigger,
                )
            event_log.append("scene-attach")
            lift_status = self._e_record_snapshot("lift-complete")
            if lift_status != "recorded":
                return self._finalize_e_attempt(
                    scenario_id, spec, readiness, start_wall, "evidence-invalid",
                    event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                    task_error=f"lift-complete snapshot rejected: {lift_status}",
                    pick_goal_sent=True, place_goal_sent=False,
                    place_goal_accepted=False, goals_sent=1,
                    pick_goal_id=pick_goal_id, trigger=trigger,
                )
            event_log.append("lift-complete")
        trigger_status = self._e_record_snapshot(trigger_event)
        if trigger_status != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"{trigger_event} snapshot rejected: {trigger_status}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
            )
        event_log.append(trigger_event)
        cancel_requested = self._e_record_snapshot("cancel-requested")
        if cancel_requested != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"cancel-requested snapshot rejected: {cancel_requested}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
            )
        event_log.append("cancel-requested")
        cancel_response = self._cancel_execute_goal(
            outcome["goal_handle"],
            expected_goal_uuid=pick_goal_id or "",
            timeout_s=self._thresholds().get("cancel_timeout_s", 3.0),
        )
        if cancel_response.get("response") != "accepted":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"cancel was not accepted: {cancel_response.get('response')}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
                cancel_response=cancel_response,
            )
        # F1.4: boundedly await the exact canceled Pick terminal and record both
        # status domains from the actual handle/result.  A mismatched/unknown/
        # nonterminal status fails closed.
        interrupted = self._e_wait_interrupted_result(outcome["goal_handle"])
        interrupted_status = interrupted.get("status")
        interrupted_result_status = interrupted.get("result_status")
        interrupted_terminal_status = interrupted.get("terminal_status")
        if (
            interrupted_status != "done"
            or interrupted_result_status != PICK_PLACE_RESULT_CANCELED
            or interrupted_terminal_status != "canceled"
        ):
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=(
                    f"{kind} canceled Pick terminal mismatch: result "
                    f"{interrupted_status}/status {interrupted_result_status} "
                    f"({interrupted.get('result_status_string')}) terminal "
                    f"{interrupted_terminal_status}"
                ),
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
                cancel_response=cancel_response,
                task_result_status=interrupted_result_status,
                terminal_status=interrupted_terminal_status,
            )
        if not self._e_wait_quiescent(baseline):
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"{kind} cancel did not reach quiescence",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
                cancel_response=cancel_response,
                task_result_status=interrupted_result_status,
                terminal_status=interrupted_terminal_status,
            )
        quiescent = self._e_record_snapshot("quiescent")
        if quiescent != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"quiescent snapshot rejected: {quiescent}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
                cancel_response=cancel_response,
                task_result_status=interrupted_result_status,
                terminal_status=interrupted_terminal_status,
            )
        event_log.append("quiescent")
        teardown_status = self._e_record_snapshot("teardown")
        if teardown_status != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"teardown snapshot rejected: {teardown_status}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
                cancel_response=cancel_response,
                task_result_status=interrupted_result_status,
                terminal_status=interrupted_terminal_status,
            )
        event_log.append("teardown")
        return self._finalize_e_attempt(
            scenario_id, spec, readiness, start_wall, "diagnostic-pass",
            event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
            pick_goal_sent=True, place_goal_sent=False,
            place_goal_accepted=False, goals_sent=1,
            pick_goal_id=pick_goal_id, trigger=trigger,
            cancel_response=cancel_response,
            task_result_status=interrupted_result_status,
            terminal_status=interrupted_terminal_status,
        )

    def _e_wait_approach_started(self, baseline: Mapping[str, object], spec) -> Mapping[str, object] | None:
        """Wait for the cancel-approach trigger (fresh FJT EXECUTING + TCP motion).

        F1.6/F1.10: the approach FJT must be received within
        ``E_FJT_CORRELATION_TIMEOUT_S`` of the goal-acceptance baseline, the
        target must never attach, and the injected native gripper action-goal
        count must not increase (zero native gripper goals before any
        cancellation).  Missing/stale/provider-error gripper evidence or a
        received-before-baseline FJT can never satisfy the trigger.
        """
        timeout_s = float(spec.get("trigger_timeout_s") or 10.0)
        speed_limit = self._e_trigger_speed_m_s()
        window_s = float(E_FJT_CORRELATION_TIMEOUT_S)
        captured: dict[str, object] = {}

        def _seen() -> bool:
            self._e_record_tcp_sample(self._last_tcp_pose_provider)
            if self._e_target_attached():
                return False
            gripper_now = self._e_native_gripper_count()
            if gripper_now is None:
                return False
            baseline_count = self._e_native_gripper_count_baseline
            if baseline_count is None or gripper_now != baseline_count:
                return False
            first = self._e_first_fjt_after_acceptance(baseline)
            if first is None:
                return False
            if not _fjt_within_receipt_window(first, baseline.get("start_mono"), window_s):
                return False
            speed = self._e_tcp_speed_m_s()
            if speed is None or speed < speed_limit:
                return False
            captured.update(dict(first))
            captured["trigger_kind"] = "approach"
            captured["tcp_speed_m_s"] = float(speed)
            captured["tcp_z_m"] = self._e_tcp_z_m()
            captured["scene_target_attached"] = False
            captured["native_gripper_goal_count_baseline"] = baseline_count
            captured["native_gripper_goal_count_now"] = gripper_now
            captured["fjt_receipt_delta_s"] = _fjt_receipt_delta_s(
                first, baseline.get("start_mono")
            )
            # F3.3: retain the observed FJT trigger for the unexpected-exception
            # controller-truth derivation (approach FJT is controller traffic).
            self._e_observed_fjt_trigger = dict(captured)
            return True

        if not self._wait_for(_seen, timeout_s):
            return None
        return captured

    def _e_wait_transport_started(self, baseline: Mapping[str, object], spec) -> Mapping[str, object] | None:
        """Two-phase transport trigger (F1.2/F1.6).

        Phase 1 ``lift_complete`` latches only after observed target attachment,
        TCP z above the configured lift threshold, ``max_abs_arm_velocity_rad_s
        <= settled`` and at least two consecutive fresh normal-state samples.
        Phase 2 ``transport_started`` then requires a **later** fresh FJT
        ``EXECUTING`` entry, target still attached, and fresh TCP speed >= the
        trigger limit; it never re-requires the settled condition while moving.
        Receipt sequences/timestamps prove the transport FJT/TCP evidence is
        later than the lift latch (``fjt_receipt_delta_s`` within
        ``E_FJT_CORRELATION_TIMEOUT_S`` of the lift latch boundary).
        """
        timeout_s = float(spec.get("trigger_timeout_s") or 15.0)
        speed_limit = self._e_trigger_speed_m_s()
        window_s = float(E_FJT_CORRELATION_TIMEOUT_S)
        settled_limit = self._e_settled_speed_m_s()
        lift_z = (
            float(spec["geometry"]["grasp_tcp_xyz"][2])
            + self._e_object_lift_m()
            - self._e_lift_z_tolerance_m()
        )
        captured: dict[str, object] = {}
        lift: dict[str, object] = {}
        deadline = time.monotonic() + timeout_s

        def _lift_latched() -> bool:
            self._e_record_tcp_sample(self._last_tcp_pose_provider)
            if not self._e_target_attached():
                return False
            tcp_z = self._e_tcp_z_m()
            if tcp_z is None or tcp_z < lift_z:
                return False
            max_velocity = self._e_max_abs_velocity_rad_s(baseline)
            if max_velocity is None or max_velocity > settled_limit:
                return False
            normal_samples = self._e_normal_state_sample_count(baseline)
            if normal_samples < self._e_normal_state_samples():
                return False
            newest = self._newest_fjt_status()
            lift.update(
                {
                    "lift_tcp_z_m": float(tcp_z),
                    "lift_max_abs_arm_velocity_rad_s": float(max_velocity),
                    "lift_normal_state_samples": int(normal_samples),
                    "lift_scene_target_attached": True,
                    "lift_mono": float(time.monotonic()),
                    "lift_fjt_seq": int(
                        newest.get("seq", 0)
                        if newest is not None
                        else baseline.get("fjt_seq", 0)
                    ),
                }
            )
            # F3.1: record the lift-latch wall instant so a deterministic test
            # barrier can inject the transport FJT strictly after the latch.
            self._e_lift_latch_mono = float(lift["lift_mono"])
            return True

        def _transport_seen() -> bool:
            self._e_record_tcp_sample(self._last_tcp_pose_provider)
            if not self._e_target_attached():
                return False
            next_goal = _next_fjt_goal(
                self._fresh_fjt_entries(baseline), after_seq=lift.get("lift_fjt_seq", 0)
            )
            if next_goal is None:
                return False
            if not _fjt_within_receipt_window(next_goal, lift.get("lift_mono"), window_s):
                return False
            speed = self._e_tcp_speed_m_s()
            if speed is None or speed < speed_limit:
                return False
            captured.update(dict(next_goal))
            captured["trigger_kind"] = "transport"
            captured["tcp_speed_m_s"] = float(speed)
            captured["tcp_z_m"] = self._e_tcp_z_m()
            captured["scene_target_attached"] = True
            captured["fjt_receipt_delta_s"] = _fjt_receipt_delta_s(
                next_goal, lift.get("lift_mono")
            )
            captured.update(lift)
            # F3.3: retain the observed FJT trigger so the unexpected-exception
            # path can derive controller traffic from real FJT evidence only.
            self._e_observed_fjt_trigger = dict(captured)
            return True

        while time.monotonic() < deadline:
            if lift and _transport_seen():
                return captured
            if not lift and _lift_latched():
                continue
            self._spin_once()
        return None

    def _e_wait_quiescent(self, baseline: Mapping[str, object]) -> bool:
        timeout_s = float(self._thresholds().get("quiescence_timeout_s", 5.0))
        limit = self._e_settled_speed_m_s()

        def _quiet() -> bool:
            return self._e_normal_state_sample_count(baseline) >= self._e_normal_state_samples()

        return self._wait_for(_quiet, timeout_s)

    def _run_e_safety_transport(
        self, scenario_id, spec, readiness, start_wall, *, event_log, fixture_ready_recorded,
        current_tcp_pose_provider,
    ) -> dict[str, object]:
        """safety-transport: assert operator safety on the transport trigger."""
        before_pick = self._e_record_snapshot("before-pick")
        if before_pick != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"before-pick snapshot rejected: {before_pick}",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        event_log.append("before-pick")
        self._append_visual_request("before-pick", scenario_id, spec, kind="gate-e-diagnostic")
        try:
            goal = self._e_pick_goal(spec)
        except Exception as exc:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick goal construction failed: {exc}",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        outcome = self._send_e_action_goal("/pickup_action", goal)
        if outcome["status"] != "accepted":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick goal not accepted: {outcome.get('reason_code')}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=outcome.get("goal_id"),
            )
        pick_goal_id = outcome.get("goal_id")
        baseline = self._d_baseline()
        trigger = self._e_wait_transport_started(baseline, spec)
        if trigger is None:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error="safety-transport trigger timed out before observable evidence",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id,
            )
        attach_status = self._e_record_diff_from_current("scene-attach")
        if attach_status != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"scene-attach diff rejected: {attach_status}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
            )
        event_log.append("scene-attach")
        for label in ("lift-complete", "transport"):
            status = self._e_record_snapshot(label)
            if status != "recorded":
                return self._finalize_e_attempt(
                    scenario_id, spec, readiness, start_wall, "evidence-invalid",
                    event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                    task_error=f"{label} snapshot rejected: {status}",
                    pick_goal_sent=True, place_goal_sent=False,
                    place_goal_accepted=False, goals_sent=1,
                    pick_goal_id=pick_goal_id, trigger=trigger,
                )
            event_log.append(label)
        # Assert operator safety: assert with publish_operator(True) (a True
        # operator input is a protective stop request), then require the
        # effective safety-stop, await the exact safety-stopped Pick terminal
        # (F1.4), clear only after the stop, and never send a later goal.
        operator_published: list[bool] = []
        try:
            self.publish_operator(True)
            operator_published.append(True)
        except Exception as exc:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"operator assert publish failed: {exc}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
            )
        if not self._wait_for_safety_stop(float(self._thresholds().get("safety_stop_wait_s", 0.5))):
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error="effective safety-stop was not observed after operator assert",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
            )
        effective_stop = self._e_record_snapshot("effective-stop")
        if effective_stop != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"effective-stop snapshot rejected: {effective_stop}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
            )
        event_log.append("effective-stop")
        trigger["operator_published"] = list(operator_published)
        # F1.4: boundedly await the exact safety-stopped Pick terminal.  Current
        # production semantics: interruption_result maps a SafetyStop interrupt to
        # ResultStatus::SafetyStop, and complete_pick aborts (GoalStatus=ABORTED)
        # for any status other than Success / (Canceled while canceling), so the
        # action-client terminal is "aborted" and the Pick Result.status is 5.
        interrupted = self._e_wait_interrupted_result(outcome["goal_handle"])
        interrupted_status = interrupted.get("status")
        interrupted_result_status = interrupted.get("result_status")
        interrupted_terminal_status = interrupted.get("terminal_status")
        if (
            interrupted_status != "done"
            or interrupted_result_status != PICK_PLACE_RESULT_SAFETY_STOP
            or interrupted_terminal_status != "aborted"
        ):
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=(
                    f"safety-transport Pick terminal mismatch: result "
                    f"{interrupted_status}/status {interrupted_result_status} "
                    f"({interrupted.get('result_status_string')}) terminal "
                    f"{interrupted_terminal_status}"
                ),
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
                task_result_status=interrupted_result_status,
                terminal_status=interrupted_terminal_status,
            )
        try:
            self.publish_operator(False)
            operator_published.append(False)
        except Exception as exc:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"operator clear publish failed: {exc}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
                task_result_status=interrupted_result_status,
                terminal_status=interrupted_terminal_status,
            )
        trigger["operator_published"] = list(operator_published)
        operator_clear = self._e_record_snapshot("operator-clear")
        if operator_clear != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"operator-clear snapshot rejected: {operator_clear}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
                task_result_status=interrupted_result_status,
                terminal_status=interrupted_terminal_status,
            )
        event_log.append("operator-clear")
        if not self._e_wait_quiescent(baseline):
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error="safety-transport did not reach quiescence (auto-resume suspected)",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
                task_result_status=interrupted_result_status,
                terminal_status=interrupted_terminal_status,
            )
        quiescent = self._e_record_snapshot("quiescent")
        if quiescent != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"quiescent snapshot rejected: {quiescent}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
                task_result_status=interrupted_result_status,
                terminal_status=interrupted_terminal_status,
            )
        event_log.append("quiescent")
        teardown_status = self._e_record_snapshot("teardown")
        if teardown_status != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"teardown snapshot rejected: {teardown_status}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, trigger=trigger,
                task_result_status=interrupted_result_status,
                terminal_status=interrupted_terminal_status,
            )
        event_log.append("teardown")
        return self._finalize_e_attempt(
            scenario_id, spec, readiness, start_wall, "diagnostic-pass",
            event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
            pick_goal_sent=True, place_goal_sent=False,
            place_goal_accepted=False, goals_sent=1,
            pick_goal_id=pick_goal_id, trigger=trigger,
            task_result_status=interrupted_result_status,
            terminal_status=interrupted_terminal_status,
        )

    def _run_e_occupied_place(
        self, scenario_id, spec, readiness, start_wall, *, event_log, fixture_ready_recorded,
        current_tcp_pose_provider,
    ) -> dict[str, object]:
        """occupied-place: Pick to attached success, then cancel Place at target motion."""
        before_pick = self._e_record_snapshot("before-pick")
        if before_pick != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"before-pick snapshot rejected: {before_pick}",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        event_log.append("before-pick")
        self._append_visual_request("before-pick", scenario_id, spec, kind="gate-e-diagnostic")
        try:
            pick_goal = self._e_pick_goal(spec)
        except Exception as exc:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick goal construction failed: {exc}",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        pick_outcome = self._send_e_action_goal("/pickup_action", pick_goal)
        if pick_outcome["status"] != "accepted":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick goal not accepted: {pick_outcome.get('reason_code')}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_outcome.get("goal_id"),
            )
        pick_goal_id = pick_outcome.get("goal_id")
        baseline = self._d_baseline()
        # F1.1/F1.2: observe and latch the lift and transport checkpoints WHILE
        # the Pick goal remains executing (production Pick with ``stay=false``
        # returns to ``Q_OUTBOUND`` only after lift, so the transient return
        # motion is already gone once the Pick result is published).  Only after
        # the transport latch may the flow await the Pick terminal and require
        # success.  Never infer transport from the later Pick result.
        transport_trigger = self._e_wait_transport_started(baseline, spec)
        if transport_trigger is None:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error="occupied-place transport evidence was not observed during Pick execution",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id,
            )
        pick_result = self._e_wait_action_result(pick_outcome["goal_handle"])
        result = pick_result.get("result")
        pick_status = getattr(result, "status", None) if result is not None else None
        if pick_result["status"] != "done" or result is None or pick_status != PICK_PLACE_RESULT_SUCCESS:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=(
                    f"occupied-place Pick must succeed, got "
                    f"{pick_result.get('status')}/{pick_status}"
                ),
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, task_result_status=pick_status,
                terminal_status=pick_result.get("terminal_status"),
                trigger=transport_trigger,
            )
        for label in ("scene-attach", "lift-complete", "transport"):
            if label == "scene-attach":
                status = self._e_record_diff_from_current("scene-attach")
            else:
                status = self._e_record_snapshot(label)
            if status != "recorded":
                return self._finalize_e_attempt(
                    scenario_id, spec, readiness, start_wall, "evidence-invalid",
                    event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                    task_error=f"{label} record rejected: {status}",
                    pick_goal_sent=True, place_goal_sent=False,
                    place_goal_accepted=False, goals_sent=1,
                    pick_goal_id=pick_goal_id, task_result_status=pick_status,
                    terminal_status=pick_result.get("terminal_status"),
                    trigger=transport_trigger,
                )
            event_log.append(label)
        try:
            place_goal = self._e_place_goal(spec)
        except Exception as exc:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"place goal construction failed: {exc}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, task_result_status=pick_status,
                terminal_status=pick_result.get("terminal_status"),
                trigger=transport_trigger,
            )
        place_outcome = self._send_e_action_goal("/place_action", place_goal)
        if place_outcome["status"] != "accepted":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"place goal not accepted: {place_outcome.get('reason_code')}",
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=False, goals_sent=2,
                pick_goal_id=pick_goal_id, place_goal_id=place_outcome.get("goal_id"),
                task_result_status=pick_status, terminal_status=pick_result.get("terminal_status"),
                trigger=transport_trigger,
            )
        place_goal_id = place_outcome.get("goal_id")
        place_goal_accepted = True
        place_goals_sent = 2
        place_accepted = self._e_record_snapshot("place-goal-accepted")
        if place_accepted != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"place-goal-accepted snapshot rejected: {place_accepted}",
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=place_goals_sent,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=pick_status, terminal_status=pick_result.get("terminal_status"),
                trigger=transport_trigger,
            )
        event_log.append("place-goal-accepted")
        place_baseline = self._d_baseline()
        place_trigger = self._e_wait_place_target_motion(place_baseline, spec)
        if place_trigger is None:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error="occupied-place target-motion trigger timed out before cancel",
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=place_goals_sent,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=pick_status, terminal_status=pick_result.get("terminal_status"),
            )
        # F3.2: record the pre-cancel PlanningScene baseline (the latest valid
        # scene sequence and receipt time used for the transport/Place trigger)
        # so the post-cancel re-observation can be gated on a strictly newer
        # scene rather than accepting the last cached pre-cancel attached scene.
        pre_cancel_scene = self._latest_planning_scene
        pre_cancel_scene_sequence = (
            int(pre_cancel_scene.get("scene_sequence", -1))
            if isinstance(pre_cancel_scene, Mapping)
            else -1
        )
        pre_cancel_scene_receipt = (
            float(pre_cancel_scene.get("scene_timestamp", 0.0))
            if isinstance(pre_cancel_scene, Mapping)
            else None
        )
        cancel_requested = self._e_record_snapshot("cancel-requested")
        if cancel_requested != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"cancel-requested snapshot rejected: {cancel_requested}",
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=place_goals_sent,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=pick_status, terminal_status=pick_result.get("terminal_status"),
                trigger=place_trigger,
            )
        event_log.append("cancel-requested")
        cancel_response = self._cancel_execute_goal(
            place_outcome["goal_handle"],
            expected_goal_uuid=place_goal_id or "",
            timeout_s=self._thresholds().get("cancel_timeout_s", 3.0),
        )
        if cancel_response.get("response") != "accepted":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"place cancel was not accepted: {cancel_response.get('response')}",
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=place_goals_sent,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=pick_status, terminal_status=pick_result.get("terminal_status"),
                trigger=place_trigger, cancel_response=cancel_response,
            )
        # F1.4: boundedly await the exact canceled Place terminal and record both
        # status domains from the actual handle/result.  Do not reuse the Pick's
        # success status as the Place status.
        interrupted = self._e_wait_interrupted_result(place_outcome["goal_handle"])
        interrupted_status = interrupted.get("status")
        place_result_status = interrupted.get("result_status")
        place_terminal_status = interrupted.get("terminal_status")
        if (
            interrupted_status != "done"
            or place_result_status != PICK_PLACE_RESULT_CANCELED
            or place_terminal_status != "canceled"
        ):
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=(
                    f"occupied-place canceled Place terminal mismatch: result "
                    f"{interrupted_status}/status {place_result_status} "
                    f"({interrupted.get('result_status_string')}) terminal "
                    f"{place_terminal_status}"
                ),
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=place_goals_sent,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=place_result_status,
                terminal_status=place_terminal_status,
                trigger=place_trigger, cancel_response=cancel_response,
            )
        # F3.2: the exact Place cancel terminal wall instant.  A fresh post-cancel
        # PlanningScene observation must be received strictly after this instant.
        cancel_terminal_mono = float(time.monotonic())
        if not self._e_wait_quiescent(place_baseline):
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error="occupied-place cancel did not reach quiescence",
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=place_goals_sent,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=place_result_status,
                terminal_status=place_terminal_status,
                trigger=place_trigger, cancel_response=cancel_response,
            )
        quiescent = self._e_record_snapshot("quiescent")
        if quiescent != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"quiescent snapshot rejected: {quiescent}",
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=place_goals_sent,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=place_result_status,
                terminal_status=place_terminal_status,
                trigger=place_trigger, cancel_response=cancel_response,
            )
        event_log.append("quiescent")
        # F2.5/F3.2: after the exact Place cancel terminal and quiescence, require
        # a STRICTLY FRESH PlanningScene observation proving
        # ``pick_and_place/object_mesh`` remains attached.  The last cached
        # pre-cancel attached scene is never accepted: the runner boundedly waits
        # for a valid scene whose ``scene_sequence`` is strictly greater than the
        # pre-cancel baseline AND whose receipt time is after the cancel terminal.
        # Only that fresh scene may establish ``post_cancel_target_attached``;
        # timeout, malformed newer scene, unchanged sequence, or detached target
        # is ``evidence-invalid``.  Baseline sequence, post-cancel sequence,
        # receipt delta, attachment state, and timeout/error reason are recorded
        # in the final trigger artifact.
        post_cancel_trigger = dict(place_trigger)
        post_cancel_trigger["pre_cancel_scene_sequence"] = int(pre_cancel_scene_sequence)
        if pre_cancel_scene_receipt is not None:
            post_cancel_trigger["pre_cancel_scene_receipt_mono"] = float(pre_cancel_scene_receipt)
        post_cancel_trigger["post_cancel_scene_sequence"] = int(pre_cancel_scene_sequence)
        post_cancel_trigger["post_cancel_scene_receipt_delta_s"] = None
        post_cancel_trigger["post_cancel_target_attached"] = False
        post_cancel_trigger["post_cancel_fresh_scene_reason"] = "pending"
        fresh = self._e_wait_post_cancel_fresh_scene(
            pre_cancel_scene_sequence,
            cancel_terminal_mono,
            timeout_s=float(self._thresholds().get("post_cancel_scene_wait_s", 2.0)),
        )
        if fresh is None:
            if (
                self._planning_scene_invalid
                and self._scene_invalid_sequence is not None
                and int(self._scene_invalid_sequence) > int(pre_cancel_scene_sequence)
            ):
                post_cancel_trigger["post_cancel_fresh_scene_reason"] = (
                    "post-cancel newer scene malformed/provider-error (fail-closed)"
                )
            else:
                post_cancel_trigger["post_cancel_fresh_scene_reason"] = (
                    "post-cancel fresh scene timed out (no strictly newer valid scene)"
                )
            post_cancel_trigger["post_cancel_scene_sequence"] = int(pre_cancel_scene_sequence)
            post_cancel_trigger["post_cancel_scene_receipt_delta_s"] = None
            post_cancel_trigger["post_cancel_target_attached"] = False
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=post_cancel_trigger["post_cancel_fresh_scene_reason"],
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=place_goals_sent,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=place_result_status,
                terminal_status=place_terminal_status,
                trigger=post_cancel_trigger, cancel_response=cancel_response,
            )
        post_cancel_trigger.update(dict(fresh))
        post_cancel_trigger["post_cancel_fresh_scene_reason"] = "fresh-observed"
        if not bool(fresh.get("post_cancel_target_attached")):
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=(
                    "occupied-place target detached before post-cancel re-observation "
                    "(open/detach won the cancel race)"
                ),
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=place_goals_sent,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=place_result_status,
                terminal_status=place_terminal_status,
                trigger=post_cancel_trigger, cancel_response=cancel_response,
            )
        event_log.append("post-cancel-attached")
        teardown_status = self._e_record_snapshot("teardown")
        if teardown_status != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"teardown snapshot rejected: {teardown_status}",
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=place_goals_sent,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=place_result_status,
                terminal_status=place_terminal_status,
                trigger=post_cancel_trigger, cancel_response=cancel_response,
            )
        event_log.append("teardown")
        return self._finalize_e_attempt(
            scenario_id, spec, readiness, start_wall, "diagnostic-pass",
            event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
            pick_goal_sent=True, place_goal_sent=True,
            place_goal_accepted=place_goal_accepted, goals_sent=place_goals_sent,
            pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
            task_result_status=place_result_status,
            terminal_status=place_terminal_status,
            trigger=post_cancel_trigger, cancel_response=cancel_response,
        )

    def _e_wait_place_target_motion(self, baseline: Mapping[str, object], spec) -> Mapping[str, object] | None:
        """Wait for the first fresh Place FJT EXECUTING entry with TCP motion.

        The place trigger requires the target to remain attached (the Place
        server may open/detach only on its natural failure path, which we never
        wait for here).  F1.6: the Place FJT must be received within
        ``E_FJT_CORRELATION_TIMEOUT_S`` of the Place goal-acceptance baseline.
        """
        timeout_s = float(spec.get("trigger_timeout_s") or 15.0)
        speed_limit = self._e_trigger_speed_m_s()
        window_s = float(E_FJT_CORRELATION_TIMEOUT_S)
        captured: dict[str, object] = {}

        def _seen() -> bool:
            self._e_record_tcp_sample(self._last_tcp_pose_provider)
            if not self._e_target_attached():
                return False
            first = self._e_first_fjt_after_acceptance(baseline)
            if first is None:
                return False
            if not _fjt_within_receipt_window(first, baseline.get("start_mono"), window_s):
                return False
            speed = self._e_tcp_speed_m_s()
            if speed is None or speed < speed_limit:
                return False
            captured.update(dict(first))
            captured["trigger_kind"] = "place-target-motion"
            captured["tcp_speed_m_s"] = float(speed)
            captured["tcp_z_m"] = self._e_tcp_z_m()
            captured["scene_target_attached"] = True
            captured["fjt_receipt_delta_s"] = _fjt_receipt_delta_s(
                first, baseline.get("start_mono")
            )
            # F3.3: retain the observed FJT trigger for the unexpected-exception
            # controller-truth derivation (place target-motion FJT is controller
            # traffic).
            self._e_observed_fjt_trigger = dict(captured)
            return True

        if not self._wait_for(_seen, timeout_s):
            return None
        return captured

    def _e_wait_post_cancel_fresh_scene(
        self, pre_cancel_seq, cancel_terminal_mono, *, timeout_s
    ) -> dict[str, object] | None:
        """F3.2: boundedly wait for a strictly newer valid PlanningScene.

        After the exact Place cancel terminal and quiescence, occupied-place must
        NOT accept the last cached pre-cancel attached scene.  This wait requires
        a valid PlanningScene observation whose ``scene_sequence`` is strictly
        greater than the pre-cancel baseline AND whose receipt time is after the
        cancel terminal.  Only that fresh scene may establish
        ``post_cancel_target_attached``.  Returns the captured observation
        (sequence, receipt delta, attachment state) or ``None`` on timeout /
        unchanged sequence / malformed-newer (the fail-closed latch keeps the
        last valid cached scene, so a malformed newer callback simply never
        advances the observable sequence).
        """
        captured: dict[str, object] = {}

        def _seen() -> bool:
            scene = self._latest_planning_scene
            if scene is None:
                return False
            seq = int(scene.get("scene_sequence", -1))
            if seq <= int(pre_cancel_seq):
                return False
            receipt = scene.get("scene_timestamp")
            if receipt is None:
                return False
            try:
                receipt_s = float(receipt)
            except (TypeError, ValueError):
                return False
            if receipt_s < float(cancel_terminal_mono):
                return False
            captured["post_cancel_scene_sequence"] = seq
            captured["post_cancel_scene_receipt_delta_s"] = (
                receipt_s - float(cancel_terminal_mono)
            )
            captured["post_cancel_target_attached"] = bool(self._e_target_attached())
            return True

        if not self._wait_for(_seen, float(timeout_s)):
            return None
        return captured

    def _e_wait_detached(self) -> bool:
        """Wait for the target to leave ``attached_collision_objects``."""
        timeout_s = float(self._thresholds().get("detach_wait_s", 5.0))

        def _seen() -> bool:
            scene = self._latest_planning_scene
            if scene is None:
                return False
            return TARGET_OBJECT_ID not in list(scene.get("attached_ids", []))

        return self._wait_for(_seen, timeout_s)

    # -- Gate E positive sequence -------------------------------------------

    def _run_e_positive(
        self,
        spec: Mapping[str, object],
        *,
        current_tcp_pose_provider: Callable[[], Mapping[str, object]] | None = None,
        native_gripper_goal_count_provider: Callable[[], Mapping[str, object]] | None = None,
    ) -> dict[str, object]:
        scenario_id = spec["scenario_id"]
        start_wall = time.monotonic()
        event_log: list[str] = []
        fixture_ready_recorded = False
        prepared = self._e_prepare(spec, current_tcp_pose_provider=current_tcp_pose_provider)
        if prepared is None:
            return self._evidence_invalid_e(
                scenario_id, "prepare-failed", ["E preamble produced no record"], handler=spec["kind"],
            )
        if prepared.get("status") == "evidence-invalid":
            return prepared
        fixture_ready_recorded = True
        event_log.append("fixture-ready")
        readiness = prepared["readiness"]

        before_pick = self._e_record_snapshot("before-pick")
        if before_pick != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"before-pick snapshot rejected: {before_pick}",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        event_log.append("before-pick")
        self._append_visual_request("before-pick", scenario_id, spec, kind="gate-e-diagnostic")
        try:
            pick_goal = self._e_pick_goal(spec)
        except Exception as exc:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick goal construction failed: {exc}",
                pick_goal_sent=False, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=0,
            )
        pick_outcome = self._send_e_action_goal("/pickup_action", pick_goal)
        if pick_outcome["status"] != "accepted":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"pick goal not accepted: {pick_outcome.get('reason_code')}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_outcome.get("goal_id"),
            )
        pick_goal_id = pick_outcome.get("goal_id")
        baseline = self._d_baseline()
        # F1.1/F1.2: observe and latch the lift and transport checkpoints WHILE
        # the Pick goal remains executing (production Pick with ``stay=false``
        # returns to ``Q_OUTBOUND`` only after lift, so the transient return
        # motion is already gone once the Pick result is published).  Only after
        # the transport latch may the flow await the Pick terminal and require
        # success.  Never infer transport from the later Pick result.
        transport_trigger = self._e_wait_transport_started(baseline, spec)
        if transport_trigger is None:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error="positive transport evidence was not observed during Pick execution",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id,
            )
        pick_result = self._e_wait_action_result(pick_outcome["goal_handle"])
        result = pick_result.get("result")
        pick_status = getattr(result, "status", None) if result is not None else None
        if pick_result["status"] != "done" or result is None or pick_status != PICK_PLACE_RESULT_SUCCESS:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"positive Pick must succeed, got {pick_result.get('status')}/{pick_status}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, task_result_status=pick_status,
                terminal_status=pick_result.get("terminal_status"),
                trigger=transport_trigger,
            )
        # Record lift and transport only after their online predicates became
        # true (observed attachment plus a later fresh FJT EXECUTING entry and
        # fresh TCP motion; never action-result inference for scene-attach).
        for label in ("scene-attach", "lift-complete", "transport"):
            if label == "scene-attach":
                status = self._e_record_diff_from_current("scene-attach")
            else:
                status = self._e_record_snapshot(label)
            if status != "recorded":
                return self._finalize_e_attempt(
                    scenario_id, spec, readiness, start_wall, "evidence-invalid",
                    event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                    task_error=f"{label} record rejected: {status}",
                    pick_goal_sent=True, place_goal_sent=False,
                    place_goal_accepted=False, goals_sent=1,
                    pick_goal_id=pick_goal_id, task_result_status=pick_status,
                    terminal_status=pick_result.get("terminal_status"),
                    trigger=transport_trigger,
                )
            event_log.append(label)
        before_release = self._e_record_snapshot("before-release")
        if before_release != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"before-release snapshot rejected: {before_release}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, task_result_status=pick_status,
                terminal_status=pick_result.get("terminal_status"),
                trigger=transport_trigger,
            )
        event_log.append("before-release")
        try:
            place_goal = self._e_place_goal(spec)
        except Exception as exc:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"place goal construction failed: {exc}",
                pick_goal_sent=True, place_goal_sent=False,
                place_goal_accepted=False, goals_sent=1,
                pick_goal_id=pick_goal_id, task_result_status=pick_status,
                terminal_status=pick_result.get("terminal_status"),
                trigger=transport_trigger,
            )
        place_outcome = self._send_e_action_goal("/place_action", place_goal)
        if place_outcome["status"] != "accepted":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"place goal not accepted: {place_outcome.get('reason_code')}",
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=False, goals_sent=2,
                pick_goal_id=pick_goal_id, place_goal_id=place_outcome.get("goal_id"),
                task_result_status=pick_status, terminal_status=pick_result.get("terminal_status"),
                trigger=transport_trigger,
            )
        place_goal_id = place_outcome.get("goal_id")
        place_goal_accepted = True
        place_result = self._e_wait_action_result(place_outcome["goal_handle"])
        place_result_msg = place_result.get("result")
        place_status = getattr(place_result_msg, "status", None) if place_result_msg is not None else None
        if place_result["status"] != "done" or place_result_msg is None or place_status != PICK_PLACE_RESULT_SUCCESS:
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=(
                    f"positive Place must succeed, got "
                    f"{place_result.get('status')}/{place_status}"
                ),
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=2,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=place_status,
                terminal_status=place_result.get("terminal_status"),
                trigger=transport_trigger,
            )
        # Require observed detach before recording scene-detach.
        if not self._e_wait_detached():
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error="positive scene-detach was not observed after Place success",
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=2,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=place_status,
                terminal_status=place_result.get("terminal_status"),
                trigger=transport_trigger,
            )
        scene_detach = self._e_record_diff_from_current("scene-detach")
        if scene_detach != "recorded":
            return self._finalize_e_attempt(
                scenario_id, spec, readiness, start_wall, "evidence-invalid",
                event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                task_error=f"scene-detach diff rejected: {scene_detach}",
                pick_goal_sent=True, place_goal_sent=True,
                place_goal_accepted=place_goal_accepted, goals_sent=2,
                pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                task_result_status=place_status,
                terminal_status=place_result.get("terminal_status"),
                trigger=transport_trigger,
            )
        event_log.append("scene-detach")
        for label in ("released-settled", "teardown"):
            status = self._e_record_snapshot(label)
            if status != "recorded":
                return self._finalize_e_attempt(
                    scenario_id, spec, readiness, start_wall, "evidence-invalid",
                    event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
                    task_error=f"{label} snapshot rejected: {status}",
                    pick_goal_sent=True, place_goal_sent=True,
                    place_goal_accepted=place_goal_accepted, goals_sent=2,
                    pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
                    task_result_status=place_status,
                    terminal_status=place_result.get("terminal_status"),
                    trigger=transport_trigger,
                )
            event_log.append(label)
        return self._finalize_e_attempt(
            scenario_id, spec, readiness, start_wall, "diagnostic-pass",
            event_log=event_log, fixture_ready_recorded=fixture_ready_recorded,
            pick_goal_sent=True, place_goal_sent=True,
            place_goal_accepted=place_goal_accepted, goals_sent=2,
            pick_goal_id=pick_goal_id, place_goal_id=place_goal_id,
            task_result_status=place_status,
            terminal_status=place_result.get("terminal_status"),
            trigger=transport_trigger,
        )

    # -- Gate E evidence / artifacts ----------------------------------------

    def _evidence_invalid_e(
        self,
        scenario_id: str,
        reason_code: str,
        reasons: Sequence[str],
        *,
        handler: str | None = None,
        spec: Mapping[str, object] | None = None,
        post_grasp_lift_m_observed: object = None,
        pick_goal_sent: bool = False,
        place_goal_sent: bool = False,
        place_goal_accepted: bool = False,
        goals_sent: int = 0,
        pick_goal_id: object = None,
        place_goal_id: object = None,
        cleanup: Mapping[str, object] | None = None,
        trigger: Mapping[str, object] | None = None,
        controller_goal_sent: bool | None = None,
        controller_goal_uuid: object = None,
    ) -> dict[str, object]:
        """Task 6: E-stage evidence-invalid record with the E schema and durable rows.

        F2.1: an observed ``post_grasp_lift_m`` runtime parameter (when present)
        is persisted so pre-goal failures keep the observed lift evidence.
        F2.6: when an action goal was accepted, the truthful goal-state fields
        (pick/place goal-sent flags, goal IDs, goals sent, cleanup outcome) are
        persisted into every durable artifact; no row claims no goal was sent
        when one was accepted.
        F3.3: ``controller_goal_sent`` is true ONLY when an actual FJT
        transaction/status/UUID was observed for the attempt.  Accepting/canceling
        a Pick/Place goal, attempting task-goal cleanup, or observing the
        ``post_grasp_lift_m`` runtime parameter never implies a controller goal
        was sent.  When ``controller_goal_sent`` is not passed explicitly it is
        derived from a trigger carrying an FJT ``goal_uuid``.
        """
        if controller_goal_sent is None:
            controller_goal_sent = bool(
                trigger is not None and trigger.get("goal_uuid") is not None
            )
        controller_goal_sent = bool(controller_goal_sent)
        controller_endpoint = FJT_ENDPOINT if controller_goal_sent else None
        record: dict[str, object] = {
            "scenario_id": scenario_id,
            "stage": "E",
            "diagnostic_only": True,
            "physical_verdict": None,
            "status": "evidence-invalid",
            "reason_code": reason_code,
            "reasons": list(reasons),
            "pick_goal_sent": bool(pick_goal_sent),
            "place_goal_sent": bool(place_goal_sent),
            "place_goal_accepted": bool(place_goal_accepted),
            "goals_sent": int(goals_sent),
            "pick_goal_id": pick_goal_id,
            "place_goal_id": place_goal_id,
            "controller_goal_sent": bool(controller_goal_sent),
            "controller_goal_uuid": controller_goal_uuid,
            "controller_endpoint": controller_endpoint,
            "isaac_joint_commands_published": False,
        }
        if post_grasp_lift_m_observed is not None:
            record["post_grasp_lift_m_observed"] = dict(post_grasp_lift_m_observed)
        if cleanup is not None:
            record["cleanup"] = dict(cleanup)
        if trigger is not None:
            record["trigger"] = dict(trigger)
        if handler is not None:
            record["handler"] = handler
        try:
            row: dict[str, object] = {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "event": "gate-e",
                "stage": "E",
                "status": "evidence-invalid",
                "reason_code": reason_code,
                "reasons": list(reasons),
                "diagnostic_only": True,
                "pick_goal_sent": bool(pick_goal_sent),
                "place_goal_sent": bool(place_goal_sent),
                "place_goal_accepted": bool(place_goal_accepted),
                "controller_goal_sent": bool(controller_goal_sent),
                "controller_goal_uuid": controller_goal_uuid,
                "controller_endpoint": controller_endpoint,
                "isaac_joint_commands_published": False,
                "timestamp": float(time.monotonic()),
            }
            if post_grasp_lift_m_observed is not None:
                row["post_grasp_lift_m_observed"] = dict(post_grasp_lift_m_observed)
            if cleanup is not None:
                row["cleanup"] = dict(cleanup)
            if trigger is not None:
                row["trigger"] = dict(trigger)
            if handler is not None:
                row["handler"] = handler
            self._append_jsonl(self.attempt_dir / "integrated-execution.jsonl", row)
        except Exception:
            pass
        try:
            self._write_json_atomic(
                self.attempt_dir / "integrated-execution.json",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "stage": "E",
                    "handler": handler,
                    "diagnostic_only": True,
                    "physical_verdict": None,
                    "status": "evidence-invalid",
                    "reason_code": reason_code,
                    "reasons": list(reasons),
                    "pick_goal_sent": bool(pick_goal_sent),
                    "place_goal_sent": bool(place_goal_sent),
                    "place_goal_accepted": bool(place_goal_accepted),
                    "goals_sent": int(goals_sent),
                    "pick_goal_id": pick_goal_id,
                    "place_goal_id": place_goal_id,
                    "controller_goal_sent": bool(controller_goal_sent),
                    "controller_goal_uuid": controller_goal_uuid,
                    "controller_endpoint": controller_endpoint,
                    "post_grasp_lift_m_observed": (
                        dict(post_grasp_lift_m_observed)
                        if post_grasp_lift_m_observed is not None
                        else None
                    ),
                    "cleanup": dict(cleanup) if cleanup is not None else None,
                    "trigger": dict(trigger) if trigger is not None else None,
                    "isaac_joint_commands_published": False,
                },
            )
        except Exception:
            pass
        if spec is not None:
            self._write_e_pregoal_durable_rows(
                scenario_id, spec, record, handler=handler,
                task_result_status=None, terminal_status=None,
            )
        self._e_journal_failure(reason=reason_code, graph_diagnosis="E diagnostic journal")
        return record

    def _write_e_pregoal_durable_rows(
        self,
        scenario_id: str,
        spec: Mapping[str, object],
        record: Mapping[str, object],
        *,
        handler: str | None,
        task_result_status: object,
        terminal_status: str | None,
    ) -> None:
        """Write the non-execution durable E rows for an evidence-invalid path.

        F2.6: ``_evidence_invalid_e`` is used by the unexpected-exception path
        after a Pick/Place goal was accepted.  Every durable artifact must
        truthfully preserve the accepted-goal state (goal-sent flags, goal IDs,
        goals sent, cleanup/cancel outcome, ``status=evidence-invalid``); no row
        may claim no goal was sent when one was accepted.
        """
        try:
            self._append_jsonl(
                self.attempt_dir / "moveit-plans.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "goal_kind": spec.get("kind"),
                    "status": "evidence-invalid",
                    "row_kind": "lifecycle",
                    "pick_goal_sent": record.get("pick_goal_sent"),
                    "place_goal_sent": record.get("place_goal_sent"),
                    "diagnostic_only": True,
                },
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "controller-results.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "status": "evidence-invalid",
                    # F3.3: task-goal truth is preserved consistently here too —
                    # controller_goal_sent stays False even when a task goal was
                    # accepted (accepting/canceling a task goal is not controller
                    # traffic).
                    "pick_goal_sent": record.get("pick_goal_sent"),
                    "place_goal_sent": record.get("place_goal_sent"),
                    "place_goal_accepted": record.get("place_goal_accepted"),
                    "goals_sent": record.get("goals_sent"),
                    "pick_goal_id": record.get("pick_goal_id"),
                    "place_goal_id": record.get("place_goal_id"),
                    "controller_goal_sent": record.get("controller_goal_sent"),
                    "controller_goal_uuid": record.get("controller_goal_uuid"),
                    "controller_endpoint": FJT_ENDPOINT if record.get("controller_goal_sent") else None,
                    "gripper_goal_sent": False,
                    "task_result_status": task_result_status,
                    "terminal_status": terminal_status,
                    "diagnostic_only": True,
                },
            )
        except Exception:
            pass
        try:
            goal_path = self.attempt_dir / "goals" / f"{scenario_id}.json"
            goal_path.parent.mkdir(parents=True, exist_ok=True)
            _atomic_write_json(
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "handler": handler,
                    "stage": "E",
                    "diagnostic_only": True,
                    "physical_verdict": None,
                    "polarity": spec.get("polarity"),
                    "status": "evidence-invalid",
                    "reason_code": record.get("reason_code"),
                    "pick_goal_sent": record.get("pick_goal_sent"),
                    "place_goal_sent": record.get("place_goal_sent"),
                    "place_goal_accepted": record.get("place_goal_accepted"),
                    "goals_sent": record.get("goals_sent"),
                    "pick_goal_id": record.get("pick_goal_id"),
                    "place_goal_id": record.get("place_goal_id"),
                    "controller_goal_sent": record.get("controller_goal_sent"),
                    "controller_goal_uuid": record.get("controller_goal_uuid"),
                    "controller_endpoint": (
                        FJT_ENDPOINT if record.get("controller_goal_sent") else None
                    ),
                    "geometry": dict(spec.get("geometry") or {}),
                    "cleanup": dict(record.get("cleanup") or {}),
                    "trigger": dict(record.get("trigger") or {}),
                    "isaac_joint_commands_published": False,
                },
                goal_path,
            )
        except Exception:
            pass

    def _e_journal_failure(self, *, reason: str, graph_diagnosis: str) -> str:
        """Write the canonical E failure planning-scene.json, returning its status.

        F1.13: a pre-goal failure may leave the journal empty; ``finalize_failure``
        rejects that, but the fail-dominant artifact must still be written so no
        pre-goal ``evidence-invalid`` path omits ``planning-scene.json``.
        """
        try:
            self.journal.finalize_failure(
                reason=reason,
                graph_diagnosis=graph_diagnosis,
                json_path=self.attempt_dir / "planning-scene.json",
            )
            return "written"
        except ValueError as exc:
            if "empty PlanningScene journal" not in str(exc):
                return f"failed: {exc}"
            try:
                _atomic_write_json(
                    {
                        "schema_version": int(getattr(self.journal, "SCHEMA_VERSION", 1)),
                        "status": "evidence-invalid",
                        "authority": "physics_truth",
                        "reason": reason,
                        "graph_diagnosis": graph_diagnosis,
                        "events": [],
                        "records": [],
                        "graph": {},
                    },
                    self.attempt_dir / "planning-scene.json",
                )
                return "written"
            except Exception as write_exc:
                return f"failed: {write_exc}"
        except Exception as exc:
            return f"failed: {exc}"

    def _finalize_e_attempt(
        self,
        scenario_id: str,
        spec: Mapping[str, object],
        readiness: Mapping[str, object],
        start_wall: float,
        final_status: str,
        *,
        event_log: Sequence[str],
        fixture_ready_recorded: bool,
        task_error: str | None = None,
        pick_goal_sent: bool = False,
        place_goal_sent: bool = False,
        place_goal_accepted: bool = False,
        goals_sent: int = 0,
        pick_goal_id: object = None,
        place_goal_id: object = None,
        task_result_status: object = None,
        terminal_status: str | None = None,
        trigger: Mapping[str, object] | None = None,
        cancel_response: Mapping[str, object] | None = None,
        cleanup: Mapping[str, object] | None = None,
        fjt_goal_uuid: object = None,
        fjt_status: object = None,
    ) -> dict[str, object]:
        """Fail-dominant E attempt finalization and artifact write.

        ``diagnostic-pass`` requires the exact short E journal finalized with the
        observed graph projection; any journal/graph/artifact failure downgrades
        every already-created status-bearing E artifact to ``evidence-invalid``.
        Raw contact/lift/release/collision verdicts are never claimed here.
        """
        teardown_status = (
            "recorded" if event_log and list(event_log)[-1] == "teardown" else "not-recorded"
        )
        graph_status = "unavailable"
        if final_status == "diagnostic-pass":
            try:
                graph = self._graph_observation()
                if graph is None:
                    raise ValueError("observed graph evidence is unavailable")
                projection = build_journal_graph_projection(
                    fixture_payload=self._fixture_payload_for_graph(),
                    observed_graph=graph,
                )
                self.journal.finalize(
                    "diagnostic-pass",
                    graph=projection,
                    json_path=self.attempt_dir / "planning-scene.json",
                )
                graph_status = "validated"
            except Exception as exc:
                final_status = "evidence-invalid"
                graph_status = f"invalid: {exc}"
                if fixture_ready_recorded:
                    try:
                        self.journal.finalize_failure(
                            reason=task_error or f"E journal finalize failed: {exc}",
                            graph_diagnosis=graph_status,
                            json_path=self.attempt_dir / "planning-scene.json",
                        )
                    except Exception:
                        pass
        elif fixture_ready_recorded:
            try:
                self.journal.finalize_failure(
                    reason=task_error or "E attempt failed",
                    graph_diagnosis="E diagnostic journal",
                    json_path=self.attempt_dir / "planning-scene.json",
                )
            except Exception:
                pass
        result_status = task_result_status
        result_status_string = None
        if result_status is not None:
            try:
                result_status_string = _pick_place_result_name(result_status)
            except ValueError:
                result_status_string = None
        # F3.3: controller traffic is derived ONLY from actual observed FJT
        # evidence — a trigger carrying an FJT ``goal_uuid`` or an explicit
        # ``fjt_goal_uuid``.  A task terminal (executing/succeeded/canceled) or
        # task-goal cleanup never implies a controller goal was sent.
        controller_goal_sent = bool(
            (trigger is not None and trigger.get("goal_uuid") is not None)
            or fjt_goal_uuid is not None
        )
        # F3.3: keep the observed FJT goal UUID alongside the controller-truth
        # flags so every finalize record is consistent with ``_evidence_invalid_e``
        # (``controller_goal_sent``/``controller_goal_uuid``/``controller_endpoint``).
        controller_goal_uuid = fjt_goal_uuid
        if trigger is not None and trigger.get("goal_uuid") is not None:
            controller_goal_uuid = trigger.get("goal_uuid")
        if not controller_goal_sent:
            controller_goal_uuid = None
        record: dict[str, object] = {
            "scenario_id": scenario_id,
            "handler": spec.get("kind"),
            "stage": "E",
            "polarity": spec.get("polarity"),
            "diagnostic_only": True,
            "physical_verdict": None,
            "status": final_status,
            "reason_code": task_error,
            "pick_goal_sent": bool(pick_goal_sent),
            "place_goal_sent": bool(place_goal_sent),
            "place_goal_accepted": bool(place_goal_accepted),
            "goals_sent": int(goals_sent),
            "pick_goal_id": pick_goal_id,
            "place_goal_id": place_goal_id,
            "controller_goal_sent": bool(controller_goal_sent),
            "controller_goal_uuid": controller_goal_uuid,
            "controller_endpoint": FJT_ENDPOINT if controller_goal_sent else None,
            "gripper_goal_sent": False,
            "task_result_status": result_status,
            "task_result_status_string": result_status_string,
            "terminal_status": terminal_status,
            "fjt_goal_uuid": fjt_goal_uuid,
            "fjt_status": fjt_status,
            "event_log": list(event_log),
            "elapsed_s": round(time.monotonic() - start_wall, 6),
            "teardown": teardown_status,
            "graph": graph_status,
            "isaac_joint_commands_published": False,
        }
        if self._e_post_grasp_lift_m_observed is not None:
            record["post_grasp_lift_m_observed"] = dict(self._e_post_grasp_lift_m_observed)
        if trigger is not None:
            record["trigger"] = dict(trigger)
            if "goal_uuid" in trigger and record.get("fjt_goal_uuid") is None:
                record["fjt_goal_uuid"] = trigger.get("goal_uuid")
            if "status" in trigger and record.get("fjt_status") is None:
                record["fjt_status"] = trigger.get("status")
        if cancel_response is not None:
            record["cancel_response"] = cancel_response.get("response")
            record["cancel_return_code"] = cancel_response.get("return_code")
            record["cancel_goals_canceling"] = list(cancel_response.get("goals_canceling") or [])
            if cancel_response.get("error"):
                record["cancel_error"] = cancel_response.get("error")
        if cleanup is not None:
            record["cleanup"] = dict(cleanup)
        try:
            self._write_e_artifacts(scenario_id, spec, record, readiness)
        except Exception as exc:
            record["status"] = "evidence-invalid"
            record["reason_code"] = "artifact-write-failed"
            record["artifact_error"] = str(exc)
            self._downgrade_persisted_e_evidence(scenario_id, record, readiness, final_status)
        return record

    def _write_e_artifacts(
        self,
        scenario_id: str,
        spec: Mapping[str, object],
        record: Mapping[str, object],
        readiness: Mapping[str, object],
    ) -> None:
        """Write the E-stage artifact rows (``event=gate-e``, ``stage=E``)."""
        self._append_jsonl(
            self.attempt_dir / "integrated-execution.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "event": "gate-e",
                "stage": "E",
                "handler": spec.get("kind"),
                "polarity": spec.get("polarity"),
                "status": record.get("status"),
                "reason_code": record.get("reason_code"),
                "terminal_status": record.get("terminal_status"),
                "row_kind": "lifecycle",
                "diagnostic_only": True,
                "pick_goal_sent": record.get("pick_goal_sent"),
                "place_goal_sent": record.get("place_goal_sent"),
                "place_goal_accepted": record.get("place_goal_accepted"),
                "controller_goal_sent": record.get("controller_goal_sent"),
                "post_grasp_lift_m_observed": dict(record.get("post_grasp_lift_m_observed") or {}),
                "cleanup": dict(record.get("cleanup") or {}),
                "trigger": dict(record.get("trigger") or {}),
                "isaac_joint_commands_published": False,
                "timestamp": float(time.monotonic()),
            },
        )
        self._append_jsonl(
            self.attempt_dir / "moveit-plans.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "goal_kind": spec.get("kind"),
                "status": record.get("status"),
                "row_kind": "lifecycle",
                "pick_goal_sent": record.get("pick_goal_sent"),
                "place_goal_sent": record.get("place_goal_sent"),
                "diagnostic_only": True,
            },
        )
        self._append_jsonl(
            self.attempt_dir / "controller-results.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "controller_goal_sent": record.get("controller_goal_sent"),
                "controller_goal_uuid": record.get("controller_goal_uuid"),
                "controller_endpoint": record.get("controller_endpoint"),
                "gripper_goal_sent": record.get("gripper_goal_sent"),
                "task_result_status": record.get("task_result_status"),
                "task_result_status_string": record.get("task_result_status_string"),
                "fjt_goal_uuid": record.get("fjt_goal_uuid"),
                "fjt_status": record.get("fjt_status"),
                "terminal_status": record.get("terminal_status"),
                "diagnostic_only": True,
            },
        )
        self._append_visual_request("terminal", scenario_id, spec, kind="gate-e-diagnostic")
        self._write_json_atomic(
            self.attempt_dir / "integrated-execution.json",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "stage": "E",
                "handler": spec.get("kind"),
                "polarity": spec.get("polarity"),
                "diagnostic_only": True,
                "physical_verdict": None,
                "status": record.get("status"),
                "reason_code": record.get("reason_code"),
                "pick_goal_sent": record.get("pick_goal_sent"),
                "place_goal_sent": record.get("place_goal_sent"),
                "place_goal_accepted": record.get("place_goal_accepted"),
                "goals_sent": record.get("goals_sent"),
                "pick_goal_id": record.get("pick_goal_id"),
                "place_goal_id": record.get("place_goal_id"),
                "controller_goal_sent": record.get("controller_goal_sent"),
                "controller_goal_uuid": record.get("controller_goal_uuid"),
                "controller_endpoint": record.get("controller_endpoint"),
                "task_result_status": record.get("task_result_status"),
                "task_result_status_string": record.get("task_result_status_string"),
                "terminal_status": record.get("terminal_status"),
                "post_grasp_lift_m_observed": dict(record.get("post_grasp_lift_m_observed") or {}),
                "cleanup": dict(record.get("cleanup") or {}),
                "trigger": dict(record.get("trigger") or {}),
                "event_log": record.get("event_log"),
                "elapsed_s": record.get("elapsed_s"),
                "isaac_joint_commands_published": False,
            },
        )
        goal_path = self.attempt_dir / "goals" / f"{scenario_id}.json"
        goal_path.parent.mkdir(parents=True, exist_ok=True)
        _atomic_write_json(
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "handler": spec.get("kind"),
                "stage": "E",
                "diagnostic_only": True,
                "physical_verdict": None,
                "polarity": spec.get("polarity"),
                "status": record.get("status"),
                "reason_code": record.get("reason_code"),
                "pick_goal_sent": record.get("pick_goal_sent"),
                "place_goal_sent": record.get("place_goal_sent"),
                "place_goal_accepted": record.get("place_goal_accepted"),
                "goals_sent": record.get("goals_sent"),
                "pick_goal_id": record.get("pick_goal_id"),
                "place_goal_id": record.get("place_goal_id"),
                # F3.3: controller truth is preserved in the goal artifact too.
                "controller_goal_sent": record.get("controller_goal_sent"),
                "controller_goal_uuid": record.get("controller_goal_uuid"),
                "controller_endpoint": record.get("controller_endpoint"),
                "geometry": dict(spec.get("geometry") or {}),
                "post_grasp_lift_m_observed": dict(record.get("post_grasp_lift_m_observed") or {}),
                "cleanup": dict(record.get("cleanup") or {}),
                "trigger": dict(record.get("trigger") or {}),
                "event_log": record.get("event_log"),
                "isaac_joint_commands_published": False,
            },
            goal_path,
        )

    def _write_e_fail_dominant_execution_json(
        self,
        scenario_id: str,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        *,
        downgraded_from: object,
    ) -> None:
        self._write_json_atomic(
            self.attempt_dir / "integrated-execution.json",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "stage": "E",
                "diagnostic_only": True,
                "physical_verdict": None,
                "status": "evidence-invalid",
                "reason_code": record.get("reason_code", "artifact-write-failed"),
                "reasons": [str(record.get("artifact_error") or "E artifact final output failed")],
                "readiness": {
                    "ready": readiness.get("ready", False),
                    "reasons": readiness.get("reasons", []),
                },
                "pick_goal_sent": record.get("pick_goal_sent"),
                "place_goal_sent": record.get("place_goal_sent"),
                "place_goal_accepted": record.get("place_goal_accepted"),
                # F4.1: the authoritative fail-dominant E summary must preserve
                # controller truth exactly like the primary ``_write_e_artifacts``
                # summary.  Values come from the pre-downgrade truthful record:
                # controller true only with observed FJT evidence; UUID/endpoint
                # None when false.  A late artifact failure never erases the
                # controller/action/task identity the attempt actually observed.
                "controller_goal_sent": record.get("controller_goal_sent"),
                "controller_goal_uuid": record.get("controller_goal_uuid"),
                "controller_endpoint": record.get("controller_endpoint"),
                "post_grasp_lift_m_observed": dict(record.get("post_grasp_lift_m_observed") or {}),
                "cleanup": dict(record.get("cleanup") or {}),
                "trigger": dict(record.get("trigger") or {}),
                "downgraded_from": downgraded_from,
                "isaac_joint_commands_published": False,
            },
        )

    def _downgrade_persisted_e_evidence(
        self,
        scenario_id: str,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        downgraded_from: object,
    ) -> None:
        """After an E artifact write failure, downgrade every status stream."""
        try:
            self._write_e_fail_dominant_execution_json(
                scenario_id, record, readiness, downgraded_from=downgraded_from
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "integrated-execution.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "event": "gate-e",
                    "stage": "E",
                    "handler": record.get("handler"),
                    "status": "evidence-invalid",
                    "reason_code": record.get("reason_code"),
                    "terminal_status": record.get("terminal_status"),
                    "row_kind": "final",
                    "downgraded_from": downgraded_from,
                    "error": record.get("artifact_error"),
                    "diagnostic_only": True,
                    "pick_goal_sent": record.get("pick_goal_sent"),
                    "place_goal_sent": record.get("place_goal_sent"),
                    "place_goal_accepted": record.get("place_goal_accepted"),
                    # F4.1: the final downgrade row carries the full controller
                    # truth (uuid + endpoint alongside the sent flag) so the
                    # integrated-execution stream agrees with the summary and
                    # controller-results rows.
                    "controller_goal_sent": record.get("controller_goal_sent"),
                    "controller_goal_uuid": record.get("controller_goal_uuid"),
                    "controller_endpoint": record.get("controller_endpoint"),
                    "post_grasp_lift_m_observed": dict(record.get("post_grasp_lift_m_observed") or {}),
                    "cleanup": dict(record.get("cleanup") or {}),
                    "trigger": dict(record.get("trigger") or {}),
                    "isaac_joint_commands_published": False,
                    "timestamp": float(time.monotonic()),
                },
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "moveit-plans.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "goal_kind": record.get("handler"),
                    "status": "evidence-invalid",
                    "row_kind": "final",
                    "pick_goal_sent": record.get("pick_goal_sent"),
                    "place_goal_sent": record.get("place_goal_sent"),
                    "downgraded_from": downgraded_from,
                    "error": record.get("artifact_error"),
                    "diagnostic_only": True,
                },
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "controller-results.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "status": "evidence-invalid",
                    "row_kind": "final",
                    # F4.1: the final controller-results downgrade row preserves
                    # controller_goal_uuid alongside the sent flag and endpoint so
                    # no downgrade row drops the observed FJT identity.
                    "controller_goal_sent": record.get("controller_goal_sent"),
                    "controller_goal_uuid": record.get("controller_goal_uuid"),
                    "controller_endpoint": record.get("controller_endpoint"),
                    "gripper_goal_sent": record.get("gripper_goal_sent"),
                    "task_result_status": record.get("task_result_status"),
                    "task_result_status_string": record.get("task_result_status_string"),
                    "terminal_status": record.get("terminal_status"),
                    "post_grasp_lift_m_observed": dict(record.get("post_grasp_lift_m_observed") or {}),
                    "cleanup": dict(record.get("cleanup") or {}),
                    "trigger": dict(record.get("trigger") or {}),
                    "downgraded_from": downgraded_from,
                    "error": record.get("artifact_error"),
                    "diagnostic_only": True,
                },
            )
        except Exception:
            pass

    def _finalize_d_attempt(
        self,
        scenario_id: str,
        spec: Mapping[str, object],
        plan_outcome: Mapping[str, object] | None,
        planner_status: object,
        final_status: str,
        readiness: Mapping[str, object],
        start_wall: float,
        *,
        event_log: Sequence[str],
        planning_goal_id: object,
        fixture_ready_recorded: bool,
        execute_goal_id: object = None,
        execute_outcome: Mapping[str, object] | None = None,
        execute_error: str | None = None,
        fjt_evidence: object = None,
        fjt_goal_id: object = None,
        goals_canceling: Sequence[object] | None = None,
        terminal_status: str | None = None,
        trajectory_digest: str | None = None,
        plan_applicable: bool = True,
        controller_goal_sent: bool | None = None,
        controller_endpoint: str | None = None,
        cleanup: object = None,
        journal_issues: Sequence[str] | None = None,
        env_cloud_evidence: Mapping[str, object] | None = None,
        retreat_source: object = None,
        retreat_target: object = None,
        retreat_goal_id: object = None,
        gripper_command_records: Sequence[Mapping[str, object]] | None = None,
        native_action: bool = False,
        open_first: bool = True,
        cancel_response: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        """Fail-dominant D attempt finalization and artifact write.

        The authoritative ``status`` is computed after the transaction outcome
        *and* every required evidence finalization step.  A pass finalizes the
        D journal with the real observed-graph projection (Task-3 schema
        unchanged) and writes ``planning-scene.json``; a failure writes a
        canonical failure artifact.  Any write failure downgrades every
        already-created status-bearing D artifact.
        """
        # The flow methods record the complete D journal (including teardown)
        # on the pass path before finalization; a failed attempt records no
        # teardown and instead writes a canonical failure planning-scene.json.
        teardown_status = (
            "recorded"
            if event_log and list(event_log)[-1] == "teardown"
            else "not-recorded"
        )
        graph_status = "unavailable"
        if final_status == "diagnostic-pass":
            try:
                graph = self._graph_observation()
                if graph is None:
                    raise ValueError("observed graph evidence is unavailable")
                projection = build_journal_graph_projection(
                    fixture_payload=self._fixture_payload_for_graph(),
                    observed_graph=graph,
                )
                self.journal.finalize(
                    "diagnostic-pass",
                    graph=projection,
                    json_path=self.attempt_dir / "planning-scene.json",
                )
                graph_status = "validated"
            except Exception as exc:
                final_status = "evidence-invalid"
                graph_status = f"invalid: {exc}"
                if fixture_ready_recorded:
                    try:
                        self.journal.finalize_failure(
                            reason=execute_error or f"D journal finalize failed: {exc}",
                            graph_diagnosis=graph_status,
                            json_path=self.attempt_dir / "planning-scene.json",
                        )
                    except Exception:
                        pass
        elif fixture_ready_recorded:
            try:
                self.journal.finalize_failure(
                    reason=execute_error or "D attempt failed",
                    graph_diagnosis="D diagnostic journal",
                    json_path=self.attempt_dir / "planning-scene.json",
                )
            except Exception:
                pass
        executed_digest = trajectory_digest
        if execute_outcome is not None:
            result_status = execute_outcome.get("execute_result_status")
            try:
                result_status_string = _execute_status_name(result_status)
            except ValueError:
                result_status_string = None
        else:
            result_status = None
            result_status_string = None
        # F1.6/F2.7: truthful controller/execute traffic flags.  The
        # ExecuteTrajectory goal is sent only by the split-path execute handler
        # once a valid goal was accepted; cancel/safety observe a mid-flight
        # execute transaction (already accepted elsewhere) and never send one.
        # ``execute_goal_id`` existing in the execute outcome is the truthful
        # sent marker.  F2.7 pins ``controller_goal_sent`` to the EXACT FJT
        # semantic (a ``follow_joint_trajectory`` controller goal); it is True
        # for split-path execute and for cancel/safety observing a mid-flight
        # FJT transaction, and False for retreat/gripper.  Retreat and gripper
        # action traffic is surfaced through the dedicated generic/action fields
        # (``action_goal_sent``/``action_endpoint``/``cartesian_goal_sent``/
        # ``gripper_goal_sent``) so it is visible in ``controller-results.jsonl``
        # without pretending it was an FJT goal.
        execute_trajectory_goal_sent = bool(
            execute_outcome.get("execute_goal_id") if isinstance(execute_outcome, Mapping) else False
        )
        if controller_goal_sent is None:
            controller_goal_sent = bool(execute_trajectory_goal_sent or goals_canceling)
        cartesian_goal_sent = bool(retreat_goal_id is not None)
        gripper_goal_sent = bool(gripper_command_records is not None)
        action_goal_sent = bool(
            execute_trajectory_goal_sent or cartesian_goal_sent or gripper_goal_sent
        )
        if execute_trajectory_goal_sent:
            action_endpoint = EXECUTE_TRAJECTORY_ENDPOINT
        elif cartesian_goal_sent:
            action_endpoint = CARTESIAN_MOVE_ENDPOINT
        elif gripper_goal_sent:
            action_endpoint = GRIPPER_ENDPOINT
        else:
            action_endpoint = None
        record: dict[str, object] = {
            "scenario_id": scenario_id,
            "handler": spec.get("kind"),
            "stage": "D",
            "polarity": spec.get("polarity"),
            "diagnostic_only": True,
            "physical_verdict": None,
            "status": final_status,
            "planner_status": planner_status,
            "plan_applicable": bool(plan_applicable),
            "execute_trajectory_goal_sent": execute_trajectory_goal_sent,
            "controller_goal_sent": bool(controller_goal_sent),
            "controller_endpoint": controller_endpoint,
            "action_goal_sent": action_goal_sent,
            "action_endpoint": action_endpoint,
            "cartesian_goal_sent": cartesian_goal_sent,
            "gripper_goal_sent": gripper_goal_sent,
            "planning_goal_id": planning_goal_id,
            "execute_goal_id": execute_goal_id,
            "fjt_goal_id": fjt_goal_id if fjt_goal_id is not None else (
                fjt_evidence.get("goal_uuid") if isinstance(fjt_evidence, Mapping) else None
            ),
            "goals_canceling": list(goals_canceling) if goals_canceling is not None else [],
            "planned_trajectory_digest": plan_outcome.get("trajectory_digest") if isinstance(plan_outcome, Mapping) else None,
            "executed_trajectory_digest": executed_digest,
            "fjt_goal_digest": trajectory_digest,
            "fjt_goal_uuid": fjt_evidence.get("goal_uuid") if isinstance(fjt_evidence, Mapping) else None,
            "fjt_status": fjt_evidence.get("status") if isinstance(fjt_evidence, Mapping) else None,
            "execute_result_status": result_status,
            "execute_result_status_string": result_status_string,
            "terminal_status": terminal_status,
            "event_log": list(event_log),
            "elapsed_s": round(time.monotonic() - start_wall, 6),
            "teardown": teardown_status,
            "graph": graph_status,
            "isaac_joint_commands_published": False,
        }
        if cleanup is not None:
            record["cleanup"] = dict(cleanup)
        if cancel_response is not None:
            record["cancel_response"] = cancel_response.get("response")
            record["cancel_return_code"] = cancel_response.get("return_code")
            record["cancel_goals_canceling"] = list(cancel_response.get("goals_canceling") or [])
            if cancel_response.get("error"):
                record["cancel_error"] = cancel_response.get("error")
        if journal_issues:
            record["journal_issues"] = list(journal_issues)
        if env_cloud_evidence is not None:
            record["env_cloud_evidence"] = dict(env_cloud_evidence)
        if retreat_source is not None:
            record["source_pose"] = dict(retreat_source)
        if retreat_target is not None:
            record["target_pose"] = dict(retreat_target)
        if retreat_goal_id is not None:
            record["retreat_goal_id"] = retreat_goal_id
            record["endpoint"] = CARTESIAN_MOVE_ENDPOINT
            record["distance_m"] = RETREAT_DISTANCE_M
            record["axis"] = RETREAT_AXIS
            record["collision_checking"] = bool(env_cloud_evidence)
            record["command_gateway_bypassed"] = False
        if gripper_command_records is not None:
            record["commands"] = [item["command"] for item in gripper_command_records]
            record["command_records"] = list(gripper_command_records)
            record["goal_uuids"] = [item.get("goal_id") for item in gripper_command_records]
            record["native_action"] = native_action
            record["open_first"] = open_first
            record["endpoint"] = GRIPPER_ENDPOINT
            record["command_gateway_bypassed"] = False
        if execute_error is not None:
            record["execute_error"] = execute_error
        try:
            self._write_d_artifacts(scenario_id, spec, record, readiness)
        except Exception as exc:
            record["status"] = "evidence-invalid"
            record["reason_code"] = "artifact-write-failed"
            record["artifact_error"] = str(exc)
            self._downgrade_persisted_d_evidence(scenario_id, record, readiness, final_status)
        return record

    def _write_d_artifacts(
        self,
        scenario_id: str,
        spec: Mapping[str, object],
        record: Mapping[str, object],
        readiness: Mapping[str, object],
    ) -> None:
        """Write the D-stage artifact rows (separate shape; Gate-C bytes unchanged).

        F1.6: every D handler writes the complete authoritative set —
        ``integrated-execution.jsonl/.json``, ``moveit-plans.jsonl`` (with an
        explicit ``plan_applicable`` flag that is false for the non-MoveIt
        retreat/gripper handlers), ``controller-results.jsonl``, plus the
        scenario-specific ``goals/<scenario_id>.json`` goal artifact.  A failure
        in any required write propagates into the fail-dominant downgrade.
        """
        self._append_jsonl(
            self.attempt_dir / "integrated-execution.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "event": "gate-d",
                "stage": "D",
                "handler": spec.get("kind"),
                "polarity": spec.get("polarity"),
                "status": record.get("status"),
                "reason_code": record.get("reason_code"),
                "planner_status": record.get("planner_status"),
                "plan_applicable": record.get("plan_applicable"),
                "controller_endpoint": record.get("controller_endpoint"),
                "terminal_status": record.get("terminal_status"),
                "row_kind": "lifecycle",
                "diagnostic_only": True,
                "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                "controller_goal_sent": record.get("controller_goal_sent"),
                "isaac_joint_commands_published": False,
                "timestamp": float(time.monotonic()),
            },
        )
        self._append_jsonl(
            self.attempt_dir / "moveit-plans.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "goal_kind": spec.get("kind"),
                "status": record.get("status"),
                "planner_status": record.get("planner_status"),
                "plan_applicable": record.get("plan_applicable"),
                "row_kind": "lifecycle",
                "planning_goal_id": record.get("planning_goal_id"),
                "execute_goal_id": record.get("execute_goal_id"),
                "trajectory_digest": record.get("planned_trajectory_digest"),
                "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                "diagnostic_only": True,
            },
        )
        self._append_jsonl(
            self.attempt_dir / "controller-results.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "controller_goal_sent": record.get("controller_goal_sent"),
                "controller_endpoint": record.get("controller_endpoint"),
                "action_goal_sent": record.get("action_goal_sent"),
                "action_endpoint": record.get("action_endpoint"),
                "cartesian_goal_sent": record.get("cartesian_goal_sent"),
                "gripper_goal_sent": record.get("gripper_goal_sent"),
                "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                "execute_result_status": record.get("execute_result_status"),
                "execute_result_status_string": record.get("execute_result_status_string"),
                "fjt_goal_id": record.get("fjt_goal_id"),
                "fjt_goal_uuid": record.get("fjt_goal_uuid"),
                "fjt_status": record.get("fjt_status"),
                "fjt_goal_digest": record.get("fjt_goal_digest"),
                "terminal_status": record.get("terminal_status"),
                "diagnostic_only": True,
            },
        )
        self._write_json_atomic(
            self.attempt_dir / "integrated-execution.json",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "stage": "D",
                "handler": spec.get("kind"),
                "polarity": spec.get("polarity"),
                "diagnostic_only": True,
                "physical_verdict": None,
                "status": record.get("status"),
                "reason_code": record.get("reason_code"),
                "planner_status": record.get("planner_status"),
                "plan_applicable": record.get("plan_applicable"),
                "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                "controller_goal_sent": record.get("controller_goal_sent"),
                "controller_endpoint": record.get("controller_endpoint"),
                "planning_goal_id": record.get("planning_goal_id"),
                "execute_goal_id": record.get("execute_goal_id"),
                "goals_canceling": record.get("goals_canceling"),
                "cancel_response": record.get("cancel_response"),
                "cancel_return_code": record.get("cancel_return_code"),
                "cancel_goals_canceling": record.get("cancel_goals_canceling"),
                "planned_trajectory_digest": record.get("planned_trajectory_digest"),
                "executed_trajectory_digest": record.get("executed_trajectory_digest"),
                "fjt_goal_digest": record.get("fjt_goal_digest"),
                "fjt_goal_id": record.get("fjt_goal_id"),
                "fjt_goal_uuid": record.get("fjt_goal_uuid"),
                "fjt_status": record.get("fjt_status"),
                "execute_result_status": record.get("execute_result_status"),
                "execute_result_status_string": record.get("execute_result_status_string"),
                "terminal_status": record.get("terminal_status"),
                "cleanup": record.get("cleanup"),
                "journal_issues": record.get("journal_issues"),
                "env_cloud_evidence": record.get("env_cloud_evidence"),
                "event_log": record.get("event_log"),
                "elapsed_s": record.get("elapsed_s"),
                "isaac_joint_commands_published": False,
            },
        )
        # F1.6: scenario-specific goal artifact for retreat/gripper (the only D
        # handlers that send a goal without a MoveIt plan).
        retreat_goal_id = record.get("retreat_goal_id")
        if retreat_goal_id is not None:
            goal_path = self.attempt_dir / "goals" / f"{scenario_id}.json"
            goal_path.parent.mkdir(parents=True, exist_ok=True)
            _atomic_write_json(
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "handler": "retreat",
                    "stage": "D",
                    "diagnostic_only": True,
                    "physical_verdict": None,
                    "endpoint": CARTESIAN_MOVE_ENDPOINT,
                    "axis": RETREAT_AXIS,
                    "distance_m": RETREAT_DISTANCE_M,
                    "target_frame": "base_link",
                    "source_pose": dict(record.get("source_pose") or {}),
                    "target_pose": dict(record.get("target_pose") or {}),
                    "retreat_goal_id": retreat_goal_id,
                    "collision_checking": bool(record.get("env_cloud_evidence")),
                    "command_gateway_bypassed": False,
                    "isaac_joint_commands_published": False,
                },
                goal_path,
            )
        if record.get("command_records") is not None:
            goal_path = self.attempt_dir / "goals" / f"{scenario_id}.json"
            goal_path.parent.mkdir(parents=True, exist_ok=True)
            _atomic_write_json(
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "handler": "gripper",
                    "stage": "D",
                    "diagnostic_only": True,
                    "physical_verdict": None,
                    "endpoint": GRIPPER_ENDPOINT,
                    "commands": record.get("command_records"),
                    "native_action": bool(record.get("native_action")),
                    "open_first": bool(record.get("open_first")),
                    "isaac_joint_commands_published": False,
                },
                goal_path,
            )

    def _write_d_fail_dominant_execution_json(
        self,
        scenario_id: str,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        *,
        downgraded_from: object,
    ) -> None:
        self._write_json_atomic(
            self.attempt_dir / "integrated-execution.json",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "stage": "D",
                "diagnostic_only": True,
                "physical_verdict": None,
                "status": "evidence-invalid",
                "reason_code": record.get("reason_code", "artifact-write-failed"),
                "reasons": [str(record.get("artifact_error") or "D artifact final output failed")],
                "planner_status": record.get("planner_status"),
                "readiness": {
                    "ready": readiness.get("ready", False),
                    "reasons": readiness.get("reasons", []),
                },
                "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                "controller_goal_sent": record.get("controller_goal_sent"),
                "downgraded_from": downgraded_from,
                "isaac_joint_commands_published": False,
            },
        )

    def _downgrade_persisted_d_evidence(
        self,
        scenario_id: str,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        downgraded_from: object,
    ) -> None:
        """F3.1/F2.1: after a D artifact write failure, downgrade every already
        created status-bearing D artifact to evidence-invalid.

        F2.1: every status stream (``integrated-execution.jsonl``,
        ``moveit-plans.jsonl``, ``controller-results.jsonl``) receives a final
        corrective row with ``row_kind="final"`` and ``status="evidence-invalid"``
        preserving the planner/plan/controller/action/UUID/digest fields and the
        ``downgraded_from`` provenance.  A corrective append never claims pass.
        Each append is contained: a failed corrective write must not escape and
        must not mask the authoritative atomic summary (already written by
        ``_write_d_fail_dominant_execution_json``, itself fail-dominant)."""
        try:
            self._write_d_fail_dominant_execution_json(
                scenario_id, record, readiness, downgraded_from=downgraded_from
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "integrated-execution.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "event": "gate-d",
                    "stage": "D",
                    "handler": record.get("handler"),
                    "status": "evidence-invalid",
                    "reason_code": record.get("reason_code"),
                    "planner_status": record.get("planner_status"),
                    "plan_applicable": record.get("plan_applicable"),
                    "controller_endpoint": record.get("controller_endpoint"),
                    "row_kind": "final",
                    "downgraded_from": downgraded_from,
                    "error": record.get("artifact_error"),
                    "diagnostic_only": True,
                    "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                    "controller_goal_sent": record.get("controller_goal_sent"),
                    "isaac_joint_commands_published": False,
                    "timestamp": float(time.monotonic()),
                },
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "moveit-plans.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "goal_kind": record.get("handler"),
                    "status": "evidence-invalid",
                    "planner_status": record.get("planner_status"),
                    "plan_applicable": record.get("plan_applicable"),
                    "row_kind": "final",
                    "planning_goal_id": record.get("planning_goal_id"),
                    "execute_goal_id": record.get("execute_goal_id"),
                    "trajectory_digest": record.get("planned_trajectory_digest"),
                    "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                    "downgraded_from": downgraded_from,
                    "error": record.get("artifact_error"),
                    "diagnostic_only": True,
                },
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "controller-results.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "status": "evidence-invalid",
                    "row_kind": "final",
                    "controller_goal_sent": record.get("controller_goal_sent"),
                    "controller_endpoint": record.get("controller_endpoint"),
                    "action_goal_sent": record.get("action_goal_sent"),
                    "action_endpoint": record.get("action_endpoint"),
                    "cartesian_goal_sent": record.get("cartesian_goal_sent"),
                    "gripper_goal_sent": record.get("gripper_goal_sent"),
                    "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                    "execute_result_status": record.get("execute_result_status"),
                    "execute_result_status_string": record.get("execute_result_status_string"),
                    "fjt_goal_uuid": record.get("fjt_goal_uuid"),
                    "fjt_status": record.get("fjt_status"),
                    "fjt_goal_digest": record.get("fjt_goal_digest"),
                    "terminal_status": record.get("terminal_status"),
                    "downgraded_from": downgraded_from,
                    "error": record.get("artifact_error"),
                    "diagnostic_only": True,
                },
            )
        except Exception:
            pass

    def _acquire_scene(
        self,
        scenario_id: str,
        *,
        d_handler: str | None = None,
        e_handler: str | None = None,
    ) -> dict[str, object] | None:
        """F2.5/F3.2: bounded pre-goal scene acquisition through the private spinner.

        Acquisition requires a valid observation received after the last
        normalization failure (proved by ``scene_sequence`` exceeding the last
        invalid sequence), so a stale pre-invalid cached scene is never used.
        When the newest observation is invalid the executor spins up to the
        finite timeout to try to recover with a newer valid scene; timeout with
        no valid-after-invalid observation fails closed with zero goals.

        F2.3: when *d_handler* is supplied (a Stage-D handler kind), a failed
        acquisition returns the D schema/labels (``stage=D``, ``event=gate-d``,
        D controller/artifact shape) instead of the Gate-C ``_evidence_invalid``
        record.  Task 6 adds *e_handler* (a Stage-E handler kind) which returns
        the E schema/labels (``stage=E``, ``event=gate-e``, ``pick_goal_sent``/
        ``place_goal_sent`` flags) the same way.  Gate-C callers pass neither
        and keep the exact Gate-C bytes.
        """
        timeout_s = float(self._thresholds().get("scene_acquire_timeout_s", 5.0))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if not self._planning_scene_invalid:
                scene = self._latest_planning_scene
                if scene is not None and (
                    self._scene_invalid_sequence is None
                    or int(scene["scene_sequence"]) > self._scene_invalid_sequence
                ):
                    return None
            self._spin_once()
        if e_handler is not None:
            if self._planning_scene_invalid:
                return self._evidence_invalid_e(
                    scenario_id,
                    "planning-scene-invalid",
                    [
                        "a received PlanningScene failed normalization and no valid "
                        "scene arrived after it within the acquisition timeout"
                    ],
                    handler=e_handler,
                )
            return self._evidence_invalid_e(
                scenario_id,
                "no-planning-scene",
                [f"no valid PlanningScene cached within {timeout_s:.3f}s of self-spin"],
                handler=e_handler,
            )
        if d_handler is not None:
            if self._planning_scene_invalid:
                return self._evidence_invalid_d(
                    scenario_id,
                    "planning-scene-invalid",
                    [
                        "a received PlanningScene failed normalization and no valid "
                        "scene arrived after it within the acquisition timeout"
                    ],
                    handler=d_handler,
                )
            return self._evidence_invalid_d(
                scenario_id,
                "no-planning-scene",
                [f"no valid PlanningScene cached within {timeout_s:.3f}s of self-spin"],
                handler=d_handler,
            )
        if self._planning_scene_invalid:
            return self._evidence_invalid(
                scenario_id,
                "planning-scene-invalid",
                [
                    "a received PlanningScene failed normalization and no valid "
                    "scene arrived after it within the acquisition timeout"
                ],
            )
        return self._evidence_invalid(
            scenario_id,
            "no-planning-scene",
            [f"no valid PlanningScene cached within {timeout_s:.3f}s of self-spin"],
        )

    def _fixture_scene_error(
        self, scene: Mapping[str, object], *, allow_e_target: bool = False
    ) -> str | None:
        """F2.6/F3.3: return a reason when *scene* does not match the fixture contract.

        Beyond the exact ordered owned-ID set and empty attached set, the scene
        must carry the exact declared fixture geometry projection digest
        (primitive/mesh geometry, dimensions/scales, frame, and poses), so a
        stale full scene with the same IDs but an old cube pose is never
        labeled fixture-ready.

        F1.8: only an explicit Stage-E path may pass ``allow_e_target=True``,
        permitting the exact task-owned target ``pick_and_place/object_mesh`` in
        the world.  Gate C/D paths keep the strict fixture-only validation
        unchanged: any non-fixture world object — including an arbitrary
        task-namespace object — is a owned-set mismatch and fails the exact
        ordered check (same ``must equal`` message shape as the original).
        """
        declaration = _as_mapping(
            self.scenario.get("planning_scene_declaration") or self.scenario.get("planning_scene")
        )
        expected_ids = list(fixture_owned_ids(declaration))
        owned_ids = list(scene.get("owned_ids", []))
        allowed_world_ids = {TARGET_OBJECT_ID} if allow_e_target else set()
        # Task-namespace world objects are permitted only when they are the exact
        # allowed E target; every other non-fixture object stays in the owned set
        # and fails the exact ordered check below.
        task_world_ids = [
            object_id for object_id in owned_ids
            if object_id not in expected_ids
            and str(object_id).startswith(TASK_NAMESPACE)
            and object_id in allowed_world_ids
        ]
        owned_fixture_ids = [
            object_id for object_id in owned_ids if object_id not in task_world_ids
        ]
        if owned_fixture_ids != expected_ids:
            return (
                "fixture-ready owned_ids must equal the declared ordered fixture ids: "
                f"scene {owned_ids} != declared {expected_ids}"
            )
        attached_ids = list(scene.get("attached_ids", []))
        if attached_ids:
            return f"fixture-ready scene must not carry attached objects: {attached_ids}"
        expected_digest = self._expected_fixture_geometry_digest()
        observed_digest = scene.get("fixture_geometry_digest")
        if expected_digest is None or observed_digest != expected_digest:
            return (
                "fixture-ready scene geometry/pose must match the declared fixture "
                f"projection: scene {observed_digest} != declared {expected_digest}"
            )
        return None

    def _append_visual_request(
        self,
        phase: str,
        scenario_id: str,
        spec: Mapping[str, object],
        *,
        kind: str = "plan-only",
    ) -> None:
        """F2.7: durably append one visual-capture request record at the truthful phase.

        The default ``kind="plan-only"`` preserves the Gate-C byte shape exactly;
        D handlers pass ``kind="gate-d-diagnostic"`` (execution-neutral, never a
        fabricated planner capture).
        """
        self._append_jsonl(
            self.attempt_dir / "visual-capture-requests.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "phase": phase,
                "capture": {"kind": kind, "target": spec.get("target_pose")},
                "diagnostic_only": True,
            },
        )

    def _finalize_failure_artifact(self, reason: str, graph_diagnosis: str) -> str:
        """F2.1: write planning-scene.json as a canonical failure artifact."""
        try:
            self.journal.finalize_failure(
                reason=reason,
                graph_diagnosis=graph_diagnosis,
                json_path=self.attempt_dir / "planning-scene.json",
            )
            return "written"
        except Exception as exc:
            return f"failed: {exc}"

    def _evidence_invalid_after_fixture_ready(
        self,
        scenario_id: str,
        exc: Exception,
        spec: Mapping[str, object],
        readiness: Mapping[str, object],
    ) -> dict[str, object]:
        """F2.2: complete evidence for a failure after fixture-ready was recorded."""
        teardown_status = "not-recorded"
        later_join = self._join_key()
        if later_join is not None:
            try:
                self.journal.snapshot(
                    "teardown", frame_index=later_join[0], timestamp=later_join[1]
                )
                teardown_status = "recorded"
            except (ValueError, TypeError) as exc2:
                teardown_status = f"rejected: {exc2}"
        reason = f"unexpected-exception: {exc}"
        graph_status = "unavailable"
        self._finalize_failure_artifact(reason, graph_status)
        record = {
            "scenario_id": scenario_id,
            "status": "evidence-invalid",
            "reason_code": "unexpected-exception",
            "reasons": [reason],
            "planner_status": None,
            "teardown": teardown_status,
            "graph": graph_status,
            "goal_digest": None,
            "diagnostic_only": True,
            "execute_trajectory_goal_sent": False,
            "isaac_joint_commands_published": False,
        }
        try:
            self._write_artifacts(scenario_id, spec, None, record, readiness, graph_status)
        except Exception:
            # F2.2: artifact output must never escape; fall back to the durable
            # fail-dominant summary only.
            try:
                self._write_fail_dominant_execution_json(
                    scenario_id,
                    record,
                    readiness,
                    graph_status,
                    planner_status=None,
                    reason="artifact output failed after an unexpected exception",
                )
            except Exception:
                pass
        return record

    def _fixture_payload_for_graph(self) -> str:
        if self._fixture_payload is not None:
            return self._fixture_payload
        raise ValueError(
            "no canonical fixture payload was cached before journal finalization"
        )

    # -- artifacts -----------------------------------------------------------

    def _append_jsonl(self, path: Path, record: Mapping[str, object]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(dict(record), sort_keys=True, separators=(",", ":")) + "\n")
            stream.flush()
            os.fsync(stream.fileno())

    def _evidence_invalid(
        self, scenario_id: str, reason_code: str, reasons: Sequence[str]
    ) -> dict[str, object]:
        record = {
            "scenario_id": scenario_id,
            "status": "evidence-invalid",
            "reason_code": reason_code,
            "reasons": list(reasons),
            "diagnostic_only": True,
            "execute_trajectory_goal_sent": False,
            "isaac_joint_commands_published": False,
        }
        try:
            self._append_jsonl(
                self.attempt_dir / "integrated-execution.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "event": "gate-c-plan-only",
                    "status": "evidence-invalid",
                    "reason_code": reason_code,
                    "reasons": list(reasons),
                    "diagnostic_only": True,
                    "timestamp": float(time.monotonic()),
                },
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "controller-results.jsonl",
                {
                    "scenario_id": scenario_id,
                    "controller_goal_sent": False,
                    "execute_trajectory_goal_sent": False,
                    "diagnostic_only": True,
                },
            )
        except Exception:
            pass
        try:
            self._write_json_atomic(
                self.attempt_dir / "integrated-execution.json",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "diagnostic_only": True,
                    "status": "evidence-invalid",
                    "reason_code": reason_code,
                    "reasons": list(reasons),
                    "execute_trajectory_goal_sent": False,
                    "isaac_joint_commands_published": False,
                    "physical_verdict": None,
                },
            )
        except Exception:
            pass
        return record

    def _write_json_atomic(self, path: Path, value: Mapping[str, object]) -> None:
        _atomic_write_json(value, path)

    def _write_fail_dominant_execution_json(
        self,
        scenario_id: str,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        graph_status: str,
        *,
        planner_status: str | None,
        reason: str,
    ) -> None:
        """F2.1: durable fail-dominant execution summary when artifact output fails."""
        self._write_json_atomic(
            self.attempt_dir / "integrated-execution.json",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "diagnostic_only": True,
                "status": "evidence-invalid",
                "reason_code": record.get("reason_code", "artifact-write-failed"),
                "reasons": [reason],
                "planner_status": planner_status,
                "readiness": {
                    "ready": readiness.get("ready", False),
                    "reasons": readiness.get("reasons", []),
                },
                "graph": graph_status,
                "execute_trajectory_goal_sent": False,
                "isaac_joint_commands_published": False,
                "physical_verdict": None,
            },
        )

    def _downgrade_persisted_evidence(
        self,
        scenario_id: str,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        graph_status: str,
        planner_status: str | None,
        *,
        downgraded_from: object,
        goal_kind: object,
    ) -> None:
        """F3.1: after an artifact write fails post-provisional-pass, downgrade
        every already-created status-bearing artifact to evidence-invalid.

        Rewrites the atomic execution summary fail-dominantly, appends a
        corrective ``row_kind="final"`` row to each JSONL lifecycle stream so an
        early provisional row can never be mistaken for a completed pass, and
        writes planning-scene.json as a canonical failure artifact.  Every step
        is individually contained so no downgrade failure can escape the API.
        """
        try:
            self._write_fail_dominant_execution_json(
                scenario_id,
                record,
                readiness,
                graph_status,
                planner_status=planner_status,
                reason=str(record.get("artifact_error") or "artifact final output failed"),
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "integrated-execution.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "event": "gate-c-plan-only",
                    "status": "evidence-invalid",
                    "reason_code": record.get("reason_code"),
                    "planner_status": planner_status,
                    "row_kind": "final",
                    "downgraded_from": downgraded_from,
                    "error": record.get("artifact_error"),
                    "diagnostic_only": True,
                    "execute_trajectory_goal_sent": False,
                    "isaac_joint_commands_published": False,
                    "timestamp": float(time.monotonic()),
                },
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "moveit-plans.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "goal_kind": goal_kind,
                    "status": "evidence-invalid",
                    "planner_status": planner_status,
                    "row_kind": "final",
                    "downgraded_from": downgraded_from,
                    "error_code": record.get("error_code"),
                    "error_code_classification": record.get("error_code_classification"),
                    "nonempty_plan": record.get("nonempty_plan"),
                    "goal_digest": record.get("goal_digest"),
                    "trajectory_digest": record.get("trajectory_digest"),
                    "error": record.get("artifact_error"),
                    "diagnostic_only": True,
                },
            )
        except Exception:
            pass
        try:
            if self.journal.record_count > 0:
                self._finalize_failure_artifact(
                    str(record.get("artifact_error") or "artifact final output failed"),
                    graph_status,
                )
        except Exception:
            pass

    def _write_artifacts(
        self,
        scenario_id: str,
        spec: Mapping[str, object],
        goal: Any,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        graph_status: str,
    ) -> None:
        goal_digest = record.get("goal_digest")
        self._append_jsonl(
            self.attempt_dir / "integrated-execution.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "event": "gate-c-plan-only",
                "status": record.get("status"),
                "reason_code": record.get("reason_code"),
                "planner_status": record.get("planner_status"),
                "row_kind": "lifecycle",
                "diagnostic_only": True,
                "readiness": {
                    "ready": readiness.get("ready", False),
                    "reasons": readiness.get("reasons", []),
                },
                "graph": graph_status,
                "execute_trajectory_goal_sent": False,
                "isaac_joint_commands_published": False,
                "timestamp": float(time.monotonic()),
            },
        )
        self._append_jsonl(
            self.attempt_dir / "moveit-plans.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "goal_kind": spec.get("kind"),
                "status": record.get("status"),
                "planner_status": record.get("planner_status"),
                "row_kind": "lifecycle",
                "error_code": record.get("error_code"),
                "error_code_classification": record.get("error_code_classification"),
                "nonempty_plan": record.get("nonempty_plan"),
                "goal_digest": goal_digest,
                "trajectory_digest": record.get("trajectory_digest"),
                "diagnostic_only": True,
            },
        )
        self._append_jsonl(
            self.attempt_dir / "controller-results.jsonl",
            {
                "scenario_id": scenario_id,
                "controller_goal_sent": False,
                "execute_trajectory_goal_sent": False,
                "diagnostic_only": True,
            },
        )
        goal_path = self.attempt_dir / "goals" / f"{scenario_id}.json"
        goal_path.parent.mkdir(parents=True, exist_ok=True)
        _atomic_write_json(
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "kind": spec.get("kind"),
                "group_name": "xarm7",
                "pipeline_id": "ompl",
                "num_planning_attempts": 3,
                "allowed_planning_time": 3.0,
                "plan_only": True,
                "replan": False,
                "joints": spec.get("joints"),
                "target_pose": spec.get("target_pose"),
                "goal_digest": goal_digest,
                "diagnostic_only": True,
            },
            goal_path,
        )
        self._write_json_atomic(
            self.attempt_dir / "integrated-execution.json",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "diagnostic_only": True,
                "status": record.get("status"),
                "reason_code": record.get("reason_code"),
                "planner_status": record.get("planner_status"),
                "readiness": {
                    "ready": readiness.get("ready", False),
                    "reasons": readiness.get("reasons", []),
                },
                "goal": {
                    "kind": spec.get("kind"),
                    "group_name": "xarm7",
                    "pipeline_id": "ompl",
                    "num_planning_attempts": 3,
                    "allowed_planning_time": 3.0,
                    "plan_only": True,
                    "replan": False,
                    "goal_digest": goal_digest,
                },
                "result": {
                    "error_code": record.get("error_code"),
                    "error_code_classification": record.get("error_code_classification"),
                    "nonempty_plan": record.get("nonempty_plan"),
                    "trajectory_digest": record.get("trajectory_digest"),
                },
                "journal": {
                    "jsonl": str(self.attempt_dir / "planning-scene.jsonl"),
                    "json": str(self.attempt_dir / "planning-scene.json"),
                },
                "graph": graph_status,
                "execute_trajectory_goal_sent": False,
                "isaac_joint_commands_published": False,
                "physical_verdict": None,
            },
        )

    # -- teardown ------------------------------------------------------------

    def _destroy_owned_action_clients(self) -> None:
        """F1.1: destroy every real owned ActionClient exactly once.

        Humble ``Node.destroy_node()`` does not destroy action waitables; only
        ``ActionClient.destroy()`` removes them from the node waitable set.  The
        private ``_owned_action_clients`` collection keeps the real clients
        apart from the mutable ``_action_clients`` map tests may replace with
        fakes.  Any additional real owned client still present in the current
        map is destroyed once; test doubles lacking ``destroy`` are skipped.
        """
        owned = getattr(self, "_owned_action_clients", None) or []
        current = getattr(self, "_action_clients", None)
        destroyable: list[Any] = []
        for client in owned:
            if client is not None and all(client is not seen for seen in destroyable):
                destroyable.append(client)
        if isinstance(current, dict):
            for client in current.values():
                if client is None:
                    continue
                if all(client is not seen for seen in destroyable):
                    destroyable.append(client)
        for client in destroyable:
            destroy = getattr(client, "destroy", None)
            if not callable(destroy):
                continue
            try:
                destroy()
            except Exception:
                pass
        if owned:
            self._owned_action_clients = []
        if isinstance(current, dict):
            self._action_clients = {}

    def shutdown(self) -> None:
        """Idempotently destroy the node and shut down the executor-owned context.

        Order (F1.1): destroy every owned ActionClient (removing its waitable
        and C handle) before destroying the node and shutting down the private
        context; then clear lifecycle members so GC cannot double-finalize a
        destroyed context.  Repeated shutdown and construct→shutdown→construct
        remain supported.
        """
        if not self._context_initialized:
            return
        self._destroy_owned_action_clients()
        spinner = getattr(self, "_spinner", None)
        if spinner is not None:
            try:
                spinner.shutdown()
            except Exception:
                pass
        node = getattr(self, "node", None)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        context = getattr(self, "context", None)
        if context is not None:
            try:
                self.ros["rclpy"].shutdown(context=context)
            except Exception:
                pass
        self._context_initialized = False
        self.node = None
        self.context = None
        self._spinner = None
        self.operator_publisher = None
        self._service_clients = {}


def _service_type_to_ros(service_type: str, ros: Mapping[str, Any]) -> Any:
    """Map a canonical service type string to the imported generated class."""
    message_name = service_type.rsplit("/", 1)[-1]
    if service_type.startswith("controller_manager_msgs/srv/"):
        mapping = {
            "ListControllers": ros["ListControllers"],
            "LoadController": ros["LoadController"],
            "ConfigureController": ros["ConfigureController"],
            "SwitchController": ros["SwitchController"],
        }
    elif service_type.startswith("moveit_msgs/srv/"):
        mapping = {
            "GetPlanningScene": ros["GetPlanningScene"],
            "ApplyPlanningScene": ros["ApplyPlanningScene"],
            "GetStateValidity": ros["GetStateValidity"],
            "GetCartesianPath": ros["GetCartesianPath"],
        }
    elif service_type == "std_srvs/srv/Trigger":
        mapping = {"Trigger": ros["Trigger"]}
    elif service_type == "tinker_arm_msgs/srv/ArmJointService":
        mapping = {"ArmJointService": ros["ArmJointService"]}
    else:
        raise ValueError(f"unsupported service type: {service_type}")
    if message_name not in mapping:
        raise ValueError(f"unsupported service type: {service_type}")
    return mapping[message_name]


__all__ = [
    "ARTIFACT_JSONL_FILES",
    "CARTESIAN_MOVE_ENDPOINT",
    "CONTROLLER_MANAGER_NODE",
    "DIGEST",
    "D_FORBIDDEN_EVENTS",
    "EARLIER_OPERATION_OPTIONAL_FIELDS",
    "EXECUTE_STATUS_ABORTED",
    "EXECUTE_STATUS_CANCELED",
    "EXECUTE_STATUS_EXECUTING",
    "EXECUTE_STATUS_SUCCEEDED",
    "EXECUTE_TRAJECTORY_ENDPOINT",
    "FINAL_SIMULATION_STATE",
    "FIXTURE_OWNER",
    "FIXTURE_PUBLISHER_NODE",
    "FIXTURE_TARGET_HANDOFF",
    "FIXTURE_TOPIC",
    "FJT_ENDPOINT",
    "FJT_STATUS_CACHE_LIMIT",
    "FJT_STATUS_TOPIC",
    "GATE_C_FORBIDDEN_EVENTS",
    "GATE_C_REQUIRED_EVENT_ORDER",
    "GRIPPER_CLOSE_FIRST_EVENT_ORDER",
    "GRIPPER_CLOSE_POSITION",
    "GRIPPER_ENDPOINT",
    "GRIPPER_MAX_EFFORT",
    "GRIPPER_OPEN_POSITION",
    "IDENTITY_KEYS",
    "INTEGRATED_EXECUTION_PROFILE",
    "ISAAC_COMMAND_TOPIC",
    "JOINT_STATES_TOPIC",
    "JOURNAL_FIXTURE_TOPIC_QOS",
    "JOURNAL_PLANNING_SCENE_TOPIC_QOS",
    "JOURNAL_SERVICE_QOS",
    "JOURNAL_TOPIC_QOS",
    "MOVEIT_PLANNING_NON_SUCCESS_CODES",
    "MOVEIT_SUCCESS_CODE",
    "MOVE_GROUP_NODE",
    "NODE_BASENAME",
    "OPERATION_KEYS",
    "OPERATOR_NODE",
    "OPERATOR_NODE_NAMESPACE",
    "OPERATOR_TOPIC",
    "PHYSICS_READY_BOUNDARY",
    "PHYSICS_READY_GATE_NODE",
    "Q_OUTBOUND",
    "REPORT_KEYS",
    "REPORT_REVISION",
    "REQUIRED_ACTIONS",
    "REQUIRED_SERVICES",
    "REQUIRED_TOPICS",
    "RETREAT_AXIS",
    "RETREAT_DISTANCE_M",
    "RMW_IMPLEMENTATION",
    "SAFETY_STOP_TOPIC",
    "SAFETY_SUPERVISOR_NODE",
    "SIMULATION_STATE_PLAYING",
    "STAGE_C_SCENARIOS",
    "STAGE_D_EXPECTED_PHYSICAL",
    "STAGE_D_EXPECTED_POLARITY",
    "STAGE_D_KIND",
    "STAGE_D_REQUIRED_EVENT_ORDER",
    "STAGE_D_SCENARIOS",
    "STAGE_E_EXPECTED_NEGATIVE",
    "STAGE_E_EXPECTED_PHYSICAL",
    "STAGE_E_EXPECTED_POLARITY",
    "STAGE_E_FORBIDDEN_EVENTS",
    "STAGE_E_KIND",
    "STAGE_E_REQUIRED_EVENT_ORDER",
    "STAGE_E_SCENARIOS",
    "STAGE_E_TRIGGER_TIMEOUT_S",
    "TARGET_OBJECT_ID",
    "TASK_NAMESPACE",
    "_fjt_receipt_delta_s",
    "_fjt_within_receipt_window",
    "IntegratedGateExecutor",
    "build_cartesian_move_goal",
    "build_execute_trajectory_goal",
    "build_gripper_goal",
    "build_joint_move_group_goal",
    "build_journal_graph_projection",
    "build_pick_goal",
    "build_place_goal",
    "build_pose_move_group_goal",
    "derive_retreat_target_pose",
    "deterministic_cube_cloud",
    "evaluate_executor_readiness",
    "expected_fixture_geometry_digest",
    "expected_physics_ready_report",
    "stage_c_dispatch",
    "stage_d_dispatch",
    "stage_e_dispatch",
    "validate_physics_ready_snapshot",
]
