"""Test fixture helpers for the integrated gate verifier suite (ROS-free).

Task 7 fixture module.  Every helper creates complete raw/evaluator/executor/
scene/controller artifacts — never a reduced metric-only verdict input.  The
helpers are Python 3.12 ROS-free (no ``rclpy`` / generated messages / geometry
packages); all report identity, canonical digest, raw/evaluator correlation,
and window-selection tests run in the simulator venv.

``write_integrated_attempt`` builds a complete attempt directory for the
scenario implied by the named override (default ``qualification-pick-place-
positive``) and then applies exactly one named fault by mutating the artifact
that owns the metric.
"""
from __future__ import annotations

import copy
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))
sys.path.insert(0, str(ROOT / "tests"))

from qualification_test_helpers import load_test_scenario  # noqa: E402
from validation.integrated_gate_executor import (  # noqa: E402
    EXECUTE_STATUS_CANCELED,
    FJT_ENDPOINT,
    GRIPPER_ENDPOINT,
    Q_OUTBOUND,
    STAGE_D_KIND,
    STAGE_E_KIND,
    CARTESIAN_MOVE_ENDPOINT,
)
from validation.planning_scene_journal import (  # noqa: E402
    CANONICAL_LINK_TCP,
    CANONICAL_TARGET_HANDOFF,
    CANONICAL_TOUCH_LINKS,
)

PHYSICS_HZ = 120.0
ATTEMPT_ID = "attempt-1"
JOINT_NAMES = [f"joint{index}" for index in range(1, 8)] + ["drive_joint"]
HOME_JOINTS = [0.0] * 7
E_CUBE_START = [0.65, 0.0, 0.64]
E_TCP_GRASP = [0.65, 0.0, 0.72]
PLACE_REGION = [0.85, 0.0, 0.64]

#: Per-kind journal event frame indices (sim frame keys).
_EVENT_FRAMES: Mapping[str, list[tuple[str, int]]] = {
    "plan-joint": [("fixture-ready", 10), ("teardown", 40)],
    "plan-pose": [("fixture-ready", 10), ("teardown", 40)],
    "plan-blocked": [("fixture-ready", 10), ("teardown", 40)],
    "execute-joint": [
        ("fixture-ready", 10), ("execution-start", 20),
        ("execution-terminal", 60), ("teardown", 70),
    ],
    "execute-pose": [
        ("fixture-ready", 10), ("execution-start", 20),
        ("execution-terminal", 60), ("teardown", 70),
    ],
    "retreat": [
        ("fixture-ready", 10), ("retreat-start", 20),
        ("retreat-terminal", 60), ("teardown", 70),
    ],
    "gripper": [
        ("fixture-ready", 10), ("gripper-open-terminal", 30),
        ("gripper-close-terminal", 60), ("teardown", 70),
    ],
    "cancel": [
        ("fixture-ready", 10), ("execution-start", 20),
        ("cancel-requested", 40), ("quiescent", 60), ("teardown", 70),
    ],
    "safety": [
        ("fixture-ready", 10), ("execution-start", 20),
        ("effective-stop", 40), ("operator-clear", 50),
        ("quiescent", 60), ("teardown", 70),
    ],
    "positive": [
        ("fixture-ready", 10), ("before-pick", 20), ("scene-attach", 40),
        ("lift-complete", 60), ("transport", 80), ("before-release", 100),
        ("scene-detach", 110), ("released-settled", 130), ("teardown", 140),
    ],
    "blocked-approach": [
        ("fixture-ready", 10), ("before-pick", 20),
        ("pick-terminal", 40), ("teardown", 50),
    ],
    "unreachable-grasp": [
        ("fixture-ready", 10), ("before-pick", 20),
        ("pick-terminal", 40), ("teardown", 50),
    ],
    "malformed-back": [("fixture-ready", 10), ("teardown", 30)],
    "cancel-approach": [
        ("fixture-ready", 10), ("before-pick", 20), ("approach-start", 30),
        ("cancel-requested", 40), ("quiescent", 55), ("teardown", 60),
    ],
    "cancel-transport": [
        ("fixture-ready", 10), ("before-pick", 20), ("scene-attach", 40),
        ("lift-complete", 60), ("transport", 80), ("cancel-requested", 90),
        ("quiescent", 105), ("teardown", 110),
    ],
    "safety-transport": [
        ("fixture-ready", 10), ("before-pick", 20), ("scene-attach", 40),
        ("lift-complete", 60), ("transport", 80), ("effective-stop", 90),
        ("operator-clear", 95), ("quiescent", 105), ("teardown", 110),
    ],
    "occupied-place": [
        ("fixture-ready", 10), ("before-pick", 20), ("scene-attach", 40),
        ("lift-complete", 60), ("transport", 80), ("place-goal-accepted", 90),
        ("cancel-requested", 95), ("quiescent", 105), ("teardown", 110),
    ],
}

#: Scenario-specific expected_objects / measured object ids.
_OBJECTS_BY_SCENARIO: Mapping[str, list[str]] = {
    "qualification-pick-place-positive": ["qualification_cube"],
    "qualification-pick-place-blocked-approach": [
        "qualification_cube", "qualification_plan_blocker",
    ],
    "qualification-pick-place-unreachable-grasp": ["qualification_cube"],
    "qualification-pick-place-malformed-back": ["qualification_cube"],
    "qualification-pick-place-cancel-approach": ["qualification_cube"],
    "qualification-pick-place-cancel-transport": ["qualification_cube"],
    "qualification-pick-place-safety-transport": ["qualification_cube"],
    "qualification-pick-place-occupied-place": [
        "qualification_cube", "qualification_place_occupant",
    ],
}


def _implied_scenario(overrides: Mapping[str, Any]) -> str:
    """Map a named override to the scenario it implies (or the default).

    The brief's ``_verify`` helper passes the positive scenario by default and
    the named brief overrides are written against the positive scenario, so
    every unnamed override defaults to ``qualification-pick-place-positive``.
    Scenario-targeted overrides are always used with an explicit ``scenario``
    kwarg (matrix/§8 tests), which wins here.
    """
    explicit = overrides.get("scenario")
    if explicit:
        return str(explicit)
    if "occupied_release" in overrides:
        return "qualification-pick-place-occupied-place"
    return "qualification-pick-place-positive"


def load_test_config() -> dict[str, Any]:
    """Return the committed integrated config merged with resolved physics.hz."""
    config_path = ROOT / "simulation" / "qualification" / "integrated-ompl.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    core = json.loads((ROOT / config["core_config"]).read_text(encoding="utf-8"))
    merged = copy.deepcopy(config)
    merged["physics"] = {"hz": float(core["physics"]["hz"])}
    return merged


# --------------------------------------------------------------------------- #
# Raw frame building blocks
# --------------------------------------------------------------------------- #
def _contact(
    body_a: str,
    body_b: str,
    force: float = 3.0,
    point: list[float] | None = None,
) -> dict[str, Any]:
    return {
        "body_a": body_a,
        "body_b": body_b,
        "normal_force": force,
        "point": point or [0.0, 0.0, 0.0],
        "normal": [0.0, 0.0, 1.0],
    }


def _cube_object(xyz: list[float], twist: list[float]) -> dict[str, Any]:
    return {
        "id": "qualification_cube",
        "class_name": "cube",
        "prim_path": "/World/Scenario/qualification_cube",
        "pose": {"xyz": list(xyz), "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "twist": {"linear": list(twist), "angular": [0.0, 0.0, 0.0]},
    }


def _generic_object(object_id: str, xyz: list[float]) -> dict[str, Any]:
    return {
        "id": object_id,
        "class_name": str(object_id).rsplit("/", 1)[-1],
        "prim_path": f"/World/Scenario/{object_id}",
        "pose": {"xyz": list(xyz), "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "twist": {"linear": [0.0, 0.0, 0.0], "angular": [0.0, 0.0, 0.0]},
    }


def _frame(
    index: int,
    scenario_id: str,
    *,
    joints: list[float] | None = None,
    velocities: list[float] | None = None,
    tcp_xyz: list[float] | None = None,
    tcp_quat: list[float] | None = None,
    cube_xyz: list[float] | None = None,
    cube_twist: list[float] | None = None,
    contacts: list[dict[str, Any]] | None = None,
    safety: bool = False,
    command_joints: list[float] | None = None,
    objects: list[dict[str, Any]] | None = None,
    expected_objects: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    if joints is None:
        joints = [0.0] * 8
    if len(joints) == 7:
        joints = list(joints) + [0.0]
    if velocities is None:
        velocities = [0.0] * 8
    if command_joints is None:
        command_joints = list(joints)
    if tcp_xyz is None:
        tcp_xyz = [0.65, 0.0, 0.72]
    if tcp_quat is None:
        tcp_quat = [0.0, 0.0, 0.0, 1.0]
    if contacts is None:
        contacts = []
    if objects is None:
        objects = []
    if expected_objects is None:
        expected_objects = {}
    if cube_xyz is not None and cube_twist is None:
        cube_twist = [0.0, 0.0, 0.0]
    robot = {
        "base_pose": {"xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "tcp_pose": {"xyz": list(tcp_xyz), "quaternion_xyzw": list(tcp_quat)},
        "base_twist": {"linear": [0.0, 0.0, 0.0], "angular": [0.0, 0.0, 0.0]},
        "joint_names": list(JOINT_NAMES),
        "joint_positions": [float(value) for value in joints],
        "joint_velocities": [float(value) for value in velocities],
        "joint_efforts": [0.0] * 8,
        "safety_stop": bool(safety),
    }
    frame: dict[str, Any] = {
        "schema_version": 2,
        "frame_index": int(index),
        "timestamp": float(index) / PHYSICS_HZ,
        "scenario": scenario_id,
        "task": scenario_id,
        "robot": robot,
        "command_targets": {
            "joint_names": list(JOINT_NAMES),
            "joint_positions": [float(value) for value in command_joints],
            "joint_velocities": [0.0] * 8,
            "joint_efforts": [0.0] * 8,
            "snapshot_id": index,
            "gripper_effort_limit": 10.0,
        },
        "physics_device": "cpu",
        "seed": 7,
        "contacts": list(contacts),
        "contact_state": {},
        "contact_pairs": list(contacts),
        "expected_objects": dict(expected_objects),
        "objects": list(objects),
        "object": objects[0] if objects else None,
        "safety_stop": bool(safety),
        "actuator_limits": {"drive_joint": 10.0},
        "command_gateway": {
            "last_command_error": None,
            "command_stream_lost": False,
            "active_epoch": 1,
            "last_snapshot_id": None,
        },
    }
    return frame


def _fingers_contacts() -> list[dict[str, Any]]:
    return [
        _contact("/World/Tinker/left_finger", "/World/Scenario/qualification_cube", 3.0),
        _contact("/World/Tinker/right_finger", "/World/Scenario/qualification_cube", 3.0),
    ]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _state_at(
    scenario_id: str,
    kind: str,
    stage: str,
    eframes: list[tuple[str, int]],
    index: int,
) -> dict[str, Any]:
    """Return per-frame raw state overrides for the scenario kind."""
    fi = {event: frame for event, frame in eframes}
    state: dict[str, Any] = {}
    if stage == "C":
        return state  # fully static
    if kind in ("execute-joint",):
        start = fi.get("execution-start", 10)
        terminal = fi.get("execution-terminal", start + 1)
        if index >= start:
            joints = list(Q_OUTBOUND)
            state["joints"] = joints
            state["command_joints"] = joints
        return state
    if kind == "execute-pose":
        target_xyz = [0.45, 0.2, 0.89]
        target_quat = [0.0, 0.0, 0.382683, 0.92388]
        start = fi.get("execution-start", 10)
        if index >= start:
            state["tcp_xyz"] = target_xyz
            state["tcp_quat"] = target_quat
        return state
    if kind == "retreat":
        start = fi.get("retreat-start", 10)
        if index >= start:
            state["tcp_xyz"] = [0.65, 0.0, 0.84]
        return state
    if kind == "gripper":
        if index >= fi.get("gripper-close-terminal", 60):
            state["joints"] = [0.0] * 7 + [0.85]
            state["command_joints"] = [0.0] * 7 + [0.85]
        elif index >= fi.get("gripper-open-terminal", 30):
            state["joints"] = [0.0] * 7 + [0.0]
            state["command_joints"] = [0.0] * 7 + [0.0]
        return state
    if kind == "cancel":
        start = fi.get("execution-start", 20)
        cancel = fi.get("cancel-requested", 40)
        if cancel <= index:
            # Frozen at the cancel position; zero velocity.
            state["joints"] = [0.12, -0.12, 0.10, 0.20, -0.10, 0.12, 0.10, 0.0]
            state["command_joints"] = [0.12, -0.12, 0.10, 0.20, -0.10, 0.12, 0.10, 0.0]
        elif index >= start:
            frac = _clamp((index - start) / max(1, cancel - start), 0.0, 1.0)
            joints = [round(value * frac, 6) for value in (0.12, -0.12, 0.10, 0.20, -0.10, 0.12, 0.10)]
            state["joints"] = joints + [0.0]
            state["command_joints"] = joints + [0.0]
            state["velocities"] = [0.05] * 7 + [0.0]
        return state
    if kind == "safety":
        start = fi.get("execution-start", 20)
        effective = fi.get("effective-stop", 40)
        clear = fi.get("operator-clear", 50)
        if index >= clear:
            state["joints"] = [0.12, -0.12, 0.10, 0.20, -0.10, 0.12, 0.10, 0.0]
            state["command_joints"] = [0.12, -0.12, 0.10, 0.20, -0.10, 0.12, 0.10, 0.0]
            state["safety"] = False
        elif index >= effective:
            state["joints"] = [0.12, -0.12, 0.10, 0.20, -0.10, 0.12, 0.10, 0.0]
            state["command_joints"] = [0.12, -0.12, 0.10, 0.20, -0.10, 0.12, 0.10, 0.0]
            state["safety"] = True
        elif index >= start:
            frac = _clamp((index - start) / max(1, effective - start), 0.0, 1.0)
            joints = [round(value * frac, 6) for value in (0.12, -0.12, 0.10, 0.20, -0.10, 0.12, 0.10)]
            state["joints"] = joints + [0.0]
            state["command_joints"] = joints + [0.0]
            state["velocities"] = [0.05] * 7 + [0.0]
        return state
    # ---- Gate E (all kinds share cube/TCP helpers) ------------------------
    return _e_state_at(kind, fi, index)


def _e_state_at(kind: str, fi: Mapping[str, int], index: int) -> dict[str, Any]:
    """Gate E per-frame state (cube lift/transport/release + fingers)."""
    state: dict[str, Any] = {}
    cube_start = list(E_CUBE_START)
    if kind == "malformed-back":
        return state  # fully static, goal rejected pre-send
    attach = fi.get("scene-attach")
    detach = fi.get("scene-detach")
    if attach is None:
        # Non-attached E negatives: cube at rest, TCP may approach slightly.
        if kind in ("cancel-approach",):
            approach = fi.get("approach-start", 30)
            cancel = fi.get("cancel-requested", 40)
            if approach <= index < cancel:
                frac = _clamp((index - approach) / max(1, cancel - approach), 0.0, 1.0)
                state["tcp_xyz"] = [0.65, 0.0, 0.72 + 0.01 * frac]
        return state
    # Attached E kinds.
    lift_complete = fi.get("lift-complete", 60)
    transport = fi.get("transport", 80)
    before_release = fi.get("before-release")
    # Stop key for the attached negatives (cancel-transport / safety-transport /
    # occupied-place): transport halts at the first stop journal key.
    stop_key = None
    for stop_event in ("cancel-requested", "effective-stop", "place-goal-accepted", "operator-clear"):
        if stop_event in fi:
            stop_key = fi[stop_event]
            break
    if detach is not None and index >= detach:
        # Released / settled.
        state["cube_xyz"] = list(PLACE_REGION)
        state["cube_twist"] = [0.0, 0.0, 0.0]
        state["tcp_xyz"] = [0.85, 0.0, 0.72]
        state["contacts"] = []
        return state
    if index >= attach:
        x = cube_start[0]
        z = cube_start[2]
        if index < lift_complete:
            frac = _clamp((index - attach) / max(1, lift_complete - attach), 0.0, 1.0)
            z = cube_start[2] + 0.12 * frac
        elif index < transport:
            z = cube_start[2] + 0.12
        else:
            end_x_key = before_release if before_release is not None else stop_key
            # occupied-place stops short of the occupied region (the forbidden
            # target_region_settled predicate requires final_radial > 0.06).
            x_end = 0.77 if kind == "occupied-place" else 0.85
            span = x_end - cube_start[0]
            if end_x_key is None or index >= end_x_key:
                x = cube_start[0] + span
            else:
                frac = _clamp((index - transport) / max(1, end_x_key - transport), 0.0, 1.0)
                x = cube_start[0] + span * frac
            z = cube_start[2] + 0.12
        cube = [x, 0.0, z]
        state["cube_xyz"] = cube
        state["cube_twist"] = [0.0, 0.0, 0.0]
        state["tcp_xyz"] = [x, 0.0, z + 0.08]
        if kind == "safety-transport":
            effective = fi.get("effective-stop", 90)
            clear = fi.get("operator-clear", 95)
            state["safety"] = bool(effective <= index < clear)
    # Bilateral finger/cube contact strictly precedes the scene-attach journal
    # join key (Task-7 rule): contacts begin a few frames before attach.
    if index >= attach - 5:
        state["contacts"] = _fingers_contacts()
    else:
        state["contacts"] = []
    return state


def _objects_for(scenario_id: str, kind: str, stage: str) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    object_ids = list(_OBJECTS_BY_SCENARIO.get(scenario_id, []))
    # The integrated verifier unconditionally requires a qualification_cube in
    # the pre-start frame for every stage; C/D gates also carry the sim-fixture
    # pick target at rest in raw truth.
    if "qualification_cube" not in object_ids:
        object_ids = ["qualification_cube", *object_ids]
    objects: list[dict[str, Any]] = []
    for object_id in object_ids:
        if object_id == "qualification_cube":
            objects.append(_cube_object(list(E_CUBE_START), [0.0, 0.0, 0.0]))
        else:
            objects.append(_generic_object(object_id, [0.65, 0.0, 0.60]))
    expected: dict[str, Any] = {}
    for object_id in object_ids:
        expected[object_id] = {
            "class_name": "cube" if object_id == "qualification_cube" else str(object_id),
            "actual_prim_path": f"/World/Scenario/{object_id}",
        }
    return objects, expected


# --------------------------------------------------------------------------- #
# Journal builder
# --------------------------------------------------------------------------- #
def _scene_record(
    event: str,
    sequence: int,
    frame_index: int,
    owned_ids: list[str],
    attached_ids: list[str],
    fixture_revision: str,
) -> dict[str, Any]:
    attached_links: dict[str, str] = {}
    touch_links: dict[str, list[str]] = {}
    if CANONICAL_TARGET_HANDOFF in attached_ids:
        attached_links[CANONICAL_TARGET_HANDOFF] = CANONICAL_LINK_TCP
        touch_links[CANONICAL_TARGET_HANDOFF] = list(CANONICAL_TOUCH_LINKS)
    return {
        "event": event,
        "journal_sequence": sequence,
        "frame_index": int(frame_index),
        "timestamp": float(frame_index) / PHYSICS_HZ,
        "scene_sequence": sequence,
        "scene_timestamp": float(sequence) * 0.1,
        "scene_revision_digest": "a" * 64,
        "owned_ids": list(owned_ids),
        "attached_ids": list(attached_ids),
        "attached_links": attached_links,
        "touch_links": touch_links,
        "fixture_revision": fixture_revision,
        "acm_digest": "b" * 64,
        "robot_state_digest": "c" * 64,
        "source": "/planning_scene",
    }


def _build_journal(
    scenario_id: str,
    kind: str,
    stage: str,
    eframes: list[tuple[str, int]],
    owned_ids: list[str],
    fixture_revision: str,
) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    attached = False
    detach_pending = False
    for sequence, (event, frame_index) in enumerate(eframes, 1):
        if event == "scene-attach":
            attached = True
        elif event == "scene-detach":
            # The scene-detach journal key itself still carries the attached
            # target (the retained phase ends strictly after that record).
            detach_pending = True
        elif detach_pending:
            attached = False
        attached_ids = [CANONICAL_TARGET_HANDOFF] if attached else []
        records.append(
            _scene_record(
                event,
                sequence,
                frame_index,
                owned_ids,
                attached_ids,
                fixture_revision,
            )
        )
    return records


# --------------------------------------------------------------------------- #
# Executor / controller / moveit / goal artifacts
# --------------------------------------------------------------------------- #
def _build_executor_artifacts(
    scenario_id: str,
    kind: str,
    stage: str,
    polarity: str,
    status: str,
    *,
    planner_status: str = "success",
    execute_goal_sent: bool = True,
    controller_goal_sent: bool = True,
    terminal_status: str = "succeeded",
    execute_result_status: int = 4,
    task_result_status: int | None = 0,
    pick_goal_sent: bool = True,
    place_goal_sent: bool = False,
    place_goal_accepted: bool = False,
    plan_applicable: bool = True,
    nonempty_plan: bool = True,
    error_code: int | None = 1,
    trajectory_digest: str = "d" * 64,
) -> tuple[list[dict[str, Any]], dict[str, Any], list[dict[str, Any]], list[dict[str, Any]], dict[str, Any] | None]:
    """Return (execution_jsonl, execution_json, moveit_plans, controller_rows, goal)."""
    execution_jsonl: list[dict[str, Any]] = []
    if stage == "C":
        execution_jsonl.append(
            {
                "schema_version": 1,
                "report_revision": 7,
                "scenario_id": scenario_id,
                "event": "gate-c-plan-only",
                "status": status,
                "reason_code": None,
                "planner_status": planner_status,
                "row_kind": "lifecycle",
                "diagnostic_only": True,
                "readiness": {"ready": True, "reasons": []},
                "graph": "validated",
                "execute_trajectory_goal_sent": False,
                "isaac_joint_commands_published": False,
                "timestamp": 100.0,
            }
        )
    elif stage == "D":
        execution_jsonl.append(
            {
                "schema_version": 1,
                "report_revision": 7,
                "scenario_id": scenario_id,
                "event": "gate-d",
                "stage": "D",
                "handler": kind,
                "polarity": polarity,
                "status": status,
                "reason_code": None,
                "planner_status": planner_status,
                "plan_applicable": plan_applicable,
                "controller_endpoint": FJT_ENDPOINT,
                "terminal_status": terminal_status,
                "row_kind": "lifecycle",
                "diagnostic_only": True,
                "execute_trajectory_goal_sent": execute_goal_sent,
                "controller_goal_sent": controller_goal_sent,
                "isaac_joint_commands_published": False,
                "timestamp": 100.0,
            }
        )
    else:
        execution_jsonl.append(
            {
                "schema_version": 1,
                "report_revision": 7,
                "scenario_id": scenario_id,
                "event": "gate-e",
                "stage": "E",
                "handler": kind,
                "polarity": polarity,
                "status": status,
                "reason_code": None,
                "terminal_status": terminal_status,
                "row_kind": "lifecycle",
                "diagnostic_only": True,
                "pick_goal_sent": pick_goal_sent,
                "place_goal_sent": place_goal_sent,
                "place_goal_accepted": place_goal_accepted,
                "controller_goal_sent": controller_goal_sent,
                "post_grasp_lift_m_observed": {},
                "cleanup": {},
                "trigger": {},
                "isaac_joint_commands_published": False,
                "timestamp": 100.0,
            }
        )

    moveit_plans: list[dict[str, Any]] = []
    if stage == "C":
        moveit_plans.append(
            {
                "schema_version": 1,
                "report_revision": 7,
                "scenario_id": scenario_id,
                "goal_kind": kind,
                "status": status,
                "planner_status": planner_status,
                "row_kind": "lifecycle",
                "error_code": error_code,
                "error_code_classification": "success" if nonempty_plan else "failure",
                "nonempty_plan": nonempty_plan,
                "goal_digest": "e" * 64,
                "trajectory_digest": trajectory_digest if nonempty_plan else None,
                "diagnostic_only": True,
            }
        )
    elif stage == "D":
        moveit_plans.append(
            {
                "schema_version": 1,
                "report_revision": 7,
                "scenario_id": scenario_id,
                "goal_kind": kind,
                "status": status,
                "planner_status": planner_status,
                "plan_applicable": plan_applicable,
                "row_kind": "lifecycle",
                "planning_goal_id": "goal-1",
                "execute_goal_id": "execute-1",
                "trajectory_digest": trajectory_digest,
                "execute_trajectory_goal_sent": execute_goal_sent,
                "diagnostic_only": True,
            }
        )
    else:
        moveit_plans.append(
            {
                "schema_version": 1,
                "report_revision": 7,
                "scenario_id": scenario_id,
                "goal_kind": kind,
                "status": status,
                "row_kind": "lifecycle",
                "pick_goal_sent": pick_goal_sent,
                "place_goal_sent": place_goal_sent,
                "diagnostic_only": True,
            }
        )

    controller_rows: list[dict[str, Any]] = []
    if stage == "C":
        controller_rows.append(
            {
                "schema_version": 1,
                "report_revision": 7,
                "scenario_id": scenario_id,
                "controller_goal_sent": False,
                "execute_trajectory_goal_sent": False,
                "diagnostic_only": True,
            }
        )
    elif stage == "D":
        controller_rows.append(
            {
                "schema_version": 1,
                "report_revision": 7,
                "scenario_id": scenario_id,
                "controller_goal_sent": controller_goal_sent,
                "controller_endpoint": FJT_ENDPOINT,
                "action_goal_sent": execute_goal_sent,
                "action_endpoint": "/execute_trajectory",
                "execute_trajectory_goal_sent": execute_goal_sent,
                "execute_result_status": execute_result_status,
                "execute_result_status_string": str(terminal_status),
                "fjt_status": str(terminal_status),
                "terminal_status": terminal_status,
                "diagnostic_only": True,
            }
        )
    else:
        controller_rows.append(
            {
                "schema_version": 1,
                "report_revision": 7,
                "scenario_id": scenario_id,
                "controller_goal_sent": controller_goal_sent,
                "controller_goal_uuid": "uuid-1",
                "controller_endpoint": GRIPPER_ENDPOINT,
                "gripper_goal_sent": True,
                "task_result_status": task_result_status,
                "task_result_status_string": "success" if task_result_status == 0 else "failure",
                "fjt_status": str(terminal_status),
                "terminal_status": terminal_status,
                "diagnostic_only": True,
            }
        )

    execution_json: dict[str, Any] = {}
    if stage == "C":
        execution_json = {
            "schema_version": 1,
            "report_revision": 7,
            "scenario_id": scenario_id,
            "diagnostic_only": True,
            "status": status,
            "reason_code": None,
            "planner_status": planner_status,
            "readiness": {"ready": True, "reasons": []},
            "goal": {"kind": kind, "group_name": "xarm7", "pipeline_id": "ompl",
                     "num_planning_attempts": 3, "allowed_planning_time": 3.0,
                     "plan_only": True, "replan": False, "goal_digest": "e" * 64},
            "result": {"error_code": error_code,
                       "error_code_classification": "success" if nonempty_plan else "failure",
                       "nonempty_plan": nonempty_plan,
                       "trajectory_digest": trajectory_digest if nonempty_plan else None},
            "journal": {"jsonl": "planning-scene.jsonl", "json": "planning-scene.json"},
            "graph": "validated",
            "execute_trajectory_goal_sent": False,
            "isaac_joint_commands_published": False,
            "physical_verdict": None,
        }
    elif stage == "D":
        execution_json = {
            "schema_version": 1,
            "report_revision": 7,
            "scenario_id": scenario_id,
            "stage": "D",
            "handler": kind,
            "polarity": polarity,
            "diagnostic_only": True,
            "physical_verdict": None,
            "status": status,
            "reason_code": None,
            "planner_status": planner_status,
            "plan_applicable": plan_applicable,
            "execute_trajectory_goal_sent": execute_goal_sent,
            "controller_goal_sent": controller_goal_sent,
            "controller_endpoint": FJT_ENDPOINT,
            "planning_goal_id": "goal-1",
            "execute_goal_id": "execute-1",
            "goals_canceling": [],
            "cancel_response": None,
            "cancel_return_code": None,
            "cancel_goals_canceling": [],
            "planned_trajectory_digest": trajectory_digest,
            "executed_trajectory_digest": trajectory_digest,
            "fjt_goal_digest": trajectory_digest,
            "fjt_goal_uuid": "uuid-1",
            "fjt_status": str(terminal_status),
            "execute_result_status": execute_result_status,
            "execute_result_status_string": str(terminal_status),
            "terminal_status": terminal_status,
            "cleanup": {},
            "journal_issues": [],
            "env_cloud_evidence": {},
            "event_log": [],
            "elapsed_s": 1.0,
            "isaac_joint_commands_published": False,
        }
    else:
        execution_json = {
            "schema_version": 1,
            "report_revision": 7,
            "scenario_id": scenario_id,
            "stage": "E",
            "handler": kind,
            "polarity": polarity,
            "diagnostic_only": True,
            "physical_verdict": None,
            "status": status,
            "reason_code": None,
            "pick_goal_sent": pick_goal_sent,
            "place_goal_sent": place_goal_sent,
            "place_goal_accepted": place_goal_accepted,
            "goals_sent": list("pick" if pick_goal_sent else ""),
            "pick_goal_id": "pick-1" if pick_goal_sent else None,
            "place_goal_id": "place-1" if place_goal_sent else None,
            "controller_goal_sent": controller_goal_sent,
            "controller_goal_uuid": "uuid-1",
            "controller_endpoint": GRIPPER_ENDPOINT,
            "task_result_status": task_result_status,
            "task_result_status_string": "success" if task_result_status == 0 else "failure",
            "terminal_status": terminal_status,
            "post_grasp_lift_m_observed": {},
            "cleanup": {},
            "trigger": {},
            "event_log": [],
            "elapsed_s": 1.0,
            "isaac_joint_commands_published": False,
        }

    goal: dict[str, Any] | None = None
    if stage == "C":
        goal = {
            "schema_version": 1,
            "report_revision": 7,
            "scenario_id": scenario_id,
            "kind": kind,
            "group_name": "xarm7",
            "pipeline_id": "ompl",
            "num_planning_attempts": 3,
            "allowed_planning_time": 3.0,
            "plan_only": True,
            "replan": False,
            "joints": list(Q_OUTBOUND) if kind == "plan-joint" else None,
            "target_pose": None,
            "goal_digest": "e" * 64,
            "diagnostic_only": True,
        }
    elif stage == "D" and kind in ("retreat", "gripper"):
        if kind == "retreat":
            goal = {
                "schema_version": 1,
                "report_revision": 7,
                "scenario_id": scenario_id,
                "handler": "retreat",
                "stage": "D",
                "diagnostic_only": True,
                "physical_verdict": None,
                "endpoint": CARTESIAN_MOVE_ENDPOINT,
                "axis": "+z",
                "distance_m": 0.10,
                "target_frame": "base_link",
                "source_pose": {"xyz": [0.65, 0.0, 0.72], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                "target_pose": {"xyz": [0.65, 0.0, 0.84], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                "retreat_goal_id": "retreat-1",
                "collision_checking": True,
                "command_gateway_bypassed": False,
                "isaac_joint_commands_published": False,
            }
        else:
            goal = {
                "schema_version": 1,
                "report_revision": 7,
                "scenario_id": scenario_id,
                "handler": "gripper",
                "stage": "D",
                "diagnostic_only": True,
                "physical_verdict": None,
                "endpoint": GRIPPER_ENDPOINT,
                "commands": [],
                "native_action": True,
                "open_first": True,
                "isaac_joint_commands_published": False,
            }
    elif stage == "E":
        goal = {
            "schema_version": 1,
            "report_revision": 7,
            "scenario_id": scenario_id,
            "handler": kind,
            "stage": "E",
            "diagnostic_only": True,
            "physical_verdict": None,
            "polarity": polarity,
            "status": status,
            "reason_code": None,
            "pick_goal_sent": pick_goal_sent,
            "place_goal_sent": place_goal_sent,
            "place_goal_accepted": place_goal_accepted,
            "goals_sent": list("pick" if pick_goal_sent else ""),
            "pick_goal_id": "pick-1" if pick_goal_sent else None,
            "place_goal_id": "place-1" if place_goal_sent else None,
            "controller_goal_sent": controller_goal_sent,
            "controller_goal_uuid": "uuid-1",
            "controller_endpoint": GRIPPER_ENDPOINT,
            "geometry": {},
            "post_grasp_lift_m_observed": {},
            "cleanup": {},
            "trigger": {},
            "event_log": [],
            "isaac_joint_commands_published": False,
        }
    return execution_jsonl, execution_json, moveit_plans, controller_rows, goal


def _manifest(scenario_id: str, seed: int) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "attempt_id": ATTEMPT_ID,
        "gate": scenario_id,
        "scenario": {"id": scenario_id, "seed": seed},
        "config": {"profile": "integrated-ompl"},
    }


# --------------------------------------------------------------------------- #
# write_integrated_attempt
# --------------------------------------------------------------------------- #
def write_integrated_attempt(root: Path, **overrides: Any) -> Path:
    """Create a complete attempt directory and apply one named fault.

    Returns the created attempt directory path.
    """
    root = Path(root)
    scenario_id = _implied_scenario(overrides)
    bundle = load_test_scenario(scenario_id)
    integrated = bundle["integrated"]
    stage = str(integrated["stage"])
    polarity = str(integrated["acceptance"]["polarity"])
    seed = int(bundle["scenario"]["seed"])
    kind = _kind_for(scenario_id, stage)
    eframes = _EVENT_FRAMES[kind]

    attempt = root / ATTEMPT_ID
    attempt.mkdir(parents=True, exist_ok=True)

    owned_ids = list(integrated["expected_scene"]["owned_ids"])
    fixture_revision = str(bundle["planning_scene_declaration"]["revision"])

    # --- Raw frames ---------------------------------------------------------
    max_frame = max(frame for _, frame in eframes) + 10
    raw_frames_list: list[dict[str, Any]] = []
    for index in range(max_frame + 1):
        state = _state_at(scenario_id, kind, stage, eframes, index)
        objects, expected_objects = _objects_for(scenario_id, kind, stage)
        cube_xyz = state.get("cube_xyz")
        cube_twist = state.get("cube_twist")
        objects = copy.deepcopy(objects)
        if cube_xyz is not None:
            for obj in objects:
                if obj.get("id") == "qualification_cube":
                    obj["pose"]["xyz"] = list(cube_xyz)
                    obj["twist"]["linear"] = list(cube_twist or [0.0, 0.0, 0.0])
        frame = _frame(
            index,
            scenario_id,
            joints=state.get("joints"),
            velocities=state.get("velocities"),
            tcp_xyz=state.get("tcp_xyz"),
            tcp_quat=state.get("tcp_quat"),
            contacts=state.get("contacts"),
            safety=bool(state.get("safety", False)),
            command_joints=state.get("command_joints"),
            objects=objects,
            expected_objects=expected_objects,
        )
        raw_frames_list.append(frame)

    # --- Journal ------------------------------------------------------------
    journal = _build_journal(scenario_id, kind, stage, eframes, owned_ids, fixture_revision)

    # --- Executor artifacts -------------------------------------------------
    if stage == "C":
        status = "diagnostic-pass" if kind != "plan-blocked" else "diagnostic-fail"
        planner_status = "success" if kind != "plan-blocked" else "failure"
        nonempty_plan = kind != "plan-blocked"
        error_code = 1 if kind != "plan-blocked" else -2
        task_result_status = None
        terminal_status = "succeeded" if kind != "plan-blocked" else "failed"
        execute_result_status = 4 if kind != "plan-blocked" else 6
        pick_goal_sent = place_goal_sent = place_goal_accepted = False
        execute_goal_sent = False
        controller_goal_sent = False
    elif stage == "D":
        if kind in ("execute-joint", "execute-pose", "retreat", "gripper"):
            status = "diagnostic-pass"
            planner_status = "success"
            nonempty_plan = True
            error_code = 1
            task_result_status = None
            terminal_status = "succeeded"
            execute_result_status = 4
            execute_goal_sent = kind in ("execute-joint", "execute-pose")
            controller_goal_sent = kind in ("execute-joint", "execute-pose")
            pick_goal_sent = place_goal_sent = place_goal_accepted = False
        elif kind == "cancel":
            status = "diagnostic-pass"
            planner_status = "success"
            nonempty_plan = True
            error_code = 1
            task_result_status = None
            terminal_status = "canceled"
            execute_result_status = EXECUTE_STATUS_CANCELED
            execute_goal_sent = True
            controller_goal_sent = True
            pick_goal_sent = place_goal_sent = place_goal_accepted = False
        else:  # safety
            status = "diagnostic-pass"
            planner_status = "success"
            nonempty_plan = True
            error_code = 1
            task_result_status = None
            terminal_status = "aborted"
            execute_result_status = 6
            execute_goal_sent = True
            controller_goal_sent = True
            pick_goal_sent = place_goal_sent = place_goal_accepted = False
    else:
        if polarity == "positive":
            status = "diagnostic-pass"
            task_result_status = 0
            terminal_status = "succeeded"
            execute_result_status = 4
            pick_goal_sent = True
            place_goal_sent = True
            place_goal_accepted = True
        elif kind == "malformed-back":
            status = "diagnostic-pass"
            task_result_status = None
            terminal_status = "rejected"
            execute_result_status = None
            pick_goal_sent = False
            place_goal_sent = False
            place_goal_accepted = False
        else:
            status = "diagnostic-fail"
            task_result_status = 2 if kind in ("blocked-approach", "unreachable-grasp") else 4
            terminal_status = "canceled" if kind in ("cancel-approach", "cancel-transport") else "aborted"
            execute_result_status = None
            pick_goal_sent = True
            place_goal_sent = kind == "occupied-place"
            place_goal_accepted = kind == "occupied-place"
        planner_status = "success"
        nonempty_plan = True
        error_code = 1
        execute_goal_sent = False
        controller_goal_sent = kind not in ("malformed-back",)

    execution_jsonl, execution_json, moveit_plans, controller_rows, goal = (
        _build_executor_artifacts(
            scenario_id,
            kind,
            stage,
            polarity,
            status,
            planner_status=planner_status,
            execute_goal_sent=execute_goal_sent,
            controller_goal_sent=controller_goal_sent,
            terminal_status=terminal_status,
            execute_result_status=execute_result_status,
            task_result_status=task_result_status,
            pick_goal_sent=pick_goal_sent,
            place_goal_sent=place_goal_sent,
            place_goal_accepted=place_goal_accepted,
            plan_applicable=True,
            nonempty_plan=nonempty_plan,
            error_code=error_code,
        )
    )

    # --- Gate-window / manifest ---------------------------------------------
    gate_window = {
        "schema_version": 1,
        "gate": scenario_id,
        "attempt_id": ATTEMPT_ID,
        "raw_start_index": 0,
        "evaluator_start_index": 0,
        "wall_timestamp": "2026-08-04T00:00:00+00:00",
    }

    # --- Apply named faults (mutate the owning artifact) --------------------
    (
        raw_frames_list,
        evaluator_frames_list,
        journal,
        execution_jsonl,
        execution_json,
        moveit_plans,
        controller_rows,
    ) = _apply_overrides(
        raw_frames_list,
        journal,
        execution_jsonl,
        execution_json,
        moveit_plans,
        controller_rows,
        overrides,
        scenario_id=scenario_id,
        stage=stage,
        kind=kind,
    )

    # --- Write all artifacts -------------------------------------------------
    (attempt / "manifest.json").write_text(
        json.dumps(_manifest(scenario_id, seed), sort_keys=True) + "\n", encoding="utf-8"
    )
    (attempt / "gate-window.json").write_text(
        json.dumps(gate_window, sort_keys=True) + "\n", encoding="utf-8"
    )
    _write_jsonl(attempt / "physics_truth.jsonl", raw_frames_list)
    _write_jsonl(attempt / "evaluator.jsonl", evaluator_frames_list)
    _write_jsonl(attempt / "integrated-execution.jsonl", execution_jsonl)
    _write_jsonl(attempt / "moveit-plans.jsonl", moveit_plans)
    _write_jsonl(attempt / "controller-results.jsonl", controller_rows)
    (attempt / "integrated-execution.json").write_text(
        json.dumps(execution_json, sort_keys=True) + "\n", encoding="utf-8"
    )
    _write_jsonl(attempt / "planning-scene.jsonl", journal)
    goals_dir = attempt / "goals"
    if goal is not None:
        goals_dir.mkdir(parents=True, exist_ok=True)
        (goals_dir / f"{scenario_id}.json").write_text(
            json.dumps(goal, sort_keys=True) + "\n", encoding="utf-8"
        )
    # planning-scene.json final (optional-by-stage; written for all here).
    final = {
        "schema_version": 1,
        "status": "diagnostic-pass",
        "authority": "physics_truth",
        "events": [record["event"] for record in journal],
        "records": copy.deepcopy(journal),
        "graph": {},
    }
    (attempt / "planning-scene.json").write_text(
        json.dumps(final, sort_keys=True) + "\n", encoding="utf-8"
    )
    return attempt


def _kind_for(scenario_id: str, stage: str) -> str:
    if stage == "C":
        return {
            "qualification-moveit-plan-joint": "plan-joint",
            "qualification-moveit-plan-pose": "plan-pose",
            "qualification-moveit-plan-blocked": "plan-blocked",
        }.get(scenario_id, "plan-joint")
    if stage == "D":
        return STAGE_D_KIND.get(scenario_id, "execute-joint")
    return STAGE_E_KIND.get(scenario_id, "positive")


def _write_jsonl(path: Path, records: Sequence[Mapping[str, Any]]) -> None:
    lines = [json.dumps(record, sort_keys=True) for record in records]
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def _apply_overrides(
    raw_frames_list: list[dict[str, Any]],
    journal: list[dict[str, Any]],
    execution_jsonl: list[dict[str, Any]],
    execution_json: dict[str, Any],
    moveit_plans: list[dict[str, Any]],
    controller_rows: list[dict[str, Any]],
    overrides: Mapping[str, Any],
    *,
    scenario_id: str,
    stage: str,
    kind: str,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    """Apply exactly the named faults and return (raw, evaluator, journal)."""
    raw = copy.deepcopy(raw_frames_list)
    journal_out = copy.deepcopy(journal)
    exec_jsonl = copy.deepcopy(execution_jsonl)
    exec_json = copy.deepcopy(execution_json)
    plans_out = copy.deepcopy(moveit_plans)
    controller_out = copy.deepcopy(controller_rows)

    def sync_evaluator() -> list[dict[str, Any]]:
        return [{"frame": copy.deepcopy(frame)} for frame in raw]

    # placement_error_m: shift cube x by the error (positive E).
    if "placement_error_m" in overrides:
        error = float(overrides["placement_error_m"])
        for frame in raw:
            for obj in frame.get("objects", []):
                if obj.get("id") == "qualification_cube":
                    obj["pose"]["xyz"][0] += error

    # release_region_error_m: same as placement_error_m (positive E).
    if "release_region_error_m" in overrides:
        error = float(overrides["release_region_error_m"])
        for frame in raw:
            for obj in frame.get("objects", []):
                if obj.get("id") == "qualification_cube":
                    obj["pose"]["xyz"][0] += error

    # bilateral_contact=False: remove finger/cube contacts.
    if overrides.get("bilateral_contact") is False:
        for frame in raw:
            contacts = [
                contact for contact in frame.get("contacts", [])
                if "qualification_cube" not in str(contact.get("body_b", "")) + str(contact.get("body_a", ""))
            ]
            frame["contacts"] = contacts
            frame["contact_pairs"] = contacts

    # scene_attached=True: ensure the journal records scene-attach with the
    # task target attached (idempotent for scenarios that already attach).
    if overrides.get("scene_attached") is True:
        pass  # positive/attached negatives already carry scene-attach

    # expected_cube=True: ensure expected_objects carries the cube.
    if overrides.get("expected_cube") is True:
        for frame in raw:
            expected = frame.get("expected_objects", {})
            expected.setdefault(
                "qualification_cube",
                {"class_name": "cube", "actual_prim_path": "/World/Scenario/qualification_cube"},
            )

    # omit_measured_cube=True: remove the cube from measured objects.
    if overrides.get("omit_measured_cube") is True:
        for frame in raw:
            frame["objects"] = [
                obj for obj in frame.get("objects", [])
                if obj.get("id") != "qualification_cube"
            ]
            remaining = frame["objects"]
            frame["object"] = remaining[0] if remaining else None

    # frame_indices=[a, b]: keep only those raw frames (non-contiguous).
    if "frame_indices" in overrides:
        wanted = {int(value) for value in overrides["frame_indices"]}
        raw = [frame for frame in raw if int(frame["frame_index"]) in wanted]

    # raw_frame_count / evaluator_frame_count: truncate drains.
    if "raw_frame_count" in overrides or "evaluator_frame_count" in overrides:
        raw_count = int(overrides.get("raw_frame_count", len(raw)))
        evaluator_count = int(overrides.get("evaluator_frame_count", raw_count))
        raw = raw[:raw_count]
        if evaluator_count != raw_count:
            evaluator_payloads = [{"frame": copy.deepcopy(frame)} for frame in raw]
            return (
                raw,
                evaluator_payloads[:evaluator_count],
                journal_out,
                exec_jsonl,
                exec_json,
                plans_out,
                controller_out,
            )

    # action_success=True: executor claim success (endpoint claim only).
    if overrides.get("action_success") is True:
        for row in exec_jsonl:
            row["status"] = "diagnostic-pass"
            row["terminal_status"] = "succeeded"
            row["execute_result_status"] = 4
            row["task_result_status"] = 0
        exec_json["status"] = "diagnostic-pass"
        exec_json["terminal_status"] = "succeeded"
        exec_json["execute_result_status"] = 4
        exec_json["task_result_status"] = 0

    # plan_only_target_delta: on a Gate-C plan scenario, move command_targets
    # from the fixture-ready key onward (pre-start stays committed, so the plan
    # gate sees a command-target delta > tolerance).  On the default positive
    # scenario (the brief's _verify contract) a plan-only world leaves the cube
    # untouched: no contact, no lift, no transport -> bilateral/lift/transport
    # all fail, yielding verified-fail.
    if "plan_only_target_delta" in overrides:
        delta = float(overrides["plan_only_target_delta"])
        if kind == "positive":
            for frame in raw:
                frame["contacts"] = []
                frame["contact_pairs"] = []
                for obj in frame.get("objects", []):
                    if obj.get("id") == "qualification_cube":
                        obj["pose"]["xyz"] = list(E_CUBE_START)
                        obj["twist"]["linear"] = [0.0, 0.0, 0.0]
        else:
            fixture_key = next(
                (int(record["frame_index"]) for record in journal if record["event"] == "fixture-ready"),
                None,
            )
            start = fixture_key if fixture_key is not None else 0
            for frame in raw:
                if int(frame["frame_index"]) < start:
                    continue
                targets = frame.get("command_targets", {})
                positions = list(targets.get("joint_positions", [0.0] * 8))
                if len(positions) >= 8:
                    positions[0] += delta
                    targets["joint_positions"] = positions

    # cancel_post_terminal_motion: on the D cancel scenario, fresh joint motion
    # strictly after the quiescent key (post-terminal resume) -> no_later_stage
    # fails.  On the default positive scenario the object is still moving in the
    # release subwindow -> settled_speed fails.
    if "cancel_post_terminal_motion" in overrides:
        delta = float(overrides["cancel_post_terminal_motion"])
        if kind == "positive":
            detach_key = next(
                (int(record["frame_index"]) for record in journal if record["event"] == "scene-detach"),
                None,
            )
            if detach_key is not None:
                for frame in raw:
                    if int(frame["frame_index"]) < detach_key:
                        continue
                    for obj in frame.get("objects", []):
                        if obj.get("id") == "qualification_cube":
                            obj["twist"]["linear"] = [2.5 * delta, 0.0, 0.0]
        else:
            quiescent_key = next(
                (int(record["frame_index"]) for record in journal if record["event"] == "quiescent"),
                None,
            )
            if quiescent_key is not None:
                for frame in raw:
                    if int(frame["frame_index"]) <= quiescent_key:
                        continue
                    joints = list(frame["robot"]["joint_positions"])
                    if len(joints) >= 8:
                        joints[0] += delta
                        frame["robot"]["joint_positions"] = joints
                        frame["robot"]["joint_velocities"] = [delta] * 8
                    targets = frame.get("command_targets", {})
                    positions = list(targets.get("joint_positions", [0.0] * 8))
                    if len(positions) >= 8:
                        positions[0] += delta
                        targets["joint_positions"] = positions

    # safety_post_clear_target_motion: on the D safety scenario, target change
    # after operator-clear -> no_auto_resume fails.  On the default positive
    # scenario the released cube ends outside the place region ->
    # release_in_place_region fails.
    if "safety_post_clear_target_motion" in overrides:
        delta = float(overrides["safety_post_clear_target_motion"])
        if kind == "positive":
            detach_key = next(
                (int(record["frame_index"]) for record in journal if record["event"] == "scene-detach"),
                None,
            )
            if detach_key is not None:
                for frame in raw:
                    if int(frame["frame_index"]) < detach_key:
                        continue
                    for obj in frame.get("objects", []):
                        if obj.get("id") == "qualification_cube":
                            obj["pose"]["xyz"] = [0.95, 0.0, 0.64]
        else:
            clear_key = next(
                (int(record["frame_index"]) for record in journal if record["event"] == "operator-clear"),
                None,
            )
            if clear_key is not None:
                for frame in raw:
                    if int(frame["frame_index"]) > clear_key:
                        targets = frame.get("command_targets", {})
                        positions = list(targets.get("joint_positions", [0.0] * 8))
                        if len(positions) >= 8:
                            positions[0] += delta
                            targets["joint_positions"] = positions
                        joints = list(frame["robot"]["joint_positions"])
                        if len(joints) >= 8:
                            joints[0] += delta
                            frame["robot"]["joint_positions"] = joints

    # occupied_release: emit a release/detach in the occupied-place attempt.
    if overrides.get("occupied_release") is True:
        # Remove the retained contact after the place failure and mark a
        # scene-detach-style world state (forbidden for occupied-place).
        quiescent_key = next(
            (int(record["frame_index"]) for record in journal if record["event"] == "quiescent"),
            None,
        )
        place_key = next(
            (int(record["frame_index"]) for record in journal if record["event"] == "place-goal-accepted"),
            None,
        )
        if quiescent_key is not None and place_key is not None:
            midpoint = (place_key + quiescent_key) // 2
            for frame in raw:
                if int(frame["frame_index"]) >= midpoint:
                    frame["contacts"] = []
                    frame["contact_pairs"] = []
                    for obj in frame.get("objects", []):
                        if obj.get("id") == "qualification_cube":
                            obj["pose"]["xyz"] = list(PLACE_REGION)
                            obj["twist"]["linear"] = [0.0, 0.0, 0.0]

    # --- Additional named faults for the §8 adversarial suite -----------------
    # joint_tracking_error_rad: D execute-joint — executed joints deviate.
    if "joint_tracking_error_rad" in overrides:
        error = float(overrides["joint_tracking_error_rad"])
        start = next(
            (int(record["frame_index"]) for record in journal if record["event"] == "execution-start"),
            0,
        )
        for frame in raw:
            if int(frame["frame_index"]) < start:
                continue
            joints = frame["robot"]["joint_positions"]
            if len(joints) >= 8:
                joints[0] += error
            targets = frame.get("command_targets", {})
            positions = list(targets.get("joint_positions", [0.0] * 8))
            if len(positions) >= 8:
                positions[0] += error
                targets["joint_positions"] = positions

    # tcp_tracking_error_m: D execute-pose — executed TCP position deviates.
    if "tcp_tracking_error_m" in overrides:
        error = float(overrides["tcp_tracking_error_m"])
        start = next(
            (int(record["frame_index"]) for record in journal if record["event"] == "execution-start"),
            0,
        )
        for frame in raw:
            if int(frame["frame_index"]) < start:
                continue
            pose = frame["robot"].get("tcp_pose", {})
            if "xyz" in pose:
                pose["xyz"] = [pose["xyz"][0] + error, pose["xyz"][1], pose["xyz"][2]]

    # retreat_short=True: retreat TCP displacement stays below RETREAT_DISTANCE_M.
    if overrides.get("retreat_short") is True:
        start = next(
            (int(record["frame_index"]) for record in journal if record["event"] == "retreat-start"),
            0,
        )
        for frame in raw:
            if int(frame["frame_index"]) < start:
                continue
            pose = frame["robot"].get("tcp_pose", {})
            if "xyz" in pose:
                pose["xyz"] = [0.65, 0.0, 0.75]

    # gripper_travel_short=True: clamp drive_joint below min travel.
    if overrides.get("gripper_travel_short") is True:
        for frame in raw:
            joints = frame["robot"]["joint_positions"]
            if len(joints) >= 8 and joints[7] > 0.5:
                joints[7] = 0.5

    # plan_result_success=True: plan-blocked flips its plan diagnostic to success.
    if overrides.get("plan_result_success") is True:
        for row in plans_out:
            row["planner_status"] = "success"
            row["nonempty_plan"] = True
            row["error_code"] = 1
            row["error_code_classification"] = "success"
        exec_json["result"] = {
            "error_code": 1,
            "error_code_classification": "success",
            "nonempty_plan": True,
        }
        exec_json["planner_status"] = "success"

    # contact_force_exact_1_0=True: every contact sits exactly at the threshold.
    if overrides.get("contact_force_exact_1_0") is True:
        for frame in raw:
            contacts = frame.get("contacts", [])
            for contact in contacts:
                contact["normal_force"] = 1.0
            frame["contact_pairs"] = contacts

    # obstacle_contact=True: arm-pedestal contact present (positive E / retreat).
    if overrides.get("obstacle_contact") is True:
        contact = _contact("/World/Tinker/link2", "/World/Scenario/qualification_pedestal", 3.0)
        for frame in raw:
            contacts = list(frame.get("contacts", []))
            contacts.append(contact)
            frame["contacts"] = contacts
            frame["contact_pairs"] = contacts

    # transport_away=True: mirror the cube about the source x so transport
    # points away from the place region (m3 direction guard).
    if overrides.get("transport_away") is True:
        mirror = 2.0 * E_CUBE_START[0]
        for frame in raw:
            for obj in frame.get("objects", []):
                if obj.get("id") == "qualification_cube":
                    obj["pose"]["xyz"][0] = mirror - obj["pose"]["xyz"][0]

    # journal_drop_target_mid_transport=True: retained phase drops the target.
    if overrides.get("journal_drop_target_mid_transport") is True:
        for record in journal_out:
            if record["event"] == "transport":
                record["attached_ids"] = []
                record["attached_links"] = {}
                record["touch_links"] = {}

    # journal_pre_attach_target=True: a pre-attach record carries the target.
    if overrides.get("journal_pre_attach_target") is True:
        for record in journal_out:
            if record["event"] == "before-pick":
                record["attached_ids"] = [CANONICAL_TARGET_HANDOFF]
                record["attached_links"] = {CANONICAL_TARGET_HANDOFF: CANONICAL_LINK_TCP}
                record["touch_links"] = {CANONICAL_TARGET_HANDOFF: list(CANONICAL_TOUCH_LINKS)}

    # endpoint_forbidden=True: a direct Isaac command endpoint appears.
    if overrides.get("endpoint_forbidden") is True:
        exec_json["controller_endpoint"] = "/isaac_joint_commands"

    # post_cancel_motion=True (E cancel-transport): fresh motion after quiescent.
    if overrides.get("post_cancel_motion") is True:
        quiescent_key = next(
            (int(record["frame_index"]) for record in journal if record["event"] == "quiescent"),
            None,
        )
        if quiescent_key is not None:
            for frame in raw:
                if int(frame["frame_index"]) <= quiescent_key:
                    continue
                joints = list(frame["robot"]["joint_positions"])
                if len(joints) >= 8:
                    joints[0] += 0.02
                    frame["robot"]["joint_positions"] = joints
                    frame["robot"]["joint_velocities"] = [0.02] * 8

    # post_clear_resume=True (E safety-transport): motion after operator-clear.
    if overrides.get("post_clear_resume") is True:
        clear_key = next(
            (int(record["frame_index"]) for record in journal if record["event"] == "operator-clear"),
            None,
        )
        if clear_key is not None:
            for frame in raw:
                if int(frame["frame_index"]) <= clear_key:
                    continue
                joints = list(frame["robot"]["joint_positions"])
                if len(joints) >= 8:
                    joints[0] += 0.02
                    frame["robot"]["joint_positions"] = joints
                    frame["robot"]["joint_velocities"] = [0.02] * 8

    # approach_contact=True (E blocked-approach / cancel-approach): finger/cube contact.
    if overrides.get("approach_contact") is True:
        contact = _contact("/World/Tinker/left_finger", "/World/Scenario/qualification_cube", 3.0)
        contact_r = _contact("/World/Tinker/right_finger", "/World/Scenario/qualification_cube", 3.0)
        for frame in raw:
            contacts = list(frame.get("contacts", []))
            contacts.append(contact)
            contacts.append(contact_r)
            frame["contacts"] = contacts
            frame["contact_pairs"] = contacts

    # approach_tcp_motion=True (E unreachable-grasp): TCP approaches the object
    # after the fixture-ready key (pre-start frame stays at the committed pose).
    if overrides.get("approach_tcp_motion") is True:
        fixture_key = next(
            (int(record["frame_index"]) for record in journal if record["event"] == "fixture-ready"),
            0,
        )
        for frame in raw:
            if int(frame["frame_index"]) < fixture_key:
                continue
            pose = frame["robot"].get("tcp_pose", {})
            if "xyz" in pose:
                pose["xyz"] = [0.65, 0.0, 0.76]

    # malformed_pick_goal_sent=True: malformed-back claims a pick goal was sent.
    if overrides.get("malformed_pick_goal_sent") is True:
        exec_json["pick_goal_sent"] = True
        for row in exec_jsonl:
            row["pick_goal_sent"] = True

    # Return every artifact so override mutations reach the written files.
    return raw, sync_evaluator(), journal_out, exec_jsonl, exec_json, plans_out, controller_out


# --------------------------------------------------------------------------- #
# raw_frames unit fixture
# --------------------------------------------------------------------------- #
def raw_frames(
    root: Path,
    *,
    before: int,
    inside: int,
    after_terminal: int,
) -> tuple[Path, list[Mapping[str, Any]]]:
    """Create complete raw records for the boundary-selection unit test.

    The synthetic pre-start frame has ``frame_index=-1``; this is test-only.
    Real pre-start frames are non-negative and are never normalized.
    """
    if before != 1 or inside < 1 or after_terminal < 0:
        raise ValueError("raw_frames test fixture requires one pre-start frame")
    attempt = Path(root) / "attempt-1"
    attempt.mkdir(parents=True, exist_ok=True)
    records = [
        {
            "frame_index": index,
            "timestamp": float(index),
            "scenario": "qualification-pick-place-positive",
            "seed": 7,
        }
        for index in range(-before, inside + after_terminal + 1)
    ]
    (attempt / "gate-window.json").write_text(
        json.dumps(
            {
                "gate": "qualification-pick-place-positive",
                "attempt_id": "attempt-1",
                "raw_start_index": 0,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    return attempt, records
