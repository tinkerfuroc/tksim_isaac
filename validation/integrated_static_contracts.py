#!/usr/bin/env python3
"""Integrated Gate B static contract checks (offline).

This module implements the nine semantic static checks for the integrated OMPL
qualification Gate B, updated for the review-clean contracts:

1. model-fingerprint -- canonical model bundle/fingerprint and exact planning
   frame/TCP/groups/seven arm joints/``drive_joint``/eight touch-link order;
2. controller-mapping -- exact arm/gripper controller/action/service mapping;
3. selected-launch -- selected launch exclusions and no active cuMotion
   provider/import/client (``cumotion``-containing names are allowed only when
   the resolved literal value is ``false``);
4. provider-cardinality -- singleton provider cardinality;
5. fixture-ownership -- current fixture ownership/revision/handoff;
6. action-lifecycle -- hardened action result fields, managed runtime, bounded
   cancellation, deterministic shutdown;
7. scene-and-collision-safety -- no global scene cleanup; strict ``sim_ompl``
   lift remains collision-aware; hardware compatibility is separately guarded;
8. source-identities -- three source authorizations and identities from the
   produced three-entry source-lock manifest (never combined/self-authorizing
   policy data) plus the pinned prerequisite check;
9. transport-contract -- ROS Humble/RMW/Fast DDS profile consistency, valid
   scenario domains ``<=232``, ``/isaac_joint_commands`` QoS depth 50, and the
   public one-key integrated report separate from the full runtime mapping.

The checker consumes structured JSON/YAML/XML records and the produced
three-entry source-lock manifest.  Marker checks are supplemental defense;
comments/commit messages/path-only claims never satisfy a check.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

SCHEMA_VERSION = 1
HEX40 = re.compile(r"^[0-9a-f]{40}$")
HEX64 = re.compile(r"^(?!0{64}$)[0-9a-f]{64}$")

SOURCE_LOCK_ROLES = ("simulator_overlay", "production", "qualification_tooling")
TOUCH_LINKS = (
    "xarm_gripper_base_link",
    "left_outer_knuckle",
    "left_finger",
    "left_inner_knuckle",
    "right_inner_knuckle",
    "right_outer_knuckle",
    "right_finger",
    "link_tcp",
)
ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7")
GRIPPER_JOINT = "drive_joint"
COMMAND_TOPIC = "/isaac_joint_commands"
COMMAND_DEPTH = 50
MAX_ROS_DOMAIN = 232
PUBLIC_INTEGRATED = {"execution_profile": "sim_ompl"}

PRODUCTION_LAUNCH_REL = "src/mobile_bringup/launch/manipulation_planning_task_only.launch.py"
MODEL_BUNDLE_REL = "outputs/ompl-overlay/model-bundle-r2/model-bundle.json"
PROVIDER_MANIFEST_REL = "ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json"
SCENARIO_DIR_REL = "simulation/scenarios"
RECORDS_REL = "qualification/records"

STATUS_PASS = "verified-pass"
STATUS_FAIL = "verified-fail"
STATUS_INVALID = "evidence-invalid"


@dataclass(frozen=True)
class StaticCheck:
    name: str
    passed: bool
    details: dict[str, object] = field(default_factory=dict)
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class StaticReport:
    status: str
    checks: tuple[StaticCheck, ...]
    model_fingerprint: str | None
    source_identities: dict[str, object]


def _canonical_json_bytes(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _sha256_json(value: object) -> str:
    return hashlib.sha256(_canonical_json_bytes(value)).hexdigest()


def _read_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return raw if isinstance(raw, dict) else None


def _read_text(path: Path) -> str | None:
    if not path.is_file():
        return None
    try:
        return path.read_text(encoding="utf-8")
    except OSError:
        return None


def _missing_reason(relative: str) -> str:
    return "required record missing: {}".format(relative)


# ---------------------------------------------------------------------------
# record readers
# ---------------------------------------------------------------------------
def _model_record(production_root: Path) -> tuple[dict[str, Any] | None, tuple[str, ...]]:
    path = production_root / RECORDS_REL / "model.json"
    record = _read_json(path)
    if record is None:
        return None, (_missing_reason(str(path.relative_to(production_root))),)
    return record, ()


def _controllers_record(production_root: Path) -> tuple[dict[str, Any] | None, tuple[str, ...]]:
    path = production_root / RECORDS_REL / "controllers.json"
    record = _read_json(path)
    if record is None:
        return None, (_missing_reason(str(path.relative_to(production_root))),)
    return record, ()


def _providers_record(production_root: Path) -> tuple[dict[str, Any] | None, tuple[str, ...]]:
    path = production_root / RECORDS_REL / "providers.json"
    record = _read_json(path)
    if record is None:
        return None, (_missing_reason(str(path.relative_to(production_root))),)
    return record, ()


def _runtime_markers(production_root: Path) -> tuple[dict[str, Any] | None, tuple[str, ...]]:
    path = production_root / RECORDS_REL / "runtime_markers.json"
    record = _read_json(path)
    if record is None:
        return None, (_missing_reason(str(path.relative_to(production_root))),)
    return record, ()


def _result_contract(production_root: Path) -> tuple[dict[str, Any] | None, tuple[str, ...]]:
    path = production_root / RECORDS_REL / "result_contract.json"
    record = _read_json(path)
    if record is None:
        return None, (_missing_reason(str(path.relative_to(production_root))),)
    return record, ()


def _prerequisites(production_root: Path) -> tuple[dict[str, Any] | None, tuple[str, ...]]:
    path = production_root / RECORDS_REL / "prerequisites.json"
    record = _read_json(path)
    if record is None:
        return None, (_missing_reason(str(path.relative_to(production_root))),)
    return record, ()


def _overlay_contract(simulator_root: Path, relative: str) -> tuple[dict[str, Any] | None, tuple[str, ...]]:
    path = simulator_root / relative
    record = _read_json(path)
    if record is None:
        return None, (_missing_reason(str(path)),)
    return record, ()


def _provider_manifest(simulator_root: Path) -> tuple[dict[str, Any] | None, tuple[str, ...]]:
    path = simulator_root / PROVIDER_MANIFEST_REL
    record = _read_json(path)
    if record is None:
        return None, (_missing_reason(str(path)),)
    return record, ()


def _model_bundle(simulator_root: Path) -> tuple[dict[str, Any] | None, tuple[str, ...]]:
    path = simulator_root / MODEL_BUNDLE_REL
    record = _read_json(path)
    if record is None:
        return None, (_missing_reason(str(path)),)
    return record, ()


def _scenario_declarations(simulator_root: Path) -> tuple[list[dict[str, Any]], tuple[str, ...]]:
    directory = simulator_root / SCENARIO_DIR_REL
    if not directory.is_dir():
        return [], ("scenario directory missing: {}".format(directory),)
    declarations: list[dict[str, Any]] = []
    for path in sorted(directory.glob("*.json")):
        record = _read_json(path)
        if record is None:
            continue
        declarations.append(record)
    if not declarations:
        return [], ("no scenario declarations under {}".format(directory),)
    return declarations, ()


def _launch_text(production_root: Path) -> tuple[str | None, tuple[str, ...]]:
    path = production_root / PRODUCTION_LAUNCH_REL
    text = _read_text(path)
    if text is None:
        return None, (_missing_reason(PRODUCTION_LAUNCH_REL),)
    return text, ()


# ---------------------------------------------------------------------------
# individual checks
# ---------------------------------------------------------------------------
def _check_model_fingerprint(
    *, simulator_root: Path, production_root: Path, config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}

    model_config = config.get("model")
    if not isinstance(model_config, dict):
        reasons.append("config.model is missing or not an object")

    model_record, record_reasons = _model_record(production_root)
    if record_reasons:
        reasons.extend(record_reasons)
    model = model_record if isinstance(model_record, dict) else (model_config or {})

    expected_tcp = (model_config or {}).get("tcp_link")
    if not model.get("tcp_link"):
        reasons.append("model tcp_link is missing/empty")
    elif expected_tcp is not None and model.get("tcp_link") != expected_tcp:
        reasons.append("model tcp_link mismatch: {!r} != {!r}".format(model.get("tcp_link"), expected_tcp))
    if model.get("planning_frame") != "base_link":
        reasons.append("model planning_frame must be base_link")
    if model.get("arm_group") != "xarm7":
        reasons.append("model arm_group must be xarm7")
    if model.get("gripper_group") != "xarm_gripper":
        reasons.append("model gripper_group must be xarm_gripper")
    arm_joints = model.get("arm_joints")
    if arm_joints != list(ARM_JOINTS):
        reasons.append("model arm_joints must be the exact seven-joint order")
    if model.get("gripper_joint") != GRIPPER_JOINT:
        reasons.append("model gripper_joint must be drive_joint")
    touch_links = model.get("touch_links")
    if touch_links != list(TOUCH_LINKS):
        reasons.append("model touch_links must be the verbatim eight-link order (permutations fail)")

    bundle, bundle_reasons = _model_bundle(simulator_root)
    if bundle_reasons:
        reasons.extend(bundle_reasons)
    else:
        structural = bundle.get("structural_fingerprint")
        if not isinstance(structural, str) or not HEX64.fullmatch(structural):
            reasons.append("model bundle structural_fingerprint must match 64-hex and not be all-zero")
        contract = bundle.get("contract")
        if isinstance(contract, dict):
            if contract.get("tcp_link") != list(TOUCH_LINKS)[-1]:
                reasons.append("model bundle contract tcp_link must be link_tcp")
            if contract.get("touch_links") != list(TOUCH_LINKS):
                reasons.append("model bundle contract touch_links must match the eight-link order")
            if contract.get("gripper_joint") != GRIPPER_JOINT:
                reasons.append("model bundle contract gripper_joint must be drive_joint")
        normalization = bundle.get("normalization")
        if isinstance(normalization, dict):
            groups = normalization.get("groups")
            if not isinstance(groups, dict) or groups.get("arm") != "xarm7" or groups.get("gripper") != "xarm_gripper":
                reasons.append("model bundle normalization groups mismatch")
            ordered = normalization.get("ordered_joints")
            if ordered != list(ARM_JOINTS) + [GRIPPER_JOINT]:
                reasons.append("model bundle ordered_joints must be seven arm joints then drive_joint")
        if reasons.count("model bundle structural_fingerprint must match 64-hex and not be all-zero") == 0:
            fingerprint = str(structural)
            details["fingerprint"] = fingerprint
            details["model"] = dict(model)

    if overlay is not None:
        contract_bundle = overlay.get("model_bundle")
        if isinstance(contract_bundle, dict):
            recorded_fingerprint = contract_bundle.get("structural_fingerprint")
            if bundle is not None:
                actual = bundle.get("structural_fingerprint")
                if recorded_fingerprint is not None and actual != recorded_fingerprint:
                    reasons.append(
                        "model bundle fingerprint differs from the overlay contract record"
                    )
            if not isinstance(recorded_fingerprint, str) or not HEX64.fullmatch(recorded_fingerprint):
                reasons.append("overlay contract structural_fingerprint must match 64-hex")
            if details.get("fingerprint") is None:
                details["fingerprint"] = recorded_fingerprint

    passed = not reasons
    return StaticCheck(
        name="model-fingerprint",
        passed=passed,
        details=details,
        reasons=tuple(reasons),
    )


def _check_controller_mapping(
    *, production_root: Path, config: Mapping[str, Any], overlay: Mapping[str, Any] | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    controllers, record_reasons = _controllers_record(production_root)
    if record_reasons:
        reasons.extend(record_reasons)
    else:
        mapping = controllers.get("controller_mapping")
        if not isinstance(mapping, dict):
            reasons.append("controllers record controller_mapping is not an object")
        else:
            details["controller_mapping"] = dict(mapping)
            arm = mapping.get("xarm7")
            gripper = mapping.get("xarm_gripper")
            if arm != "/xarm7_traj_controller/follow_joint_trajectory":
                reasons.append("controller xarm7 must map to /xarm7_traj_controller/follow_joint_trajectory")
            if gripper != "/xarm_gripper/gripper_action":
                reasons.append("controller gripper must map to /xarm_gripper/gripper_action")
            joints = mapping.get("arm_joints")
            if joints != list(ARM_JOINTS):
                reasons.append("controller arm joint order must be the exact seven-joint order")

    if overlay is not None:
        typed = overlay.get("typed_contract")
        if isinstance(typed, dict):
            actions = typed.get("actions")
            if not isinstance(actions, dict):
                reasons.append("overlay typed_contract.actions is not an object")
            else:
                arm_action = actions.get("/xarm7_traj_controller/follow_joint_trajectory")
                if not isinstance(arm_action, dict) or arm_action.get("type") != "control_msgs/action/FollowJointTrajectory":
                    reasons.append("overlay FJT action type mismatch")
                gripper_action = actions.get("/xarm_gripper/gripper_action")
                if not isinstance(gripper_action, dict) or gripper_action.get("type") != "control_msgs/action/GripperCommand":
                    reasons.append("overlay gripper action type mismatch")
            controllers_typed = typed.get("controller_resources")
            if isinstance(controllers_typed, dict):
                if controllers_typed.get("xarm7_traj_controller") != "active":
                    reasons.append("overlay xarm7_traj_controller must be active")
                if controllers_typed.get("joint_state_broadcaster") != "active":
                    reasons.append("overlay joint_state_broadcaster must be active")

    passed = not reasons
    return StaticCheck(name="controller-mapping", passed=passed, details=details, reasons=tuple(reasons))


_ACTIVE_CUMOTION_PATTERNS = (
    r"\bfrom\s+cumotion\b",
    r"\bimport\s+cumotion\b",
    r"\bcumotion\S*\.launch\b",
    r"\bros2\s+launch\s+\S*cumotion\S*",
    r"\bcumotion_ros\b",
    r"\buse_cumotion\S*\s*=\s*(true|1)\b",
)


def _selected_launch_ast_values(
    text: str,
) -> tuple[list[str], dict[str, Any]]:
    """Return ``(active_cumotion_hits, literal_values)`` from the launch source.

    Names containing ``cumotion`` are allowed when they are parameter keys whose
    resolved literal value is exactly ``false``; they are rejected when used as
    an active provider/import/launch/client.  Other forbidden tokens (AnyGrasp,
    ``start_grasp``, ``isaac_joint_commands``) are rejected on any occurrence.

    The raw-text active-pattern scan runs even when the launch is not parseable
    Python, so an unparseable file cannot hide a forbidden reference.
    """
    active: list[str] = []
    lowered = text.lower()
    for pattern in _ACTIVE_CUMOTION_PATTERNS:
        if re.search(pattern, lowered):
            active.append(pattern)
    # cuMotion/cumotion are handled above as active patterns and by the literal
    # value check; they are allowed in parameter names when the value is false.
    # The other forbidden tokens are rejected on any occurrence.
    for token in ("AnyGrasp", "start_grasp", "isaac_joint_commands"):
        if token.lower() in lowered:
            active.append(token)

    literal: dict[str, Any] = {}
    try:
        tree = ast.parse(text)
    except (SyntaxError, ValueError):
        return active + (["<launch is not parseable Python>"] if not active else []), literal
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and isinstance(key.value, str):
                    name = key.value
                    if isinstance(value, ast.Constant):
                        literal[name] = value.value
                    else:
                        literal[name] = "<non-literal>"
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                if "cumotion" in (alias.name or "").lower():
                    active.append("cumotion import")
        if isinstance(node, ast.Call):
            func = node.func
            rendered = ""
            if isinstance(func, ast.Name):
                rendered = func.id
            elif isinstance(func, ast.Attribute):
                rendered = func.attr
            if "cumotion" in rendered.lower():
                active.append("cumotion call:{}".format(rendered))
    return active, literal


def _check_selected_launch(
    *, simulator_root: Path, production_root: Path, config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    text, text_reasons = _launch_text(production_root)
    if text_reasons:
        reasons.extend(text_reasons)
        return StaticCheck(name="selected-launch", passed=False, details=details, reasons=tuple(reasons))

    execution = config.get("execution_contract")
    forbidden = set()
    if isinstance(execution, dict):
        tokens = execution.get("forbidden_tokens")
        if isinstance(tokens, list):
            forbidden = {str(token).lower() for token in tokens}
    hits, literal = _selected_launch_ast_values(text)
    details["launch_file"] = PRODUCTION_LAUNCH_REL
    details["forbidden_token_hits"] = hits
    details["cumotion_literal_values"] = {
        key: value for key, value in literal.items() if "cumotion" in key.lower()
    }

    if hits:
        reasons.append("selected launch contains an active forbidden reference: {}".format(", ".join(hits)))

    # Names containing "cumotion" are allowed only when the resolved literal
    # value is exactly False.
    for key, value in literal.items():
        lowered = key.lower()
        if "cumotion" in lowered and value is not False:
            reasons.append(
                "cumotion parameter {!r} must resolve to literal false (got {!r})".format(key, value)
            )

    if overlay is not None:
        production_overlay = overlay.get("production_overlay")
        if isinstance(production_overlay, dict):
            compat = production_overlay.get("sim_compatibility_parameters_literal_false")
            if isinstance(compat, dict):
                for key, value in compat.items():
                    if "cumotion" in key.lower() and value is not False:
                        reasons.append(
                            "overlay contract records {!r} as literal true; must be false".format(key)
                        )
            launch_file = production_overlay.get("launch_file")
            if launch_file and str(launch_file) != "manipulation_planning_task_only.launch.py":
                reasons.append("overlay contract launch_file mismatch")

    passed = not reasons
    return StaticCheck(name="selected-launch", passed=passed, details=details, reasons=tuple(reasons))


def _check_provider_cardinality(
    *, simulator_root: Path, production_root: Path, config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    providers, record_reasons = _providers_record(production_root)
    if record_reasons:
        reasons.extend(record_reasons)
    else:
        counts = providers.get("provider_counts")
        if not isinstance(counts, dict):
            reasons.append("providers record provider_counts is not an object")
        else:
            details["provider_counts"] = dict(counts)
            for name, count in counts.items():
                if count != 1:
                    reasons.append(
                        "provider {!r} cardinality must be exactly 1 (got {!r}); "
                        "duplicate controller manager is not allowed".format(name, count)
                    )

    manifest, manifest_reasons = _provider_manifest(simulator_root)
    if manifest_reasons:
        reasons.extend(manifest_reasons)
    else:
        bad: list[str] = []
        for section in ("persistent_nodes", "one_shot_processes"):
            entries = manifest.get(section)
            if isinstance(entries, list):
                for entry in entries:
                    if isinstance(entry, dict) and entry.get("cardinality") != 1:
                        bad.append(
                            "{}:{!r}".format(section, entry.get("key") or entry.get("executable"))
                        )
        controllers = manifest.get("controller_resources")
        if isinstance(controllers, list):
            for entry in controllers:
                if isinstance(entry, dict) and entry.get("cardinality") != 1:
                    bad.append("controller_resource:{!r}".format(entry.get("resource_name")))
        if bad:
            reasons.append("provider manifest non-singleton cardinality: {}".format(", ".join(bad)))
        if overlay is not None:
            recorded = overlay.get("provider_manifest")
            if isinstance(recorded, dict):
                canonical = recorded.get("canonical_self_hash")
                if not isinstance(canonical, str) or not HEX64.fullmatch(canonical):
                    reasons.append("overlay provider manifest canonical_self_hash must match 64-hex")

    passed = not reasons
    return StaticCheck(name="provider-cardinality", passed=passed, details=details, reasons=tuple(reasons))


def _check_fixture_ownership(
    *, simulator_root: Path, config: Mapping[str, Any], overlay: Mapping[str, Any] | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    declarations, decl_reasons = _scenario_declarations(simulator_root)
    if decl_reasons:
        reasons.extend(decl_reasons)
        return StaticCheck(name="fixture-ownership", passed=False, details=details, reasons=tuple(reasons))

    declared: list[dict[str, Any]] = []
    for declaration in declarations:
        planning_scene = declaration.get("planning_scene")
        if not isinstance(planning_scene, dict):
            reasons.append("scenario {!r} has no planning_scene object".format(declaration.get("id")))
            continue
        revision = planning_scene.get("revision")
        revision_digest = planning_scene.get("revision_digest")
        frame_id = planning_scene.get("frame_id")
        target_source_id = planning_scene.get("target_source_id")
        target_handoff = planning_scene.get("target_handoff")
        if not isinstance(revision, str) or not revision:
            reasons.append("scenario {!r} revision is missing".format(declaration.get("id")))
        if not isinstance(revision_digest, str) or not HEX64.fullmatch(revision_digest):
            reasons.append("scenario {!r} revision_digest is invalid".format(declaration.get("id")))
        if frame_id != "base_link":
            reasons.append("scenario {!r} planning frame must be base_link".format(declaration.get("id")))
        if target_handoff != "pick_and_place/object_mesh":
            reasons.append("scenario {!r} target_handoff must be pick_and_place/object_mesh".format(declaration.get("id")))
        if not isinstance(target_source_id, str) or not target_source_id.startswith("sim_fixture/"):
            reasons.append("scenario {!r} target_source_id must be under sim_fixture/".format(declaration.get("id")))
        owned_ids = planning_scene.get("owned_ids")
        if isinstance(owned_ids, list) and owned_ids:
            non_owned = [oid for oid in owned_ids if not str(oid).startswith("sim_fixture/")]
            if non_owned:
                reasons.append("scenario {!r} has non sim_fixture owned ids".format(declaration.get("id")))
        declared.append(
            {
                "id": declaration.get("id"),
                "revision": revision,
                "revision_digest": revision_digest,
                "target_source_id": target_source_id,
                "target_handoff": target_handoff,
            }
        )
    details["fixtures"] = declared

    if overlay is not None:
        fixture_contract = overlay.get("fixture_contract")
        if isinstance(fixture_contract, dict):
            if fixture_contract.get("target_handoff") != "pick_and_place/object_mesh":
                reasons.append("overlay fixture_contract target_handoff mismatch")
            if fixture_contract.get("target_source_id") != "sim_fixture/public_target":
                reasons.append("overlay fixture_contract target_source_id mismatch")

    passed = not reasons
    return StaticCheck(name="fixture-ownership", passed=passed, details=details, reasons=tuple(reasons))


def _check_action_lifecycle(
    *, production_root: Path, config: Mapping[str, Any], overlay: Mapping[str, Any] | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    markers, marker_reasons = _runtime_markers(production_root)
    if marker_reasons:
        reasons.extend(marker_reasons)
    else:
        details["runtime_markers"] = dict(markers)
        if markers.get("detached_motion_thread") is True:
            reasons.append("action lifecycle uses a detached motion thread (must be managed)")
        if markers.get("bounded_cancellation") is not True:
            reasons.append("action lifecycle must have bounded cancellation")
        if markers.get("deterministic_shutdown") is not True:
            reasons.append("action lifecycle must have deterministic shutdown")

    result, result_reasons = _result_contract(production_root)
    if result_reasons:
        reasons.extend(result_reasons)
    else:
        details["result_fields"] = dict(result)
        required = result.get("required")
        present = result.get("present")
        if not isinstance(required, list) or not isinstance(present, list):
            reasons.append("result_contract required/present must be lists")
        else:
            missing = [field for field in required if field not in present]
            if missing:
                reasons.append(
                    "action result contract missing required fields: {}".format(", ".join(missing))
                )

    if overlay is not None:
        production_overlay = overlay.get("production_overlay")
        if isinstance(production_overlay, dict):
            lifecycle = production_overlay.get("task_owned_lifecycle")
            if not isinstance(lifecycle, str) or not lifecycle:
                reasons.append("overlay production_overlay.task_owned_lifecycle is missing")

    passed = not reasons
    return StaticCheck(name="action-lifecycle", passed=passed, details=details, reasons=tuple(reasons))


def _check_scene_and_collision_safety(
    *, production_root: Path, config: Mapping[str, Any], overlay: Mapping[str, Any] | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    markers, marker_reasons = _runtime_markers(production_root)
    if marker_reasons:
        reasons.extend(marker_reasons)
    else:
        details["scene_safety_markers"] = {
            key: markers.get(key) for key in ("global_scene_cleanup", "lift_collision_checking", "hardware_compat_guarded")
        }
        if markers.get("global_scene_cleanup") is True:
            reasons.append("global scene cleanup must not be enabled")
        if markers.get("lift_collision_checking") is not True:
            reasons.append("strict sim_ompl lift must remain collision-aware")
        if markers.get("hardware_compat_guarded") is not True:
            reasons.append("hardware compatibility branch must remain separately guarded")

    if overlay is not None:
        production_overlay = overlay.get("production_overlay")
        if isinstance(production_overlay, dict):
            lifecycle = production_overlay.get("task_owned_lifecycle")
            if isinstance(lifecycle, str) and "creates and owns" not in lifecycle:
                reasons.append("overlay task_owned_lifecycle does not confirm scene ownership")

    passed = not reasons
    return StaticCheck(name="scene-and-collision-safety", passed=passed, details=details, reasons=tuple(reasons))


def _check_source_identities(
    *, simulator_root: Path, production_root: Path, source_lock_manifest: Path,
    config: Mapping[str, Any],
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    manifest = _read_json(source_lock_manifest)
    if manifest is None:
        reasons.append("source-lock manifest is missing or not finite JSON")
        return StaticCheck(name="source-identities", passed=False, details=details, reasons=tuple(reasons))

    details["source_lock_manifest"] = source_lock_manifest.name
    status = manifest.get("status")
    if status != STATUS_PASS:
        reasons.append("source-lock manifest status must be verified-pass (got {!r})".format(status))

    policies = config.get("source_lock_policies")
    if not isinstance(policies, dict):
        reasons.append("config.source_lock_policies is missing")
    else:
        if set(policies) != set(SOURCE_LOCK_ROLES):
            reasons.append(
                "config.source_lock_policies must be exactly {} (got {})".format(
                    sorted(SOURCE_LOCK_ROLES), sorted(policies)
                )
            )
        identities: dict[str, object] = {}
        for role in SOURCE_LOCK_ROLES:
            record = manifest.get(role)
            if not isinstance(record, dict):
                reasons.append("source-lock manifest missing repository record {!r}".format(role))
                continue
            expected_path = policies.get(role)
            if expected_path and str(record.get("policy_path")) != str(expected_path):
                reasons.append(
                    "source-lock manifest {!r} policy_path {!r} != config {!r}".format(
                        role, record.get("policy_path"), expected_path
                    )
                )
            resolved = record.get("resolved_policy_commit")
            implementation = record.get("implementation_head")
            if not isinstance(resolved, str) or not HEX40.fullmatch(resolved):
                reasons.append("source-lock manifest {!r} resolved_policy_commit is invalid".format(role))
            if not isinstance(implementation, str) or not HEX40.fullmatch(implementation):
                reasons.append("source-lock manifest {!r} implementation_head is invalid".format(role))
            if record.get("status") != STATUS_PASS:
                reasons.append("source-lock manifest repository {!r} is not verified-pass".format(role))
            identities[role] = {
                "policy_path": record.get("policy_path"),
                "implementation_head": implementation,
                "resolved_policy_commit": resolved,
            }
        details["identities"] = identities

    prerequisites, prereq_reasons = _prerequisites(production_root)
    if prereq_reasons:
        reasons.extend(prereq_reasons)
    else:
        details["prerequisites"] = dict(prerequisites)
        if prerequisites.get("pinned") is not True:
            reasons.append(
                "source identities prerequisite is not pinned to a recorded commit"
            )
        commit = prerequisites.get("commit")
        if not isinstance(commit, str) or not HEX40.fullmatch(commit):
            reasons.append("source identities prerequisite commit must match 40-hex")

    passed = not reasons
    return StaticCheck(name="source-identities", passed=passed, details=details, reasons=tuple(reasons))


def _collect_domain_values(config: Mapping[str, Any], overlay: Mapping[str, Any] | None) -> list[object]:
    values: list[object] = []
    for section in ("stages", "thresholds", "execution_contract", "model"):
        block = config.get(section)
        if isinstance(block, dict):
            for key, value in block.items():
                if "domain" in key.lower():
                    values.append(value)
    if overlay is not None:
        ros_policy = overlay.get("ros_policy")
        if isinstance(ros_policy, dict):
            for key, value in ros_policy.items():
                if "domain" in key.lower():
                    values.append(value)
    return values


def _check_transport_contract(
    *, simulator_root: Path, production_root: Path, config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    if overlay is None:
        reasons.append("overlay contract is unavailable")
        return StaticCheck(name="transport-contract", passed=False, details=details, reasons=tuple(reasons))

    ros_policy = overlay.get("ros_policy")
    if not isinstance(ros_policy, dict):
        reasons.append("overlay contract ros_policy is missing")
    else:
        details["ros_policy"] = dict(ros_policy)
        if ros_policy.get("distro") != "humble":
            reasons.append("ROS distro must be humble")
        if ros_policy.get("rmw_implementation") != "rmw_fastrtps_cpp":
            reasons.append("RMW implementation must be rmw_fastrtps_cpp")
        dds_profiles = ros_policy.get("dds_profiles")
        if not isinstance(dds_profiles, dict) or not dds_profiles:
            reasons.append("Fast DDS profile must be documented (local/lan)")

    for value in _collect_domain_values(config, overlay):
        if isinstance(value, bool):
            reasons.append("ROS domain must be an integer in [0, 232], not a boolean")
        elif isinstance(value, (int, float)):
            integer = int(value)
            if value != integer or integer < 0 or integer > MAX_ROS_DOMAIN:
                reasons.append(
                    "ROS domain {} is outside [0, {}]".format(value, MAX_ROS_DOMAIN)
                )
        elif value is not None:
            reasons.append("ROS domain must be an integer in [0, 232], got {!r}".format(value))

    typed = overlay.get("typed_contract")
    if isinstance(typed, dict):
        publishers = typed.get("publishers")
        if isinstance(publishers, dict):
            command = publishers.get(COMMAND_TOPIC)
            if not isinstance(command, dict):
                reasons.append("/isaac_joint_commands publisher record is missing")
            else:
                depth = command.get("depth")
                if depth != COMMAND_DEPTH:
                    reasons.append(
                        "/isaac_joint_commands QoS depth must be {} (got {!r})".format(COMMAND_DEPTH, depth)
                    )
                if command.get("type") != "sensor_msgs/msg/JointState":
                    reasons.append("/isaac_joint_commands type must be sensor_msgs/msg/JointState")
        separation = typed.get("public_report_separation")
        if isinstance(separation, dict):
            public_integrated = separation.get("public_integrated")
            if public_integrated != PUBLIC_INTEGRATED:
                reasons.append(
                    "public integrated mapping must be exactly {!r}".format(PUBLIC_INTEGRATED)
                )
            expected_sha = separation.get("public_integrated_sha256")
            if expected_sha != _sha256_json(PUBLIC_INTEGRATED):
                reasons.append("public integrated_sha256 does not match the one-key public mapping")
            if not isinstance(separation.get("runtime_contract_sha256"), str):
                reasons.append("full runtime mapping must be recorded separately as runtime_contract_sha256")

    passed = not reasons
    return StaticCheck(name="transport-contract", passed=passed, details=details, reasons=tuple(reasons))


# ---------------------------------------------------------------------------
# public API
# ---------------------------------------------------------------------------
def validate_static_contracts(
    *,
    simulator_root: str | Path,
    production_root: str | Path,
    source_lock_manifest: str | Path,
    config: Mapping[str, object],
) -> StaticReport:
    """Run all nine Gate B static checks and return a ``StaticReport``."""
    simulator = Path(simulator_root)
    production = Path(production_root)
    manifest_path = Path(source_lock_manifest)

    overlay_relative = config.get("overlay_contract")
    overlay = None
    if isinstance(overlay_relative, str):
        overlay, _ = _overlay_contract(simulator, overlay_relative)

    checks = (
        _check_model_fingerprint(
            simulator_root=simulator, production_root=production, config=config, overlay=overlay
        ),
        _check_controller_mapping(
            production_root=production, config=config, overlay=overlay
        ),
        _check_selected_launch(
            simulator_root=simulator, production_root=production, config=config, overlay=overlay
        ),
        _check_provider_cardinality(
            simulator_root=simulator, production_root=production, config=config, overlay=overlay
        ),
        _check_fixture_ownership(simulator_root=simulator, config=config, overlay=overlay),
        _check_action_lifecycle(production_root=production, config=config, overlay=overlay),
        _check_scene_and_collision_safety(production_root=production, config=config, overlay=overlay),
        _check_source_identities(
            simulator_root=simulator,
            production_root=production,
            source_lock_manifest=manifest_path,
            config=config,
        ),
        _check_transport_contract(
            simulator_root=simulator, production_root=production, config=config, overlay=overlay
        ),
    )

    model_check = next(check for check in checks if check.name == "model-fingerprint")
    source_check = next(check for check in checks if check.name == "source-identities")
    status = STATUS_PASS if all(check.passed for check in checks) else STATUS_FAIL
    return StaticReport(
        status=status,
        checks=tuple(checks),
        model_fingerprint=(
            str(model_check.details.get("fingerprint")) if model_check.passed else None
        ),
        source_identities=dict(source_check.details.get("identities", {})),
    )


def _report_to_json(report: StaticReport) -> dict[str, object]:
    return {
        "schema_version": SCHEMA_VERSION,
        "status": report.status,
        "model_fingerprint": report.model_fingerprint,
        "source_identities": report.source_identities,
        "checks": [
            {
                "name": check.name,
                "passed": check.passed,
                "details": check.details,
                "reasons": list(check.reasons),
            }
            for check in report.checks
        ],
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the integrated Gate B static contract checks."
    )
    parser.add_argument("--simulator-root", required=True)
    parser.add_argument("--production-root", required=True)
    parser.add_argument("--source-lock-manifest", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args(argv)

    config = _read_json(Path(arguments.config))
    if config is None:
        print("config is missing or not finite JSON", file=sys.stderr)
        return 2

    report = validate_static_contracts(
        simulator_root=arguments.simulator_root,
        production_root=arguments.production_root,
        source_lock_manifest=arguments.source_lock_manifest,
        config=config,
    )
    output_path = Path(arguments.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(_report_to_json(report), sort_keys=True, separators=(",", ":")),
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "status": report.status,
                "checks": [check.name for check in report.checks],
            },
            sort_keys=True,
        )
    )
    return 0 if report.status == STATUS_PASS else 1


if __name__ == "__main__":
    sys.exit(main())
