#!/usr/bin/env python3
"""Integrated Gate B static contract checks (offline).

This module implements the nine semantic static checks for the integrated OMPL
qualification Gate B.  It binds Gate B to real artifacts (F1-F5):

1. model-fingerprint -- canonical model bundle/fingerprint, exact planning
   frame/TCP/groups/seven arm joints/``drive_joint``/eight touch-link order,
   every recorded model artifact SHA-256, the ``production_source_commits``
   blobs, and the immutable production SRDF ``_xarm7_macro.srdf.xacro``;
2. controller-mapping -- exact arm/gripper controller/action/service mapping
   from the immutable production ``controllers.yaml`` files and the overlay
   typed contract;
3. selected-launch -- selected launch exclusions and no active cuMotion
   provider/import/client (``cumotion``-containing names are allowed only when
   the resolved literal value is ``false``);
4. provider-cardinality -- singleton provider cardinality, the provider-manifest
   canonical/raw hashes, and the provider executable set reconciliation;
5. fixture-ownership -- exactly the 17 configured scenario identities, derived
   ``planning_scene.objects`` owned ids vs ``integrated.expected_scene.owned_ids``,
   revisions, digests, frames, source/handoff, and overlay scenario contracts;
6. action-lifecycle -- managed/joined worker/coordinator/executor threads, no
   ``.detach()``, bounded shutdown, and the required terminal result-field
   writes against the real ``.action`` schemas;
7. scene-and-collision-safety -- SimOmpl scene-cleanup early return, task-owned
   hardware cleanup, collision-aware SimOmpl lift, guarded hardware
   compatibility branch, and ``move_straight`` avoid-collisions forwarding;
8. source-identities -- three source authorizations and identities from the
   produced three-entry source-lock manifest, the F1.5 pinned-prerequisite
   identity/ancestry binding, and the manifest-vs-overlay identity equality;
9. transport-contract -- ROS Humble/RMW/Fast DDS profile consistency, valid
   scenario domains ``<=232``, ``/isaac_joint_commands`` QoS depth 50, the
   public one-key integrated report separate from the sibling
   ``typed_contract.runtime_contract_sha256`` full runtime mapping.

Every production source is read as an immutable Git blob via
``git show <implementation_head>:<path>`` -- never the authorized-dirty working
copy.  A source mutation at the inspected commit fails the corresponding check.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

try:
    import yaml
except Exception:  # pragma: no cover - never exercised in the fixture tree
    yaml = None

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
EXPECTED_CONFIGURE_ID = 17

STATUS_PASS = "verified-pass"
STATUS_FAIL = "verified-fail"
STATUS_INVALID = "evidence-invalid"

# Immutable production paths (relative to the production repository root).
PROD_SRDF_REL = "src/xarm_ros2/xarm_moveit_config/srdf/_xarm7_macro.srdf.xacro"
PROD_XARM7_CONTROLLERS_REL = "src/xarm_ros2/xarm_moveit_config/config/xarm7/controllers.yaml"
PROD_GRIPPER_CONTROLLERS_REL = "src/xarm_ros2/xarm_moveit_config/config/xarm_gripper/controllers.yaml"
PROD_LAUNCH_REL = "src/mobile_bringup/launch/manipulation_planning_task_only.launch.py"
PROD_PICK_AND_PLACE_CPP_REL = "src/pick_and_place/src/pick_and_place.cpp"
PROD_ACTION_EXECUTION_CPP_REL = "src/pick_and_place/src/action_execution.cpp"
PROD_PACKAGE_UTILS_CPP_REL = "src/pick_and_place/src/package_utils.cpp"
PROD_SCENE_OWNERSHIP_CPP_REL = "src/pick_and_place/src/scene_ownership.cpp"
PROD_GRASP_NODE_HPP_REL = "src/pick_and_place/include/grasp_node.hpp"
PROD_ACTION_DIR_REL = "src/tinker_arm_msgs/action"
PROD_CPP_PATHS = (
    PROD_PICK_AND_PLACE_CPP_REL,
    PROD_ACTION_EXECUTION_CPP_REL,
    PROD_PACKAGE_UTILS_CPP_REL,
    PROD_SCENE_OWNERSHIP_CPP_REL,
    PROD_GRASP_NODE_HPP_REL,
)
ACTION_SCHEMA_FILES = (
    "Pick.action",
    "Place.action",
    "CartesianMove.action",
    "JointMove.action",
    "Fold.action",
)

# Simulator-side artifact paths.
MODEL_BUNDLE_REL = "outputs/ompl-overlay/model-bundle-r2/model-bundle.json"
PROVIDER_MANIFEST_REL = "ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json"
SCENARIO_DIR_REL = "simulation/scenarios"

# The full runtime mapping recompute keys (typed_contract subset whose canonical
# JSON digest equals typed_contract.runtime_contract_sha256).  See the real
# tinker_sim_bridge integrated_readiness.build_integrated_mapping().
RUNTIME_MAPPING_KEYS = (
    "report_revision",
    "actions",
    "services",
    "publishers",
    "joint_names",
    "touch_links",
    "tf",
    "controller_resources",
    "final_simulation_state",
)

_ACTIVE_CUMOTION_PATTERNS = (
    r"\bfrom\s+cumotion\b",
    r"\bimport\s+cumotion\b",
    r"\bcumotion\S*\.launch\b",
    r"\bros2\s+launch\s+\S*cumotion\S*",
    r"\bcumotion_ros\b",
    r"\buse_cumotion\S*\s*=\s*(true|1)\b",
)


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


# ---------------------------------------------------------------------------
# low-level helpers
# ---------------------------------------------------------------------------
def _canonical_json_bytes(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _sha256_json(value: object) -> str:
    return hashlib.sha256(_canonical_json_bytes(value)).hexdigest()


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _read_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return raw if isinstance(raw, dict) else None


def _missing_reason(relative: str) -> str:
    return "required record missing: {}".format(relative)


def _git(repo_root: Path, *args: str) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        ["git", "-C", str(repo_root), *args],
        capture_output=True,
        env={**os.environ, "LC_ALL": "C"},
        check=False,
    )


def _git_commit_exists(repo_root: Path, commit: str) -> bool:
    proc = _git(repo_root, "cat-file", "-e", "{}".format(commit) + "^{commit}")
    return proc.returncode == 0


def _git_is_ancestor(repo_root: Path, ancestor: str, descendant: str) -> bool:
    if ancestor == descendant:
        return True
    proc = _git(repo_root, "merge-base", "--is-ancestor", ancestor, descendant)
    return proc.returncode == 0


def _git_show_bytes(repo_root: Path, commit: str, rel_path: str) -> bytes | None:
    proc = _git(repo_root, "show", "{}:{}".format(commit, rel_path))
    if proc.returncode != 0:
        return None
    return proc.stdout


def _git_show_text(repo_root: Path, commit: str, rel_path: str) -> str | None:
    blob = _git_show_bytes(repo_root, commit, rel_path)
    if blob is None:
        return None
    try:
        return blob.decode("utf-8")
    except UnicodeDecodeError:
        return None


def _strip_cpp_comments(text: str) -> str:
    """Remove // and /* */ comments while preserving newlines and strings."""
    out: list[str] = []
    i = 0
    n = len(text)
    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if ch == "/" and nxt == "/":
            while i < n and text[i] != "\n":
                i += 1
            continue
        if ch == "/" and nxt == "*":
            i += 2
            while i < n:
                if text[i] == "*" and i + 1 < n and text[i + 1] == "/":
                    i += 2
                    break
                if text[i] == "\n":
                    out.append("\n")
                i += 1
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def _function_body(text: str, name: str) -> str | None:
    """Return the brace-matched body of the first ``<name>`` function/method."""
    idx = text.find(name)
    if idx < 0:
        return None
    open_idx = text.find("{", idx)
    if open_idx < 0:
        return None
    depth = 0
    for i in range(open_idx, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[open_idx : i + 1]
    return text[open_idx:]


def _revision_digest(planning_scene: Mapping[str, Any]) -> str:
    payload = {
        str(key): value
        for key, value in planning_scene.items()
        if key != "revision_digest"
    }
    return _sha256_json(payload)


def _fixture_owned_ids(planning_scene: Mapping[str, Any]) -> list[str]:
    owned: list[str] = []
    for record in planning_scene.get("objects", []) or []:
        if isinstance(record, dict) and isinstance(record.get("id"), str):
            owned.append(record["id"])
    for record in planning_scene.get("diagnostic_objects", []) or []:
        if isinstance(record, dict) and record.get("enter_collision_bodies") is True:
            owned.append(str(record["id"]))
    return owned


def _normalize_repo_relative(root: Path, raw_path: str) -> str | None:
    """Normalize a policy/config path to repository-relative POSIX form.

    Traversal or cross-root absolute paths fail (return None).  Relative paths
    are canonicalized; absolute paths must start with the resolved repository
    root.
    """
    path = Path(raw_path)
    if path.is_absolute():
        try:
            rel = path.resolve().relative_to(root.resolve())
        except ValueError:
            return None
        parts = [part for part in rel.parts if part not in ("", ".", "..")]
    else:
        parts = [part for part in path.parts if part not in ("", ".", "..")]
    normalized = "/".join(parts)
    if ".." in parts:
        return None
    return normalized


def _parse_srdf_gripper(text: str) -> tuple[list[str], str | None, list[str]]:
    """Return ``(xarm_gripper link order, end_effector parent_link, xarm7 arm joints)``.

    The production ``_xarm7_macro.srdf.xacro`` writes every name with a
    ``${prefix}`` interpolation token; the parser accepts an optional
    ``${prefix}`` and strips it from extracted names.
    """
    prefix = r"(?:\$\{prefix\})?"
    gripper_links: list[str] = []
    gripper_match = re.search(
        r"<group\s+name=\"" + prefix + r"xarm_gripper\">(.*?)</group>",
        text,
        re.DOTALL,
    )
    if gripper_match:
        for link_match in re.finditer(
            r"<link\s+name=\"" + prefix + r"([^\"]+)\"\s*/>", gripper_match.group(1)
        ):
            gripper_links.append(link_match.group(1))
    ee_parent: str | None = None
    ee_match = re.search(
        r"<end_effector[^>]*name=\"" + prefix + r"xarm_gripper\"[^>]*parent_link=\""
        + prefix + r"([^\"]+)\"[^>]*/?>",
        text,
    )
    if ee_match:
        ee_parent = ee_match.group(1)
    arm_joints: list[str] = []
    arm_match = re.search(
        r"<group\s+name=\"" + prefix + r"xarm7\">(.*?)</group>", text, re.DOTALL
    )
    if arm_match:
        for joint_match in re.finditer(
            r"<joint\s+name=\"" + prefix + r"(joint\d)\"\s*/>", arm_match.group(1)
        ):
            arm_joints.append(joint_match.group(1))
    return gripper_links, ee_parent, arm_joints


def _parse_controllers_yaml(text: str) -> dict[str, dict[str, object]]:
    """Parse a MoveIt controllers.yaml into ``{controller_name: {..}}``."""
    if yaml is None:
        return {}
    try:
        data = yaml.safe_load(text)
    except Exception:
        return {}
    if not isinstance(data, dict):
        return {}
    result: dict[str, dict[str, object]] = {}
    for name in data.get("controller_names", []) or []:
        block = data.get(str(name))
        if isinstance(block, dict):
            result[str(name)] = dict(block)
    return result


def _parse_action_result_fields(text: str) -> list[str]:
    """Extract the declared result field names from a ROS2 .action file."""
    fields: list[str] = []
    in_result = False
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("#"):
            if "Result" in stripped:
                in_result = True
            continue
        if in_result:
            if stripped.startswith("---"):
                break
            if not stripped:
                continue
            parts = stripped.split()
            if len(parts) >= 2 and parts[1].startswith(("_", "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z")):
                fields.append(parts[1].rstrip(";"))
    return fields


def _selected_launch_ast_values(
    text: str,
) -> tuple[list[str], dict[str, Any]]:
    """Return ``(active_cumotion_hits, literal_values)`` from launch source."""
    active: list[str] = []
    lowered = text.lower()
    for pattern in _ACTIVE_CUMOTION_PATTERNS:
        if re.search(pattern, lowered):
            active.append(pattern)
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


# ---------------------------------------------------------------------------
# individual checks
# ---------------------------------------------------------------------------
def _check_model_fingerprint(
    *,
    simulator_root: Path,
    production_root: Path,
    config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
    impl_head: str | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}

    model_config = config.get("model")
    if not isinstance(model_config, dict):
        reasons.append("config.model is missing or not an object")
    else:
        if model_config.get("planning_frame") != "base_link":
            reasons.append("model planning_frame must be base_link")
        if model_config.get("tcp_link") != "link_tcp":
            reasons.append("model tcp_link must be link_tcp")
        if model_config.get("arm_group") != "xarm7":
            reasons.append("model arm_group must be xarm7")
        if model_config.get("gripper_group") != "xarm_gripper":
            reasons.append("model gripper_group must be xarm_gripper")
        if model_config.get("arm_joints") != list(ARM_JOINTS):
            reasons.append("model arm_joints must be the exact seven-joint order")
        if model_config.get("gripper_joint") != GRIPPER_JOINT:
            reasons.append("model gripper_joint must be drive_joint")
        if model_config.get("touch_links") != list(TOUCH_LINKS):
            reasons.append("model touch_links must be the verbatim eight-link order (permutations fail)")

    bundle = _read_json(simulator_root / MODEL_BUNDLE_REL)
    if bundle is None:
        reasons.append(_missing_reason(MODEL_BUNDLE_REL))
    else:
        contract = bundle.get("contract")
        structural = bundle.get("structural_fingerprint")
        if not isinstance(contract, dict):
            reasons.append("model bundle contract is not an object")
        elif structural != _sha256_json(contract):
            reasons.append("model bundle structural_fingerprint must equal sha256(canonical bundle contract)")
        if not isinstance(structural, str) or not HEX64.fullmatch(str(structural)):
            reasons.append("model bundle structural_fingerprint must match 64-hex and not be all-zero")
        if isinstance(contract, dict):
            if contract.get("planning_frame") != "base_link":
                reasons.append("model bundle contract planning_frame must be base_link")
            if contract.get("tcp_link") != "link_tcp":
                reasons.append("model bundle contract tcp_link must be link_tcp")
            if contract.get("gripper_joint") != GRIPPER_JOINT:
                reasons.append("model bundle contract gripper_joint must be drive_joint")
            if contract.get("touch_links") != list(TOUCH_LINKS):
                reasons.append("model bundle contract touch_links must match the eight-link order")
            if contract.get("arm_joints") != list(ARM_JOINTS):
                reasons.append("model bundle contract arm_joints must be the exact seven-joint order")
            groups = contract.get("groups")
            if isinstance(groups, dict):
                gripper = groups.get("xarm_gripper")
                if not isinstance(gripper, dict) or gripper.get("joints") != [GRIPPER_JOINT]:
                    reasons.append("model bundle contract xarm_gripper group must contain exactly drive_joint")
            ee = contract.get("end_effector")
            if not isinstance(ee, dict) or ee.get("group") != "xarm_gripper" or ee.get("parent_link") != "link_tcp":
                reasons.append("model bundle contract end_effector must be xarm_gripper with parent link_tcp")
            details["fingerprint"] = structural
            details["contract"] = contract

        if overlay is not None:
            bundle_record = overlay.get("model_bundle")
            if isinstance(bundle_record, dict):
                recorded = bundle_record.get("structural_fingerprint")
                if recorded != structural:
                    reasons.append("model bundle fingerprint differs from the overlay contract record")

        artifacts = bundle.get("artifacts")
        if isinstance(artifacts, dict):
            for key, artifact in artifacts.items():
                if not isinstance(artifact, dict):
                    reasons.append("model artifact {!r} must be an object".format(key))
                    continue
                recorded = artifact.get("sha256")
                if not isinstance(recorded, str) or not HEX64.fullmatch(recorded):
                    reasons.append("model artifact {!r} must have 64-hex sha256".format(key))
                    continue
                path_rel = artifact.get("path_relative")
                abs_path = artifact.get("path")
                candidates: list[Path] = []
                if isinstance(path_rel, str):
                    candidates.append(simulator_root / path_rel)
                    candidates.append(production_root / path_rel)
                if isinstance(abs_path, str):
                    candidates.append(Path(abs_path))
                matched = False
                for candidate in candidates:
                    if candidate.is_file():
                        matched = True
                        if _sha256_bytes(candidate.read_bytes()) != recorded:
                            reasons.append("model artifact {!r} sha256 mismatch: recorded {!r}".format(key, recorded))
                        break
                if not matched and candidates:
                    reasons.append("model artifact {!r} file not found for hash binding".format(key))

        source_commits = bundle.get("production_source_commits")
        if isinstance(source_commits, dict):
            for key, entry in source_commits.items():
                if not isinstance(entry, dict):
                    continue
                commit = entry.get("commit")
                path_rel = entry.get("path_relative")
                recorded = entry.get("sha256")
                repo_path = entry.get("repo_path")
                if not isinstance(commit, str) or not HEX40.fullmatch(commit):
                    reasons.append("production source {!r} commit must be 40-hex".format(key))
                    continue
                if not isinstance(path_rel, str) or not isinstance(recorded, str) or not HEX64.fullmatch(recorded):
                    reasons.append("production source {!r} must have path_relative and 64-hex sha256".format(key))
                    continue
                repo = production_root
                if isinstance(repo_path, str) and Path(repo_path).is_dir():
                    repo = Path(repo_path)
                blob = _git_show_bytes(repo, commit, path_rel)
                if blob is None:
                    reasons.append(
                        "production source {!r} blob {!r}@{!r} not found in {!r}".format(key, path_rel, commit, str(repo))
                    )
                elif _sha256_bytes(blob) != recorded:
                    reasons.append("production source {!r} sha256 mismatch at {!r}".format(key, path_rel))

    if impl_head is None or not HEX40.fullmatch(impl_head):
        reasons.append("production implementation_head is unavailable/invalid; cannot inspect immutable SRDF")
    else:
        srdf = _git_show_text(production_root, impl_head, PROD_SRDF_REL)
        if srdf is None:
            reasons.append("immutable production SRDF not found: " + PROD_SRDF_REL)
        else:
            gripper_links, ee_parent, arm_joints = _parse_srdf_gripper(srdf)
            if gripper_links != list(TOUCH_LINKS):
                reasons.append("SRDF xarm_gripper touch-link order is not the verbatim eight-link order (permutations fail)")
            if ee_parent != "link_tcp":
                reasons.append("SRDF end_effector parent_link must be link_tcp")
            if arm_joints != list(ARM_JOINTS):
                reasons.append("SRDF xarm7 group must contain the exact seven-joint order")
            details["srdf"] = {
                "gripper_links": gripper_links,
                "end_effector_parent": ee_parent,
                "arm_joints": arm_joints,
            }

    passed = not reasons
    return StaticCheck(
        name="model-fingerprint",
        passed=passed,
        details=details,
        reasons=tuple(reasons),
    )


def _check_controller_mapping(
    *,
    production_root: Path,
    config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
    impl_head: str | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    if impl_head is None or not HEX40.fullmatch(impl_head):
        reasons.append("production implementation_head is unavailable/invalid; cannot inspect controllers.yaml")
    else:
        xarm7_text = _git_show_text(production_root, impl_head, PROD_XARM7_CONTROLLERS_REL)
        gripper_text = _git_show_text(production_root, impl_head, PROD_GRIPPER_CONTROLLERS_REL)
        if xarm7_text is None:
            reasons.append("immutable production controllers.yaml missing: " + PROD_XARM7_CONTROLLERS_REL)
        else:
            controllers = _parse_controllers_yaml(xarm7_text)
            block = controllers.get("xarm7_traj_controller")
            if block is None:
                reasons.append("controllers.yaml must declare xarm7_traj_controller")
            else:
                details["xarm7_controller"] = dict(block)
                if block.get("action_ns") != "follow_joint_trajectory":
                    reasons.append("xarm7_traj_controller action_ns must be follow_joint_trajectory")
                if block.get("type") != "FollowJointTrajectory":
                    reasons.append("xarm7_traj_controller MoveIt type must be FollowJointTrajectory")
                if list(block.get("joints") or []) != list(ARM_JOINTS):
                    reasons.append("xarm7_traj_controller joints must be the exact seven-joint order")
        if gripper_text is None:
            reasons.append("immutable production controllers.yaml missing: " + PROD_GRIPPER_CONTROLLERS_REL)
        else:
            controllers = _parse_controllers_yaml(gripper_text)
            block = controllers.get("xarm_gripper")
            if block is None:
                reasons.append("controllers.yaml must declare xarm_gripper")
            else:
                details["gripper_controller"] = dict(block)
                if block.get("action_ns") != "gripper_action":
                    reasons.append("xarm_gripper action_ns must be gripper_action")
                if block.get("type") != "GripperCommand":
                    reasons.append("xarm_gripper MoveIt type must be GripperCommand")
                if list(block.get("joints") or []) != [GRIPPER_JOINT]:
                    reasons.append("xarm_gripper joints must be [drive_joint]")

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
                for endpoint, expected_type in (
                    ("/pickup_action", "tinker_arm_msgs/action/Pick"),
                    ("/place_action", "tinker_arm_msgs/action/Place"),
                    ("/cartesian_move_action", "tinker_arm_msgs/action/CartesianMove"),
                    ("/joint_move_action", "tinker_arm_msgs/action/JointMove"),
                    ("/fold_action", "tinker_arm_msgs/action/Fold"),
                ):
                    action = actions.get(endpoint)
                    if not isinstance(action, dict) or action.get("type") != expected_type:
                        reasons.append("overlay task action {!r} type must be {!r}".format(endpoint, expected_type))
            resources = typed.get("controller_resources")
            if isinstance(resources, dict):
                if resources.get("xarm7_traj_controller") != "active":
                    reasons.append("overlay xarm7_traj_controller must be active")
                if resources.get("joint_state_broadcaster") != "active":
                    reasons.append("overlay joint_state_broadcaster must be active")

    passed = not reasons
    return StaticCheck(name="controller-mapping", passed=passed, details=details, reasons=tuple(reasons))


def _check_selected_launch(
    *,
    production_root: Path,
    config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
    impl_head: str | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    if impl_head is None or not HEX40.fullmatch(impl_head):
        reasons.append("production implementation_head is unavailable/invalid; cannot inspect launch")
        return StaticCheck(name="selected-launch", passed=False, details=details, reasons=tuple(reasons))
    text = _git_show_text(production_root, impl_head, PROD_LAUNCH_REL)
    if text is None:
        reasons.append("immutable production launch missing: " + PROD_LAUNCH_REL)
        return StaticCheck(name="selected-launch", passed=False, details=details, reasons=tuple(reasons))

    execution = config.get("execution_contract")
    if isinstance(execution, dict):
        tokens = execution.get("forbidden_tokens")
        details["forbidden_tokens"] = list(tokens) if isinstance(tokens, list) else []
    hits, literal = _selected_launch_ast_values(text)
    details["launch_file"] = PROD_LAUNCH_REL
    details["forbidden_token_hits"] = hits
    details["cumotion_literal_values"] = {
        key: value for key, value in literal.items() if "cumotion" in key.lower()
    }
    if hits:
        reasons.append("selected launch contains an active forbidden reference: {}".format(", ".join(hits)))
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
    *,
    simulator_root: Path,
    production_root: Path,
    config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
    impl_head: str | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    manifest = _read_json(simulator_root / PROVIDER_MANIFEST_REL)
    if manifest is None:
        reasons.append(_missing_reason(PROVIDER_MANIFEST_REL))
        return StaticCheck(name="provider-cardinality", passed=False, details=details, reasons=tuple(reasons))

    recomputed = _sha256_json(
        {k: v for k, v in manifest.items() if k != "provider_manifest_sha256"}
    )
    recorded = manifest.get("provider_manifest_sha256")
    if recorded != recomputed:
        reasons.append("provider manifest recorded sha256 does not match its canonical self-hash")
    details["provider_manifest_sha256"] = recomputed

    raw_bytes = (simulator_root / PROVIDER_MANIFEST_REL).read_bytes()
    if overlay is not None:
        recorded_manifest = overlay.get("provider_manifest")
        if isinstance(recorded_manifest, dict):
            if recorded_manifest.get("canonical_self_hash") != recomputed:
                reasons.append("overlay provider_manifest.canonical_self_hash does not match the recomputed canonical hash")
            if recorded_manifest.get("raw_sha256") != _sha256_bytes(raw_bytes):
                reasons.append("overlay provider_manifest.raw_sha256 does not match the raw file bytes")

    bad: list[str] = []
    for section in ("persistent_nodes", "one_shot_processes"):
        for entry in manifest.get(section, []) or []:
            if isinstance(entry, dict) and entry.get("cardinality") != 1:
                bad.append("{!r}:{!r}".format(section, entry.get("key") or entry.get("executable")))
    for entry in manifest.get("controller_resources", []) or []:
        if isinstance(entry, dict) and entry.get("cardinality") != 1:
            bad.append("controller_resource:{!r}".format(entry.get("resource_name")))
    for entry in manifest.get("publishers", []) or []:
        if isinstance(entry, dict) and entry.get("cardinality") != 1:
            bad.append("publisher:{!r}".format(entry.get("topic")))
    if bad:
        reasons.append("provider manifest non-singleton cardinality: {}".format(", ".join(bad)))
    details["non_singleton"] = bad

    derived: set[str] = set()
    for section in ("persistent_nodes", "one_shot_processes"):
        for entry in manifest.get(section, []) or []:
            if isinstance(entry, dict) and isinstance(entry.get("executable"), str):
                derived.add(entry["executable"])
    if overlay is not None:
        production_overlay = overlay.get("production_overlay")
        if isinstance(production_overlay, dict):
            provider_set = production_overlay.get("simulator_overlay_provider_set")
            if isinstance(provider_set, dict) and isinstance(provider_set.get("executables"), list):
                expected = {str(item) for item in provider_set["executables"]}
                if derived != expected:
                    reasons.append("provider executable set differs from overlay simulator_overlay_provider_set")
                details["provider_executable_set"] = sorted(derived)

    passed = not reasons
    return StaticCheck(name="provider-cardinality", passed=passed, details=details, reasons=tuple(reasons))


def _check_fixture_ownership(
    *,
    simulator_root: Path,
    config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}

    stages = config.get("stages")
    configured: list[str] = []
    if isinstance(stages, dict):
        for stage_key in ("C", "D"):
            block = stages.get(stage_key)
            if isinstance(block, dict) and isinstance(block.get("scenarios"), list):
                configured.extend(str(item) for item in block["scenarios"])
        e = stages.get("E")
        if isinstance(e, dict):
            if isinstance(e.get("positive"), str):
                configured.append(e["positive"])
            if isinstance(e.get("negative"), list):
                configured.extend(str(item) for item in e["negative"])
    unique = list(dict.fromkeys(configured))
    details["configured_scenarios"] = unique
    if len(unique) != EXPECTED_CONFIGURE_ID:
        reasons.append(
            "configured integrated scenario count must be exactly {} (got {})".format(EXPECTED_CONFIGURE_ID, len(unique))
        )

    overlay_scenarios = None
    if overlay is not None:
        overlay_scenarios = overlay.get("scenarios")
        if not isinstance(overlay_scenarios, dict):
            reasons.append("overlay contract scenarios map is missing")

    for sid in unique:
        path = simulator_root / SCENARIO_DIR_REL / (sid + ".json")
        raw = _read_json(path)
        if raw is None:
            reasons.append("configured scenario missing: " + sid)
            continue
        if raw.get("id") != sid:
            reasons.append("scenario declaration id mismatch: {!r} != {!r}".format(raw.get("id"), sid))
        decl = {k: v for k, v in raw.items() if k not in ("id", "seed")}
        decl_sha = _sha256_json({"id": sid, "seed": raw.get("seed"), "declaration": decl})
        if isinstance(overlay_scenarios, dict) and sid in overlay_scenarios:
            recorded_decl = overlay_scenarios[sid].get("scenario_declaration_sha256")
            if recorded_decl != decl_sha:
                reasons.append("scenario {!r} declaration sha256 differs from the overlay contract".format(sid))
            overlay_scene = overlay_scenarios[sid].get("planning_scene")
            if not isinstance(overlay_scene, dict):
                reasons.append("overlay scenario {!r} has no planning_scene".format(sid))

        ps = raw.get("planning_scene")
        if not isinstance(ps, dict):
            reasons.append("configured scenario {!r} has no planning_scene object".format(sid))
            continue
        owned_ids = _fixture_owned_ids(ps)
        integrated = raw.get("integrated")
        expected_ids = None
        if isinstance(integrated, dict):
            expected = integrated.get("expected_scene")
            if isinstance(expected, dict):
                expected_ids = expected.get("owned_ids")
        if list(expected_ids or []) != owned_ids:
            reasons.append(
                "scenario {!r} planning_scene.objects owned ids != integrated.expected_scene.owned_ids".format(sid)
            )
        target = ps.get("target_source_id")
        if not isinstance(target, str) or target not in owned_ids:
            reasons.append("scenario {!r} target_source_id not in the owned set".format(sid))
        revision = ps.get("revision")
        if not isinstance(revision, str) or not revision:
            reasons.append("scenario {!r} revision is missing".format(sid))
        rev_digest = ps.get("revision_digest")
        if not isinstance(rev_digest, str) or not HEX64.fullmatch(rev_digest):
            reasons.append("scenario {!r} revision_digest is invalid".format(sid))
        elif rev_digest != _revision_digest(ps):
            reasons.append("scenario {!r} revision_digest is not canonical".format(sid))
        if ps.get("frame_id") != "base_link":
            reasons.append("scenario {!r} planning frame must be base_link".format(sid))
        if not isinstance(target, str) or not target.startswith("sim_fixture/"):
            reasons.append("scenario {!r} target_source_id must be under sim_fixture/".format(sid))
        if ps.get("target_handoff") != "pick_and_place/object_mesh":
            reasons.append("scenario {!r} target_handoff must be pick_and_place/object_mesh".format(sid))

        if isinstance(overlay_scenarios, dict) and sid in overlay_scenarios:
            overlay_scene = overlay_scenarios[sid].get("planning_scene")
            if isinstance(overlay_scene, dict):
                if list(overlay_scene.get("owned_ids") or []) != owned_ids:
                    reasons.append("overlay scenario {!r} owned_ids mismatch".format(sid))
                if overlay_scene.get("revision") != revision:
                    reasons.append("overlay scenario {!r} revision mismatch".format(sid))
                if overlay_scene.get("revision_digest") != rev_digest:
                    reasons.append("overlay scenario {!r} revision_digest mismatch".format(sid))
                if overlay_scene.get("frame_id") != "base_link":
                    reasons.append("overlay scenario {!r} frame_id mismatch".format(sid))
                if overlay_scene.get("target_source_id") != target:
                    reasons.append("overlay scenario {!r} target_source_id mismatch".format(sid))
                if overlay_scene.get("target_handoff") != "pick_and_place/object_mesh":
                    reasons.append("overlay scenario {!r} target_handoff mismatch".format(sid))

    passed = not reasons
    return StaticCheck(name="fixture-ownership", passed=passed, details=details, reasons=tuple(reasons))


def _check_action_lifecycle(
    *,
    production_root: Path,
    config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
    impl_head: str | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    if impl_head is None or not HEX40.fullmatch(impl_head):
        reasons.append("production implementation_head is unavailable/invalid; cannot inspect C++ lifecycle")
        return StaticCheck(name="action-lifecycle", passed=False, details=details, reasons=tuple(reasons))

    all_cpp: list[str] = []
    cpp_texts: dict[str, str] = {}
    for rel in PROD_CPP_PATHS:
        text = _git_show_text(production_root, impl_head, rel)
        if text is None:
            reasons.append("immutable production C++ missing: " + rel)
            continue
        cpp_texts[rel] = text
        all_cpp.append(_strip_cpp_comments(text))

    if all_cpp:
        if ".detach()" in "\n".join(all_cpp):
            reasons.append("action lifecycle uses a detached thread (must be managed)")
        combined = "\n".join(all_cpp)
        if "motion_runtime_.shutdown(" not in combined:
            reasons.append("action lifecycle must call motion_runtime_.shutdown(...)")
        if "executor_thread_.join()" not in combined:
            reasons.append("action lifecycle must join the executor thread")
        if "executor_thread_.joinable()" not in combined and "executor_thread_.join()" in combined:
            # guarded join present elsewhere; the joinable guard is preferred
            pass
        pick = cpp_texts.get(PROD_PICK_AND_PLACE_CPP_REL, "")
        stripped_pick = _strip_cpp_comments(pick)
        if "motion_runtime_.shutdown(" not in stripped_pick:
            reasons.append("bounded runtime shutdown must appear in the executable C++ structure")
        if "executor_thread_.join()" not in stripped_pick:
            reasons.append("executor thread must be joined in the executable C++ structure")
        details["cpp_inspected"] = list(cpp_texts.keys())

    action_fields: dict[str, list[str]] = {}
    for schema in ACTION_SCHEMA_FILES:
        rel = PROD_ACTION_DIR_REL + "/" + schema
        text = _git_show_text(production_root, impl_head, rel)
        if text is None:
            reasons.append("immutable production .action missing: " + rel)
            continue
        action_fields[schema] = _parse_action_result_fields(text)
    details["action_result_fields"] = action_fields

    result_src = _git_show_text(production_root, impl_head, PROD_ACTION_EXECUTION_CPP_REL)
    if result_src is None:
        reasons.append("immutable production action_execution.cpp missing")
    else:
        stripped = _strip_cpp_comments(result_src)
        required_common = ("result->stage", "result->status", "result->error_msg")
        for field in required_common:
            if field not in stripped:
                reasons.append("action result contract missing required field write: " + field)
        for schema, fields in action_fields.items():
            if "success" in fields and "result->success" not in stripped:
                reasons.append("action {!r} schema requires result->success write".format(schema))

    if overlay is not None:
        production_overlay = overlay.get("production_overlay")
        if isinstance(production_overlay, dict):
            lifecycle = production_overlay.get("task_owned_lifecycle")
            if not isinstance(lifecycle, str) or not lifecycle:
                reasons.append("overlay production_overlay.task_owned_lifecycle is missing")

    passed = not reasons
    return StaticCheck(name="action-lifecycle", passed=passed, details=details, reasons=tuple(reasons))


def _check_scene_and_collision_safety(
    *,
    production_root: Path,
    config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
    impl_head: str | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
    if impl_head is None or not HEX40.fullmatch(impl_head):
        reasons.append("production implementation_head is unavailable/invalid; cannot inspect scene safety C++")
        return StaticCheck(name="scene-and-collision-safety", passed=False, details=details, reasons=tuple(reasons))

    utils = _git_show_text(production_root, impl_head, PROD_PACKAGE_UTILS_CPP_REL)
    if utils is None:
        reasons.append("immutable production package_utils.cpp missing")
    else:
        stripped = _strip_cpp_comments(utils)
        body = _function_body(stripped, "clean_planning_scene")
        if body is None:
            reasons.append("clean_planning_scene() must be present in the executable C++ structure")
        else:
            sim_idx = body.find("ExecutionProfile::SimOmpl")
            ret_idx = body.find("return", sim_idx) if sim_idx >= 0 else -1
            apply_idx = body.find("applyPlanningScene")
            # The SimOmpl guard must return before the hardware applyPlanningScene cleanup.
            if sim_idx < 0:
                reasons.append("clean_planning_scene() must gate on ExecutionProfile::SimOmpl")
            if ret_idx < 0:
                reasons.append("clean_planning_scene() must return early on the SimOmpl branch")
            if sim_idx >= 0 and ret_idx > sim_idx and apply_idx >= 0 and apply_idx < ret_idx:
                reasons.append("clean_planning_scene() must not run the hardware scene cleanup on the SimOmpl branch")
        if "task_cleanup_remove_ids(" not in stripped:
            reasons.append("hardware scene cleanup must route through task-owned task_cleanup_remove_ids()")
        details["clean_planning_scene"] = body is not None

    ownership = _git_show_text(production_root, impl_head, PROD_SCENE_OWNERSHIP_CPP_REL)
    if ownership is None:
        reasons.append("immutable production scene_ownership.cpp missing")
    else:
        stripped = _strip_cpp_comments(ownership)
        if "execute_lift(ctx, true," not in stripped:
            reasons.append("SimOmpl post-close branch must call collision-aware execute_lift(ctx, true, ...)")
        if "execute_lift(ctx, false," not in stripped:
            reasons.append("hardware compatibility branch must call collision-disabled execute_lift(ctx, false, ...)")
        if "confirms_obstruction()" not in stripped:
            reasons.append("SimOmpl post-close must require close_result.confirms_obstruction()")
        details["execute_lift_branches"] = {
            "sim_collision_aware": "execute_lift(ctx, true," in stripped,
            "hardware_compat": "execute_lift(ctx, false," in stripped,
        }

    pick = _git_show_text(production_root, impl_head, PROD_PICK_AND_PLACE_CPP_REL)
    if pick is None:
        reasons.append("immutable production pick_and_place.cpp missing")
    else:
        stripped = _strip_cpp_comments(pick)
        if "request.avoid_collisions = avoid_collisions;" not in stripped:
            reasons.append("move_straight must forward avoid_collisions to request.avoid_collisions")

    if overlay is not None:
        production_overlay = overlay.get("production_overlay")
        if isinstance(production_overlay, dict):
            lifecycle = production_overlay.get("task_owned_lifecycle")
            if isinstance(lifecycle, str) and "creates and owns" not in lifecycle:
                reasons.append("overlay task_owned_lifecycle does not confirm scene ownership")

    passed = not reasons
    return StaticCheck(name="scene-and-collision-safety", passed=passed, details=details, reasons=tuple(reasons))


def _check_source_identities(
    *,
    simulator_root: Path,
    production_root: Path,
    source_lock_manifest: Path,
    config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
    manifest: dict[str, Any] | None,
) -> StaticCheck:
    reasons: list[str] = []
    details: dict[str, object] = {}
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
            expected_raw = policies.get(role)
            role_root = simulator_root if role in ("simulator_overlay", "qualification_tooling") else production_root
            expected_rel = None
            if isinstance(expected_raw, str):
                expected_rel = _normalize_repo_relative(role_root, expected_raw)
            observed_rel = record.get("policy_path")
            if expected_rel is not None and observed_rel != expected_rel:
                reasons.append(
                    "source-lock manifest {!r} policy_path {!r} != config {!r}".format(
                        role, observed_rel, expected_rel
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
                "policy_path": observed_rel,
                "implementation_head": implementation,
                "resolved_policy_commit": resolved,
            }
        details["identities"] = identities

        prod_record = manifest.get("production")
        if isinstance(prod_record, dict):
            prod_impl = prod_record.get("implementation_head")
            if isinstance(prod_impl, str) and HEX40.fullmatch(prod_impl) and overlay is not None:
                repos = overlay.get("repositories")
                if isinstance(repos, dict):
                    prod_identity = repos.get("production", {}).get("implementation_identity")
                    if isinstance(prod_identity, str) and prod_identity != prod_impl:
                        reasons.append(
                            "manifest production implementation_head differs from the overlay immutable production identity"
                        )
                    if not isinstance(prod_identity, str) or not HEX40.fullmatch(prod_identity):
                        reasons.append("overlay repositories.production.implementation_identity is missing/invalid")
                else:
                    reasons.append("overlay repositories map is missing")
            if isinstance(prod_impl, str) and HEX40.fullmatch(prod_impl):
                if not _git_commit_exists(production_root, prod_impl):
                    reasons.append("production implementation_head is not a real commit in the production repository")
                # F1.5 pinned prerequisite: production source commits bound as
                # ancestors of the implementation identity.
                if overlay is not None:
                    model_bundle = overlay.get("model_bundle")
                    if isinstance(model_bundle, dict):
                        source_commits = model_bundle.get("production_source_commits")
                        if isinstance(source_commits, dict):
                            for key, entry in source_commits.items():
                                if not isinstance(entry, dict):
                                    continue
                                commit = entry.get("commit")
                                repo_path = entry.get("repo_path")
                                if not isinstance(commit, str) or not HEX40.fullmatch(commit):
                                    continue
                                repo = production_root
                                if isinstance(repo_path, str) and Path(repo_path).is_dir():
                                    repo = Path(repo_path)
                                if str(repo.resolve()) == str(production_root.resolve()):
                                    if not _git_is_ancestor(repo, commit, prod_impl):
                                        reasons.append(
                                            "production source {!r} commit is not an ancestor of the implementation head".format(key)
                                        )
                                elif _git_commit_exists(repo, commit) is False and repo.exists():
                                    reasons.append(
                                        "production source {!r} commit is not resolvable in {!r}".format(key, str(repo))
                                    )
    passed = not reasons
    return StaticCheck(name="source-identities", passed=passed, details=details, reasons=tuple(reasons))


def _check_transport_contract(
    *,
    simulator_root: Path,
    production_root: Path,
    config: Mapping[str, Any],
    overlay: Mapping[str, Any] | None,
    impl_head: str | None,
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
        domain_id = ros_policy.get("domain_id")
        if isinstance(domain_id, bool):
            reasons.append("ROS domain must be an integer in [0, 232], not a boolean")
        elif isinstance(domain_id, (int, float)):
            if int(domain_id) != domain_id or domain_id < 0 or domain_id > MAX_ROS_DOMAIN:
                reasons.append("ROS domain {} is outside [0, {}]".format(domain_id, MAX_ROS_DOMAIN))
        elif domain_id is not None:
            reasons.append("ROS domain must be an integer in [0, 232], got {!r}".format(domain_id))

    for value in _collect_domain_values(config, overlay):
        if isinstance(value, bool):
            reasons.append("ROS domain must be an integer in [0, 232], not a boolean")
        elif isinstance(value, (int, float)):
            integer = int(value)
            if value != integer or integer < 0 or integer > MAX_ROS_DOMAIN:
                reasons.append("ROS domain {} is outside [0, {}]".format(value, MAX_ROS_DOMAIN))
        elif value is not None:
            reasons.append("ROS domain must be an integer in [0, 232], got {!r}".format(value))

    typed = overlay.get("typed_contract")
    if not isinstance(typed, dict):
        reasons.append("overlay typed_contract is missing")
    else:
        publishers = typed.get("publishers")
        if isinstance(publishers, dict):
            command = publishers.get(COMMAND_TOPIC)
            if not isinstance(command, dict):
                reasons.append("/isaac_joint_commands publisher record is missing")
            else:
                if command.get("depth") != COMMAND_DEPTH:
                    reasons.append("/isaac_joint_commands QoS depth must be {} (got {!r})".format(COMMAND_DEPTH, command.get("depth")))
                if command.get("type") != "sensor_msgs/msg/JointState":
                    reasons.append("/isaac_joint_commands type must be sensor_msgs/msg/JointState")
        separation = typed.get("public_report_separation")
        if not isinstance(separation, dict):
            reasons.append("typed_contract.public_report_separation is missing")
        else:
            if "runtime_contract_sha256" in separation:
                reasons.append("runtime_contract_sha256 must not be nested inside public_report_separation")
            public_integrated = separation.get("public_integrated")
            if public_integrated != PUBLIC_INTEGRATED:
                reasons.append("public integrated mapping must be exactly {!r}".format(PUBLIC_INTEGRATED))
            expected_sha = separation.get("public_integrated_sha256")
            if expected_sha != _sha256_json(PUBLIC_INTEGRATED):
                reasons.append("public integrated_sha256 does not match the one-key public mapping")
        runtime_sha = typed.get("runtime_contract_sha256")
        if not isinstance(runtime_sha, str) or not HEX64.fullmatch(runtime_sha):
            reasons.append("typed_contract.runtime_contract_sha256 must be a sibling 64-hex value")
        else:
            subset = {key: typed.get(key) for key in RUNTIME_MAPPING_KEYS}
            if any(value is None for value in subset.values()):
                reasons.append("typed_contract is missing full runtime mapping keys for the recompute")
            elif _sha256_json(subset) != runtime_sha:
                reasons.append("typed_contract.runtime_contract_sha256 does not match the recomputed canonical full runtime mapping")
            evidence = overlay.get("evidence")
            task6 = evidence.get("task6") if isinstance(evidence, dict) else None
            if not isinstance(task6, dict) or task6.get("runtime_contract_sha256") != runtime_sha:
                reasons.append("typed_contract.runtime_contract_sha256 does not match evidence.task6.runtime_contract_sha256")

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
    """Run all nine Gate B static checks and return a ``StaticReport``.

    Status semantics (F4):

    * missing/malformed/non-pass source-lock evidence => aggregate
      ``evidence-invalid``;
    * structurally valid authorization plus a semantic contract mismatch =>
      ``verified-fail``;
    * all nine checks pass => ``verified-pass``.
    """
    simulator = Path(simulator_root)
    production = Path(production_root)
    manifest_path = Path(source_lock_manifest)

    manifest = _read_json(manifest_path)
    impl_head = None
    if manifest is not None:
        prod_record = manifest.get("production")
        if isinstance(prod_record, dict):
            candidate = prod_record.get("implementation_head")
            if isinstance(candidate, str) and HEX40.fullmatch(candidate):
                impl_head = candidate

    overlay_relative = config.get("overlay_contract")
    overlay = None
    if isinstance(overlay_relative, str):
        overlay = _read_json(simulator / overlay_relative)

    checks = (
        _check_model_fingerprint(
            simulator_root=simulator, production_root=production, config=config,
            overlay=overlay, impl_head=impl_head,
        ),
        _check_controller_mapping(
            production_root=production, config=config, overlay=overlay, impl_head=impl_head,
        ),
        _check_selected_launch(
            production_root=production, config=config, overlay=overlay, impl_head=impl_head,
        ),
        _check_provider_cardinality(
            simulator_root=simulator, production_root=production, config=config,
            overlay=overlay, impl_head=impl_head,
        ),
        _check_fixture_ownership(simulator_root=simulator, config=config, overlay=overlay),
        _check_action_lifecycle(
            production_root=production, config=config, overlay=overlay, impl_head=impl_head,
        ),
        _check_scene_and_collision_safety(
            production_root=production, config=config, overlay=overlay, impl_head=impl_head,
        ),
        _check_source_identities(
            simulator_root=simulator,
            production_root=production,
            source_lock_manifest=manifest_path,
            config=config,
            overlay=overlay,
            manifest=manifest,
        ),
        _check_transport_contract(
            simulator_root=simulator, production_root=production, config=config,
            overlay=overlay, impl_head=impl_head,
        ),
    )

    model_check = next(check for check in checks if check.name == "model-fingerprint")
    source_check = next(check for check in checks if check.name == "source-identities")

    if manifest is None or manifest.get("status") != STATUS_PASS:
        status = STATUS_INVALID
    elif all(check.passed for check in checks):
        status = STATUS_PASS
    else:
        status = STATUS_FAIL

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


def _atomic_write_fsync(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_name, str(path))
        dir_fd = os.open(str(path.parent), os.O_DIRECTORY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except BaseException:
        try:
            os.unlink(tmp_name)
        except OSError:
            pass
        raise


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the integrated Gate B static contract checks."
    )
    parser.add_argument("--simulator-root", required=True)
    parser.add_argument("--production-root", required=True)
    parser.add_argument("--source-lock-manifest", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True, help="output directory for evidence files")
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
    output_dir = Path(arguments.output)
    _atomic_write_fsync(
        output_dir / "static-contract.json",
        json.dumps(_report_to_json(report), sort_keys=True, separators=(",", ":")).encode("utf-8"),
    )
    _atomic_write_fsync(
        output_dir / "model-fingerprint.json",
        json.dumps(
            {"schema_version": SCHEMA_VERSION, "model_fingerprint": report.model_fingerprint},
            sort_keys=True, separators=(",", ":"),
        ).encode("utf-8"),
    )
    _atomic_write_fsync(
        output_dir / "source-identities.json",
        json.dumps(
            {"schema_version": SCHEMA_VERSION, "source_identities": report.source_identities},
            sort_keys=True, separators=(",", ":"),
        ).encode("utf-8"),
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
    if report.status == STATUS_PASS:
        return 0
    if report.status == STATUS_FAIL:
        return 1
    return 2


if __name__ == "__main__":
    sys.exit(main())
