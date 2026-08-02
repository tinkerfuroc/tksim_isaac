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
PROD_ACTION_RUNTIME_CPP_REL = "src/pick_and_place/src/action_runtime.cpp"
PROD_PACKAGE_UTILS_CPP_REL = "src/pick_and_place/src/package_utils.cpp"
PROD_SCENE_OWNERSHIP_CPP_REL = "src/pick_and_place/src/scene_ownership.cpp"
PROD_GRASP_NODE_HPP_REL = "src/pick_and_place/include/grasp_node.hpp"
PROD_ACTION_DIR_REL = "src/tinker_arm_msgs/action"
PROD_CPP_PATHS = (
    PROD_PICK_AND_PLACE_CPP_REL,
    PROD_ACTION_EXECUTION_CPP_REL,
    PROD_ACTION_RUNTIME_CPP_REL,
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
# Each task result builder must write the fields declared by its corresponding
# .action result schema (F2.1 result builders).
BUILDER_SCHEMA = {
    "make_pick_result": "Pick.action",
    "make_place_result": "Place.action",
    "make_cartesian_result": "CartesianMove.action",
    "make_joint_result": "JointMove.action",
    "make_fold_result": "Fold.action",
}

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


# ---------------------------------------------------------------------------
# deliberate-scope C++ lexical/structural layer (fix round 2 / F2.1)
#
# These helpers are intentionally NOT a general C++ compiler.  They are just
# enough to prove the load-bearing production contracts listed in the fix
# brief: comments and every literal body (ordinary strings, character literals,
# raw strings) are blanked, ``#if 0`` dead regions are removed, actual function
# definitions are located by qualified name and brace-matched, and ``if``
# branches are extracted by their real condition and brace structure.  A
# required semantic token that appears only inside a literal or a dead block
# never satisfies a check.
# ---------------------------------------------------------------------------
def _cpp_match(text: str, open_idx: int, open_ch: str, close_ch: str) -> int | None:
    """Return the index of the matching close char, or None if unterminated."""
    depth = 0
    for i in range(open_idx, len(text)):
        if text[i] == open_ch:
            depth += 1
        elif text[i] == close_ch:
            depth -= 1
            if depth == 0:
                return i
    return None


def _cpp_raw_string_prefix(text: str, i: int) -> tuple[str, int] | None:
    """Return ``(delimiter, index_of_open_paren)`` if a raw string begins at i.

    Only a standalone ``[u8]R`` token (not an identifier suffix) is treated as
    a raw-string prefix; the preceding character must not be an identifier char.
    """
    if i > 0 and (text[i - 1].isalnum() or text[i - 1] == "_"):
        return None
    for prefix in ("u8R", "uR", "UR", "LR", "R"):
        if text.startswith(prefix + '"', i):
            j = i + len(prefix) + 1
            delim_start = j
            while j < len(text) and text[j] != "(":
                ch = text[j]
                if ch in " \t\n\r\\":
                    return None
                j += 1
            if j >= len(text):
                return None
            delim = text[delim_start:j]
            if len(delim) > 16:
                return None
            return delim, j
    return None


def _cpp_mask_literals(text: str) -> str:
    """Blank comments and all literal bodies with spaces (newlines preserved).

    The returned string is the same length as the input so brace/newline/offset
    positions are preserved.  Literal content cannot satisfy a semantic check.
    """
    out = list(text)
    n = len(text)
    i = 0
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if c == "/" and nxt == "/":
            j = i
            while j < n and text[j] != "\n":
                j += 1
            for k in range(i, j):
                out[k] = " "
            i = j
            continue
        if c == "/" and nxt == "*":
            j = i + 2
            while j + 1 < n and not (text[j] == "*" and text[j + 1] == "/"):
                j += 1
            end = min(j + 2, n)
            for k in range(i, end):
                if text[k] != "\n":
                    out[k] = " "
            i = end
            continue
        raw = _cpp_raw_string_prefix(text, i)
        if raw is not None:
            delim, open_paren = raw
            close = ")" + delim + '"'
            end = text.find(close, open_paren + 1)
            if end < 0:
                end = n
            else:
                end += len(close)
            for k in range(i, end):
                if text[k] != "\n":
                    out[k] = " "
            i = end
            continue
        if c == '"':
            j = i + 1
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text[j] == '"':
                    j += 1
                    break
                j += 1
            for k in range(i, min(j, n)):
                if text[k] != "\n":
                    out[k] = " "
            i = j
            continue
        if c == "'":
            j = i + 1
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text[j] == "'":
                    j += 1
                    break
                j += 1
            for k in range(i, min(j, n)):
                if text[k] != "\n":
                    out[k] = " "
            i = j
            continue
        i += 1
    return "".join(out)


_IF0_RE = re.compile(r"#\s*if\s*\(?\s*0\b")
_PP_CONDITIONAL_RE = re.compile(r"^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b")


def _cpp_strip_preprocessor_dead(text: str) -> str:
    """Blank ``#if 0`` dead regions while preserving newlines and offsets.

    Nested ``#if`` directives are tracked.  Directive lines (``#if``/``#ifdef``/
    ``#ifndef``/``#elif``/``#else``/``#endif``) are intentionally left intact so
    that :func:`_cpp_has_preprocessor_conditional` can reject any load-bearing
    body carrying a conditional-preprocessor anchor; only the *content* of a
    provably-dead ``#if 0`` region is blanked so it cannot satisfy a check.
    """
    lines = text.splitlines(keepends=True)
    out = list(lines)
    stack: list[tuple[str, bool]] = []  # (kind, seen_else); kind in {"if0", "if"}
    for i, line in enumerate(lines):
        content = line[:-1] if line.endswith("\n") else line
        stripped = content.lstrip()
        if not stripped.startswith("#"):
            if any(kind == "if0" and not seen_else for kind, seen_else in stack):
                out[i] = (" " * len(content)) + ("\n" if line.endswith("\n") else "")
            continue
        directive = stripped[1:].strip()
        keyword = directive.split()[0] if directive else ""
        if keyword in ("if", "ifdef", "ifndef"):
            is_if0 = keyword == "if" and bool(_IF0_RE.match(stripped))
            stack.append(("if0" if is_if0 else "if", False))
            # directive line left intact: _cpp_has_preprocessor_conditional sees it
        elif keyword in ("else", "elif"):
            if stack:
                kind, seen_else = stack[-1]
                if kind == "if0" and not seen_else:
                    stack[-1] = ("if0", True)
            # directive line left intact
        elif keyword == "endif":
            if stack:
                stack.pop()
            # directive line left intact
    return "".join(out)


def _cpp_sanitize(text: str) -> str:
    """Return C++ with comments/literals blanked and ``#if 0`` dead regions
    removed, preserving braces/newlines/offsets (F2.1.1)."""
    return _cpp_strip_preprocessor_dead(_cpp_mask_literals(text))


def _cpp_has_preprocessor_conditional(body: str) -> bool:
    """True if a load-bearing body still carries an ``#if``/``#ifdef``/
    ``#ifndef``/``#elif``/``#else``/``#endif`` directive (dead-code anchors must
    not satisfy a check, F2.1.2)."""
    for line in body.splitlines():
        if _PP_CONDITIONAL_RE.match(line):
            return True
    return False


def _cpp_skip_after_paren(text: str, close: int) -> int:
    """Skip whitespace and trailing function specifiers after a ')' to reach the
    opening '{' of a definition (e.g. ``) noexcept {``)."""
    j = close + 1
    while j < len(text):
        while j < len(text) and text[j] in " \t\r\n":
            j += 1
        if j >= len(text):
            break
        for keyword in ("noexcept", "override", "final", "const", "volatile"):
            if text.startswith(keyword, j):
                tail = j + len(keyword)
                if tail >= len(text) or not (text[tail].isalnum() or text[tail] == "_"):
                    j = tail
                    break
        else:
            if text[j : j + 2] == "&&":
                j += 2
            elif text[j] == "&":
                j += 1
            else:
                break
    return j


def _cpp_find_function(text: str, qualified_name: str) -> str | None:
    """Return the brace-matched body of the single actual definition of the
    named function/method, or None if missing/ambiguous/unterminated (F2.1.3).

    The definition is located by its signature: the qualified name, a ``(``,
    a paren-balanced parameter list, and a ``{`` (after any ``noexcept`` etc.).
    A call site ends in ``;`` and is never mistaken for a definition.
    """
    candidates: list[str] = []
    start = 0
    while True:
        idx = text.find(qualified_name, start)
        if idx < 0:
            break
        before = text[idx - 1] if idx > 0 else ""
        if before.isalnum() or before == "_" or before == "." or before == ">":
            start = idx + len(qualified_name)
            continue
        after = idx + len(qualified_name)
        if after >= len(text) or text[after] != "(":
            start = after
            continue
        close = _cpp_match(text, after, "(", ")")
        if close is None:
            start = after
            continue
        j = _cpp_skip_after_paren(text, close)
        if j < len(text) and text[j] == "{":
            body_end = _cpp_match(text, j, "{", "}")
            if body_end is not None:
                candidates.append(text[j:body_end])
                start = body_end + 1
                continue
            start = j + 1
        else:
            start = close + 1
    if len(candidates) == 1:
        return candidates[0]
    return None


def _cpp_if_blocks(body: str) -> list[dict[str, str]]:
    """Return ``[{'condition': str, 'body': str}]`` for every braced ``if``
    block in the function body (any nesting depth), extracted by actual
    condition and brace structure (F2.1.4)."""
    blocks: list[dict[str, str]] = []
    for match in re.finditer(r"\bif\s*\(", body):
        open_paren = match.end() - 1
        close_paren = _cpp_match(body, open_paren, "(", ")")
        if close_paren is None:
            continue
        condition = body[open_paren + 1 : close_paren]
        j = close_paren + 1
        while j < len(body) and body[j] in " \t\r\n":
            j += 1
        if j < len(body) and body[j] == "{":
            close_brace = _cpp_match(body, j, "{", "}")
            if close_brace is not None:
                blocks.append({"condition": condition, "body": body[j:close_brace]})
    return blocks


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

    ``..`` traversal is rejected *before* any filtering (F2.4.3); cross-root
    absolute paths fail.  Relative paths are canonicalized; absolute paths must
    start with the resolved repository root.
    """
    path = Path(raw_path)
    if path.is_absolute():
        try:
            rel = path.resolve().relative_to(root.resolve())
        except ValueError:
            return None
        parts = rel.parts
    else:
        parts = path.parts
    if ".." in parts:
        return None
    return "/".join(part for part in parts if part not in ("", "."))


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
    """Extract the declared result field names from a ROS2 .action file.

    The file is split on ``---`` section separators; the Result section is the
    one whose header comment contains ``Result``, or (when there is no explicit
    header) the second section in the Goal/Result/Feedback order.
    """
    sections: list[list[str]] = []
    current: list[str] = []
    for line in text.splitlines():
        if line.strip() == "---":
            sections.append(current)
            current = []
        else:
            current.append(line)
    sections.append(current)

    result_section: list[str] | None = None
    for section in sections:
        if any("#" in line and "Result" in line for line in section):
            result_section = section
            break
    if result_section is None and len(sections) >= 2:
        result_section = sections[1]

    fields: list[str] = []
    seen: set[str] = set()
    if result_section:
        for line in result_section:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) >= 2 and parts[1].rstrip(";").isidentifier():
                name = parts[1].rstrip(";")
                if name not in seen:
                    fields.append(name)
                    seen.add(name)
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
# F2.2 authoritative production source-commit binding
# ---------------------------------------------------------------------------
def _verify_overlay_source_commits(
    overlay: Mapping[str, Any],
    production_root: Path,
    impl_head: str | None,
) -> tuple[list[str], dict[str, dict[str, Any]]]:
    """Verify every ``model_bundle.production_source_commits`` entry from the
    authoritative overlay contract against immutable Git blobs.

    Returns ``(reasons, verified_entries)``.  Each entry must name a real
    40-hex commit that exists in the exact recorded ``repo_path`` repository,
    whose ``path_relative`` blob exists at that commit with the recorded
    SHA-256.  Entries bound to the manipulation repository must additionally be
    an ancestor of the manifest production ``implementation_head``.  External
    model-source repositories (e.g. ``tk25_basic``) are accepted only via their
    exact recorded commit/blob/hash -- never the current working bytes.  No
    malformed entry is silently skipped (F2.2).
    """
    reasons: list[str] = []
    model_bundle = overlay.get("model_bundle")
    if not isinstance(model_bundle, dict):
        reasons.append("overlay model_bundle is missing")
        return reasons, {}
    source_commits = model_bundle.get("production_source_commits")
    if not isinstance(source_commits, dict) or not source_commits:
        reasons.append(
            "overlay model_bundle.production_source_commits must be a non-empty object"
        )
        return reasons, {}

    production_resolved = Path(production_root).resolve()
    verified: dict[str, dict[str, Any]] = {}
    for key, entry in source_commits.items():
        if not isinstance(entry, dict):
            reasons.append("production source {!r} entry must be an object".format(key))
            continue
        commit = entry.get("commit")
        path_rel = entry.get("path_relative")
        recorded = entry.get("sha256")
        repo_path = entry.get("repo_path")
        if not isinstance(commit, str) or not HEX40.fullmatch(commit):
            reasons.append("production source {!r} commit must be 40-hex".format(key))
            continue
        if not isinstance(path_rel, str) or not path_rel:
            reasons.append("production source {!r} path_relative is missing".format(key))
            continue
        if not isinstance(recorded, str) or not HEX64.fullmatch(recorded):
            reasons.append("production source {!r} sha256 must be 64-hex and not all-zero".format(key))
            continue
        repo = Path(repo_path) if isinstance(repo_path, str) else None
        if repo is None or not repo.is_dir():
            reasons.append("production source {!r} repo_path is not a real repository directory".format(key))
            continue
        repo = repo.resolve()
        if not _git_commit_exists(repo, commit):
            reasons.append(
                "production source {!r} commit {!r} does not exist in {!r}".format(
                    key, commit, str(repo)
                )
            )
            continue
        blob = _git_show_bytes(repo, commit, path_rel)
        if blob is None:
            reasons.append(
                "production source {!r} blob {!r}@{!r} not found in {!r}".format(
                    key, path_rel, commit, str(repo)
                )
            )
            continue
        if _sha256_bytes(blob) != recorded:
            reasons.append("production source {!r} sha256 mismatch at {!r}".format(key, path_rel))
            continue
        if impl_head is not None and repo == production_resolved:
            if not _git_is_ancestor(repo, commit, impl_head):
                reasons.append(
                    "production source {!r} commit is not an ancestor of the implementation head".format(key)
                )
                continue
        verified[key] = dict(entry)
    return reasons, verified


def _bind_bundle_artifact(
    key: str,
    artifact: Mapping[str, Any],
    overlay_artifacts: Mapping[str, Any],
    verified_source_commits: Mapping[str, dict[str, Any]],
    simulator_root: Path,
) -> list[str]:
    """Bind one model-bundle artifact to verified immutable bytes (F2.2).

    A simulator-local artifact (under ``outputs/`` or ``artifacts/`` of the
    simulator root) is hashed from those exact bytes.  A production/external
    source artifact is bound to a verified immutable source-commit record with
    the same digest.  An arbitrary working-tree path is never hashed.
    """
    reasons: list[str] = []
    recorded = artifact.get("sha256")
    if not isinstance(recorded, str) or not HEX64.fullmatch(recorded):
        return ["model artifact {!r} must have 64-hex sha256".format(key)]
    overlay_artifact = overlay_artifacts.get(key)
    if not isinstance(overlay_artifact, dict):
        return ["overlay model_bundle.artifacts is missing {!r}".format(key)]
    if overlay_artifact.get("sha256") != recorded:
        return [
            "model artifact {!r} sha256 differs from the overlay contract record".format(key)
        ]

    path_rel = artifact.get("path_relative")
    if not isinstance(path_rel, str) or not path_rel:
        path_rel = overlay_artifact.get("path_relative")
    if isinstance(path_rel, str) and path_rel:
        raw = Path(path_rel)
        # Simulator-local artifact: hash the exact simulator bytes.
        if raw.is_absolute():
            try:
                raw.relative_to(simulator_root.resolve())
                sim_relative = raw.resolve().relative_to(simulator_root.resolve())
            except ValueError:
                sim_relative = None
        else:
            sim_relative = None
            if (simulator_root / raw).is_file():
                sim_relative = raw
        if sim_relative is not None:
            actual = _sha256_bytes((simulator_root / sim_relative).read_bytes())
            if actual != recorded:
                reasons.append("model artifact {!r} sha256 mismatch (simulator-local)".format(key))
            return reasons

    # Production/external source artifact: bind to a verified immutable
    # source-commit record carrying the same digest/path identity.
    for entry in verified_source_commits.values():
        if entry.get("sha256") == recorded:
            return reasons
    reasons.append(
        "model artifact {!r} has no verified immutable source-commit binding".format(key)
    )
    return reasons


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

        # F2.2: the authoritative production source commits live in the overlay
        # contract (the real bundle's root production_source_commits is null).
        # Verify every entry against immutable Git blobs, then bind each bundle
        # artifact to either the exact simulator-local bytes or a verified
        # immutable source-commit record -- never the working tree.
        overlay_artifacts: dict[str, Any] = {}
        source_reasons: list[str] = []
        verified_sources: dict[str, dict[str, Any]] = {}
        if overlay is not None:
            bundle_record = overlay.get("model_bundle")
            if isinstance(bundle_record, dict):
                overlay_artifacts = (
                    bundle_record.get("artifacts")
                    if isinstance(bundle_record.get("artifacts"), dict)
                    else {}
                )
                source_reasons, verified_sources = _verify_overlay_source_commits(
                    overlay, production_root, impl_head
                )
        else:
            source_reasons = ["overlay contract is unavailable; cannot bind production source commits"]
        reasons.extend(source_reasons)

        artifacts = bundle.get("artifacts")
        if isinstance(artifacts, dict):
            for key, artifact in artifacts.items():
                if not isinstance(artifact, dict):
                    reasons.append("model artifact {!r} must be an object".format(key))
                    continue
                reasons.extend(
                    _bind_bundle_artifact(
                        key,
                        artifact,
                        overlay_artifacts,
                        verified_sources,
                        simulator_root,
                    )
                )
        details["verified_source_commits"] = sorted(verified_sources)

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

    # F2.3: the overlay scenario key set must equal the configured C/D/E set
    # exactly.  A configured id absent from the overlay and an overlay extra
    # that is not configured both fail before per-scenario comparison.
    if isinstance(overlay_scenarios, dict):
        configured_set = set(unique)
        overlay_set = set(overlay_scenarios)
        if configured_set != overlay_set:
            missing = sorted(configured_set - overlay_set)
            extra = sorted(overlay_set - configured_set)
            reasons.append(
                "overlay scenario set must equal the configured C/D/E set exactly "
                "(missing: {}; extra: {})".format(missing, extra)
            )

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


# ---------------------------------------------------------------------------
# F2.1 structural C++ bindings
# ---------------------------------------------------------------------------
def _bind_destructor(sanitized_pick: str, reasons: list[str]) -> None:
    """Bind the actual ``GraspNode::~GraspNode`` destructor body (F2.1): the
    bounded deadline, ``motion_runtime_.shutdown(...)``, the joined
    ``executor_thread_``, and the state-validity client reset.  Required tokens
    in another function do not count."""
    body = _cpp_find_function(sanitized_pick, "GraspNode::~GraspNode")
    if body is None:
        reasons.append("GraspNode::~GraspNode() definition must be present in the executable C++ structure")
        return
    if _cpp_has_preprocessor_conditional(body):
        reasons.append("GraspNode::~GraspNode() body must not contain conditional-preprocessor directives")
        return
    if "shutdown_deadline" not in body and "std::chrono::seconds(5)" not in body:
        reasons.append("GraspNode destructor must establish a bounded shutdown deadline")
    if "motion_runtime_.shutdown(" not in body:
        reasons.append("GraspNode destructor must call motion_runtime_.shutdown(...)")
    if "executor_thread_.joinable()" not in body or "executor_thread_.join()" not in body:
        reasons.append("GraspNode destructor must join the executor thread")
    if "check_state_validity_client_.reset()" not in body:
        reasons.append("GraspNode destructor must reset the state-validity client")


def _bind_managed_runtime(sanitized_runtime: str, reasons: list[str]) -> None:
    """Bind the managed MotionRuntime (F2.1): a real joined worker in its
    defining coordinator path and a joined coordinator in the runtime
    destructor, with bounded shutdown.  ``#if 0``/literal content never
    satisfies a token."""
    if "transaction->worker = std::thread(" not in sanitized_runtime:
        reasons.append("MotionRuntime must spawn the transaction worker as a managed std::thread")
    coord_body = _cpp_find_function(sanitized_runtime, "coordinator_main")
    if coord_body is None:
        reasons.append("coordinator_main() definition must be present in the executable C++ structure")
    else:
        if "transaction->worker.joinable()" not in coord_body or "transaction->worker.join()" not in coord_body:
            reasons.append("coordinator_main() must join the transaction worker thread")
    dtor_body = _cpp_find_function(sanitized_runtime, "~MotionRuntime")
    if dtor_body is None:
        reasons.append("MotionRuntime destructor must be present in the executable C++ structure")
    else:
        if "coordinator_.joinable()" not in dtor_body or "coordinator_.join()" not in dtor_body:
            reasons.append("MotionRuntime destructor must join the coordinator thread")
        if "shutdown(" not in dtor_body:
            reasons.append("MotionRuntime destructor must call shutdown(...)")


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

    sanitized_cpp: dict[str, str] = {}
    for rel in PROD_CPP_PATHS:
        text = _git_show_text(production_root, impl_head, rel)
        if text is None:
            reasons.append("immutable production C++ missing: " + rel)
            continue
        sanitized_cpp[rel] = _cpp_sanitize(text)

    if sanitized_cpp:
        combined = "\n".join(sanitized_cpp.values())
        if ".detach()" in combined:
            reasons.append("action lifecycle uses a detached thread (must be managed)")
        details["cpp_inspected"] = list(sanitized_cpp.keys())

        pick = sanitized_cpp.get(PROD_PICK_AND_PLACE_CPP_REL, "")
        if pick:
            _bind_destructor(pick, reasons)
        runtime = sanitized_cpp.get(PROD_ACTION_RUNTIME_CPP_REL, "")
        if runtime:
            _bind_managed_runtime(runtime, reasons)

    action_fields: dict[str, list[str]] = {}
    for schema in ACTION_SCHEMA_FILES:
        rel = PROD_ACTION_DIR_REL + "/" + schema
        text = _git_show_text(production_root, impl_head, rel)
        if text is None:
            reasons.append("immutable production .action missing: " + rel)
            continue
        action_fields[schema] = _parse_action_result_fields(text)
    details["action_result_fields"] = action_fields

    result_src = sanitized_cpp.get(PROD_ACTION_EXECUTION_CPP_REL)
    if result_src is None:
        reasons.append("immutable production action_execution.cpp missing")
    else:
        for builder, schema_name in BUILDER_SCHEMA.items():
            body = _cpp_find_function(result_src, builder)
            if body is None:
                reasons.append("result builder {}() definition must be present in the executable C++ structure".format(builder))
                continue
            if _cpp_has_preprocessor_conditional(body):
                reasons.append("{}() body must not contain conditional-preprocessor directives".format(builder))
                continue
            for field in action_fields.get(schema_name, []):
                if "result->{}".format(field) not in body:
                    reasons.append(
                        "result builder {!r} must write result->{} (per {})".format(builder, field, schema_name)
                    )

    if overlay is not None:
        production_overlay = overlay.get("production_overlay")
        if isinstance(production_overlay, dict):
            lifecycle = production_overlay.get("task_owned_lifecycle")
            if not isinstance(lifecycle, str) or not lifecycle:
                reasons.append("overlay production_overlay.task_owned_lifecycle is missing")

    passed = not reasons
    return StaticCheck(name="action-lifecycle", passed=passed, details=details, reasons=tuple(reasons))


def _bind_run_post_close_pick(sanitized_ownership: str, reasons: list[str]) -> None:
    """Bind ``run_post_close_pick`` branch ownership (F2.1): the SimOmpl
    obstruction guard must call ``confirms_obstruction()`` in its own condition;
    the ``ExecutionProfile::Hardware`` block must call collision-disabled
    ``execute_lift(ctx, false, ...)`` and no true lift; the SimOmpl lift block
    must call collision-aware ``execute_lift(ctx, true, ...)`` and no false lift.
    A true/false swap must fail."""
    body = _cpp_find_function(sanitized_ownership, "run_post_close_pick")
    if body is None:
        reasons.append("run_post_close_pick() definition must be present in the executable C++ structure")
        return
    if _cpp_has_preprocessor_conditional(body):
        reasons.append("run_post_close_pick() body must not contain conditional-preprocessor directives")
        return
    guard = None
    hardware = None
    sim_lift = None
    for block in _cpp_if_blocks(body):
        condition = block["condition"]
        if "ExecutionProfile::SimOmpl" in condition:
            if "confirms_obstruction" in condition:
                guard = block
            elif "execute_lift" in block["body"]:
                sim_lift = block
        if "ExecutionProfile::Hardware" in condition:
            hardware = block
    if guard is None:
        reasons.append("SimOmpl post-close must gate on close_result.confirms_obstruction() in its own condition")
    else:
        if "confirms_obstruction()" not in guard["condition"]:
            reasons.append("SimOmpl obstruction guard must call close_result.confirms_obstruction()")
    if hardware is None:
        reasons.append("hardware compatibility branch must be guarded by ExecutionProfile::Hardware")
    else:
        if "execute_lift(ctx, false," not in hardware["body"]:
            reasons.append("hardware branch must call collision-disabled execute_lift(ctx, false, ...)")
        if "execute_lift(ctx, true," in hardware["body"]:
            reasons.append("hardware branch must not call collision-aware execute_lift(ctx, true, ...)")
    if sim_lift is None:
        reasons.append("SimOmpl post-close branch must call collision-aware execute_lift(ctx, true, ...)")
    else:
        if "execute_lift(ctx, true," not in sim_lift["body"]:
            reasons.append("SimOmpl post-close branch must call collision-aware execute_lift(ctx, true, ...)")
        if "execute_lift(ctx, false," in sim_lift["body"]:
            reasons.append("SimOmpl post-close branch must not call collision-disabled execute_lift(ctx, false, ...)")


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
        sanitized_utils = _cpp_sanitize(utils)
        body = _cpp_find_function(sanitized_utils, "GraspNode::clean_planning_scene")
        if body is None:
            reasons.append("GraspNode::clean_planning_scene() definition must be present in the executable C++ structure")
        else:
            if _cpp_has_preprocessor_conditional(body):
                reasons.append("clean_planning_scene() body must not contain conditional-preprocessor directives")
            sim_block = None
            for block in _cpp_if_blocks(body):
                if "ExecutionProfile::SimOmpl" in block["condition"]:
                    sim_block = block
                    break
            if sim_block is None:
                reasons.append("clean_planning_scene() must gate on ExecutionProfile::SimOmpl")
            else:
                sim_body = sim_block["body"]
                if "return;" not in sim_body:
                    reasons.append("clean_planning_scene() SimOmpl block must contain a real return;")
                if "task_cleanup_remove_ids(" in sim_body or "applyPlanningScene(" in sim_body:
                    reasons.append("clean_planning_scene() must not run the hardware scene cleanup on the SimOmpl branch")
            if "task_cleanup_remove_ids(" not in body:
                reasons.append("hardware scene cleanup must route through task-owned task_cleanup_remove_ids()")
            if "applyPlanningScene(" not in body:
                reasons.append("hardware scene cleanup must call applyPlanningScene(...)")
        details["clean_planning_scene"] = body is not None

    ownership = _git_show_text(production_root, impl_head, PROD_SCENE_OWNERSHIP_CPP_REL)
    if ownership is None:
        reasons.append("immutable production scene_ownership.cpp missing")
    else:
        sanitized_ownership = _cpp_sanitize(ownership)
        _bind_run_post_close_pick(sanitized_ownership, reasons)

    pick = _git_show_text(production_root, impl_head, PROD_PICK_AND_PLACE_CPP_REL)
    if pick is None:
        reasons.append("immutable production pick_and_place.cpp missing")
    else:
        sanitized_pick = _cpp_sanitize(pick)
        body = _cpp_find_function(sanitized_pick, "GraspNode::move_straight")
        if body is None:
            reasons.append("GraspNode::move_straight() definition must be present in the executable C++ structure")
        else:
            if _cpp_has_preprocessor_conditional(body):
                reasons.append("move_straight() body must not contain conditional-preprocessor directives")
            if "request.avoid_collisions = avoid_collisions;" not in body:
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
                # F2.2 pinned prerequisite: the authoritative overlay production
                # source-commit mapping must be non-empty, structurally complete,
                # and every entry content-verified against immutable Git blobs.
                # The same verified mapping drives the model artifact binding, so
                # this is never vacuous and never merely ``commit == impl_head``.
                if overlay is not None:
                    source_reasons, _verified = _verify_overlay_source_commits(
                        overlay, production_root, prod_impl
                    )
                    reasons.extend(source_reasons)
                    details["pinned_source_commits"] = sorted(_verified)
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
