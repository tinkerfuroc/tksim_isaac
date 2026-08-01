"""ROS-free canonical Tinker manipulation model contract.

This module produces and validates the exact normalized selected-subgraph
contract consumed by the production ``xarm_moveit_config`` model-bundle
validator (``launch/lib/tinker_model_bundle.py``).  It imports neither ROS,
Isaac Sim, nor simulator-extension packages and runs under both simulator
CPython 3.12 and system Humble CPython 3.10.

The contract is deliberately narrow: simulator and production artifacts are
parsed independently and only the selected manipulation subgraph is compared.
Backend-specific ``<ros2_control>`` blocks are treated as transport metadata
after parsing, while selected links, joints, limits, the fixed mount, SRDF
groups, the end-effector parent, touch links, collision geometry, and
kinematics semantics are retained.
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Mapping, Sequence

SCHEMA_VERSION = 1
PRODUCER = {"name": "tinker_sim_bridge.model_bundle", "version": "1"}
ARTIFACT_NAMES = (
    "simulator_full_urdf",
    "planning_urdf",
    "planning_srdf",
    "joint_limits",
    "kinematics",
)
ARM_JOINTS = tuple("joint{}".format(i) for i in range(1, 8))
ORDERED_JOINTS = ARM_JOINTS + ("drive_joint",)
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
GROUPS = {"arm": "xarm7", "gripper": "xarm_gripper"}
_HEX64 = re.compile(r"^(?!0{64}$)[0-9a-f]{64}$")
_REQUIRED_TOP_LEVEL = {
    "schema_version",
    "producer",
    "artifacts",
    "normalization",
    "contract",
    "structural_fingerprint",
}


class ModelContractError(RuntimeError):
    """Typed validation failure with a stable machine-readable classification."""

    def __init__(self, code: str, message: str, *, field: str | None = None) -> None:
        super().__init__(message)
        self.code = code
        self.field = field
        self.message = message

    def __repr__(self) -> str:
        return "ModelContractError(code={!r}, message={!r}, field={!r})".format(
            self.code, self.message, self.field
        )


def sha256_file(path: Path | str) -> str:
    """Return the SHA-256 digest of the exact bytes at *path*."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")


def sha256_json(value: Any) -> str:
    return hashlib.sha256(canonical_json(value)).hexdigest()


def contract_fingerprint(contract: Mapping[str, object]) -> str:
    """Return the canonical structural fingerprint of a normalized contract."""
    return sha256_json(contract)


def _error(code: str, message: str, field: str | None = None) -> ModelContractError:
    return ModelContractError(code, message, field=field)


def _finite(value: Any, *, field: str) -> Any:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise _error("invalid_value", "{} must be finite".format(field), field)
        return value
    if isinstance(value, list):
        return [_finite(item, field="{}[]".format(field)) for item in value]
    if isinstance(value, tuple):
        return [_finite(item, field="{}[]".format(field)) for item in value]
    if isinstance(value, dict):
        return {str(k): _finite(v, field="{}.{}".format(field, k)) for k, v in value.items()}
    return value


def _number_list(value: Any, field: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise _error("invalid_value", "{} must contain three numbers".format(field), field)
    result = []
    for index, item in enumerate(value):
        if isinstance(item, bool):
            raise _error("invalid_value", "{}[{}] must be finite".format(field, index), field)
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise _error("invalid_value", "{}[{}] must be numeric".format(field, index), field) from exc
        if not math.isfinite(number):
            raise _error("invalid_value", "{}[{}] must be finite".format(field, index), field)
        result.append(number)
    return result


def _strip_prefix(name: str, prefix: str) -> str:
    if prefix and name.startswith(prefix):
        return name[len(prefix):]
    return name


def _vec(element: ET.Element | None, tag: str, default: Sequence[float] = (0.0, 0.0, 0.0)) -> list[float]:
    if element is None:
        return [float(x) for x in default]
    text = element.attrib.get(tag, "0 0 0").split()
    if len(text) != 3:
        raise _error("invalid_urdf", "{} must have three values".format(tag), tag)
    try:
        values = [float(x) for x in text]
    except ValueError as exc:
        raise _error("invalid_urdf", "{} contains a non-number".format(tag), tag) from exc
    if not all(math.isfinite(x) for x in values):
        raise _error("invalid_urdf", "{} contains a non-finite value".format(tag), tag)
    return values


def _origin(element: ET.Element | None) -> dict[str, list[float]]:
    return {"xyz": _vec(element, "xyz"), "rpy": _vec(element, "rpy")}


def _parse_float(value: str | None, field: str) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except ValueError as exc:
        raise _error("invalid_urdf", "{} is not numeric".format(field), field) from exc
    if not math.isfinite(result):
        raise _error("invalid_urdf", "{} is not finite".format(field), field)
    return result


def _parse_xml_string(xml_text: str, kind: str) -> ET.Element:
    try:
        return ET.fromstring(xml_text)
    except (TypeError, ET.ParseError) as exc:
        raise _error("invalid_{}".format(kind), "unable to parse supplied {} XML".format(kind)) from exc


def _geometry(link: ET.Element, link_name: str) -> list[dict[str, Any]]:
    geometries: list[dict[str, Any]] = []
    for collision in link.findall("collision"):
        geometry = collision.find("geometry")
        if geometry is None or len(geometry) != 1:
            raise _error("invalid_urdf", "collision geometry missing for {}".format(link_name), link_name)
        shape = geometry[0]
        item: dict[str, Any] = {"origin": _origin(collision.find("origin")), "type": shape.tag}
        if shape.tag == "box":
            item["size"] = _number_list(shape.attrib.get("size", "").split(), "box.size")
        elif shape.tag == "cylinder":
            item["radius"] = _parse_float(shape.attrib.get("radius"), "cylinder.radius")
            item["length"] = _parse_float(shape.attrib.get("length"), "cylinder.length")
        elif shape.tag == "sphere":
            item["radius"] = _parse_float(shape.attrib.get("radius"), "sphere.radius")
        elif shape.tag == "mesh":
            filename = shape.attrib.get("filename")
            if not filename:
                raise _error("invalid_urdf", "mesh filename missing for {}".format(link_name), link_name)
            item["filename"] = filename
            if "scale" in shape.attrib:
                item["scale"] = _number_list(shape.attrib["scale"].split(), "mesh.scale")
        else:
            raise _error("invalid_urdf", "unsupported collision geometry {}".format(shape.tag), link_name)
        geometries.append(_finite(item, field="collision.{}".format(link_name)))
    return geometries


def _joint_record(joint: ET.Element, prefix: str) -> dict[str, Any]:
    name = _strip_prefix(joint.attrib["name"], prefix)
    parent_element = joint.find("parent")
    child_element = joint.find("child")
    if parent_element is None or child_element is None:
        raise _error("invalid_urdf", "joint {} lacks parent or child".format(name), name)
    record: dict[str, Any] = {
        "name": name,
        "type": joint.attrib.get("type", ""),
        "parent": _strip_prefix(parent_element.attrib.get("link", ""), prefix),
        "child": _strip_prefix(child_element.attrib.get("link", ""), prefix),
        "origin": _origin(joint.find("origin")),
    }
    axis = joint.find("axis")
    record["axis"] = _vec(axis, "xyz", (0.0, 0.0, 1.0)) if axis is not None else [0.0, 0.0, 1.0]
    limit = joint.find("limit")
    record["limit"] = {
        "lower": _parse_float(limit.attrib.get("lower") if limit is not None else None, "{}.lower".format(name)),
        "upper": _parse_float(limit.attrib.get("upper") if limit is not None else None, "{}.upper".format(name)),
        "effort": _parse_float(limit.attrib.get("effort") if limit is not None else None, "{}.effort".format(name)),
        "velocity": _parse_float(limit.attrib.get("velocity") if limit is not None else None, "{}.velocity".format(name)),
    }
    return _finite(record, field="joint.{}".format(name))


def _group_members(srdf: ET.Element, prefix: str) -> dict[str, list[ET.Element]]:
    result: dict[str, list[ET.Element]] = {}
    for group in srdf.findall("group"):
        name = _strip_prefix(group.attrib.get("name", ""), prefix)
        if name:
            result[name] = list(group)
    return result


def _resolve_group(
    name: str,
    groups: Mapping[str, list[ET.Element]],
    joints: Mapping[str, dict[str, Any]],
    prefix: str,
    stack: tuple[str, ...] = (),
) -> dict[str, list[str]]:
    if name in stack:
        raise _error("invalid_srdf", "recursive group cycle at {}".format(name), name)
    members = groups.get(name)
    if members is None:
        raise _error("invalid_srdf", "group {} is not defined".format(name), name)
    result = {"joints": [], "links": []}

    def add(kind: str, value: str) -> None:
        value = _strip_prefix(value, prefix)
        if value and value not in result[kind]:
            result[kind].append(value)

    for member in members:
        if member.tag == "joint":
            joint_name = _strip_prefix(member.attrib.get("name", ""), prefix)
            add("joints", joint_name)
            if joint_name not in joints:
                raise _error("invalid_srdf", "group {} references unknown joint {}".format(name, joint_name), name)
            add("links", joints[joint_name]["parent"])
            add("links", joints[joint_name]["child"])
        elif member.tag == "link":
            add("links", member.attrib.get("name", ""))
        elif member.tag == "subgroup":
            nested = _resolve_group(_strip_prefix(member.attrib.get("name", ""), prefix), groups, joints, prefix, stack + (name,))
            for joint in nested["joints"]:
                add("joints", joint)
            for link in nested["links"]:
                add("links", link)
        elif member.tag == "chain":
            base = _strip_prefix(member.attrib.get("base_link", ""), prefix)
            tip = _strip_prefix(member.attrib.get("tip_link", ""), prefix)
            current = base
            visited: set[str] = set()
            add("links", base)
            while current != tip:
                candidates = [record for record in joints.values() if record["parent"] == current]
                if len(candidates) != 1 or current in visited:
                    raise _error("invalid_srdf", "cannot resolve chain {} -> {}".format(base, tip), name)
                record = candidates[0]
                visited.add(current)
                add("joints", record["name"])
                current = record["child"]
                add("links", current)
        elif member.tag not in {"group_state", "disable_collisions"}:
            raise _error("invalid_srdf", "unsupported group member {}".format(member.tag), name)
    return result


def _find_chain(joints: Mapping[str, Mapping[str, Any]], base: str, tip: str) -> tuple[list[str], list[str]]:
    """Resolve one deterministic URDF parent-child path from base to tip."""
    adjacency: dict[str, list[Mapping[str, Any]]] = {}
    for record in joints.values():
        adjacency.setdefault(str(record["parent"]), []).append(record)

    def walk(link: str, seen: tuple[str, ...]) -> tuple[list[str], list[str]] | None:
        if link == tip:
            return [], [link]
        candidates = [item for item in adjacency.get(link, []) if item["name"] not in seen]
        paths = []
        for record in candidates:
            tail = walk(str(record["child"]), seen + (str(record["name"]),))
            if tail is not None:
                paths.append(([str(record["name"])] + tail[0], [link] + tail[1]))
        if len(paths) > 1:
            raise _error("semantic_mismatch", "URDF has multiple {} -> {} paths".format(base, tip), "chain")
        return paths[0] if paths else None

    result = walk(base, ())
    if result is None:
        raise _error("semantic_mismatch", "cannot resolve URDF chain {} -> {}".format(base, tip), "chain")
    return result


def _actuated_sequence(group: Mapping[str, Sequence[str]], joints: Mapping[str, Mapping[str, Any]]) -> list[str]:
    actuated = [name for name in group["joints"] if joints[name]["type"] not in {"fixed", "floating", "planar"}]
    if tuple(name for name in actuated if name in ARM_JOINTS) != ARM_JOINTS:
        raise _error("semantic_mismatch", "xarm7 actuated sequence is not joint1 through joint7", "xarm7.joints")
    extras = [name for name in actuated if name not in ARM_JOINTS]
    if extras:
        raise _error("semantic_mismatch", "xarm7 has unexpected actuated joints {}".format(extras), "xarm7.joints")
    return actuated


def _validate_simulator_drive_control(root: ET.Element) -> dict[str, Any]:
    matches = []
    for control in root.findall("ros2_control"):
        for joint in control.findall("joint"):
            if joint.attrib.get("name") == "drive_joint":
                matches.append(joint)
    if len(matches) != 1:
        raise _error("semantic_mismatch", "simulator URDF must contain exactly one ros2_control drive_joint", "simulator_full_urdf.ros2_control")
    joint = matches[0]
    states = [item.attrib.get("name") for item in joint.findall("state_interface")]
    commands = [item.attrib.get("name") for item in joint.findall("command_interface")]
    if sorted(states) != ["effort", "position", "velocity"] or commands:
        raise _error("semantic_mismatch", "drive_joint requires position/velocity/effort state interfaces and no command interfaces", "drive_joint.ros2_control")
    return {"joint": "drive_joint", "state_interfaces": ["position", "velocity", "effort"], "command_interfaces": []}


def canonical_contract(
    simulator_urdf_xml: str,
    planning_urdf_xml: str,
    planning_srdf_xml: str,
    joint_limits: Mapping[str, Mapping[str, object]],
    kinematics: Mapping[str, Mapping[str, object]],
    *,
    prefix: str,
    mount: Mapping[str, object],
) -> dict[str, object]:
    """Compute the canonical selected-subgraph contract from parsed artifact text.

    *joint_limits* is the inner ``{joint_name: {limit...}}`` mapping extracted
    from a ``joint_limits:`` YAML root.  *kinematics* is the parsed kinematics
    YAML mapping (keyed by planning group).  *mount* is the declared
    ``{parent, child, xyz, rpy}`` fixed mount and must be the exact zero
    ``world -> base_link`` transform.
    """
    sim_root = _parse_xml_string(simulator_urdf_xml, "urdf")
    plan_root = _parse_xml_string(planning_urdf_xml, "urdf")
    srdf_root = _parse_xml_string(planning_srdf_xml, "srdf")
    yaml_joint_limits = joint_limits
    kinematics_root = kinematics

    def parse_urdf(root: ET.Element) -> tuple[dict[str, dict[str, Any]], dict[str, ET.Element], dict[str, Any]]:
        raw_links = [item.attrib.get("name", "") for item in root.findall("link")]
        raw_joint_names = [item.attrib.get("name", "") for item in root.findall("joint")]
        if prefix and (prefix + "base_link" not in raw_links or prefix + "joint1" not in raw_joint_names):
            raise _error("semantic_mismatch", "declared prefix is not present on the selected URDF graph", "normalization.prefix")
        links = {_strip_prefix(item.attrib.get("name", ""), prefix): item for item in root.findall("link")}
        parsed_joints = {}
        for element in root.findall("joint"):
            record = _joint_record(element, prefix)
            parsed_joints[record["name"]] = record
        if "base_link" not in links:
            raise _error("semantic_mismatch", "base_link is absent after prefix normalization", "base_link")
        fixed_mounts = [record for record in parsed_joints.values() if record["type"] == "fixed" and record["child"] == "base_link"]
        if len(fixed_mounts) != 1:
            raise _error("semantic_mismatch", "expected exactly one fixed mount to base_link", "mount")
        actual_mount = {
            "parent": fixed_mounts[0]["parent"],
            "child": fixed_mounts[0]["child"],
            "xyz": fixed_mounts[0]["origin"]["xyz"],
            "rpy": fixed_mounts[0]["origin"]["rpy"],
        }
        exact_mount = {"parent": "world", "child": "base_link", "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}
        if mount != exact_mount or actual_mount != exact_mount:
            raise _error("semantic_mismatch", "mount must be the exact finite zero world -> base_link transform", "mount")
        return parsed_joints, links, actual_mount

    simulator_control = _validate_simulator_drive_control(sim_root)
    sim_joints, sim_links, _ = parse_urdf(sim_root)
    plan_joints, plan_links, _ = parse_urdf(plan_root)

    groups = _group_members(srdf_root, prefix)
    sim_resolved = {name: _resolve_group(name, groups, sim_joints, prefix) for name in ("xarm7", "xarm_gripper")}
    plan_resolved = {name: _resolve_group(name, groups, plan_joints, prefix) for name in ("xarm7", "xarm_gripper")}
    if sim_resolved != plan_resolved:
        raise _error("semantic_mismatch", "SRDF groups resolve differently against simulator and planning URDF", "groups")
    resolved = plan_resolved
    actuated_arm = _actuated_sequence(resolved["xarm7"], plan_joints)
    if tuple(resolved["xarm_gripper"]["joints"]) != ("drive_joint",):
        raise _error("semantic_mismatch", "xarm_gripper must contain drive_joint", "xarm_gripper.joints")
    support_joints = [name for name in resolved["xarm7"]["joints"] if name not in actuated_arm]

    end_effectors = [item for item in srdf_root.findall("end_effector") if _strip_prefix(item.attrib.get("group", ""), prefix) == "xarm_gripper"]
    if len(end_effectors) != 1:
        raise _error("semantic_mismatch", "one xarm_gripper end-effector is required", "end_effector")
    end_effector = end_effectors[0]
    parent_link = _strip_prefix(end_effector.attrib.get("parent_link", ""), prefix)
    if parent_link != "link_tcp":
        raise _error("semantic_mismatch", "xarm_gripper end-effector parent must be link_tcp", "end_effector.parent_link")
    touch_links = list(resolved["xarm_gripper"]["links"])
    if tuple(touch_links) != TOUCH_LINKS:
        raise _error("semantic_mismatch", "resolved xarm_gripper touch links do not match the eight-link contract", "touch_links")
    if parent_link not in touch_links:
        raise _error("semantic_mismatch", "end-effector parent is not a touch link", "touch_links")

    all_joints = plan_joints
    for name in ORDERED_JOINTS:
        if name not in all_joints:
            raise _error("semantic_mismatch", "selected joint {} is absent from URDF".format(name), name)
    for name in ORDERED_JOINTS:
        if name not in sim_joints:
            raise _error("semantic_mismatch", "selected joint {} is absent from simulator URDF".format(name), name)
    chain_joints, chain_links = _find_chain(plan_joints, "base_link", "link_tcp")
    if not set(actuated_arm).issubset(chain_joints):
        raise _error("semantic_mismatch", "the actuated arm is not on the base_link -> link_tcp chain", "chain")
    selected_links: list[str] = []
    for link in chain_links + list(resolved["xarm_gripper"]["links"]):
        if link not in selected_links:
            selected_links.append(link)
    if "link_tcp" not in selected_links or "link_tcp" not in plan_links:
        raise _error("semantic_mismatch", "link_tcp is absent from the resolved gripper graph", "link_tcp")

    selected_limits: dict[str, Any] = {}
    selected_joint_semantics: dict[str, Any] = {}
    for name in ORDERED_JOINTS:
        urdf_record = all_joints[name]
        sim_record = sim_joints[name]
        if any(urdf_record["limit"][field] is None for field in ("lower", "upper", "effort", "velocity")):
            raise _error("invalid_limits", "URDF limits for {} are incomplete".format(name), name)
        entry = yaml_joint_limits.get(name)
        if not isinstance(entry, dict):
            raise _error("invalid_limits", "YAML limits for {} are missing or not a mapping".format(name), name)
        required = {"has_velocity_limits", "max_velocity", "has_acceleration_limits", "max_acceleration"}
        if name in ARM_JOINTS:
            required |= {"min_position", "max_position"}
        allowed = required | {"has_position_limits", "max_effort"}
        if name == "drive_joint":
            allowed |= {"min_position", "max_position"}
        if not required.issubset(entry) or not set(entry).issubset(allowed):
            raise _error("invalid_limits", "YAML limits schema for {} is incomplete or has unknown fields".format(name), name)
        if entry["has_velocity_limits"] is not True or entry["has_acceleration_limits"] is not True:
            raise _error("invalid_limits", "YAML velocity and acceleration limits for {} must be enabled".format(name), name)
        for field in required - {"has_velocity_limits", "has_acceleration_limits"}:
            value = entry[field]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise _error("invalid_limits", "YAML limit {}.{} must be finite numeric".format(name, field), name)
        selected_limits[name] = {"state_only": name == "drive_joint", "urdf": urdf_record["limit"], "yaml": {key: entry[key] for key in sorted(entry)}}
        selected_joint_semantics[name] = {
            "type": urdf_record["type"],
            "parent": urdf_record["parent"],
            "child": urdf_record["child"],
            "axis": urdf_record["axis"],
            "origin": urdf_record["origin"],
            "limit": urdf_record["limit"],
        }
        sim_semantics = {
            "type": sim_record["type"],
            "parent": sim_record["parent"],
            "child": sim_record["child"],
            "axis": sim_record["axis"],
            "origin": sim_record["origin"],
            "limit": sim_record["limit"],
        }
        if selected_joint_semantics[name] != sim_semantics:
            raise _error("semantic_mismatch", "joint {} differs between simulator and planning URDF".format(name), name)

    support_joint_semantics: dict[str, Any] = {}
    for name in support_joints:
        if name not in sim_joints:
            raise _error("semantic_mismatch", "support joint {} is absent from simulator URDF".format(name), name)
        if plan_joints[name] != sim_joints[name]:
            raise _error("semantic_mismatch", "support joint {} differs between simulator and planning URDF".format(name), name)
        support_joint_semantics[name] = plan_joints[name]

    collision_geometry: dict[str, Any] = {}
    for link_name in selected_links:
        if link_name not in plan_links or link_name not in sim_links:
            raise _error("semantic_mismatch", "selected link {} is absent from both URDFs".format(link_name), link_name)
        plan_geometry = _geometry(plan_links[link_name], link_name)
        sim_geometry = _geometry(sim_links[link_name], link_name)
        if plan_geometry != sim_geometry:
            raise _error("semantic_mismatch", "collision geometry differs for {}".format(link_name), link_name)
        collision_geometry[link_name] = plan_geometry

    kinematics_contract: dict[str, Any] = {}
    for group_name in ("xarm7", "xarm_gripper"):
        value = kinematics_root.get(group_name, {})
        if value is None:
            value = {}
        if not isinstance(value, dict):
            raise _error("invalid_kinematics", "kinematics group {} must be a mapping".format(group_name), group_name)
        kinematics_contract[group_name] = _finite(dict(value), field="kinematics." + group_name)
    expected_base, expected_tip = "base_link", "link_tcp"
    declared_base = kinematics_contract["xarm7"].get("base_link", expected_base)
    declared_tip = kinematics_contract["xarm7"].get("tip_link", expected_tip)
    if declared_base != expected_base or declared_tip != expected_tip:
        raise _error("semantic_mismatch", "declared xarm7 kinematics base_link/tip_link do not match the selected chain", "kinematics.xarm7")
    kinematics_contract["xarm7"]["base_link"] = expected_base
    kinematics_contract["xarm7"]["tip_link"] = expected_tip
    kinematics_contract["xarm7"] = _finite(kinematics_contract["xarm7"], field="kinematics.xarm7")
    if not kinematics_contract["xarm7"].get("kinematics_solver"):
        raise _error("semantic_mismatch", "xarm7 kinematics_solver is required", "kinematics.xarm7")

    contract = {
        "planning_frame": "base_link",
        "tcp_link": "link_tcp",
        "arm_joints": list(ARM_JOINTS),
        "gripper_joint": "drive_joint",
        "support_joints": support_joints,
        "chain_joints": chain_joints,
        "simulator_control": simulator_control,
        "groups": {
            "xarm7": {"joints": list(resolved["xarm7"]["joints"]), "links": list(resolved["xarm7"]["links"])},
            "xarm_gripper": {"joints": list(resolved["xarm_gripper"]["joints"]), "links": list(resolved["xarm_gripper"]["links"])},
        },
        "end_effector": {"group": "xarm_gripper", "parent_link": parent_link},
        "touch_links": touch_links,
        "joint_limits": selected_limits,
        "joint_semantics": selected_joint_semantics,
        "support_joint_semantics": support_joint_semantics,
        "collision_geometry": collision_geometry,
        "kinematics": kinematics_contract,
        "mount": mount,
        "selected_links": selected_links,
    }
    return _finite(contract, field="contract")


def _normalize_groups_value(value: object) -> dict[str, str]:
    if isinstance(value, dict):
        names = {str(item) for item in value.values()}
    elif isinstance(value, list):
        names = {str(item) for item in value}
    else:
        raise _error("invalid_manifest", "normalization.groups must be a mapping or list", "normalization.groups")
    if names != set(GROUPS.values()):
        raise _error("invalid_manifest", "normalization.groups must name xarm7 and xarm_gripper", "normalization.groups")
    return dict(GROUPS)


def validate_bundle_manifest(manifest: Mapping[str, object]) -> None:
    """Validate a model-bundle manifest structure without recomputing bytes."""
    if not isinstance(manifest, dict):
        raise _error("invalid_manifest", "manifest must contain a JSON object")
    missing = sorted(_REQUIRED_TOP_LEVEL - set(manifest))
    if missing:
        raise _error("invalid_manifest", "manifest missing required fields {}".format(missing))
    if manifest["schema_version"] != SCHEMA_VERSION:
        raise _error("schema_version", "unsupported model bundle schema version", "schema_version")
    if manifest["producer"] != PRODUCER:
        raise _error("producer", "unexpected model bundle producer", "producer")
    artifacts = manifest["artifacts"]
    if not isinstance(artifacts, dict) or set(artifacts) != set(ARTIFACT_NAMES):
        raise _error("invalid_manifest", "artifacts must contain exactly the five canonical entries", "artifacts")
    for name in ARTIFACT_NAMES:
        entry = artifacts[name]
        if not isinstance(entry, dict) or set(entry) != {"path", "sha256"}:
            raise _error("invalid_manifest", "artifact {} must contain path and sha256".format(name), "artifacts." + name)
        path = entry["path"]
        digest = entry["sha256"]
        if not isinstance(path, str) or not path or not path.startswith("/"):
            raise _error("invalid_manifest", "artifact {} path must be an absolute path string".format(name), "artifacts." + name)
        if not isinstance(digest, str) or not _HEX64.fullmatch(digest):
            raise _error("artifact_digest", "artifact {} has an invalid nonzero SHA-256".format(name), "artifacts." + name)
    normalization = manifest["normalization"]
    if not isinstance(normalization, dict):
        raise _error("invalid_manifest", "normalization must be an object", "normalization")
    norm_required = {"prefix", "mount", "groups", "ordered_joints", "selected_links"}
    if not norm_required.issubset(normalization):
        raise _error("invalid_manifest", "normalization is incomplete", "normalization")
    if not isinstance(normalization["prefix"], str):
        raise _error("invalid_manifest", "normalization.prefix must be a string", "normalization.prefix")
    _normalize_groups_value(normalization["groups"])
    if tuple(normalization["ordered_joints"]) != ORDERED_JOINTS:
        raise _error("invalid_manifest", "normalization.ordered_joints is not canonical", "normalization.ordered_joints")
    selected_links = normalization["selected_links"]
    if not isinstance(selected_links, list) or not selected_links:
        raise _error("invalid_manifest", "normalization.selected_links must be nonempty", "normalization.selected_links")
    mount = normalization["mount"]
    if not isinstance(mount, dict) or not {"parent", "child", "xyz", "rpy"}.issubset(mount):
        raise _error("invalid_manifest", "normalization.mount is incomplete", "normalization.mount")
    _number_list(mount["xyz"], "normalization.mount.xyz")
    _number_list(mount["rpy"], "normalization.mount.rpy")
    contract = manifest["contract"]
    if not isinstance(contract, dict):
        raise _error("invalid_manifest", "contract must be an object", "contract")
    _finite(contract, field="contract")
    fingerprint = manifest["structural_fingerprint"]
    if not isinstance(fingerprint, str) or not _HEX64.fullmatch(fingerprint):
        raise _error("structural_fingerprint", "structural_fingerprint is not a nonzero lowercase SHA-256", "structural_fingerprint")
    if fingerprint != sha256_json(contract):
        raise _error("structural_mismatch", "structural_fingerprint does not match contract", "structural_fingerprint")


__all__ = [
    "ARM_JOINTS",
    "ARTIFACT_NAMES",
    "GROUPS",
    "ModelContractError",
    "ORDERED_JOINTS",
    "PRODUCER",
    "SCHEMA_VERSION",
    "TOUCH_LINKS",
    "canonical_contract",
    "canonical_json",
    "contract_fingerprint",
    "sha256_file",
    "sha256_json",
    "validate_bundle_manifest",
]
