"""Pure model-contract tests for the canonical manipulation bundle schema."""
from __future__ import annotations

import json
import sys
from hashlib import sha256
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

import pytest
import yaml

from tinker_sim_bridge.model_contract import (
    GROUPS,
    ORDERED_JOINTS,
    TOUCH_LINKS,
    ModelContractError,
    canonical_contract,
    contract_fingerprint,
    sha256_json,
    validate_bundle_manifest,
)

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
SELECTED_LINKS = ["base_link", "link_base"] + ["link{}".format(i) for i in range(1, 8)] + ["link_eef", "xarm_gripper_base_link", "link_tcp"] + [link for link in TOUCH_LINKS if link not in {"xarm_gripper_base_link", "link_tcp"}]

MOUNT = {"parent": "world", "child": "base_link", "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}


def _link(name: str, size: str = "0.10 0.10 0.10") -> str:
    return '  <link name="{name}"><collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="{size}"/></geometry></collision></link>\n'.format(name=name, size=size)


def _joint(name: str, parent: str, child: str, axis: str = "0 0 1", lower: str = "-1.0", upper: str = "1.0") -> str:
    return '  <joint name="{name}" type="revolute"><parent link="{parent}"/><child link="{child}"/><origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="{axis}"/><limit lower="{lower}" upper="{upper}" effort="20" velocity="2.0"/></joint>\n'.format(name=name, parent=parent, child=child, axis=axis, lower=lower, upper=upper)


URDF_CORE = '<?xml version="1.0"?><robot name="fixture">\n' + _link("world") + _link("base_link") + _link("link_base") + """  <joint name="world_joint" type="fixed"><parent link="world"/><child link="base_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
  <joint name="base_to_arm_joint" type="fixed"><parent link="base_link"/><child link="link_base"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
""" + "".join(
    _link("link{}".format(i)) for i in range(1, 8)
) + "".join(
    _joint("joint1", "link_base", "link1", "0 0 1", "-6.28", "6.28")
    if i == 1 else _joint("joint{}".format(i), "link{}".format(i - 1), "link{}".format(i), "0 1 0", "-2.0", "2.0")
    for i in range(1, 8)
) + _link("link_eef") + _link("xarm_gripper_base_link") + """  <joint name="joint_eef" type="fixed"><parent link="link7"/><child link="link_eef"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
  <joint name="gripper_fix" type="fixed"><parent link="link_eef"/><child link="xarm_gripper_base_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
""" + "".join(_link(name) for name in TOUCH_LINKS[1:-1]) + _link("link_tcp") + _joint("drive_joint", "xarm_gripper_base_link", "left_outer_knuckle", "1 0 0", "0.0", "0.8") + """  <joint name="joint_tcp" type="fixed"><parent link="xarm_gripper_base_link"/><child link="link_tcp"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
</robot>
"""

SRDF_XML = """<?xml version="1.0"?><robot name="fixture">
  <group name="xarm7"><joint name="world_joint"/><joint name="base_to_arm_joint"/><joint name="joint1"/><joint name="joint2"/><joint name="joint3"/><joint name="joint4"/><joint name="joint5"/><joint name="joint6"/><joint name="joint7"/><joint name="joint_eef"/><joint name="gripper_fix"/><joint name="joint_tcp"/></group>
  <group name="xarm_gripper">
    <joint name="drive_joint"/>
    <link name="xarm_gripper_base_link"/><link name="left_outer_knuckle"/><link name="left_finger"/>
    <link name="left_inner_knuckle"/><link name="right_inner_knuckle"/><link name="right_outer_knuckle"/>
    <link name="right_finger"/><link name="link_tcp"/>
  </group>
  <end_effector name="xarm_gripper_eef" parent_link="link_tcp" group="xarm_gripper"/>
</robot>
"""

LIMITS_YAML = "joint_limits:\n" + "".join(
    "  {name}:\n    has_velocity_limits: true\n    max_velocity: 2.0\n    has_acceleration_limits: true\n    max_acceleration: 10.0\n    min_position: {lower}\n    max_position: {upper}\n".format(name=name, lower="-6.28" if name == "joint1" else "-2.0" if name != "drive_joint" else "0.0", upper="6.28" if name == "joint1" else "2.0" if name != "drive_joint" else "0.8")
    for name in ORDERED_JOINTS
)

KINEMATICS_YAML = """xarm7:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 5
"""


def add_topic_control_block(urdf_xml: str) -> str:
    block = '  <ros2_control name="fixture_sim" type="system"><hardware><plugin>mock_components/GenericSystem</plugin></hardware><joint name="drive_joint"><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint></ros2_control>\n'
    return urdf_xml.replace("</robot>", block + "</robot>")


def parse_limits() -> dict[str, dict[str, object]]:
    root = yaml.safe_load(LIMITS_YAML)
    return root["joint_limits"]


def parse_kinematics() -> dict[str, dict[str, object]]:
    return yaml.safe_load(KINEMATICS_YAML)


def build_contract(sim_xml: str | None = None, plan_xml: str | None = None, srdf_xml: str | None = None, kinematics=None, prefix: str = "", mount=None) -> dict[str, object]:
    return canonical_contract(
        sim_xml if sim_xml is not None else add_topic_control_block(URDF_CORE),
        plan_xml if plan_xml is not None else URDF_CORE,
        srdf_xml if srdf_xml is not None else SRDF_XML,
        parse_limits(),
        kinematics if kinematics is not None else parse_kinematics(),
        prefix=prefix,
        mount=mount if mount is not None else MOUNT,
    )


def valid_manifest() -> dict[str, object]:
    contract = build_contract()
    return {
        "schema_version": 1,
        "producer": {"name": "tinker_sim_bridge.model_bundle", "version": "1"},
        "artifacts": {
            name: {"path": "/tmp/{}.urdf".format(name), "sha256": "a" * 64}
            for name in ("simulator_full_urdf", "planning_urdf", "planning_srdf", "joint_limits", "kinematics")
        },
        "normalization": {
            "prefix": "",
            "mount": MOUNT,
            "groups": GROUPS,
            "ordered_joints": list(ORDERED_JOINTS),
            "selected_links": SELECTED_LINKS,
        },
        "contract": contract,
        "structural_fingerprint": contract_fingerprint(contract),
    }


def test_canonical_contract_populates_full_selected_fields() -> None:
    contract = build_contract()
    assert contract["planning_frame"] == "base_link"
    assert contract["tcp_link"] == "link_tcp"
    assert contract["arm_joints"] == ARM_JOINTS
    assert contract["gripper_joint"] == "drive_joint"
    assert contract["touch_links"] == list(TOUCH_LINKS)
    assert contract["end_effector"] == {"group": "xarm_gripper", "parent_link": "link_tcp"}
    assert contract["groups"]["xarm7"]["joints"] == ["world_joint", "base_to_arm_joint"] + ARM_JOINTS + ["joint_eef", "gripper_fix", "joint_tcp"]
    assert contract["groups"]["xarm_gripper"]["joints"] == ["drive_joint"]
    assert contract["mount"] == MOUNT
    assert contract["selected_links"] == SELECTED_LINKS
    assert contract["chain_joints"][0] == "base_to_arm_joint"
    assert contract["chain_joints"][-1] == "joint_tcp"
    assert contract["support_joints"] == ["world_joint", "base_to_arm_joint", "joint_eef", "gripper_fix", "joint_tcp"]
    assert contract["simulator_control"] == {
        "joint": "drive_joint",
        "state_interfaces": ["position", "velocity", "effort"],
        "command_interfaces": [],
    }
    assert contract["kinematics"]["xarm7"]["kinematics_solver"] == "kdl_kinematics_plugin/KDLKinematicsPlugin"
    assert contract["kinematics"]["xarm7"]["base_link"] == "base_link"
    assert contract["kinematics"]["xarm7"]["tip_link"] == "link_tcp"
    assert contract["kinematics"]["xarm7"]["kinematics_solver_attempts"] == 5


def test_contract_fingerprint_is_deterministic_canonical_sha256() -> None:
    contract = build_contract()
    expected = sha256(json.dumps(contract, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()
    assert contract_fingerprint(contract) == expected
    assert contract_fingerprint(contract) == contract_fingerprint(contract)
    assert len(contract_fingerprint(contract)) == 64


def test_unrelated_simulator_link_keeps_fingerprint() -> None:
    base = build_contract()
    sim = add_topic_control_block(URDF_CORE).replace(
        "</robot>", '<link name="unrelated_camera"><collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="1 1 1"/></geometry></collision></link></robot>'
    )
    changed = build_contract(sim_xml=sim)
    assert changed["selected_links"] == SELECTED_LINKS
    assert "unrelated_camera" not in changed["collision_geometry"]
    assert contract_fingerprint(changed) == contract_fingerprint(base)


def test_mount_change_raises_semantic_mismatch() -> None:
    sim = add_topic_control_block(URDF_CORE).replace(
        'name="world_joint" type="fixed"><parent link="world"/><child link="base_link"/><origin xyz="0 0 0"',
        'name="world_joint" type="fixed"><parent link="not_world"/><child link="base_link"/><origin xyz="0.1 0 0"',
    )
    with pytest.raises(ModelContractError) as error:
        build_contract(sim_xml=sim)
    assert error.value.code == "semantic_mismatch"


def test_group_order_mismatch_raises_semantic_mismatch() -> None:
    srdf = SRDF_XML.replace(
        '<link name="left_finger"/>\n    <link name="left_inner_knuckle"/>',
        '<link name="left_inner_knuckle"/>\n    <link name="left_finger"/>',
    )
    with pytest.raises(ModelContractError) as error:
        build_contract(srdf_xml=srdf)
    assert error.value.code == "semantic_mismatch"


def test_end_effector_parent_mismatch_raises() -> None:
    srdf = SRDF_XML.replace('parent_link="link_tcp"', 'parent_link="right_finger"')
    with pytest.raises(ModelContractError) as error:
        build_contract(srdf_xml=srdf)
    assert error.value.code == "semantic_mismatch"


def test_collision_change_raises_semantic_mismatch() -> None:
    plan = URDF_CORE.replace('name="link1"><collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="0.10 0.10 0.10"', 'name="link1"><collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="0.11 0.10 0.10"')
    with pytest.raises(ModelContractError) as error:
        build_contract(plan_xml=plan)
    assert error.value.code == "semantic_mismatch"


def test_joint_semantics_change_raises() -> None:
    plan = URDF_CORE.replace('name="joint3" type="revolute"', 'name="joint3" type="continuous"')
    with pytest.raises(ModelContractError) as error:
        build_contract(plan_xml=plan)
    assert error.value.code == "semantic_mismatch"


def test_kinematics_solver_change_changes_fingerprint() -> None:
    base = build_contract()
    kin = dict(parse_kinematics())
    changed = dict(kin["xarm7"])
    changed["kinematics_solver"] = "changed_plugin/ChangedPlugin"
    kin["xarm7"] = changed
    updated = build_contract(kinematics=kin)
    assert contract_fingerprint(updated) != contract_fingerprint(base)


def test_limits_value_change_changes_fingerprint() -> None:
    base = build_contract()
    limits = dict(parse_limits())
    entry = dict(limits["joint1"])
    entry["max_velocity"] = 3.5
    limits["joint1"] = entry
    updated = canonical_contract(
        add_topic_control_block(URDF_CORE), URDF_CORE, SRDF_XML, limits, parse_kinematics(), prefix="", mount=MOUNT
    )
    assert contract_fingerprint(updated) != contract_fingerprint(base)


def test_missing_limit_raises_invalid_limits() -> None:
    limits = dict(parse_limits())
    entry = dict(limits["drive_joint"])
    del entry["max_velocity"]
    limits["drive_joint"] = entry
    with pytest.raises(ModelContractError) as error:
        canonical_contract(
            add_topic_control_block(URDF_CORE), URDF_CORE, SRDF_XML, limits, parse_kinematics(), prefix="", mount=MOUNT
        )
    assert error.value.code == "invalid_limits"


def test_validate_bundle_manifest_accepts_valid() -> None:
    validate_bundle_manifest(valid_manifest())


def test_validate_bundle_manifest_rejects_missing_fields() -> None:
    manifest = valid_manifest()
    del manifest["structural_fingerprint"]
    with pytest.raises(ModelContractError) as error:
        validate_bundle_manifest(manifest)
    assert error.value.code == "invalid_manifest"


def test_validate_bundle_manifest_rejects_bad_fingerprint() -> None:
    manifest = valid_manifest()
    manifest["structural_fingerprint"] = "f" * 64
    with pytest.raises(ModelContractError) as error:
        validate_bundle_manifest(manifest)
    assert error.value.code == "structural_mismatch"


def test_validate_bundle_manifest_rejects_wrong_schema() -> None:
    manifest = valid_manifest()
    manifest["schema_version"] = 2
    with pytest.raises(ModelContractError) as error:
        validate_bundle_manifest(manifest)
    assert error.value.code == "schema_version"


def test_validate_bundle_manifest_rejects_relative_artifact_path() -> None:
    manifest = valid_manifest()
    manifest["artifacts"]["planning_urdf"]["path"] = "relative/path.urdf"
    with pytest.raises(ModelContractError) as error:
        validate_bundle_manifest(manifest)
    assert error.value.code == "invalid_manifest"
