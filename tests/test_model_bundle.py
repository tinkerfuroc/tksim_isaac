"""Producer tests for the canonical model-bundle manifest."""
from __future__ import annotations

import json
import sys
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
    sha256_file,
    sha256_json,
    validate_bundle_manifest,
)
from tinker_sim_bridge.model_bundle import (
    build_manifest,
    main as bundle_main,
    resolve_simulator_full_urdf,
    write_manifest,
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


def write_complete_test_artifacts(tmp_path: Path) -> dict[str, Path]:
    planning = tmp_path / "planning.urdf"
    simulator = tmp_path / "simulator_full.urdf"
    srdf = tmp_path / "planning.srdf"
    limits = tmp_path / "joint_limits.yaml"
    kinematics = tmp_path / "kinematics.yaml"
    planning.write_text(URDF_CORE, encoding="utf-8")
    simulator.write_text(add_topic_control_block(URDF_CORE), encoding="utf-8")
    srdf.write_text(SRDF_XML, encoding="utf-8")
    limits.write_text(LIMITS_YAML, encoding="utf-8")
    kinematics.write_text(KINEMATICS_YAML, encoding="utf-8")
    return {
        "simulator_full_urdf": simulator,
        "planning_urdf": planning,
        "planning_srdf": srdf,
        "joint_limits": limits,
        "kinematics": kinematics,
    }


def build_fixture_manifest(tmp_path: Path) -> dict[str, object]:
    artifacts = write_complete_test_artifacts(tmp_path)
    return build_manifest(
        simulator_full_urdf=artifacts["simulator_full_urdf"],
        planning_urdf=artifacts["planning_urdf"],
        planning_srdf=artifacts["planning_srdf"],
        joint_limits=artifacts["joint_limits"],
        kinematics=artifacts["kinematics"],
        prefix="",
        mount=MOUNT,
    )


def test_build_manifest_produces_complete_valid_manifest(tmp_path: Path) -> None:
    manifest = build_fixture_manifest(tmp_path)
    validate_bundle_manifest(manifest)
    assert manifest["schema_version"] == 1
    assert manifest["producer"] == {"name": "tinker_sim_bridge.model_bundle", "version": "1"}
    assert manifest["normalization"]["ordered_joints"] == list(ORDERED_JOINTS)
    assert manifest["normalization"]["groups"] == GROUPS
    assert manifest["normalization"]["selected_links"] == SELECTED_LINKS
    assert manifest["contract"]["touch_links"] == list(TOUCH_LINKS)
    for name, entry in manifest["artifacts"].items():
        path = Path(entry["path"])
        assert path.is_absolute() and path.is_file()
        assert entry["sha256"] == sha256_file(path)
        assert entry["sha256"] != "0" * 64
    assert manifest["structural_fingerprint"] == sha256_json(manifest["contract"])


def test_build_manifest_rejects_missing_drive_joint_limits(tmp_path: Path) -> None:
    artifacts = write_complete_test_artifacts(tmp_path)
    root = yaml.safe_load(artifacts["joint_limits"].read_text())
    del root["joint_limits"]["drive_joint"]
    artifacts["joint_limits"].write_text(yaml.safe_dump(root), encoding="utf-8")
    with pytest.raises(ModelContractError) as error:
        build_manifest(
            simulator_full_urdf=artifacts["simulator_full_urdf"],
            planning_urdf=artifacts["planning_urdf"],
            planning_srdf=artifacts["planning_srdf"],
            joint_limits=artifacts["joint_limits"],
            kinematics=artifacts["kinematics"],
            prefix="",
            mount=MOUNT,
        )
    assert error.value.code == "invalid_limits"


def test_build_manifest_rejects_missing_input_path(tmp_path: Path) -> None:
    artifacts = write_complete_test_artifacts(tmp_path)
    missing = tmp_path / "missing.urdf"
    with pytest.raises(ModelContractError) as error:
        build_manifest(
            simulator_full_urdf=artifacts["simulator_full_urdf"],
            planning_urdf=missing,
            planning_srdf=artifacts["planning_srdf"],
            joint_limits=artifacts["joint_limits"],
            kinematics=artifacts["kinematics"],
            prefix="",
            mount=MOUNT,
        )
    assert error.value.code == "artifact_path"


def test_write_manifest_is_atomic_and_deterministic(tmp_path: Path) -> None:
    manifest = build_fixture_manifest(tmp_path)
    output = tmp_path / "output" / "model-bundle.json"
    write_manifest(manifest, output)
    assert output.is_file()
    assert json.loads(output.read_text(encoding="utf-8"))["structural_fingerprint"] == manifest["structural_fingerprint"]
    leftovers = [path for path in output.parent.iterdir() if path.name.startswith(".model-bundle.json.")]
    assert leftovers == []


def test_bundle_main_end_to_end(tmp_path: Path, capsys) -> None:
    artifacts = write_complete_test_artifacts(tmp_path)
    output = tmp_path / "model-bundle.json"
    rc = bundle_main(
        [
            "--simulator-full-urdf", str(artifacts["simulator_full_urdf"]),
            "--planning-urdf", str(artifacts["planning_urdf"]),
            "--planning-srdf", str(artifacts["planning_srdf"]),
            "--joint-limits", str(artifacts["joint_limits"]),
            "--kinematics", str(artifacts["kinematics"]),
            "--prefix", "",
            "--mount-parent", "world",
            "--mount-child", "base_link",
            "--output", str(output),
        ]
    )
    assert rc == 0
    manifest = json.loads(output.read_text(encoding="utf-8"))
    validate_bundle_manifest(manifest)
    assert manifest["structural_fingerprint"] == sha256_json(manifest["contract"])
    captured = capsys.readouterr()
    assert "wrote model bundle manifest" in captured.out
    assert manifest["structural_fingerprint"] in captured.out


def test_resolve_simulator_full_urdf_follows_current_json(tmp_path: Path) -> None:
    from model_fixtures import write_legacy_current

    artifact_id = "abc123456789def0"
    urdf = write_legacy_current(tmp_path, artifact_id, b"<robot/>")
    resolved = resolve_simulator_full_urdf(tmp_path)
    assert resolved == urdf


def test_resolve_simulator_full_urdf_missing_current_json(tmp_path: Path) -> None:
    with pytest.raises(ModelContractError) as error:
        resolve_simulator_full_urdf(tmp_path)
    assert error.value.code == "artifact_current"


def test_setup_registers_model_entrypoints() -> None:
    setup = Path(__file__).resolve().parents[1] / "ros2_ws/src/tinker_sim_bridge/setup.py"
    text = setup.read_text(encoding="utf-8")
    assert "model_bundle = tinker_sim_bridge.model_bundle:main" in text
    assert "model_preflight = tinker_sim_bridge.model_preflight:main" in text
    assert "model_limits = tinker_sim_bridge.model_limits:main" in text


def test_build_wrapper_forces_job_cap() -> None:
    script = Path(__file__).resolve().parents[1] / "scripts/build-humble-overlay"
    text = script.read_text(encoding="utf-8")
    # The memory-safety bound must be FORCED, not default-preserving, so a
    # preset higher MAKEFLAGS cannot escape the -j2 -l2 cap.
    assert "export MAKEFLAGS=\"-j2 -l2\"" in text
    assert '${MAKEFLAGS:--j2 -l2}' not in text
    assert "--parallel-workers 2" in text
