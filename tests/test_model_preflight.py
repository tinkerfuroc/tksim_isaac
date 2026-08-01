"""Bounded preflight tests for the canonical model-bundle manifest."""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

import pytest

from tinker_sim_bridge.model_bundle import build_manifest
from tinker_sim_bridge.model_preflight import main as preflight_main
from tinker_sim_bridge.model_preflight import preflight_manifest

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
""" + "".join(_link(name) for name in (
    "left_outer_knuckle", "left_finger", "left_inner_knuckle",
    "right_inner_knuckle", "right_outer_knuckle", "right_finger",
)) + _link("link_tcp") + _joint("drive_joint", "xarm_gripper_base_link", "left_outer_knuckle", "1 0 0", "0.0", "0.8") + """  <joint name="joint_tcp" type="fixed"><parent link="xarm_gripper_base_link"/><child link="link_tcp"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
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
    for name in list("joint{}".format(i) for i in range(1, 8)) + ["drive_joint"]
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


def write_artifacts(tmp_path: Path) -> dict[str, Path]:
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


def write_ready_manifest(tmp_path: Path) -> Path:
    artifacts = write_artifacts(tmp_path)
    manifest = build_manifest(
        simulator_full_urdf=artifacts["simulator_full_urdf"],
        planning_urdf=artifacts["planning_urdf"],
        planning_srdf=artifacts["planning_srdf"],
        joint_limits=artifacts["joint_limits"],
        kinematics=artifacts["kinematics"],
        prefix="",
        mount=MOUNT,
    )
    # Provide a valid synthetic authoritative current selector so a fully-ready
    # preflight can prove identity with the selected simulator artifact
    # regardless of the repository cwd: mirror the canonical URDF into a legacy
    # artifact tree and point the manifest at that tree's robot.urdf.
    from model_fixtures import write_legacy_current
    sim_bytes = artifacts["simulator_full_urdf"].read_bytes()
    selected_urdf = write_legacy_current(tmp_path, "abcdef0123456789", sim_bytes)
    manifest["artifacts"]["simulator_full_urdf"]["path"] = str(selected_urdf)
    manifest["artifacts"]["simulator_full_urdf"]["sha256"] = _sha(selected_urdf)
    output = tmp_path / "model-bundle.json"
    from tinker_sim_bridge.model_bundle import write_manifest as _write
    _write(manifest, output)
    return output


def test_ready_preflight_reports_all_checks(tmp_path: Path) -> None:
    manifest_path = write_ready_manifest(tmp_path)
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=None)
    assert result["status"] == "ready"
    assert result["ready"] is True
    assert result["structural_fingerprint"] == json.loads(manifest_path.read_text(encoding="utf-8"))["structural_fingerprint"]
    names = [check["name"] for check in result["checks"]]
    for expected in ("manifest_schema", "manifest_structure", "artifact_path_simulator_full_urdf", "artifact_hash_simulator_full_urdf", "contract", "fingerprint", "finite_json", "artifact_identity"):
        assert expected in names
    assert all(check["ok"] for check in result["checks"])


def test_main_writes_report_only_on_ready(tmp_path: Path) -> None:
    manifest_path = write_ready_manifest(tmp_path)
    report = tmp_path / "report.json"
    rc = preflight_main(
        ["--model-bundle-manifest", str(manifest_path), "--report", str(report), "--timeout", "60"]
    )
    assert rc == 0
    assert report.is_file()
    report_data = json.loads(report.read_text(encoding="utf-8"))
    assert report_data["status"] == "ready"
    assert report_data["ready"] is True


def test_hash_mismatch_is_not_ready_and_no_report(tmp_path: Path) -> None:
    manifest_path = write_ready_manifest(tmp_path)
    report = tmp_path / "report.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    artifact = Path(manifest["artifacts"]["planning_urdf"]["path"])
    artifact.write_text(artifact.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    rc = preflight_main(
        ["--model-bundle-manifest", str(manifest_path), "--report", str(report), "--timeout", "60"]
    )
    assert rc == 1
    assert not report.exists()
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=None)
    assert result["status"] == "mismatch"
    assert result["ready"] is False


def test_semantic_mismatch_with_fresh_hash_is_detected(tmp_path: Path) -> None:
    manifest_path = write_ready_manifest(tmp_path)
    report = tmp_path / "report.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    artifact = Path(manifest["artifacts"]["planning_urdf"]["path"])
    mutated = artifact.read_text(encoding="utf-8").replace('name="joint3" type="revolute"', 'name="joint3" type="continuous"')
    artifact.write_text(mutated, encoding="utf-8")
    from tinker_sim_bridge.model_bundle import sha256_file as _sha
    manifest["artifacts"]["planning_urdf"]["sha256"] = _sha(artifact)
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    rc = preflight_main(
        ["--model-bundle-manifest", str(manifest_path), "--report", str(report), "--timeout", "60"]
    )
    assert rc == 1
    assert not report.exists()
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=None)
    assert result["status"] == "mismatch"
    contract_check = [check for check in result["checks"] if check["name"] == "contract"]
    assert contract_check and contract_check[0]["ok"] is False


def test_malformed_manifest_is_invalid(tmp_path: Path) -> None:
    manifest_path = tmp_path / "broken.json"
    manifest_path.write_text("{not valid json", encoding="utf-8")
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=None)
    assert result["status"] == "invalid"
    assert result["ready"] is False


def test_timeout_is_bounded_and_not_ready(tmp_path: Path) -> None:
    manifest_path = write_ready_manifest(tmp_path)
    result = preflight_manifest(manifest_path, timeout=1e-12, project_root=None)
    assert result["status"] == "timeout"
    assert result["ready"] is False


def test_artifact_identity_follows_current_json(tmp_path: Path) -> None:
    from model_fixtures import write_legacy_current

    manifest_path = write_ready_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    sim_path = Path(manifest["artifacts"]["simulator_full_urdf"]["path"])
    # Mirror the artifact tree: selected generation contains exactly this URDF.
    gen = "abcdef0123456789"
    urdf = write_legacy_current(tmp_path, gen, sim_path.read_bytes())
    # Point the manifest at the mirrored generation path so identity matches.
    manifest["artifacts"]["simulator_full_urdf"]["path"] = str(urdf)
    manifest["artifacts"]["simulator_full_urdf"]["sha256"] = _sha(urdf)
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=tmp_path)
    identity = [check for check in result["checks"] if check["name"] == "artifact_identity"]
    assert identity and identity[0]["ok"] is True
    assert result["status"] == "ready"


def test_artifact_identity_mismatch_is_not_ready(tmp_path: Path) -> None:
    from model_fixtures import write_legacy_current

    manifest_path = write_ready_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    sim_bytes = Path(manifest["artifacts"]["simulator_full_urdf"]["path"]).read_bytes()
    # The manifest references a generation that is NOT what current.json selects
    # and whose bytes differ from the selected generation (stale outside-tree
    # copy), so byte-identity must reject it.
    selected_gen = "aaaa1111aaaa1111"
    other_gen = "bbbb2222bbbb2222"
    other_urdf = write_legacy_current(tmp_path, other_gen, sim_bytes + b"\n")
    write_legacy_current(tmp_path, selected_gen, sim_bytes)
    # Re-point the selector at the selected generation (last helper call won).
    (tmp_path / "artifacts" / "robot" / "tinker2" / "current.json").write_text(
        json.dumps(
            {"artifact_id": selected_gen, "manifest": "artifacts/robot/tinker2/{}/manifest.json".format(selected_gen)}
        ),
        encoding="utf-8",
    )
    manifest["artifacts"]["simulator_full_urdf"]["path"] = str(other_urdf)
    manifest["artifacts"]["simulator_full_urdf"]["sha256"] = _sha(other_urdf)
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=tmp_path)
    assert result["status"] == "mismatch"
    identity = [check for check in result["checks"] if check["name"] == "artifact_identity"]
    assert identity and identity[0]["ok"] is False


def test_reversed_selected_links_not_ready_and_consumer_rejects(tmp_path: Path) -> None:
    manifest_path = write_ready_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    links = manifest["normalization"]["selected_links"]
    manifest["normalization"]["selected_links"] = list(reversed(links))
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=None)
    assert result["status"] == "mismatch"
    contract_check = [check for check in result["checks"] if check["name"] == "contract"]
    assert contract_check and contract_check[0]["ok"] is False


def test_wrong_groups_value_not_ready(tmp_path: Path) -> None:
    manifest_path = write_ready_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["normalization"]["groups"] = {"arm": "xarm7", "gripper": "other_group"}
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=None)
    assert result["status"] in {"invalid", "mismatch"}
    assert result["ready"] is False


def test_identity_fails_closed_without_project_root(tmp_path: Path, monkeypatch) -> None:
    manifest_path = write_ready_manifest(tmp_path)
    # write_ready_manifest provides a synthetic authoritative selector; remove
    # it so no authoritative current.json can be resolved from any root.
    selector = tmp_path / "artifacts" / "robot" / "tinker2" / "current.json"
    selector.unlink()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    # Move the manifest's sim artifact inside an artifact tree with no selector.
    sim_src = Path(manifest["artifacts"]["simulator_full_urdf"]["path"])
    artifact_dir = tmp_path / "artifacts" / "robot" / "tinker2" / "deadbeefdeadbeef"
    artifact_dir.mkdir(parents=True, exist_ok=True)
    urdf = artifact_dir / "robot.urdf"
    urdf.write_bytes(sim_src.read_bytes())
    manifest["artifacts"]["simulator_full_urdf"]["path"] = str(urdf)
    manifest["artifacts"]["simulator_full_urdf"]["sha256"] = _sha(urdf)
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    # Simulate a non-repository cwd and no TINKER_SIM_ROOT: no authoritative
    # current.json can be resolved, so identity must fail closed, not be omitted.
    monkeypatch.delenv("TINKER_SIM_ROOT", raising=False)
    monkeypatch.chdir(tmp_path)
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=None)
    identity = [check for check in result["checks"] if check["name"] == "artifact_identity"]
    assert identity and identity[0]["ok"] is False
    assert result["status"] == "mismatch"
    assert result["ready"] is False


def test_identity_ready_from_manifest_derived_root(tmp_path: Path) -> None:
    from model_fixtures import write_legacy_current

    manifest_path = write_ready_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    sim_src = Path(manifest["artifacts"]["simulator_full_urdf"]["path"])
    # Put the manifest sim path inside a valid legacy artifact tree (the tree
    # carries its own current.json), and pass project_root=None so the root is
    # derived from the manifest path.
    urdf = write_legacy_current(tmp_path, "abcdef0123456789", sim_src.read_bytes())
    manifest["artifacts"]["simulator_full_urdf"]["path"] = str(urdf)
    manifest["artifacts"]["simulator_full_urdf"]["sha256"] = _sha(urdf)
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=None)
    identity = [check for check in result["checks"] if check["name"] == "artifact_identity"]
    assert identity and identity[0]["ok"] is True
    assert result["status"] == "ready"


def test_outside_tree_copied_current_bytes_ready(tmp_path: Path) -> None:
    manifest_path = write_ready_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = Path(manifest["artifacts"]["simulator_full_urdf"]["path"])
    # Copy the selected canonical URDF bytes OUTSIDE any artifact tree; the
    # bytes are identical to the authoritative selection, so identity holds.
    copied = tmp_path / "copied-simulator-full.urdf"
    copied.write_bytes(selected.read_bytes())
    manifest["artifacts"]["simulator_full_urdf"]["path"] = str(copied)
    manifest["artifacts"]["simulator_full_urdf"]["sha256"] = _sha(copied)
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=None)
    identity = [check for check in result["checks"] if check["name"] == "artifact_identity"]
    assert identity and identity[0]["ok"] is True
    assert result["status"] == "ready"
    assert result["ready"] is True


def test_outside_tree_stale_bytes_not_ready(tmp_path: Path) -> None:
    manifest_path = write_ready_manifest(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = Path(manifest["artifacts"]["simulator_full_urdf"]["path"])
    # A stale outside-tree copy whose bytes differ from the authoritative
    # selection must fail identity even though the manifest hashes the copy.
    stale = tmp_path / "stale-simulator-full.urdf"
    stale.write_bytes(selected.read_bytes() + b"\n")
    manifest["artifacts"]["simulator_full_urdf"]["path"] = str(stale)
    manifest["artifacts"]["simulator_full_urdf"]["sha256"] = _sha(stale)
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")
    result = preflight_manifest(manifest_path, timeout=60.0, project_root=None)
    identity = [check for check in result["checks"] if check["name"] == "artifact_identity"]
    assert identity and identity[0]["ok"] is False
    assert result["status"] == "mismatch"
    assert result["ready"] is False


def _sha(path: Path) -> str:
    from tinker_sim_bridge.model_bundle import sha256_file
    return sha256_file(path)
