"""Committed arm+gripper joint-limit synthesis tests.

The canonical model-bundle schema requires all eight selected joints in
``joint_limits``, while the production arm file defines only ``joint1``..``joint7``.
``model_limits`` deterministically merges the arm and gripper source YAML files
into the canonical eight-joint artifact that is itself hashed into the manifest.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

import pytest
import yaml

from tinker_sim_bridge.model_contract import ARM_JOINTS, ORDERED_JOINTS, ModelContractError
from tinker_sim_bridge.model_limits import main as limits_main
from tinker_sim_bridge.model_limits import synthesize_joint_limits, write_synthesized

ARM_LIMITS_YAML = "joint_limits:\n" + "".join(
    "  {name}:\n    has_velocity_limits: true\n    max_velocity: 2.14\n    has_acceleration_limits: true\n    max_acceleration: 10.0\n    min_position: -1.0\n    max_position: 1.0\n".format(name=name)
    for name in ARM_JOINTS
)

GRIPPER_LIMITS_YAML = """joint_limits:
  drive_joint:
    has_velocity_limits: true
    max_velocity: 3.14
    has_acceleration_limits: true
    max_acceleration: 10.0
"""


def _write_sources(tmp_path: Path) -> tuple[Path, Path]:
    arm = tmp_path / "arm_limits.yaml"
    gripper = tmp_path / "gripper_limits.yaml"
    arm.write_text(ARM_LIMITS_YAML, encoding="utf-8")
    gripper.write_text(GRIPPER_LIMITS_YAML, encoding="utf-8")
    return arm, gripper


def test_synthesize_merges_canonical_eight_joints(tmp_path: Path) -> None:
    arm, gripper = _write_sources(tmp_path)
    merged = synthesize_joint_limits(arm, gripper)
    assert set(merged) == {"joint_limits"}
    inner = merged["joint_limits"]
    assert set(inner.keys()) == set(ORDERED_JOINTS)
    assert len(inner) == 8
    assert inner["drive_joint"]["max_velocity"] == 3.14
    assert inner["joint1"]["max_velocity"] == 2.14


def test_synthesize_missing_drive_joint_rejected(tmp_path: Path) -> None:
    arm, gripper = _write_sources(tmp_path)
    gripper.write_text("joint_limits:\n  left_finger_joint:\n    max_velocity: 1.0\n", encoding="utf-8")
    with pytest.raises(ModelContractError) as error:
        synthesize_joint_limits(arm, gripper)
    assert error.value.code == "invalid_limits"


def test_synthesize_missing_arm_joint_rejected(tmp_path: Path) -> None:
    arm, gripper = _write_sources(tmp_path)
    arm.write_text("joint_limits:\n  joint1:\n    max_velocity: 1.0\n", encoding="utf-8")
    with pytest.raises(ModelContractError) as error:
        synthesize_joint_limits(arm, gripper)
    assert error.value.code == "invalid_limits"


def test_write_synthesized_is_deterministic_and_atomic(tmp_path: Path) -> None:
    arm, gripper = _write_sources(tmp_path)
    output = tmp_path / "out" / "joint_limits.yaml"
    first = write_synthesized(arm, gripper, output)
    second = write_synthesized(arm, gripper, tmp_path / "out2" / "joint_limits.yaml")
    assert first.read_bytes() == second.read_bytes()
    leftovers = [path for path in output.parent.iterdir() if path.name.startswith(".joint_limits.yaml.")]
    assert leftovers == []
    parsed = yaml.safe_load(first.read_text(encoding="utf-8"))
    assert set(parsed["joint_limits"].keys()) == set(ORDERED_JOINTS)
    assert len(parsed["joint_limits"]) == 8


def test_limits_main_end_to_end(tmp_path: Path, capsys) -> None:
    arm, gripper = _write_sources(tmp_path)
    output = tmp_path / "model_limits.yaml"
    rc = limits_main(["--arm-joint-limits", str(arm), "--gripper-joint-limits", str(gripper), "--output", str(output)])
    assert rc == 0
    assert output.is_file()
    parsed = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert set(parsed["joint_limits"].keys()) == set(ORDERED_JOINTS)
    assert len(parsed["joint_limits"]) == 8
    assert "wrote synthesized joint limits" in capsys.readouterr().out


def test_setup_registers_model_limits_entrypoint() -> None:
    setup = ROOT / "ros2_ws/src/tinker_sim_bridge/setup.py"
    text = setup.read_text(encoding="utf-8")
    assert "model_limits = tinker_sim_bridge.model_limits:main" in text


# ---------------------------------------------------------------------------
# Real-source regression (plan acceptance).  Runs in this repository, skips
# only when the external tk25_ws cross-repo sources are absent.
# ---------------------------------------------------------------------------

def _tk25_root() -> Path | None:
    env = __import__("os").environ.get("TINKER_WS")
    if env and Path(env).is_dir():
        return Path(env)
    candidate = Path("/home/tinker/tk25_ws")
    return candidate if candidate.is_dir() else None


def test_real_source_limits_synthesis_producer_preflight_consumer(tmp_path: Path) -> None:
    tk25 = _tk25_root()
    if tk25 is None:
        pytest.skip("missing cross-repo tk25_ws source tree (set TINKER_WS)")
    arm_source = tk25 / "src/tk25_manipulation/src/xarm_ros2/xarm_moveit_config/config/xarm7/joint_limits.yaml"
    gripper_source = tk25 / "src/tk25_manipulation/src/xarm_ros2/xarm_moveit_config/config/xarm_gripper/joint_limits.yaml"
    planning_urdf = tk25 / "src/tk25_basic/src/cumotion_description/config/xarm7.urdf"
    planning_srdf = tk25 / "src/tk25_basic/src/cumotion_description/config/xarm7.srdf"
    kinematics = tk25 / "src/tk25_manipulation/src/xarm_ros2/xarm_moveit_config/config/xarm7/kinematics.yaml"
    required = {"arm limits": arm_source, "gripper limits": gripper_source, "planning urdf": planning_urdf, "planning srdf": planning_srdf, "kinematics": kinematics}
    missing = [name for name, path in required.items() if not path.is_file()]
    if missing:
        pytest.skip("missing cross-repo source files: {}".format(", ".join(missing)))

    consumer_lib = tk25 / "src/tk25_manipulation/src/xarm_ros2/xarm_moveit_config/launch/lib"
    if not (consumer_lib / "tinker_model_bundle.py").is_file():
        pytest.skip("missing production model-bundle consumer source")
    sys.path.insert(0, str(consumer_lib))
    from tinker_model_bundle import load_model_bundle

    sim_urdf = None
    current = ROOT / "artifacts" / "robot" / "tinker2" / "current.json"
    if current.is_file():
        from tinker_sim_bridge.model_bundle import resolve_simulator_full_urdf
        sim_urdf = resolve_simulator_full_urdf(ROOT)
    if sim_urdf is None or not sim_urdf.is_file():
        pytest.skip("simulator artifact tree is not present in this checkout")

    from tinker_sim_bridge.model_bundle import build_manifest, write_manifest
    from tinker_sim_bridge.model_preflight import preflight_manifest

    # Deterministic synthesis from the committed mechanism.
    merged = tmp_path / "joint_limits.yaml"
    write_synthesized(arm_source, gripper_source, merged)
    merged_again = tmp_path / "joint_limits_again.yaml"
    write_synthesized(arm_source, gripper_source, merged_again)
    assert merged.read_bytes() == merged_again.read_bytes()
    parsed = yaml.safe_load(merged.read_text(encoding="utf-8"))
    assert set(parsed["joint_limits"].keys()) == set(ORDERED_JOINTS)
    assert len(parsed["joint_limits"]) == 8

    manifest = build_manifest(
        simulator_full_urdf=sim_urdf,
        planning_urdf=planning_urdf,
        planning_srdf=planning_srdf,
        joint_limits=merged,
        kinematics=kinematics,
        prefix="",
        mount={"parent": "world", "child": "base_link", "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
    )
    manifest_path = tmp_path / "model-bundle.json"
    write_manifest(manifest, manifest_path)

    result = preflight_manifest(manifest_path, timeout=60.0, project_root=ROOT)
    assert result["status"] == "ready", result

    loaded = load_model_bundle(manifest_path)
    assert loaded.contract["touch_links"] == list(__import__("tinker_sim_bridge.model_contract", fromlist=["TOUCH_LINKS"]).TOUCH_LINKS)
    assert loaded.contract["gripper_joint"] == "drive_joint"
    assert loaded.structural_fingerprint == manifest["structural_fingerprint"]
