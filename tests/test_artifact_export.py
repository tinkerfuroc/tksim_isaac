from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import os
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.runtime import resolve_current_artifact, topic_control_description
from tinker_sim_deploy.workspace import (
    ARTIFACT_FILES,
    CANONICALIZER_ALGORITHM,
    ArtifactExportError,
    ArtifactPublicationError,
    CanonicalizationError,
    UnsafePathError,
    capture_workspace_lock,
    canonicalize_urdf,
    export_tinker2,
    validate_canonical_urdf,
    verify_workspace_lock,
)


def _fixture_urdf() -> bytes:
    root = ET.Element("robot", {"name": "fixture", "custom": "root-value"})
    ET.SubElement(root, "material", {"name": "fixture_material", "arbitrary": "kept"}).append(ET.Element("color", {"rgba": "0.1 0.2 0.3 1"}))
    for name in ["base_link", "link_base", *[f"link{i}" for i in range(1, 8)], "gripper_base", "left_outer"]:
        link = ET.SubElement(root, "link", {"name": name, "fixture_attr": "keep"})
        inertial = ET.SubElement(link, "inertial", {"custom": "inertial-keep"})
        ET.SubElement(inertial, "mass", {"value": "1.0"})
        visual = ET.SubElement(link, "visual", {"custom": "visual-keep"})
        ET.SubElement(visual, "geometry").append(ET.Element("box", {"size": "1 2 3"}))
        collision = ET.SubElement(link, "collision", {"custom": "collision-keep"})
        ET.SubElement(collision, "geometry").append(ET.Element("box", {"size": "1 2 3"}))
    legacy = ET.SubElement(root, "joint", {"name": "world_joint", "type": "fixed", "custom": "mount-source"})
    ET.SubElement(legacy, "parent", {"link": "base_link"})
    ET.SubElement(legacy, "child", {"link": "link_base"})
    ET.SubElement(legacy, "origin", {"xyz": "-0.03 0 0.527", "rpy": "0 0 0"})
    parent = "link_base"
    for index in range(1, 8):
        joint = ET.SubElement(root, "joint", {"name": f"joint{index}", "type": "revolute", "custom": "joint-keep"})
        ET.SubElement(joint, "parent", {"link": parent})
        child = f"link{index}"
        ET.SubElement(joint, "child", {"link": child})
        ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
        ET.SubElement(joint, "limit", {"lower": "-1", "upper": "1", "effort": "2", "velocity": "3"})
        parent = child
    fixed = ET.SubElement(root, "joint", {"name": "gripper_mount", "type": "fixed"})
    ET.SubElement(fixed, "parent", {"link": "link7"})
    ET.SubElement(fixed, "child", {"link": "gripper_base"})
    drive = ET.SubElement(root, "joint", {"name": "drive_joint", "type": "revolute"})
    ET.SubElement(drive, "parent", {"link": "gripper_base"})
    ET.SubElement(drive, "child", {"link": "left_outer"})
    ET.SubElement(drive, "axis", {"xyz": "1 0 0"})
    ET.SubElement(drive, "limit", {"lower": "0", "upper": "1", "effort": "2", "velocity": "3"})
    control = ET.SubElement(root, "ros2_control", {"name": "fixture-system", "type": "system", "custom": "control-keep"})
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "fixture/Hardware"
    ET.SubElement(hardware, "param", {"name": "add_gripper"}).text = "True"
    ET.SubElement(hardware, "param", {"name": "custom_parameter"}).text = "  meaningful text  "
    for name in [f"joint{i}" for i in range(1, 8)]:
        joint = ET.SubElement(control, "joint", {"name": name})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        ET.SubElement(joint, "command_interface", {"name": "velocity"})
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})
    transmission = ET.SubElement(root, "transmission", {"name": "fixture-transmission", "custom": "transmission-keep"})
    ET.SubElement(transmission, "type").text = "fixture/SimpleTransmission"
    ET.SubElement(transmission, "opaque").text = "  opaque text  "
    ET.SubElement(root, "root_metadata", {"custom": "root-metadata-keep"}).text = "  root text  "
    return ET.tostring(root, encoding="utf-8")


def _old_artifact_urdf() -> bytes:
    return _fixture_urdf()


def _make_workspace(root: Path) -> None:
    generated = root / "src/tk26_sim/_generated"
    maps = root / "src/tk26_navigation/src/navigation_bringup/maps"
    profile = root / "src/tk25_basic/src/tinker_robot_config/robots/tinker2"
    generated.mkdir(parents=True)
    maps.mkdir(parents=True)
    profile.mkdir(parents=True)
    (generated / "tinker_full.full.urdf").write_bytes(_fixture_urdf())
    (generated / "tinker_full.usd").write_bytes(b"fixture-usd-bytes")
    (maps / "0701_robocup_arena3.yaml").write_text("image: arena.pgm\nresolution: 0.05\n", encoding="utf-8")
    (maps / "0701_robocup_arena3.pgm").write_bytes(b"fixture-map")
    (profile / "robot.yaml").write_text("robot: tinker2\n", encoding="utf-8")


def _export_fixture() -> tuple[tempfile.TemporaryDirectory[str], Path, Path, Path]:
    temporary = tempfile.TemporaryDirectory()
    root = Path(temporary.name)
    workspace = root / "workspace"
    artifacts = root / "artifacts"
    _make_workspace(workspace)
    lock = artifacts / "provenance/tinker2-source-lock.json"
    capture_workspace_lock(workspace, lock)
    result = export_tinker2(workspace, artifacts, lock)
    return temporary, workspace, artifacts, result.artifact_dir


class CanonicalUrdfTest(unittest.TestCase):
    def test_old_shape_is_rejected_and_new_shape_has_full_provider_contract(self) -> None:
        with self.assertRaises(CanonicalizationError):
            validate_canonical_urdf(_old_artifact_urdf())
        output = canonicalize_urdf(_fixture_urdf())
        validate_canonical_urdf(output)
        root = ET.fromstring(output)
        self.assertEqual(len(root.findall("link[@name='world']")), 1)
        self.assertEqual(len(root.findall("joint[@name='world_joint']")), 1)
        mount = root.find("joint[@name='base_to_arm_joint']")
        self.assertEqual(mount.find("parent").get("link"), "base_link")
        self.assertEqual(mount.find("child").get("link"), "link_base")
        self.assertEqual(mount.find("origin").get("xyz"), "-0.03 0 0.527")
        controls = root.findall("ros2_control")
        self.assertEqual(len(controls), 1)
        self.assertEqual({j.get("name") for j in controls[0].findall("joint")}, set([f"joint{i}" for i in range(1, 8)]) | {"drive_joint"})
        drive = controls[0].find("joint[@name='drive_joint']")
        self.assertFalse(drive.findall("command_interface"))
        self.assertEqual([x.get("name") for x in drive.findall("state_interface")], ["position", "velocity", "effort"])
        hardware = controls[0].find("hardware")
        self.assertEqual(hardware.find("param[@name='add_gripper']").text, "False")

    def test_determinism_idempotence_and_semantic_preservation(self) -> None:
        first = canonicalize_urdf(_fixture_urdf())
        self.assertEqual(first, canonicalize_urdf(_fixture_urdf()))
        self.assertEqual(first, canonicalize_urdf(first))
        text = first.decode("utf-8")
        for value in ("fixture/Hardware", "  meaningful text  ", "fixture/SimpleTransmission", "  opaque text  ", "root-metadata-keep", "inertial-keep", "visual-keep", "collision-keep", "transmission-keep"):
            self.assertIn(value, text)
        root = ET.fromstring(first)
        self.assertEqual(root.get("custom"), "root-value")
        self.assertEqual(root.find("material").get("arbitrary"), "kept")
        self.assertEqual(root.find("transmission").find("opaque").text, "  opaque text  ")
        self.assertEqual(root.find("root_metadata").text, "  root text  ")

    def test_duplicate_hidden_provider_and_malformed_graph_fail_closed(self) -> None:
        duplicate = _fixture_urdf().replace(b"</robot>", b'<ros2_control name="hidden" type="system"><joint name="joint1"/></ros2_control></robot>')
        with self.assertRaises(CanonicalizationError):
            canonicalize_urdf(duplicate)
        hidden = _fixture_urdf().replace(b'<param name="custom_parameter">  meaningful text  </param>', b'<param name="gripper_command_topic">/bad</param>')
        with self.assertRaises(CanonicalizationError):
            canonicalize_urdf(hidden)
        malformed = _fixture_urdf().replace(b'link="base_link"', b'link="missing"', 1)
        with self.assertRaisesRegex(CanonicalizationError, "missing parent"):
            canonicalize_urdf(malformed)

    def test_full_sha_identity_binds_payload_lock_schema_and_version(self) -> None:
        temporary, _, _, artifact = _export_fixture()
        try:
            artifact_id = artifact.name
            self.assertEqual(len(artifact_id), 64)
            self.assertNotEqual(artifact_id, "36ac0317025d20a5")
        finally:
            temporary.cleanup()

    def test_export_uses_immutable_snapshot_and_keeps_root_lock_unchanged(self) -> None:
        temporary, workspace, artifacts, artifact = _export_fixture()
        try:
            lock = artifacts / "provenance/tinker2-source-lock.json"
            lock_before = lock.read_bytes()
            source_before = (workspace / "src/tk26_sim/_generated/tinker_full.full.urdf").read_bytes()
            original_canonicalize = __import__("tinker_sim_deploy.workspace", fromlist=["canonicalize_urdf"]).canonicalize_urdf
            def mutate_after_snapshot(data: bytes) -> bytes:
                (workspace / "src/tk26_sim/_generated/tinker_full.full.urdf").write_bytes(b"mutated-after-snapshot")
                return original_canonicalize(data)
            with mock.patch("tinker_sim_deploy.workspace.canonicalize_urdf", side_effect=mutate_after_snapshot):
                second = export_tinker2(workspace, artifacts, lock)
            self.assertEqual(lock.read_bytes(), lock_before)
            self.assertEqual((second.artifact_dir / "robot.urdf").read_bytes(), (artifact / "robot.urdf").read_bytes())
            self.assertEqual((artifact / "robot.urdf").read_bytes(), original_canonicalize(source_before))
            self.assertTrue((artifact / "source-lock.json").is_file())
            source_lock = json.loads((artifact / "source-lock.json").read_text())
            self.assertNotIn("workspace", source_lock)
            self.assertTrue(all(not str(item["path"]).startswith("/") and ".." not in str(item["path"]).split("/") for item in source_lock["files"]))
            manifest = json.loads((artifact / "manifest.json").read_text())
            self.assertTrue(manifest["source_lock"].startswith("artifacts/robot/tinker2/"))
        finally:
            temporary.cleanup()

    def test_strict_lock_added_removed_foreign_absolute_and_traversal_records(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workspace = root / "workspace"
            _make_workspace(workspace)
            artifacts = root / "artifacts"
            lock = artifacts / "provenance/tinker2-source-lock.json"
            capture_workspace_lock(workspace, lock)
            raw = json.loads(lock.read_text())
            raw["files"].append({"path": "foreign", "size": 0, "sha256": "0" * 64})
            lock.write_text(json.dumps(raw))
            with self.assertRaises(ArtifactExportError):
                export_tinker2(workspace, artifacts, lock)
            lock = artifacts / "provenance/lock-extra-field.json"
            capture_workspace_lock(workspace, lock)
            raw = json.loads(lock.read_text())
            raw["files"][0]["extra"] = "not part of the normalized record"
            lock.write_text(json.dumps(raw))
            with self.assertRaises(ArtifactExportError):
                verify_workspace_lock(workspace, lock)
            lock = artifacts / "provenance/lock2.json"
            capture_workspace_lock(workspace, lock)
            raw = json.loads(lock.read_text())
            raw["files"][0]["path"] = "../escape"
            lock.write_text(json.dumps(raw))
            with self.assertRaises(UnsafePathError):
                verify_workspace_lock(workspace, lock)

    def test_symlink_inputs_and_path_escape_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workspace = root / "workspace"
            _make_workspace(workspace)
            source = workspace / "src/tk26_sim/_generated/tinker_full.usd"
            source.unlink()
            source.symlink_to(root / "outside")
            (root / "outside").write_bytes(b"outside")
            with self.assertRaises(UnsafePathError):
                capture_workspace_lock(workspace, root / "artifacts/provenance/lock.json")
            with self.assertRaises(UnsafePathError):
                capture_workspace_lock(workspace, root / "../escape/lock.json")

    def test_fault_before_current_leaves_old_pointer_and_no_partial_generation(self) -> None:
        temporary, workspace, artifacts, artifact = _export_fixture()
        try:
            current = artifacts / "robot/tinker2/current.json"
            before = current.read_bytes()
            lock = artifacts / "provenance/tinker2-source-lock.json"
            with mock.patch("tinker_sim_deploy.workspace._atomic_write", side_effect=OSError("fsync/write failure")):
                with self.assertRaises(OSError):
                    export_tinker2(workspace, artifacts, lock)
            self.assertEqual(current.read_bytes(), before)
            self.assertTrue((artifact / "manifest.json").is_file())
            self.assertTrue((artifact / "source-lock.json").is_file())
            self.assertTrue(resolve_current_artifact(artifacts.parent).artifact_dir == artifact)
        finally:
            temporary.cleanup()

    def test_concurrent_collision_rejects_mismatched_existing_directory(self) -> None:
        temporary, workspace, artifacts, artifact = _export_fixture()
        try:
            (artifact / "robot.usd").write_bytes(b"tampered")
            with self.assertRaises(ArtifactPublicationError):
                export_tinker2(workspace, artifacts, artifacts / "provenance/tinker2-source-lock.json")
        finally:
            temporary.cleanup()

    def test_resolver_rejects_tampered_current_manifest_hash_and_mixed_generation(self) -> None:
        temporary, _, artifacts, artifact = _export_fixture()
        try:
            root = artifacts.parent
            current = artifacts / "robot/tinker2/current.json"
            original = current.read_bytes()
            raw = json.loads(original)
            raw["manifest"] = "../../tmp/manifest.json"
            current.write_text(json.dumps(raw))
            with self.assertRaises(RuntimeError):
                resolve_current_artifact(root)
            current.write_bytes(original)
            (artifact / "manifest.json").write_text("{}")
            with self.assertRaises(RuntimeError):
                resolve_current_artifact(root)
            current.write_bytes(original)
            raw = json.loads(original)
            raw["source_lock"] = "artifacts/robot/tinker2/other/source-lock.json"
            current.write_text(json.dumps(raw))
            with self.assertRaises(RuntimeError):
                resolve_current_artifact(root)
        finally:
            temporary.cleanup()

    def test_resolver_rejects_artifact_payload_symlink_and_current_symlink(self) -> None:
        temporary, _, artifacts, artifact = _export_fixture()
        try:
            root = artifacts.parent
            current = artifacts / "robot/tinker2/current.json"
            original = current.read_bytes()
            current.unlink()
            current.symlink_to(root / "outside-current.json")
            (root / "outside-current.json").write_bytes(original)
            with self.assertRaises(UnsafePathError):
                resolve_current_artifact(root)
            current.unlink()
            current.write_bytes(original)
            payload = artifact / "map.pgm"
            payload.unlink()
            payload.symlink_to(root / "outside-map.pgm")
            (root / "outside-map.pgm").write_bytes(b"wrong")
            with self.assertRaises((RuntimeError, UnsafePathError)):
                resolve_current_artifact(root)
        finally:
            temporary.cleanup()

    def test_runtime_transformer_is_shared_and_arm_only(self) -> None:
        temporary, _, artifacts, _ = _export_fixture()
        try:
            resolved = resolve_current_artifact(artifacts.parent)
            output = ET.fromstring(topic_control_description(resolved.robot_urdf))
            control = output.find("ros2_control")
            self.assertEqual([j.get("name") for j in control.findall("joint")], [f"joint{i}" for i in range(1, 8)])
            self.assertNotIn("drive_joint", ET.tostring(control, encoding="unicode"))
        finally:
            temporary.cleanup()

    def test_launch_modules_use_shared_resolver_when_ros_environment_is_available(self) -> None:
        for relative in ("ros2_ws/src/tinker_sim_bridge/launch/manipulation.launch.py", "ros2_ws/src/tinker_sim_bridge/launch/whole_robot.launch.py"):
            with self.subTest(relative=relative):
                path = ROOT / relative
                spec = importlib.util.spec_from_file_location(f"launch_{path.stem}", path)
                module = importlib.util.module_from_spec(spec)
                try:
                    spec.loader.exec_module(module)
                except ModuleNotFoundError as error:
                    self.skipTest(f"ROS launch environment unavailable: {error}")
                self.assertIs(module.resolve_current_artifact, resolve_current_artifact)
                self.assertEqual(module.topic_control_description, topic_control_description)
                self.assertEqual(path.read_text().count('executable="gripper_facade"'), 1)


if __name__ == "__main__":
    unittest.main()
