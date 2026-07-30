from __future__ import annotations

import ast
import hashlib
import json
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.config import sha256_file
from tinker_sim_deploy.workspace import (
    CANONICALIZER_ALGORITHM,
    CanonicalizationError,
    artifact_identity,
    canonicalize_urdf,
    export_tinker2,
    validate_canonical_urdf,
)

SOURCE_URDF = Path("/home/tinker/tk25_ws/src/tk26_sim/_generated/tinker_full.full.urdf")
OLD_ARTIFACT = ROOT / "artifacts/robot/tinker2/311d7492c6ffffc2"


def _source_bytes() -> bytes:
    return SOURCE_URDF.read_bytes()


def _root(data: bytes) -> ET.Element:
    return ET.fromstring(data)


def _joint(root: ET.Element, name: str) -> ET.Element:
    matches = [joint for joint in root.findall("joint") if joint.get("name") == name]
    if len(matches) != 1:
        raise AssertionError(f"expected one {name}, found {len(matches)}")
    return matches[0]


def _control_joint(root: ET.Element, name: str) -> list[ET.Element]:
    return [
        joint
        for control in root.findall("ros2_control")
        for joint in control.findall("joint")
        if joint.get("name") == name
    ]


def _topic_transformer(path: Path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "_topic_control_description")
    arm_joints = next(node for node in tree.body if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "ARM_JOINTS" for target in node.targets))
    module = ast.Module(body=[arm_joints, function], type_ignores=[])
    namespace = {"ET": ET}
    exec(compile(module, str(path), "exec"), namespace)
    return namespace["_topic_control_description"]


class CanonicalUrdfTest(unittest.TestCase):
    def test_selected_old_artifact_is_rejected_by_canonical_invariants(self) -> None:
        with self.assertRaisesRegex(CanonicalizationError, "world|drive_joint"):
            validate_canonical_urdf((OLD_ARTIFACT / "robot.urdf").read_bytes())

    def test_canonicalization_has_exact_mount_and_state_only_drive_contract(self) -> None:
        output = canonicalize_urdf(_source_bytes())
        root = _root(output)

        self.assertEqual(len(root.findall("link[@name='world']")), 1)
        world_joint = _joint(root, "world_joint")
        self.assertEqual(world_joint.get("type"), "fixed")
        self.assertEqual(world_joint.find("parent").get("link"), "world")
        self.assertEqual(world_joint.find("child").get("link"), "base_link")
        self.assertEqual(world_joint.find("origin").get("xyz"), "0 0 0")
        self.assertEqual(world_joint.find("origin").get("rpy"), "0 0 0")

        mount = _joint(root, "base_to_arm_joint")
        self.assertEqual(mount.get("type"), "fixed")
        self.assertEqual(mount.find("parent").get("link"), "base_link")
        self.assertEqual(mount.find("child").get("link"), "link_base")
        self.assertEqual(mount.find("origin").get("xyz"), "-0.03 0 0.527")
        self.assertEqual(mount.find("origin").get("rpy"), "0 0 0")

        self.assertEqual(len(root.findall("joint[@name='drive_joint']")), 1)
        controls = _control_joint(root, "drive_joint")
        self.assertEqual(len(controls), 1)
        self.assertEqual([child.tag for child in controls[0]], ["state_interface"] * 3)
        self.assertEqual([child.get("name") for child in controls[0]], ["position", "velocity", "effort"])
        self.assertFalse(controls[0].findall("command_interface"))
        self.assertEqual([joint.get("name") for joint in root.findall("ros2_control/joint")[:7]], [f"joint{i}" for i in range(1, 8)])

    def test_canonicalization_is_deterministic_and_idempotent(self) -> None:
        first = canonicalize_urdf(_source_bytes())
        self.assertEqual(first, canonicalize_urdf(_source_bytes()))
        self.assertEqual(first, canonicalize_urdf(first))

    def test_unrelated_model_content_is_preserved(self) -> None:
        source = _root(_source_bytes())
        output = _root(canonicalize_urdf(_source_bytes()))
        source_links = {link.get("name") for link in source.findall("link")}
        output_links = {link.get("name") for link in output.findall("link")}
        self.assertEqual(output_links, source_links | {"world"})

        ignored_joints = {"world_joint", "base_to_arm_joint"}
        source_joints = {
            joint.get("name"): ET.canonicalize(ET.tostring(joint, encoding="unicode"), strip_text=True)
            for joint in source.findall("joint")
            if joint.get("name") not in ignored_joints
        }
        output_joints = {
            joint.get("name"): ET.canonicalize(ET.tostring(joint, encoding="unicode"), strip_text=True)
            for joint in output.findall("joint")
            if joint.get("name") not in ignored_joints
        }
        self.assertEqual(output_joints, source_joints)
        self.assertEqual(
            [ET.canonicalize(ET.tostring(joint, encoding="unicode"), strip_text=True) for joint in source.findall("ros2_control/joint") if joint.get("name") != "drive_joint"],
            [ET.canonicalize(ET.tostring(joint, encoding="unicode"), strip_text=True) for joint in output.findall("ros2_control/joint") if joint.get("name") != "drive_joint"],
        )

    def test_duplicate_special_definitions_are_rejected(self) -> None:
        mutations = {
            "world": b'<link name="world"/>',
            "world_joint": b'<joint name="world_joint" type="fixed"><parent link="world"/><child link="base_link"/></joint>',
            "base_to_arm_joint": b'<joint name="base_to_arm_joint" type="fixed"><parent link="base_link"/><child link="link_base"/><origin xyz="-0.03 0 0.527" rpy="0 0 0"/></joint>',
            "drive_control": b'<ros2_control name="duplicate" type="system"><joint name="drive_joint"><state_interface name="position"/></joint></ros2_control>',
        }
        for label, fragment in mutations.items():
            with self.subTest(label=label):
                source = _source_bytes()
                if label == "drive_control":
                    source = source.replace(b"</robot>", fragment + fragment + b"</robot>", 1)
                elif label == "world":
                    source = source.replace(b"</robot>", fragment + fragment + b"</robot>", 1)
                else:
                    source = source.replace(b"</robot>", fragment + b"</robot>", 1)
                with self.assertRaisesRegex(CanonicalizationError, label.replace("_", "|")):
                    canonicalize_urdf(source)

    def test_malformed_source_graph_has_typed_actionable_error(self) -> None:
        with self.assertRaisesRegex(CanonicalizationError, "malformed XML"):
            canonicalize_urdf(b"<robot>")
        source = _source_bytes().replace(b'<parent link="base_link"/>', b'<parent link="missing_link"/>', 1)
        with self.assertRaisesRegex(CanonicalizationError, "missing parent link"):
            canonicalize_urdf(source)

    def test_artifact_identity_binds_output_and_algorithm_version(self) -> None:
        source_hashes = {"robot.usd": "usd-hash", "map.pgm": "map-hash"}
        output = canonicalize_urdf(_source_bytes())
        baseline = artifact_identity(source_hashes, output, CANONICALIZER_ALGORITHM)
        changed_output = artifact_identity(source_hashes, output + b"\n", CANONICALIZER_ALGORITHM)
        changed_algorithm = artifact_identity(source_hashes, output, CANONICALIZER_ALGORITHM + "-next")
        self.assertNotEqual(baseline, changed_output)
        self.assertNotEqual(baseline, changed_algorithm)

    def test_export_preserves_old_artifact_and_publishes_hashed_complete_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temporary = Path(temporary)
            workspace = temporary / "workspace"
            artifacts = temporary / "artifacts"
            self._make_workspace(workspace)
            lock_path = artifacts / "provenance/tinker2-source-lock.json"
            old = artifacts / "robot/tinker2/311d7492c6ffffc2"
            old.mkdir(parents=True)
            old_urdf = b"old artifact bytes"
            old_usd = (workspace / "src/tk26_sim/_generated/tinker_full.usd").read_bytes()
            (old / "robot.urdf").write_bytes(old_urdf)
            (old / "robot.usd").write_bytes(old_usd)
            current = artifacts / "robot/tinker2/current.json"
            current.parent.mkdir(parents=True, exist_ok=True)
            current.write_text(json.dumps({"artifact_id": old.name, "manifest": str((old / "manifest.json").relative_to(artifacts.parent))}), encoding="utf-8")
            before_old = {path.name: path.read_bytes() for path in old.iterdir()}

            result = export_tinker2(workspace, artifacts, lock_path)
            self.assertNotEqual(result.artifact_dir.name, old.name)
            self.assertEqual({path.name: path.read_bytes() for path in old.iterdir()}, before_old)
            self.assertEqual((result.artifact_dir / "robot.usd").read_bytes(), (workspace / "src/tk26_sim/_generated/tinker_full.usd").read_bytes())
            self.assertEqual(sha256_file(old / "robot.usd"), sha256_file(result.artifact_dir / "robot.usd"))
            manifest = json.loads((result.artifact_dir / "manifest.json").read_text(encoding="utf-8"))
            records = {record["path"]: record["sha256"] for record in manifest["files"]}
            for path in result.artifact_dir.iterdir():
                if path.name == "manifest.json":
                    continue
                self.assertEqual(records[path.relative_to(artifacts.parent).as_posix()], sha256_file(path))
            self.assertEqual(manifest["canonicalization"]["algorithm"], CANONICALIZER_ALGORITHM)
            self.assertEqual(manifest["canonicalization"]["output_sha256"], sha256_file(result.artifact_dir / "robot.urdf"))
            current_raw = json.loads(current.read_text(encoding="utf-8"))
            self.assertEqual(current_raw["artifact_id"], result.artifact_dir.name)
            self.assertEqual(current_raw["manifest"], (result.artifact_dir / "manifest.json").relative_to(artifacts.parent).as_posix())
            lock = json.loads(lock_path.read_text(encoding="utf-8"))
            self.assertEqual(lock["artifact_export"]["output_sha256"], sha256_file(result.artifact_dir / "robot.urdf"))

    def test_failed_export_does_not_switch_current_or_publish_partial_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            temporary = Path(temporary)
            workspace = temporary / "workspace"
            artifacts = temporary / "artifacts"
            self._make_workspace(workspace)
            (workspace / "src/tk26_sim/_generated/tinker_full.full.urdf").write_bytes(b"<robot>")
            lock_path = artifacts / "provenance/tinker2-source-lock.json"
            from tinker_sim_deploy.workspace import capture_workspace_lock
            capture_workspace_lock(workspace, lock_path)
            current = artifacts / "robot/tinker2/current.json"
            current.parent.mkdir(parents=True, exist_ok=True)
            original_current = b'{"artifact_id":"old","manifest":"old/manifest.json"}\n'
            current.write_bytes(original_current)
            with self.assertRaises(CanonicalizationError):
                export_tinker2(workspace, artifacts, lock_path)
            self.assertEqual(current.read_bytes(), original_current)
            self.assertEqual(list((artifacts / "robot/tinker2").iterdir()), [current])

    def test_both_runtime_topic_transformers_remain_arm_only(self) -> None:
        urdf = canonicalize_urdf(_source_bytes()).decode("utf-8")
        for relative in ("ros2_ws/src/tinker_sim_bridge/launch/manipulation.launch.py", "ros2_ws/src/tinker_sim_bridge/launch/whole_robot.launch.py"):
            with self.subTest(relative=relative):
                output = _root(_topic_transformer(ROOT / relative)(urdf))
                controls = output.findall("ros2_control")
                self.assertEqual(len(controls), 1)
                joints = controls[0].findall("joint")
                self.assertEqual([joint.get("name") for joint in joints], [f"joint{i}" for i in range(1, 8)])
                self.assertNotIn("drive_joint", {joint.get("name") for joint in joints})
                self.assertNotIn("gripper", ET.tostring(controls[0], encoding="unicode").lower())
                source = (ROOT / relative).read_text(encoding="utf-8")
                self.assertEqual(source.count('executable="gripper_facade"'), 1)

    @staticmethod
    def _make_workspace(workspace: Path) -> None:
        generated = workspace / "src/tk26_sim/_generated"
        maps = workspace / "src/tk26_navigation/src/navigation_bringup/maps"
        profile = workspace / "src/tk25_basic/src/tinker_robot_config/robots/tinker2"
        generated.mkdir(parents=True)
        maps.mkdir(parents=True)
        profile.mkdir(parents=True)
        shutil.copy2(SOURCE_URDF, generated / "tinker_full.full.urdf")
        shutil.copy2(ROOT / "artifacts/robot/tinker2/311d7492c6ffffc2/robot.usd", generated / "tinker_full.usd")
        (maps / "0701_robocup_arena3.yaml").write_text("image: arena.pgm\n", encoding="utf-8")
        (maps / "0701_robocup_arena3.pgm").write_bytes(b"map")
        (profile / "robot.yaml").write_text("robot: tinker2\n", encoding="utf-8")


if __name__ == "__main__":
    unittest.main()
