from __future__ import annotations

import hashlib
import json
import re
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.workspace import (
    PUBLICATION_SCHEMA, ArtifactPublicationError, UnsafePathError, _normalized_source_lock,
    canonicalize_urdf, publish_robot_artifact,
)


def _publish(artifacts: Path, robot: str = "demo", urdf: bytes = b"<robot name='d'/>\n") -> tuple[Path, dict]:
    lock = _normalized_source_lock([{"path": "designs/demo/robot.urdf", "size": len(urdf), "sha256": hashlib.sha256(urdf).hexdigest()}], robot=robot)
    result = publish_robot_artifact(
        artifacts, robot=robot,
        file_bytes={"robot.urdf": urdf, "robot.usd": b"usd", "robot-profile.yaml": b"robot: demo\n", "meshes/a.stl": b"solid"},
        canonical_urdf=urdf, source_lock_bytes=lock, canonicalizer="tinker-designs-canonical-v1",
        manifest_extra={"kinematics": {"wheel_radius_m": 0.1}}, source_path="designs/demo/robot.urdf",
        source_sha256=hashlib.sha256(urdf).hexdigest(),
    )
    return result.artifact_dir, result.manifest


class PublishRobotArtifactTest(unittest.TestCase):
    def test_publishes_under_robot_family_with_pointer(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifacts = Path(temporary) / "artifacts"
            artifact_dir, manifest = _publish(artifacts)
            self.assertTrue(re.fullmatch(r"[0-9a-f]{64}", artifact_dir.name))
            self.assertEqual(artifact_dir.parent, artifacts / "robot" / "demo")
            self.assertEqual(manifest["robot"], "demo")
            self.assertEqual(manifest["schema_version"], PUBLICATION_SCHEMA)
            self.assertEqual(manifest["kinematics"], {"wheel_radius_m": 0.1})
            self.assertEqual(manifest["canonicalization"]["algorithm"], "tinker-designs-canonical-v1")
            self.assertEqual({item["path"] for item in manifest["files"]}, {f"artifacts/robot/demo/{artifact_dir.name}/{n}" for n in ("robot.urdf", "robot.usd", "robot-profile.yaml", "meshes/a.stl")})
            self.assertTrue((artifact_dir / "meshes" / "a.stl").is_file())
            current = json.loads((artifacts / "robot" / "demo" / "current.json").read_text())
            self.assertEqual(current["robot"], "demo")
            self.assertEqual(current["artifact_dir"], f"artifacts/robot/demo/{artifact_dir.name}")
            self.assertEqual(current["robot_urdf_sha256"], hashlib.sha256(b"<robot name='d'/>\n").hexdigest())

    def test_identity_is_content_addressed_and_republish_is_idempotent(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifacts = Path(temporary) / "artifacts"
            first, _ = _publish(artifacts)
            second, _ = _publish(artifacts)
            self.assertEqual(first, second)
            third, _ = _publish(artifacts, urdf=b"<robot name='e'/>\n")
            self.assertNotEqual(first, third)

    def test_robot_name_and_file_keys_are_validated(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifacts = Path(temporary) / "artifacts"
            with self.assertRaises(ArtifactPublicationError):
                _publish(artifacts, robot="Bad Name")
            lock = _normalized_source_lock([], robot="demo")
            with self.assertRaises(UnsafePathError):
                publish_robot_artifact(
                    artifacts, robot="demo", file_bytes={"../escape": b"x", "robot.urdf": b"<robot/>"},
                    canonical_urdf=b"<robot/>", source_lock_bytes=lock, canonicalizer="c", manifest_extra={},
                    source_path="p", source_sha256="0" * 64,
                )

    def test_absolute_file_key_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifacts = Path(temporary) / "artifacts"
            lock = _normalized_source_lock([], robot="demo")
            with self.assertRaises(UnsafePathError):
                publish_robot_artifact(
                    artifacts, robot="demo", file_bytes={"/etc/passwd": b"x", "robot.urdf": b"<robot/>"},
                    canonical_urdf=b"<robot/>", source_lock_bytes=lock, canonicalizer="c", manifest_extra={},
                    source_path="p", source_sha256="0" * 64,
                )
            self.assertFalse((artifacts / "robot").exists())

    def test_manifest_extra_reserved_key_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifacts = Path(temporary) / "artifacts"
            lock = _normalized_source_lock([], robot="demo")
            with self.assertRaisesRegex(ValueError, "schema_version"):
                publish_robot_artifact(
                    artifacts, robot="demo", file_bytes={"robot.urdf": b"<robot/>", "robot.usd": b"usd"},
                    canonical_urdf=b"<robot/>", source_lock_bytes=lock, canonicalizer="c",
                    manifest_extra={"schema_version": 99}, source_path="p", source_sha256="0" * 64,
                )

    def test_missing_required_file_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifacts = Path(temporary) / "artifacts"
            lock = _normalized_source_lock([], robot="demo")
            with self.assertRaisesRegex(ValueError, "robot.usd"):
                publish_robot_artifact(
                    artifacts, robot="demo", file_bytes={"robot.urdf": b"<robot/>"},
                    canonical_urdf=b"<robot/>", source_lock_bytes=lock, canonicalizer="c",
                    manifest_extra={}, source_path="p", source_sha256="0" * 64,
                )

    def test_source_lock_carries_robot(self) -> None:
        lock = json.loads(_normalized_source_lock([], robot="demo"))
        self.assertEqual(lock["robot"], "demo")
        self.assertEqual(json.loads(_normalized_source_lock([]))["robot"], "tinker2")


class CanonicalizeMountOriginTest(unittest.TestCase):
    def test_mount_origin_parameter_is_honoured(self) -> None:
        sys.path.insert(0, str(ROOT / "tests"))
        from test_artifact_export import _fixture_urdf
        moved = _fixture_urdf().replace(b'xyz="-0.03 0 0.527"', b'xyz="0.1 0 0.6"')
        with self.assertRaises(Exception):
            canonicalize_urdf(moved)
        canonical = canonicalize_urdf(moved, mount_origin=(0.1, 0.0, 0.6))
        self.assertIn(b'xyz="0.1 0 0.6"', canonical)


if __name__ == "__main__":
    unittest.main()
