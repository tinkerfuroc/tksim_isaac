"""Content-addressed asset artifact publication tests.

Covers the trust-anchor module used by the arena and objects importer CLIs:
identity derivation, atomic pointer choreography (mirroring the Tinker 2
robot artifact exporter in ``workspace.py``), crash recovery, and the
independent verifier that re-derives identity from on-disk bytes.
"""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy import arena_artifact

_SOURCE_LOCK = {
    "repository": "https://example/repo",
    "commit": "a" * 40,
    "records": [{"path": "worlds/x.world.xacro", "size": 3, "sha256": "b" * 64}],
}


class PublicationTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)

    def _publish(self, tmp_root, payload=None):
        return arena_artifact.publish_asset_artifact(
            tmp_root,
            kind="arena",
            asset_id="rcw2026",
            payload=payload or {"arena.usd": b"usd-bytes", "map.yaml": b"image: map.pgm\n"},
            source_lock=_SOURCE_LOCK,
        )

    def test_publish_creates_immutable_dir_and_pointer(self):
        root = self.root
        pub = self._publish(root)
        self.assertRegex(pub.artifact_dir.name, r"^[0-9a-f]{64}$")
        self.assertEqual(
            pub.artifact_dir.parent,
            root / "artifacts" / "arena" / "rcw2026",
        )
        self.assertEqual((pub.artifact_dir / "arena.usd").read_bytes(), b"usd-bytes")
        pointer = json.loads(
            (root / "artifacts" / "arena" / "rcw2026" / "current.json").read_text()
        )
        self.assertEqual(
            pointer["manifest"],
            f"artifacts/arena/rcw2026/{pub.identity}/manifest.json",
        )
        manifest = json.loads((pub.artifact_dir / "manifest.json").read_text())
        self.assertEqual(manifest["identity"], pub.identity)

    def test_republish_is_idempotent(self):
        root = self.root
        first = self._publish(root)
        second = self._publish(root)
        self.assertEqual(first.identity, second.identity)
        self.assertTrue(first.created)
        self.assertFalse(second.created)

    def test_payload_change_changes_identity(self):
        root = self.root
        first = self._publish(root)
        second = self._publish(
            root,
            payload={"arena.usd": b"different-usd-bytes", "map.yaml": b"image: map.pgm\n"},
        )
        self.assertNotEqual(first.identity, second.identity)
        self.assertTrue(first.created)
        self.assertTrue(second.created)
        self.assertNotEqual(first.artifact_dir, second.artifact_dir)
        # both content-addressed dirs must persist independently
        self.assertTrue(first.artifact_dir.is_dir())
        self.assertTrue(second.artifact_dir.is_dir())

    def test_crash_before_pointer_leaves_orphan_only(self):
        # Patch arena_artifact._atomic_write to raise only on the current.json
        # write, letting every earlier staging write go through unmodified.
        root = self.root
        real_atomic_write = arena_artifact._atomic_write

        def side_effect(path, data):
            if Path(path).name == "current.json":
                raise RuntimeError("simulated crash before pointer commit")
            return real_atomic_write(path, data)

        pointer_path = root / "artifacts" / "arena" / "rcw2026" / "current.json"
        artifact_root = root / "artifacts" / "arena" / "rcw2026"

        with mock.patch.object(arena_artifact, "_atomic_write", side_effect=side_effect):
            with self.assertRaises(RuntimeError):
                self._publish(root)

        # Pointer must be absent -- the crash happened strictly before its write.
        self.assertFalse(pointer_path.exists())

        # No leftover staging directories: the content-addressed dir was fully
        # written and renamed into place before the crash; only the pointer
        # commit failed, so at most an orphan (unreferenced) artifact dir
        # remains, never a half-written ".artifact-stage-*" directory.
        if artifact_root.is_dir():
            leftover_stage_dirs = [
                entry for entry in artifact_root.iterdir()
                if entry.is_dir() and entry.name.startswith(".artifact-stage-")
            ]
            self.assertEqual(leftover_stage_dirs, [])

        # Re-publishing without the patch must succeed and commit the pointer.
        pub = self._publish(root)
        self.assertTrue(pointer_path.exists())
        pointer = json.loads(pointer_path.read_text())
        self.assertEqual(
            pointer["manifest"],
            f"artifacts/arena/rcw2026/{pub.identity}/manifest.json",
        )
        manifest = json.loads((pub.artifact_dir / "manifest.json").read_text())
        self.assertEqual(manifest["identity"], pub.identity)

    def test_verify_detects_mutation(self):
        pub = self._publish(self.root)
        (pub.artifact_dir / "arena.usd").write_bytes(b"tampered")
        self.assertTrue(arena_artifact.verify_asset_artifact(pub.artifact_dir))

    def test_verify_clean(self):
        pub = self._publish(self.root)
        self.assertEqual(arena_artifact.verify_asset_artifact(pub.artifact_dir), [])

    def test_reserved_payload_names_rejected(self):
        with self.assertRaises(arena_artifact.AssetArtifactError):
            self._publish(self.root, payload={"manifest.json": b"x"})


class ValidationEdgeCaseTest(unittest.TestCase):
    """Extra fail-closed coverage beyond the brief's minimum sketch."""

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)

    def _publish(self, payload):
        return arena_artifact.publish_asset_artifact(
            self.root,
            kind="arena",
            asset_id="rcw2026",
            payload=payload,
            source_lock=_SOURCE_LOCK,
        )

    def test_source_lock_reserved_name_rejected(self):
        with self.assertRaises(arena_artifact.AssetArtifactError):
            self._publish({"source-lock.json": b"x"})

    def test_current_json_reserved_name_rejected(self):
        with self.assertRaises(arena_artifact.AssetArtifactError):
            self._publish({"current.json": b"x"})

    def test_absolute_payload_path_rejected(self):
        with self.assertRaises(arena_artifact.AssetArtifactError):
            self._publish({"/etc/passwd": b"x"})

    def test_traversal_payload_path_rejected(self):
        with self.assertRaises(arena_artifact.AssetArtifactError):
            self._publish({"../escape.txt": b"x"})

    def test_nested_traversal_payload_path_rejected(self):
        with self.assertRaises(arena_artifact.AssetArtifactError):
            self._publish({"sub/../../escape.txt": b"x"})


class AttributionMarkdownTest(unittest.TestCase):
    def test_deterministic_concatenation(self):
        result = arena_artifact.attribution_markdown(
            [("RoboCup Arena", "Body one.  "), ("YCB Objects", "Body two.\n\n")]
        )
        self.assertEqual(
            result,
            b"## RoboCup Arena\n\nBody one.\n\n## YCB Objects\n\nBody two.\n",
        )

    def test_is_pure_no_hidden_state(self):
        sections = [("A", "x"), ("B", "y")]
        first = arena_artifact.attribution_markdown(sections)
        second = arena_artifact.attribution_markdown(sections)
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
