from __future__ import annotations

import io
import json
import os
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.bundle import (
    _checksums,
    _copy_entry,
    _write_reproducible_tar_gz,
    create,
    restore,
)
from tinker_sim_deploy.config import Config


class BundleSafetyTest(unittest.TestCase):
    def test_create_rejects_clean_clone_without_verified_robot_generation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            raw = json.loads((ROOT / "deployment.json").read_text())
            config = Config(root=root, raw=raw)
            with self.assertRaises(RuntimeError):
                create(config, root / "bundle.tar.gz", root / "uv")

    def test_rejects_path_traversal(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            archive_path = root / "bad.tar.gz"
            with tarfile.open(archive_path, "w:gz") as archive:
                payload = b"escape"
                info = tarfile.TarInfo("../escape")
                info.size = len(payload)
                archive.addfile(info, io.BytesIO(payload))
            with self.assertRaises(RuntimeError):
                restore(archive_path, root / "destination")
            self.assertFalse((root / "escape").exists())

    def test_refuses_nonempty_destination(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            destination = root / "destination"
            destination.mkdir()
            (destination / "keep").write_text("user data", encoding="utf-8")
            with self.assertRaises(RuntimeError):
                restore(root / "missing.tar.gz", destination)
            self.assertEqual((destination / "keep").read_text(encoding="utf-8"), "user data")

    def test_restores_file_and_relative_symlink_with_integrity(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            stage = root / "stage"
            stage.mkdir()
            (stage / "payload").write_text("content", encoding="utf-8")
            (stage / "alias").symlink_to("payload")
            manifest = {"schema_version": 1, "files": _checksums(stage)}
            (stage / "checksums.json").write_text(json.dumps(manifest), encoding="utf-8")
            archive = root / "bundle.tar.gz"
            _write_reproducible_tar_gz(stage, archive)
            destination = restore(archive, root / "destination", profile="physics_only")
            self.assertEqual((destination / "payload").read_text(), "content")
            self.assertTrue((destination / "alias").is_symlink())
            self.assertEqual(os.readlink(destination / "alias"), "payload")

    def test_rewrites_internal_absolute_symlink_for_portability(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source"
            source.mkdir()
            (source / "target").write_text("content", encoding="utf-8")
            (source / "alias").symlink_to((source / "target").resolve())
            destination = root / "destination"
            _copy_entry(source, destination)
            self.assertEqual(os.readlink(destination / "alias"), "target")


if __name__ == "__main__":
    unittest.main()
