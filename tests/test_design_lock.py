from __future__ import annotations

import hashlib
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from tinker_designs.lock import design_records, design_source_lock


class DesignLockTest(unittest.TestCase):
    def test_records_cover_design_files_and_extras_in_path_order(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = repo / "designs" / "demo"
            (design_dir / "meshes").mkdir(parents=True)
            (design_dir / "robot.urdf").write_bytes(b"<robot/>")
            (design_dir / "design.yaml").write_bytes(b"name: demo\n")
            (design_dir / "meshes" / "a.stl").write_bytes(b"solid")
            upstream = repo.parent / "upstream_mesh.stl"
            upstream.write_bytes(b"external")
            records = design_records(design_dir, repo, [upstream])
            self.assertEqual([r["path"] for r in records], sorted([
                "designs/demo/design.yaml", "designs/demo/meshes/a.stl", "designs/demo/robot.urdf", str(upstream),
            ]))
            design_yaml_record = next(r for r in records if r["path"] == "designs/demo/design.yaml")
            self.assertEqual(design_yaml_record["sha256"], hashlib.sha256(b"name: demo\n").hexdigest())
            lock = json.loads(design_source_lock(design_dir, repo, [upstream]))
            self.assertEqual(lock["robot"], "demo")
            self.assertEqual(lock["schema_version"], 3)
            self.assertEqual(len(lock["files"]), 4)

    def test_extra_file_already_under_design_dir_is_not_duplicated(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = repo / "designs" / "demo"
            (design_dir / "meshes").mkdir(parents=True)
            (design_dir / "robot.urdf").write_bytes(b"<robot/>")
            (design_dir / "design.yaml").write_bytes(b"name: demo\n")
            (design_dir / "meshes" / "a.stl").write_bytes(b"solid")
            records = design_records(design_dir, repo, [design_dir / "meshes" / "a.stl"])
            paths = [r["path"] for r in records]
            self.assertEqual(len(records), 3)
            self.assertEqual(len(set(paths)), 3)
            lock = json.loads(design_source_lock(design_dir, repo, [design_dir / "meshes" / "a.stl"]))
            self.assertEqual(len(lock["files"]), 3)

    def test_symlinks_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = repo / "designs" / "demo"
            design_dir.mkdir(parents=True)
            (design_dir / "robot.urdf").write_bytes(b"<robot/>")
            os.symlink(design_dir / "robot.urdf", design_dir / "link.urdf")
            with self.assertRaises(Exception):
                design_records(design_dir, repo, [])


if __name__ == "__main__":
    unittest.main()
