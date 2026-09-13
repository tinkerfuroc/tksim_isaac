# tests/test_design_clean.py
from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from tinker_designs.clean import CleanError, canonical_bytes, expand_xacro, package_share_dirs, resolve_mesh_uris, strip_gazebo

URDF = b"""<?xml version="1.0"?>
<robot name="c">
  <gazebo reference="base_link"><material>x</material></gazebo>
  <link name="base_link">
    <visual><geometry><mesh filename="package://demo_pkg/meshes/body.stl"/></geometry></visual>
    <collision><geometry><mesh filename="meshes/local.stl"/></geometry></collision>
    <gazebo><nested/></gazebo>
  </link>
</robot>
"""


class CleanTest(unittest.TestCase):
    def test_strip_gazebo_removes_nested_blocks(self) -> None:
        root = ET.fromstring(URDF)
        self.assertEqual(strip_gazebo(root), 2)
        self.assertEqual(root.findall(".//gazebo"), [])

    def test_package_share_dirs_scans_ament_prefix_path(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            prefix = Path(temporary) / "install"
            share = prefix / "share" / "demo_pkg"
            share.mkdir(parents=True)
            (share / "package.xml").write_text("<package/>", encoding="utf-8")
            (prefix / "share" / "not_a_pkg").mkdir()
            found = package_share_dirs({"AMENT_PREFIX_PATH": f"{prefix}:/nonexistent"})
            self.assertEqual(found, {"demo_pkg": share})

    def test_resolve_rewrites_package_and_relative_uris(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            share = Path(temporary) / "share" / "demo_pkg"
            (share / "meshes").mkdir(parents=True)
            (share / "meshes" / "body.stl").write_bytes(b"solid")
            design_dir = Path(temporary) / "design"
            (design_dir / "meshes").mkdir(parents=True)
            (design_dir / "meshes" / "local.stl").write_bytes(b"solid")
            root = ET.fromstring(URDF)
            resolved = resolve_mesh_uris(root, design_dir=design_dir, packages={"demo_pkg": share})
            names = [mesh.get("filename") for mesh in root.iter("mesh")]
            self.assertEqual(names, [f"file://{share / 'meshes' / 'body.stl'}", f"file://{design_dir / 'meshes' / 'local.stl'}"])
            self.assertEqual(sorted(resolved), sorted([share / "meshes" / "body.stl", design_dir / "meshes" / "local.stl"]))

    def test_resolve_reports_every_unresolved_uri(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = ET.fromstring(URDF)
            with self.assertRaises(CleanError) as caught:
                resolve_mesh_uris(root, design_dir=Path(temporary), packages={})
            message = str(caught.exception)
            self.assertIn("demo_pkg", message)
            self.assertIn("meshes/local.stl", message)

    def test_canonical_bytes_is_idempotent_and_drops_comments(self) -> None:
        root = ET.fromstring(b"<robot name='c'><!-- note --><link name='a'/></robot>")
        first = canonical_bytes(root)
        self.assertNotIn(b"note", first)
        self.assertTrue(first.endswith(b"\n"))
        self.assertEqual(canonical_bytes(ET.fromstring(first)), first)

    def test_expand_xacro_shells_out_and_surfaces_stderr(self) -> None:
        calls = []

        def runner(command, **kwargs):
            calls.append(command)
            return subprocess.CompletedProcess(command, 0, stdout=b"<robot name='x'/>", stderr=b"")

        self.assertEqual(expand_xacro(Path("/tmp/r.urdf.xacro"), runner=runner), b"<robot name='x'/>")
        self.assertEqual(calls[0][:2], ["xacro", "/tmp/r.urdf.xacro"])

        def failing(command, **kwargs):
            return subprocess.CompletedProcess(command, 1, stdout=b"", stderr=b"boom")

        with self.assertRaisesRegex(CleanError, "boom"):
            expand_xacro(Path("/tmp/r.urdf.xacro"), runner=failing)


if __name__ == "__main__":
    unittest.main()
