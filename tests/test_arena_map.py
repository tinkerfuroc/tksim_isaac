from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_deploy import arena_map, arena_world
from tinker_sim_core.occupancy import OccupancyMap

URDF_SNIPPET = b"""<robot name="t">
  <joint name="livox_joint" type="fixed">
    <parent link="base_link"/><child link="livox_frame"/>
    <origin rpy="0 0 0" xyz="0.09 0 0.195"/>
  </joint>
</robot>"""


class ScanHeightTest(unittest.TestCase):
    def test_reads_livox_joint_z(self):
        self.assertAlmostEqual(arena_map.livox_scan_height(URDF_SNIPPET), 0.195)

    def test_missing_joint_fails(self):
        with self.assertRaises(arena_map.ArenaMapError):
            arena_map.livox_scan_height(b"<robot name='t'/>")


def _square_room() -> arena_world.ArenaLayout:
    walls = []
    for name, center, size in (
        ("n", (0.0, 2.0, 0.6), (4.1, 0.1, 1.2)),
        ("s", (0.0, -2.0, 0.6), (4.1, 0.1, 1.2)),
        ("e", (2.0, 0.0, 0.6), (0.1, 4.1, 1.2)),
        ("w", (-2.0, 0.0, 0.6), (0.1, 4.1, 1.2)),
    ):
        walls.append(arena_world.WallBox(name=name, size=size, center=center, yaw=0.0))
    table = arena_world.FurniturePlacement(
        model_id="rcw26_kitchen_table", position=(1.0, 1.0, 0.0), yaw=0.0, static=True
    )
    return arena_world.ArenaLayout(walls=tuple(walls), furniture=(table,))


class RasterizeTest(unittest.TestCase):
    COLLIDERS = {
        "rcw26_kitchen_table": (
            # tabletop: entirely ABOVE the 0.195 scan plane, must not mark cells
            arena_world.BoxCollider(size=(0.8, 0.4, 0.04), center=(0.0, 0.0, 0.72), yaw=0.0),
            # leg/body box spanning z [0.0, 0.70]: crosses the scan plane, marks cells
            arena_world.BoxCollider(size=(0.7, 0.3, 0.70), center=(0.0, 0.0, 0.35), yaw=0.0),
        )
    }

    def _load(self, colliders=None):
        pgm, yaml_bytes = arena_map.rasterize(
            _square_room(), colliders if colliders is not None else self.COLLIDERS,
            scan_height=0.195,
        )
        import tempfile, pathlib
        with tempfile.TemporaryDirectory() as tmp:
            base = pathlib.Path(tmp)
            (base / "map.pgm").write_bytes(pgm)
            (base / "map.yaml").write_bytes(yaml_bytes)
            meta = {}
            for line in yaml_bytes.decode().splitlines():
                key, _, value = line.partition(":")
                meta[key.strip()] = value.strip()
            self.assertEqual(meta["image"], "map.pgm")
            self.assertEqual(meta["resolution"], "0.05")
            origin = json.loads(meta["origin"])
            return OccupancyMap.from_pgm(
                base / "map.pgm", resolution=0.05,
                origin_x=origin[0], origin_y=origin[1],
            )

    def test_walls_table_and_free_space(self):
        grid = self._load()
        self.assertTrue(grid.occupied_at_world(0.0, 2.0))    # north wall
        self.assertTrue(grid.occupied_at_world(1.0, 1.0))    # table top slice
        self.assertFalse(grid.occupied_at_world(0.0, 0.0))   # centre is free
        self.assertFalse(grid.occupied_at_world(-1.5, -1.5)) # corner floor free

    def test_outside_scan_plane_not_marked(self):
        for center_z, size_z in ((0.05, 0.10), (0.72, 0.04)):  # plinth below, top above
            colliders = {
                "rcw26_kitchen_table": (
                    arena_world.BoxCollider(size=(0.8, 0.4, size_z), center=(0.0, 0.0, center_z), yaw=0.0),
                )
            }
            grid = self._load(colliders)
            self.assertFalse(grid.occupied_at_world(1.0, 1.0))


if __name__ == "__main__":
    unittest.main()
