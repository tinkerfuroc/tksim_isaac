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


WHEELS = b"""
  <link name="front_left_wheel"><collision><origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <geometry><cylinder length="0.063" radius="0.0525"/></geometry></collision></link>
  <joint name="front_left_wheel_joint" type="continuous"><parent link="base_link"/>
    <child link="front_left_wheel"/><origin xyz="0 0.125 -0.025" rpy="0 0 0"/></joint>
</robot>"""


class ScanHeightTest(unittest.TestCase):
    def test_without_wheels_base_link_is_taken_as_the_floor(self):
        self.assertAlmostEqual(arena_map.livox_scan_height(URDF_SNIPPET), 0.195)

    def test_adds_the_base_link_rest_height_from_the_wheels(self):
        # In sim base_link rests 0.0775 m up (wheel bottoms at z -0.0775), so
        # the sensor plane is 0.195 + 0.0775 above the floor.
        wheeled = URDF_SNIPPET.replace(b"</robot>", WHEELS)
        self.assertAlmostEqual(arena_map.livox_scan_height(wheeled), 0.2725)

    def test_band_is_what_pointcloud_to_laserscan_keeps(self):
        # pointcloud_to_laserscan keeps points 0..2 m above the sensor
        # (livox360 frame) and projects them all into the 2-D scan.
        wheeled = URDF_SNIPPET.replace(b"</robot>", WHEELS)
        low, high = arena_map.livox_scan_band(wheeled)
        self.assertAlmostEqual(low, 0.2725)
        self.assertAlmostEqual(high, 2.2725)
        import yaml
        cfg = yaml.safe_load((ROOT / "ros2_ws/src/tinker_sim_bridge/config/pointcloud_to_laserscan.yaml")
                             .read_text(encoding="utf-8"))["pointcloud_to_laserscan"]["ros__parameters"]
        self.assertEqual(cfg["target_frame"], "livox360")
        self.assertAlmostEqual(cfg["min_height"], arena_map.SCAN_BAND_MIN_ABOVE_SENSOR_M)
        self.assertAlmostEqual(cfg["max_height"], arena_map.SCAN_BAND_MAX_ABOVE_SENSOR_M)

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
            # tabletop: above the sensor plane, inside the scan band
            arena_world.BoxCollider(size=(0.8, 0.4, 0.04), center=(0.0, 0.0, 0.72), yaw=0.0),
            # leg/body box spanning z [0.0, 0.70]: crosses the scan plane, marks cells
            arena_world.BoxCollider(size=(0.7, 0.3, 0.70), center=(0.0, 0.0, 0.35), yaw=0.0),
        )
    }

    BAND = (0.2725, 2.2725)

    def _load(self, colliders=None, **scan):
        pgm, yaml_bytes = arena_map.rasterize(
            _square_room(), colliders if colliders is not None else self.COLLIDERS,
            **(scan or {"scan_band": self.BAND}),
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

    @staticmethod
    def _one_box(center_z, size_z):
        return {"rcw26_kitchen_table": (
            arena_world.BoxCollider(size=(0.8, 0.4, size_z), center=(0.0, 0.0, center_z), yaw=0.0),)}

    def test_geometry_outside_the_scan_band_is_not_marked(self):
        # A plinth below the sensor plane, a fixture above the band: the
        # scan never sees either.
        for center_z, size_z in ((0.05, 0.10), (2.5, 0.2)):
            grid = self._load(self._one_box(center_z, size_z))
            self.assertFalse(grid.occupied_at_world(1.0, 1.0), (center_z, size_z))

    def test_geometry_anywhere_in_the_band_is_marked(self):
        # A tabletop or shelf board above the sensor plane is a scan hit:
        # pointcloud_to_laserscan projects the whole band into the scan.
        for center_z, size_z in ((0.72, 0.04), (1.95, 0.05), (0.30, 0.04)):
            grid = self._load(self._one_box(center_z, size_z))
            self.assertTrue(grid.occupied_at_world(1.0, 1.0), (center_z, size_z))

    def test_a_single_height_is_still_a_plane_slice(self):
        grid = self._load(self._one_box(0.72, 0.04), scan_height=0.195)
        self.assertFalse(grid.occupied_at_world(1.0, 1.0))


if __name__ == "__main__":
    unittest.main()
