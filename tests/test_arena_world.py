from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy import arena_world

FIXTURE_WORLD = b"""<?xml version="1.0"?>
<sdf version="1.10">
  <world name="rcw2026_arena">
    <model name="arena_walls">
      <static>true</static>
      <pose>1 0 0 0 0 0</pose>
      <link name="wall_north">
        <pose>0 4.5 0.6 0 0 0</pose>
        <collision name="c"><geometry><box><size>9 0.1 1.2</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>9 0.1 1.2</size></box></geometry></visual>
      </link>
    </model>
    <include>
      <uri>model://rcw26_kitchen_table</uri>
      <name>kitchen_table</name>
      <pose>2.0 1.0 0 0 0 1.5707963267948966</pose>
      <static>true</static>
    </include>
  </world>
</sdf>"""

FIXTURE_MODEL_SDF = b"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="rcw26_kitchen_table">
    <link name="body">
      <collision name="top">
        <pose>0 0 0.72 0 0 0</pose>
        <geometry><box><size>1.2 0.6 0.04</size></box></geometry>
      </collision>
      <collision name="mesh_part">
        <geometry><mesh><uri>meshes/rcw26_kitchen_table.glb</uri></mesh></geometry>
      </collision>
    </link>
  </model>
</sdf>"""


class ParseWorldTest(unittest.TestCase):
    def test_walls_compose_model_and_link_pose(self):
        layout = arena_world.parse_world(
            FIXTURE_WORLD, frozenset({"rcw26_kitchen_table"})
        )
        self.assertEqual(len(layout.walls), 1)
        wall = layout.walls[0]
        self.assertEqual(wall.size, (9.0, 0.1, 1.2))
        # model pose (1,0,0) + link pose (0,4.5,0.6)
        self.assertEqual(wall.center, (1.0, 4.5, 0.6))
        self.assertEqual(wall.yaw, 0.0)

    def test_furniture_include_parsed(self):
        layout = arena_world.parse_world(
            FIXTURE_WORLD, frozenset({"rcw26_kitchen_table"})
        )
        self.assertEqual(len(layout.furniture), 1)
        item = layout.furniture[0]
        self.assertEqual(item.model_id, "rcw26_kitchen_table")
        self.assertEqual(item.position, (2.0, 1.0, 0.0))
        self.assertAlmostEqual(item.yaw, 1.5707963267948966)
        self.assertTrue(item.static)

    def test_unknown_model_fails_closed(self):
        with self.assertRaises(arena_world.ArenaWorldError):
            arena_world.parse_world(FIXTURE_WORLD, frozenset({"rcw26_sofa"}))

    def test_nonzero_roll_pitch_fails_closed(self):
        bad = FIXTURE_WORLD.replace(b"0 0 1.5707963267948966", b"0.3 0 0")
        with self.assertRaises(arena_world.ArenaWorldError):
            arena_world.parse_world(bad, frozenset({"rcw26_kitchen_table"}))

    def test_model_colliders(self):
        colliders = arena_world.parse_model_colliders(FIXTURE_MODEL_SDF)
        self.assertEqual(len(colliders), 2)
        box = colliders[0]
        self.assertEqual(box.size, (1.2, 0.6, 0.04))
        self.assertEqual(box.center, (0.0, 0.0, 0.72))
        self.assertIsInstance(colliders[1], arena_world.MeshCollider)
        self.assertEqual(colliders[1].uri, "meshes/rcw26_kitchen_table.glb")


if __name__ == "__main__":
    unittest.main()
