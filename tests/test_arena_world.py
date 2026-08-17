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


FIXTURE_WORLD_WITH_SKIPPABLE = b"""<?xml version="1.0"?>
<sdf version="1.10">
  <world name="rcw2026_arena">
    <model name="arena_walls">
      <link name="wall_north">
        <pose>0 4.5 0.6 0 0 0</pose>
        <collision name="c"><geometry><box><size>9 0.1 1.2</size></box></geometry></collision>
      </link>
    </model>
    <include>
      <uri>model://wrc_ground_plane</uri>
      <name>floor_plane</name>
      <pose>5 5 0 0 0 0</pose>
      <static>1</static>
    </include>
    <include>
      <uri>model://rcw26_kitchen_table</uri>
      <name>kitchen_table</name>
      <pose>2.0 1.0 0 0 0 0</pose>
      <static>1</static>
    </include>
  </world>
</sdf>"""

FIXTURE_MODEL_SDF_CYLINDER = b"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="rcw26_kitchen_table">
    <link name="link">
      <collision name="collision">
        <pose>0 0 0.36 0 0 0</pose>
        <geometry><cylinder><radius>0.434</radius><length>0.72</length></cylinder></geometry>
      </collision>
    </link>
  </model>
</sdf>"""

FIXTURE_MODEL_SDF_MESH_COLLISION_ROLLED = b"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="rcw26_chair">
    <link name="link">
      <collision name="collision">
        <pose>0 0 0.45 1.5708 0 0</pose>
        <geometry><mesh><uri>model://rcw26_chair/meshes/chair.glb</uri><scale>0.9 0.9 0.9</scale></mesh></geometry>
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

    def test_static_accepts_numeric_and_word_forms(self):
        # Real upstream SOBITS worlds use <static>1</static>/<static>0</static>
        # (Gazebo's numeric boolean form) rather than the word form the
        # original fixture used; both must be recognized.
        world = FIXTURE_WORLD.replace(
            b"<static>true</static>", b"<static>1</static>"
        )
        layout = arena_world.parse_world(world, frozenset({"rcw26_kitchen_table"}))
        self.assertTrue(layout.furniture[0].static)

        world_false = FIXTURE_WORLD.replace(
            b"<static>true</static>", b"<static>0</static>"
        )
        layout_false = arena_world.parse_world(
            world_false, frozenset({"rcw26_kitchen_table"})
        )
        self.assertFalse(layout_false.furniture[0].static)

    def test_model_skiplist_excludes_include_without_error(self):
        # Real upstream worlds include benign non-furniture models (e.g. an
        # externally-sourced ground plane) that are not on the RCW26 model
        # allowlist and are not meant to become importer furniture. A
        # skiplist entry excludes them without tripping fail-closed.
        layout = arena_world.parse_world(
            FIXTURE_WORLD_WITH_SKIPPABLE,
            frozenset({"rcw26_kitchen_table"}),
            model_skiplist=frozenset({"wrc_ground_plane"}),
        )
        self.assertEqual(len(layout.furniture), 1)
        self.assertEqual(layout.furniture[0].model_id, "rcw26_kitchen_table")

    def test_model_skiplist_does_not_relax_default_fail_closed(self):
        # A skiplist that does not name the offending model must not change
        # the default fail-closed behaviour for genuinely unknown models.
        with self.assertRaises(arena_world.ArenaWorldError):
            arena_world.parse_world(
                FIXTURE_WORLD_WITH_SKIPPABLE,
                frozenset({"rcw26_kitchen_table"}),
                model_skiplist=frozenset({"some_other_model"}),
            )

    def test_model_colliders_cylinder_becomes_axis_aligned_box(self):
        # Real upstream SDFs (e.g. rcw26_kitchen_table) use a <cylinder>
        # collision geometry rather than <box>. The importer needs a box for
        # map rasterization and collider authoring, so a cylinder collider is
        # represented as its axis-aligned bounding box (diameter x diameter x
        # length), centred/yawed exactly as declared.
        colliders = arena_world.parse_model_colliders(FIXTURE_MODEL_SDF_CYLINDER)
        self.assertEqual(len(colliders), 1)
        box = colliders[0]
        self.assertIsInstance(box, arena_world.BoxCollider)
        self.assertAlmostEqual(box.size[0], 0.868)
        self.assertAlmostEqual(box.size[1], 0.868)
        self.assertAlmostEqual(box.size[2], 0.72)
        self.assertEqual(box.center, (0.0, 0.0, 0.36))

    def test_model_colliders_mesh_pose_roll_is_not_validated(self):
        # Real upstream SDFs (e.g. rcw26_chair) give a mesh collision the
        # SAME <pose> as its visual -- including the 90-degree roll that
        # corrects the source GLB's Y-up authoring to Z-up. MeshCollider
        # carries no pose (author_model_colliders applies convexHull
        # directly to the already visually-corrected mesh prim), so this
        # roll must not trip parse_model_colliders's box/wall roll==0
        # fail-closed rule -- that rule exists for authored box geometry,
        # not for a pose value the importer never reads.
        colliders = arena_world.parse_model_colliders(FIXTURE_MODEL_SDF_MESH_COLLISION_ROLLED)
        self.assertEqual(len(colliders), 1)
        self.assertIsInstance(colliders[0], arena_world.MeshCollider)
        self.assertEqual(colliders[0].uri, "model://rcw26_chair/meshes/chair.glb")


if __name__ == "__main__":
    unittest.main()
