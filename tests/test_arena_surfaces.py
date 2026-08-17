from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy import arena_surfaces, arena_world


class WorldSurfaceTest(unittest.TestCase):
    def test_rotation_composition(self):
        furniture = arena_world.FurniturePlacement(
            model_id="rcw26_kitchen_table", position=(1.0, 2.0, 0.0),
            yaw=math.pi / 2.0, static=True,
        )
        surface = arena_surfaces.world_surface(
            furniture, surface_name="top",
            local_center=(0.5, 0.0, 0.74), size_xy=(1.2, 0.6), edge_margin=0.05,
        )
        self.assertAlmostEqual(surface.center_xyz[0], 1.0)
        self.assertAlmostEqual(surface.center_xyz[1], 2.5)
        self.assertAlmostEqual(surface.center_xyz[2], 0.74)
        self.assertAlmostEqual(surface.yaw, math.pi / 2.0)
        self.assertEqual(surface.surface_id, "kitchen_table#top")

    def test_placement_json_is_canonical_and_schema1(self):
        furniture = arena_world.FurniturePlacement(
            model_id="rcw26_kitchen_table", position=(1.0, 2.0, 0.0),
            yaw=math.pi / 2.0, static=True,
        )
        surface = arena_surfaces.world_surface(
            furniture, surface_name="top",
            local_center=(0.5, 0.0, 0.74), size_xy=(1.2, 0.6), edge_margin=0.05,
        )
        payload = json.loads(arena_surfaces.placement_json("rcw2026", [surface]))
        self.assertEqual(payload["schema_version"], 1)
        self.assertEqual(payload["arena_id"], "rcw2026")
        self.assertEqual(len(payload["surfaces"]), 1)
        record = payload["surfaces"][0]
        self.assertEqual(
            set(record),
            {"surface_id", "furniture_id", "center_xyz", "size_xy", "yaw", "edge_margin"},
        )
        # byte-determinism
        self.assertEqual(
            arena_surfaces.placement_json("rcw2026", [surface]),
            arena_surfaces.placement_json("rcw2026", [surface]),
        )


if __name__ == "__main__":
    unittest.main()
