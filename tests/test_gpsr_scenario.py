from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

SCENARIO = ROOT / "simulation/scenarios/gpsr-rcw2026.json"
ARENA = ROOT / "artifacts/arena/rcw2026/d2b559b43207c8d54ae2609f638dca1cc36ee8b7adc7e4d94aee86e7fb56729c"


class GpsrScenarioTest(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SCENARIO.is_file(), f"{SCENARIO} does not exist")
        self.raw = json.loads(SCENARIO.read_text(encoding="utf-8"))

    def test_id_matches_filename_stem(self):
        self.assertEqual(self.raw["id"], SCENARIO.stem)

    def test_targets_the_arena(self):
        self.assertEqual(self.raw["world"], {"mode": "arena", "arena": "rcw2026"})

    def test_objects_reference_existing_ycb_assets(self):
        self.assertTrue(self.raw["objects"], "scenario must spawn at least one object")
        for record in self.raw["objects"]:
            uri = record["asset_uri"]
            self.assertIn("artifacts/objects/ycb/", uri)
            self.assertTrue((ROOT / uri).is_file(), f"missing asset: {uri}")

    @unittest.skipUnless((ARENA / "map.pgm").is_file(),
                         "rcw2026 arena artifact not present in this checkout")
    def test_robot_spawn_is_free(self):
        from tinker_sim_core.occupancy import OccupancyMap
        occupancy = OccupancyMap.from_pgm(
            ARENA / "map.pgm", resolution=0.05, origin_x=-5.05, origin_y=-6.0
        )
        x, y, _ = self.raw["robot"]["initial_pose"]
        self.assertTrue(occupancy.free_with_clearance(x, y, 0.35))

    def test_objects_sit_on_declared_surfaces(self):
        surfaces = {
            s["surface_id"]: s
            for s in json.loads((ARENA / "placement.json").read_text())["surfaces"]
        } if (ARENA / "placement.json").is_file() else {}
        if not surfaces:
            self.skipTest("arena artifact not present")
        for record in self.raw["objects"]:
            x, y, z = record["pose"]["xyz"]
            on = [
                s for s in surfaces.values()
                if abs(x - s["center_xyz"][0]) <= s["size_xy"][0] / 2
                and abs(y - s["center_xyz"][1]) <= s["size_xy"][1] / 2
                and abs(z - s["center_xyz"][2]) < 0.05
            ]
            self.assertTrue(on, f"object {record['id']} rests on no declared surface")


if __name__ == "__main__":
    unittest.main()
