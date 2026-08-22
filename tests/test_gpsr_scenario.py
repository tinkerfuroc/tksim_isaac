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


    # ------------------------------------------------------------------ #
    # the person the command asks for
    #
    # This scenario's own dialogue commands "go to the kitchen table and
    # find a person" while shipping "actors": []. GPSR run12 drove to the
    # table and then scanned 3930 times for somebody who was not there --
    # `no matches for "person" via vlm_sam`, the correct answer to an empty
    # room. A fixture must be able to satisfy the task it commands.
    # ------------------------------------------------------------------ #
    def _person(self):
        people = [a for a in self.raw["actors"] if "person" in a["id"]]
        self.assertTrue(people, "scenario declares no person actor")
        return people[0]

    def test_the_commanded_task_has_a_person_to_find(self):
        commanded = " ".join(
            str(line["outcome"])
            for line in self.raw["dialogue"]
            if line.get("endpoint") == "listen_action"
        )
        if "person" not in commanded:
            self.skipTest("this scenario's command does not ask for a person")
        self.assertTrue(
            self.raw["actors"],
            f"command {commanded!r} asks for a person but actors is empty",
        )
        self._person()

    def test_person_uses_the_provenanced_mesh_not_the_capsule(self):
        """A detector has to be able to call it a person.

        simulation/assets/primitives/person.usda is a bare capsule (r=0.25,
        h=1.2); no open-vocabulary detector labels that a person, so a run
        against it can only ever fail or hallucinate.
        """
        uri = self._person()["asset_uri"]
        self.assertNotIn("primitives/person.usda", uri)
        self.assertIn("artifacts/people/", uri)
        self.assertTrue((ROOT / uri).is_file(), f"missing person asset: {uri}")

    @unittest.skipUnless((ARENA / "map.pgm").is_file(),
                         "rcw2026 arena artifact not present in this checkout")
    def test_person_stands_in_free_space(self):
        from tinker_sim_core.occupancy import OccupancyMap
        occupancy = OccupancyMap.from_pgm(
            ARENA / "map.pgm", resolution=0.05, origin_x=-5.05, origin_y=-6.0
        )
        x, y, _ = self._person()["pose"]["xyz"]
        self.assertTrue(
            occupancy.free_with_clearance(x, y, 0.30),
            f"person at ({x}, {y}) overlaps furniture or wall",
        )

    def test_person_stands_on_the_floor(self):
        """The imported figure's feet sit at its own origin."""
        self.assertAlmostEqual(self._person()["pose"]["xyz"][2], 0.0, places=3)

    @unittest.skipUnless((ARENA / "placement.json").is_file(),
                         "rcw2026 arena artifact not present in this checkout")
    def test_person_is_at_the_kitchen_table(self):
        """'find a person at the kitchen table' means near that table."""
        import math
        surfaces = {
            s["surface_id"]: s
            for s in json.loads((ARENA / "placement.json").read_text())["surfaces"]
        }
        table = surfaces["kitchen_table#top"]["center_xyz"]
        x, y, _ = self._person()["pose"]["xyz"]
        self.assertLess(
            math.dist((x, y), (table[0], table[1])), 1.5,
            "person is not within 1.5 m of the kitchen table",
        )


if __name__ == "__main__":
    unittest.main()
