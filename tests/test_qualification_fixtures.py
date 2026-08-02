from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT))

from tinker_sim_core.orchestration import standard_operations  # noqa: E402
from tinker_sim_core.scenario import load_named_scenario  # noqa: E402
from validation.manipulation_qualification import QualificationRunner  # noqa: E402
from validation.run_sim import _expected_scenario_objects  # noqa: E402


class QualificationFixtureTest(unittest.TestCase):
    GATES = ("obstructed-gripper", "retention")
    CUBE_ROOT_Z = 0.60
    CUBE_SIZE = 0.08
    CUBE_LOCAL_CENTER_Z = 0.04
    PEDESTAL_TOP_Z = 0.60

    def test_pedestal_is_static_and_cube_is_separate_dynamic_object(self) -> None:
        pedestal_asset = (
            ROOT / "simulation/assets/primitives/qualification-pedestal.usda"
        ).read_text(encoding="utf-8")
        self.assertIn('prepend apiSchemas = ["PhysicsCollisionAPI"]', pedestal_asset)
        self.assertNotIn("PhysicsRigidBodyAPI", pedestal_asset)
        self.assertIn("xformOp:scale = (0.12, 0.12, 0.60)", pedestal_asset)
        self.assertIn("xformOp:translate = (0, 0, 0.30)", pedestal_asset)
        self.assertIn(
            'xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
            pedestal_asset,
        )

        for gate in self.GATES:
            scenario = load_named_scenario(ROOT, f"qualification-{gate}")
            actors = {str(record["id"]): record for record in scenario.actors}
            records = {str(record["id"]): record for record in scenario.objects}
            self.assertEqual(set(actors), {"qualification_pedestal"})
            self.assertEqual(set(records), {"qualification_cube"})

            pedestal = actors["qualification_pedestal"]
            self.assertEqual(pedestal["class_name"], "static_pedestal")
            self.assertTrue(pedestal["fixed"])
            self.assertEqual(
                pedestal["asset_uri"],
                "simulation/assets/primitives/qualification-pedestal.usda",
            )
            self.assertEqual(pedestal["pose"]["xyz"], [0.65, 0.0, 0.0])

            cube = records["qualification_cube"]
            self.assertEqual(cube["class_name"], "dynamic_cube")
            self.assertNotIn("fixed", cube)
            self.assertTrue(cube["gravity"])
            self.assertTrue(cube["collision"])
            self.assertEqual(cube["asset_uri"], "simulation/assets/primitives/task-object.usda")
            self.assertEqual(cube["pose"]["xyz"], [0.65, 0.0, self.CUBE_ROOT_Z])

            cube_bottom = cube["pose"]["xyz"][2] + self.CUBE_LOCAL_CENTER_Z - self.CUBE_SIZE / 2
            self.assertAlmostEqual(cube_bottom, self.PEDESTAL_TOP_Z)
            self.assertAlmostEqual(
                cube["pose"]["xyz"][2] + self.CUBE_LOCAL_CENTER_Z,
                0.64,
            )

    def test_spawn_contract_and_expected_object_provenance_include_pedestal(self) -> None:
        for gate in self.GATES:
            scenario = load_named_scenario(ROOT, f"qualification-{gate}")
            operations = standard_operations(ROOT, scenario, 7)
            spawns = {
                operation.payload["logical_id"]: operation.payload
                for operation in operations
                if operation.kind == "spawn_entity"
            }
            self.assertEqual(set(spawns), {"qualification_pedestal", "qualification_cube"})
            self.assertNotEqual(
                spawns["qualification_pedestal"]["prim_path"],
                spawns["qualification_cube"]["prim_path"],
            )
            self.assertTrue(spawns["qualification_pedestal"]["uri"].endswith("qualification-pedestal.usda"))
            self.assertTrue(spawns["qualification_cube"]["uri"].endswith("task-object.usda"))

            expected = _expected_scenario_objects(ROOT, f"qualification-{gate}")
            self.assertEqual(set(expected), {"qualification_cube"})
            self.assertEqual(expected["qualification_cube"]["class_name"], "dynamic_cube")
            self.assertEqual(
                expected["qualification_cube"]["pose"]["position"],
                [0.65, 0.0, self.CUBE_ROOT_Z],
            )

            runner = object.__new__(QualificationRunner)
            runner.scenario_path = ROOT / "simulation/scenarios" / f"qualification-{gate}.json"
            spec = runner._scenario_spec()
            self.assertEqual(
                spec["entity_ids"], ["qualification_pedestal", "qualification_cube"]
            )
            self.assertEqual(spec["object_ids"], ["qualification_cube"])

    def test_scenario_json_keeps_qualification_cube_as_verifier_target(self) -> None:
        for gate in self.GATES:
            path = ROOT / "simulation/scenarios" / f"qualification-{gate}.json"
            data = json.loads(path.read_text(encoding="utf-8"))
            cube_ids = [record["id"] for record in data["objects"] if record["class_name"] == "dynamic_cube"]
            self.assertEqual(cube_ids, ["qualification_cube"])
            pedestal_ids = [record["id"] for record in data["actors"] if record["class_name"] == "static_pedestal"]
            self.assertEqual(pedestal_ids, ["qualification_pedestal"])

    # ------------------------------------------------------------------
    # Task 1: integrated pick-place scenarios keep the physical spawn records
    # and the current primitive/mesh planning-scene representation.
    # ------------------------------------------------------------------
    PICK_PLACE_SCENARIOS = (
        "qualification-pick-place-positive",
        "qualification-pick-place-blocked-approach",
        "qualification-pick-place-unreachable-grasp",
        "qualification-pick-place-malformed-back",
        "qualification-pick-place-cancel-approach",
        "qualification-pick-place-cancel-transport",
        "qualification-pick-place-safety-transport",
        "qualification-pick-place-occupied-place",
    )

    def test_pick_place_positive_spawns_pedestal_and_cube(self) -> None:
        scenario = load_named_scenario(ROOT, "qualification-pick-place-positive")
        actors = {str(record["id"]): record for record in scenario.actors}
        records = {str(record["id"]): record for record in scenario.objects}
        # F2: the place-support pedestal is a declared static actor in every
        # E-stage scenario.
        self.assertEqual(
            set(actors), {"qualification_pedestal", "qualification_place_pedestal"}
        )
        self.assertEqual(set(records), {"qualification_cube"})

        pedestal = actors["qualification_pedestal"]
        self.assertEqual(pedestal["class_name"], "static_pedestal")
        self.assertTrue(pedestal["fixed"])
        self.assertEqual(
            pedestal["asset_uri"],
            "simulation/assets/primitives/qualification-pedestal.usda",
        )
        self.assertEqual(pedestal["role"], "support")
        self.assertEqual(pedestal["region"], "source-region")
        self.assertEqual(pedestal["planning_scene_id"], "sim_fixture/pedestal")
        self.assertEqual(pedestal["pose"]["xyz"], [0.65, 0.0, 0.0])

        place_pedestal = actors["qualification_place_pedestal"]
        self.assertEqual(place_pedestal["class_name"], "static_pedestal")
        self.assertTrue(place_pedestal["fixed"])
        self.assertEqual(
            place_pedestal["asset_uri"],
            "simulation/assets/primitives/qualification-pedestal.usda",
        )
        self.assertEqual(place_pedestal["role"], "support")
        self.assertEqual(place_pedestal["region"], "place-region")
        self.assertEqual(place_pedestal["planning_scene_id"], "sim_fixture/place_pedestal")
        self.assertEqual(place_pedestal["pose"]["xyz"], [0.85, 0.0, 0.0])

        cube = records["qualification_cube"]
        self.assertEqual(cube["class_name"], "dynamic_cube")
        self.assertTrue(cube["gravity"])
        self.assertTrue(cube["collision"])
        self.assertEqual(cube["asset_uri"], "simulation/assets/primitives/task-object.usda")
        self.assertEqual(cube["role"], "pick-target")
        self.assertEqual(cube["planning_scene_id"], "sim_fixture/qualification_cube")
        # F1: physical root is bottom-origin; the PlanningScene center is the
        # cube at z 0.64 (root + half-extent 0.04).
        self.assertEqual(cube["pose"]["xyz"], [0.65, 0.0, self.CUBE_ROOT_Z])
        ps_cube = next(
            obj for obj in scenario.planning_scene["objects"]
            if obj["id"] == "sim_fixture/qualification_cube"
        )
        self.assertEqual(ps_cube["pose"]["xyz"], [0.65, 0.0, 0.64])

        operations = standard_operations(ROOT, scenario, 7)
        spawns = {
            operation.payload["logical_id"]: operation.payload
            for operation in operations
            if operation.kind == "spawn_entity"
        }
        self.assertEqual(
            set(spawns),
            {"qualification_pedestal", "qualification_place_pedestal", "qualification_cube"},
        )
        final_state = operations[-1].payload
        self.assertEqual(final_state["state"], 1)
        self.assertEqual(final_state["boundary"], "PHYSICS_READY")
        self.assertEqual(final_state["scenario"]["id"], "qualification-pick-place-positive")
        self.assertEqual(final_state["scenario"]["seed"], 7)
        self.assertEqual(final_state["planning_scene"]["revision"], "qualification-v1")
        self.assertEqual(final_state["integrated"]["stage"], "E")

    def test_pick_place_scenarios_use_primitive_mesh_planning_scene(self) -> None:
        for name in self.PICK_PLACE_SCENARIOS:
            path = ROOT / "simulation/scenarios" / f"{name}.json"
            data = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(data["schema_version"], 2)
            self.assertIn("integrated", data)
            planning_scene = data["planning_scene"]
            self.assertEqual(planning_scene["frame_id"], "base_link")
            self.assertEqual(
                planning_scene["target_handoff"], "pick_and_place/object_mesh"
            )
            for record in planning_scene["objects"]:
                self.assertIn("primitive", record)
                self.assertNotIn("mesh", record)
                self.assertIn("pose", record)
                self.assertEqual(len(record["pose"]["xyz"]), 3)
                self.assertEqual(len(record["pose"]["quaternion_xyzw"]), 4)

    def test_pick_place_blocked_and_occupied_add_only_their_obstacle_or_occupant(
        self,
    ) -> None:
        positive = json.loads(
            (ROOT / "simulation/scenarios/qualification-pick-place-positive.json").read_text(
                encoding="utf-8"
            )
        )
        blocked = json.loads(
            (ROOT / "simulation/scenarios/qualification-pick-place-blocked-approach.json").read_text(
                encoding="utf-8"
            )
        )
        occupied = json.loads(
            (ROOT / "simulation/scenarios/qualification-pick-place-occupied-place.json").read_text(
                encoding="utf-8"
            )
        )
        positive_ids = [obj["id"] for obj in positive["planning_scene"]["objects"]]
        blocked_ids = [obj["id"] for obj in blocked["planning_scene"]["objects"]]
        occupied_ids = [obj["id"] for obj in occupied["planning_scene"]["objects"]]
        # F1/F2: the declared order is pedestal, cube, place_pedestal, then the
        # scenario-specific obstacle/occupant.
        self.assertEqual(
            positive_ids,
            [
                "sim_fixture/pedestal",
                "sim_fixture/qualification_cube",
                "sim_fixture/place_pedestal",
            ],
        )
        self.assertEqual(
            blocked_ids,
            [
                "sim_fixture/pedestal",
                "sim_fixture/qualification_cube",
                "sim_fixture/place_pedestal",
                "sim_fixture/plan_blocker",
            ],
        )
        self.assertEqual(
            occupied_ids,
            [
                "sim_fixture/pedestal",
                "sim_fixture/qualification_cube",
                "sim_fixture/place_pedestal",
                "sim_fixture/place_occupant",
            ],
        )
        # F3: the blocker physically rests at root z 0.70 with its PlanningScene
        # center at 0.85 (0.30 m cube half-height), covering the target TCP.
        blocker_ps = next(
            obj for obj in blocked["planning_scene"]["objects"]
            if obj["id"] == "sim_fixture/plan_blocker"
        )
        self.assertEqual(blocker_ps["pose"]["xyz"], [0.65, 0.0, 0.85])
        blocker_obj = next(
            obj for obj in blocked["objects"]
            if obj["id"] == "qualification_plan_blocker"
        )
        self.assertEqual(blocker_obj["pose"]["xyz"], [0.65, 0.0, 0.70])
        # F1: the occupant rests on the place support; its PS center is 0.64.
        occupant_ps = next(
            obj for obj in occupied["planning_scene"]["objects"]
            if obj["id"] == "sim_fixture/place_occupant"
        )
        self.assertEqual(occupant_ps["pose"]["xyz"], [0.85, 0.0, 0.64])
        occupant_obj = next(
            obj for obj in occupied["objects"]
            if obj["id"] == "qualification_place_occupant"
        )
        self.assertEqual(occupant_obj["pose"]["xyz"], [0.85, 0.0, 0.60])


if __name__ == "__main__":
    unittest.main()
