from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.orchestration import standard_operations
from tinker_sim_core.scenario import load_named_scenario


class ScenarioOrchestrationTest(unittest.TestCase):
    def test_every_scenario_compiles_to_standard_operations(self) -> None:
        for path in sorted((ROOT / "simulation/scenarios").glob("*.json")):
            scenario = load_named_scenario(ROOT, path.stem)
            operations = standard_operations(ROOT, scenario, 7)
            self.assertEqual(operations[0].kind, "reset_spawned")
            self.assertEqual(operations[-1].kind, "set_simulation_state")
            self.assertEqual(operations[1].kind, "set_simulation_state")
            self.assertEqual(operations[1].payload["state"], 0)
            self.assertEqual(operations[-1].payload["state"], 1)
            self.assertEqual(operations[-1].payload["boundary"], "PHYSICS_READY")
            spawn_indices = [
                index for index, operation in enumerate(operations)
                if operation.kind == "spawn_entity"
            ]
            self.assertTrue(all(1 < index < len(operations) - 1 for index in spawn_indices))
            self.assertNotIn("set_seed", {item.kind for item in operations})
            for operation in operations:
                if operation.kind == "spawn_entity":
                    self.assertEqual(operation.payload["entity_namespace"], "Scenario")
                    logical_id = operation.payload["logical_id"]
                    self.assertEqual(operation.payload["name"], f"/World/Scenario/{logical_id}")
                    self.assertEqual(operation.payload["prim_path"], f"/World/Scenario/{logical_id}")

    def test_seed_must_fit_uint64(self) -> None:
        scenario = load_named_scenario(ROOT, "find-and-approach-person")
        with self.assertRaises(ValueError):
            standard_operations(ROOT, scenario, -1)

    def test_spawn_while_playing_drops_the_stop_bracket(self) -> None:
        # A STOP -> PLAY cycle leaves subsequently spawned rigid bodies
        # unpaired with the robot's articulation links (they pass through
        # the gripper), so this mode must issue no reset_spawned and no
        # state-0 stop while keeping the spawns and the final state-1 op
        # (a no-op play when already playing) untouched.
        for path in sorted((ROOT / "simulation/scenarios").glob("*.json")):
            try:
                scenario = load_named_scenario(ROOT, path.stem)
            except ValueError:
                # Non-scenario data files (e.g. rcw2026-placements.json)
                # share the directory; the compile-all test above owns that
                # dispute.
                continue
            if scenario.world.get("uri"):
                continue
            operations = standard_operations(
                ROOT, scenario, 7, spawn_while_playing=True
            )
            kinds = [operation.kind for operation in operations]
            self.assertNotIn("reset_spawned", kinds)
            self.assertNotIn("load_world", kinds)
            states = [
                operation.payload["state"]
                for operation in operations
                if operation.kind == "set_simulation_state"
            ]
            self.assertEqual(states, [1])
            self.assertEqual(operations[-1].payload["boundary"], "PHYSICS_READY")
            baseline = standard_operations(ROOT, scenario, 7)
            self.assertEqual(
                [op.payload for op in operations if op.kind == "spawn_entity"],
                [op.payload for op in baseline if op.kind == "spawn_entity"],
            )

    def test_spawn_while_playing_refuses_world_loads(self) -> None:
        scenario = load_named_scenario(ROOT, "find-and-approach-person")
        if not scenario.world.get("uri"):
            self.skipTest("scenario has no world uri")
        with self.assertRaises(ValueError):
            standard_operations(ROOT, scenario, 7, spawn_while_playing=True)


if __name__ == "__main__":
    unittest.main()
