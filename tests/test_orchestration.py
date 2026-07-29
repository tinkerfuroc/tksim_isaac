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


if __name__ == "__main__":
    unittest.main()
