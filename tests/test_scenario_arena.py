from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.scenario import ScenarioDefinition, validate_world_selection


def _scenario(world: dict) -> ScenarioDefinition:
    return ScenarioDefinition(
        schema_version=2,
        scenario_id="test-scenario",
        world=world,
        robot={},
        actors=(),
        objects=(),
        regions=(),
        events=(),
        dialogue=(),
        postconditions=(),
    )


class ValidateWorldSelectionTest(unittest.TestCase):
    def test_current_mode_ignores_arena(self) -> None:
        scenario = _scenario({"mode": "current"})
        validate_world_selection(scenario, "rcw2026")

    def test_current_mode_without_arena(self) -> None:
        scenario = _scenario({"mode": "current"})
        validate_world_selection(scenario, None)

    def test_arena_mode_matching(self) -> None:
        scenario = _scenario({"mode": "arena", "arena": "rcw2026"})
        validate_world_selection(scenario, "rcw2026")

    def test_arena_mode_without_launcher_arena(self) -> None:
        scenario = _scenario({"mode": "arena", "arena": "rcw2026"})
        with self.assertRaises(ValueError):
            validate_world_selection(scenario, None)

    def test_arena_mode_mismatch(self) -> None:
        scenario = _scenario({"mode": "arena", "arena": "rcw2026"})
        with self.assertRaises(ValueError):
            validate_world_selection(scenario, "other")

    def test_unknown_mode_rejected(self) -> None:
        scenario = _scenario({"mode": "gazebo"})
        with self.assertRaises(ValueError):
            validate_world_selection(scenario, "rcw2026")

    def test_arena_mode_with_uri_rejected(self) -> None:
        scenario = _scenario({"mode": "arena", "arena": "x", "uri": "y"})
        with self.assertRaises(ValueError):
            validate_world_selection(scenario, "x")


if __name__ == "__main__":
    unittest.main()
