from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.control import EpisodeController


class EpisodeControllerTest(unittest.TestCase):
    def test_reset_does_not_rewind_public_time(self) -> None:
        control = EpisodeController(7)
        control.advance(3.0)
        control.reset(9)
        self.assertEqual(control.simulation_time, 3.0)
        self.assertEqual(control.status().episode_time, 0.0)
        self.assertEqual(control.status().episode_id, 1)

    def test_load_and_step_require_pause(self) -> None:
        control = EpisodeController()
        with self.assertRaises(RuntimeError):
            control.load_scenario("navigation-open", 3)
        control.pause(True)
        control.load_scenario("navigation-open", 3)
        control.step_paused(12, 1.0 / 120.0)
        self.assertAlmostEqual(control.simulation_time, 0.1)
        self.assertEqual(control.status().scenario, "navigation-open")


if __name__ == "__main__":
    unittest.main()
