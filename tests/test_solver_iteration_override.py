"""The articulation solver iteration count must be tunable, default untouched.

The robot USD authors `physxArticulation:solverPositionIterationCount = 32`
(velocity 1). Every PhysX step pays those iterations for the whole
articulation, so the count is a direct multiplier on the ~5.4 ms CPU solve
measured per 120 Hz step (reports/gpsr-sim-2026-08-20/profile-best.log).
Lowering it trades joint-drive / contact convergence for speed, so it is
opt-in (`TINKER_SIM_SOLVER_POSITION_ITERATIONS`, `TINKER_SIM_SOLVER_VELOCITY_ITERATIONS`);
unset, the USD value is used unchanged.
"""

from __future__ import annotations

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "simulation/tinker_sim_isaac/backend.py"


def _load():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "_physics_rate_probe2", ROOT / "simulation/tinker_sim_isaac/physics_rate.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SolverIterationOverrideTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load()

    def test_unset_means_keep_the_usd_value(self):
        resolve = self.module.resolve_solver_iterations
        self.assertIsNone(resolve("position", None))
        self.assertIsNone(resolve("position", ""))
        self.assertIsNone(resolve("velocity", "  "))

    def test_override_is_a_positive_integer(self):
        resolve = self.module.resolve_solver_iterations
        self.assertEqual(resolve("position", "8"), 8)
        self.assertEqual(resolve("velocity", "1"), 1)
        self.assertEqual(resolve("position", "255"), 255)

    def test_override_rejects_nonsense(self):
        resolve = self.module.resolve_solver_iterations
        for bad in ("0", "-4", "4.5", "abc", "256", "nan"):
            with self.assertRaises(ValueError, msg=f"accepted {bad!r}"):
                resolve("position", bad)

    def test_backend_applies_them_through_articulation_props(self):
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn("TINKER_SIM_SOLVER_POSITION_ITERATIONS", source)
        self.assertIn("TINKER_SIM_SOLVER_VELOCITY_ITERATIONS", source)
        self.assertIn("ArticulationRootPropertiesCfg", source)
        self.assertIn("solver_position_iteration_count", source)
        self.assertIn("solver_velocity_iteration_count", source)


if __name__ == "__main__":
    unittest.main()
