"""The Kit/Python step rate must be separable from the PhysX solver rate.

Every per-step cost outside the solver -- the command-target writes,
`write_data_to_sim`, the robot-handle refresh, object-view discovery, the
gateway publish, Isaac Lab's tensor bookkeeping -- is paid once per
*control* step, while contact fidelity depends only on the *PhysX* step.
Today the two are one number (`physics_hz`), so the only way to buy
wall-clock was `TINKER_SIM_PHYSICS_HZ=60`, which also halves the contact
resolution every validated result was produced against.

omni.physx substeps natively: `IPhysxSimulation.simulate(elapsed)` runs
`elapsed * physxScene:timeStepsPerSecond` solver steps (bounded by
`/persistent/simulation/minFrameRate`). Keeping `timeStepsPerSecond` at
the validated 120 Hz while Isaac Lab steps at 1/60 therefore keeps the
solver behaviour and pays the wrapper overhead half as often.

The control rate is opt-in (`TINKER_SIM_CONTROL_HZ`); unset, it equals the
physics rate and the backend behaves exactly as before.
"""

from __future__ import annotations

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "simulation/tinker_sim_isaac/backend.py"


def _load():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "_physics_rate_probe", ROOT / "simulation/tinker_sim_isaac/physics_rate.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ControlRateOverrideTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load()

    def test_default_is_the_physics_rate(self):
        """Unset, one PhysX step per control step -- today's behaviour."""
        resolve = self.module.resolve_control_hz
        self.assertEqual(resolve(120.0, None), 120.0)
        self.assertEqual(resolve(120.0, ""), 120.0)
        self.assertEqual(resolve(120.0, "  "), 120.0)
        self.assertEqual(resolve(60.0, None), 60.0)

    def test_override_lowers_the_control_rate(self):
        resolve = self.module.resolve_control_hz
        self.assertEqual(resolve(120.0, "60"), 60.0)
        self.assertEqual(resolve(120.0, "40"), 40.0)
        self.assertEqual(resolve(120.0, "30.0"), 30.0)

    def test_override_may_not_exceed_the_physics_rate(self):
        """A control step shorter than the solver step has no meaning."""
        resolve = self.module.resolve_control_hz
        with self.assertRaises(ValueError):
            resolve(120.0, "240")
        with self.assertRaises(ValueError):
            resolve(60.0, "120")

    def test_override_must_divide_the_physics_rate_evenly(self):
        """PhysX can only run whole substeps per control step."""
        resolve = self.module.resolve_control_hz
        with self.assertRaises(ValueError):
            resolve(120.0, "50")
        with self.assertRaises(ValueError):
            resolve(120.0, "45")

    def test_override_has_a_floor(self):
        """Below ~30 Hz the command/clock cadence is too coarse for the stack."""
        resolve = self.module.resolve_control_hz
        with self.assertRaises(ValueError):
            resolve(120.0, "20")
        with self.assertRaises(ValueError):
            resolve(120.0, "10")

    def test_override_rejects_nonsense(self):
        resolve = self.module.resolve_control_hz
        for bad in ("0", "-60", "abc", "nan", "inf"):
            with self.assertRaises(ValueError, msg=f"accepted {bad!r}"):
                resolve(120.0, bad)

    def test_substeps_are_the_ratio(self):
        substeps = self.module.physics_substeps
        self.assertEqual(substeps(120.0, 120.0), 1)
        self.assertEqual(substeps(120.0, 60.0), 2)
        self.assertEqual(substeps(120.0, 40.0), 3)
        self.assertEqual(substeps(120.0, 30.0), 4)
        self.assertEqual(substeps(60.0, 30.0), 2)

    def test_composes_with_the_physics_rate_override(self):
        """Lowering both still yields whole substeps of the lowered solver rate."""
        physics_hz = self.module.resolve_physics_hz(120.0, "60")
        control_hz = self.module.resolve_control_hz(physics_hz, "30")
        self.assertEqual(self.module.physics_substeps(physics_hz, control_hz), 2)

    def test_backend_consults_the_resolver(self):
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn("resolve_control_hz", source)
        self.assertIn("TINKER_SIM_CONTROL_HZ", source)

    def test_backend_steps_kit_at_the_control_rate(self):
        """`self.dt` is the control step: it drives SimulationCfg, the wheel
        slew, `Articulation.update`, the gateway strides and /clock."""
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn("self.dt = 1.0 / control_hz", source)
        self.assertIn("self.physics_dt = 1.0 / physics_hz", source)

    def test_backend_keeps_physx_at_the_physics_rate(self):
        """Isaac Lab is configured with the solver dt and stepped
        physics_substeps times per control step. omni.physx's
        simulate(elapsed) does not substep (measured 2026-08-21), so the
        substeps must be explicit rather than a timeStepsPerSecond override."""
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn("SimulationCfg(\n                dt=self.physics_dt", source)
        self.assertIn("for index in range(self.physics_substeps):", source)
        self.assertNotIn("physxScene:timeStepsPerSecond", source)

    def test_clock_and_frame_index_follow_the_substeps(self):
        """/clock advances physics_dt per solver step; the published frame
        index advances once per control step so evaluators see it contiguous."""
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn("return float(max(0, steps)) * self.physics_dt", source)
        self.assertIn('// max(1, getattr(self, "physics_substeps", 1))', source)

    def test_physics_rate_default_is_unchanged(self):
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn("physics_hz: float = 120.0", source)


if __name__ == "__main__":
    unittest.main()
