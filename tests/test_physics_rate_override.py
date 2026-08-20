"""The physics rate must be tunable, and default to the validated 120 Hz.

Every per-step cost -- PhysX, the command-target writes, `write_data_to_sim`,
the robot-handle refresh, object-view discovery -- is paid `physics_hz` times
per simulated second, so the physics rate is the single largest multiplier on
wall-clock cost. It was hardcoded at 120.0 with no way to lower it.

Why it matters, measured live 2026-08-20 (domain 71, gpsr-rcw2026): with the
full GPSR stack attached the simulator runs at RTF ~0.09, and Nav2's
controller loop still ticks at ~24 Hz of *wall* time even with
`use_sim_time:=true`. The robot therefore advances ~0.2 mm per control cycle,
DWB's acceleration-limited velocity window stays anchored near the measured
(near-zero) velocity, and it commands ~0.05 m/s against its own 0.4 m/s limit
-- so navigation never converges and Nav2 falls into "Failed to make progress"
recovery spins.

Lowering the physics rate trades simulated-contact fidelity for wall-clock
speed, so it is opt-in and the default is unchanged: runs that validate
behaviour keep the 120 Hz they were validated at.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

BACKEND = ROOT / "simulation/tinker_sim_isaac/backend.py"


def _load():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "_backend_probe", ROOT / "simulation/tinker_sim_isaac/physics_rate.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PhysicsRateOverrideTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load()

    def test_default_is_the_validated_rate(self):
        resolve = self.module.resolve_physics_hz
        self.assertEqual(resolve(120.0, None), 120.0)
        self.assertEqual(resolve(120.0, ""), 120.0)
        self.assertEqual(resolve(120.0, "   "), 120.0)

    def test_override_lowers_the_rate(self):
        resolve = self.module.resolve_physics_hz
        self.assertEqual(resolve(120.0, "60"), 60.0)
        self.assertEqual(resolve(120.0, "30.0"), 30.0)

    def test_override_may_not_exceed_the_default(self):
        """Raising it silently would change validated contact behaviour."""
        resolve = self.module.resolve_physics_hz
        with self.assertRaises(ValueError):
            resolve(120.0, "240")

    def test_override_has_a_floor(self):
        """Below ~30 Hz PhysX contact resolution is not trustworthy."""
        resolve = self.module.resolve_physics_hz
        with self.assertRaises(ValueError):
            resolve(120.0, "10")

    def test_override_rejects_nonsense(self):
        resolve = self.module.resolve_physics_hz
        for bad in ("0", "-60", "abc", "nan", "inf"):
            with self.assertRaises(ValueError, msg=f"accepted {bad!r}"):
                resolve(120.0, bad)

    def test_backend_default_signature_is_unchanged(self):
        """A run that passes nothing must still get exactly 120 Hz."""
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn("physics_hz: float = 120.0", source)

    def test_backend_consults_the_resolver(self):
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn("resolve_physics_hz", source)
        self.assertIn("TINKER_SIM_PHYSICS_HZ", source)

    def test_halving_the_rate_halves_steps_per_simulated_second(self):
        """The whole point: per-step cost is paid physics_hz times a second."""
        resolve = self.module.resolve_physics_hz
        self.assertAlmostEqual(1.0 / resolve(120.0, None), 1.0 / 120.0)
        self.assertAlmostEqual(1.0 / resolve(120.0, "60"), 1.0 / 60.0)


if __name__ == "__main__":
    unittest.main()
