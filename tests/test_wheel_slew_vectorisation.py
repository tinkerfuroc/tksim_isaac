"""The vectorised wheel slew must be exactly the per-wheel scalar slew.

`IsaacWholeRobotBackend.step` bounded every wheel transient with a Python loop
that, per wheel per step, pulled a tensor element out to a Python float
(`float(self._velocity_targets[0, index])`) and wrote a scalar back. Each of
those is a tensor<->scalar round trip, and the loop ran at 120 Hz: measured
2026-08-20 at 3.0 ms of a 13.2 ms physics step (23%), more than half the cost
of the PhysX solve itself.

The whole loop is one clamp:

    applied_next = applied + clip(target - applied, -max_delta, +max_delta)

This module pins that identity against the shipped scalar helper, because the
optimisation is only safe if it is bit-for-bit the same decision for every
wheel -- this code bounds real wheel acceleration, so "close enough" is not
acceptable. Tested with plain floats/numpy so it runs without Isaac or torch;
the runtime path applies the same expression to a torch tensor.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.backend import slew_velocity_target  # noqa: E402

BACKEND = ROOT / "simulation/tinker_sim_isaac/backend.py"


def _vectorised(applied, target, max_delta):
    """The expression the runtime uses, evaluated with numpy."""
    applied = np.asarray(applied, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    return applied + np.clip(target - applied, -max_delta, max_delta)


class WheelSlewVectorisationTest(unittest.TestCase):
    def test_matches_scalar_slew_exactly(self):
        max_delta = 0.25
        values = [-10.0, -3.3, -0.25, -0.1, 0.0, 0.1, 0.25, 1.0, 3.3, 10.0]
        for applied in values:
            for target in values:
                expected = slew_velocity_target(applied, target, max_delta)
                actual = float(_vectorised([applied], [target], max_delta)[0])
                self.assertAlmostEqual(
                    actual, expected, places=12,
                    msg=f"applied={applied} target={target} md={max_delta}",
                )

    def test_matches_across_many_random_cases(self):
        rng = np.random.default_rng(20260820)
        for _ in range(2000):
            applied = float(rng.uniform(-25.0, 25.0))
            target = float(rng.uniform(-25.0, 25.0))
            max_delta = float(rng.uniform(0.0, 2.0))
            expected = slew_velocity_target(applied, target, max_delta)
            actual = float(_vectorised([applied], [target], max_delta)[0])
            self.assertAlmostEqual(actual, expected, places=12)

    def test_zero_max_delta_holds_the_current_value(self):
        """max_delta 0 must freeze the wheel, never jump to target."""
        self.assertEqual(slew_velocity_target(2.0, 9.0, 0.0), 2.0)
        self.assertAlmostEqual(float(_vectorised([2.0], [9.0], 0.0)[0]), 2.0, places=12)

    def test_reaches_target_exactly_when_within_one_step(self):
        self.assertEqual(slew_velocity_target(1.0, 1.1, 0.25), 1.1)
        self.assertAlmostEqual(float(_vectorised([1.0], [1.1], 0.25)[0]), 1.1, places=12)

    def test_multiple_wheels_slew_independently(self):
        applied = [0.0, 5.0, -5.0, 1.0]
        target = [1.0, 0.0, 0.0, 1.05]
        max_delta = 0.5
        got = _vectorised(applied, target, max_delta)
        want = [slew_velocity_target(a, t, max_delta) for a, t in zip(applied, target)]
        for g, w in zip(got, want):
            self.assertAlmostEqual(float(g), w, places=12)

    def test_runtime_path_is_vectorised_not_a_python_loop(self):
        source = BACKEND.read_text(encoding="utf-8")
        start = source.index("def step(self) -> None:")
        nxt = source.find("\n    def ", start + 1)
        body = source[start : nxt if nxt != -1 else len(source)]
        self.assertNotIn(
            "for index in self._wheel_indices:", body,
            "step() must not pull wheel targets out one scalar at a time",
        )
        self.assertIn(
            "_slew_wheel_targets", body,
            "step() should delegate to the vectorised wheel slew helper",
        )

    def test_non_finite_inputs_are_still_rejected(self):
        """The scalar helper's finite guard must not be silently dropped."""
        for bad in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                slew_velocity_target(0.0, bad, 0.25)
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn(
            "isfinite", source,
            "the vectorised path must keep a finiteness check on wheel targets",
        )


if __name__ == "__main__":
    unittest.main()
