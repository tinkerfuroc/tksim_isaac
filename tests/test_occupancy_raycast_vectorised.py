"""`raycast_many` must reproduce the scalar `raycast` exactly.

The development lidar casts 181 rays per frame through `OccupancyMap.raycast`,
a Python loop marching 0.025 m at a time up to 40 m. Measured 2026-08-21 in
the sensor-rich loop (outputs/bench/sr-base.log, publish_breakdown_ms) that
is ~35 ms per lidar frame -- ~350 ms of every simulated second, the single
largest per-step publish cost, and independent of the physics rate.

The vectorised path is only acceptable if it is result-neutral: AMCL parity
evidence was produced against the scalar ray-cast, so every returned
distance must be bit-identical, including the `inf` miss and the
out-of-bounds-is-occupied rule.
"""

from __future__ import annotations

import math
import random
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.occupancy import OccupancyMap  # noqa: E402


def _random_map(rng: random.Random, width: int, height: int, density: float) -> OccupancyMap:
    rows = tuple(
        tuple(rng.random() < density for _ in range(width)) for _ in range(height)
    )
    return OccupancyMap(
        width=width,
        height=height,
        resolution=rng.choice((0.05, 0.1, 0.025, 0.07)),
        origin_x=rng.uniform(-10.0, 1.0),
        origin_y=rng.uniform(-10.0, 1.0),
        occupied=rows,
    )


class VectorisedRaycastTest(unittest.TestCase):
    def test_matches_scalar_raycast_bit_for_bit(self):
        rng = random.Random(20260821)
        for _ in range(40):
            grid = _random_map(rng, rng.randint(5, 60), rng.randint(5, 60), rng.choice((0.0, 0.01, 0.05, 0.2)))
            span_x = grid.width * grid.resolution
            span_y = grid.height * grid.resolution
            x = grid.origin_x + rng.uniform(-1.0, span_x + 1.0)
            y = grid.origin_y + rng.uniform(-1.0, span_y + 1.0)
            angles = [rng.uniform(-math.pi, math.pi) for _ in range(25)]
            # the gateway's exact call shape as well
            yaw = rng.uniform(-math.pi, math.pi)
            angles += [yaw + math.radians(d) for d in range(-90, 91, 9)]
            expected = [grid.raycast(x, y, a) for a in angles]
            actual = grid.raycast_many(x, y, angles)
            self.assertEqual(len(actual), len(expected))
            for e, a, ang in zip(expected, actual, angles):
                self.assertIsInstance(a, float)
                if math.isinf(e):
                    self.assertTrue(math.isinf(a), f"angle {ang}: expected miss, got {a}")
                else:
                    self.assertEqual(a, e, f"angle {ang}: {a!r} != {e!r}")

    def test_custom_limits_match_scalar(self):
        rng = random.Random(7)
        grid = _random_map(rng, 40, 30, 0.03)
        x = grid.origin_x + 1.0
        y = grid.origin_y + 1.0
        angles = [rng.uniform(-math.pi, math.pi) for _ in range(50)]
        for minimum, maximum in ((0.0, 5.0), (0.3, 40.0), (1.0, 2.0), (0.3, 0.3)):
            expected = [grid.raycast(x, y, a, minimum=minimum, maximum=maximum) for a in angles]
            actual = grid.raycast_many(x, y, angles, minimum=minimum, maximum=maximum)
            self.assertEqual(actual, expected)

    def test_empty_angles(self):
        grid = _random_map(random.Random(1), 5, 5, 0.0)
        self.assertEqual(grid.raycast_many(0.0, 0.0, []), [])


if __name__ == "__main__":
    unittest.main()
