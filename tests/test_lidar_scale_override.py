"""Trading lidar ray count for RTF, deliberately.

The full Mid-360 pattern is 57 x 349 = 19,893 rays, and measured live in
navigation-parity at 60 Hz physics / 30 Hz control it costs about 0.99 s of
wall clock per simulated second:

    occupancy lidar (no raycast)   RTF 0.927   -> 1.079 s/sim-s
    raycast at 19,893 rays         RTF 0.483   -> 2.070 s/sim-s

So the sensor, not the simulator, is the budget. An RTF floor of 0.5 leaves
the lidar 0.92 s/sim-s, which full scale overruns. Ray count is the only lever
that moves it: max_range and min_range were both measured to be worth <4%,
because the cost is TESTING each ray against the robot's ~200 convex hulls,
not hitting anything.

This mirrors ``resolve_physics_hz``: the contract file stays the hardware
truth, and a run may deliberately lower the scale. Raising it is refused --
the contract is what the hardware does, and a run silently claiming MORE than
Mid-360 scale would invalidate every parity claim made against it.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.lidar_rig import (  # noqa: E402
    DEFAULT_LIDAR_CHANNELS,
    DEFAULT_LIDAR_COLUMNS,
    MINIMUM_LIDAR_CHANNELS,
    MINIMUM_LIDAR_COLUMNS,
    resolve_lidar_scale,
)


class ResolveLidarScaleTest(unittest.TestCase):
    def test_unset_returns_the_validated_default(self) -> None:
        """Unset is the DEFAULT, which is no longer the contract.

        The live sensor is on by default now, so an unset override has to
        resolve to something affordable with the whole stack attached. That is
        the 44 x 349 the live Nav2 battery actually passed at RTF 0.521, not
        the contract's 57 channels, which was never measured with Nav2 on the
        box and left no margin bare-sim.
        """
        self.assertEqual(
            resolve_lidar_scale(57, 349, None, None),
            (DEFAULT_LIDAR_CHANNELS, DEFAULT_LIDAR_COLUMNS),
        )
        self.assertEqual(
            resolve_lidar_scale(57, 349, "", "  "),
            (DEFAULT_LIDAR_CHANNELS, DEFAULT_LIDAR_COLUMNS),
        )

    def test_the_default_never_exceeds_a_smaller_contract(self) -> None:
        """A contract below the default must still cap it.

        The default is a budget, not a licence: if hardware-parity.json ever
        declares fewer channels than the default budget, the contract wins.
        """
        self.assertEqual(resolve_lidar_scale(16, 120, None, None), (16, 120))

    def test_lowering_is_allowed(self) -> None:
        self.assertEqual(
            resolve_lidar_scale(57, 349, "32", None),
            (32, DEFAULT_LIDAR_COLUMNS),
        )
        self.assertEqual(
            resolve_lidar_scale(57, 349, None, "180"),
            (DEFAULT_LIDAR_CHANNELS, 180),
        )
        self.assertEqual(resolve_lidar_scale(57, 349, "32", "180"), (32, 180))

    def test_raising_from_the_default_up_to_the_contract_is_allowed(self) -> None:
        """How a parity run asks for the full fan back.

        The refusal is about the CONTRACT ceiling, not the default budget --
        otherwise flipping the default would have quietly made full-resolution
        parity runs impossible.
        """
        self.assertEqual(resolve_lidar_scale(57, 349, "57", None), (57, 349))
        self.assertGreater(57, DEFAULT_LIDAR_CHANNELS)

    def test_raising_is_refused(self) -> None:
        """A run must never claim more than the hardware contract."""
        with self.assertRaises(ValueError) as ctx:
            resolve_lidar_scale(57, 349, "64", None)
        self.assertIn("exceeds", str(ctx.exception))
        with self.assertRaises(ValueError):
            resolve_lidar_scale(57, 349, None, "400")

    def test_equal_to_the_contract_is_fine(self) -> None:
        self.assertEqual(resolve_lidar_scale(57, 349, "57", "349"), (57, 349))

    def test_floors_are_enforced(self) -> None:
        """Below the floor the scan stops being able to see a person."""
        with self.assertRaises(ValueError) as ctx:
            resolve_lidar_scale(57, 349, str(MINIMUM_LIDAR_CHANNELS - 1), None)
        self.assertIn("floor", str(ctx.exception))
        with self.assertRaises(ValueError):
            resolve_lidar_scale(57, 349, None, str(MINIMUM_LIDAR_COLUMNS - 1))

    def test_non_numeric_and_nonsense_are_refused(self) -> None:
        for bad in ("abc", "12.5", "-4", "0"):
            with self.assertRaises(ValueError):
                resolve_lidar_scale(57, 349, bad, None)

    def test_the_floors_still_see_a_dynamic_obstacle(self) -> None:
        """The floor must not be so coarse that a person can hide between rays.

        Azimuth spacing at the column floor, checked against a 0.4 m-wide
        person at 4 m: the arc between adjacent rays must stay well under the
        target width or an obstacle can fall between two beams.
        """
        import math

        spacing_rad = 2.0 * math.pi / MINIMUM_LIDAR_COLUMNS
        arc_at_4m = spacing_rad * 4.0
        self.assertLess(
            arc_at_4m,
            0.4,
            "at the column floor a 0.4 m person at 4 m could fall between rays",
        )


if __name__ == "__main__":
    unittest.main()
