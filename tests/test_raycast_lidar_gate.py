"""Which source feeds ``/livox/lidar``.

The live PhysX raycast sensor is the right long-term source, but it does not
work yet: in a live navigation-parity run it reports ``is_valid``, returns a
full 19,893-ray reading, casts full-length 40 m rays from the correct world
origin -- and hits NOTHING. Not the ground plane its first ray descends into
~2.2 m ahead, not the robot it is bolted to. It therefore publishes
correctly-timed EMPTY clouds, which is strictly worse for navigation than the
occupancy-map fake it replaced: Nav2 gets no ``/scan``, AMCL never converges,
and no goal is ever accepted.

So it is OPT-IN until that is understood. These tests pin that default, because
getting it wrong ships a silently broken navigation stack.

``run_sim`` defers every Isaac import into ``main()``, so importing it here
needs no simulator.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "tools"))

import run_sim as rs  # noqa: E402


class RaycastLidarGateTest(unittest.TestCase):
    def test_raycast_is_off_by_default_in_every_profile(self) -> None:
        """The failure mode is a silent one, so the default must be the safe one."""
        for profile in (
            "navigation-parity",
            "sensor-rich",
            "manipulation-core",
            "physics-only",
        ):
            for qualification in (False, True):
                self.assertFalse(
                    rs.raycast_lidar_enabled(profile, qualification, False),
                    f"{profile} (qualification={qualification}) must not default "
                    "to the raycast lidar while it publishes empty clouds",
                )

    def test_opt_in_enables_it_where_a_lidar_is_published(self) -> None:
        self.assertTrue(rs.raycast_lidar_enabled("navigation-parity", False, True))
        self.assertTrue(rs.raycast_lidar_enabled("sensor-rich", False, True))

    def test_opt_in_still_respects_the_per_profile_predicate(self) -> None:
        """Opting in must not publish a lidar in a profile that has none."""
        self.assertFalse(rs.raycast_lidar_enabled("physics-only", False, True))
        self.assertFalse(rs.raycast_lidar_enabled("manipulation-core", False, True))
        self.assertTrue(rs.raycast_lidar_enabled("manipulation-core", True, True))

    def test_exactly_one_source_feeds_the_topic(self) -> None:
        """Both sources publishing to /livox/lidar at once would interleave."""
        for profile in ("navigation-parity", "sensor-rich"):
            self.assertTrue(rs.gateway_lidar_enabled(profile, False))
            self.assertFalse(
                rs.raycast_lidar_enabled(profile, False, False),
                "with the raycast source off, the occupancy lidar is the only one",
            )

    def test_the_occupancy_default_keeps_navigation_working(self) -> None:
        """navigation-parity must publish SOMETHING on /livox/lidar."""
        self.assertTrue(rs.gateway_lidar_enabled("navigation-parity", False))


if __name__ == "__main__":
    unittest.main()
