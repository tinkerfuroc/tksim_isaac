"""Which source feeds ``/livox/lidar``.

The live PhysX raycast sensor is the right long-term source: it casts against
the physics scene, so it sees spawned objects and the person capsule, which the
occupancy-map raycast structurally cannot.

It published EMPTY clouds until the scene-query fix, and the cause was never the
sensor. IsaacLab's ``SimulationCfg.enable_scene_query_support`` defaults to
False, which stops PhysX building a scene query manager at all, so every ray
missed while the sensor still reported ``is_valid`` and returned a full-length
reading. See ``tests/test_raycast_scene_query_contract.py``, which pins the
wiring that turns it on.

It stays OPT-IN here pending a live RTF measurement -- scene queries plus 19,893
rays cost PhysX time, and whether that fits the sim's budget is a separate
question from whether it works. These tests pin the default so flipping it stays
a deliberate decision rather than a drive-by.

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
