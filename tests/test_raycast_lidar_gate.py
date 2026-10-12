"""Which source feeds ``/livox/lidar``.

The live PhysX raycast sensor is now the DEFAULT, and the occupancy-map
raycast is the opt-out (``--map-lidar``). The two flags traded places on
2026-09-11; ``--raycast-lidar`` survives as an accepted no-op so existing
scripts keep working.

Why the default moved. The occupancy raycast reads the arena PGM, so it can
only ever report what ``map.yaml`` already says: no spawned object, no person,
no moved furniture. A navigation run against it is largely validating Nav2
against Nav2's own ``static_layer``. The live sensor casts against the physics
scene and sees all of that -- measured, a person spawned 1.5 m ahead adds +324
points in 6 of 6 frames and deleting it returns the count to exact baseline.

It was held opt-in for one reason only: it published EMPTY clouds, and the
cause was never the sensor. IsaacLab's
``SimulationCfg.enable_scene_query_support`` defaults to False, which stops
PhysX building a scene query manager at all, so every ray missed while the
sensor still reported ``is_valid`` and returned a full-length reading. See
``tests/test_raycast_scene_query_contract.py``, which pins the wiring that
turns it on. That is fixed, and the sensor has since passed a live Nav2
battery including a completed ``navigate_to_pose``.

The cost of being on by default is paid in ray budget, not in physics rate:
see ``test_lidar_scale_override.py`` and ``DEFAULT_LIDAR_CHANNELS``. Lowering
the physics rate was rejected as the lever because it would change contact
behaviour for every manipulation result, not just the lidar.

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
    def test_raycast_is_on_by_default_where_a_lidar_is_published(self) -> None:
        """The default is now the source that can actually see the world."""
        for profile in ("navigation-parity", "sensor-rich"):
            self.assertTrue(
                rs.raycast_lidar_enabled(profile, False, False),
                f"{profile} must default to the live raycast lidar; the "
                "occupancy raycast cannot see a spawned object or a person",
            )

    def test_map_lidar_opts_out(self) -> None:
        """``--map-lidar`` is the escape hatch back to the occupancy raycast."""
        for profile in ("navigation-parity", "sensor-rich"):
            self.assertFalse(
                rs.raycast_lidar_enabled(profile, False, True),
                f"--map-lidar must return {profile} to the occupancy source",
            )

    def test_the_default_still_respects_the_per_profile_predicate(self) -> None:
        """Defaulting on must not conjure a lidar in a profile that has none."""
        self.assertFalse(rs.raycast_lidar_enabled("physics-only", False, False))
        self.assertFalse(
            rs.raycast_lidar_enabled("manipulation-core", False, False)
        )
        self.assertTrue(
            rs.raycast_lidar_enabled("manipulation-core", True, False),
            "manipulation-core qualification publishes a lidar, so it gets the "
            "live one like every other lidar-publishing profile",
        )

    def test_opting_out_where_there_is_no_lidar_is_still_no_lidar(self) -> None:
        for profile in ("physics-only", "manipulation-core"):
            self.assertFalse(rs.raycast_lidar_enabled(profile, False, True))

    def test_exactly_one_source_feeds_the_topic(self) -> None:
        """Both sources publishing to /livox/lidar at once would interleave.

        The gateway predicate says a lidar is published at all; the raycast
        predicate says WHICH source. They must never both be live, so the
        occupancy source is exactly the complement of the raycast one within
        the profiles that publish anything.
        """
        for profile in ("navigation-parity", "sensor-rich"):
            self.assertTrue(rs.gateway_lidar_enabled(profile, False))
            self.assertTrue(
                rs.raycast_lidar_enabled(profile, False, False),
                "default: raycast source live, occupancy source idle",
            )
            self.assertFalse(
                rs.raycast_lidar_enabled(profile, False, True),
                "opted out: occupancy source live, raycast source idle",
            )

    def test_navigation_parity_always_publishes_something(self) -> None:
        """navigation-parity must publish on /livox/lidar either way."""
        self.assertTrue(rs.gateway_lidar_enabled("navigation-parity", False))
        for map_lidar in (False, True):
            self.assertTrue(
                rs.gateway_lidar_enabled("navigation-parity", False),
                "a navigation profile with no lidar at all is never correct",
            )

    def test_raycast_lidar_flag_is_a_no_op(self) -> None:
        """The old opt-in must not become an accidental opt-OUT.

        ``--raycast-lidar`` and ``--map-lidar`` swapped roles. A script still
        passing the old flag has to keep getting the live sensor, so the flag
        is accepted and ignored rather than rewired.
        """
        source = (ROOT / "validation/run_sim.py").read_text(encoding="utf-8")
        self.assertIn("--raycast-lidar", source, "flag must stay accepted")
        self.assertNotIn(
            "args.raycast_lidar",
            source,
            "--raycast-lidar must be a pure no-op: reading it again risks "
            "reviving the old sense, in which the old opt-in flag would now "
            "select the occupancy lidar",
        )


if __name__ == "__main__":
    unittest.main()
