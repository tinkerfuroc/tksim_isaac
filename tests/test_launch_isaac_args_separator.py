"""``tinker-sim launch ... -- --flag`` must actually deliver ``--flag``.

The wrapper collects everything after the launch options into an
``argparse.REMAINDER`` positional and forwards it to ``run_sim.py``. Python's
argparse puts the literal ``--`` separator INTO that remainder as its first
element, so a naive forward sends ``run_sim.py`` a spurious leading ``--``.

``run_sim.py`` parses with ``parse_known_args()``, and argparse's own ``--``
means "everything after this is positional". The forwarded flag is therefore
demoted to an unknown positional and its ``store_true`` destination keeps its
default. Reproduced exactly:

    parse_known_args(["--", "--raycast-lidar"])  -> raycast_lidar=False
    parse_known_args(["--raycast-lidar"])        -> raycast_lidar=True

(Historical note: the examples here show the bug as it was found, when
``--raycast-lidar`` was the opt-IN. The flags have since traded places -- the
live sensor is the default and ``--map-lidar`` is the opt-out -- so these tests
now use ``--map-lidar`` as the fixture. A dropped ``--raycast-lidar`` would no
longer be observable, and the test would pass while pinning nothing.)

So ``tinker-sim launch ... -- --raycast-lidar`` silently ran the OCCUPANCY
lidar: ``/sim/status/isaac`` reported ``lidar_source: "occupancy"`` and
``lidar: null``, with no error and no warning. This is the same class of
silent failure as the scene-query default it was being used to investigate --
the run looks healthy and measures the wrong thing -- and it costs a GPU run
every time it happens.

The separator is required to get flags past the wrapper's own parser at all,
so it must be stripped on the way out rather than banned on the way in.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.cli import _parser as build_parser  # noqa: E402
from tinker_sim_deploy.cli import _launch_command  # noqa: E402


BASE = [
    "launch",
    "--sensor-profile",
    "navigation-parity",
    "--profile",
    "parity",
    "--scenario",
    "gpsr-rcw2026",
    "--seed",
    "0",
    "--headless",
]


def _command(extra: list[str]) -> list[str]:
    args = build_parser().parse_args(BASE + extra)
    return _launch_command(args)


class LaunchSeparatorTest(unittest.TestCase):
    def test_separator_form_delivers_the_flag(self) -> None:
        """The documented way to pass a run_sim-only flag must work."""
        command = _command(["--", "--map-lidar"])
        self.assertIn("--map-lidar", command)

    def test_separator_token_is_not_forwarded(self) -> None:
        """A forwarded '--' silently demotes every flag after it."""
        command = _command(["--", "--map-lidar"])
        self.assertNotIn(
            "--",
            command,
            "the '--' separator must be stripped before forwarding: run_sim "
            "parses with parse_known_args, where a leading '--' turns every "
            "following flag into an ignored positional",
        )

    def test_the_forwarded_flag_actually_parses_in_run_sim(self) -> None:
        """End to end: what the wrapper emits must set the destination.

        The two tests above pin the mechanism; this one pins the OUTCOME, so
        the contract survives a change in how the stripping is done.
        """
        sys.path.insert(0, str(ROOT / "validation"))
        import argparse

        command = _command(["--", "--map-lidar"])
        # Everything from run_sim.py onward is what run_sim's parser sees.
        index = next(
            i for i, token in enumerate(command) if token.endswith("run_sim.py")
        )
        forwarded = command[index + 1 :]

        parser = argparse.ArgumentParser()
        parser.add_argument("--map-lidar", action="store_true")
        parsed, _unknown = parser.parse_known_args(forwarded)
        self.assertTrue(
            parsed.map_lidar,
            f"run_sim would receive {forwarded!r} and leave map_lidar "
            "False, silently running the LIVE sensor when the run asked for "
            "the cheap occupancy one",
        )

    def test_multiple_trailing_flags_all_survive(self) -> None:
        command = _command(["--", "--map-lidar", "--camera-pointcloud"])
        self.assertIn("--map-lidar", command)
        self.assertIn("--camera-pointcloud", command)

    def test_no_extra_args_is_unchanged(self) -> None:
        """Stripping must not disturb the ordinary no-passthrough case."""
        command = _command([])
        self.assertNotIn("--", command)
        self.assertIn("--headless", command)


if __name__ == "__main__":
    unittest.main()
