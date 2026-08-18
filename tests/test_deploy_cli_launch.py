"""Deploy-CLI launch argument forwarding.

Regression tests for the argparse abbreviation trap: the launch subparser
defines ``--arena-colors``, and before ``--arena`` existed here, argparse's
default prefix matching silently rewrote ``--arena rcw2026`` into
``--arena-colors`` plus a stray Kit argument instead of failing loudly.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.cli import _launch_command, _parser  # noqa: E402


class LaunchArenaForwardingTest(unittest.TestCase):
    def test_arena_flag_parses_and_forwards(self) -> None:
        args = _parser().parse_args(["launch", "--arena", "rcw2026"])
        self.assertEqual(args.arena, "rcw2026")
        self.assertFalse(args.arena_colors)
        self.assertEqual(args.isaac_args, [])
        command = _launch_command(args)
        index = command.index("--arena")
        self.assertEqual(command[index + 1], "rcw2026")
        self.assertNotIn("--arena-colors", command)

    def test_arena_absent_not_forwarded(self) -> None:
        args = _parser().parse_args(["launch"])
        self.assertIsNone(args.arena)
        self.assertNotIn("--arena", _launch_command(args))

    def test_arena_colors_still_forwards(self) -> None:
        args = _parser().parse_args(
            ["launch", "--sensor-profile", "sensor-rich", "--arena-colors"]
        )
        self.assertTrue(args.arena_colors)
        self.assertIn("--arena-colors", _launch_command(args))

    def test_launch_flag_prefixes_fail_loudly(self) -> None:
        # With abbreviation matching disabled, an unmirrored or misspelled
        # dashed flag must be rejected, never silently prefix-matched onto
        # a different option.
        with self.assertRaises(SystemExit):
            _parser().parse_args(["launch", "--arena-col"])


if __name__ == "__main__":
    unittest.main()
