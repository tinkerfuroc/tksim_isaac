from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_isaac.backend import validate_spawn_xy  # noqa: E402
from run_sim import parse_spawn_xy  # noqa: E402
from tinker_sim_deploy.cli import _launch_command, _parser  # noqa: E402


class ValidateSpawnXyTest(unittest.TestCase):
    def test_default_origin_passes(self) -> None:
        self.assertEqual(validate_spawn_xy((0.0, 0.0)), (0.0, 0.0))

    def test_free_space_spawn_passes(self) -> None:
        self.assertEqual(validate_spawn_xy((-2.0, -2.5)), (-2.0, -2.5))

    def test_wrong_arity_fails(self) -> None:
        with self.assertRaisesRegex(ValueError, "two-element"):
            validate_spawn_xy((1.0,))
        with self.assertRaisesRegex(ValueError, "two-element"):
            validate_spawn_xy((1.0, 2.0, 3.0))

    def test_non_numeric_fails(self) -> None:
        with self.assertRaisesRegex(ValueError, "numbers"):
            validate_spawn_xy(("a", 2.0))
        with self.assertRaisesRegex(ValueError, "numbers"):
            validate_spawn_xy((True, 2.0))

    def test_non_finite_fails(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            validate_spawn_xy((float("nan"), 0.0))
        with self.assertRaisesRegex(ValueError, "finite"):
            validate_spawn_xy((0.0, float("inf")))


class ParseSpawnXyTest(unittest.TestCase):
    def test_parses_pair(self) -> None:
        self.assertEqual(parse_spawn_xy("-2.0,-2.5"), (-2.0, -2.5))

    def test_rejects_wrong_arity(self) -> None:
        with self.assertRaisesRegex(ValueError, "X,Y"):
            parse_spawn_xy("1.0")
        with self.assertRaisesRegex(ValueError, "X,Y"):
            parse_spawn_xy("1,2,3")

    def test_rejects_non_numeric(self) -> None:
        with self.assertRaisesRegex(ValueError, "numbers"):
            parse_spawn_xy("a,2")

    def test_rejects_non_finite(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            parse_spawn_xy("nan,0")


class DeployCliSpawnForwardingTest(unittest.TestCase):
    def _launch_args(self, extra: list[str]):
        parser = _parser()
        return parser.parse_args(["launch"] + extra)

    def test_spawn_xy_forwarded(self) -> None:
        # "=" form is required for negative coordinates: argparse would treat
        # a separate "-2.0,-2.0" token as an option string.
        args = self._launch_args(
            ["--sensor-profile", "navigation-parity", "--arena", "rcw2026",
             "--spawn-xy=-2.0,-2.0"]
        )
        command = _launch_command(args)
        self.assertIn("--spawn-xy=-2.0,-2.0", command)

    def test_spawn_xy_absent_by_default(self) -> None:
        args = self._launch_args(["--sensor-profile", "navigation-parity"])
        self.assertFalse(
            any(token.startswith("--spawn-xy") for token in _launch_command(args))
        )


if __name__ == "__main__":
    unittest.main()
