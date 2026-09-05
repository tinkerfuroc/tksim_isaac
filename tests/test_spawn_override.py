from __future__ import annotations

import sys
import tempfile
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


def _write_arena_map(directory: Path, rows: list[str]) -> None:
    # '#' -> occupied (0), '.' -> free (254); row 0 = TOP of image (world max y)
    width, height = len(rows[0]), len(rows)
    header = f"P5\n{width} {height}\n255\n".encode()
    payload = bytes(0 if ch == "#" else 254 for row in rows for ch in row)
    (directory / "map.pgm").write_bytes(header + payload)
    (directory / "map.yaml").write_text(
        "image: map.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0]\n"
    )


class ValidateArenaSpawnTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.arena = Path(self.temporary.name)

    def test_clear_spawn_passes(self):
        _write_arena_map(self.arena, ["." * 20] * 20)
        from run_sim import validate_arena_spawn
        validate_arena_spawn(self.arena, (1.0, 1.0))  # no raise

    def test_occupied_spawn_fails_with_suggestion(self):
        rows = ["." * 20 for _ in range(20)]
        for r in range(8, 12):
            rows[r] = rows[r][:8] + "####" + rows[r][12:]
        _write_arena_map(self.arena, rows)
        from run_sim import validate_arena_spawn
        with self.assertRaisesRegex(RuntimeError, r"--spawn-xy="):
            validate_arena_spawn(self.arena, (1.0, 1.0))

    def test_fully_occupied_map_fails_without_suggestion_crash(self):
        _write_arena_map(self.arena, ["#" * 6] * 6)
        from run_sim import validate_arena_spawn
        with self.assertRaisesRegex(RuntimeError, "no free cell"):
            validate_arena_spawn(self.arena, (0.3, 0.3))


if __name__ == "__main__":
    unittest.main()
