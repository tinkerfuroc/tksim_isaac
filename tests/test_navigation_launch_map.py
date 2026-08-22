from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]

from test_artifact_export import _launch_stub_modules


def _load_navigation_launch():
    source = ROOT / "ros2_ws/src/tinker_sim_bridge/launch/navigation.launch.py"
    with mock.patch.dict(sys.modules, _launch_stub_modules()):
        spec = importlib.util.spec_from_file_location("navigation_launch_map", source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class NavigationLaunchMapTest(unittest.TestCase):
    def setUp(self) -> None:
        self.module = _load_navigation_launch()
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.artifact = Path(self.temporary.name) / "artifact"
        self.artifact.mkdir()

    def test_default_is_the_artifact_colocated_map(self) -> None:
        artifact_map = self.artifact / "map.yaml"
        artifact_map.write_text("image: map.pgm\n")
        resolved = self.module.resolve_map_yaml("", self.artifact)
        self.assertEqual(resolved, artifact_map)

    def test_blank_override_is_treated_as_absent(self) -> None:
        artifact_map = self.artifact / "map.yaml"
        artifact_map.write_text("image: map.pgm\n")
        resolved = self.module.resolve_map_yaml("   ", self.artifact)
        self.assertEqual(resolved, artifact_map)

    def test_override_selects_the_named_map(self) -> None:
        arena_map = Path(self.temporary.name) / "arena-map.yaml"
        arena_map.write_text("image: arena.pgm\n")
        resolved = self.module.resolve_map_yaml(str(arena_map), self.artifact)
        self.assertEqual(resolved, arena_map.resolve())

    def test_missing_default_map_fails_closed(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "navigation map does not exist"):
            self.module.resolve_map_yaml("", self.artifact)

    def test_missing_override_fails_closed(self) -> None:
        missing = Path(self.temporary.name) / "missing.yaml"
        with self.assertRaisesRegex(RuntimeError, "navigation map does not exist"):
            self.module.resolve_map_yaml(str(missing), self.artifact)

    def test_arena_map_wins_over_the_artifact_map(self) -> None:
        (self.artifact / "map.yaml").write_text("image: hardware.pgm\n")
        arena_map = Path(self.temporary.name) / "arena" / "map.yaml"
        arena_map.parent.mkdir()
        arena_map.write_text("image: arena.pgm\n")
        resolved = self.module.resolve_map_yaml("", self.artifact, arena_map)
        self.assertEqual(resolved, arena_map)

    def test_explicit_override_wins_over_the_arena_map(self) -> None:
        override = Path(self.temporary.name) / "override.yaml"
        override.write_text("image: override.pgm\n")
        arena_map = Path(self.temporary.name) / "arena" / "map.yaml"
        arena_map.parent.mkdir()
        arena_map.write_text("image: arena.pgm\n")
        resolved = self.module.resolve_map_yaml(str(override), self.artifact, arena_map)
        self.assertEqual(resolved, override.resolve())

    def test_missing_arena_map_fails_closed(self) -> None:
        (self.artifact / "map.yaml").write_text("image: hardware.pgm\n")
        missing = Path(self.temporary.name) / "arena" / "map.yaml"
        with self.assertRaisesRegex(RuntimeError, "navigation map does not exist"):
            self.module.resolve_map_yaml("", self.artifact, missing)

    def test_launch_declares_an_arena_argument(self) -> None:
        source = (ROOT / "ros2_ws/src/tinker_sim_bridge/launch/navigation.launch.py").read_text()
        self.assertIn('DeclareLaunchArgument("arena"', source)
        self.assertIn("resolve_arena_map_yaml(", source)


if __name__ == "__main__":
    unittest.main()
