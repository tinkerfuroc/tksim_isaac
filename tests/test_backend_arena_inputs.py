from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac import backend  # noqa: E402


class ResolveArenaInputsTest(unittest.TestCase):
    def test_none_passthrough(self) -> None:
        self.assertEqual(
            backend.resolve_arena_inputs(None, Path("m.yaml")),
            (None, Path("m.yaml")),
        )

    def test_both_set_rejected(self) -> None:
        with self.assertRaises(ValueError):
            backend.resolve_arena_inputs(Path("a"), Path("m.yaml"))

    def test_arena_resolves_colocated_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifact = Path(temporary)
            (artifact / "arena.usd").write_text("usd", encoding="utf-8")
            (artifact / "map.yaml").write_text("map", encoding="utf-8")
            arena_usd, effective_map_yaml = backend.resolve_arena_inputs(artifact, None)
            self.assertEqual(arena_usd, artifact / "arena.usd")
            self.assertEqual(effective_map_yaml, artifact / "map.yaml")

    def test_missing_arena_usd_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifact = Path(temporary)
            (artifact / "map.yaml").write_text("map", encoding="utf-8")
            with self.assertRaises(FileNotFoundError):
                backend.resolve_arena_inputs(artifact, None)

    def test_missing_map_yaml_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifact = Path(temporary)
            (artifact / "arena.usd").write_text("usd", encoding="utf-8")
            with self.assertRaises(FileNotFoundError):
                backend.resolve_arena_inputs(artifact, None)


if __name__ == "__main__":
    unittest.main()
