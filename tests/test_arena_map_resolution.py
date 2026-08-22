from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.runtime import (  # noqa: E402
    resolve_arena_map_yaml,
    scenario_arena_id,
)


class ArenaMapResolutionTest(unittest.TestCase):
    """AMCL must localize against the arena the simulator raycasts.

    The robot artifact's colocated ``map.yaml`` is the hardware arena map
    (``0701_robocup_arena3``); an ``--arena rcw2026`` simulation synthesizes
    its lidar from ``artifacts/arena/rcw2026/<current>/map.yaml``.  The two
    share no occupied cell, so a launch that defaults to the artifact map
    leaves AMCL confidently wrong and Nav2 never reaches its goal.
    """

    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        arena_dir = self.root / "artifacts/arena/rcw2026/abc123"
        arena_dir.mkdir(parents=True)
        (arena_dir / "map.yaml").write_text("image: map.pgm\n")
        (arena_dir / "manifest.json").write_text("{}")
        (self.root / "artifacts/arena/rcw2026/current.json").write_text(
            json.dumps({"manifest": "artifacts/arena/rcw2026/abc123/manifest.json", "schema_version": 1})
        )
        self.arena_map = arena_dir / "map.yaml"

    def test_resolves_the_current_arena_colocated_map(self) -> None:
        self.assertEqual(resolve_arena_map_yaml(self.root, "rcw2026"), self.arena_map)

    def test_unknown_arena_fails_closed(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "arena artifact pointer does not exist"):
            resolve_arena_map_yaml(self.root, "nowhere")

    def test_unsafe_arena_id_is_rejected(self) -> None:
        for unsafe in ("../robot", "a/b", "", "."):
            with self.assertRaises(RuntimeError):
                resolve_arena_map_yaml(self.root, unsafe)

    def test_missing_arena_map_fails_closed(self) -> None:
        self.arena_map.unlink()
        with self.assertRaisesRegex(RuntimeError, "arena artifact missing map.yaml"):
            resolve_arena_map_yaml(self.root, "rcw2026")

    def test_scenario_arena_id_reads_world_arena(self) -> None:
        scenario = json.loads((ROOT / "simulation/scenarios/gpsr-rcw2026.json").read_text())
        self.assertEqual(scenario_arena_id(scenario), "rcw2026")

    def test_scenario_without_arena_yields_none(self) -> None:
        self.assertIsNone(scenario_arena_id({"world": {"mode": "empty"}}))
        self.assertIsNone(scenario_arena_id({}))
        self.assertIsNone(scenario_arena_id({"world": {"mode": "arena", "arena": ""}}))


if __name__ == "__main__":
    unittest.main()
