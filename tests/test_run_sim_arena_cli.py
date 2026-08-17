"""Task 8: ``--arena`` CLI wiring -- ``resolve_arena_artifact`` pointer resolution.

Covers ``run_sim.resolve_arena_artifact``, the module-level helper the
``--arena`` launch flag uses to turn an arena id into the on-disk artifact
directory carrying ``arena.usd`` and ``map.yaml``. It mirrors the robot
artifact pointer consumption already in ``run_sim.py`` (``current.json`` ->
manifest -> payload directory) and fails closed on every missing piece.

``run_sim`` module-level code has no Isaac imports (they are all deferred
into ``main()``), so it is safe to import directly here without a running
Isaac Sim process.
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "tools"))

import run_sim as rs  # noqa: E402
from tinker_sim_deploy import arena_artifact  # noqa: E402

_SOURCE_LOCK = {
    "repository": "https://example/repo",
    "commit": "a" * 40,
    "records": [{"path": "worlds/x.world.xacro", "size": 3, "sha256": "b" * 64}],
}


class ResolveArenaArtifactTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)

    def _publish(self, payload=None):
        return arena_artifact.publish_asset_artifact(
            self.root,
            kind="arena",
            asset_id="rcw2026",
            payload=payload or {"arena.usd": b"usd-bytes", "map.yaml": b"image: map.pgm\n"},
            source_lock=_SOURCE_LOCK,
        )

    def test_resolves_via_pointer(self):
        pub = self._publish()
        resolved = rs.resolve_arena_artifact(self.root, "rcw2026")
        self.assertEqual(resolved, pub.artifact_dir)
        self.assertTrue((resolved / "arena.usd").is_file())
        self.assertTrue((resolved / "map.yaml").is_file())

    def test_missing_pointer_fails(self):
        # No artifacts/arena/rcw2026/current.json has ever been published.
        with self.assertRaises(FileNotFoundError):
            rs.resolve_arena_artifact(self.root, "rcw2026")

    def test_missing_payload_fails(self):
        # Pointer and manifest resolve cleanly, but the payload file the
        # backend actually needs (arena.usd) is absent -- fail closed rather
        # than handing the backend a directory missing its scene.
        pub = self._publish()
        (pub.artifact_dir / "arena.usd").unlink()
        with self.assertRaises(FileNotFoundError):
            rs.resolve_arena_artifact(self.root, "rcw2026")


class ArenaFlagSupportedTest(unittest.TestCase):
    """Final review Finding 4: ``--arena`` was silently ignored under
    ``--sensor-profile physics-only`` -- that branch never reads
    ``args.arena`` at all. ``arena_flag_supported`` is the pure predicate
    ``main()``'s parser-level guard uses (mirrors ``sensor_rich_implies_ros``/
    ``gateway_lidar_enabled``'s existing pure-helper pattern -- ``run_sim``
    has no Isaac imports at module level, so it is importable/testable under
    plain system Python).
    """

    def test_physics_only_unsupported(self):
        self.assertFalse(rs.arena_flag_supported("physics-only"))

    def test_other_profiles_supported(self):
        for profile in ("sensor-rich", "navigation-parity", "manipulation-core"):
            self.assertTrue(rs.arena_flag_supported(profile))


class ArenaFlagParserRejectionTest(unittest.TestCase):
    """End-to-end proof the guard is actually wired into ``main()``'s
    argument parsing, exercised via subprocess: the ``parser.error()`` call
    happens before ``main()`` ever imports ``isaacsim``, so this needs no
    GPU/Isaac Sim and runs under plain system Python (same rationale as
    ``--help``-style parser validation).
    """

    def test_arena_with_physics_only_is_rejected_before_isaac_import(self):
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "validation" / "run_sim.py"),
                "--sensor-profile", "physics-only",
                "--arena", "rcw2026",
                "--profile", "parity",
                "--scenario", "empty",
                "--seed", "0",
            ],
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn(
            "--arena requires a profile that loads the robot backend", result.stderr
        )


if __name__ == "__main__":
    unittest.main()
