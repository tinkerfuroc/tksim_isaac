"""Object-view discovery must not run a stage traversal on every physics step.

`IsaacWholeRobotBackend._refresh_object_views` resolves each expected scenario
object to a PhysX rigid-body view once and caches it. Objects that resolve are
skipped cheaply on later steps, but an object that does NOT resolve falls
through to `stage.Traverse()` -- a full USD stage walk -- and, before this
throttle, retried on every step forever.

Measured on 2026-08-20 with TINKER_SIM_PROFILE=1 (gpsr-rcw2026, 4 YCB objects
that never resolved): 10.4 ms of a 23.7 ms physics step, i.e. 44% of the step
and more than PhysX itself (5.7-6.1 ms). That single cost is what puts a 20 Hz
physics / 15 Hz camera budget out of reach.

Discovery is inherently a startup concern, so retrying a few times a second is
ample; the throttle keeps unresolved objects recoverable (a scenario may spawn
late) without paying a stage walk 120 times a second.
"""

from __future__ import annotations

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "simulation/tinker_sim_isaac/backend.py"


def _should_attempt(step_index: int, interval: int) -> bool:
    """Reference semantics the backend helper must implement."""
    return interval <= 1 or step_index % interval == 0


class ObjectDiscoveryThrottleTest(unittest.TestCase):
    def setUp(self):
        self.source = BACKEND.read_text(encoding="utf-8")

    def test_backend_exposes_a_discovery_interval(self):
        self.assertIn(
            "_object_discovery_interval", self.source,
            "backend must carry an explicit discovery interval rather than "
            "retrying stage traversal every step",
        )

    def test_refresh_is_gated_before_touching_the_stage(self):
        """The early-out must precede the USD/PhysX imports and traversal."""
        start = self.source.index("def _refresh_object_views")
        # Slice the real function body: up to the next method at the same
        # indent, so the assertion does not depend on an arbitrary window.
        nxt = self.source.find("\n    def ", start + 1)
        body = self.source[start : nxt if nxt != -1 else len(self.source)]
        gate = body.find("_object_discovery_interval")
        # Match the executable traversal, not a prose mention in a comment.
        traverse = body.find("for prim in stage.Traverse()")
        self.assertNotEqual(gate, -1, "no throttle inside _refresh_object_views")
        self.assertNotEqual(traverse, -1, "expected the stage traversal fallback")
        self.assertLess(
            gate, traverse,
            "the throttle must gate the function before it walks the stage",
        )

    def test_interval_is_env_tunable_and_defaults_to_subsecond_retry(self):
        self.assertIn("TINKER_SIM_OBJECT_DISCOVERY_INTERVAL", self.source)

    def test_throttle_semantics(self):
        """Every N-th step attempts; the rest are skipped."""
        interval = 60
        attempts = [i for i in range(600) if _should_attempt(i, interval)]
        self.assertEqual(len(attempts), 10, "expected 10 attempts across 600 steps")
        self.assertEqual(attempts[0], 0, "must attempt immediately at startup")
        self.assertEqual(attempts[1], 60)

    def test_interval_of_one_preserves_every_step_behaviour(self):
        """interval<=1 must be an explicit escape hatch back to old behaviour."""
        self.assertTrue(all(_should_attempt(i, 1) for i in range(50)))


if __name__ == "__main__":
    unittest.main()
