"""Redundant joint-target writes must be skipped, and never a real one.

Profiled 2026-08-20 (gpsr-rcw2026, simulator alone, TINKER_SIM_PROFILE=1),
per 12.2 ms physics step:

    physx          5.0 ms   41%
    write_data     3.95 ms  32%   <- set_dof_*_targets into PhysX
    targets        2.85 ms  23%   <- set_joint_*_target on the articulation
    robot_update   0.20 ms
    object_views   0.19 ms

So 57% of "physics" is Isaac Lab tensor plumbing rather than the solver. It is
mostly wasted: physics runs at 120 Hz while commands arrive far slower (Nav2
publishes /cmd_vel at ~25 Hz), so most steps re-send byte-identical targets.

Skipping them is semantically identical. `write_data_to_sim` writes external
wrenches only when a wrench composer is active (this backend never uses one),
runs the *implicit* actuator model, which is stateless, and then calls
`set_dof_actuation_forces` / `set_dof_position_targets` /
`set_dof_velocity_targets`. PhysX drive targets persist until changed, so
re-writing the same values is a no-op.

The dangerous failure is the other direction -- skipping a write that mattered
-- so the gate is conservative: it writes on the first step, whenever any
target differs, whenever anything forces it (re-resolved articulation handles,
safety-stop transitions), and always if the escape hatch is set.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.target_write_gate import TargetWriteGate  # noqa: E402

BACKEND = ROOT / "simulation/tinker_sim_isaac/backend.py"


def _snap(t):
    return tuple(list(x) for x in t)


class TargetWriteGateTest(unittest.TestCase):
    def setUp(self):
        self.gate = TargetWriteGate(equal=lambda a, b: list(a) == list(b))

    def test_first_step_always_writes(self):
        self.assertTrue(self.gate.should_write(([1.0], [0.0], [0.0])))

    def test_identical_targets_are_skipped(self):
        t = ([1.0], [2.0], [3.0])
        self.assertTrue(self.gate.should_write(t))
        self.gate.note_written(_snap(t))
        self.assertFalse(self.gate.should_write(t))
        self.assertFalse(self.gate.should_write(t))

    def test_any_changed_component_forces_a_write(self):
        base = ([1.0], [2.0], [3.0])
        self.gate.should_write(base)
        self.gate.note_written(_snap(base))
        for changed in (([9.0], [2.0], [3.0]), ([1.0], [9.0], [3.0]), ([1.0], [2.0], [9.0])):
            self.assertTrue(
                self.gate.should_write(changed), f"missed a change in {changed}"
            )

    def test_a_skipped_step_does_not_lose_a_later_change(self):
        """The snapshot must stay pinned to what was actually written."""
        base = ([1.0], [2.0], [3.0])
        self.gate.should_write(base)
        self.gate.note_written(_snap(base))
        self.assertFalse(self.gate.should_write(base))
        self.assertTrue(self.gate.should_write(([1.5], [2.0], [3.0])))

    def test_force_next_overrides_equality(self):
        t = ([1.0], [2.0], [3.0])
        self.gate.should_write(t)
        self.gate.note_written(_snap(t))
        self.assertFalse(self.gate.should_write(t))
        self.gate.force_next()
        self.assertTrue(self.gate.should_write(t))

    def test_force_is_only_cleared_by_an_actual_write(self):
        t = ([1.0], [2.0], [3.0])
        self.gate.note_written(_snap(t))
        self.gate.force_next()
        self.assertTrue(self.gate.should_write(t))
        self.assertTrue(self.gate.should_write(t), "force cleared without a write")
        self.gate.note_written(_snap(t))
        self.assertFalse(self.gate.should_write(t))

    def test_always_write_escape_hatch(self):
        gate = TargetWriteGate(always_write=True, equal=lambda a, b: list(a) == list(b))
        t = ([1.0], [2.0], [3.0])
        gate.note_written(_snap(t))
        for _ in range(5):
            self.assertTrue(gate.should_write(t))

    def test_backend_uses_the_gate_and_keeps_an_escape_hatch(self):
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn("TargetWriteGate", source)
        self.assertIn("TINKER_SIM_ALWAYS_WRITE_TARGETS", source)

    def test_backend_forces_a_write_after_handles_are_re_resolved(self):
        """A fresh articulation view has no targets set in PhysX yet."""
        source = BACKEND.read_text(encoding="utf-8")
        self.assertIn("force_next", source)


if __name__ == "__main__":
    unittest.main()
