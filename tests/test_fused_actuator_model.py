"""The backend must bind the fused actuator-model path (and allow opting out).

Isaac Lab's `Articulation._apply_actuator_model` runs two Warp launches per
actuator group; with tinker2's five implicit groups that is ten launches per
target push -- measured 2026-08-21 as ~3.2 ms of a ~3.6 ms push on CPU
PhysX. `_fused_apply_actuator_model` keeps every group's own `compute()`
(gains, effort limits, telemetry buffers unchanged) and issues each kernel
once over the concatenated joint set. Verified on the GPU host
(outputs/bench/probe_fused.py): all seven staging/data buffers bit-identical
to the stock loop in held and driving regimes; push 4.3-6.4 ms -> 1.8 ms.

Isaac Lab refuses subclasses of its factory `Articulation` outside the
package, so the override is bound per instance.
"""

from __future__ import annotations

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "simulation/tinker_sim_isaac/backend.py"


class FusedActuatorModelTest(unittest.TestCase):
    def setUp(self):
        self.source = BACKEND.read_text(encoding="utf-8")

    def test_bound_per_instance_after_construction(self):
        self.assertIn("def bind_fused_actuator_model(robot", self.source)
        self.assertIn(
            "robot._apply_actuator_model = types.MethodType(_fused_apply_actuator_model, robot)",
            self.source,
        )
        self.assertLess(
            self.source.index("self._robot = Articulation(robot_cfg)"),
            self.source.index("bind_fused_actuator_model(self._robot)"),
        )

    def test_stock_loop_opt_out(self):
        """TINKER_SIM_STOCK_ACTUATOR_MODEL=1 keeps Isaac Lab's per-group loop."""
        self.assertIn('TINKER_SIM_STOCK_ACTUATOR_MODEL", "") != "1"', self.source)

    def test_keeps_per_group_compute_and_launches_each_kernel_once(self):
        body = self.source[
            self.source.index("def _fused_apply_actuator_model") : self.source.index(
                "def bind_fused_actuator_model"
            )
        ]
        self.assertEqual(body.count("actuator.compute("), 1)
        self.assertIn("for actuator, idx in zip(actuators, group_long):", body)
        self.assertEqual(body.count("articulation_kernels.update_targets"), 1)
        self.assertEqual(body.count("articulation_kernels.update_actuator_state_model"), 1)
        # gear-ratio (explicit) actuators are not handled: fall back to stock.
        self.assertIn('hasattr(actuator, "gear_ratio")', body)
        self.assertIn("type(self)._apply_actuator_model(self)", body)


if __name__ == "__main__":
    unittest.main()
