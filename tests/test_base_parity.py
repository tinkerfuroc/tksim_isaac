from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.base import BaseParityModel, Twist2D
from tinker_sim_core.calibration import BaseCalibration, CalibrationStatus


class BaseParityModelTest(unittest.TestCase):
    def setUp(self) -> None:
        self.calibration = BaseCalibration.development_default()
        self.model = BaseParityModel(self.calibration)

    def test_clamps_and_converts_cmd_vel(self) -> None:
        self.model.accept_command(Twist2D(9.0, 9.0), 10.0)
        command = self.model.wheel_command(10.0)
        self.assertFalse(command.watchdog_stop)
        self.assertAlmostEqual(command.left_rad_s, (0.60 - 0.125) / 0.0525)
        self.assertAlmostEqual(command.right_rad_s, (0.60 + 0.125) / 0.0525)

    def test_watchdog_and_safety_stop_force_zero(self) -> None:
        self.model.accept_command(Twist2D(0.2, 0.1), 0.0)
        self.assertNotEqual(self.model.wheel_command(0.1).left_rad_s, 0.0)
        stale = self.model.wheel_command(0.251)
        self.assertTrue(stale.watchdog_stop)
        self.assertEqual((stale.left_rad_s, stale.right_rad_s), (0.0, 0.0))
        self.model.accept_command(Twist2D(0.2, 0.0), 1.0)
        self.model.set_safety_stop(True)
        stopped = self.model.wheel_command(1.0)
        self.assertEqual((stopped.left_rad_s, stopped.right_rad_s), (0.0, 0.0))

    def test_odometry_uses_wheels_and_rejects_backward_time(self) -> None:
        omega = 0.3 / self.calibration.wheel_radius_m
        self.model.observe_wheels(omega, omega, 5.0)
        estimate = self.model.observe_wheels(omega, omega, 7.0)
        self.assertAlmostEqual(estimate.x, 0.6)
        self.assertAlmostEqual(estimate.y, 0.0)
        with self.assertRaises(ValueError):
            self.model.observe_wheels(omega, omega, 6.0)

    def test_missing_calibration_cannot_qualify(self) -> None:
        self.assertEqual(self.calibration.status, CalibrationStatus.MISSING)
        self.assertIn("missing", self.calibration.qualification_error().lower())


if __name__ == "__main__":
    unittest.main()
