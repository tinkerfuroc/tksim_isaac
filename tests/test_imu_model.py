"""The simulated Mid-360 IMU's physics.

The publisher this replaces had four defects, and the tests below are written
against those specifically, because each one produced a message that LOOKED
well-formed:

* `linear_acceleration` was never assigned, so it read (0, 0, 0). An
  accelerometer measures specific force, so a sensor at rest reads ~+9.81
  m/s^2 up; zero means freefall. FAST-LIO uses that vector to find "down".
* the angular velocity was world-frame but stamped `livox360`. For a planar
  base yawing about z the two coincide -- so it was accidentally right for
  ordinary driving and wrong under any roll or pitch.
* it was sampled at the articulation root with no lever-arm term.
* the covariances were all-zero, which by REP-145 means "unknown".
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.imu_model import (  # noqa: E402
    ANGULAR_VELOCITY_VARIANCE,
    GRAVITY_WORLD,
    LINEAR_ACCELERATION_VARIANCE,
    ORIENTATION_UNAVAILABLE,
    cross,
    finite_difference_acceleration,
    imu_sample,
    lever_arm_acceleration,
    rotate_vector,
    specific_force,
    world_to_sensor,
)

IDENTITY = (1.0, 0.0, 0.0, 0.0)


def _yaw(degrees: float) -> tuple[float, float, float, float]:
    half = math.radians(degrees) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _roll(degrees: float) -> tuple[float, float, float, float]:
    half = math.radians(degrees) / 2.0
    return (math.cos(half), math.sin(half), 0.0, 0.0)


class TestSpecificForce(unittest.TestCase):
    def test_a_sensor_at_rest_reads_one_g_upward(self) -> None:
        """The defect that mattered most: this used to be (0, 0, 0)."""
        force = specific_force((0.0, 0.0, 0.0))
        self.assertAlmostEqual(force[0], 0.0)
        self.assertAlmostEqual(force[1], 0.0)
        self.assertAlmostEqual(force[2], 9.80665, places=5)

    def test_freefall_reads_zero(self) -> None:
        """The only state in which (0, 0, 0) is the honest answer."""
        force = specific_force(GRAVITY_WORLD)
        self.assertAlmostEqual(force[2], 0.0, places=6)

    def test_upward_acceleration_adds_to_the_reading(self) -> None:
        force = specific_force((0.0, 0.0, 2.0))
        self.assertAlmostEqual(force[2], 11.80665, places=5)

    def test_horizontal_acceleration_passes_through(self) -> None:
        force = specific_force((1.5, -2.5, 0.0))
        self.assertAlmostEqual(force[0], 1.5)
        self.assertAlmostEqual(force[1], -2.5)
        self.assertAlmostEqual(force[2], 9.80665, places=5)


class TestFrameRotation(unittest.TestCase):
    def test_identity_rotation_is_a_no_op(self) -> None:
        self.assertEqual(rotate_vector(IDENTITY, (1.0, 2.0, 3.0)), (1.0, 2.0, 3.0))

    def test_world_to_sensor_undoes_the_body_yaw(self) -> None:
        """A vector along world +x, with the body yawed 90 deg, is -y in body."""
        result = world_to_sensor(_yaw(90.0), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(result[0], 0.0, places=6)
        self.assertAlmostEqual(result[1], -1.0, places=6)
        self.assertAlmostEqual(result[2], 0.0, places=6)

    def test_yaw_does_not_change_a_vertical_vector(self) -> None:
        """Why the old world-frame bug hid: yaw alone leaves z untouched."""
        result = world_to_sensor(_yaw(37.0), (0.0, 0.0, 9.80665))
        self.assertAlmostEqual(result[2], 9.80665, places=5)

    def test_roll_tips_gravity_into_the_lateral_axis(self) -> None:
        """The case the old publisher got wrong: any roll/pitch, not yaw."""
        result = world_to_sensor(_roll(90.0), (0.0, 0.0, 9.80665))
        self.assertAlmostEqual(result[1], 9.80665, places=5)
        self.assertAlmostEqual(result[2], 0.0, places=5)

    def test_rotation_preserves_magnitude(self) -> None:
        vector = (1.0, -2.0, 3.0)
        rotated = world_to_sensor(_yaw(53.0), vector)
        self.assertAlmostEqual(
            math.dist((0, 0, 0), rotated), math.dist((0, 0, 0), vector), places=6
        )


class TestLeverArm(unittest.TestCase):
    def test_no_offset_means_no_extra_acceleration(self) -> None:
        self.assertEqual(
            lever_arm_acceleration((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            (0.0, 0.0, 0.0),
        )

    def test_steady_spin_gives_centripetal_acceleration_inward(self) -> None:
        """omega x (omega x r) = -omega^2 r for r perpendicular to omega."""
        omega = (0.0, 0.0, 2.0)
        offset = (0.5, 0.0, 0.0)
        extra = lever_arm_acceleration(omega, (0.0, 0.0, 0.0), offset)
        self.assertAlmostEqual(extra[0], -2.0, places=6)  # -(2^2) * 0.5
        self.assertAlmostEqual(extra[1], 0.0, places=6)

    def test_angular_acceleration_gives_tangential_acceleration(self) -> None:
        extra = lever_arm_acceleration((0.0, 0.0, 0.0), (0.0, 0.0, 3.0), (0.5, 0.0, 0.0))
        self.assertAlmostEqual(extra[0], 0.0, places=6)
        self.assertAlmostEqual(extra[1], 1.5, places=6)

    def test_offset_along_the_spin_axis_feels_nothing(self) -> None:
        extra = lever_arm_acceleration((0.0, 0.0, 2.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.4))
        for component in extra:
            self.assertAlmostEqual(component, 0.0, places=6)

    def test_cross_product_is_right_handed(self) -> None:
        self.assertEqual(cross((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)), (0.0, 0.0, 1.0))


class TestImuSample(unittest.TestCase):
    def test_stationary_level_robot(self) -> None:
        sample = imu_sample(
            quaternion_wxyz=IDENTITY,
            angular_velocity_world=(0.0, 0.0, 0.0),
            linear_acceleration_world=(0.0, 0.0, 0.0),
        )
        self.assertEqual(sample.angular_velocity, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(sample.linear_acceleration[2], 9.80665, places=5)

    def test_angular_velocity_is_reported_in_the_sensor_frame(self) -> None:
        """A body-frame yaw rate must stay on z regardless of heading."""
        sample = imu_sample(
            quaternion_wxyz=_yaw(90.0),
            angular_velocity_world=(0.0, 0.0, 1.0),
            linear_acceleration_world=(0.0, 0.0, 0.0),
        )
        self.assertAlmostEqual(sample.angular_velocity[2], 1.0, places=6)
        self.assertAlmostEqual(sample.angular_velocity[0], 0.0, places=6)

    def test_a_rolled_robot_sees_gravity_off_axis(self) -> None:
        sample = imu_sample(
            quaternion_wxyz=_roll(90.0),
            angular_velocity_world=(0.0, 0.0, 0.0),
            linear_acceleration_world=(0.0, 0.0, 0.0),
        )
        self.assertAlmostEqual(sample.linear_acceleration[1], 9.80665, places=5)
        self.assertAlmostEqual(sample.linear_acceleration[2], 0.0, places=5)

    def test_driving_forward_shows_up_on_the_body_x_axis(self) -> None:
        """Yawed 90 deg and accelerating along world +y is forward in body."""
        sample = imu_sample(
            quaternion_wxyz=_yaw(90.0),
            angular_velocity_world=(0.0, 0.0, 0.0),
            linear_acceleration_world=(0.0, 1.0, 0.0),
        )
        self.assertAlmostEqual(sample.linear_acceleration[0], 1.0, places=6)
        self.assertAlmostEqual(sample.linear_acceleration[2], 9.80665, places=5)

    def test_lever_arm_is_applied_when_supplied(self) -> None:
        with_arm = imu_sample(
            quaternion_wxyz=IDENTITY,
            angular_velocity_world=(0.0, 0.0, 2.0),
            linear_acceleration_world=(0.0, 0.0, 0.0),
            lever_arm_world=(0.5, 0.0, 0.0),
        )
        self.assertAlmostEqual(with_arm.linear_acceleration[0], -2.0, places=6)

    def test_lever_arm_defaults_to_absent(self) -> None:
        """Sampling the sensor's own link already includes those terms."""
        sample = imu_sample(
            quaternion_wxyz=IDENTITY,
            angular_velocity_world=(0.0, 0.0, 2.0),
            linear_acceleration_world=(0.0, 0.0, 0.0),
        )
        self.assertAlmostEqual(sample.linear_acceleration[0], 0.0, places=6)


class TestFiniteDifferenceFallback(unittest.TestCase):
    def test_first_sample_has_no_previous_velocity(self) -> None:
        self.assertEqual(
            finite_difference_acceleration((1.0, 0.0, 0.0), None, 0.005),
            (0.0, 0.0, 0.0),
        )

    def test_backward_difference(self) -> None:
        result = finite_difference_acceleration((1.0, 0.0, 0.0), (0.5, 0.0, 0.0), 0.005)
        self.assertAlmostEqual(result[0], 100.0, places=6)

    def test_non_positive_dt_is_refused_rather_than_dividing(self) -> None:
        self.assertEqual(
            finite_difference_acceleration((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0),
            (0.0, 0.0, 0.0),
        )

    def test_constant_velocity_gives_zero_acceleration(self) -> None:
        result = finite_difference_acceleration((2.0, 1.0, 0.0), (2.0, 1.0, 0.0), 0.005)
        self.assertEqual(result, (0.0, 0.0, 0.0))


class TestPublishedConstants(unittest.TestCase):
    def test_orientation_is_declared_unavailable(self) -> None:
        """REP-145: negative first covariance element = no orientation."""
        self.assertLess(ORIENTATION_UNAVAILABLE, 0.0)

    def test_covariances_are_non_zero(self) -> None:
        """All-zero would mean 'unknown' and read as a perfect sensor."""
        self.assertGreater(ANGULAR_VELOCITY_VARIANCE, 0.0)
        self.assertGreater(LINEAR_ACCELERATION_VARIANCE, 0.0)

    def test_gravity_points_down(self) -> None:
        self.assertLess(GRAVITY_WORLD[2], 0.0)
        self.assertAlmostEqual(abs(GRAVITY_WORLD[2]), 9.80665, places=5)


if __name__ == "__main__":
    unittest.main()
