"""The gateway's ``/livox/imu`` message.

`tests/test_imu_model.py` covers the physics; this covers the message the
gateway actually puts on the wire. Each test pins one of the four defects the
previous stub had, all of which produced a well-formed-looking message:

* `linear_acceleration` was never assigned -- (0, 0, 0), i.e. permanent
  freefall rather than the ~9.81 m/s^2 a level sensor reads at rest;
* the angular velocity was world-frame but stamped `livox360`;
* it was sampled at the articulation root, with no lever-arm term;
* the covariances were all-zero, which by REP-145 means "unknown".

Gateways are built with `object.__new__` to exercise `publish`-path helpers
without a live backend or ROS graph -- the same construction the lidar and
status tests in `test_ros_gateway.py` use.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac import ros_gateway as gateway_module  # noqa: E402
from tinker_sim_isaac.ros_gateway import RosStandardGateway  # noqa: E402


class _ImuBackend:
    """Backend exposing the newer `imu_state` reader."""

    def __init__(self, state):
        self.state = state
        self.dt = 1.0 / 120.0

    def imu_state(self):
        return self.state


class _LegacyImuBackend:
    """Older backend with only `root_state`, exercising the fallback path."""

    def __init__(self, **state):
        self._state = state
        self.dt = 1.0 / 120.0

    def root_state(self):
        return self._state


def _stamp():
    from builtin_interfaces.msg import Time  # type: ignore[import-untyped]

    return Time(sec=0, nanosec=0)


def _gateway(backend) -> RosStandardGateway:
    from sensor_msgs.msg import Imu  # type: ignore[import-untyped]

    gateway = object.__new__(RosStandardGateway)
    gateway.backend = backend
    gateway._Imu = Imu
    gateway._imu_frame_id = gateway_module._LIVOX_FRAME_ID
    gateway._imu_sample_period_s = 1.0 / 200.0
    gateway._imu_mount_offset = gateway_module._LIVOX_MOUNT_OFFSET_XYZ
    gateway._imu_previous_velocity = None
    return gateway


def _state(**overrides):
    state = {
        "body": "livox_frame",
        "quaternion_wxyz": (1.0, 0.0, 0.0, 0.0),
        "angular_velocity_world": (0.0, 0.0, 0.0),
        "linear_velocity_world": (0.0, 0.0, 0.0),
        "linear_acceleration_world": (0.0, 0.0, 0.0),
        "angular_acceleration_world": (0.0, 0.0, 0.0),
    }
    state.update(overrides)
    return state


class RosLivoxImuTest(unittest.TestCase):
    def test_a_resting_robot_reports_one_g_not_zero(self) -> None:
        """The headline defect: linear_acceleration was never assigned."""
        message = _gateway(_ImuBackend(_state()))._imu_message(_stamp())
        self.assertAlmostEqual(message.linear_acceleration.z, 9.80665, places=4)
        self.assertAlmostEqual(message.linear_acceleration.x, 0.0, places=6)
        self.assertAlmostEqual(message.linear_acceleration.y, 0.0, places=6)

    def test_frame_id_is_the_livox_frame(self) -> None:
        message = _gateway(_ImuBackend(_state()))._imu_message(_stamp())
        self.assertEqual(message.header.frame_id, "livox360")

    def test_orientation_is_declared_unavailable(self) -> None:
        message = _gateway(_ImuBackend(_state()))._imu_message(_stamp())
        self.assertLess(message.orientation_covariance[0], 0.0)

    def test_covariances_are_declared_rather_than_left_zero(self) -> None:
        """All-zero means 'unknown' (REP-145) and reads as a perfect sensor."""
        message = _gateway(_ImuBackend(_state()))._imu_message(_stamp())
        for axis in (0, 4, 8):
            self.assertGreater(message.angular_velocity_covariance[axis], 0.0)
            self.assertGreater(message.linear_acceleration_covariance[axis], 0.0)

    def test_angular_velocity_is_rotated_into_the_sensor_frame(self) -> None:
        """It used to publish the WORLD vector stamped livox360."""
        half = math.radians(90.0) / 2.0
        message = _gateway(
            _ImuBackend(
                _state(
                    quaternion_wxyz=(math.cos(half), math.sin(half), 0.0, 0.0),
                    angular_velocity_world=(0.0, 0.0, 1.0),
                )
            )
        )._imu_message(_stamp())
        # Rolled +90 deg about x, the body->world map sends y->z and z->-y, so
        # the inverse sends a world-z rate onto body +y.
        self.assertAlmostEqual(message.angular_velocity.y, 1.0, places=5)
        self.assertAlmostEqual(message.angular_velocity.z, 0.0, places=5)

    def test_yaw_alone_leaves_the_reading_unchanged(self) -> None:
        """Why the world-frame bug hid for so long on a planar base."""
        half = math.radians(63.0) / 2.0
        message = _gateway(
            _ImuBackend(
                _state(
                    quaternion_wxyz=(math.cos(half), 0.0, 0.0, math.sin(half)),
                    angular_velocity_world=(0.0, 0.0, 1.0),
                )
            )
        )._imu_message(_stamp())
        self.assertAlmostEqual(message.angular_velocity.z, 1.0, places=6)

    def test_no_lever_arm_when_sampling_the_sensor_link(self) -> None:
        """PhysX already includes those terms for livox_frame itself."""
        message = _gateway(
            _ImuBackend(_state(angular_velocity_world=(0.0, 0.0, 3.0)))
        )._imu_message(_stamp())
        self.assertAlmostEqual(message.linear_acceleration.x, 0.0, places=6)

    def test_lever_arm_applied_when_sampling_base_link(self) -> None:
        """Welded import: the 0.09 m forward offset is carried by hand."""
        message = _gateway(
            _ImuBackend(_state(body="base_link", angular_velocity_world=(0.0, 0.0, 3.0)))
        )._imu_message(_stamp())
        # Centripetal, -omega^2 * x_offset = -(3^2) * 0.09.
        self.assertAlmostEqual(message.linear_acceleration.x, -0.81, places=5)

    def test_falls_back_to_differencing_without_an_acceleration_view(self) -> None:
        backend = _ImuBackend(_state(linear_acceleration_world=None))
        gateway = _gateway(backend)
        gateway._imu_message(_stamp())  # primes the previous velocity
        backend.state["linear_velocity_world"] = (1.0, 0.0, 0.0)
        message = gateway._imu_message(_stamp())
        # 1.0 m/s gained over one 200 Hz sample = 200 m/s^2.
        self.assertAlmostEqual(message.linear_acceleration.x, 200.0, places=3)

    def test_first_differenced_sample_does_not_invent_acceleration(self) -> None:
        message = _gateway(
            _ImuBackend(_state(linear_acceleration_world=None))
        )._imu_message(_stamp())
        self.assertAlmostEqual(message.linear_acceleration.x, 0.0, places=6)
        self.assertAlmostEqual(message.linear_acceleration.z, 9.80665, places=4)

    def test_legacy_backend_without_imu_state_still_publishes(self) -> None:
        backend = _LegacyImuBackend(
            quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            angular_velocity_world=(0.0, 0.0, 0.5),
            linear_velocity_world=(0.0, 0.0, 0.0),
        )
        message = _gateway(backend)._imu_message(_stamp())
        self.assertAlmostEqual(message.linear_acceleration.z, 9.80665, places=4)
        self.assertAlmostEqual(message.angular_velocity.z, 0.5, places=6)


if __name__ == "__main__":
    unittest.main()
