"""Physics for the simulated Livox Mid-360 IMU published on ``/livox/imu``.

The gateway's original IMU was a stub with four defects, all of which this
module exists to fix:

* ``linear_acceleration`` was never assigned, so it published ``(0, 0, 0)``.
  That is not a noise-free ideal reading -- it is physically wrong. An
  accelerometer measures SPECIFIC FORCE, ``f = a - g``, so a stationary sensor
  reads ``+9.81 m/s^2`` along its gravity-opposing axis. Zero means freefall.
  FAST-LIO uses exactly this to find "down" before it will initialise.
* the angular velocity came from ``root_ang_vel_w`` -- the WORLD frame -- while
  the message was stamped ``livox360``. For a planar base yawing about z the
  two coincide, so it was accidentally right for ordinary driving and wrong the
  moment the robot pitched or rolled.
* it was sampled at the articulation root, with no lever-arm term. A real IMU
  ~0.2 m above the rotation centre feels centripetal and tangential
  acceleration whenever the base turns.
* the angular-velocity and linear-acceleration covariances were left all-zero,
  which by REP-145 means "unknown"; a consumer that trusts it reads zero
  variance, i.e. a perfect sensor.

Everything here is pure: no Isaac import, no ROS import. The backend supplies
world-frame state, the gateway fills the message, and the physics in between
is testable on its own.

NO NOISE OR BIAS is modelled. A real Mid-360's ICM-40609 has both, but the
simulator's value is determinism and the reference sim publishes a clean
signal too. If FAST-LIO later needs realistic noise, add it here behind a
spec flag rather than smearing it through the gateway.
"""
from __future__ import annotations

from dataclasses import dataclass

#: Standard gravity, world frame, z-up (the stage's up-axis is z).
GRAVITY_WORLD = (0.0, 0.0, -9.80665)

#: Diagonal covariances published alongside the sample.
#:
#: All-zero would mean "unknown" (REP-145) and invites consumers to treat the
#: signal as exact. These are deliberately small but non-zero: the sim's IMU is
#: clean, yet it is derived from a discretised physics step, so it is not
#: perfect either. Roughly one LSB of a consumer-grade MEMS part.
ANGULAR_VELOCITY_VARIANCE = 1e-4
LINEAR_ACCELERATION_VARIANCE = 1e-3

#: REP-145: a negative first element declares the orientation quaternion
#: invalid. The sim reports no orientation, exactly as the real driver does not.
ORIENTATION_UNAVAILABLE = -1.0


@dataclass(frozen=True)
class ImuSample:
    """One IMU reading, already in the sensor's own frame."""

    angular_velocity: tuple[float, float, float]
    linear_acceleration: tuple[float, float, float]


def quaternion_conjugate(
    quaternion_wxyz: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    w, x, y, z = quaternion_wxyz
    return (w, -x, -y, -z)


def rotate_vector(
    quaternion_wxyz: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    """``q v q^-1`` for a unit quaternion (w, x, y, z).

    Mirrors ``camera_rig._rotate_by_quaternion`` rather than pulling in a
    dependency for three lines of algebra.
    """
    w, x, y, z = quaternion_wxyz
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def world_to_sensor(
    quaternion_wxyz: tuple[float, float, float, float],
    vector_world: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Express a world-frame vector in the sensor's frame."""
    return rotate_vector(quaternion_conjugate(quaternion_wxyz), vector_world)


def cross(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def lever_arm_acceleration(
    angular_velocity_world: tuple[float, float, float],
    angular_acceleration_world: tuple[float, float, float],
    offset_world: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Extra world-frame acceleration at a point offset from the body origin.

    ``a_point = a_body + alpha x r + omega x (omega x r)`` -- the tangential and
    centripetal terms. Only needed when the IMU's own link is not a distinct
    body in the articulation and we have to carry the offset by hand; when
    ``livox_frame`` survives the URDF import as its own body, PhysX already
    reports its acceleration with these terms included.
    """
    tangential = cross(angular_acceleration_world, offset_world)
    centripetal = cross(angular_velocity_world, cross(angular_velocity_world, offset_world))
    return (
        tangential[0] + centripetal[0],
        tangential[1] + centripetal[1],
        tangential[2] + centripetal[2],
    )


def specific_force(
    linear_acceleration_world: tuple[float, float, float],
    gravity_world: tuple[float, float, float] = GRAVITY_WORLD,
) -> tuple[float, float, float]:
    """``a - g``: what an accelerometer actually measures, still world-frame.

    At rest ``a = 0`` and ``g = (0, 0, -9.80665)``, so this returns
    ``(0, 0, +9.80665)`` -- the familiar "1 g up" a level sensor reports.
    """
    return (
        linear_acceleration_world[0] - gravity_world[0],
        linear_acceleration_world[1] - gravity_world[1],
        linear_acceleration_world[2] - gravity_world[2],
    )


def imu_sample(
    *,
    quaternion_wxyz: tuple[float, float, float, float],
    angular_velocity_world: tuple[float, float, float],
    linear_acceleration_world: tuple[float, float, float],
    angular_acceleration_world: tuple[float, float, float] = (0.0, 0.0, 0.0),
    lever_arm_world: tuple[float, float, float] = (0.0, 0.0, 0.0),
    gravity_world: tuple[float, float, float] = GRAVITY_WORLD,
) -> ImuSample:
    """Build the sensor-frame reading from world-frame body state.

    ``lever_arm_world`` is the sensor's offset from the sampled body, already
    rotated into the world frame; pass ``(0, 0, 0)`` (the default) when the
    sampled body IS the sensor's link, which is the normal case.
    """
    acceleration_world = linear_acceleration_world
    if lever_arm_world != (0.0, 0.0, 0.0):
        extra = lever_arm_acceleration(
            angular_velocity_world, angular_acceleration_world, lever_arm_world
        )
        acceleration_world = (
            acceleration_world[0] + extra[0],
            acceleration_world[1] + extra[1],
            acceleration_world[2] + extra[2],
        )
    return ImuSample(
        angular_velocity=world_to_sensor(quaternion_wxyz, angular_velocity_world),
        linear_acceleration=world_to_sensor(
            quaternion_wxyz, specific_force(acceleration_world, gravity_world)
        ),
    )


def finite_difference_acceleration(
    velocity_world: tuple[float, float, float],
    previous_velocity_world: tuple[float, float, float] | None,
    dt: float,
) -> tuple[float, float, float]:
    """Backward difference of body velocity, for backends with no acceleration view.

    PhysX reports link accelerations directly (``body_com_acc_w``), which is
    the preferred source: differencing lags by half a step and amplifies the
    solver's per-step jitter. This exists so a backend that cannot supply
    accelerations still publishes something physical rather than zeros.
    Returns zeros on the first sample, when there is no previous velocity.
    """
    if previous_velocity_world is None or dt <= 0.0:
        return (0.0, 0.0, 0.0)
    return (
        (velocity_world[0] - previous_velocity_world[0]) / dt,
        (velocity_world[1] - previous_velocity_world[1]) / dt,
        (velocity_world[2] - previous_velocity_world[2]) / dt,
    )
