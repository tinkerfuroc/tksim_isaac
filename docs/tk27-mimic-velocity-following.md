# Measured gripper linkage velocity

The software mimic must follow both the measured position and velocity of the
master joint. Previously it predicted position by `q + qdot * dt` while leaving
the follower velocity target at zero. At 120 Hz, gains 1500/55, and a perfectly
aligned linkage moving at 0.3 rad/s, this requested -12.75 N*m from each follower
before the existing 2.5 N*m torque saturation. This parasitic braking is not a
gear-coupling constraint.

The mirror now copies measured q and qdot independently. It leaves the master
command, unrelated joints, effort limits, contact materials, collision geometry,
and physical qualification thresholds unchanged. A focused test fails with the
original implementation and passes with the correction. It evaluates the PD
torque of a co-moving linkage, rather than only checking target assignments.

Verification covers all 210 runtime/reset tests and seven subtests: 204 tests
pass in the simulator Python 3.12 environment, and the six real-rclpy import
cases pass under the ROS Humble Python 3.10 environment. Attempting those six
cases with the 3.12 interpreter fails because Humble's extension is built for
3.10; this is not counted as a passing combined invocation.

TK27 evidence remains under
`/home/tinker/tk25_ws/src/tk27_manipulation/outputs/`:

- `native-full-batch-cycle-20260910-12`: original mirror completes the physical
  sequence, but transport reaches 8.1078 degrees of relative drift and fails the
  unchanged five-degree bound.
- `native-full-batch-cycle-20260910-13`: corrected mirror completes grasp and
  123.52 mm physical lift, with a 1.70 s stable post-pick hold and maximum
  recorded angular drift 0.7194 degrees. Place fails terminal IK before any
  transfer. This is not proof that full transport retention is fixed.

The native correction is a demonstrated control-semantic fix. Complete physical
transport remains a separate acceptance requirement.
