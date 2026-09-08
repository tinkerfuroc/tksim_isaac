"""Pure helper for gripper_close_probe.py's ``--arm-stream-hz`` diagnostic.

Builds the synthetic JTC-style "hold" packet the probe injects into
``IsaacWholeRobotBackend.command_joints()`` -- the same backend entry point
``RosStandardGateway._joint_command`` -> ``spin_once()`` uses for a real
``/isaac_joint_commands`` message (``command_gateway.py`` / ``ros_gateway.py``
both build a ``JointCommand`` from parallel name/position/velocity/effort
sequences and hand it straight to ``backend.command_joints``). Split out of
the probe script -- which imports ``isaacsim`` at module load, so it cannot
be imported by a plain unit test -- so the packet shape can be exercised
with no Isaac dependency.
"""
from __future__ import annotations

# xArm7 joint order, matching gripper_close_probe.py's ARM tuple. A stream
# packet never names a gripper joint (drive_joint or any of its followers).
ARM_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))


def build_arm_hold_packet(
    positions: "list[float] | tuple[float, ...]",
) -> tuple[tuple[str, ...], tuple[float, ...], tuple[float, ...]]:
    """``positions``: 7 floats, the CURRENT MEASURED joint1..7 angles (rad),
    in ``ARM_JOINT_NAMES`` order.

    Returns ``(names, positions, velocities)``: ``names`` is
    ``ARM_JOINT_NAMES``, ``positions`` echoes the given measured vector
    (a JTC "hold" -- command the arm to stay exactly where it already is),
    and ``velocities`` is an all-zero tuple the same length -- the shape a
    real hold command takes on ``/isaac_joint_commands``: the vendored
    ``topic_based_ros2_control`` config for the seven arm joints declares
    both a ``position`` AND a ``velocity`` command interface (see
    ``ros2_ws/src/tinker_sim_bridge/config/tinker_topic_control.ros2_control.xacro``),
    so a real hold packet carries a non-empty, explicit zero velocity array
    rather than an empty/omitted one.

    Raises ``ValueError`` (naming the bad count) if ``positions`` is not
    exactly 7 long, so a caller can surface it as a normal usage error.
    """
    positions = tuple(float(p) for p in positions)
    if len(positions) != len(ARM_JOINT_NAMES):
        raise ValueError(
            f"arm-stream hold packet needs {len(ARM_JOINT_NAMES)} positions "
            f"(joint1..joint{len(ARM_JOINT_NAMES)}), got {len(positions)}: {positions!r}"
        )
    velocities = tuple(0.0 for _ in ARM_JOINT_NAMES)
    return ARM_JOINT_NAMES, positions, velocities
