import math
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# Pure helper only -- gripper_close_probe.py itself imports isaacsim at
# module load and cannot be imported here.
from validation.arm_joints_parse import ARM_JOINT_NAMES, parse_arm_joints_deg  # noqa: E402
from validation.arm_stream_packet import (  # noqa: E402
    ARM_JOINT_NAMES as STREAM_ARM_JOINT_NAMES,
    build_arm_hold_packet,
)


def test_parse_arm_joints_deg_converts_degrees_to_radians_in_joint_order():
    text = "0,10,-20,30,-40,50,180"
    out = parse_arm_joints_deg(text)
    assert list(out.keys()) == list(ARM_JOINT_NAMES)
    degrees = [0.0, 10.0, -20.0, 30.0, -40.0, 50.0, 180.0]
    for name, deg in zip(ARM_JOINT_NAMES, degrees):
        assert out[name] == pytest.approx(math.radians(deg))


def test_parse_arm_joints_deg_tolerates_whitespace():
    out = parse_arm_joints_deg(" 0 , 10 , -20 , 30 , -40 , 50 , 180 ")
    assert out["joint2"] == pytest.approx(math.radians(10.0))


def test_parse_arm_joints_deg_rejects_wrong_count():
    with pytest.raises(ValueError, match="needs 7 comma-separated"):
        parse_arm_joints_deg("0,10,20")


def test_parse_arm_joints_deg_rejects_non_numeric():
    with pytest.raises(ValueError, match="not a number"):
        parse_arm_joints_deg("0,10,20,30,40,50,nope")


# --------------------------------------------- --arm-stream-hz packet builder
def test_build_arm_hold_packet_uses_joint1_to_7_and_zero_velocities():
    measured = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    names, positions, velocities = build_arm_hold_packet(measured)
    assert names == STREAM_ARM_JOINT_NAMES == tuple(f"joint{i}" for i in range(1, 8))
    assert positions == tuple(measured)
    assert velocities == (0.0,) * 7
    # No gripper joint (drive_joint or a follower) is ever in the packet.
    assert not any("finger" in n or "knuckle" in n or n == "drive_joint" for n in names)


def test_build_arm_hold_packet_echoes_a_fake_measured_vector_in_order():
    fake_measured = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    _, positions, _ = build_arm_hold_packet(fake_measured)
    for value, expected in zip(positions, fake_measured):
        assert value == pytest.approx(expected)


def test_build_arm_hold_packet_rejects_wrong_count():
    with pytest.raises(ValueError, match="needs 7 positions"):
        build_arm_hold_packet([0.0, 1.0, 2.0])


# --------------------------------------------------------- --arm-stream-hz CLI
# gripper_close_probe.py's argparse.parse_args() runs at module top-level,
# BEFORE `from isaacsim import SimulationApp` -- so a subprocess invocation
# exercises the real CLI parser without ever needing an Isaac install. An
# argparse usage error (unknown flag / bad type) always exits 2 before that
# import line is reached; any other outcome (isaacsim missing, or -- on a
# machine where it is installed -- blocked on its interactive EULA prompt
# with stdin closed) proves parsing itself succeeded, since the process got
# past parse_args() to reach it.
PROBE = ROOT / "validation" / "gripper_close_probe.py"


def _run_probe_cli(*extra_args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(PROBE), "--out", "/tmp/gripper_close_probe_arm_stream_cli_test.jsonl", *extra_args],
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=60,
    )


def test_arm_stream_hz_flag_is_registered_and_defaults_off():
    result = _run_probe_cli("--help")
    assert result.returncode == 0
    assert "--arm-stream-hz ARM_STREAM_HZ" in result.stdout


def test_arm_stream_hz_accepts_a_numeric_value():
    result = _run_probe_cli("--phase", "B", "--arm-stream-hz", "360")
    assert result.returncode != 2, result.stderr
    assert "--arm-stream-hz" not in result.stderr


def test_arm_stream_hz_rejects_a_non_numeric_value():
    result = _run_probe_cli("--phase", "B", "--arm-stream-hz", "notanumber")
    assert result.returncode == 2
    assert "argument --arm-stream-hz: invalid float value" in result.stderr
