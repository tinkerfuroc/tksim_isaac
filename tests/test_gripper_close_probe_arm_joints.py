import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

# Pure helper only -- gripper_close_probe.py itself imports isaacsim at
# module load and cannot be imported here.
from validation.arm_joints_parse import ARM_JOINT_NAMES, parse_arm_joints_deg  # noqa: E402


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
