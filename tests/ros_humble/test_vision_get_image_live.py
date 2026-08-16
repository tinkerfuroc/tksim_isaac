"""Live vision round-trip: the real vision_util get_image node against the sim.

Requires (see README "Vision hardware-parity cameras"):
  1. sensor-rich sim running with --ros --arena-colors on ROS_DOMAIN_ID=42;
  2. `ros2 run vision_util get_image` under system Humble + the tk25_ws
     overlay on the same domain;
  3. TINKER_SIM_VISION_LIVE=1 in this test's environment.
Skipped otherwise; never part of the offline suite.

Head vs. wrist camera roles: the head camera's spawn pose aims it at the sky
(the tilt joint's default), so its frames are uniform sky-gray - a correct
render, not a defect.  `test_head_camera_round_trip` therefore asserts
transport only (status, encodings, dimensions, matched stamps) and does not
gate on depth plausibility or scene hue.  The wrist camera faces the colored
arena at spawn, so depth-plausibility and hue-presence assertions live on
`test_wrist_camera_round_trip` instead.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

if os.environ.get("TINKER_SIM_VISION_LIVE") != "1":
    pytest.skip(
        "set TINKER_SIM_VISION_LIVE=1 with the sensor-rich sim and get_image running",
        allow_module_level=True,
    )

rclpy = pytest.importorskip("rclpy", reason="requires Humble ROS Python runtime")
pytest.importorskip("sensor_msgs.msg", reason="requires Humble sensor_msgs")
srv_module = pytest.importorskip(
    "tinker_vision_msgs_26.srv", reason="requires the sourced tk25_ws overlay"
)

import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "validation"))
from arena_vision_smoke import hue_presence  # noqa: E402

EVIDENCE_DIR = ROOT / "reports/vision-roundtrip"
SERVICE_NAME = "get_image_service"


@pytest.fixture(scope="module")
def get_image_client():
    if not rclpy.ok():
        rclpy.init(args=[])
    node = rclpy.create_node("tinker_sim_vision_roundtrip_probe")
    client = node.create_client(srv_module.GetImage, SERVICE_NAME)
    assert client.wait_for_service(timeout_sec=15.0), (
        "get_image_service not available - is `ros2 run vision_util get_image` "
        "running on this ROS_DOMAIN_ID?"
    )
    yield node, client
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _call(node, client, camera: str):
    request = srv_module.GetImage.Request()
    request.camera = camera
    request.depth = True
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    assert future.done(), f"get_image call for {camera!r} timed out"
    return future.result()


def _decode(response, width: int, height: int):
    rgb = response.rgb_image
    depth = response.depth_image
    assert response.status == 0, response.error_msg
    assert rgb.encoding == "rgb8"
    assert (rgb.height, rgb.width) == (height, width)
    assert depth.encoding == "16UC1"
    assert (depth.height, depth.width) == (height, width)
    # One capture -> one stamp: the pair the synchronizer matched is exact.
    assert (rgb.header.stamp.sec, rgb.header.stamp.nanosec) == (
        depth.header.stamp.sec,
        depth.header.stamp.nanosec,
    )
    image = np.frombuffer(bytes(rgb.data), dtype=np.uint8).reshape(height, width, 3)
    depth_mm = np.frombuffer(bytes(depth.data), dtype=np.uint16).reshape(height, width)
    return image, depth_mm


def _save_evidence(name: str, image, depth_mm, extra: dict) -> None:
    from PIL import Image as PilImage

    EVIDENCE_DIR.mkdir(parents=True, exist_ok=True)
    PilImage.fromarray(image, mode="RGB").save(EVIDENCE_DIR / f"{name}-color.png")
    normalized = np.clip(depth_mm.astype(np.float64) / 8000.0 * 255.0, 0, 255)
    PilImage.fromarray(normalized.astype(np.uint8), mode="L").save(
        EVIDENCE_DIR / f"{name}-depth.png"
    )
    (EVIDENCE_DIR / f"{name}.json").write_text(
        json.dumps(extra, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def test_head_camera_round_trip(get_image_client) -> None:
    node, client = get_image_client
    image, depth_mm = _decode(_call(node, client, "orbbec"), 1280, 720)
    # The head camera's spawn pose aims it at the sky (tilt joint default),
    # so its frames are uniform sky-gray - a correct render.  This test
    # therefore proves transport only; content assertions live on the wrist
    # camera below.
    _save_evidence(
        "head",
        image,
        depth_mm,
        {"head_view": "sky at spawn tilt pose; content assertions live on the wrist camera"},
    )


def test_wrist_camera_round_trip(get_image_client) -> None:
    node, client = get_image_client
    image, depth_mm = _decode(_call(node, client, "realsense"), 848, 480)
    valid = depth_mm[depth_mm > 0]
    assert valid.size > 5_000, "wrist depth is almost entirely invalid"
    assert 50 < float(np.median(valid)) < 40_000, "wrist depth scale implausible"
    report = hue_presence(image)
    present = sorted(
        name for name, stats in report["colors"].items() if stats["present"]
    )
    # The wrist camera faces the colored arena at spawn; several distinct
    # palette hues prove authored-scene -> vision-node flow.
    assert len(present) >= 4, report
    _save_evidence(
        "wrist", image, depth_mm, {"hues_present": present, "report": report}
    )
