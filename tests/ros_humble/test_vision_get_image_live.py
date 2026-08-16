"""Live vision round-trip against the sensor-rich sim.

Requires (see README "Vision hardware-parity cameras"):
  1. sensor-rich sim running with --ros --arena-colors on ROS_DOMAIN_ID=42;
  2. TINKER_SIM_VISION_LIVE=1 in this test's environment.
Skipped otherwise; never part of the offline suite.

This module consumes the camera streams directly, with the same semantics
tk26_vision binds (RELIABLE QoS, raw-buffer decodes, stamp pairing), and is
therefore the consumer-of-record for the round-trip acceptance.  It
additionally probes the real `vision_util get_image` service as an
informational check: that service has a known async-callback defect under
Humble (`async def` callbacks registered directly on message_filters'
ApproximateTimeSynchronizer are invoked synchronously and so are never
awaited, leaving its cache permanently empty) which makes it expected-fail
until upstream (tk26_vision, out of scope to patch here) fixes it. See
README "Vision hardware-parity cameras" for the full writeup.

Head vs. wrist camera roles: the head camera's spawn pose aims it at the sky
(the tilt joint's default), so its frames are uniform sky-gray - a correct
render, not a defect.  `test_head_camera_transport` therefore asserts
transport only (encodings, dimensions, matched stamps) and does not gate on
depth plausibility or scene hue.  The wrist camera faces the colored arena at
spawn, so depth-plausibility and hue-presence assertions live on
`test_wrist_camera_content` instead.
"""
from __future__ import annotations

import json
import os
import sys
import time as wall
from pathlib import Path

import pytest

if os.environ.get("TINKER_SIM_VISION_LIVE") != "1":
    pytest.skip(
        "set TINKER_SIM_VISION_LIVE=1 with the sensor-rich sim and get_image running",
        allow_module_level=True,
    )

rclpy = pytest.importorskip("rclpy", reason="requires Humble ROS Python runtime")
pytest.importorskip("sensor_msgs.msg", reason="requires Humble sensor_msgs")

import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "validation"))
from arena_vision_smoke import hue_presence  # noqa: E402

EVIDENCE_DIR = ROOT / "reports/vision-roundtrip"
SERVICE_NAME = "get_image_service"


@pytest.fixture(scope="module")
def camera_frames():
    """Collect one same-stamp color+depth pair per camera via RELIABLE subs."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    if not rclpy.ok():
        rclpy.init(args=[])
    node = rclpy.create_node("tinker_sim_vision_roundtrip_probe")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    latest: dict[str, Image] = {}
    topics = {
        "head_color": "/camera/color/image_raw",
        "head_depth": "/camera/depth/image_raw",
        "wrist_color": "/camera/xarm_camera/color/image_raw",
        "wrist_depth": "/camera/xarm_camera/aligned_depth_to_color/image_raw",
    }

    def _keep(key):
        def callback(message):
            latest[key] = message
        return callback

    for key, topic in topics.items():
        node.create_subscription(Image, topic, _keep(key), qos)

    def _paired(prefix):
        color = latest.get(f"{prefix}_color")
        depth = latest.get(f"{prefix}_depth")
        return (
            color is not None
            and depth is not None
            and (color.header.stamp.sec, color.header.stamp.nanosec)
            == (depth.header.stamp.sec, depth.header.stamp.nanosec)
        )

    deadline = wall.time() + 30.0
    while wall.time() < deadline and not (_paired("head") and _paired("wrist")):
        rclpy.spin_once(node, timeout_sec=0.5)
    assert _paired("head"), "no same-stamp head color+depth pair within 30s"
    assert _paired("wrist"), "no same-stamp wrist color+depth pair within 30s"
    frames = dict(latest)
    yield frames
    # rclpy.shutdown() is intentionally NOT called here: the get_image defect
    # probe below reuses this same context and owns the final shutdown, since
    # it is the last fixture used by the module's tests.
    try:
        node.destroy_node()
    except Exception:
        pass


@pytest.fixture(scope="module")
def get_image_node(camera_frames):
    """Node for the get_image defect probe.  Reuses the rclpy context that
    ``camera_frames`` already initialized, and - being the last fixture used
    in this module - owns the final rclpy shutdown.  Node/session teardown
    order between fixtures is not guaranteed relative to each other, so both
    ``destroy_node`` calls are defensive against an already-shutdown context.
    """
    node = rclpy.create_node("tinker_sim_vision_get_image_probe")
    yield node
    try:
        node.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()


def _decode_pair(frames: dict, prefix: str, width: int, height: int):
    """Decode a color+depth pair with tk26_vision's person_track convention:
    raw uint8 RGB buffer and raw uint16 millimeter depth buffer."""
    color = frames[f"{prefix}_color"]
    depth = frames[f"{prefix}_depth"]
    assert color.encoding == "rgb8"
    assert (color.height, color.width) == (height, width)
    assert depth.encoding == "16UC1"
    assert (depth.height, depth.width) == (height, width)
    # Re-assert explicitly: the fixture only waits for a matched pair, this
    # is the test's own proof the pair it decodes is in fact stamp-matched.
    assert (color.header.stamp.sec, color.header.stamp.nanosec) == (
        depth.header.stamp.sec,
        depth.header.stamp.nanosec,
    )
    image = np.frombuffer(bytes(color.data), dtype=np.uint8).reshape(height, width, 3)
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


def test_head_camera_transport(camera_frames) -> None:
    image, depth_mm = _decode_pair(camera_frames, "head", 1280, 720)
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


def test_wrist_camera_content(camera_frames) -> None:
    image, depth_mm = _decode_pair(camera_frames, "wrist", 848, 480)
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


def test_get_image_service_known_defect(get_image_node) -> None:
    """`ros2 run vision_util get_image` registers `async def` request and
    image-pair callbacks directly on message_filters' ApproximateTimeSynchronizer,
    which invokes callbacks synchronously.  The coroutines are therefore
    never awaited, the synchronizer's cache never fills, and the service
    always answers "No camera data" (status=1) - even though the camera
    streams ARE delivered and time-paired, as `test_head_camera_transport`
    and `test_wrist_camera_content` above independently prove, and as the
    node's own log confirms (a "coroutine ... was never awaited"
    RuntimeWarning fires for both cameras).  Patching tk26_vision is a spec
    non-goal, so this probe documents the defect instead of a live
    vision_util fix: it PASSES while the defect exists, and FAILS loudly the
    day it's fixed, at which point it should be promoted to a hard assertion
    on real data.  See README "Vision hardware-parity cameras".
    """
    srv_module = pytest.importorskip(
        "tinker_vision_msgs_26.srv", reason="requires the sourced tk25_ws overlay"
    )
    node = get_image_node
    client = node.create_client(srv_module.GetImage, SERVICE_NAME)
    if not client.wait_for_service(timeout_sec=5.0):
        pytest.skip("get_image node not running")
    request = srv_module.GetImage.Request()
    request.camera = "realsense"
    request.depth = True
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    assert future.done(), "get_image call for 'realsense' timed out"
    response = future.result()
    if response.status == 0:
        pytest.fail(
            "get_image unexpectedly returned data - its async-callback defect "
            "may have been fixed; promote this probe to a hard assertion"
        )
    assert "No camera data" in response.error_msg
