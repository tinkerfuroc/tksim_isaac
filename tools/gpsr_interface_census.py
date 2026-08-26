#!/usr/bin/env python3
"""Check that every ROS interface GPSR needs is being served.

Run with the ROS overlay sourced and ROS_DOMAIN_ID matching the stack.
Exit 0 when everything GPSR calls exists; exit 1 with a per-stack breakdown.
"""
from __future__ import annotations

import argparse
import json
import sys

SERVICES = {
    "sim bridge": [("announce", "tinker_audio_msgs/srv/TextToSpeech")],
    "tk26_vision": [
        ("object_detection_generalist", "tinker_vision_msgs_26/srv/ObjectDetectionGeneralist"),
        ("object_detection_yolo", "tinker_vision_msgs_26/srv/ObjectDetection"),
        ("door_detection_srv", "tinker_vision_msgs_26/srv/DoorDetection"),
    ],
    "tk26_navigation": [
        ("find_approach_pose", "tinker_nav_msgs/srv/FindApproachPose"),
        ("orientation_angle_service", "tinker_nav_msgs/srv/OrientationAngle"),
    ],
}

ACTIONS = {
    "sim bridge": ["listen_action", "/xarm_gripper/gripper_action"],
    "nav2": ["navigate_to_pose"],
    "tk26_navigation": ["go_to_approach"],
    "tk26_vision": ["feature_extraction_service", "detect_waving_persons"],
    "tk25_manipulation": ["joint_move_action", "start_grasp"],
}

TOPICS = {
    "sim cameras": [
        ("/camera/color/image_raw", "sensor_msgs/msg/Image"),
        ("/camera/depth/image_raw", "sensor_msgs/msg/Image"),
        ("/camera/color/camera_info", "sensor_msgs/msg/CameraInfo"),
    ],
    # Split from "sim cameras": the wrist camera is disabled in hybrid runs
    # (TINKER_SIM_DISABLE_WRIST_CAMERA=1, manipulation mocked -- it is the
    # wrist camera's only consumer), so gpsr-stack only requires this stack
    # for live-manipulation runs (see scripts/gpsr-stack's
    # _gate_census_stacks).
    "sim cameras wrist": [
        ("/camera/xarm_camera/color/image_raw", "sensor_msgs/msg/Image"),
    ],
    "sim bridge": [("/pan_tilt_controller/state", "tinker_vision_msgs_26/msg/PanTiltState")],
}


def _action_present(name: str, have_actions: set[str]) -> bool:
    """True if `name` (bare or leading-slash) is in `have_actions`.

    Graph action names are always fully-qualified with a leading slash
    (e.g. "/listen_action"), so normalize to that form before comparing.
    """
    return ("/" + name.lstrip("/")) in have_actions


def main() -> int:
    import rclpy
    from rclpy.node import Node

    parser = argparse.ArgumentParser()
    parser.add_argument("--json", default=None)
    args = parser.parse_args()

    rclpy.init()
    node = Node("gpsr_interface_census")
    # Let discovery settle; a bare graph query right after init under-reports.
    end = node.get_clock().now().nanoseconds + 5_000_000_000
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    have_services = {name: types for name, types in node.get_service_names_and_types()}
    have_topics = {name: types for name, types in node.get_topic_names_and_types()}
    # Actions surface as a /_action/send_goal service per action name.
    have_actions = {
        name[: -len("/_action/send_goal")]
        for name in have_services
        if name.endswith("/_action/send_goal")
    }

    missing: dict[str, list[str]] = {}

    def miss(stack: str, what: str) -> None:
        missing.setdefault(stack, []).append(what)

    for stack, entries in SERVICES.items():
        for name, type_name in entries:
            found = have_services.get(name) or have_services.get("/" + name)
            if not found:
                miss(stack, f"service {name}")
            elif type_name not in found:
                miss(stack, f"service {name} has type {found}, expected {type_name}")

    for stack, names in ACTIONS.items():
        for name in names:
            if not _action_present(name, have_actions):
                miss(stack, f"action {name}")

    for stack, entries in TOPICS.items():
        for name, type_name in entries:
            found = have_topics.get(name)
            if not found:
                miss(stack, f"topic {name}")
            elif type_name not in found:
                miss(stack, f"topic {name} has type {found}, expected {type_name}")

    node.destroy_node()
    rclpy.shutdown()

    report = {"missing": missing, "ok": not missing}
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=1, sort_keys=True)
    if missing:
        for stack, items in sorted(missing.items()):
            print(f"MISSING [{stack}]")
            for item in items:
                print(f"  - {item}")
        return 1
    print("all GPSR interfaces present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
