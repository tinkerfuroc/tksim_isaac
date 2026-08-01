"""Task 6: live integrated overlay graph probe (ROS Humble).

Imports ``rclpy`` and interface types only inside the test body, after a sourced
Humble environment.  When the integrated OMPL overlay is not running the probe
skips cleanly; during live qualification it verifies every typed action, service,
and publisher endpoint directly against the live ROS graph using the same
probe logic as ``tinker_sim_bridge.integrated_readiness_node``.
"""
from __future__ import annotations

import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
import sys

sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))


def _live_graph_running(node) -> bool:
    names = {name for name, _namespace in node.get_node_names_and_namespaces()}
    return "move_group" in names and "pick_and_place" in names


def test_live_integrated_overlay_graph() -> None:
    """Probe the live integrated overlay graph and verify every typed endpoint."""
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    probe = Node("live_graph_probe")
    try:
        time.sleep(2.0)
        if not _live_graph_running(probe):
            pytest.skip("integrated OMPL overlay graph is not running")
        probe.get_logger().info("integrated OMPL overlay graph detected")

        from tinker_sim_bridge.integrated_readiness import (
            INTEGRATED_ACTIONS,
            INTEGRATED_PUBLISHERS,
            INTEGRATED_SERVICES,
        )
        from tinker_sim_bridge.integrated_readiness_node import IntegratedReadiness

        # A uniquely-named observer with no status publisher so observing a
        # running overlay never creates a duplicate /integrated_readiness node
        # or a second /sim/status/integrated_manipulation publisher (which would
        # perturb the very cardinality evidence this probe asserts).
        readiness = IntegratedReadiness(
            node_name="live_graph_observer",
            create_status_publisher=False,
        )
        try:
            time.sleep(2.0)
            actions = readiness._probe_actions()
            for endpoint, expected in INTEGRATED_ACTIONS.items():
                entry = actions.get(endpoint, {})
                assert entry.get("count") == 1, (
                    "action {} count {} != 1".format(endpoint, entry.get("count"))
                )
                goal_type = "{}/action/{}_SendGoal".format(
                    expected["type"].split("/action/")[0],
                    expected["type"].split("/action/")[1],
                )
                assert goal_type in entry.get("observed_types", []), (
                    "action {} observed goal-service types {} do not include {}".format(
                        endpoint, entry.get("observed_types"), goal_type
                    )
                )
                if not expected["source"].startswith("controller_resource:"):
                    assert entry.get("source") == expected["source"], (
                        "action {} source {!r} != {!r}".format(
                            endpoint, entry.get("source"), expected["source"]
                        )
                    )
                else:
                    assert entry.get("source") == expected["source"]

            services = readiness._graph_services()
            for endpoint, expected in INTEGRATED_SERVICES.items():
                entry = services.get(endpoint, {})
                assert entry.get("count") == 1, (
                    "service {} count {} != 1".format(endpoint, entry.get("count"))
                )
                assert entry.get("type") == expected["type"], (
                    "service {} type mismatch".format(endpoint)
                )
                assert entry.get("source") == expected["source"], (
                    "service {} source {!r} != {!r}".format(
                        endpoint, entry.get("source"), expected["source"]
                    )
                )

            for topic, expected in INTEGRATED_PUBLISHERS.items():
                publishers = readiness.get_publishers_info_by_topic(topic)
                assert len(publishers) == expected["cardinality"], (
                    "topic {} publisher count {} != {}".format(
                        topic, len(publishers), expected["cardinality"]
                    )
                )
        finally:
            readiness.destroy_node()
    finally:
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
