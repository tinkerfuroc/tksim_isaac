from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.contract_guard import (  # noqa: E402
    MANIPULATION_REQUIRED_TOPICS,
    NAVIGATION_REQUIRED_TOPICS,
    REQUIRED_STANDARD_SERVICES,
    evaluate_cardinality,
    evaluate_contract,
    evaluate_overall_state,
)


def test_contract_status_publisher_uses_latched_reliable_qos() -> None:
    source = (ROOT / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/contract_guard.py").read_text(
        encoding="utf-8"
    )
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert "QoSProfile" in source
    assert '"/sim/status/contract", status_qos' in source


def _complete(profile: str) -> dict[str, object]:
    topics = (
        MANIPULATION_REQUIRED_TOPICS
        if profile == "manipulation"
        else NAVIGATION_REQUIRED_TOPICS
    )
    return evaluate_contract(
        profile,
        topics,
        REQUIRED_STANDARD_SERVICES,
        ["/tinker_sim_command_gateway"],
        ["/tinker_truth_evaluator"] if profile == "manipulation" else [],
        safety_stop_publishers=(
            ["/tinker_sim_safety_supervisor"] if profile == "manipulation" else []
        ),
        safety_source_publishers=(
            {
                "xarm": ["/xarm_facade"],
                "collision": ["/tinker_isaac_gateway"],
            }
            if profile == "manipulation"
            else {}
        ),
    )


def test_manipulation_profile_excludes_navigation_topics() -> None:
    assert "/livox/lidar" not in MANIPULATION_REQUIRED_TOPICS
    assert "/scan" not in MANIPULATION_REQUIRED_TOPICS
    assert _complete("manipulation")["missing_topics"] == []


def test_manipulation_requires_typed_truth_and_single_command_owner() -> None:
    topics = dict(MANIPULATION_REQUIRED_TOPICS)
    topics.pop("/sim/truth/task_state")
    result = evaluate_contract(
        "manipulation",
        topics,
        REQUIRED_STANDARD_SERVICES,
        ["/other_node", "/tinker_sim_command_gateway"],
        ["/unexpected_truth_consumer"],
    )
    assert "/sim/truth/task_state" in result["missing_topics"]
    assert result["wrong_command_publishers"] == ["/other_node"]
    assert result["forbidden_truth_subscribers"] == ["/unexpected_truth_consumer"]


def test_manipulation_rejects_zero_and_duplicate_cardinality() -> None:
    topics = dict(MANIPULATION_REQUIRED_TOPICS)
    zero = evaluate_contract(
        "manipulation", topics, REQUIRED_STANDARD_SERVICES, [], []
    )
    assert zero["command_publisher_count"] == 0
    assert zero["raw_truth_subscriber_count"] == 0
    assert not zero["command_publisher_cardinality"]["ok"]
    assert not zero["raw_truth_subscriber_cardinality"]["ok"]

    duplicate = evaluate_contract(
        "manipulation",
        topics,
        REQUIRED_STANDARD_SERVICES,
        ["/tinker_sim_command_gateway", "/tinker_sim_command_gateway"],
        ["/tinker_truth_evaluator", "/tinker_truth_evaluator"],
    )
    assert duplicate["command_publisher_count"] == 2
    assert duplicate["raw_truth_subscriber_count"] == 2
    assert not duplicate["command_publisher_cardinality"]["ok"]
    assert not duplicate["raw_truth_subscriber_cardinality"]["ok"]


def test_manipulation_requires_single_owned_effective_safety_publisher() -> None:
    result = evaluate_contract(
        "manipulation",
        MANIPULATION_REQUIRED_TOPICS,
        REQUIRED_STANDARD_SERVICES,
        ["/tinker_sim_command_gateway"],
        ["/tinker_truth_evaluator"],
        safety_stop_publishers=[
            "/wrong_supervisor",
            "/tinker_sim_safety_supervisor",
        ],
        safety_source_publishers={
            "xarm": ["/xarm_facade"],
            "collision": [],
        },
        startup_grace_elapsed=False,
    )
    assert result["wrong_safety_stop_publishers"] == ["/wrong_supervisor"]
    assert result["safety_stop_publisher_cardinality"]["state"] == "fail"
    assert result["xarm_source_publisher_cardinality"]["state"] == "pass"
    assert result["collision_source_publisher_cardinality"]["state"] == "starting"
    assert evaluate_overall_state(result, False) == "fail"


def test_manipulation_safety_cardinality_fails_after_discovery_grace() -> None:
    result = evaluate_contract(
        "manipulation",
        MANIPULATION_REQUIRED_TOPICS,
        REQUIRED_STANDARD_SERVICES,
        ["/tinker_sim_command_gateway"],
        ["/tinker_truth_evaluator"],
        startup_grace_elapsed=True,
        safety_stop_publishers=[],
        safety_source_publishers={"xarm": [], "collision": []},
    )
    assert result["safety_stop_publisher_cardinality"]["state"] == "fail"
    assert result["xarm_source_publisher_cardinality"]["state"] == "fail"
    assert result["collision_source_publisher_cardinality"]["state"] == "fail"
    assert evaluate_overall_state(result, True) == "fail"


def test_manipulation_zero_cardinality_is_starting_during_grace() -> None:
    result = evaluate_cardinality("manipulation", 0, 0, False)
    assert result["command_publisher"]["state"] == "starting"
    assert result["raw_truth_subscriber"]["state"] == "starting"
    assert not result["command_publisher"]["ok"]
    assert not result["raw_truth_subscriber"]["ok"]


def test_manipulation_zero_cardinality_fails_after_grace() -> None:
    result = evaluate_cardinality("manipulation", 0, 0, True)
    assert result["command_publisher"]["state"] == "fail"
    assert result["raw_truth_subscriber"]["state"] == "fail"


def test_contract_snapshot_preserves_starting_cardinality_state() -> None:
    result = evaluate_contract(
        "manipulation",
        MANIPULATION_REQUIRED_TOPICS,
        REQUIRED_STANDARD_SERVICES,
        [],
        [],
        startup_grace_elapsed=False,
    )
    assert result["command_publisher_cardinality"]["state"] == "starting"
    assert result["raw_truth_subscriber_cardinality"]["state"] == "starting"


def test_overall_state_starting_cardinality_does_not_publish_pass() -> None:
    contract = _complete("manipulation")
    contract["command_publisher_cardinality"]["state"] = "starting"
    assert evaluate_overall_state(contract, False) == "starting"


def test_overall_state_failures_take_precedence_over_starting() -> None:
    contract = _complete("manipulation")
    contract["raw_truth_subscriber_cardinality"]["state"] = "starting"
    contract["wrong_types"] = ["/clock:['wrong/type']"]
    assert evaluate_overall_state(contract, False) == "fail"


def test_cardinality_duplicates_fail_during_grace() -> None:
    result = evaluate_cardinality("manipulation", 2, 2, False)
    assert result["command_publisher"]["state"] == "fail"
    assert result["raw_truth_subscriber"]["state"] == "fail"


def test_navigation_raw_truth_expectation_remains_zero() -> None:
    result = evaluate_cardinality("navigation", 1, 0, False)
    assert result["raw_truth_subscriber"]["expected"] == 0
    assert result["raw_truth_subscriber"]["state"] == "pass"
    unexpected = evaluate_cardinality("navigation", 1, 1, False)
    assert unexpected["raw_truth_subscriber"]["state"] == "fail"


def test_navigation_profile_retains_sensor_contract() -> None:
    result = _complete("navigation")
    assert result["missing_topics"] == []
    assert result["raw_truth_subscriber_cardinality"]["state"] == "pass"
    assert result["safety_stop_publisher_cardinality"]["state"] == "not-applicable"
    assert "/livox/lidar" in NAVIGATION_REQUIRED_TOPICS
