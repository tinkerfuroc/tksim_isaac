"""Adversarial tests for the independent integrated raw-physics verifier.

Python 3.12, ROS-free: this suite never imports ``rclpy``, generated ROS
messages, or geometry packages.  Report identity, canonical digest,
raw/evaluator correlation, and window-selection tests run in the simulator venv.
Humble generated-message/geometry tests stay in
``tests/test_integrated_gate_executor_ros.py`` and are never collected here.
"""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "tests"))

from integrated_verifier_fixtures import (  # noqa: E402
    load_test_config,
    load_test_scenario,
    raw_frames,
    write_integrated_attempt,
)
from integrated_gate_verifier import (  # noqa: E402
    select_integrated_gate_window,
    verify_integrated_attempt,
)

ALL_17_SCENARIOS = [
    "qualification-moveit-plan-joint",
    "qualification-moveit-plan-pose",
    "qualification-moveit-plan-blocked",
    "qualification-moveit-execute-joint",
    "qualification-moveit-execute-pose",
    "qualification-moveit-cartesian-retreat",
    "qualification-moveit-gripper",
    "qualification-moveit-cancel",
    "qualification-moveit-safety",
    "qualification-pick-place-positive",
    "qualification-pick-place-blocked-approach",
    "qualification-pick-place-unreachable-grasp",
    "qualification-pick-place-malformed-back",
    "qualification-pick-place-cancel-approach",
    "qualification-pick-place-cancel-transport",
    "qualification-pick-place-safety-transport",
    "qualification-pick-place-occupied-place",
]


def _verify(tmp_path, **overrides):
    return verify_integrated_attempt(
        scenario=load_test_scenario(overrides.pop("scenario", "qualification-pick-place-positive")),
        attempt_dir=write_integrated_attempt(tmp_path, **overrides),
        config=load_test_config(),
    )


def _verify_at(tmp_path, scenario, **overrides):
    """Verify with an explicit scenario (the fixture infers the same one)."""
    attempt = write_integrated_attempt(tmp_path, scenario=scenario, **overrides)
    return verify_integrated_attempt(
        scenario=load_test_scenario(scenario),
        attempt_dir=attempt,
        config=load_test_config(),
    )


# --------------------------------------------------------------------------- #
# The eight brief Step-1 acceptance tests (verbatim).
# --------------------------------------------------------------------------- #
def test_action_success_cannot_override_bad_placement(tmp_path):
    verdict = _verify(tmp_path, action_success=True, placement_error_m=0.20)
    assert verdict["status"] == "verified-fail"


def test_scene_attachment_cannot_prove_grasp(tmp_path):
    verdict = _verify(tmp_path, scene_attached=True, bilateral_contact=False)
    assert verdict["status"] == "verified-fail"


def test_expected_objects_are_not_measured_truth(tmp_path):
    verdict = _verify(tmp_path, expected_cube=True, omit_measured_cube=True)
    assert verdict["status"] == "evidence-invalid"


def test_raw_drain_after_terminal_is_outside_selected_gate_window(tmp_path):
    attempt, records = raw_frames(tmp_path, before=1, inside=5, after_terminal=3)
    window, _ = select_integrated_gate_window(
        records,
        attempt,
        "qualification-pick-place-positive",
        attempt_id="attempt-1",
        manifest_present=True,
        gate_start=1.0,
        gate_end=5.0,
        physics_hz=1.0,
    )
    assert [frame["frame_index"] for frame in window] == [0, 1, 2, 3, 4, 5]
    assert max(frame["frame_index"] for frame in records) == 8


def test_frame_gap_or_raw_evaluator_mismatch_invalidates_evidence(tmp_path):
    assert _verify(tmp_path / "gap", frame_indices=[1, 3])["status"] == "evidence-invalid"
    assert _verify(tmp_path / "count", raw_frame_count=4, evaluator_frame_count=3)["status"] == "evidence-invalid"


def test_plan_only_motion_or_cancel_resume_fails(tmp_path):
    assert _verify(tmp_path / "plan", plan_only_target_delta=0.01)["status"] == "verified-fail"
    assert _verify(tmp_path / "cancel", cancel_post_terminal_motion=0.02)["status"] == "verified-fail"


def test_safety_replay_release_outside_and_occupied_release_fail(tmp_path):
    assert _verify(tmp_path / "safety", safety_post_clear_target_motion=0.01)["status"] == "verified-fail"
    assert _verify(tmp_path / "place", release_region_error_m=0.20)["status"] == "verified-fail"
    assert _verify(
        tmp_path / "occupied", scenario="qualification-pick-place-occupied-place",
        occupied_release=True,
    )["status"] == "verified-fail"


# --------------------------------------------------------------------------- #
# Full-matrix sweep (§8.1): all 17 scenarios produce a verified-pass attempt.
# --------------------------------------------------------------------------- #
def test_full_matrix_all_17_scenarios_pass():
    import tempfile
    for scenario_id in ALL_17_SCENARIOS:
        with tempfile.TemporaryDirectory() as directory:
            verdict = _verify_at(Path(directory), scenario_id)
            assert verdict["status"] == "verified-pass", (
                f"{scenario_id}: {verdict['status']}: {verdict['errors'][:1]}"
            )


def test_full_matrix_verified_fail_per_class():
    import tempfile
    fails = {
        "qualification-moveit-plan-joint": dict(plan_only_target_delta=0.01),
        "qualification-moveit-plan-blocked": dict(plan_result_success=True),
        "qualification-moveit-execute-joint": dict(joint_tracking_error_rad=0.05),
        "qualification-moveit-execute-pose": dict(tcp_tracking_error_m=0.05),
        "qualification-moveit-cartesian-retreat": dict(retreat_short=True),
        "qualification-moveit-gripper": dict(gripper_travel_short=True),
        "qualification-moveit-cancel": dict(cancel_post_terminal_motion=0.02),
        "qualification-moveit-safety": dict(safety_post_clear_target_motion=0.01),
        "qualification-pick-place-positive": dict(placement_error_m=0.20),
        "qualification-pick-place-blocked-approach": dict(approach_contact=True),
        "qualification-pick-place-unreachable-grasp": dict(approach_tcp_motion=True),
        "qualification-pick-place-malformed-back": dict(malformed_pick_goal_sent=True),
        "qualification-pick-place-cancel-transport": dict(post_cancel_motion=True),
        "qualification-pick-place-safety-transport": dict(post_clear_resume=True),
        "qualification-pick-place-occupied-place": dict(occupied_release=True),
    }
    for scenario_id, overrides in fails.items():
        with tempfile.TemporaryDirectory() as directory:
            verdict = _verify_at(Path(directory), scenario_id, **overrides)
            assert verdict["status"] == "verified-fail", (
                f"{scenario_id}: {verdict['status']}: {verdict['errors'][:1]}"
            )


# --------------------------------------------------------------------------- #
# §8.2 C1 — terminal anchors exclude post-terminal drain.
# --------------------------------------------------------------------------- #
def _journal_records(attempt):
    return [json.loads(line) for line in (attempt / "planning-scene.jsonl").read_text().splitlines() if line.strip()]


def _raw_records(attempt):
    return [json.loads(line) for line in (attempt / "physics_truth.jsonl").read_text().splitlines() if line.strip()]


def _first_key(journal, event):
    for record in journal:
        if record.get("event") == event:
            return record["frame_index"], record["timestamp"]
    raise AssertionError(f"journal has no {event!r} key")


def test_terminal_anchor_excludes_post_terminal_drain(tmp_path):
    for scenario_id, anchor_event in [
        ("qualification-moveit-execute-joint", "execution-terminal"),
        ("qualification-pick-place-positive", "released-settled"),
        ("qualification-moveit-cancel", "quiescent"),
    ]:
        attempt = write_integrated_attempt(tmp_path / scenario_id, scenario=scenario_id)
        journal = _journal_records(attempt)
        records = _raw_records(attempt)
        start_frame, start_ts = _first_key(journal, "fixture-ready")
        end_frame, end_ts = _first_key(journal, anchor_event)
        window, _ = select_integrated_gate_window(
            records,
            attempt,
            scenario_id,
            attempt_id="attempt-1",
            manifest_present=True,
            gate_start=float(start_ts),
            gate_end=float(end_ts),
            physics_hz=120.0,
        )
        window_frames = [int(r["frame_index"]) for r in window]
        assert max(window_frames) == end_frame, f"{scenario_id}: window leaks post-anchor frames"
        # A raw frame exists strictly after the terminal anchor (the drain).
        assert any(int(r["frame_index"]) > end_frame for r in records), f"{scenario_id}: no drain frames"
        # teardown (the last journal key) is never in the authoritative window.
        _, teardown_frame = _first_key(journal, "teardown")
        assert teardown_frame not in window_frames


# --------------------------------------------------------------------------- #
# §8.3 C1 — observation subwindows end at quiescent, never teardown.
# --------------------------------------------------------------------------- #
def test_observation_subwindow_ends_at_quiescent_never_teardown(tmp_path):
    # Fresh motion strictly after quiescent (before teardown) is outside the
    # [cancel-requested, quiescent] observation subwindow, so it fails via
    # no_later_stage (post-quiescent scan), not via quiescent_after_cancel.
    verdict = _verify_at(tmp_path, "qualification-moveit-cancel", cancel_post_terminal_motion=0.02)
    assert verdict["status"] == "verified-fail"
    names = [c["name"] for c in verdict["checks"] if not c["passed"]]
    assert "no_later_stage" in names
    assert "quiescent_after_cancel" not in names


# --------------------------------------------------------------------------- #
# §8.4 M5 — blocked-by-gate-b marker.
# --------------------------------------------------------------------------- #
def test_blocked_by_gate_b_marker(tmp_path):
    import tempfile
    # Well-formed marker -> blocked-by-gate-b.
    with tempfile.TemporaryDirectory() as directory:
        attempt = write_integrated_attempt(Path(directory))
        (attempt / "gate-b-status.json").write_text(
            json.dumps({"schema_version": 1, "status": "blocked"}), encoding="utf-8"
        )
        verdict = verify_integrated_attempt(
            scenario=load_test_scenario("qualification-pick-place-positive"),
            attempt_dir=attempt,
            config=load_test_config(),
        )
        assert verdict["status"] == "blocked-by-gate-b"
        assert verdict["verified"] is False
        assert verdict["authority"] == "physics_truth.jsonl"

    # Malformed marker -> evidence-invalid (fail closed).
    for marker in [
        {"schema_version": 1, "status": "unblocked"},
        {"schema_version": 2, "status": "blocked"},
        "not-json",
    ]:
        with tempfile.TemporaryDirectory() as directory:
            attempt = write_integrated_attempt(Path(directory))
            content = json.dumps(marker) if not isinstance(marker, str) else marker
            (attempt / "gate-b-status.json").write_text(content, encoding="utf-8")
            verdict = verify_integrated_attempt(
                scenario=load_test_scenario("qualification-pick-place-positive"),
                attempt_dir=attempt,
                config=load_test_config(),
            )
            assert verdict["status"] == "evidence-invalid", marker

    # Absent marker -> normal path.
    with tempfile.TemporaryDirectory() as directory:
        attempt = write_integrated_attempt(Path(directory))
        verdict = verify_integrated_attempt(
            scenario=load_test_scenario("qualification-pick-place-positive"),
            attempt_dir=attempt,
            config=load_test_config(),
        )
        assert verdict["status"] == "verified-pass"


# --------------------------------------------------------------------------- #
# §8.5 B2 — physics_hz resolved through core_config.
# --------------------------------------------------------------------------- #
def test_physics_hz_resolved_from_core_config():
    config = load_test_config()
    assert float(config["physics"]["hz"]) == 120.0
    assert config["core_config"] == "simulation/qualification/manipulation-core.json"


def test_physics_hz_rejects_bad_core_config(tmp_path):
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        attempt = write_integrated_attempt(Path(directory))
        base = load_test_config()
        # Missing core_config.
        bad = {k: v for k, v in base.items() if k != "core_config"}
        verdict = verify_integrated_attempt(
            scenario=load_test_scenario("qualification-pick-place-positive"),
            attempt_dir=attempt,
            config=bad,
        )
        assert verdict["status"] == "evidence-invalid"
        # Non-positive hz.
        core_path = ROOT / "simulation" / "qualification" / "manipulation-core.json"
        core = json.loads(core_path.read_text(encoding="utf-8"))
        core["physics"]["hz"] = -1.0
        alt = ROOT / "simulation" / "qualification" / "task7-bad-core.json"
        alt.write_text(json.dumps(core), encoding="utf-8")
        try:
            bad_cfg = dict(base)
            bad_cfg["core_config"] = str(alt.relative_to(ROOT))
            verdict = verify_integrated_attempt(
                scenario=load_test_scenario("qualification-pick-place-positive"),
                attempt_dir=attempt,
                config=bad_cfg,
            )
            assert verdict["status"] == "evidence-invalid"
        finally:
            alt.unlink(missing_ok=True)


# --------------------------------------------------------------------------- #
# §8.6 C4 — endpoint allowlist.
# --------------------------------------------------------------------------- #
def test_endpoint_allowlist_is_required_actions_union_services(tmp_path):
    # A forbidden direct-Isaac endpoint in the summary -> evidence-invalid.
    verdict = _verify(tmp_path / "forbidden", endpoint_forbidden=True)
    assert verdict["status"] == "evidence-invalid"
    assert any("allowlist" in error or "forbidden" in error for error in verdict["errors"])

    # diagnostic_only executor rows alone never fail the endpoint check.
    verdict = _verify(tmp_path / "diag")
    assert verdict["status"] == "verified-pass"


# --------------------------------------------------------------------------- #
# §8.7 C5 — verdict gate is the exact scenario id.
# --------------------------------------------------------------------------- #
def test_verdict_gate_is_scenario_id(tmp_path):
    verdict = _verify(tmp_path)
    assert verdict["gate"] == "qualification-pick-place-positive"
    assert verdict["stage"] == "E"
    assert verdict["polarity"] == "positive"
    assert "E-positive" not in verdict["gate"]
    assert verdict["authority"] == "physics_truth.jsonl"
    assert verdict["action_results_diagnostic_only"] is True


# --------------------------------------------------------------------------- #
# §8.8 M7 — distinct raw/evaluator drain-mismatch reason code.
# --------------------------------------------------------------------------- #
def test_drain_mismatch_reason_code(tmp_path):
    verdict = _verify(tmp_path, raw_frame_count=4, evaluator_frame_count=3)
    assert verdict["status"] == "evidence-invalid"
    assert any("raw/evaluator drain mismatch" in error for error in verdict["errors"])


# --------------------------------------------------------------------------- #
# §8.9 m2 — contact force is strictly greater than the threshold.
# --------------------------------------------------------------------------- #
def test_contact_force_strictly_greater(tmp_path):
    # A contact at exactly the 1.0 N threshold is not counted: bilateral fails.
    verdict = _verify(tmp_path, contact_force_exact_1_0=True)
    assert verdict["status"] == "verified-fail"
    names = [c["name"] for c in verdict["checks"] if not c["passed"]]
    assert "bilateral_contact" in names


# --------------------------------------------------------------------------- #
# §8.10 m3 — transport direction guard.
# --------------------------------------------------------------------------- #
def test_transport_direction_guard(tmp_path):
    # Cube moved 0.20 m away from the place region: transport fails.
    verdict = _verify(tmp_path, transport_away=True)
    assert verdict["status"] == "verified-fail"
    transport = next(c for c in verdict["checks"] if c["name"] == "transport")
    assert transport["passed"] is False
    assert any("region" in reason for reason in transport["reasons"])


# --------------------------------------------------------------------------- #
# §8.11 m4 — lift baseline pinned to the pre-start frame's cube pose.
# --------------------------------------------------------------------------- #
def test_lift_baseline_pre_start(tmp_path):
    verdict = _verify(tmp_path)
    assert verdict["status"] == "verified-pass"
    lift = verdict["metrics"]["lift"]
    assert float(lift["initial_z_m"]) == 0.64  # pre-start cube z (source pose)
    assert float(lift["lift_m"]) >= 0.10


# --------------------------------------------------------------------------- #
# §8.12 C3 — phase-aware attachment validation.
# --------------------------------------------------------------------------- #
def test_phase_aware_attachment(tmp_path):
    import tempfile

    def verify_with_journal(attempt):
        return verify_integrated_attempt(
            scenario=load_test_scenario("qualification-pick-place-positive"),
            attempt_dir=attempt,
            config=load_test_config(),
        )

    # Passing positive: attached through lift/transport/before-release, absent
    # at/after scene-detach.
    with tempfile.TemporaryDirectory() as directory:
        attempt = write_integrated_attempt(Path(directory))
        assert verify_with_journal(attempt)["status"] == "verified-pass"

    # Mid-transport drop of the target -> evidence-invalid.
    with tempfile.TemporaryDirectory() as directory:
        attempt = write_integrated_attempt(Path(directory), journal_drop_target_mid_transport=True)
        verdict = verify_with_journal(attempt)
        assert verdict["status"] == "evidence-invalid"

    # Pre-attach record carrying the target -> evidence-invalid.
    with tempfile.TemporaryDirectory() as directory:
        attempt = write_integrated_attempt(Path(directory), journal_pre_attach_target=True)
        verdict = verify_with_journal(attempt)
        assert verdict["status"] == "evidence-invalid"


# --------------------------------------------------------------------------- #
# §8.13 M2 — no_arm_obstacle_contact excludes the grasped target.
# --------------------------------------------------------------------------- #
def test_obstacle_excludes_grasped_target(tmp_path):
    # Finger/cube grasp contact present and no pedestal contact -> passes.
    verdict = _verify(tmp_path)
    assert verdict["status"] == "verified-pass"
    obstacle = next(c for c in verdict["checks"] if c["name"] == "no_arm_obstacle_contact")
    assert obstacle["passed"] is True

    # Adding an arm-pedestal contact fails the predicate.
    verdict = _verify(tmp_path / "blocked", obstacle_contact=True)
    assert verdict["status"] == "verified-fail"
    obstacle = next(c for c in verdict["checks"] if c["name"] == "no_arm_obstacle_contact")
    assert obstacle["passed"] is False


# --------------------------------------------------------------------------- #
# §8.14 — negative forbidden-after-terminal is enforced.
# --------------------------------------------------------------------------- #
def test_negative_forbidden_after_terminal(tmp_path):
    # occupied-place: release into the region after the place failure is
    # forbidden -> verified-fail via target_region_settled.
    verdict = _verify(
        tmp_path,
        scenario="qualification-pick-place-occupied-place",
        occupied_release=True,
    )
    assert verdict["status"] == "verified-fail"
    names = [c["name"] for c in verdict["checks"] if not c["passed"]]
    assert "target_region_settled" in names
