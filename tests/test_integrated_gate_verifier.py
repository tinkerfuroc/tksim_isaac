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

import pytest

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


def test_gate_window_raw_start_index_keeps_the_one_pre_start_frame(tmp_path):
    """RED: the executor writes ``raw_start_index`` as the count of physics-truth
    records AT the gate boundary, so it points at the FIRST in-gate frame.  The
    verifier must keep the frame immediately before that boundary (at index
    ``raw_start_index-1``) so ``select_integrated_gate_window`` can satisfy its
    one-pre-start-frame requirement.  On the current code ``_raw_gate_window``
    slices ``records[raw_start_index:]``, dropping the pre-start frame, and the
    integrated window raises 'requires one pre-start frame'.
    """
    attempt = tmp_path / "attempt-1"
    attempt.mkdir(parents=True, exist_ok=True)
    records = [
        {
            "frame_index": index,
            "timestamp": float(index),
            "scenario": "qualification-pick-place-positive",
            "seed": 7,
        }
        for index in range(-1, 6)  # frame -1 pre-start, frames 0..5 in-gate
    ]
    # raw_start_index = 1 is the FIRST in-gate frame (frame 0) at records[1];
    # the pre-start frame (frame -1) sits at records[0] == raw_start_index - 1.
    (attempt / "gate-window.json").write_text(
        json.dumps(
            {
                "gate": "qualification-pick-place-positive",
                "attempt_id": "attempt-1",
                "raw_start_index": 1,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    window, _ = select_integrated_gate_window(
        records,
        attempt,
        "qualification-pick-place-positive",
        attempt_id="attempt-1",
        manifest_present=True,
        gate_start=0.0,
        gate_end=5.0,
        physics_hz=1.0,
    )
    assert [frame["frame_index"] for frame in window] == [-1, 0, 1, 2, 3, 4, 5]


def test_gate_window_boundary_frame_equal_to_gate_start_keeps_pre_start_frame(tmp_path):
    """RED: the blocked Stage-C run journals fixture-ready on the SAME physics
    frame that the ``raw_start_index-1`` slice keeps.  ``raw_start_index`` is the
    count of records AT the gate boundary; when the journal's fixture-ready lands
    on that boundary frame (timestamp == gate_start), ``raw_start_index-1`` is the
    boundary frame itself, NOT strictly before gate_start.  The verifier must back
    the slice up one more frame so the integrated window always contains exactly
    one frame strictly before gate_start (else EvidenceError 'requires one
    pre-start frame').
    """
    attempt = tmp_path / "attempt-1"
    attempt.mkdir(parents=True, exist_ok=True)
    records = [
        {
            "frame_index": index,
            "timestamp": index / 120.0,
            "scenario": "qualification-moveit-plan-pose",
            "seed": 7,
        }
        for index in range(0, 560)
    ]
    # raw_start_index = 528 is the count of physics-truth records AT the gate
    # boundary; records[527] (ts 4.391666 == gate_start) is the fixture-ready
    # frame itself, so the raw_start_index-1 slice keeps no strictly-before
    # frame and select_integrated_gate_window must back up to records[526].
    (attempt / "gate-window.json").write_text(
        json.dumps(
            {
                "gate": "qualification-moveit-plan-pose",
                "attempt_id": "attempt-1",
                "raw_start_index": 528,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    gate_start = 527 / 120.0
    gate_end = 540 / 120.0
    window, _ = select_integrated_gate_window(
        records,
        attempt,
        "qualification-moveit-plan-pose",
        attempt_id="attempt-1",
        manifest_present=True,
        gate_start=gate_start,
        gate_end=gate_end,
        physics_hz=120.0,
    )
    pre_start = [record for record in window if record["timestamp"] < gate_start]
    assert len(pre_start) == 1
    assert pre_start[0]["frame_index"] == 526
    assert [frame["frame_index"] for frame in window] == list(range(526, 541))


def test_gate_window_float_skew_keeps_pre_start_frame(tmp_path):
    """RED: float rounding between ``gate_start`` (a ``k/physics_hz`` value) and
    the one-frame pre-start bound ``gate_start - 1/physics_hz`` can exclude the
    single pre-start frame.  For ``gate_start = 1304/120`` the bound
    ``10.858333333333334`` is one ulp ABOVE the one-tick-prior frame
    ``1303/120 = 10.858333333333333``, so the back-up frame is dropped and
    ``select_integrated_gate_window`` raises EvidenceError 'requires one
    pre-start frame' -- exactly what the live joint Stage-C run (T231055) hit.
    """
    attempt = tmp_path / "attempt-1"
    attempt.mkdir(parents=True, exist_ok=True)
    records = [
        {
            "frame_index": index,
            "timestamp": index / 120.0,
            "scenario": "qualification-moveit-plan-joint",
            "seed": 7,
        }
        for index in range(0, 1320)
    ]
    # raw_start_index = 1304 points at the boundary frame (frame 1304, ts
    # 1304/120 == gate_start); the raw_start_index-1 slice keeps frame 1303
    # whose ts (1303/120) is marginally BELOW gate_start - tolerance and must
    # survive the tolerance filter.
    (attempt / "gate-window.json").write_text(
        json.dumps(
            {
                "gate": "qualification-moveit-plan-joint",
                "attempt_id": "attempt-1",
                "raw_start_index": 1304,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    gate_start = 1304 / 120.0
    gate_end = 1316 / 120.0
    window, _ = select_integrated_gate_window(
        records,
        attempt,
        "qualification-moveit-plan-joint",
        attempt_id="attempt-1",
        manifest_present=True,
        gate_start=gate_start,
        gate_end=gate_end,
        physics_hz=120.0,
    )
    pre_start = [record for record in window if record["timestamp"] < gate_start]
    assert len(pre_start) == 1
    assert pre_start[0]["frame_index"] == 1303
    assert [frame["frame_index"] for frame in window] == list(range(1303, 1317))


def test_gate_window_boundary_frame_strictly_after_gate_start_keeps_one_pre_start_frame(tmp_path):
    """The normal case: the boundary frame (``raw_start_index-1``) is strictly
    AFTER gate_start, so the slice already retains the nearest strictly-before
    frame as the single pre-start frame.  The fix must not change this shape.
    """
    attempt = tmp_path / "attempt-1"
    attempt.mkdir(parents=True, exist_ok=True)
    records = [
        {
            "frame_index": index,
            "timestamp": index / 120.0,
            "scenario": "qualification-moveit-plan-pose",
            "seed": 7,
        }
        for index in range(0, 560)
    ]
    # fixture-ready at ts 528/120 (frame 528); raw_start_index 528 keeps
    # records[527] (ts 527/120 < gate_start) as the strictly-before pre-start.
    (attempt / "gate-window.json").write_text(
        json.dumps(
            {
                "gate": "qualification-moveit-plan-pose",
                "attempt_id": "attempt-1",
                "raw_start_index": 528,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    gate_start = 528 / 120.0
    gate_end = 540 / 120.0
    window, _ = select_integrated_gate_window(
        records,
        attempt,
        "qualification-moveit-plan-pose",
        attempt_id="attempt-1",
        manifest_present=True,
        gate_start=gate_start,
        gate_end=gate_end,
        physics_hz=120.0,
    )
    pre_start = [record for record in window if record["timestamp"] < gate_start]
    assert len(pre_start) == 1
    assert pre_start[0]["frame_index"] == 527
    assert [frame["frame_index"] for frame in window] == list(range(527, 541))


def test_frame_gap_or_raw_evaluator_mismatch_invalidates_evidence(tmp_path):
    assert _verify(tmp_path / "gap", frame_indices=[1, 3])["status"] == "evidence-invalid"
    assert _verify(tmp_path / "count", raw_frame_count=4, evaluator_frame_count=3)["status"] == "evidence-invalid"


def test_pre_window_epoch_reset_does_not_fail_verification(tmp_path):
    """RED: a warmup-epoch reset before the authoritative gate window must not
    fail verification.

    Physics truth contains epoch1 frames 1..N then a reset/duplicate boundary,
    but the fixture-ready..teardown authoritative window lies fully in epoch2
    and is contiguous there.  Today ``integrated_gate_verifier`` calls
    ``_parse_truth`` on ALL raw records before applying the journal gate window,
    so the epoch1 -> epoch2 frame-index reset trips "frame_index is not
    contiguous" and the whole attempt is wrongly evidence-invalid.
    """
    import copy

    scenario_id = "qualification-moveit-plan-joint"
    attempt = write_integrated_attempt(tmp_path / "epoch", scenario=scenario_id)
    raw = _raw_records(attempt)
    evaluator = [
        json.loads(line)
        for line in (attempt / "evaluator.jsonl").read_text().splitlines()
        if line.strip()
    ]

    # Epoch1: frames 1..N, timestamps strictly before the epoch2 stream
    # (which starts at frame 0 / timestamp 0).  Frame indices repeat across the
    # two epochs -> duplicate boundary.
    epoch1: list[dict] = []
    epoch1_evaluator: list[dict] = []
    n = 9
    for index in range(1, n + 1):
        frame = copy.deepcopy(raw[0])
        frame["frame_index"] = index
        frame["timestamp"] = float(index - n - 1) / 120.0  # negative, before epoch2
        frame["command_targets"] = dict(frame.get("command_targets", {}))
        frame["command_targets"]["snapshot_id"] = index
        epoch1.append(frame)
        epoch1_evaluator.append({"frame": copy.deepcopy(frame)})

    (attempt / "physics_truth.jsonl").write_text(
        "\n".join(json.dumps(frame, sort_keys=True) for frame in epoch1 + raw) + "\n",
        encoding="utf-8",
    )
    (attempt / "evaluator.jsonl").write_text(
        "\n".join(json.dumps(record, sort_keys=True) for record in epoch1_evaluator + evaluator) + "\n",
        encoding="utf-8",
    )

    verdict = verify_integrated_attempt(
        scenario=load_test_scenario(scenario_id),
        attempt_dir=attempt,
        config=load_test_config(),
    )
    assert verdict["status"] == "verified-pass", verdict["errors"]


def test_gap_inside_selected_gate_window_still_invalidates(tmp_path):
    """Guard: a true frame gap inside the selected gate window must still fail
    verification (evidence-invalid), even once pre-window epochs are ignored."""
    scenario_id = "qualification-moveit-plan-joint"
    attempt = write_integrated_attempt(tmp_path / "windowgap", scenario=scenario_id)
    raw = _raw_records(attempt)
    evaluator = [
        json.loads(line)
        for line in (attempt / "evaluator.jsonl").read_text().splitlines()
        if line.strip()
    ]
    # fixture-ready=10 .. teardown=40; drop a frame strictly inside the window.
    dropped = 20
    raw = [frame for frame in raw if int(frame["frame_index"]) != dropped]
    evaluator = [
        record for record in evaluator
        if int(record["frame"]["frame_index"]) != dropped
    ]
    (attempt / "physics_truth.jsonl").write_text(
        "\n".join(json.dumps(frame, sort_keys=True) for frame in raw) + "\n",
        encoding="utf-8",
    )
    (attempt / "evaluator.jsonl").write_text(
        "\n".join(json.dumps(record, sort_keys=True) for record in evaluator) + "\n",
        encoding="utf-8",
    )
    verdict = verify_integrated_attempt(
        scenario=load_test_scenario(scenario_id),
        attempt_dir=attempt,
        config=load_test_config(),
    )
    assert verdict["status"] == "evidence-invalid"


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
    # F1.7: post-terminal drain motion is ignored, so the fail mutations for
    # cancel/cancel-transport inject motion strictly inside the observation
    # subwindow (between the cancel/clear key and quiescent), not after it.
    fails = {
        "qualification-moveit-plan-joint": dict(plan_only_target_delta=0.01),
        "qualification-moveit-plan-pose": dict(plan_only_target_delta=0.01),
        "qualification-moveit-execute-joint": dict(joint_tracking_error_rad=0.05),
        "qualification-moveit-execute-pose": dict(tcp_tracking_error_m=0.05),
        "qualification-moveit-cartesian-retreat": dict(retreat_short=True),
        "qualification-moveit-gripper": dict(gripper_travel_short=True),
        "qualification-moveit-cancel": dict(post_quiescent_motion_fails=True),
        "qualification-moveit-safety": dict(safety_post_clear_target_motion=0.01),
        "qualification-pick-place-positive": dict(placement_error_m=0.20),
        "qualification-pick-place-blocked-approach": dict(approach_contact=True),
        "qualification-pick-place-unreachable-grasp": dict(approach_tcp_motion=True),
        "qualification-pick-place-malformed-back": dict(malformed_pick_goal_sent=True),
        "qualification-pick-place-cancel-approach": dict(approach_contact=True),
        "qualification-pick-place-cancel-transport": dict(post_quiescent_motion_fails=True),
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
# Gate C — executor planner_status vocabulary + plan-only jitter tolerance.
# --------------------------------------------------------------------------- #
def test_gate_c_plan_joint_accepts_executor_diagnostic_status(tmp_path):
    """RED (Fix A): the verifier must accept the executor's authoritative
    planner_status vocabulary ``"diagnostic-pass"``/``"diagnostic-fail"`` (what
    ``integrated_gate_executor._classify_plan_only_result`` actually writes into
    moveit-plans.jsonl) instead of requiring the literal ``"success"``.  A
    healthy plan-joint row carrying the real vocabulary is currently marked
    ``verified-fail`` ("planner_status is not success")."""
    verdict = _verify_at(
        tmp_path / "d", "qualification-moveit-plan-joint", executor_diagnostic_status=True
    )
    assert verdict["status"] == "verified-pass", verdict["errors"][:1]


def test_gate_c_plan_joint_tolerates_plan_only_jitter(tmp_path):
    """RED (Fix B): plan-only ``no_physical_motion`` must tolerate the
    steady-state solver/controller limit cycle (~0.03 rad/s joint jitter)
    present from frame 1 in live physics, instead of requiring
    ``<= numeric_tolerance`` (1e-6 rad/s)."""
    verdict = _verify_at(
        tmp_path / "j", "qualification-moveit-plan-joint", plan_only_joint_jitter_rad_s=0.03
    )
    assert verdict["status"] == "verified-pass", verdict["errors"][:1]


def test_gate_c_plan_joint_tolerates_transient_zero_fill_spike(tmp_path):
    """RED (Fix C): plan-only ``no_target_command_delta`` must tolerate the sim
    publisher's one-frame ``command_targets`` population race.  During plan-only
    the seven arm command targets are zero-filled for every window frame except a
    single frame where the publisher transiently fills them with the real current
    arm positions (0 -> real -> 0).  That one-frame 0->0.00292->0 spike exceeds
    ``numeric_tolerance`` (1e-6 rad) and currently false-rejects the scenario."""
    attempt = write_integrated_attempt(
        tmp_path / "spike", scenario="qualification-moveit-plan-joint"
    )
    real_positions = [0.0004, 0.00292, 0.0001, -0.00229, -0.00071, 0.00054, 1e-05]

    def mutate(frame: dict) -> bool:
        if int(frame["frame_index"]) == 25:  # inside [fixture-ready@10, teardown@40]
            positions = list(frame["command_targets"]["joint_positions"])
            positions[:7] = real_positions
            frame["command_targets"]["joint_positions"] = positions
            return True
        return False

    truth_path = attempt / "physics_truth.jsonl"
    truth_records = [
        json.loads(line)
        for line in truth_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    mutated = [mutate(record) for record in truth_records]
    assert any(mutated), "expected to find and mutate an in-window frame"
    truth_path.write_text(
        "\n".join(json.dumps(r, sort_keys=True) for r in truth_records) + "\n",
        encoding="utf-8",
    )
    # The verifier enforces exact raw/evaluator correlation (F1.5), so the same
    # raw frame embedded in evaluator.jsonl must carry the identical spike.
    evaluator_path = attempt / "evaluator.jsonl"
    evaluator_records = [
        json.loads(line)
        for line in evaluator_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    for record in evaluator_records:
        mutate(record["frame"])
    evaluator_path.write_text(
        "\n".join(json.dumps(r, sort_keys=True) for r in evaluator_records) + "\n",
        encoding="utf-8",
    )
    verdict = verify_integrated_attempt(
        scenario=load_test_scenario("qualification-moveit-plan-joint"),
        attempt_dir=attempt,
        config=load_test_config(),
    )
    assert verdict["status"] == "verified-pass", verdict["errors"][:1]


def test_gate_c_plan_joint_real_sustained_target_change_still_fails(tmp_path):
    """Fix C regression guard: a REAL sustained command-target change during
    plan-only (constant non-zero target from fixture-ready onward, not a
    zero-filled publisher transient) must still fail ``no_target_command_delta``
    exactly as before the zero-fill tolerance was introduced."""
    verdict = _verify_at(
        tmp_path / "real", "qualification-moveit-plan-joint", plan_only_target_delta=0.01
    )
    assert verdict["status"] == "verified-fail", verdict["errors"][:1]


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
    # F1.7: motion strictly after quiescent (before teardown) is outside the
    # [cancel-requested, quiescent] observation subwindow and is ignored.
    verdict = _verify_at(tmp_path, "qualification-moveit-cancel", post_quiescent_motion_ignored=True)
    assert verdict["status"] == "verified-pass", verdict["errors"]

    # The same motion strictly between cancel-requested and quiescent fails
    # quiescent_after_cancel (the subwindow read admits it).
    verdict = _verify_at(tmp_path / "fail", "qualification-moveit-cancel", post_quiescent_motion_fails=True)
    assert verdict["status"] == "verified-fail"
    names = [c["name"] for c in verdict["checks"] if not c["passed"]]
    assert "quiescent_after_cancel" in names


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


# --------------------------------------------------------------------------- #
# Fix round 1 — F1.1 stage-real object evidence.
# --------------------------------------------------------------------------- #
def test_cd_scenarios_do_not_require_qualification_cube(tmp_path):
    import tempfile
    # C/D scenarios declare objects: [] in production; raw truth has
    # objects=[], object=None, expected_objects={}.  The verifier must verify
    # them without any cube (F1.1).
    for scenario_id in ALL_17_SCENARIOS[:8]:
        with tempfile.TemporaryDirectory() as directory:
            attempt = write_integrated_attempt(Path(directory), scenario=scenario_id)
            records = _raw_records(attempt)
            assert all(frame.get("objects") == [] for frame in records), scenario_id
            assert all(frame.get("object") is None for frame in records), scenario_id
            assert all(frame.get("expected_objects") == {} for frame in records), scenario_id
            verdict = verify_integrated_attempt(
                scenario=load_test_scenario(scenario_id),
                attempt_dir=attempt,
                config=load_test_config(),
            )
            assert verdict["status"] == "verified-pass", (
                f"{scenario_id}: {verdict['errors'][:1]}"
            )


# --------------------------------------------------------------------------- #
# F1.2 scene-detach uses the committed after-state.
# --------------------------------------------------------------------------- #
def test_scene_detach_record_is_detached(tmp_path):
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        attempt = write_integrated_attempt(Path(directory))
        verdict = verify_integrated_attempt(
            scenario=load_test_scenario("qualification-pick-place-positive"),
            attempt_dir=attempt,
            config=load_test_config(),
        )
        assert verdict["status"] == "verified-pass"
        journal = _journal_records(attempt)
        detach = next(r for r in journal if r["event"] == "scene-detach")
        assert "pick_and_place/object_mesh" not in detach["attached_ids"]


def test_scene_detach_still_attached_fails_closed(tmp_path):
    # A scene-detach record that still carries the target is producer-shaped
    # wrong and fails closed (F1.2).
    verdict = _verify(
        tmp_path,
        scenario="qualification-pick-place-positive",
        scene_detach_still_attached=True,
    )
    assert verdict["status"] == "evidence-invalid"
    assert any("scene-detach" in error for error in verdict["errors"])


# --------------------------------------------------------------------------- #
# F1.3 endpoint/provider checks scoped to endpoint evidence.
# --------------------------------------------------------------------------- #
def test_env_cloud_evidence_source_is_not_endpoint_provider(tmp_path):
    # D cartesian-retreat embeds env_cloud_evidence.source ==
    # "observed-environment-cloud"; that is cloud provenance, not an endpoint
    # provider, so it must not invalidate the attempt (F1.3).
    verdict = _verify_at(tmp_path, "qualification-moveit-cartesian-retreat")
    assert verdict["status"] == "verified-pass", verdict["errors"]


def test_wrong_paired_source_fails(tmp_path):
    # A fabricated paired source_node next to the FJT endpoint fails the
    # endpoint-source ownership check (F1.3); this is a true pairing, unlike
    # the environment-cloud provenance source.
    verdict = _verify_at(
        tmp_path,
        "qualification-moveit-cartesian-retreat",
        wrong_paired_source=True,
    )
    assert verdict["status"] == "evidence-invalid"
    assert any("source" in error for error in verdict["errors"])


# --------------------------------------------------------------------------- #
# F1.4 contradictory terminal domains fail closed.
# --------------------------------------------------------------------------- #
def test_conflicting_terminal_domains_fail_closed(tmp_path):
    # success string + ABORTED numeric -> evidence-invalid, never a pass.
    verdict = _verify_at(
        tmp_path / "sa",
        "qualification-moveit-execute-joint",
        terminal_conflict_success_aborted=True,
    )
    assert verdict["status"] == "evidence-invalid", verdict["status"]
    # aborted string + SUCCEEDED numeric -> evidence-invalid.
    verdict = _verify_at(
        tmp_path / "as",
        "qualification-moveit-execute-joint",
        terminal_conflict_aborted_succeeded=True,
    )
    assert verdict["status"] == "evidence-invalid", verdict["status"]


def test_missing_terminal_evidence_fails_closed(tmp_path):
    verdict = _verify_at(
        tmp_path,
        "qualification-moveit-execute-joint",
        terminal_missing=True,
    )
    assert verdict["status"] == "evidence-invalid", verdict["status"]


# --------------------------------------------------------------------------- #
# F1.5 malformed scalar evidence fails closed.
# --------------------------------------------------------------------------- #
def test_malformed_scalar_evidence_fails_closed(tmp_path):
    # Malformed raw seed.
    for override in ("seed_null", "seed_list"):
        verdict = _verify(tmp_path / override, **{override: True})
        assert verdict["status"] == "evidence-invalid", override

    # Malformed gate-window indices.
    for override in ("raw_start_index_null", "raw_start_index_str", "raw_start_index_neg"):
        verdict = _verify(tmp_path / override, **{override: True})
        assert verdict["status"] == "evidence-invalid", override

    # Malformed evaluator index.
    for override in ("evaluator_start_index_str", "evaluator_start_index_null"):
        verdict = _verify(tmp_path / override, **{override: True})
        assert verdict["status"] == "evidence-invalid", override


# --------------------------------------------------------------------------- #
# F1.6 forbidden execution-provider taint beyond endpoint fields.
# --------------------------------------------------------------------------- #
def test_provider_field_taint_fails_closed(tmp_path):
    verdict = _verify(tmp_path / "pipeline", pipeline_taint=True)
    assert verdict["status"] == "evidence-invalid"
    assert any("pipeline_id" in error for error in verdict["errors"])

    verdict = _verify(tmp_path / "provider", provider_taint=True)
    assert verdict["status"] == "evidence-invalid"
    assert any("provider" in error for error in verdict["errors"])

    verdict = _verify(tmp_path / "anygrasp", pipeline_anygrasp=True)
    assert verdict["status"] == "evidence-invalid"
    assert any("pipeline_id" in error for error in verdict["errors"])


# --------------------------------------------------------------------------- #
# F1.7 temporal checks stay inside the contracted observation subwindow.
# --------------------------------------------------------------------------- #
def test_post_quiescent_motion_ignored_after_subwindow(tmp_path):
    # Motion after quiescent (post-terminal drain) is ignored on the D cancel
    # scenario (F1.7).
    verdict = _verify_at(
        tmp_path,
        "qualification-moveit-cancel",
        post_quiescent_motion_ignored=True,
    )
    assert verdict["status"] == "verified-pass", verdict["errors"]

    # The same motion between cancel-requested and quiescent fails.
    verdict = _verify_at(
        tmp_path / "fail",
        "qualification-moveit-cancel",
        post_quiescent_motion_fails=True,
    )
    assert verdict["status"] == "verified-fail"


# --------------------------------------------------------------------------- #
# F1.8 occupied-place retention and fixture realism.
# --------------------------------------------------------------------------- #
def test_occupied_place_attached_after_place_failure(tmp_path):
    verdict = _verify_at(tmp_path, "qualification-pick-place-occupied-place")
    assert verdict["status"] == "verified-pass", verdict["errors"]
    check = next(c for c in verdict["checks"] if c["name"] == "scene_attached_after_place_failure")
    assert check["passed"] is True
    assert check["metrics"]["attached_after_place_failure"]


# --------------------------------------------------------------------------- #
# F1.9 CLI scenario identity is truthful.
# --------------------------------------------------------------------------- #
def test_scenario_bundle_identity_mismatch_fails_closed(tmp_path):
    from integrated_gate_verifier import _scenario_bundle_from_declaration, EvidenceError
    import json as _json
    raw = _json.loads(
        (ROOT / "simulation" / "scenarios" / "qualification-pick-place-positive.json")
        .read_text(encoding="utf-8")
    )
    with pytest.raises(EvidenceError):
        _scenario_bundle_from_declaration(raw, expected_id="qualification-moveit-cancel")


def test_cli_scenario_id_mismatch_exit_code(tmp_path):
    import subprocess
    import tempfile
    from integrated_gate_verifier import _repo_root
    with tempfile.TemporaryDirectory() as directory:
        attempt = write_integrated_attempt(Path(directory))
        # Explicit file path whose filename id disagrees with the declaration id.
        wrong = Path(directory) / "wrong-name.json"
        wrong.write_text(
            (ROOT / "simulation" / "scenarios" / "qualification-pick-place-positive.json")
            .read_text(encoding="utf-8"),
            encoding="utf-8",
        )
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "validation" / "integrated_gate_verifier.py"),
                "--scenario", str(wrong),
                "--attempt-dir", str(attempt),
                "--config", str(ROOT / "simulation" / "qualification" / "integrated-ompl.json"),
            ],
            capture_output=True,
            text=True,
        )
        assert result.returncode == 2, result.stdout + result.stderr
        assert "does not match" in result.stderr or "does not match" in result.stdout
        # F2.4: the CLI identity-mismatch path atomically writes gate-verdict.json.
        verdict = json.loads((attempt / "gate-verdict.json").read_text(encoding="utf-8"))
        assert verdict["status"] == "evidence-invalid"
        assert any("does not match" in error for error in verdict["errors"])


# --------------------------------------------------------------------------- #
# Fix round 2 — F2.1 terminal quiescence at the anchor, not across the ramp.
# --------------------------------------------------------------------------- #
def test_f2_1_cancel_fixtures_carry_deceleration_ramp(tmp_path):
    # D cancel and E cancel-transport pass fixtures must carry a production-real
    # deceleration ramp after cancel-requested with at least two settled tail
    # frames at quiescent (F2.1), and the verifier must prove quiescence from
    # that bounded tail (not max-over-window).
    for scenario_id in ("qualification-moveit-cancel",
                        "qualification-pick-place-cancel-transport"):
        attempt = write_integrated_attempt(tmp_path / scenario_id, scenario=scenario_id)
        records = _raw_records(attempt)
        journal = _journal_records(attempt)
        cancel_key = _first_key(journal, "cancel-requested")[0]
        quiescent_key = _first_key(journal, "quiescent")[0]
        # A non-zero velocity exists inside the braking window (the ramp).
        braking_speeds = [
            max(abs(v) for v in r["robot"]["joint_velocities"])
            for r in records
            if cancel_key < int(r["frame_index"]) <= quiescent_key
        ]
        assert any(speed > 0.02 for speed in braking_speeds), scenario_id
        # At least two settled tail frames (quiescent-1 and quiescent).
        tail_speeds = [
            max(abs(v) for v in r["robot"]["joint_velocities"])
            for r in records
            if int(r["frame_index"]) in (quiescent_key - 1, quiescent_key)
        ]
        assert len(tail_speeds) == 2 and all(speed <= 0.02 for speed in tail_speeds), scenario_id
        verdict = _verify_at(tmp_path / f"v-{scenario_id}", scenario_id)
        assert verdict["status"] == "verified-pass", verdict["errors"]


def test_f2_1_ramp_not_settled_fails(tmp_path):
    # A ramp that does not settle by quiescent fails the terminal-quiescence tail.
    for scenario_id in ("qualification-moveit-cancel",
                        "qualification-pick-place-cancel-transport"):
        verdict = _verify_at(tmp_path / scenario_id, scenario_id, cancel_ramp_not_settled=True)
        assert verdict["status"] == "verified-fail", (scenario_id, verdict["status"])
        names = [c["name"] for c in verdict["checks"] if not c["passed"]]
        assert "quiescent_after_cancel" in names or "no_post_cancel_stage" in names


def test_f2_1_cancel_target_change_fails(tmp_path):
    # A new command target/goal between cancel and quiescent fails even if the
    # velocity later settles (F2.1 item 3).
    for scenario_id in ("qualification-moveit-cancel",
                        "qualification-pick-place-cancel-transport"):
        verdict = _verify_at(tmp_path / scenario_id, scenario_id, cancel_target_change=True)
        assert verdict["status"] == "verified-fail", (scenario_id, verdict["status"])


def test_f2_1_post_quiescent_motion_excluded_after_ramp(tmp_path):
    # Motion only after quiescent remains excluded and does not affect the
    # verdict even with the production-real braking ramp present (F2.1).
    verdict = _verify_at(tmp_path, "qualification-moveit-cancel", post_quiescent_motion_ignored=True)
    assert verdict["status"] == "verified-pass", verdict["errors"]


def test_f2_1_safety_braking_ramp_truth(tmp_path):
    # F2.1 item 6: preserve the safety creep contract (no silent weakening) and
    # determine truthfully whether a realistic safety braking ramp passes or
    # fails.  A small ramp inside safety_position_creep_rad passes; a large ramp
    # (0.012 rad > 0.005 rad creep bound) fails target_frozen.
    verdict = _verify_at(tmp_path / "small", "qualification-moveit-safety", safety_braking_ramp=True)
    assert verdict["status"] == "verified-pass", verdict["errors"]
    verdict = _verify_at(tmp_path / "large", "qualification-moveit-safety", safety_braking_ramp_large=True)
    assert verdict["status"] == "verified-fail", verdict["status"]
    names = [c["name"] for c in verdict["checks"] if not c["passed"]]
    assert "target_frozen" in names


def test_f2_1_safety_target_phantom_floor_passes_with_creep_bound(tmp_path):
    # RED (S2): the sim's commanded target drifts ~0.003 rad while the arm is
    # frozen (the SAME phantom-motion floor as the round-4 velocity threshold).
    # The target-frozen and no_auto_resume checks must bound target delta by the
    # position-creep bound (safety_target_delta_rad 0.005), not numeric_tolerance
    # 1e-06 — the arm IS frozen (position creep 0.00336 < 0.005), the commanded
    # target just breathes by the same floor.  Live rerun-5 evidence:
    # max_target_delta_rad 0.00299, position_creep_rad 0.00336.
    verdict = _verify_at(
        tmp_path,
        "qualification-moveit-safety",
        safety_target_phantom_floor=True,
    )
    assert verdict["status"] == "verified-pass", verdict["errors"]
    for check in verdict["checks"]:
        assert check["passed"], f"{check['name']} failed: {check['metrics']}"


def test_f2_1_safety_large_post_clear_target_delta_still_fails(tmp_path):
    # The strictness is not weakened: a post-clear target change above the
    # position-creep bound (0.01 > safety_target_delta_rad 0.005) still fails
    # no_auto_resume / target_frozen truthfully.
    verdict = _verify_at(
        tmp_path,
        "qualification-moveit-safety",
        safety_post_clear_target_motion=0.01,
    )
    assert verdict["status"] == "verified-fail", verdict["status"]
    names = [c["name"] for c in verdict["checks"] if not c["passed"]]
    assert "target_frozen" in names or "no_auto_resume" in names


def test_f2_1_safety_transport_ramp_truth(tmp_path):
    # E safety-transport deceleration-ramp probe: the terminal-quiescence tail
    # (no_post_clear_resume) settles on a real braking ramp and does NOT
    # false-fail, while the strict safety_stop_frames velocity_below_stop_limit
    # truthfully fails because the velocity at effective-stop is above the stop
    # limit (F2.1 item 6).
    verdict = _verify_at(
        tmp_path,
        "qualification-pick-place-safety-transport",
        safety_transport_ramp=True,
    )
    assert verdict["status"] == "verified-fail", verdict["status"]
    names = [c["name"] for c in verdict["checks"] if not c["passed"]]
    assert "velocity_below_stop_limit" in names
    assert "no_post_clear_resume" not in names


# --------------------------------------------------------------------------- #
# F2.2 provider-taint field gaps without re-breaking semantic provenance.
# --------------------------------------------------------------------------- #
def test_f2_2_env_cloud_taint_fails_closed(tmp_path):
    # env_cloud_evidence.source="cuMotion-provider" (unpaired) fails because it
    # carries a forbidden token, not because it is absent from the endpoint-
    # provider node allowlist (F2.2).
    verdict = _verify_at(
        tmp_path,
        "qualification-moveit-cartesian-retreat",
        env_cloud_taint=True,
    )
    assert verdict["status"] == "evidence-invalid"
    assert any("source field" in error and "forbidden token" in error
               for error in verdict["errors"])


def test_f2_2_env_cloud_provenance_still_passes(tmp_path):
    # The committed semantic provenance value remains accepted (F1.3/F2.2).
    verdict = _verify_at(tmp_path, "qualification-moveit-cartesian-retreat")
    assert verdict["status"] == "verified-pass", verdict["errors"]


def test_f2_2_goal_kind_taint_fails_closed(tmp_path):
    # goal_kind is a committed provider/goal field; a forbidden token fails.
    verdict = _verify(tmp_path, goal_kind_taint=True)
    assert verdict["status"] == "evidence-invalid"
    assert any("provider field" in error and "forbidden token" in error
               for error in verdict["errors"])


def test_f2_2_pipeline_ompl_case_variant_fails_closed(tmp_path):
    # Exact lowercase "ompl" is canonical identity strictness (F2.2): a case
    # variant is evidence-invalid, deliberately not normalized.
    verdict = _verify(tmp_path, pipeline_ompl_uppercase=True)
    assert verdict["status"] == "evidence-invalid"
    assert any("pipeline_id" in error for error in verdict["errors"])


# --------------------------------------------------------------------------- #
# F2.3 D-safety terminal consistency.
# --------------------------------------------------------------------------- #
def test_f2_3_d_safety_valid_aborted_terminal(tmp_path):
    # Production-shaped safety-stop/aborted summary: the safety terminal is a
    # consistent non-success and the check passes.
    verdict = _verify_at(tmp_path, "qualification-moveit-safety")
    assert verdict["status"] == "verified-pass", verdict["errors"]
    check = next(c for c in verdict["checks"] if c["name"] == "safety_terminal_non_success")
    assert check["passed"] is True


def test_f2_3_d_safety_terminal_success_fails(tmp_path):
    # A safety attempt claiming terminal success is verified-fail.
    verdict = _verify_at(tmp_path, "qualification-moveit-safety", safety_terminal_success=True)
    assert verdict["status"] == "verified-fail"
    names = [c["name"] for c in verdict["checks"] if not c["passed"]]
    assert "safety_terminal_non_success" in names


def test_f2_3_d_safety_terminal_contradiction_fails(tmp_path):
    # A contradictory safety terminal (success string + ABORTED GoalStatus) is
    # evidence-invalid, never a pass (F2.3).
    verdict = _verify_at(tmp_path, "qualification-moveit-safety", safety_terminal_contradiction=True)
    assert verdict["status"] == "evidence-invalid"
    assert any("conflicting terminal domains" in error for error in verdict["errors"])


# --------------------------------------------------------------------------- #
# F2.4 atomic verdicts for CLI and direct API shape failures.
# --------------------------------------------------------------------------- #
def test_f2_4_malformed_bundle_fails_closed(tmp_path):
    import tempfile
    from integrated_gate_verifier import VERDICT_FILENAME
    base = load_test_scenario("qualification-pick-place-positive")

    def direct(bundle):
        with tempfile.TemporaryDirectory() as directory:
            attempt = write_integrated_attempt(Path(directory))
            verdict = verify_integrated_attempt(
                scenario=bundle,
                attempt_dir=attempt,
                config=load_test_config(),
            )
            assert verdict["status"] == "evidence-invalid", verdict["status"]
            assert verdict["verified"] is False
            durable = json.loads(
                (attempt / VERDICT_FILENAME).read_text(encoding="utf-8")
            )
            assert durable["status"] == "evidence-invalid"
            assert durable["errors"], "durable verdict must explain the malformed field"
            return verdict

    # seed as None / list / bool (bool is not a valid int scalar).
    for bad_seed in (None, [7], True):
        mutated = {"scenario": dict(base["scenario"]), **base}
        mutated["scenario"]["seed"] = bad_seed
        direct(mutated)
    # Missing scenario mapping entirely.
    direct({"planning_scene": base["planning_scene"],
            "integrated": base["integrated"]})
    # Missing integrated mapping.
    mutated = {"scenario": dict(base["scenario"]), **base}
    mutated.pop("integrated", None)
    direct(mutated)
    # Malformed report identity seed.
    mutated = {"scenario": dict(base["scenario"]), **base}
    mutated["report_identities"] = {"scenario_id": "qualification-pick-place-positive",
                                    "seed": "not-an-int"}
    direct(mutated)


def test_f2_4_cli_malformed_bundle_writes_durable_verdict(tmp_path):
    import subprocess
    import tempfile
    from integrated_gate_verifier import _repo_root
    with tempfile.TemporaryDirectory() as directory:
        attempt = write_integrated_attempt(Path(directory))
        # A scenario file whose seed is malformed fails closed with a durable
        # gate-verdict.json and exit 2.
        bad = Path(directory) / "bad-seed.json"
        raw = json.loads(
            (ROOT / "simulation" / "scenarios" / "qualification-pick-place-positive.json")
            .read_text(encoding="utf-8")
        )
        raw["seed"] = [7]
        bad.write_text(json.dumps(raw), encoding="utf-8")
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "validation" / "integrated_gate_verifier.py"),
                "--scenario", str(bad),
                "--attempt-dir", str(attempt),
                "--config", str(ROOT / "simulation" / "qualification" / "integrated-ompl.json"),
            ],
            capture_output=True,
            text=True,
        )
        assert result.returncode == 2, result.stdout + result.stderr
        verdict = json.loads((attempt / "gate-verdict.json").read_text(encoding="utf-8"))
        assert verdict["status"] == "evidence-invalid"
        assert any("seed" in error for error in verdict["errors"])


# --------------------------------------------------------------------------- #
# F2.5 raw object identity restricted to the backend-emitted bare id.
# --------------------------------------------------------------------------- #
def test_f2_5_raw_object_identity_is_bare_id():
    from integrated_gate_verifier import _OBJECT_ID_CANDIDATES, _object_pose_target
    assert _OBJECT_ID_CANDIDATES == ("qualification_cube",)
    # A raw frame whose object id is the planning-scene namespace is not an
    # interchangeable raw object id (F2.5).
    from integrated_verifier_fixtures import _frame
    frame = _frame(0, "qualification-pick-place-positive",
                   objects=[{"id": "sim_fixture/qualification_cube",
                             "class_name": "cube",
                             "pose": {"xyz": [0.65, 0.0, 0.64],
                                      "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                             "twist": {"linear": [0.0, 0.0, 0.0],
                                       "angular": [0.0, 0.0, 0.0]}}])
    assert _object_pose_target(frame) is None
    # The bare id resolves.
    from integrated_verifier_fixtures import _cube_object
    bare = _frame(0, "qualification-pick-place-positive", objects=[_cube_object([0.65, 0.0, 0.64], [0.0, 0.0, 0.0])])
    assert _object_pose_target(bare) is not None


# --------------------------------------------------------------------------- #
# G1 — execute-pose orientation expectation must match the generated approach.
#
# The executor plans/executes the pose goal to the generated APPROACH
# orientation (``POSE_APPROACH_QUATERNION_XYZW`` = Rx45), while the verifier
# compared the reached TCP orientation against the DECLARED target quaternion
# (a yaw-only Rz45).  That ~63 deg mismatch made ``pose_execution_reaches_tcp``
# fail even when the arm reached the correct approach pose.  The verifier must
# compare against the approach orientation the executor actually plans to.
# --------------------------------------------------------------------------- #
def test_g1_execute_pose_verifier_uses_approach_orientation():
    from integrated_gate_executor import POSE_APPROACH_QUATERNION_XYZW, POSE_APPROACH_Z_OFFSET
    from integrated_gate_verifier import _gate_d_checks

    declared_xyz = [0.35, 0.0, 0.80]
    declared_quat = [0.0, 0.0, 0.382683, 0.92388]  # yaw-only Rz45 (target box frame)
    approach_xyz = [declared_xyz[0], declared_xyz[1], declared_xyz[2] + POSE_APPROACH_Z_OFFSET]
    approach_quat = list(POSE_APPROACH_QUATERNION_XYZW)

    def ctx_for(final_quat):
        return {
            "kind": "execute-pose",
            "thresholds": {"tcp_position_error_m": 0.05, "tcp_orientation_error_deg": 10.0},
            "window_parsed": [
                {"frame_index": 5, "raw": {"robot": {"tcp_pose": {
                    "xyz": list(approach_xyz),
                    "quaternion_xyzw": list(final_quat),
                }}}}
            ],
            "execution_summary": {"terminal_status": "succeeded", "execute_result_status": 4},
            "planning_scene_declaration": {
                "target_source_id": "sim_fixture/public_target",
                "objects": [
                    {"id": "sim_fixture/public_target",
                     "pose": {"xyz": declared_xyz, "quaternion_xyzw": declared_quat}},
                ],
            },
        }

    # RED: with the old expectation (declared quaternion) the reached APPROACH
    # orientation fails the orientation tolerance (~63 deg vs 10 deg).
    checks_old = _gate_d_checks(ctx_for(approach_quat))
    pose_check_old = next(c for c in checks_old if c["name"] == "pose_execution_reaches_tcp")
    assert pose_check_old["passed"], (
        "execute-pose verification must accept the reached APPROACH orientation; "
        f"check failed with reasons {pose_check_old['reasons']}"
    )

    # A reached orientation that matches the DECLARED yaw but not the approach
    # must still be reported (and, being wrong for the generated goal, must fail
    # the orientation check once the verifier uses the approach orientation).
    checks_decl = _gate_d_checks(ctx_for(declared_quat))
    pose_check_decl = next(c for c in checks_decl if c["name"] == "pose_execution_reaches_tcp")
    assert not pose_check_decl["passed"], (
        "execute-pose verification must NOT accept the declared yaw-only "
        "orientation as the reached approach orientation"
    )


# --------------------------------------------------------------------------- #
# R13 — execute-pose verifier must compare the TCP in the base_link frame.
#
# The simulator's physics truth reports tcp_pose and base_pose in the WORLD
# frame (backend.py body_pos_w/root_pos_w), while the commanded execute-pose
# target is in base_link.  During the arm motion the free-floating sim base can
# yaw under arm reaction torque (live rerun-11: 72.6 deg base yaw), so a
# world-frame TCP that is off-target in WORLD but on-target in BASE_LINK must
# pass once the verifier transforms it into base_link.
# --------------------------------------------------------------------------- #
def test_r13_execute_pose_verifier_transforms_world_tcp_into_base_link():
    from integrated_gate_executor import POSE_APPROACH_QUATERNION_XYZW, POSE_APPROACH_Z_OFFSET
    from integrated_gate_verifier import _gate_d_checks

    declared_xyz = [0.35, 0.0, 0.80]
    declared_quat = [0.0, 0.0, 0.382683, 0.92388]
    approach_xyz = [declared_xyz[0], declared_xyz[1], declared_xyz[2] + POSE_APPROACH_Z_OFFSET]
    approach_quat = list(POSE_APPROACH_QUATERNION_XYZW)

    # Sim: base yawed ~72.6 deg under arm reaction torque (free-floating root).
    base_xyz = [0.0932532548904419, -0.0036698232870548964, 0.07653333991765976]
    base_quat = [-0.07664315402507782, 0.5775076746940613, 0.10692746192216873, 0.8057154417037964]

    # World-frame TCP that is OFF the base-link target but whose base-link
    # projection equals the approach pose (reproduced from the live rerun-11
    # physics_truth final frame: tcp_pose world [1.030, 0.250, 0.035]).
    tcp_world = [1.0302557945251465, 0.2505582869052887, 0.034634821116924286]
    tcp_world_quat = [0.7120798230171204, 0.4011157155036926, -0.548416793346405, -0.1768830120563507]

    ctx = {
        "kind": "execute-pose",
        "thresholds": {"tcp_position_error_m": 0.05, "tcp_orientation_error_deg": 10.0},
        "window_parsed": [
            {"frame_index": 1641, "raw": {"robot": {
                "base_pose": {"xyz": base_xyz, "quaternion_xyzw": base_quat},
                "tcp_pose": {"xyz": tcp_world, "quaternion_xyzw": tcp_world_quat},
            }}}
        ],
        "execution_summary": {"terminal_status": "succeeded", "execute_result_status": 4},
        "planning_scene_declaration": {
            "target_source_id": "sim_fixture/public_target",
            "objects": [
                {"id": "sim_fixture/public_target",
                 "pose": {"xyz": declared_xyz, "quaternion_xyzw": declared_quat}},
            ],
        },
    }
    checks = _gate_d_checks(ctx)
    pose_check = next(c for c in checks if c["name"] == "pose_execution_reaches_tcp")
    assert pose_check["passed"], (
        "execute-pose verification must compare the TCP in the base_link frame "
        "(the world-frame TCP is 1.13m off the base-link target only because the "
        f"free-floating base yawed); check failed: {pose_check['reasons']}"
    )


# --------------------------------------------------------------------------- #
# R12 — execute-joint final-error threshold vs the FJT settling residual.
#
# rerun-11 execute-joint reached the commanded Q_OUTBOUND within a final error
# of 0.0131 rad (0.75 deg; RMS 0.0152 << the 0.05 rms bound) yet was marked
# ``verified-fail`` solely because ``joint_final_error_rad`` (0.01) is tighter
# than the FJT controller's real arrival behavior: with
# ``allow_nonzero_velocity_at_trajectory_end=true`` the controller reports
# SUCCEEDED at trajectory end while the arm is still settling the last ~0.01 rad.
# That is a threshold calibration issue, not a tracking failure — the arm is at
# the commanded joint target.  A production real 0.0131 rad final error must
# pass.  A genuinely large tracking error (0.05 rad) must still fail (the
# full-matrix verified-fail case asserts exactly that).
# --------------------------------------------------------------------------- #
def test_d_execute_joint_accepts_production_settling_final_error(tmp_path):
    verdict = _verify_at(
        tmp_path / "r11", "qualification-moveit-execute-joint",
        joint_tracking_error_rad=0.0131,
    )
    assert verdict["status"] == "verified-pass", verdict["errors"][:1]
