import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.sheet_events import MilestoneEvent, JudgeEvent, load_run_telemetry  # noqa: E402

TRAJ = "traj-fixture-001"

SCAN_FAILURE_FEEDBACK = (
    'ScanForGeneralist for persons pointing to the left failed status=1: '
    'no matches for "persons pointing to the left" via vlm'
)
POSTCONDITION_FEEDBACK = "postcondition unmet: counted(persons pointing to the left) (UNKNOWN)"


def _event(event_type, occurred_at, payload, sequence):
    return {
        "schema": "tinker.gpsr.telemetry",
        "schema_version": 1,
        "event_id": f"evt-{sequence}",
        "trajectory_id": TRAJ,
        "trace_id": "trace-fixture",
        "source_id": "gpsr-orchestrator:test",
        "sequence": sequence,
        "occurred_at": occurred_at,
        "event_type": event_type,
        "payload": payload,
    }


def _node_state(node_id, status, feedback):
    return {
        "id": node_id,
        "node_id": node_id,
        "status": status,
        "feedback": feedback,
    }


def _build_events():
    events = []
    seq = 0

    def add(event_type, occurred_at, payload):
        nonlocal seq
        seq += 1
        events.append(_event(event_type, occurred_at, payload, seq))

    # tree.generated rev0: names for every node referenced below.
    add(
        "tree.generated",
        "2026-08-28T10:00:00.000000Z",
        {
            "tree_revision": 0,
            "nodes": [
                {"id": "n_mat0", "node_id": "n_mat0", "name": "materialise:0:0:0"},
                {"id": "n_goto", "node_id": "n_goto", "name": "goto target"},
                {"id": "n_mat1", "node_id": "n_mat1", "name": "materialise:0:0:1"},
                {"id": "n_scan", "node_id": "n_scan", "name": "scan to count"},
                {"id": "n_mat3", "node_id": "n_mat3", "name": "materialise:0:0:3"},
                {"id": "n_announce", "node_id": "n_announce", "name": "announce vlm count"},
                {"id": "n_keepalive", "node_id": "n_keepalive", "name": "nav keepalive 0"},
                {"id": "n_post", "node_id": "n_post", "name": "postcondition gate:0:0"},
                {"id": "n_barrier", "node_id": "n_barrier", "name": "supervisor barrier:0:0"},
                {"id": "n_tuck", "node_id": "n_tuck", "name": "tuck arm before goto"},
                {"id": "n_running", "node_id": "n_running", "name": "announce running check"},
            ],
        },
    )

    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:00.500000Z",
        {"nodes": [_node_state("n_mat0", "SUCCESS", "step 0: goto({'location': 'bedroom'})")]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:01.000000Z",
        {"nodes": [_node_state("n_goto", "SUCCESS", "goal accepted :) [BtNode_GotoAction/goto target]")]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:01.500000Z",
        {
            "nodes": [
                _node_state(
                    "n_mat1",
                    "SUCCESS",
                    "step 1: count({'object': 'persons pointing to the left'})",
                )
            ]
        },
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:02.000000Z",
        {"nodes": [_node_state("n_scan", "FAILURE", SCAN_FAILURE_FEEDBACK)]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:02.500000Z",
        {"nodes": [_node_state("n_mat3", "SUCCESS", "step 3: announce({})")]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:03.000000Z",
        {"nodes": [_node_state("n_announce", "SUCCESS", "Finished announcing 0 persons.")]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:04.000000Z",
        {"nodes": [_node_state("n_keepalive", "SUCCESS", "Finished announcing I am trying to go to the destination.")]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:05.000000Z",
        {"nodes": [_node_state("n_post", "FAILURE", POSTCONDITION_FEEDBACK)]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:06.000000Z",
        {"nodes": [_node_state("n_tuck", "SUCCESS", "MOCK: auto-complete finished")]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:07.000000Z",
        {"nodes": [_node_state("n_running", "RUNNING", "MOCK: auto-completing (1/2)")]},
    )

    add(
        "tree.generated",
        "2026-08-28T10:00:08.000000Z",
        {"tree_revision": 1, "nodes": []},
    )

    add(
        "run.finished",
        "2026-08-28T10:00:09.000000Z",
        {"trajectory_id": TRAJ, "status": "complete"},
    )

    return events


def _write_events(tmp_path, events, run_name="run-000"):
    run_dir = tmp_path / run_name
    debug_dir = run_dir / "debug" / "gpsr-20260828T100000000000Z-fixture01"
    debug_dir.mkdir(parents=True)
    events_file = debug_dir / "events.jsonl"
    with events_file.open("w") as f:
        for event in events:
            f.write(json.dumps(event) + "\n")
    return run_dir


def test_load_run_telemetry_extracts_milestones_and_judge_events(tmp_path):
    run_dir = _write_events(tmp_path, _build_events())

    milestones, judge_events, meta = load_run_telemetry(run_dir)

    assert [m.kind for m in milestones] == ["NAV", "VISION", "AUDIO", "MANIP"]
    assert [m.name for m in milestones] == [
        "goto target",
        "scan to count",
        "announce vlm count",
        "tuck arm before goto",
    ]
    assert [m.status for m in milestones] == ["SUCCESS", "FAILURE", "SUCCESS", "SUCCESS"]
    # goto milestone: the preceding materialise:0:0:0 SUCCESS carries
    # "step 0: goto({'location': 'bedroom'})" -- info should end with the
    # step-context suffix.
    assert milestones[0].info == (
        "goal accepted :) [BtNode_GotoAction/goto target] | plan-step 0 goto: location=bedroom"
    )
    assert milestones[0].info.endswith("| plan-step 0 goto: location=bedroom")
    # scan milestone: materialise:0:0:1 carries the count step's context.
    assert milestones[1].info == (
        SCAN_FAILURE_FEEDBACK + " | plan-step 1 count: object=persons pointing to the left"
    )
    # announce milestone: materialise:0:0:3 has empty params -- suffix has
    # no trailing ": k=v" clause.
    assert milestones[2].info == "0 persons | plan-step 3 announce"
    # tuck: no materialise SUCCESS follows announce's, so the step-3
    # announce context is still the "most recent" one in effect.
    assert milestones[3].info == "MOCK: auto-complete finished | plan-step 3 announce"
    assert milestones[0].wall == "2026-08-28T10:00:01.000000Z"

    assert all(isinstance(m, MilestoneEvent) for m in milestones)

    assert [j.kind for j in judge_events] == ["POSTCONDITION", "REPLAN"]
    assert judge_events[0].name == "postcondition gate:0:0"
    assert judge_events[0].status == "FAILURE"
    # Judge events get the same active step-context suffix as milestones.
    assert judge_events[0].info == POSTCONDITION_FEEDBACK + " | plan-step 3 announce"
    assert judge_events[1].name == "replan"
    assert judge_events[1].status == "SUCCESS"
    assert judge_events[1].info == "tree revision 1"
    assert judge_events[1].wall == "2026-08-28T10:00:08.000000Z"

    assert all(isinstance(j, JudgeEvent) for j in judge_events)

    assert meta["trajectory_id"] == TRAJ
    assert meta["tree_revisions"] == 1
    assert meta["run_finished"] == {"trajectory_id": TRAJ, "status": "complete"}


def test_keepalive_and_running_are_excluded(tmp_path):
    run_dir = _write_events(tmp_path, _build_events())
    milestones, _judge_events, _meta = load_run_telemetry(run_dir)
    names = [m.name for m in milestones]
    assert "nav keepalive 0" not in names
    assert "announce running check" not in names


def test_missing_debug_dir_returns_empty_shapes(tmp_path):
    run_dir = tmp_path / "run-no-debug"
    run_dir.mkdir()

    milestones, judge_events, meta = load_run_telemetry(run_dir)

    assert milestones == []
    assert judge_events == []
    assert meta == {}


def test_missing_run_dir_returns_empty_shapes(tmp_path):
    milestones, judge_events, meta = load_run_telemetry(tmp_path / "does-not-exist")
    assert (milestones, judge_events, meta) == ([], [], {})


def test_garbage_line_is_skipped_rest_still_parsed(tmp_path):
    events = _build_events()
    run_dir = tmp_path / "run-garbage"
    debug_dir = run_dir / "debug" / "gpsr-20260828T100000000000Z-fixture02"
    debug_dir.mkdir(parents=True)
    events_file = debug_dir / "events.jsonl"
    with events_file.open("w") as f:
        f.write("{this is not valid json\n")
        for event in events:
            f.write(json.dumps(event) + "\n")
        f.write("\n")  # trailing blank line should also be tolerated

    milestones, judge_events, meta = load_run_telemetry(run_dir)

    assert [m.kind for m in milestones] == ["NAV", "VISION", "AUDIO", "MANIP"]
    assert [j.kind for j in judge_events] == ["POSTCONDITION", "REPLAN"]
    assert meta["tree_revisions"] == 1


def test_newest_gpsr_debug_dir_is_selected(tmp_path):
    run_dir = tmp_path / "run-multi"
    older = run_dir / "debug" / "gpsr-20260101T000000000000Z-aaaaaaaa"
    newer = run_dir / "debug" / "gpsr-20260828T100000000000Z-bbbbbbbb"
    older.mkdir(parents=True)
    newer.mkdir(parents=True)

    (older / "events.jsonl").write_text(
        json.dumps(_event("tree.generated", "2026-01-01T00:00:00Z", {"tree_revision": 0, "nodes": []}, 1)) + "\n"
    )
    with (newer / "events.jsonl").open("w") as f:
        for event in _build_events():
            f.write(json.dumps(event) + "\n")

    milestones, judge_events, meta = load_run_telemetry(run_dir)

    assert len(milestones) == 4
    assert meta["trajectory_id"] == TRAJ


def test_reset_and_clear_correction_excluded_from_judge_events(tmp_path):
    # Authoritative ruling (overrides the brief's literal "reset correction"
    # wording): any node whose name begins "reset " or "clear " is
    # blackboard housekeeping, not a judge-worthy correction attempt --
    # exclude the whole reset/clear family, not just the one literal name.
    events = [
        _event(
            "tree.generated",
            "2026-08-28T11:00:00.000000Z",
            {
                "tree_revision": 0,
                "nodes": [
                    {"id": "n_clear", "node_id": "n_clear", "name": "clear correction",
                     "type": "BtNode_BlackboardSet"},
                    {"id": "n_reset", "node_id": "n_reset", "name": "reset correction",
                     "type": "BtNode_BlackboardSet"},
                    {"id": "n_real", "node_id": "n_real", "name": "apply correction",
                     "type": "BtNode_MoveArmSingle"},
                ],
            },
            1,
        ),
        _event("tree.node_states_changed", "2026-08-28T11:00:01.000000Z",
               {"nodes": [_node_state("n_clear", "SUCCESS", "")]}, 2),
        _event("tree.node_states_changed", "2026-08-28T11:00:02.000000Z",
               {"nodes": [_node_state("n_reset", "SUCCESS", "")]}, 3),
        _event("tree.node_states_changed", "2026-08-28T11:00:03.000000Z",
               {"nodes": [_node_state("n_real", "FAILURE", "correction attempt failed")]}, 4),
    ]
    run_dir = _write_events(tmp_path, events, run_name="run-correction")

    _milestones, judge_events, _meta = load_run_telemetry(run_dir)

    kinds_by_name = {j.name: j.kind for j in judge_events}
    assert "clear correction" not in kinds_by_name
    assert "reset correction" not in kinds_by_name
    assert kinds_by_name["apply correction"] == "CORRECTION"


def test_blackboard_write_typed_arm_nodes_excluded_from_milestones(tmp_path):
    # "arm scan" / "arm nav" / "arm orbbec look" are BtNode_WriteToBlackboard
    # bookkeeping leaves (they just flag intent on the blackboard) -- they
    # must not leak into MANIP milestones even though their names match the
    # MANIP "arm" pattern. "tuck arm before goto" is a real
    # BtNode_MoveArmSingle ActionHandler and must still survive as MANIP.
    events = [
        _event(
            "tree.generated",
            "2026-08-28T12:00:00.000000Z",
            {
                "tree_revision": 0,
                "nodes": [
                    {"id": "n_arm_scan", "node_id": "n_arm_scan", "name": "arm scan",
                     "type": "BtNode_WriteToBlackboard"},
                    {"id": "n_arm_nav", "node_id": "n_arm_nav", "name": "arm nav",
                     "type": "BtNode_WriteToBlackboard"},
                    {"id": "n_arm_look", "node_id": "n_arm_look", "name": "arm orbbec look",
                     "type": "BtNode_WriteToBlackboard"},
                    {"id": "n_tuck", "node_id": "n_tuck", "name": "tuck arm before goto",
                     "type": "BtNode_MoveArmSingle"},
                ],
            },
            1,
        ),
        _event("tree.node_states_changed", "2026-08-28T12:00:01.000000Z",
               {"nodes": [_node_state("n_arm_scan", "SUCCESS", "Success writing to namespace")]}, 2),
        _event("tree.node_states_changed", "2026-08-28T12:00:02.000000Z",
               {"nodes": [_node_state("n_arm_nav", "SUCCESS", "Success writing to namespace")]}, 3),
        _event("tree.node_states_changed", "2026-08-28T12:00:03.000000Z",
               {"nodes": [_node_state("n_arm_look", "SUCCESS", "Success writing to namespace")]}, 4),
        _event("tree.node_states_changed", "2026-08-28T12:00:04.000000Z",
               {"nodes": [_node_state("n_tuck", "SUCCESS", "MOCK: auto-complete finished")]}, 5),
    ]
    run_dir = _write_events(tmp_path, events, run_name="run-blackboard")

    milestones, _judge_events, _meta = load_run_telemetry(run_dir)

    names = [m.name for m in milestones]
    assert "arm scan" not in names
    assert "arm nav" not in names
    assert "arm orbbec look" not in names
    assert names == ["tuck arm before goto"]
    assert milestones[0].kind == "MANIP"


def test_non_utf8_events_file_returns_empty_shapes(tmp_path):
    run_dir = tmp_path / "run-badenc"
    debug_dir = run_dir / "debug" / "gpsr-20260828T100000000000Z-fixture03"
    debug_dir.mkdir(parents=True)
    events_file = debug_dir / "events.jsonl"
    # 0xff is not valid in UTF-8 (or any of Python's default text codecs'
    # first bytes here) -- Path.read_text() on this raises UnicodeDecodeError.
    events_file.write_bytes(b'{"event_type": "run.finished", \xff\xfe"payload": {}}\n')

    milestones, judge_events, meta = load_run_telemetry(run_dir)

    assert (milestones, judge_events, meta) == ([], [], {})


def test_materialise_step_context_stamped_onto_milestones_and_judge_events(tmp_path):
    run_dir = _write_events(tmp_path, _build_events(), run_name="run-stepctx")

    milestones, judge_events, _meta = load_run_telemetry(run_dir)

    all_names = [m.name for m in milestones] + [j.name for j in judge_events]
    assert not any(name.startswith("materialise:") for name in all_names)

    by_name = {m.name: m for m in milestones}
    assert by_name["goto target"].info.endswith("| plan-step 0 goto: location=bedroom")
    assert by_name["scan to count"].info.endswith(
        "| plan-step 1 count: object=persons pointing to the left"
    )
    assert by_name["announce vlm count"].info.endswith("| plan-step 3 announce")

    postcondition = next(j for j in judge_events if j.kind == "POSTCONDITION")
    assert postcondition.info.endswith("| plan-step 3 announce")


def test_malformed_materialise_feedback_leaves_info_unchanged(tmp_path):
    events = [
        _event(
            "tree.generated",
            "2026-08-28T13:00:00.000000Z",
            {
                "tree_revision": 0,
                "nodes": [
                    {"id": "n_mat_bad", "node_id": "n_mat_bad", "name": "materialise:0:0:0"},
                    {"id": "n_goto", "node_id": "n_goto", "name": "goto target"},
                ],
            },
            1,
        ),
        _event(
            "tree.node_states_changed",
            "2026-08-28T13:00:01.000000Z",
            {"nodes": [_node_state("n_mat_bad", "SUCCESS", "step X: ???")]},
            2,
        ),
        _event(
            "tree.node_states_changed",
            "2026-08-28T13:00:02.000000Z",
            {"nodes": [_node_state("n_goto", "SUCCESS", "goal accepted")]},
            3,
        ),
    ]
    run_dir = _write_events(tmp_path, events, run_name="run-malformed-materialise")

    milestones, _judge_events, _meta = load_run_telemetry(run_dir)

    assert [m.name for m in milestones] == ["goto target"]
    # A materialise SUCCESS whose feedback doesn't parse never raises and
    # never stamps a bogus step-context suffix onto later events.
    assert milestones[0].info == "goal accepted"


def test_third_generation_at_revision_zero_is_a_replan(tmp_path):
    # Measured on the real corpus (289 tree.generated events, 2026-08-28):
    # tree_revision is ALWAYS 0 -- the orchestrator never increments it --
    # and every run emits exactly two generations (skeleton, then plan
    # materialisation). A third generation is therefore the only telemetry
    # signature a regenerated tree can leave, and must produce a REPLAN
    # judge event even at revision 0. Two generations must NOT.
    two_gens = [
        _event("tree.generated", "2026-01-01T00:00:00Z",
               {"tree_revision": 0, "nodes": []}, 1),
        _event("tree.generated", "2026-01-01T00:00:10Z",
               {"tree_revision": 0,
                "nodes": [{"id": "a", "node_id": "a", "name": "x"}]}, 2),
    ]
    run_dir = _write_events(tmp_path, two_gens)
    _m, judge_events, meta = load_run_telemetry(run_dir)
    assert [j for j in judge_events if j.kind == "REPLAN"] == []
    assert meta["tree_generations"] == 2

    three_gens = two_gens + [
        _event("tree.generated", "2026-01-01T00:00:20Z",
               {"tree_revision": 0,
                "nodes": [{"id": "a", "node_id": "a", "name": "x"}]}, 3),
    ]
    run_dir2 = _write_events(tmp_path, three_gens, run_name="run-001")
    _m2, judge_events2, meta2 = load_run_telemetry(run_dir2)
    replans = [j for j in judge_events2 if j.kind == "REPLAN"]
    assert len(replans) == 1
    assert replans[0].info == "tree regenerated #2 (1 nodes)"


GENERALIST_SCAN_FEEDBACK = (
    'ScanForGeneralist for brown pudding box failed status=1: '
    'no matches for "brown pudding box" via vlm_sam'
)


def test_generalist_scan_failure_is_vision_milestone(tmp_path):
    # "generalist scan" doesn't start with "scan " -- the word "scan" is
    # mid-name. It's the leaf whose FAILURE explains most find_object
    # failures, and must surface as a VISION milestone on the contact
    # sheet, not silently drop.
    events = [
        _event(
            "tree.generated",
            "2026-08-29T09:00:00.000000Z",
            {
                "tree_revision": 0,
                "nodes": [
                    {"id": "n_gscan", "node_id": "n_gscan", "name": "generalist scan",
                     "type": "BtNode_ScanForGeneralist"},
                ],
            },
            1,
        ),
        _event("tree.node_states_changed", "2026-08-29T09:00:01.000000Z",
               {"nodes": [_node_state("n_gscan", "FAILURE", GENERALIST_SCAN_FEEDBACK)]}, 2),
    ]
    run_dir = _write_events(tmp_path, events, run_name="run-generalist-scan")

    milestones, _judge_events, _meta = load_run_telemetry(run_dir)

    assert len(milestones) == 1
    milestone = milestones[0]
    assert milestone.kind == "VISION"
    assert milestone.status == "FAILURE"
    assert "no matches" in milestone.info


def test_object_scan_plus_verify_is_vision_milestone(tmp_path):
    # "object scan+verify" is another vision detection leaf whose name
    # doesn't start with "scan " -- must still classify as VISION.
    events = [
        _event(
            "tree.generated",
            "2026-08-29T09:05:00.000000Z",
            {
                "tree_revision": 0,
                "nodes": [
                    {"id": "n_osv", "node_id": "n_osv", "name": "object scan+verify",
                     "type": "BtNode_ScanAndVerify"},
                ],
            },
            1,
        ),
        _event("tree.node_states_changed", "2026-08-29T09:05:01.000000Z",
               {"nodes": [_node_state("n_osv", "SUCCESS", "verified brown pudding box")]}, 2),
    ]
    run_dir = _write_events(tmp_path, events, run_name="run-object-scan-verify")

    milestones, _judge_events, _meta = load_run_telemetry(run_dir)

    assert len(milestones) == 1
    assert milestones[0].kind == "VISION"
    assert milestones[0].status == "SUCCESS"


def test_arm_scan_blackboard_write_still_excluded_from_milestones(tmp_path):
    # "arm scan" is a BtNode_WriteToBlackboard bookkeeping leaf, same
    # family as "arm nav" / "arm orbbec look" -- the \bscan\b VISION match
    # must not override the bookkeeping-type exclusion, which is checked
    # first in _classify_milestone.
    events = [
        _event(
            "tree.generated",
            "2026-08-29T09:10:00.000000Z",
            {
                "tree_revision": 0,
                "nodes": [
                    {"id": "n_arm_scan", "node_id": "n_arm_scan", "name": "arm scan",
                     "type": "BtNode_WriteToBlackboard"},
                ],
            },
            1,
        ),
        _event("tree.node_states_changed", "2026-08-29T09:10:01.000000Z",
               {"nodes": [_node_state("n_arm_scan", "SUCCESS", "Success writing to namespace")]}, 2),
    ]
    run_dir = _write_events(tmp_path, events, run_name="run-arm-scan-blackboard")

    milestones, _judge_events, _meta = load_run_telemetry(run_dir)

    assert milestones == []
