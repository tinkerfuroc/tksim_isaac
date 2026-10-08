import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.gpsr_run_reset import (  # noqa: E402
    ARM_ACTIONS,
    NAV_ACTIONS,
    Pose2D,
    ResetError,
    ResetOutcome,
    main,
    outcome_record,
    pose_error,
    run_reset,
    scenario_spawn,
    start_pose_spread,
)

BENCH = str(ROOT / "simulation/scenarios/gpsr-rcw2026-bench.json")
SPAWN = Pose2D(-2.0, -2.0, 0.0)
TRUE_SPAWN = Pose2D(-1.93, -2.00, math.radians(-1.3))


class FakeClient:
    def __init__(self, *, truth=TRUE_SPAWN, truths=None, active=None, moving_polls=0,
                 reset_result=(True, ""), amcl=None, amcl_times_out=False,
                 servers=None, discovered=(), rejects=(), slow_cancel=(),
                 session="stack-1"):
        self.calls = []
        self.t = 0.0
        # truth_pose() returns these in turn (before, after reset, after
        # settle); the last one repeats.
        self._truths = list(truths) if truths is not None else [truth]
        self._active = dict(active or {})
        self._moving_polls = moving_polls
        self._reset_result = reset_result
        self._amcl = self._truths[-1] if amcl is None else amcl
        self._amcl_times_out = amcl_times_out
        self._discovered = list(discovered)
        self._servers = (set(NAV_ACTIONS + ARM_ACTIONS) | set(discovered)
                         if servers is None else set(servers))
        # Acknowledges the cancel but keeps executing (arm_api start_grasp).
        self._rejects = set(rejects)
        # The first cancel is lost (discovery race); a resend lands.
        self._slow_cancel = set(slow_cancel)
        self._session = session

    def names(self):
        return [call[0] for call in self.calls]

    def list_actions(self):
        return list(self._discovered)

    def stack_session(self):
        return self._session

    def cancel_all_goals(self, action):
        self.calls.append(("cancel", action))
        if action not in self._servers:
            return False
        if action in self._slow_cancel:
            self._slow_cancel.discard(action)
        elif action not in self._rejects:
            self._active[action] = 0
        return True

    def active_goals(self, action):
        return self._active.get(action, 0)

    def base_speed(self):
        if self._moving_polls > 0:
            self._moving_polls -= 1
            return (0.3, 0.0)
        return (0.0, 0.0)

    def truth_pose(self):
        self.calls.append(("truth",))
        return self._truths.pop(0) if len(self._truths) > 1 else self._truths[0]

    def reset_state(self):
        self.calls.append(("reset",))
        return self._reset_result

    def seed_localization(self, pose):
        self.calls.append(("seed", pose))
        return None if self._amcl_times_out else self._amcl

    def clear_costmaps(self):
        self.calls.append(("clear",))

    def home_arm_and_head(self):
        self.calls.append(("home",))

    def sleep(self, seconds):
        self.t += seconds

    def now(self):
        return self.t


def quiet(_message):
    pass


def test_pose_error_wraps_yaw():
    distance, degrees = pose_error(Pose2D(0, 0, math.radians(179)), Pose2D(0.3, 0.4, math.radians(-179)))
    assert distance == pytest.approx(0.5)
    assert degrees == pytest.approx(2.0)


def test_scenario_spawn_reads_robot_initial_pose(tmp_path):
    path = tmp_path / "s.json"
    path.write_text(json.dumps({"robot": {"initial_pose": [-2.0, -2.0, 0.5]}}))
    assert scenario_spawn(path) == Pose2D(-2.0, -2.0, 0.5)


def test_scenario_spawn_without_pose_raises(tmp_path):
    path = tmp_path / "s.json"
    path.write_text(json.dumps({"robot": {}}))
    with pytest.raises(ResetError, match="initial_pose"):
        scenario_spawn(path)


def test_bench_scenario_has_a_spawn():
    assert scenario_spawn(ROOT / "simulation/scenarios/gpsr-rcw2026-bench.json") == SPAWN


def test_steps_run_in_order():
    client = FakeClient(active={"/navigate_to_pose": 1})
    outcome = run_reset(client, SPAWN, log=quiet)
    names = [n for n in client.names() if n != "cancel"]
    # truth before the reset, right after it, and after the settle.
    assert names == ["truth", "reset", "truth", "seed", "clear", "home", "truth"]
    assert client.names()[: len(NAV_ACTIONS + ARM_ACTIONS)] == ["cancel"] * len(NAV_ACTIONS + ARM_ACTIONS)
    assert outcome.truth == TRUE_SPAWN
    assert outcome.error_m == pytest.approx(0.07, abs=1e-6)


def test_cancels_arm_and_gripper_actions_too():
    client = FakeClient(active={action: 1 for action in ARM_ACTIONS})
    run_reset(client, SPAWN, log=quiet)
    cancelled = {call[1] for call in client.calls if call[0] == "cancel"}
    assert set(NAV_ACTIONS) | set(ARM_ACTIONS) <= cancelled


def test_missing_action_server_is_skipped():
    client = FakeClient(servers=["/navigate_to_pose"])
    logged = []
    run_reset(client, SPAWN, log=logged.append)
    assert "reset" in client.names()
    assert any("/go_to_approach" in line for line in logged)


def test_goal_that_survives_cancel_blocks_the_reset():
    client = FakeClient(active={"/navigate_to_pose": 1}, servers=[])
    with pytest.raises(ResetError, match="/navigate_to_pose"):
        run_reset(client, SPAWN, idle_timeout_s=2.0, log=quiet)
    assert "reset" not in client.names()


def test_waits_for_base_to_stop_before_reset():
    client = FakeClient(moving_polls=5)
    run_reset(client, SPAWN, poll_s=0.2, log=quiet)
    # 5 moving polls, then a full 1.0 s stationary hold.
    assert client.t >= 5 * 0.2 + 1.0 - 1e-9


def test_base_that_never_stops_blocks_the_reset():
    client = FakeClient(moving_polls=10**6)
    with pytest.raises(ResetError, match="still moving"):
        run_reset(client, SPAWN, idle_timeout_s=3.0, log=quiet)
    assert "reset" not in client.names()


def test_reset_service_failure_carries_detail():
    client = FakeClient(reset_result=(False, "idle action/controller observation timed out"))
    with pytest.raises(ResetError, match="idle action/controller observation timed out"):
        run_reset(client, SPAWN, log=quiet)
    assert "seed" not in client.names()


def test_reset_failure_after_stop_tells_operator_to_restart():
    detail = "boom; simulation paused; reset stop remains asserted"
    client = FakeClient(reset_result=(False, detail))
    with pytest.raises(ResetError, match="restart the stack"):
        run_reset(client, SPAWN, log=quiet)


def test_dead_write_is_detected_from_truth():
    client = FakeClient(truth=Pose2D(1.49, -3.72, 0.4))
    with pytest.raises(ResetError, match="did not reach physics"):
        run_reset(client, SPAWN, log=quiet)
    assert "seed" not in client.names()


def test_amcl_that_never_answers_fails():
    client = FakeClient(amcl_times_out=True)
    with pytest.raises(ResetError, match="AMCL"):
        run_reset(client, SPAWN, log=quiet)
    assert "home" not in client.names()


def test_amcl_that_disagrees_with_truth_fails():
    client = FakeClient(amcl=Pose2D(-1.40, -2.00, 0.0))
    with pytest.raises(ResetError, match="AMCL"):
        run_reset(client, SPAWN, log=quiet)


def test_goal_that_appears_during_reset_fails():
    class LateGoal(FakeClient):
        def home_arm_and_head(self):
            super().home_arm_and_head()
            self._active["/navigate_to_pose"] = 1
    with pytest.raises(ResetError, match="started during reset"):
        run_reset(LateGoal(), SPAWN, log=quiet)



class ClosingClient(FakeClient):
    closed = False

    def close(self):
        self.closed = True


def test_main_success_appends_record(tmp_path):
    client = ClosingClient()
    record = tmp_path / "r.jsonl"
    code = main(["--scenario", BENCH, "--record", str(record)], client_factory=lambda: client)
    assert code == 0
    assert client.closed
    (line,) = record.read_text().splitlines()
    data = json.loads(line)
    assert data["truth"][:2] == pytest.approx([-1.93, -2.00])
    assert data["spawn"] == [-2.0, -2.0, 0.0]


def test_main_failure_exits_1_with_reason_and_closes(tmp_path, capsys):
    client = ClosingClient(truth=Pose2D(1.49, -3.72, 0.4))
    code = main(["--scenario", BENCH, "--record", str(tmp_path / "r.jsonl")],
                client_factory=lambda: client)
    assert code == 1
    assert client.closed
    assert "did not reach physics" in capsys.readouterr().err
    assert not (tmp_path / "r.jsonl").exists()


def test_main_client_construction_failure_exits_1(tmp_path, capsys):
    def broken():
        raise ResetError("/reset_simulation not available")
    code = main(["--scenario", BENCH, "--record", str(tmp_path / "r.jsonl")], client_factory=broken)
    assert code == 1
    assert "/reset_simulation not available" in capsys.readouterr().err


def test_start_pose_spread_is_worst_against_first():
    outcomes = [Pose2D(-1.93, -2.00, 0.0), Pose2D(-1.93, -2.03, 0.0), Pose2D(-1.90, -2.00, math.radians(3))]
    records = [outcome_record(ResetOutcome(p, p, 0.0, 0.0), SPAWN) for p in outcomes]
    distance, degrees = start_pose_spread(records)
    assert distance == pytest.approx(0.03)
    assert degrees == pytest.approx(3.0)


def test_summarize_fails_outside_tolerance(tmp_path, capsys):
    path = tmp_path / "r.jsonl"
    rows = [outcome_record(ResetOutcome(p, p, 0.0, 0.0), SPAWN)
            for p in (Pose2D(-1.93, -2.0, 0.0), Pose2D(-1.80, -2.0, 0.0))]
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    assert main(["--summarize", str(path)]) == 1
    assert "0.130 m" in capsys.readouterr().out


def test_summarize_passes_inside_tolerance(tmp_path):
    path = tmp_path / "r.jsonl"
    rows = [outcome_record(ResetOutcome(p, p, 0.0, 0.0), SPAWN)
            for p in (Pose2D(-1.93, -2.0, 0.0), Pose2D(-1.93, -2.01, math.radians(1)))]
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    assert main(["--summarize", str(path)]) == 0


def test_cancels_every_action_on_the_graph():
    client = FakeClient(discovered=["/start_grasp", "/follow_path"])
    run_reset(client, SPAWN, log=quiet)
    cancelled = {call[1] for call in client.calls if call[0] == "cancel"}
    assert {"/start_grasp", "/follow_path"} <= cancelled


def test_upstream_goal_that_ignores_cancel_blocks_the_reset():
    client = FakeClient(discovered=["/start_grasp"], active={"/start_grasp": 1},
                        rejects=["/start_grasp"])
    with pytest.raises(ResetError, match="/start_grasp"):
        run_reset(client, SPAWN, idle_timeout_s=2.0, log=quiet)
    assert "reset" not in client.names()


def test_lost_cancel_is_resent():
    client = FakeClient(active={"/navigate_to_pose": 1}, slow_cancel=["/navigate_to_pose"])
    run_reset(client, SPAWN, log=quiet)
    assert client.calls.count(("cancel", "/navigate_to_pose")) >= 2
    assert "reset" in client.names()


def test_goal_accepted_just_before_reset_is_not_reset_through():
    class LateArmGoal(FakeClient):
        def truth_pose(self):
            if "truth" not in self.names():
                self._active["/xarm7_traj_controller/follow_joint_trajectory"] = 1
            return super().truth_pose()
    client = LateArmGoal()
    with pytest.raises(ResetError, match="before the STATE reset"):
        run_reset(client, SPAWN, log=quiet)
    assert "reset" not in client.names()


def test_outcome_records_the_settled_pose_and_the_pose_before():
    far = Pose2D(1.49, -3.72, 0.4)
    settled = Pose2D(-1.94, -2.00, 0.0)
    client = FakeClient(truths=[far, TRUE_SPAWN, settled])
    outcome = run_reset(client, SPAWN, log=quiet)
    assert outcome.before == far
    assert outcome.truth == settled


def test_start_off_this_sessions_reference_fails():
    # 0.14 m from the scenario pose passes the coarse dead-write bound but
    # is 0.07 m from where this stack's first reset put the base.
    client = FakeClient(truths=[Pose2D(1.0, -3.0, 0.0), Pose2D(-1.86, -2.0, 0.0)])
    with pytest.raises(ResetError, match="first reset"):
        run_reset(client, SPAWN, reference=Pose2D(-1.93, -2.0, 0.0), log=quiet)


def test_start_within_this_sessions_reference_passes():
    client = FakeClient(truths=[Pose2D(1.0, -3.0, 0.0), Pose2D(-1.93, -2.01, math.radians(1))])
    run_reset(client, SPAWN, reference=Pose2D(-1.93, -2.0, 0.0), log=quiet)


def _write_records(path, rows):
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))


def test_main_checks_against_first_record_of_the_same_session(tmp_path, capsys):
    record = tmp_path / "r.jsonl"
    first = outcome_record(ResetOutcome(Pose2D(-1.80, -2.0, 0.0), Pose2D(-1.80, -2.0, 0.0), 0.0, 0.0,
                                        session="stack-1"), SPAWN)
    _write_records(record, [first])
    code = main(["--scenario", BENCH, "--record", str(record)],
                client_factory=lambda: ClosingClient(session="stack-1"))
    assert code == 1
    assert "first reset" in capsys.readouterr().err


def test_main_ignores_records_from_another_session(tmp_path):
    record = tmp_path / "r.jsonl"
    old = outcome_record(ResetOutcome(Pose2D(-1.80, -2.0, 0.0), Pose2D(-1.80, -2.0, 0.0), 0.0, 0.0,
                                      session="stack-0"), SPAWN)
    _write_records(record, [old])
    code = main(["--scenario", BENCH, "--record", str(record)],
                client_factory=lambda: ClosingClient(session="stack-1"))
    assert code == 0
    last = json.loads(record.read_text().splitlines()[-1])
    assert last["session"] == "stack-1"


def test_summarize_compares_within_each_session(tmp_path):
    record = tmp_path / "r.jsonl"
    rows = [outcome_record(ResetOutcome(p, p, 0.0, 0.0, session=s), SPAWN)
            for p, s in ((Pose2D(-1.93, -2.0, 0.0), "a"), (Pose2D(-1.83, -2.0, 0.0), "b"),
                         (Pose2D(-1.83, -2.01, 0.0), "b"))]
    _write_records(record, rows)
    assert main(["--summarize", str(record)]) == 0
