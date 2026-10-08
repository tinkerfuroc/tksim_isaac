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
    pose_error,
    run_reset,
    scenario_spawn,
)

SPAWN = Pose2D(-2.0, -2.0, 0.0)
TRUE_SPAWN = Pose2D(-1.93, -2.00, math.radians(-1.3))


class FakeClient:
    def __init__(self, *, truth=TRUE_SPAWN, active=None, moving_polls=0,
                 reset_result=(True, ""), amcl=None, amcl_times_out=False,
                 servers=None):
        self.calls = []
        self.t = 0.0
        self._truth = truth
        self._active = dict(active or {})
        self._moving_polls = moving_polls
        self._reset_result = reset_result
        self._amcl = truth if amcl is None else amcl
        self._amcl_times_out = amcl_times_out
        self._servers = set(NAV_ACTIONS + ARM_ACTIONS) if servers is None else set(servers)

    def names(self):
        return [call[0] for call in self.calls]

    def cancel_all_goals(self, action):
        self.calls.append(("cancel", action))
        if action not in self._servers:
            return False
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
        return self._truth

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
    assert names == ["reset", "truth", "seed", "clear", "home"]
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
