# GPSR Bench Trial Reset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every tier-2 bench trial starts with the base physically back at spawn, AMCL re-localized there, and no Nav2, approach or arm goal still running.

**Architecture:** `scripts/gpsr-run-reset` (the `--reset-cmd` every battery passes to `bench/tier2.py`) becomes a thin wrapper around a new `tools/gpsr_run_reset.py`. The tool runs these steps in order:
1. Cancel every goal on the Nav2, approach and arm/gripper action servers.
2. Wait for the base to stop.
3. Call the sim's existing scoped STATE reset (`/reset_simulation`, scope 2). It is implemented in `simulation/tinker_sim_isaac/scoped_reset.py` and writes the robot root pose through the live PhysX articulation view. That is the "method that works on the Fabric articulation" the report asks for, unlike the dead USD-layer `SetEntityState` write removed in `5fa9186`.
4. Check the result against ground truth (`/sim/truth/robot_state`).
5. Re-seed AMCL with `/initialpose`, clear the costmaps, re-home the arm and head as the script does today, and settle.

Odometry is integrated from wheel *velocities* (`base_facade.py`), and the reset zeroes them, so odom does not jump. Only `map→odom` needs the AMCL re-seed.

**Tech Stack:** ROS 2 Humble rclpy (system Python 3.10), `simulation_interfaces/srv/ResetSimulation`, `action_msgs/srv/CancelGoal`, `nav2_msgs/srv/ClearEntireCostmap`, `tinker_sim_interfaces/msg/RobotTruth`, pytest.

**Spec:** `docs/issue_reports/2026-10-08-gpsr-nav-bug-report.md` §3 (3a), with context from §1a/§1c.

## Global Constraints

- Done when: "Every trial's first pose is within 0.05 m and 2° of spawn, and no Nav2 goal is active when a trial starts." (spec §3a)
- Cancel all Nav2 goals (`navigate_to_pose`, `go_to_approach`) in the reset step. (spec §3a)
- Do not use `SetEntityState`/`GetEntityState` for the robot: they read and write the USD layer, which Isaac Lab never reads back for the articulation (`5fa9186`).
- Use reset scope **2 (STATE) only**, never TIME (1) or DEFAULT (0). TIME rewinds the public ROS clock under a running Nav2/AMCL/TF stack. DEFAULT deletes every spawned entity (comment in `scripts/gpsr-run-reset`).
- Do not touch `tk25_decision` sources. The uncommitted `start_outside_arena=False` / skip-command-point change there belongs to its owner and stays as is.
- The total reset must finish under `tier2._reset`'s 180 s timeout. The wrapper caps it at 170 s.
- The working tree has unrelated uncommitted changes. Stage **only** the files each task names. `scripts/gpsr-run-reset` already has an uncommitted rclpy rewrite, which Task 3 supersedes, so committing the whole file is intended. The uncommitted `simulation/tinker_sim_isaac/scoped_reset.py` change (referee-hold release before restore) is a **prerequisite** but is not this plan's to commit: ask the user before Task 1 whether it will be committed separately.
- Unit tests run with: `source /opt/ros/humble/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest <files> -q` from the repo root.

## Review Focus

1. **Orchestrator killed mid-arm-motion.** An active arm or gripper action makes `IdleReset` reject with "active action". Expected: the tool cancels those actions too, then resets. Pinned in Task 2 (`test_cancels_arm_and_gripper_actions_too`).
2. **Reset fails after the stop was asserted.** The sim stays paused with the reset stop held, so every later reset fails too. Expected: an error that says so and tells the operator to restart the stack; tier2 then halts after 3 ERRORs. Pinned in Task 2 (`test_reset_failure_after_stop_tells_operator_to_restart`).
3. **Silent dead write.** The service returns OK but the base did not move, which is the exact failure `5fa9186` uncovered. Expected: a loud failure, not a PASS. Pinned in Task 2 (`test_dead_write_is_detected_from_truth`).
4. **A config without `go_to_approach` or without the gripper action server.** Expected: skipped with a log line, not a failure. Pinned in Task 2 (`test_missing_action_server_is_skipped`).
5. **Stale truth or AMCL sample.** A pre-reset message read as post-reset would hide a dead write or a failed re-seed. Expected: the adapter only accepts messages received after the call (and an AMCL pose stamped after the seed). Pinned live in Task 4 step 3: drive away, then reset.

---

### Task 1: Live probe: does scope-2 STATE reset work on the GPSR stack? (gate)

`docs/native-scoped-reset.md` says the scoped reset has CPU validation only. Both native attempts (native06, native07) failed, and "active navigation ... workflows are not qualified". Everything after this task depends on it working on the full GPSR composite. **This task writes no code.** Dispatch it to the `sim-runner` agent.

**Files:** none (evidence goes to Task 4's doc update)

- [ ] **Step 1: Bring the stack up**

```bash
cd /home/tinker/tinker-sim/6.0.1
./scripts/gpsr-stack up --scenario gpsr-rcw2026-bench --seed 0 --manipulation mock
```

Probe shell environment (same as the batteries):

```bash
cd /home/tinker/tinker-sim/6.0.1
set -a; source .deployment.env; set +a
export ROS_DOMAIN_ID=${GPSR_ROS_DOMAIN_ID:-42}
source /opt/ros/humble/setup.bash; source .ros-vendor/humble/local_setup.bash
source /home/tinker/tk25_ws/install/setup.bash; source ros2_ws/install/setup.bash
```

- [ ] **Step 2: Confirm the preconditions `IdleReset` waits on**

```bash
ros2 service type /reset_simulation               # simulation_interfaces/srv/ResetSimulation
ros2 topic info /xarm7_traj_controller/follow_joint_trajectory/_action/status   # Publisher count >= 1
ros2 topic info /xarm_gripper/gripper_action/_action/status                     # Publisher count >= 1
ros2 topic echo --once /sim/hardware/safety_stop   # data: false
ros2 topic echo --once /sim/truth/robot_state --field base_pose
```

Expected: both action status topics have a publisher under `--manipulation mock`. If either is missing, `idle()` can never pass, so record which one and stop (gate fails).

- [ ] **Step 3: Drive the base away, then reset**

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -0.7, y: -3.0}, orientation: {w: 1.0}}}}"
ros2 topic echo --once /sim/truth/robot_state --field base_pose   # note pose A (should be near -0.7,-3.0)
time ros2 service call /reset_simulation simulation_interfaces/srv/ResetSimulation "{scope: 2}"
ros2 topic echo --once /sim/truth/robot_state --field base_pose   # pose B
```

Expected: the response prints `result=1` (`Result.RESULT_OK` is 1; 0 is FEATURE_UNSUPPORTED, which means the adapter was not installed) with an empty `error_message`, and pose B within 0.05 m / 2° of the stack-up pose (≈ −1.93, −2.00, yaw −1.3°). Pose B must differ from pose A.

- [ ] **Step 4: Check the stack survived**

```bash
ros2 topic hz /clock            # still ticking after the 1.2 s pause
ros2 topic echo --once /sim/hardware/safety_stop   # data: false (released)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -1.0, y: -2.0}, orientation: {w: 1.0}}}}"
```

Before the last goal, publish `/initialpose` at pose B by hand (x, y, yaw from pose B, `frame_id: map`). Expected: Nav2 drives normally and the arm controller is active (`ros2 control list_controllers`).

- [ ] **Step 5: Repeat steps 3–4 twice more, then tear down**

```bash
./scripts/gpsr-stack down
```

**Gate.** Three of three resets return OK with pose B within tolerance, and the stack keeps working → continue to Task 2. Any failure → stop and report the exact service error and `02-bridge.log` / sim log excerpts to the user. Do not start Task 2. The fallback (one trial per stack session, a `gpsr-stack down/up` per trial) is a different design and needs its own plan.

---

### Task 2: Reset sequence core (`run_reset`) with a fake client

**Files:**
- Create: `tools/gpsr_run_reset.py`
- Test: `tests/test_gpsr_run_reset.py`

**Interfaces:**
- Produces (used by Task 3):
  - `Pose2D(x: float, y: float, yaw: float)` (frozen dataclass, yaw in rad)
  - `pose_error(a: Pose2D, b: Pose2D) -> tuple[float, float]` (metres, degrees ≥ 0)
  - `scenario_spawn(path: Path) -> Pose2D`
  - `ResetError(RuntimeError)`
  - `ResetOutcome(truth: Pose2D, amcl: Pose2D, error_m: float, error_deg: float)`
  - `ResetClient` Protocol: `cancel_all_goals(action) -> bool`, `active_goals(action) -> int`, `base_speed() -> tuple[float, float]`, `truth_pose() -> Pose2D`, `reset_state() -> tuple[bool, str]`, `seed_localization(pose) -> Optional[Pose2D]`, `clear_costmaps() -> None`, `home_arm_and_head() -> None`, `sleep(seconds) -> None`, `now() -> float`
  - `run_reset(client, spawn, *, idle_timeout_s=20.0, poll_s=0.2, settle_s=9.0, log=...) -> ResetOutcome`
  - constants `NAV_ACTIONS`, `ARM_ACTIONS`, `SCOPE_STATE = 2`, `ACTIVE_GOAL_STATUSES = (1, 2, 3)`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_gpsr_run_reset.py
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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `source /opt/ros/humble/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gpsr_run_reset.py -q`
Expected: collection error, `ModuleNotFoundError: No module named 'tools.gpsr_run_reset'`

- [ ] **Step 3: Write the implementation**

```python
# tools/gpsr_run_reset.py
"""Per-trial GPSR bench reset: stop motion, put the base back at spawn,
re-localize, re-home arm and head.

Run by ``scripts/gpsr-run-reset`` (the ``--reset-cmd`` of every tier-2
battery). The base is restored with the sim's scoped STATE reset
(``/reset_simulation`` scope 2, ``simulation/tinker_sim_isaac/scoped_reset.py``),
which writes the root pose through the live PhysX articulation view. Never
use SetEntityState for the robot: it writes a USD layer the articulation
never reads back (commit 5fa9186).
"""
from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Protocol

NAV_ACTIONS = ("/navigate_to_pose", "/go_to_approach")
# IdleReset rejects a reset while either of these has an active goal.
ARM_ACTIONS = (
    "/xarm7_traj_controller/follow_joint_trajectory",
    "/xarm_gripper/gripper_action",
)
ACTIVE_GOAL_STATUSES = (1, 2, 3)  # ACCEPTED, EXECUTING, CANCELING
SCOPE_STATE = 2
# The scenario pose is the spawn *request*; the settled spawn sits ~0.07 m /
# 1.3 deg from it. This bound catches a dead write; the 0.05 m / 2 deg
# trial-to-trial bound is checked across a batch with --summarize.
SPAWN_TOLERANCE_M = 0.15
SPAWN_TOLERANCE_DEG = 5.0
AMCL_TOLERANCE_M = 0.15
AMCL_TOLERANCE_DEG = 5.0
STATIONARY_LINEAR_M_S = 0.01
STATIONARY_ANGULAR_RAD_S = 0.02
STATIONARY_HOLD_S = 1.0


class ResetError(RuntimeError):
    """The reset could not establish a clean trial start."""


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ResetOutcome:
    truth: Pose2D
    amcl: Pose2D
    error_m: float
    error_deg: float


class ResetClient(Protocol):
    def cancel_all_goals(self, action: str) -> bool: ...
    def active_goals(self, action: str) -> int: ...
    def base_speed(self) -> tuple[float, float]: ...
    def truth_pose(self) -> Pose2D: ...
    def reset_state(self) -> tuple[bool, str]: ...
    def seed_localization(self, pose: Pose2D) -> Optional[Pose2D]: ...
    def clear_costmaps(self) -> None: ...
    def home_arm_and_head(self) -> None: ...
    def sleep(self, seconds: float) -> None: ...
    def now(self) -> float: ...


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_error(a: Pose2D, b: Pose2D) -> tuple[float, float]:
    distance = math.hypot(a.x - b.x, a.y - b.y)
    dyaw = math.atan2(math.sin(a.yaw - b.yaw), math.cos(a.yaw - b.yaw))
    return distance, abs(math.degrees(dyaw))


def scenario_spawn(path: Path) -> Pose2D:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    pose = data.get("robot", {}).get("initial_pose")
    if not isinstance(pose, list) or len(pose) < 3:
        raise ResetError(f"{path}: robot.initial_pose [x, y, yaw] missing")
    return Pose2D(float(pose[0]), float(pose[1]), float(pose[2]))


def _log(message: str) -> None:
    print(f"gpsr-run-reset: {message}", file=sys.stderr, flush=True)


def _active(client: ResetClient, actions) -> dict[str, int]:
    counts = {action: client.active_goals(action) for action in actions}
    return {action: n for action, n in counts.items() if n}


def _wait_until(client: ResetClient, predicate: Callable[[], bool], timeout_s: float,
                poll_s: float, failure: Callable[[], str], *, hold_s: float = 0.0) -> None:
    deadline = client.now() + timeout_s
    held_since = None
    while True:
        if predicate():
            if held_since is None:
                held_since = client.now()
            if client.now() - held_since >= hold_s:
                return
        else:
            held_since = None
        if client.now() >= deadline:
            raise ResetError(failure())
        client.sleep(poll_s)


def _stationary(speed: tuple[float, float]) -> bool:
    linear, angular = speed
    return linear <= STATIONARY_LINEAR_M_S and angular <= STATIONARY_ANGULAR_RAD_S


def _fmt(pose: Pose2D) -> str:
    return f"({pose.x:.3f}, {pose.y:.3f}, {math.degrees(pose.yaw):.1f} deg)"


def run_reset(client: ResetClient, spawn: Pose2D, *, idle_timeout_s: float = 20.0,
              poll_s: float = 0.2, settle_s: float = 9.0,
              log: Callable[[str], None] = _log) -> ResetOutcome:
    actions = NAV_ACTIONS + ARM_ACTIONS
    # A TIMEOUT-killed orchestrator leaves its last goal driving the robot.
    for action in actions:
        if not client.cancel_all_goals(action):
            log(f"{action}: no action server, nothing to cancel")
    _wait_until(client, lambda: not _active(client, actions), idle_timeout_s, poll_s,
                lambda: f"goals still active after cancel: {_active(client, actions)}")
    # IdleReset needs a stationary robot and only waits 12 s for one.
    _wait_until(client, lambda: _stationary(client.base_speed()), idle_timeout_s, poll_s,
                lambda: "base still moving after goals were cancelled",
                hold_s=STATIONARY_HOLD_S)

    ok, detail = client.reset_state()
    if not ok:
        hint = ""
        if "reset stop remains asserted" in detail:
            hint = (" -- the sim is paused with the reset stop asserted;"
                    " restart the stack (scripts/gpsr-stack down, then up)")
        raise ResetError(f"STATE reset failed: {detail}{hint}")

    truth = client.truth_pose()
    error_m, error_deg = pose_error(truth, spawn)
    if error_m > SPAWN_TOLERANCE_M or error_deg > SPAWN_TOLERANCE_DEG:
        raise ResetError(
            f"base at {_fmt(truth)} after STATE reset, {error_m:.2f} m / {error_deg:.1f} deg"
            f" from spawn {_fmt(spawn)}: the root write did not reach physics"
        )

    # Odom integrates wheel velocities, which the reset zeroes, so only
    # map->odom moved. Seed AMCL with the truth pose (world == map).
    amcl = client.seed_localization(truth)
    if amcl is None:
        raise ResetError("AMCL did not publish a pose after /initialpose")
    amcl_m, amcl_deg = pose_error(amcl, truth)
    if amcl_m > AMCL_TOLERANCE_M or amcl_deg > AMCL_TOLERANCE_DEG:
        raise ResetError(
            f"AMCL settled at {_fmt(amcl)}, {amcl_m:.2f} m / {amcl_deg:.1f} deg from truth {_fmt(truth)}"
        )

    client.clear_costmaps()
    # STATE restores the stack-up arm pose, which blocks the head camera.
    client.home_arm_and_head()
    client.sleep(settle_s)

    leftover = _active(client, NAV_ACTIONS)
    if leftover:
        raise ResetError(f"a Nav2 goal started during reset: {leftover}")
    log(f"base at {_fmt(truth)} ({error_m:.3f} m / {error_deg:.1f} deg from scenario spawn),"
        f" AMCL {_fmt(amcl)}")
    return ResetOutcome(truth=truth, amcl=amcl, error_m=error_m, error_deg=error_deg)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `source /opt/ros/humble/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gpsr_run_reset.py -q`
Expected: `16 passed`

- [ ] **Step 5: Commit**

```bash
git add tools/gpsr_run_reset.py tests/test_gpsr_run_reset.py
git commit -m "feat(gpsr-bench): trial reset sequence — cancel goals, STATE reset, truth check, AMCL re-seed"
```

---

### Task 3: ROS adapter, CLI, batch summary, and the `gpsr-run-reset` wrapper

**Files:**
- Modify: `tools/gpsr_run_reset.py` (append adapter, records, CLI)
- Modify: `scripts/gpsr-run-reset` (replace body with a wrapper; rewrite header comment)
- Test: `tests/test_gpsr_run_reset.py` (append)

**Interfaces:**
- Consumes: everything Task 2 produces.
- Produces:
  - `RosResetClient()` implementing `ResetClient`, plus `close()`
  - `outcome_record(outcome: ResetOutcome, spawn: Pose2D) -> dict`
  - `start_pose_spread(records: list[dict]) -> tuple[float, float]`: worst (m, deg) of any record's `truth` against the first record's
  - `main(argv: Optional[list[str]] = None, client_factory: Optional[Callable[[], ResetClient]] = None) -> int`
  - CLI: `python3 tools/gpsr_run_reset.py --scenario PATH [--record PATH]` and `python3 tools/gpsr_run_reset.py --summarize PATH [--tolerance-m 0.05] [--tolerance-deg 2.0]`
  - Record file: env `GPSR_RESET_RECORD`, default `<repo>/gpsr_stack_logs/run-reset.jsonl`, one JSON object per line: `{"wall_time", "truth": [x, y, yaw], "amcl": [x, y, yaw], "spawn": [x, y, yaw], "error_m", "error_deg"}`

- [ ] **Step 1: Write the failing tests (append)**

```python
from tools.gpsr_run_reset import (  # noqa: E402
    ResetOutcome,
    main,
    outcome_record,
    start_pose_spread,
)

BENCH = str(ROOT / "simulation/scenarios/gpsr-rcw2026-bench.json")


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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `source /opt/ros/humble/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gpsr_run_reset.py -q`
Expected: collection error, `ImportError: cannot import name 'main'` (or `outcome_record` / `start_pose_spread`)

- [ ] **Step 3: Append the records, CLI and ROS adapter to `tools/gpsr_run_reset.py`**

Add `import argparse`, `import os` and `import time` to the import block. Then append:

```python
ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RECORD = ROOT / "gpsr_stack_logs" / "run-reset.jsonl"


def outcome_record(outcome: ResetOutcome, spawn: Pose2D) -> dict:
    return {
        "wall_time": time.time(),
        "truth": [outcome.truth.x, outcome.truth.y, outcome.truth.yaw],
        "amcl": [outcome.amcl.x, outcome.amcl.y, outcome.amcl.yaw],
        "spawn": [spawn.x, spawn.y, spawn.yaw],
        "error_m": outcome.error_m,
        "error_deg": outcome.error_deg,
    }


def start_pose_spread(records: list[dict]) -> tuple[float, float]:
    first = Pose2D(*records[0]["truth"])
    worst_m = worst_deg = 0.0
    for record in records[1:]:
        distance, degrees = pose_error(Pose2D(*record["truth"]), first)
        worst_m, worst_deg = max(worst_m, distance), max(worst_deg, degrees)
    return worst_m, worst_deg


def _summarize(path: Path, tolerance_m: float, tolerance_deg: float) -> int:
    records = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    if not records:
        print(f"{path}: no reset records")
        return 1
    distance, degrees = start_pose_spread(records)
    ok = distance <= tolerance_m and degrees <= tolerance_deg
    print(f"{len(records)} resets; worst start vs first: {distance:.3f} m / {degrees:.2f} deg"
          f" (tolerance {tolerance_m} m / {tolerance_deg} deg): {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


class RosResetClient:
    """The only place this module imports rclpy, constructed lazily so every
    test runs without ROS (same pattern as tools/gpsr_spawn.py)."""

    def __init__(self) -> None:
        import rclpy
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from tinker_sim_interfaces.msg import RobotTruth

        self._rclpy = rclpy
        rclpy.init(args=None)
        # Sim time so the /initialpose stamp matches the TF AMCL looks up.
        self.node = Node("gpsr_run_reset",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status: dict[str, object] = {}
        for action in NAV_ACTIONS + ARM_ACTIONS:
            self.node.create_subscription(
                GoalStatusArray, f"{action}/_action/status",
                lambda message, key=action: self._status.__setitem__(key, message), latched)
        self._truth = None
        self._truth_count = 0
        self.node.create_subscription(RobotTruth, "/sim/truth/robot_state", self._on_truth, 10)
        self._amcl = None
        self._amcl_count = 0
        self.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, latched)
        self._initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", latched)
        self._PoseWithCovarianceStamped = PoseWithCovarianceStamped

    def _on_truth(self, message) -> None:
        self._truth = message
        self._truth_count += 1

    def _on_amcl(self, message) -> None:
        self._amcl = message
        self._amcl_count += 1

    def _spin_until(self, predicate: Callable[[], bool], timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def _call(self, service_type, name: str, request, *, wait_s: float, call_s: float):
        client = self.node.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=wait_s):
                return None
            future = client.call_async(request)
            if not self._spin_until(future.done, call_s):
                raise ResetError(f"{name}: no response after {call_s:.0f}s")
            return future.result()
        finally:
            self.node.destroy_client(client)

    def _fresh_truth(self, timeout_s: float = 5.0):
        seen = self._truth_count
        if not self._spin_until(lambda: self._truth_count > seen, timeout_s):
            raise ResetError("/sim/truth/robot_state: no message (is truth_evaluator running?)")
        return self._truth

    def cancel_all_goals(self, action: str) -> bool:
        from action_msgs.srv import CancelGoal
        # An all-zero goal id and stamp cancels every goal on the server.
        return self._call(CancelGoal, f"{action}/_action/cancel_goal", CancelGoal.Request(),
                          wait_s=2.0, call_s=5.0) is not None

    def active_goals(self, action: str) -> int:
        self._rclpy.spin_once(self.node, timeout_sec=0.05)
        message = self._status.get(action)
        if message is None:
            return 0
        return sum(1 for item in message.status_list if item.status in ACTIVE_GOAL_STATUSES)

    def base_speed(self) -> tuple[float, float]:
        twist = self._fresh_truth().base_twist
        return math.hypot(twist.linear.x, twist.linear.y), abs(twist.angular.z)

    def truth_pose(self) -> Pose2D:
        pose = self._fresh_truth().base_pose
        q = pose.orientation
        return Pose2D(pose.position.x, pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def reset_state(self) -> tuple[bool, str]:
        from simulation_interfaces.msg import Result
        from simulation_interfaces.srv import ResetSimulation
        request = ResetSimulation.Request()
        request.scope = SCOPE_STATE
        response = self._call(ResetSimulation, "/reset_simulation", request,
                              wait_s=10.0, call_s=90.0)
        if response is None:
            return False, "/reset_simulation not available"
        return response.result.result == Result.RESULT_OK, response.result.error_message

    def seed_localization(self, pose: Pose2D) -> Optional[Pose2D]:
        if not self._spin_until(lambda: self.node.get_clock().now().nanoseconds > 0, 5.0):
            raise ResetError("/clock not received")
        message = self._PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        # Truth is known: 5 cm and ~2 deg standard deviation.
        message.pose.covariance[0] = 0.0025
        message.pose.covariance[7] = 0.0025
        message.pose.covariance[35] = 0.0012
        seed_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        seen = self._amcl_count

        def answered() -> bool:
            if self._amcl_count <= seen:
                return False
            stamp = self._amcl.header.stamp
            return stamp.sec * 10**9 + stamp.nanosec >= seed_ns

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            self._initial_pose_pub.publish(message)
            if self._spin_until(answered, 0.5):
                amcl = self._amcl.pose.pose
                q = amcl.orientation
                return Pose2D(amcl.position.x, amcl.position.y,
                              yaw_from_quaternion(q.x, q.y, q.z, q.w))
        return None

    def clear_costmaps(self) -> None:
        from nav2_msgs.srv import ClearEntireCostmap
        for name in ("/global_costmap/clear_entirely_global_costmap",
                     "/local_costmap/clear_entirely_local_costmap"):
            if self._call(ClearEntireCostmap, name, ClearEntireCostmap.Request(),
                          wait_s=2.0, call_s=5.0) is None:
                _log(f"{name}: not available, skipped")

    def home_arm_and_head(self) -> None:
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from tinker_vision_msgs_26.msg import PanTiltCommand
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        # Volatile to match the facade and ros2_control subscriptions during
        # graph churn; wait for both endpoints so a missing head facade
        # cannot leave the arm half-reset.
        qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        arm_pub = self.node.create_publisher(
            JointTrajectory, "/xarm7_traj_controller/joint_trajectory", qos)
        head_pub = self.node.create_publisher(PanTiltCommand, "/pan_tilt_controller/cmd", qos)
        endpoints = [("arm reset (/xarm7_traj_controller/joint_trajectory)", arm_pub),
                     ("head reset (/pan_tilt_controller/cmd)", head_pub)]
        if not self._spin_until(
                lambda: all(p.get_subscription_count() > 0 for _, p in endpoints), 30.0):
            missing = [label for label, p in endpoints if p.get_subscription_count() < 1]
            raise ResetError(f"no matching subscriber after 30s: {', '.join(missing)}")
        trajectory = JointTrajectory()
        trajectory.joint_names = [f"joint{i}" for i in range(1, 8)]
        point = JointTrajectoryPoint()
        point.positions = [0.0] * 7
        # 8 s: a 2 s swing trips the controller's 1.0 rad state tolerance.
        point.time_from_start.sec = 8
        trajectory.points = [point]
        head = PanTiltCommand()
        head.mode = 0
        head.pan_rad = 0.0
        head.tilt_rad = 0.0
        arm_pub.publish(trajectory)
        head_pub.publish(head)
        self._spin_until(lambda: False, 0.5)  # bounded chance for DDS delivery

    def sleep(self, seconds: float) -> None:
        self._spin_until(lambda: False, seconds)

    def now(self) -> float:
        return time.monotonic()

    def close(self) -> None:
        self.node.destroy_node()
        self._rclpy.try_shutdown()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario", type=Path,
                        default=ROOT / "simulation/scenarios/gpsr-rcw2026-bench.json")
    parser.add_argument("--record", type=Path,
                        default=Path(os.environ.get("GPSR_RESET_RECORD", DEFAULT_RECORD)))
    parser.add_argument("--summarize", type=Path, metavar="RECORD",
                        help="check a batch's reset records instead of resetting")
    parser.add_argument("--tolerance-m", type=float, default=0.05)
    parser.add_argument("--tolerance-deg", type=float, default=2.0)
    return parser


def main(argv: Optional[list[str]] = None,
         client_factory: Optional[Callable[[], ResetClient]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    if args.summarize is not None:
        return _summarize(args.summarize, args.tolerance_m, args.tolerance_deg)
    client = None
    try:
        spawn = scenario_spawn(args.scenario)
        client = (client_factory or RosResetClient)()
        outcome = run_reset(client, spawn)
    except ResetError as error:
        _log(str(error))
        return 1
    finally:
        close = getattr(client, "close", None)
        if close is not None:
            close()
    args.record.parent.mkdir(parents=True, exist_ok=True)
    with args.record.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(outcome_record(outcome, spawn)) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

Tests import it as `tools.gpsr_run_reset`, while the wrapper runs it by file path (`python3 tools/gpsr_run_reset.py`). Keep the module free of `tools.*` imports so both work.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `source /opt/ros/humble/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gpsr_run_reset.py -q`
Expected: `22 passed`

- [ ] **Step 5: Replace `scripts/gpsr-run-reset` with the wrapper**

```bash
#!/bin/bash
# Per-trial GPSR bench reset (tier2 --reset-cmd). Puts the base back at the
# spawn pose so every trial starts from the same place:
#
# 1. Cancel every goal on /navigate_to_pose, /go_to_approach and the arm and
#    gripper action servers. A TIMEOUT-killed orchestrator otherwise leaves
#    its last Nav2 goal driving the robot through reset, settle and spawn.
# 2. Wait for the base to stop.
# 3. /reset_simulation scope 2 (STATE): the sim's scoped reset writes the
#    root pose through the live PhysX articulation view
#    (simulation/tinker_sim_isaac/scoped_reset.py). Not SetEntityState: that
#    writes a USD layer Isaac Lab never reads back for the articulation
#    (commit 5fa9186). Not scope 0: that deletes every spawned entity.
# 4. Check the base against /sim/truth/robot_state, re-seed AMCL there
#    (wheel odom does not jump, so only map->odom moves), clear costmaps.
# 5. Zero the arm (the stack-up arm pose blocks the head camera) and re-home
#    pan/tilt through the facade, then settle.
#
# Each successful reset appends one line to $GPSR_RESET_RECORD (default
# gpsr_stack_logs/run-reset.jsonl). Check a batch with:
#   python3 tools/gpsr_run_reset.py --summarize <record>
# Expects the battery's ROS environment (tk25_ws + ros2_ws overlays sourced).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Under tier2's 180 s reset timeout.
exec timeout 170 python3 "$ROOT/tools/gpsr_run_reset.py" \
    --scenario "$ROOT/simulation/scenarios/${GPSR_SCENARIO:-gpsr-rcw2026-bench}.json" "$@"
```

Run: `bash -n scripts/gpsr-run-reset && test -x scripts/gpsr-run-reset && echo ok`
Expected: `ok`

- [ ] **Step 6: Commit**

```bash
git add tools/gpsr_run_reset.py tests/test_gpsr_run_reset.py scripts/gpsr-run-reset
git commit -m "feat(gpsr-bench): gpsr-run-reset restores the base via STATE reset and re-localizes AMCL"
```

---

### Task 4: Live acceptance on a multi-trial batch, plus docs

Dispatch steps 1–4 to the `sim-runner` agent. Probe shell environment as in Task 1 step 1.

**Files:**
- Modify: `docs/gpsr-sim-runbook.md` (reset section)
- Modify: `docs/native-scoped-reset.md` (append live GPSR evidence)
- Modify: `docs/issue_reports/2026-10-08-gpsr-nav-bug-report.md` (§3 status line)

- [ ] **Step 1: Single reset from a far, off-spawn pose**

```bash
./scripts/gpsr-stack up --scenario gpsr-rcw2026-bench --seed 0 --manipulation mock
export GPSR_RESET_RECORD=$PWD/gpsr_stack_logs/run-reset-probe.jsonl
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.49, y: -3.72}, orientation: {w: 1.0}}}}" &
sleep 15                      # goal still running: this is the orphaned-goal case
time ./scripts/gpsr-run-reset; echo "exit=$?"
ros2 action list -t; ros2 topic echo --once /navigate_to_pose/_action/status
```

Expected: `exit=0`, wall time under 90 s, a final `gpsr-run-reset: base at (-1.93…, -2.00…)` line, and no goal with status 1/2/3.

- [ ] **Step 2: Stale-sample check (Review Focus 5)**

Repeat step 1 three times. Each record's `truth` must equal the stack-up spawn, not the pre-reset pose. Then:

```bash
python3 tools/gpsr_run_reset.py --summarize "$GPSR_RESET_RECORD"
```

Expected: `... PASS`.

- [ ] **Step 3: Five-trial batch with a deliberate TIMEOUT**

Run the five-diverse battery that produced the 1a worst case (`t2-five-diverse-gpu1-20261007-rerun2`'s `battery*.sh`, copied to a new batch dir per its `battery.conf`). Export `GPSR_RESET_RECORD=<batch>/run-reset.jsonl` before launching, and set its per-trial timeout low enough (e.g. 120 s) that at least one trial ends in TIMEOUT mid-navigation.

Expected:
- `run-reset.jsonl` has one line per trial, and `--summarize` PASSes at 0.05 m / 2°.
- No `reset failed` ERROR in `run.json`.
- After a TIMEOUT trial, the next trial's `orchestrator.log` shows no robot motion before its first own goal.

- [ ] **Step 4: Cross-check with the pose traces**

```bash
cd docs/issue_reports/2026-10-08-gpsr-nav-tools
./extract_traces.sh -j 2 '<new batch name>'
python3 start_motion.py
```

Expected: every trial's start is at spawn, with no dwell-then-drift from an orphaned goal. The map-frame pose includes AMCL error, so judge position by the truth records from step 3, not by this.

- [ ] **Step 5: Update the docs**

`docs/gpsr-sim-runbook.md`: replace the description of the old joints-only reset with a short paragraph naming the five steps, `GPSR_RESET_RECORD`, `--summarize`, and the recovery for "reset stop remains asserted" (restart the stack).

`docs/native-scoped-reset.md`: append a "Live GPSR composite, 2026-10-xx" section with the Task 1 and Task 4 numbers (resets attempted/passed, worst spread, reset wall time, stack session dirs).

`docs/issue_reports/2026-10-08-gpsr-nav-bug-report.md` §3a: append a line `**Status:** fixed by tools/gpsr_run_reset.py (<commit>); verified on batch <name>`. Also note that the `tk25_decision` skip-command-point change is no longer needed for start-of-trial spinning (the trial starts 0.07 m from the command point, the < 0.1 m bucket), but mid-task returns still need §1a.

- [ ] **Step 6: Commit**

```bash
git add docs/gpsr-sim-runbook.md docs/native-scoped-reset.md docs/issue_reports/2026-10-08-gpsr-nav-bug-report.md
git commit -m "docs(gpsr-bench): trial reset restores the base; live evidence"
```
