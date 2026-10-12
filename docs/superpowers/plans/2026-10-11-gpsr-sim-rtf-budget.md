# GPSR Sim RTF Budget Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A full GPSR stack with the 3D raycast lidar holds RTF ≥ 0.5 (design target ≥ 0.55), the lidar publishes complete 10 Hz frames, and every run records its RTF.

**Architecture:** Fold the lidar after every PhysX substep through a backend hook (fixes 5 Hz frames with missing wedges). Select the lidar's ray budget per run with gpsr-stack presets (`battery`/`parity`/`planar`). Profile the lidar's Python cost and remove the dominant term only if it's measurably large. A `/clock` sampler started by gpsr-stack writes `rtf.jsonl` and warns below 0.5. Live A/B measurements set the battery channel count.

**Tech Stack:** Python 3.12 (sim venv, pytest via `scripts/pytest-clean`), Isaac Sim 6.0.1 / Isaac Lab, ROS 2 Humble (rclpy, system `python3`), Nav2.

**Spec:** `docs/superpowers/specs/2026-10-11-gpsr-sim-rtf-budget-design.md`

## Global Constraints

- Physics stays at the validated **120 Hz**; never set `TINKER_SIM_PHYSICS_HZ` in code or defaults.
- `simulation/sensors/hardware-parity.json` (the lidar contract) is **not edited**; scale changes go only through `TINKER_SIM_LIDAR_CHANNELS` / `TINKER_SIM_LIDAR_COLUMNS`.
- No edits to tk25_decision (battery runner, `GotoAction`): that repo is out of scope.
- RTF warn threshold is exactly **0.5** (`RTF_WARN_THRESHOLD = 0.5`); the battery channel count is chosen for **≥ 0.55**.
- Low RTF never kills a run.
- Default lidar preset is `battery`; columns stay at the contract's **349**.
- Run tests with `scripts/pytest-clean <paths> -q` (unsets the ROS overlay; the venv pytest breaks under a sourced ROS env).
- **Live runs (Tasks 6–9) follow the shared-host rules:** check GPU owners (`nvidia-smi`) and which ROS domains are in use (`for p in /proc/[0-9]*; do tr '\0' '\n' < $p/environ 2>/dev/null | grep -m1 '^ROS_DOMAIN_ID='; done | sort | uniq -c`), ask the owning session before using a busy GPU, pick an unused domain, wrap every `ros2` CLI/experiment step in `timeout -k 5 <s>`, set a hard wall-clock deadline for the whole session, and always `./scripts/gpsr-stack down` at the end.
- The shared `dev` checkout has an **uncommitted** gpsr-stack change (`--planar-lidar`, `raycast_lidar=True`, `GPSR_ROS_DOMAIN_ID`, `GPSR_VISION_GPU`, `GPSR_CONTROLLERS_FILE`). This branch re-implements the lidar, domain and vision-GPU parts with the same env-var semantics; before merging, the user must agree with that change's owner that `--lidar` supersedes `--planar-lidar`.

## Review Focus

1. **Control rate == physics rate (1 substep):** the lidar must fold exactly once per step: the hook and the publish-path fallback must never both fold. Pinned in Task 1 (`test_one_substep_folds_once_and_publish_path_stays_idle`).
2. **Sensor not yet valid (empty readings) while substeps run:** no exception, and the window still advances one slot per physics step. Pinned in Task 1 (`test_empty_readings_during_warmup_do_not_raise`).
3. **Caller exports `TINKER_SIM_LIDAR_CHANNELS` (e.g. for an A/B):** gpsr-stack must pass the caller's value through, not overwrite it with the preset. Pinned in Task 3 (`test_battery_preset_respects_caller_channel_export`).
4. **`/clock` jumps backwards (per-trial reset rewinds the epoch):** the monitor must not report negative or huge RTF across the jump. Pinned in Task 4 (`test_window_rtf_restarts_after_clock_rewind`, `test_summarise_skips_rewind_pairs`).
5. **No `/clock` yet, or a trial window with no records:** no division by zero; `null` RTF / `samples: 0`. Pinned in Task 4 (`test_window_rtf_needs_two_samples`, `test_summarise_empty_window`).

---

## File Structure

| file | change | responsibility |
|---|---|---|
| `simulation/tinker_sim_isaac/backend.py` | modify | `add_post_substep_hook()`; `_step_simulation` calls hooks after every substep |
| `simulation/tinker_sim_isaac/ros_gateway.py` | modify | register `rig.accumulate` as a substep hook; publish-path poll only as fallback; `lidar_profile_snapshot()` |
| `simulation/tinker_sim_isaac/lidar_rig.py` | modify | `substep` fold source + counter; opt-in profile laps; (Task 7, conditional) read only the needed reading fields |
| `validation/run_sim.py` | modify | `lidar_breakdown_ms` in the step-profile line |
| `scripts/gpsr-stack` | modify | `--lidar` presets; `GPSR_ROS_DOMAIN_ID`; `GPSR_VISION_GPU`; `GPSR_RUN_DIR` for stages; `rtf_monitor` stage |
| `tools/rtf_monitor.py` | create | `/clock` RTF sampler (rclpy, lazy import) + pure windowing/summarise functions + CLI |
| `tests/test_lidar_substep_fold.py` | create | substep hook + fold integration tests |
| `tests/test_lidar_profile.py` | create | rig profile laps + gateway snapshot |
| `tests/test_rtf_monitor.py` | create | pure RTF maths + warn state + summarise |
| `tests/test_gpsr_stack_dryrun.py` | modify | presets, domain, vision GPU, run dir, rtf stage |
| `docs/gpsr-sim-runbook.md`, `docs/developer-log.md` | modify | presets/RTF table; measurements and the 5 Hz bug |

---

### Task 1: Fold the lidar after every physics substep (spec Unit A)

**Files:**
- Modify: `simulation/tinker_sim_isaac/backend.py` (`_step_simulation`, ~line 1996)
- Modify: `simulation/tinker_sim_isaac/ros_gateway.py` (`__init__` after `self.lidar_rig = lidar_rig` ~line 159; `publish()` lidar block ~lines 1437–1447)
- Modify: `simulation/tinker_sim_isaac/lidar_rig.py` (`RaycastLidar.__init__`, `accumulate`, `diagnostics`)
- Test: `tests/test_lidar_substep_fold.py` (create)

**Interfaces:**
- Produces: `IsaacWholeRobotBackend.add_post_substep_hook(hook: Callable[[], None]) -> None`; `RosStandardGateway._attach_lidar_to_substeps() -> bool`; `RosStandardGateway._poll_lidar_if_unattached() -> None`; attribute `RosStandardGateway._lidar_folds_on_substep: bool`; `RaycastLidar.accumulate(*, source: str = "poll")` now accepts `source="substep"`; `diagnostics()` gains `"substep_folds": int`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_lidar_substep_fold.py`:

```python
"""The lidar must fold every PhysX substep, not once per control step.

At 120 Hz physics / 60 Hz control the gateway used to poll the rig once per
control step: every other sweep slice was cast but never read, a 12-slice
window took 24 physics steps (5 Hz frames), and alternating ~30 deg wedges
were never published (measured live 2026-10-11).
"""
from types import SimpleNamespace

import numpy as np

from tinker_sim_isaac.backend import IsaacWholeRobotBackend
from tinker_sim_isaac.lidar_rig import FrameAccumulator, LidarSpec, RaycastLidar, lidar_pattern
from tinker_sim_isaac.ros_gateway import RosStandardGateway

COLUMNS = 12  # one column fires per physics step: 12 steps = one 10 Hz frame at 120 Hz


def _spec() -> LidarSpec:
    return LidarSpec(
        pointcloud_topic="/livox/lidar", scan_topic="/scan", frame_id="livox360",
        tick_rate_hz=10.0, mount_prim="livox_frame", channels=1, columns=COLUMNS,
        elevation_min_deg=0.0, elevation_max_deg=0.0, min_range_m=0.2,
        max_range_m=40.0, sweep=True, self_filter=False,
    )


class _SweepSensor:
    """Each physics step fires exactly one ray (column = step % 12)."""

    def __init__(self, counter, *, valid_from_step: int = 0):
        self.counter = counter
        self.valid_from_step = valid_from_step

    def get_data(self):
        step = self.counter[0]
        if step < self.valid_from_step:
            return {"hit_positions": np.zeros((0, 3), dtype=np.float32),
                    "hit_prim_paths": [], "physics_step": step}
        hits = np.zeros((COLUMNS, 3), dtype=np.float32)
        hits[(step - 1) % COLUMNS] = (float((step - 1) % COLUMNS) + 1.0, 0.0, 0.0)
        return {"hit_positions": hits, "hit_prim_paths": [""] * COLUMNS, "physics_step": step}

    def get_sensor_reading(self):
        """Raw-struct view of the same reading (used once Task 7 lands)."""
        data = self.get_data()
        return SimpleNamespace(
            is_valid=data["hit_positions"].shape[0] > 0,
            hit_positions=data["hit_positions"],
            hit_prim_paths=data["hit_prim_paths"],
        )


def _rig(counter, **sensor_kw) -> RaycastLidar:
    spec = _spec()
    rig = RaycastLidar(spec)
    rig.sensor = _SweepSensor(counter, **sensor_kw)
    rig._step_counter = lambda: counter[0]  # read by Task 7's _read(); harmless before it
    _, _, offsets = lidar_pattern(spec)
    rig._accumulator = FrameAccumulator(spec.num_rays, COLUMNS, offsets)
    return rig


def _backend(counter, substeps: int) -> IsaacWholeRobotBackend:
    backend = object.__new__(IsaacWholeRobotBackend)
    backend.physics_substeps = substeps
    backend.render = False

    def _step(render):
        counter[0] += 1

    backend._sim = SimpleNamespace(step=_step)
    return backend


def _gateway(backend, rig) -> RosStandardGateway:
    gateway = object.__new__(RosStandardGateway)
    gateway.backend = backend
    gateway.lidar_rig = rig
    gateway._lidar_folds_on_substep = gateway._attach_lidar_to_substeps()
    return gateway


def _run_control_steps(backend, gateway, n: int) -> None:
    for _ in range(n):
        backend._step_simulation()
        gateway._poll_lidar_if_unattached()


def test_hooks_run_after_every_substep_in_order():
    counter = [0]
    backend = _backend(counter, substeps=3)
    seen = []
    backend.add_post_substep_hook(lambda: seen.append(counter[0]))
    backend._step_simulation()
    assert seen == [1, 2, 3]


def test_two_substeps_complete_a_full_frame_in_six_control_steps():
    counter = [0]
    backend, rig = _backend(counter, substeps=2), _rig(counter)
    gateway = _gateway(backend, rig)
    assert gateway._lidar_folds_on_substep is True
    _run_control_steps(backend, gateway, 6)  # 12 physics steps = one 10 Hz frame
    assert rig.frame_ready
    points, _ = rig.take_frame()
    assert sorted(points[:, 0].tolist()) == [float(i) for i in range(1, COLUMNS + 1)]
    assert rig.diagnostics()["substep_folds"] == 12
    assert rig.diagnostics()["poll_folds"] == 0


def test_one_substep_folds_once_and_publish_path_stays_idle():
    counter = [0]
    backend, rig = _backend(counter, substeps=1), _rig(counter)
    gateway = _gateway(backend, rig)
    _run_control_steps(backend, gateway, COLUMNS)
    diag = rig.diagnostics()
    assert diag["substep_folds"] == COLUMNS
    assert diag["poll_folds"] == 0
    assert diag["frames"] == 1


def test_backend_without_hook_falls_back_to_publish_path_poll():
    counter = [0]
    rig = _rig(counter)
    legacy_backend = SimpleNamespace()  # no add_post_substep_hook
    gateway = _gateway(legacy_backend, rig)
    assert gateway._lidar_folds_on_substep is False
    counter[0] = 1
    gateway._poll_lidar_if_unattached()
    assert rig.diagnostics()["poll_folds"] == 1


def test_empty_readings_during_warmup_do_not_raise():
    counter = [0]
    backend, rig = _backend(counter, substeps=2), _rig(counter, valid_from_step=5)
    gateway = _gateway(backend, rig)
    _run_control_steps(backend, gateway, 6)
    diag = rig.diagnostics()
    assert diag["substep_folds"] == 12
    assert diag["frames"] == 1  # window still advances one slot per physics step
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `scripts/pytest-clean tests/test_lidar_substep_fold.py -q`
Expected: FAIL with `AttributeError: 'IsaacWholeRobotBackend' object has no attribute 'add_post_substep_hook'` (and `_attach_lidar_to_substeps` missing).

- [ ] **Step 3: Implement the backend hook**

In `backend.py`, replace `_step_simulation` with:

```python
    def add_post_substep_hook(self, hook: Callable[[], None]) -> None:
        """Run *hook* after every PhysX substep of every control step.

        For consumers whose data changes per SOLVER step rather than per
        control step. The raycast lidar's sweep fires one slice per physics
        step and the plugin clears its reading each step, so reading it once
        per control step at 120/60 dropped every other slice (5 Hz frames
        with permanently empty wedges, measured 2026-10-11). Hooks must not
        raise: they run inside the stepping loop.
        """
        hooks = self.__dict__.get("_post_substep_hooks")
        if hooks is None:
            hooks = self._post_substep_hooks = []
        hooks.append(hook)

    def _step_simulation(self) -> None:
        """One control step: ``physics_substeps`` solver steps of ``physics_dt``.

        Each Isaac Lab ``step()`` is one ``IPhysxSimulation.simulate(dt)`` +
        ``fetch_results()`` at the validated solver dt; Kit is rendered (when
        enabled) only after the last substep. Post-substep hooks run after
        every solver step.
        """
        last = self.physics_substeps - 1
        hooks = self.__dict__.get("_post_substep_hooks", ())
        for index in range(self.physics_substeps):
            self._sim.step(render=self.render and index == last)
            for hook in hooks:
                hook()
```

(`Callable` is already imported at the top of `backend.py`.)

- [ ] **Step 4: Implement the rig's `substep` source**

In `lidar_rig.py` `RaycastLidar.__init__`, after `self._poll_folds = 0` add:

```python
        self._substep_folds = 0
```

In `accumulate`, replace

```python
        if source == "callback":
            self._callback_folds += 1
        else:
            self._poll_folds += 1
```

with

```python
        if source == "callback":
            self._callback_folds += 1
        elif source == "substep":
            self._substep_folds += 1
        else:
            self._poll_folds += 1
```

In `diagnostics()`, after the `"poll_folds": self._poll_folds,` entry add:

```python
            "substep_folds": self._substep_folds,
```

- [ ] **Step 5: Implement the gateway wiring**

In `ros_gateway.py` `__init__`, directly after `self.lidar_rig = lidar_rig`:

```python
        self._lidar_folds_on_substep = self._attach_lidar_to_substeps()
```

Add these methods to `RosStandardGateway` (next to `_lidar_diagnostics`):

```python
    def _attach_lidar_to_substeps(self) -> bool:
        """Fold the lidar after every physics substep, if the backend allows.

        The sweep fires one slice per PHYSICS step; polling once per control
        step drops all but the last substep's slice. Returns whether the rig
        is now driven by the backend, in which case the publish path must
        not poll it as well.
        """
        rig = getattr(self, "lidar_rig", None)
        register = getattr(self.backend, "add_post_substep_hook", None)
        if rig is None or register is None:
            return False
        register(lambda: rig.accumulate(source="substep"))
        return True

    def _poll_lidar_if_unattached(self) -> None:
        """Publish-path fallback for backends without substep hooks."""
        rig = getattr(self, "lidar_rig", None)
        if rig is not None and not getattr(self, "_lidar_folds_on_substep", False):
            rig.accumulate()
```

In `publish()`, replace the block

```python
        rig = getattr(self, "lidar_rig", None)
        if rig is not None:
            # Drive the frame accumulator from the publish path as well as
            ...
            rig.accumulate()
        if self._cloud_publish_enabled():
```

with

```python
        rig = getattr(self, "lidar_rig", None)
        # The rig is normally folded after every physics substep by the
        # backend (see _attach_lidar_to_substeps); this poll is only the
        # fallback for backends without substep hooks.
        self._poll_lidar_if_unattached()
        if self._cloud_publish_enabled():
```

(keep `rig` — it's used just below for `_live_point_cloud`).

- [ ] **Step 6: Run the new and neighbouring tests**

Run: `scripts/pytest-clean tests/test_lidar_substep_fold.py tests/test_lidar_rig.py tests/test_ros_gateway.py tests/test_control_rate_override.py -q`
Expected: all PASS. (If `test_control_rate_override.py` asserts the exact old `_step_simulation` source text, update only that string assertion to the new loop and say so in the commit message.)

- [ ] **Step 7: Commit**

```bash
git add simulation/tinker_sim_isaac/backend.py simulation/tinker_sim_isaac/ros_gateway.py \
        simulation/tinker_sim_isaac/lidar_rig.py tests/test_lidar_substep_fold.py
git commit -m "fix(lidar): fold every physics substep, not once per control step

At 120/60 the publish-path poll read one of two substeps: /livox/lidar ran
at 5 Hz sim time with alternating ~30 deg wedges never published."
```

---

### Task 2: Lidar profile laps (spec Unit C, measurement half)

**Files:**
- Modify: `simulation/tinker_sim_isaac/lidar_rig.py` (`RaycastLidar.__init__`, `accumulate`; new `profile_snapshot`)
- Modify: `simulation/tinker_sim_isaac/ros_gateway.py` (new `lidar_profile_snapshot`)
- Modify: `validation/run_sim.py` (`_emit_step_profile` ~line 76; its call site ~line 1596)
- Test: `tests/test_lidar_profile.py` (create)

**Interfaces:**
- Consumes: `RaycastLidar.accumulate(source=...)` from Task 1.
- Produces: `RaycastLidar.profile_enabled: bool`; `RaycastLidar.profile_snapshot() -> dict` with keys `get_data`, `self_mask`, `fold` (ms per call, float) and `calls` (int); `RosStandardGateway.lidar_profile_snapshot() -> dict | None`; `_emit_step_profile(..., lidar_breakdown: dict | None = None)` writing `step_profile.lidar_breakdown_ms`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_lidar_profile.py`:

```python
"""Opt-in wall-time attribution of the lidar's Python path (TINKER_SIM_PROFILE=1)."""
from types import SimpleNamespace

import numpy as np

from tinker_sim_isaac.lidar_rig import FrameAccumulator, LidarSpec, RaycastLidar, lidar_pattern
from tinker_sim_isaac.ros_gateway import RosStandardGateway


def _rig(profile: bool) -> RaycastLidar:
    spec = LidarSpec(
        pointcloud_topic="/livox/lidar", scan_topic="/scan", frame_id="livox360",
        tick_rate_hz=10.0, mount_prim="livox_frame", channels=1, columns=12,
        elevation_min_deg=0.0, elevation_max_deg=0.0, min_range_m=0.2,
        max_range_m=40.0, sweep=True, self_filter=True,
    )
    rig = RaycastLidar(spec)
    rig.profile_enabled = profile
    step = [0]

    def _hits():
        hits = np.zeros((12, 3), dtype=np.float32)
        hits[0] = (1.0, 0.0, 0.0)
        return hits

    def get_data():
        step[0] += 1
        return {"hit_positions": _hits(), "hit_prim_paths": ["/World/Arena/wall"] * 12,
                "physics_step": step[0]}

    def get_sensor_reading():
        step[0] += 1
        return SimpleNamespace(is_valid=True, hit_positions=_hits(),
                               hit_prim_paths=["/World/Arena/wall"] * 12)

    rig.sensor = SimpleNamespace(get_data=get_data, get_sensor_reading=get_sensor_reading)
    rig._step_counter = lambda: step[0]  # read by Task 7's _read(); harmless before it
    _, _, offsets = lidar_pattern(spec)
    rig._accumulator = FrameAccumulator(12, 12, offsets)
    return rig


def test_profile_counts_calls_and_resets():
    rig = _rig(profile=True)
    for _ in range(5):
        rig.accumulate(source="substep")
    snap = rig.profile_snapshot()
    assert snap["calls"] == 5
    assert set(snap) == {"get_data", "self_mask", "fold", "calls"}
    assert all(snap[k] >= 0.0 for k in ("get_data", "self_mask", "fold"))
    assert rig.profile_snapshot()["calls"] == 0  # window reset


def test_profile_off_records_nothing():
    rig = _rig(profile=False)
    rig.accumulate(source="substep")
    assert rig.profile_snapshot()["calls"] == 0


def test_profile_reads_env_flag(monkeypatch):
    monkeypatch.setenv("TINKER_SIM_PROFILE", "1")
    spec = _rig(profile=False).spec
    assert RaycastLidar(spec).profile_enabled is True
    monkeypatch.delenv("TINKER_SIM_PROFILE")
    assert RaycastLidar(spec).profile_enabled is False


def test_gateway_snapshot_passthrough():
    gateway = object.__new__(RosStandardGateway)
    gateway.lidar_rig = _rig(profile=True)
    gateway.lidar_rig.accumulate(source="substep")
    assert gateway.lidar_profile_snapshot()["calls"] == 1
    gateway.lidar_rig = None
    assert gateway.lidar_profile_snapshot() is None
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_lidar_profile.py -q`
Expected: FAIL with `AttributeError: ... 'profile_enabled'` / `'profile_snapshot'`.

- [ ] **Step 3: Implement the rig laps**

In `lidar_rig.py` add `import time` to the imports. In `RaycastLidar.__init__` (end of method):

```python
        #: Opt-in wall-time attribution (TINKER_SIM_PROFILE=1), read like the
        #: backend's step profile: where the lidar's Python cost per fold goes.
        self.profile_enabled = os.environ.get("TINKER_SIM_PROFILE") == "1"
        self._profile = {"get_data": 0.0, "self_mask": 0.0, "fold": 0.0, "calls": 0}
```

Replace the body of `accumulate` from `if self.sensor is None ...` through the end with:

```python
        if self.sensor is None or self._accumulator is None:
            return
        clock = time.perf_counter if self.profile_enabled else None
        t0 = clock() if clock else 0.0
        try:
            data = self.sensor.get_data()
            step = data.get("physics_step")
            if step is not None and step == self._last_physics_step:
                # This physics step was already folded by the other driver.
                return
            self._last_physics_step = step
            hits = data["hit_positions"]
            t1 = clock() if clock else 0.0
            if self.spec.self_filter:
                hits = self._drop_self_hits(hits, data.get("hit_prim_paths"))
            t2 = clock() if clock else 0.0
        except Exception as exc:  # noqa: BLE001
            self._report_once("read", exc)
            return
        if source == "callback":
            self._callback_folds += 1
        elif source == "substep":
            self._substep_folds += 1
        else:
            self._poll_folds += 1
        try:
            self._last_reading_len = int(np.asarray(hits).reshape(-1, 3).shape[0])
        except Exception:  # noqa: BLE001 - diagnostics must never raise
            self._last_reading_len = -1
        try:
            complete = self._accumulator.add_reading(hits)
        except ValueError as exc:
            self._report_once("accumulate", exc)
            return
        if complete:
            self._pending = self._accumulator.take_frame()
            self._last_points = int(self._pending[0].shape[0])
            self._frame_ready = True
        if clock:
            t3 = clock()
            prof = self._profile
            prof["get_data"] += t1 - t0
            prof["self_mask"] += t2 - t1
            prof["fold"] += t3 - t2
            prof["calls"] += 1
```

Add the method (after `diagnostics`):

```python
    def profile_snapshot(self) -> dict[str, float | int]:
        """Per-fold ms attribution since the last snapshot; resets the window."""
        prof = self._profile
        calls = prof["calls"]
        n = max(1, calls)
        out: dict[str, float | int] = {
            key: round(1000.0 * prof[key] / n, 3) for key in ("get_data", "self_mask", "fold")
        }
        out["calls"] = calls
        for key in ("get_data", "self_mask", "fold"):
            prof[key] = 0.0
        prof["calls"] = 0
        return out
```

- [ ] **Step 4: Gateway passthrough and profile line**

In `ros_gateway.py`, next to `camera_profile_snapshot`:

```python
    def lidar_profile_snapshot(self) -> dict | None:
        """Per-fold ms attribution of the raycast lidar, or None without one."""
        rig = getattr(self, "lidar_rig", None)
        reader = getattr(rig, "profile_snapshot", None)
        return None if reader is None else reader()
```

In `validation/run_sim.py` `_emit_step_profile`, add the parameter `lidar_breakdown: dict | None = None,` after `spin_breakdown`, and after the `spin_breakdown` payload block:

```python
    if lidar_breakdown is not None:
        payload["step_profile"]["lidar_breakdown_ms"] = lidar_breakdown
```

At the call site (~line 1596, the one passing `publish_breakdown=`), add:

```python
                                        lidar_breakdown=(
                                            gateway.lidar_profile_snapshot()
                                            if hasattr(gateway, "lidar_profile_snapshot")
                                            else None
                                        ),
```

- [ ] **Step 5: Run tests**

Run: `scripts/pytest-clean tests/test_lidar_profile.py tests/test_lidar_substep_fold.py tests/test_lidar_rig.py tests/test_step_profile_summary.py -q`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add simulation/tinker_sim_isaac/lidar_rig.py simulation/tinker_sim_isaac/ros_gateway.py \
        validation/run_sim.py tests/test_lidar_profile.py
git commit -m "feat(lidar): opt-in per-fold profile laps (get_data / self_mask / fold)"
```

---

### Task 3: gpsr-stack lidar presets (spec Unit B)

**Files:**
- Modify: `scripts/gpsr-stack` (constants near the top-level config; `StackConfig` ~line 220; sim stage in `stage_commands` ~lines 262–320; `_run_up` ~line 642; `_build_parser` ~line 832; `main`)
- Test: `tests/test_gpsr_stack_dryrun.py`

**Interfaces:**
- Produces: `LIDAR_PRESETS = ("battery", "parity", "planar")`; `BATTERY_LIDAR_CHANNELS: int = 24`; `StackConfig.lidar: str = "battery"`; CLI `--lidar {battery,parity,planar}`; `_run_up` prints `lidar preset: <name>` (plus `(channels=<N>)` for battery).

- [ ] **Step 1: Write the failing tests** (append to `tests/test_gpsr_stack_dryrun.py`)

```python
# --- Lidar presets (RTF budget, docs/superpowers/specs/2026-10-11-gpsr-sim-rtf-budget-design.md) ---

def _sim_stage(**kw):
    return [s for s in mod.stage_commands(_cfg(**kw)) if s["name"] == "sim"][0]


def test_default_preset_is_battery_raycast_with_reduced_channels(monkeypatch):
    monkeypatch.delenv("TINKER_SIM_LIDAR_CHANNELS", raising=False)
    sim = _sim_stage()
    assert sim["cmd"][-2:] == ["--", "--raycast-lidar"]
    assert sim["env"]["TINKER_SIM_LIDAR_CHANNELS"] == str(mod.BATTERY_LIDAR_CHANNELS)
    assert "TINKER_SIM_LIDAR_COLUMNS" not in sim["env"]


def test_parity_preset_uses_contract_scale(monkeypatch):
    monkeypatch.delenv("TINKER_SIM_LIDAR_CHANNELS", raising=False)
    sim = _sim_stage(lidar="parity")
    assert sim["cmd"][-2:] == ["--", "--raycast-lidar"]
    assert "TINKER_SIM_LIDAR_CHANNELS" not in sim["env"]


def test_planar_preset_has_no_raycast():
    sim = _sim_stage(lidar="planar")
    assert "--raycast-lidar" not in sim["cmd"]
    assert "TINKER_SIM_LIDAR_CHANNELS" not in sim["env"]


def test_battery_preset_respects_caller_channel_export(monkeypatch):
    monkeypatch.setenv("TINKER_SIM_LIDAR_CHANNELS", "32")
    assert _sim_stage()["env"]["TINKER_SIM_LIDAR_CHANNELS"] == "32"


def test_unknown_preset_is_refused():
    import pytest
    with pytest.raises(ValueError, match="lidar preset"):
        mod.stage_commands(_cfg(lidar="fast"))


def test_physics_rate_is_never_set_by_the_stack():
    for preset in mod.LIDAR_PRESETS:
        assert "TINKER_SIM_PHYSICS_HZ" not in _sim_stage(lidar=preset)["env"]


def test_cli_lidar_flag_reaches_config_and_banner(capsys):
    rc = mod.main(["up", "--manipulation", "mock", "--dry-run", "--lidar", "planar"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "lidar preset: planar" in out
    assert "--raycast-lidar" not in out
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_gpsr_stack_dryrun.py -q -k "preset or lidar or physics_rate"`
Expected: FAIL (`StackConfig` has no `lidar`; `BATTERY_LIDAR_CHANNELS` undefined).

- [ ] **Step 3: Implement**

Near the other module-level constants in `scripts/gpsr-stack`:

```python
#: Lidar presets for the sim stage (docs/gpsr-sim-runbook.md, "Lidar presets").
#: battery: live raycast lidar at BATTERY_LIDAR_CHANNELS x the contract's 349
#:   columns -- Nav2 consumes azimuth resolution, vertical channels are the
#:   RTF lever. parity: the contract's full pattern (~0.3 RTF on this host).
#:   planar: the occupancy-map lidar (no heights, no dynamic obstacles).
LIDAR_PRESETS = ("battery", "parity", "planar")
#: Set from the live A/B matrix (docs/developer-log.md, 2026-10 RTF budget):
#: the largest channel count holding RTF >= 0.55 with the full stack attached.
BATTERY_LIDAR_CHANNELS = 24
```

In `StackConfig`, after `evidence: bool = False`:

```python
    lidar: str = "battery"  # one of LIDAR_PRESETS
```

In `stage_commands`, after the `if cfg.evidence:` block and before `stages.append({"name": "sim", ...`:

```python
    if cfg.lidar not in LIDAR_PRESETS:
        raise ValueError(f"unknown lidar preset {cfg.lidar!r}; expected one of {LIDAR_PRESETS}")
    sim_lidar_args: list[str] = []
    if cfg.lidar != "planar":
        # Passthrough to the sim app (tinker-sim launch strips the "--").
        sim_lidar_args = ["--", "--raycast-lidar"]
    if cfg.lidar == "battery":
        # A caller's explicit export wins, so A/Bs need no code change.
        sim_env["TINKER_SIM_LIDAR_CHANNELS"] = (
            os.environ.get("TINKER_SIM_LIDAR_CHANNELS", "").strip()
            or str(BATTERY_LIDAR_CHANNELS)
        )
```

and append `*sim_lidar_args,` as the last element of the sim stage's `"cmd"` list (after `"--headless",`).

In `_run_up`, as the first line after `stages = stage_commands(cfg)`:

```python
    banner = f"lidar preset: {cfg.lidar}"
    if cfg.lidar == "battery":
        banner += f" (channels={stages[0]['env']['TINKER_SIM_LIDAR_CHANNELS']})"
    print(banner)
```

In `_build_parser`, after `--evidence`:

```python
    parser.add_argument(
        "--lidar", choices=LIDAR_PRESETS, default="battery",
        help="lidar preset: battery (reduced raycast, default), parity (full "
             "contract raycast, ~0.3 RTF), planar (occupancy-map lidar)")
```

In `main`, pass `lidar=args.lidar,` into `StackConfig(...)`.

- [ ] **Step 4: Run the full dry-run suite**

Run: `scripts/pytest-clean tests/test_gpsr_stack_dryrun.py -q`
Expected: all PASS (`test_sim_stage_env` checks `"--scenario" in cmd`, unaffected).

- [ ] **Step 5: Commit**

```bash
git add scripts/gpsr-stack tests/test_gpsr_stack_dryrun.py
git commit -m "feat(gpsr-stack): --lidar presets (battery default: raycast at 24 channels)"
```

---

### Task 4: RTF monitor tool (spec Unit D, tool half)

**Files:**
- Create: `tools/rtf_monitor.py`
- Test: `tests/test_rtf_monitor.py`

**Interfaces:**
- Produces (pure, importable without rclpy): `RTF_WARN_THRESHOLD = 0.5`; `window_rtf(samples: Sequence[tuple[float, float]], now_wall: float, window_s: float) -> float | None`; `class WarnState(threshold: float = RTF_WARN_THRESHOLD, remind_s: float = 60.0)` with `update(rtf_30s: float | None, now_wall: float) -> str | None`; `summarise(records: Iterable[dict], t_from: float, t_to: float) -> dict` returning `{"mean": float | None, "min_30s": float | None, "samples": int}`; `default_out_path(env: Mapping[str, str]) -> Path`; CLI `python3 tools/rtf_monitor.py [--out PATH]` and `python3 tools/rtf_monitor.py --summarise PATH --from WALL --to WALL`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_rtf_monitor.py`:

```python
"""Pure maths of tools/rtf_monitor.py (no rclpy needed)."""
import importlib.util
import json
from pathlib import Path

_PATH = Path(__file__).resolve().parents[1] / "tools" / "rtf_monitor.py"
_spec = importlib.util.spec_from_file_location("rtf_monitor", _PATH)
rtf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rtf)


def _series(rate, wall0=1000.0, sim0=0.0, n=61, dt=0.5):
    return [(wall0 + i * dt, sim0 + i * dt * rate) for i in range(n)]


def test_window_rtf_steady_rate():
    s = _series(0.4)
    assert abs(rtf.window_rtf(s, now_wall=s[-1][0], window_s=10.0) - 0.4) < 1e-9


def test_window_rtf_only_uses_the_window():
    s = _series(1.0, n=40) + [(1000.0 + 20.0 + i * 0.5, 20.0 + i * 0.5 * 0.2) for i in range(1, 41)]
    assert abs(rtf.window_rtf(s, now_wall=s[-1][0], window_s=10.0) - 0.2) < 1e-9


def test_window_rtf_needs_two_samples():
    assert rtf.window_rtf([], now_wall=5.0, window_s=10.0) is None
    assert rtf.window_rtf([(1.0, 1.0)], now_wall=5.0, window_s=10.0) is None


def test_window_rtf_restarts_after_clock_rewind():
    before = [(1000.0 + i, 500.0 + i * 0.5) for i in range(6)]       # sim 500..502.5
    after = [(1006.0 + i, 0.0 + i * 0.3) for i in range(6)]          # reset rewinds to 0
    got = rtf.window_rtf(before + after, now_wall=1011.0, window_s=30.0)
    assert abs(got - 0.3) < 1e-9


def test_warn_state_crossing_reminder_and_recovery():
    w = rtf.WarnState(threshold=0.5, remind_s=60.0)
    assert w.update(0.6, 0.0) is None
    assert "WARN rtf_30s=0.40 < 0.50" in w.update(0.4, 10.0)
    assert w.update(0.4, 20.0) is None                 # no spam
    assert "WARN" in w.update(0.45, 71.0)              # 60 s reminder
    assert w.update(0.55, 80.0).startswith("OK")       # recovery is logged once
    assert w.update(0.55, 90.0) is None
    assert w.update(None, 95.0) is None                # no data yet: silent


def test_summarise_time_weighted_mean_and_min():
    recs = [
        {"wall": 0.0, "sim": 0.0, "rtf_10s": None, "rtf_30s": None},
        {"wall": 10.0, "sim": 6.0, "rtf_10s": 0.6, "rtf_30s": 0.6},
        {"wall": 20.0, "sim": 8.0, "rtf_10s": 0.2, "rtf_30s": 0.4},
        {"wall": 30.0, "sim": 14.0, "rtf_10s": 0.6, "rtf_30s": 0.47},
    ]
    out = rtf.summarise(recs, t_from=0.0, t_to=30.0)
    assert abs(out["mean"] - 14.0 / 30.0) < 1e-9
    assert out["min_30s"] == 0.4
    assert out["samples"] == 4


def test_summarise_skips_rewind_pairs():
    recs = [
        {"wall": 0.0, "sim": 100.0, "rtf_10s": None, "rtf_30s": None},
        {"wall": 10.0, "sim": 105.0, "rtf_10s": 0.5, "rtf_30s": 0.5},
        {"wall": 20.0, "sim": 1.0, "rtf_10s": None, "rtf_30s": None},   # reset
        {"wall": 30.0, "sim": 6.0, "rtf_10s": 0.5, "rtf_30s": 0.5},
    ]
    out = rtf.summarise(recs, t_from=0.0, t_to=30.0)
    assert abs(out["mean"] - 0.5) < 1e-9


def test_summarise_empty_window():
    out = rtf.summarise([{"wall": 0.0, "sim": 0.0, "rtf_10s": None, "rtf_30s": None}], 50.0, 60.0)
    assert out == {"mean": None, "min_30s": None, "samples": 0}


def test_default_out_path_uses_run_dir(tmp_path):
    assert rtf.default_out_path({"GPSR_RUN_DIR": str(tmp_path)}) == tmp_path / "rtf.jsonl"
    assert rtf.default_out_path({}) == Path("rtf.jsonl")


def test_summarise_cli(tmp_path, capsys):
    path = tmp_path / "rtf.jsonl"
    path.write_text("\n".join(json.dumps(r) for r in [
        {"wall": 0.0, "sim": 0.0, "rtf_10s": None, "rtf_30s": None},
        {"wall": 10.0, "sim": 5.0, "rtf_10s": 0.5, "rtf_30s": 0.5},
    ]) + "\n")
    assert rtf.main(["--summarise", str(path), "--from", "0", "--to", "10"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out == {"mean": 0.5, "min_30s": 0.5, "samples": 2}
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_rtf_monitor.py -q`
Expected: FAIL (file `tools/rtf_monitor.py` does not exist).

- [ ] **Step 3: Implement `tools/rtf_monitor.py`**

```python
#!/usr/bin/env python3
"""Sample /clock and record the simulator's real-time factor (RTF).

RTF = simulated seconds per wall second. Below ~0.5 the decision layer's
wall-clock budgets start cancelling healthy Nav2 goals, so every GPSR run
records it (docs/superpowers/specs/2026-10-11-gpsr-sim-rtf-budget-design.md).

    python3 tools/rtf_monitor.py [--out PATH]          # run (needs ROS sourced)
    python3 tools/rtf_monitor.py --summarise PATH --from WALL --to WALL

Run mode appends one JSON line every 10 s wall to ``$GPSR_RUN_DIR/rtf.jsonl``
(``{"wall", "sim", "rtf_10s", "rtf_30s"}``) and prints a WARN line when the
30 s RTF drops below 0.5. It never stops the run. Summarise mode prints
``{"mean", "min_30s", "samples"}`` for one trial's wall window -- the call a
battery runner makes to stamp per-trial RTF.

The maths is pure and importable without rclpy (tested in the sim venv);
rclpy is imported only in run mode.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Iterable, Mapping, Sequence

RTF_WARN_THRESHOLD = 0.5
RECORD_PERIOD_S = 10.0
KEEP_S = 35.0


def _after_last_rewind(samples: Sequence[tuple[float, float]]) -> Sequence[tuple[float, float]]:
    """The tail of *samples* after the last backward sim-time jump (a reset)."""
    start = 0
    for i in range(1, len(samples)):
        if samples[i][1] < samples[i - 1][1]:
            start = i
    return samples[start:]


def window_rtf(
    samples: Sequence[tuple[float, float]], now_wall: float, window_s: float
) -> float | None:
    """RTF over the last *window_s* wall seconds of ``(wall, sim)`` samples."""
    recent = [s for s in samples if now_wall - window_s <= s[0] <= now_wall]
    recent = list(_after_last_rewind(recent))
    if len(recent) < 2:
        return None
    wall = recent[-1][0] - recent[0][0]
    if wall <= 0.0:
        return None
    return (recent[-1][1] - recent[0][1]) / wall


class WarnState:
    """Turns a stream of 30 s RTF values into rare, readable log lines."""

    def __init__(self, threshold: float = RTF_WARN_THRESHOLD, remind_s: float = 60.0) -> None:
        self.threshold = threshold
        self.remind_s = remind_s
        self._below_since_warn: float | None = None

    def update(self, rtf_30s: float | None, now_wall: float) -> str | None:
        if rtf_30s is None:
            return None
        if rtf_30s < self.threshold:
            if self._below_since_warn is None or now_wall - self._below_since_warn >= self.remind_s:
                self._below_since_warn = now_wall
                return f"WARN rtf_30s={rtf_30s:.2f} < {self.threshold:.2f}"
            return None
        if self._below_since_warn is not None:
            self._below_since_warn = None
            return f"OK rtf_30s={rtf_30s:.2f} >= {self.threshold:.2f}"
        return None


def summarise(records: Iterable[Mapping], t_from: float, t_to: float) -> dict:
    """Time-weighted mean RTF and worst 30 s RTF of records in [t_from, t_to]."""
    rows = [r for r in records if t_from <= float(r["wall"]) <= t_to]
    sim_total = wall_total = 0.0
    for a, b in zip(rows, rows[1:]):
        d_sim = float(b["sim"]) - float(a["sim"])
        d_wall = float(b["wall"]) - float(a["wall"])
        if d_sim < 0.0 or d_wall <= 0.0:
            continue  # a reset rewound the clock between these records
        sim_total += d_sim
        wall_total += d_wall
    lows = [float(r["rtf_30s"]) for r in rows if r.get("rtf_30s") is not None]
    return {
        "mean": (sim_total / wall_total) if wall_total > 0.0 else None,
        "min_30s": min(lows) if lows else None,
        "samples": len(rows),
    }


def default_out_path(env: Mapping[str, str]) -> Path:
    run_dir = env.get("GPSR_RUN_DIR", "").strip()
    return Path(run_dir) / "rtf.jsonl" if run_dir else Path("rtf.jsonl")


def _run(out: Path) -> int:  # pragma: no cover - needs a live ROS graph
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = Node("rtf_monitor")
    samples: deque[tuple[float, float]] = deque()
    warn = WarnState()

    def on_clock(msg) -> None:
        now = time.time()
        samples.append((now, msg.clock.sec + msg.clock.nanosec * 1e-9))
        while samples and now - samples[0][0] > KEEP_S:
            samples.popleft()

    def on_timer() -> None:
        now = time.time()
        if not samples:
            return
        r10 = window_rtf(samples, now, 10.0)
        r30 = window_rtf(samples, now, 30.0)
        record = {"wall": round(now, 3), "sim": round(samples[-1][1], 3),
                  "rtf_10s": None if r10 is None else round(r10, 4),
                  "rtf_30s": None if r30 is None else round(r30, 4)}
        with open(out, "a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")
        line = warn.update(r30, now)
        if line:
            print(f"[rtf_monitor] {line}", flush=True)

    node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
    node.create_timer(RECORD_PERIOD_S, on_timer)
    print(f"[rtf_monitor] writing {out} every {RECORD_PERIOD_S:.0f} s", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--summarise", type=Path, default=None)
    parser.add_argument("--from", dest="t_from", type=float, default=None)
    parser.add_argument("--to", dest="t_to", type=float, default=None)
    args = parser.parse_args(argv)
    if args.summarise is not None:
        if args.t_from is None or args.t_to is None:
            parser.error("--summarise needs --from and --to (unix wall seconds)")
        lines = args.summarise.read_text(encoding="utf-8").splitlines()
        records = [json.loads(line) for line in lines if line.strip()]
        print(json.dumps(summarise(records, args.t_from, args.t_to)))
        return 0
    return _run(args.out or default_out_path(os.environ))


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Run tests**

Run: `scripts/pytest-clean tests/test_rtf_monitor.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/rtf_monitor.py tests/test_rtf_monitor.py
git commit -m "feat(tools): rtf_monitor -- /clock RTF log, <0.5 warning, per-trial summarise"
```

---

### Task 5: gpsr-stack plumbing — RTF monitor stage, run dir, domain, vision GPU (spec Units D + E prerequisites)

**Files:**
- Modify: `scripts/gpsr-stack` (`_preamble_env` ~line 233; vision stage env ~line 445; end of `stage_commands`; `_run_up` Popen env ~line 665)
- Test: `tests/test_gpsr_stack_dryrun.py`

**Interfaces:**
- Consumes: `tools/rtf_monitor.py` CLI from Task 4 (`default_out_path` reads `GPSR_RUN_DIR`).
- Produces: stage `{"name": "rtf_monitor", "gate": "nav"}` appended last; every launched stage's env has `GPSR_RUN_DIR=<absolute run dir>`; `ROS_DOMAIN_ID` = `$GPSR_ROS_DOMAIN_ID` or `"42"`; vision stage `CUDA_VISIBLE_DEVICES=$GPSR_VISION_GPU` when set (same semantics as the shared checkout's uncommitted change).

- [ ] **Step 1: Update the existing name-list tests and add new ones**

In `tests/test_gpsr_stack_dryrun.py` change:
- `test_mock_skips_stage4`: expected list → `["sim", "bridge", "vision", "nav_extras", "rtf_monitor"]`
- `test_stage_order_live`: expected list → `["sim", "bridge", "vision", "manipulation", "nav_extras", "rtf_monitor"]`
- `test_gates_present_and_known`: add `assert by_name["rtf_monitor"] == "nav"`

Append:

```python
# --- RTF monitor stage, run dir, private domain, vision GPU ---

def test_rtf_monitor_stage_runs_the_tool_under_ros():
    stage = mod.stage_commands(_cfg())[-1]
    assert stage["name"] == "rtf_monitor"
    script = stage["cmd"][-1]
    assert stage["cmd"][:2] == ["bash", "-lc"]
    assert "source /opt/ros/humble/setup.bash" in script
    assert "exec python3 tools/rtf_monitor.py" in script


def test_domain_override(monkeypatch):
    monkeypatch.setenv("GPSR_ROS_DOMAIN_ID", "219")
    for stage in mod.stage_commands(_cfg()):
        assert stage["env"]["ROS_DOMAIN_ID"] == "219"
    monkeypatch.delenv("GPSR_ROS_DOMAIN_ID")
    assert mod.stage_commands(_cfg())[0]["env"]["ROS_DOMAIN_ID"] == "42"


def test_vision_gpu_override(monkeypatch):
    monkeypatch.delenv("GPSR_VISION_GPU", raising=False)
    vision = [s for s in mod.stage_commands(_cfg()) if s["name"] == "vision"][0]
    assert "CUDA_VISIBLE_DEVICES" not in vision["env"]
    monkeypatch.setenv("GPSR_VISION_GPU", "1")
    vision = [s for s in mod.stage_commands(_cfg()) if s["name"] == "vision"][0]
    assert vision["env"]["CUDA_VISIBLE_DEVICES"] == "1"
    assert vision["env"]["QT_QPA_PLATFORM"] == "offscreen"


def test_run_up_passes_run_dir_to_every_stage(tmp_path, monkeypatch):
    import os
    envs = []

    def _popen(argv, **kw):
        envs.append(kw["env"])
        return _FakePopenProc(os.getpid())

    monkeypatch.setattr(mod.subprocess, "Popen", _popen)
    monkeypatch.setattr(mod, "_wait_for_gate", lambda gate: None)
    assert mod._run_up(_cfg(log_dir=str(tmp_path)), dry_run=False) == 0
    run_dirs = {e["GPSR_RUN_DIR"] for e in envs}
    assert len(run_dirs) == 1
    run_dir = Path(run_dirs.pop())
    assert run_dir.is_absolute() and run_dir.parent == tmp_path.resolve()
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_gpsr_stack_dryrun.py -q`
Expected: FAIL on the stage lists, `rtf_monitor`, domain and vision-GPU tests, and `KeyError: 'GPSR_RUN_DIR'`.

- [ ] **Step 3: Implement**

`_preamble_env`: replace `env = {"ROS_DOMAIN_ID": "42"}` with

```python
    # Keep the runbook default, but allow an isolated run to select a
    # private DDS domain when another simulator stack is already running.
    env = {"ROS_DOMAIN_ID": os.environ.get("GPSR_ROS_DOMAIN_ID", "42")}
```

Vision stage: replace its `"env": _preamble_env({... "QT_QPA_PLATFORM": "offscreen", })` with a `vision_env` built before `stages.append({"name": "vision", ...})`:

```python
    vision_env = _preamble_env({
        # waving_person_server defaults to show_window:=true and needs a
        # display; this keeps a headless bring-up from aborting on it.
        "QT_QPA_PLATFORM": "offscreen",
    })
    # Vision inference shares the sim's GPU by default. An isolated run may
    # place it on another device (RTF A/B, docs/developer-log.md 2026-10).
    vision_gpu = os.environ.get("GPSR_VISION_GPU", "").strip()
    if vision_gpu:
        vision_env["CUDA_VISIBLE_DEVICES"] = vision_gpu
```

and use `"env": vision_env,` in that stage.

End of `stage_commands`, before `return stages`:

```python
    # --- Stage 6: RTF monitor ---------------------------------------------
    # Samples /clock for the whole run: $GPSR_RUN_DIR/rtf.jsonl every 10 s and
    # a WARN line in its log below RTF 0.5. Never stops anything.
    stages.append({
        "name": "rtf_monitor",
        "cmd": _ros_argv(
            sources=("/opt/ros/humble/setup.bash",),
            argv=["python3", "tools/rtf_monitor.py"],
        ),
        "env": _preamble_env({}),
        "cwd": REPO_ROOT,
        "gate": "nav",
    })
```

`_run_up`: change `env = _full_env(stage["env"])` to

```python
            env = _full_env(stage["env"])
            env["GPSR_RUN_DIR"] = str(run_dir.resolve())
```

- [ ] **Step 4: Run tests**

Run: `scripts/pytest-clean tests/test_gpsr_stack_dryrun.py tests/test_rtf_monitor.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add scripts/gpsr-stack tests/test_gpsr_stack_dryrun.py
git commit -m "feat(gpsr-stack): rtf_monitor stage, GPSR_RUN_DIR, private domain, vision GPU"
```

---

### Task 6: Live check — Unit A acceptance and lidar profile (needs a GPU)

**Files:** none changed (evidence only). Record numbers in `$CLAUDE_JOB_DIR/tmp/rtf-notes.md` (or another scratch file) for Task 9.

**Interfaces:**
- Consumes: Tasks 1–5. Produces: measured `lidar_breakdown_ms` and the substep fold check that gate Task 7.

- [ ] **Step 1: Pre-flight (shared host)**

```bash
nvidia-smi --query-gpu=index,memory.used,utilization.gpu --format=csv
for p in /proc/[0-9]*; do tr '\0' '\n' < $p/environ 2>/dev/null | grep -m1 '^ROS_DOMAIN_ID='; done | sort | uniq -c
```
Pick a free GPU `G` and an unused domain `D`. If the GPU is in use, ask its owning session first. Set a hard deadline (e.g. 45 min) and tear down by then no matter what.

- [ ] **Step 2: Bring up the stack at the battery preset with profiling**

```bash
export GPSR_ROS_DOMAIN_ID=D TINKER_SIM_PROFILE=1
timeout -k 10 900 ./scripts/gpsr-stack up --manipulation mock --sim-gpu G --lidar battery
```
Expected: `lidar preset: battery (channels=24)`, every stage `ready`, `stack up. logs: gpsr_stack_logs/<ts>`.

- [ ] **Step 3: Verify 10 Hz frames and full azimuth coverage**

With ROS sourced and `ROS_DOMAIN_ID=D`, run a 25 s probe of `/livox/lidar` (robot stationary): the frame period in header sim time must be **0.100 ± 0.005 s**; in a 24 × 15° azimuth histogram, every bin that's non-zero in any of the last 10 frames must be non-zero in all of them (bins blocked by the robot's own body stay 0 in every frame). Also check the status heartbeat's lidar diagnostics in `01-sim.log`: `substep_folds` rises 2 per control step and `poll_folds` stays 0.

If the period is still 0.200 s: `physics_step` from `SimulationManager.get_num_physics_steps()` isn't advancing per substep, so the de-dup is dropping folds. **Stop and report** with the diagnostics line; don't improvise a fix.

- [ ] **Step 4: Read the lidar profile while the base drives**

Drive between two known standing poses from `constants.rcw2026.json` (`command_point` (−2.0, −2.0, yaw 0) and `side_table` (−0.62, −4.18, qz −0.5878 qw 0.8090)):

```bash
for i in 1 2; do
  timeout -k 5 150 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: -0.62, y: -4.18}, orientation: {z: -0.5878, w: 0.8090}}}}"
  timeout -k 5 150 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: -2.0, y: -2.0}, orientation: {z: 0.0, w: 1.0}}}}"
done
grep '"lidar_breakdown_ms"' gpsr_stack_logs/<ts>/01-sim.log | tail -5
```

Convert each term to s/sim-s: `ms_per_fold × 120 / 1000` (one fold per physics step at 120 Hz). Record `get_data`, `self_mask`, `fold`.

- [ ] **Step 5: Tear down**

```bash
timeout -k 10 300 ./scripts/gpsr-stack down
```

---

### Task 7 (conditional): Read only the reading fields the lidar uses (spec Unit C, fix half)

**Run only if** Task 6 measured `get_data` ≥ **0.05 s/sim-s**. If `self_mask` or `fold` is the dominant term ≥ 0.05 instead, **stop and report the numbers to the user**; that needs a design decision, not this task. If no term is ≥ 0.05, skip this task and record "no lidar Python fix needed" for Task 9.

Rationale: Isaac's `RaycastSensor.get_data()` converts **every** field of the C++ reading to Python on each call (`depths`, `hit_normals`, 19,893 `hit_prim_paths` strings, `hit_positions`), and the rig uses only positions and paths.

**Files:**
- Modify: `simulation/tinker_sim_isaac/lidar_rig.py` (`accumulate`, new `_read`)
- Test: `tests/test_lidar_profile.py`

**Interfaces:**
- Produces: `RaycastLidar._read() -> tuple[int | None, Any, Any]` returning `(physics_step, hit_positions, hit_prim_paths_or_None)`; attribute `RaycastLidar._step_counter: Callable[[], int]` (default `_physics_step_count`).

- [ ] **Step 1: Write the failing test** (append to `tests/test_lidar_profile.py`)

```python
def test_reads_only_positions_and_paths_from_the_raw_reading():
    rig = _rig(profile=False)
    touched = []

    class _Reading:
        is_valid = True

        def __getattr__(self, name):
            touched.append(name)
            if name == "hit_positions":
                hits = np.zeros((12, 3), dtype=np.float32)
                hits[0] = (1.0, 0.0, 0.0)
                return hits
            if name == "hit_prim_paths":
                return ["/World/Arena/wall"] * 12
            raise AssertionError(f"unexpected field read: {name}")

    rig.sensor = SimpleNamespace(get_sensor_reading=lambda: _Reading())
    rig._step_counter = lambda: 7
    rig.accumulate(source="substep")
    assert sorted(set(touched)) == ["hit_positions", "hit_prim_paths"]
    assert rig.diagnostics()["substep_folds"] == 1
```

- [ ] **Step 2: Run to verify failure**

Run: `scripts/pytest-clean tests/test_lidar_profile.py -q -k raw_reading`
Expected: FAIL with `AttributeError` (the fake has no `get_data`).

- [ ] **Step 3: Implement**

In `lidar_rig.py`, module level:

```python
def _physics_step_count() -> int:
    """The step counter `RaycastSensor.get_data()` stamps its frames with."""
    from isaacsim.core.simulation_manager import SimulationManager

    return int(SimulationManager.get_num_physics_steps())
```

In `RaycastLidar.__init__` (end of method):

```python
        #: Physics-step counter the de-dup keys on; injectable for tests.
        self._step_counter = _physics_step_count
```

Add to `RaycastLidar`:

```python
    def _read(self) -> tuple[int | None, Any, Any]:
        """``(physics_step, hit_positions, hit_prim_paths)`` from the raw reading.

        `RaycastSensor.get_data()` converts every field of the C++ reading to
        Python on each call -- depths, normals and one path string per ray --
        while the rig uses only positions and (for the self-filter) paths.
        Reading the struct directly skips the rest.
        """
        reading = self.sensor.get_sensor_reading()
        if not reading.is_valid:
            return self._step_counter(), np.zeros((0, 3), dtype=np.float32), None
        paths = reading.hit_prim_paths if self.spec.self_filter else None
        return self._step_counter(), reading.hit_positions, paths
```

In `accumulate`, replace

```python
            data = self.sensor.get_data()
            step = data.get("physics_step")
```
…
```python
            hits = data["hit_positions"]
```
…
```python
                hits = self._drop_self_hits(hits, data.get("hit_prim_paths"))
```

with

```python
            step, hits, paths = self._read()
```
(keep the de-dup lines between them unchanged) and

```python
                hits = self._drop_self_hits(hits, paths)
```

The Task 1/2 test fakes already expose `get_sensor_reading()` and set `rig._step_counter`, so no other test changes are needed.

- [ ] **Step 4: Run tests**

Run: `scripts/pytest-clean tests/test_lidar_profile.py tests/test_lidar_substep_fold.py tests/test_lidar_rig.py -q`
Expected: all PASS.

- [ ] **Step 5: Live re-measure** (same procedure as Task 6, Steps 1, 2, 4, 5): `get_data` must drop; record before/after.

- [ ] **Step 6: Commit**

```bash
git add simulation/tinker_sim_isaac/lidar_rig.py tests/test_lidar_profile.py
git commit -m "perf(lidar): read only positions/paths from the raw reading (was every field)"
```

---

### Task 8: Live A/B matrix — choose `BATTERY_LIDAR_CHANNELS`, contention probes (spec Unit E 1–2)

**Files:**
- Modify: `scripts/gpsr-stack` (`BATTERY_LIDAR_CHANNELS` value only; plus `TINKER_SIM_CPU_THREADS` / vision GPU defaults **only** if a probe passes the rule below)
- Test: `tests/test_gpsr_stack_dryrun.py` (only if a default changes)

- [ ] **Step 1: Pre-flight** exactly as Task 6 Step 1 (GPU, domain, deadline ≈ 2 h for the whole matrix).

- [ ] **Step 2: Run each cell** (full stack, `--manipulation mock`, vision on the sim's GPU):

| cell | command line |
|---|---|
| planar | `--lidar planar` |
| 16 ch | `TINKER_SIM_LIDAR_CHANNELS=16 … --lidar battery` |
| 24 ch | `TINKER_SIM_LIDAR_CHANNELS=24 … --lidar battery` |
| 32 ch | `TINKER_SIM_LIDAR_CHANNELS=32 … --lidar battery` |
| parity | `--lidar parity` |

For each: `up` (as in Task 6 Step 2), wait 60 s idle, run the two-waypoint drive loop from Task 6 Step 4 once (≥ 60 s of driving), note the drive's wall start/end (`date +%s.%N`), then

```bash
python3 tools/rtf_monitor.py --summarise gpsr_stack_logs/<ts>/rtf.jsonl --from <start> --to <end>
timeout -k 10 300 ./scripts/gpsr-stack down
```
Record `mean` / `min_30s` per cell, plus host loadavg (`cat /proc/loadavg`) at the start of the drive. Run every cell with `TINKER_SIM_PROFILE=1` and also record the median `ms_per_cycle.physics` from `01-sim.log`'s `step_profile` lines during the drive: the raycast plugin's own cost (scene queries + hit-path resolution) lands in that bucket, so its growth from `planar` to `parity` is the plugin's share (the contract file can't be edited to A/B `report_hit_prim_paths` directly).

- [ ] **Step 3: Choose the channel count**

`BATTERY_LIDAR_CHANNELS` = the largest of {16, 24, 32} whose driving `mean` ≥ **0.55**. If even 16 is < 0.55, stop and report to the user (that's a design question: the spec's G1 target can't be met by ray count alone). Edit the constant in `scripts/gpsr-stack` and update the comment's measurement reference.

- [ ] **Step 4: Contention probes at the chosen preset** (one drive each)
- `TINKER_SIM_CPU_THREADS=16` exported before `up`.
- `GPSR_VISION_GPU=<the other GPU>` (only if that GPU is free; ask its owner otherwise).

Rule: a probe becomes a default only if its `mean` gains **≥ 0.03** over the chosen-preset baseline **and** it doesn't claim a GPU another session owns by default. `TINKER_SIM_CPU_THREADS` qualifies → add `"TINKER_SIM_CPU_THREADS": "16"` to `sim_env` with a comment citing the measurement, plus a dry-run test:

```python
def test_sim_stage_caps_kit_worker_threads():
    assert mod.stage_commands(_cfg())[0]["env"]["TINKER_SIM_CPU_THREADS"] == "16"
```

The vision-GPU default stays **opt-in** (it would take a shared GPU by default); record its measured gain for the runbook.

- [ ] **Step 5: Run tests and commit**

```bash
scripts/pytest-clean tests/test_gpsr_stack_dryrun.py -q
git add scripts/gpsr-stack tests/test_gpsr_stack_dryrun.py
git commit -m "tune(gpsr-stack): battery lidar = <N> channels (measured RTF <x> driving)"
```

---

### Task 9: Fidelity check, bench battery, documentation (spec Unit E 3–4 + docs)

**Files:**
- Modify: `docs/gpsr-sim-runbook.md` ("Expected RTF" table ~line 211; Stage 1 options ~line 31)
- Modify: `docs/developer-log.md` (new entry at the top)

- [ ] **Step 1: Dynamic-obstacle probe at the default preset**

Bring up (Task 6 Steps 1–2, `--lidar battery`). Repeat the developer-log 2026-09-10 probe: count `/livox/lidar` points in a sensor-frame box 1.0–2.0 m ahead (|y| < 0.5, 0 < z < 2) over 6 frames; spawn a person 1.5 m ahead of the robot's **world** pose (from `spawn_pose_check` in `01-sim.log`, not TF's map frame) via `/spawn_entity`; count again; delete it; count again. Pass: count rises in 6/6 frames and returns to baseline after deletion.

- [ ] **Step 2: Doorway traversal**

Send one `navigate_to_pose` goal through a doorway on the rcw2026 map (e.g. `command_point` → `laundry_desk` (−2.988, 3.677, qz 0.7071 qw 0.7071)) with `timeout -k 5 300`. Pass: SUCCEEDED, with no `collision ahead` loop longer than 30 s in `02-bridge.log`. Tear down.

- [ ] **Step 3: One bench battery on the default preset**

The battery runner lives in tk25_decision: **ask the user** to start (or approve starting) a GPSR bench battery session against this stack (`GPSR_ROS_DOMAIN_ID=D`, default `--lidar battery`). Don't spawn or resume a GPSR decision session yourself. Pass: across the battery's wall window, `rtf_monitor --summarise` reports `mean` ≥ 0.5 and the monitor log shows no sustained WARN (no WARN lasting > 120 s).

- [ ] **Step 4: Update the runbook**

In `docs/gpsr-sim-runbook.md`:
- Stage 1 options block: add
  `./scripts/gpsr-stack up ... --lidar battery|parity|planar   # lidar preset (default battery: raycast at <N> channels)`
- "Expected RTF" table: add rows from Task 8 (planar, battery <N>, parity; driving mean) and a line that `rtf.jsonl` / `tools/rtf_monitor.py --summarise` gives per-trial RTF.
- Mention `GPSR_ROS_DOMAIN_ID`, `GPSR_VISION_GPU` and (if adopted) `TINKER_SIM_CPU_THREADS=16`.

- [ ] **Step 5: Developer-log entry** (top of `docs/developer-log.md`)

Heading uses the date of the Task 8 runs: `## <YYYY-MM-DD> — RTF budget: lidar presets, the 5 Hz half-sweep bug, measured matrix`, covering: the 0.28 starting point and its causes; the substep-fold bug with the live evidence (0.200 s period, alternating empty bins) and the fixed result; the Task 6/7 profile numbers; the Task 8 matrix table; fidelity probe and battery results.

- [ ] **Step 6: Commit and push**

```bash
git add docs/gpsr-sim-runbook.md docs/developer-log.md
git commit -m "docs(gpsr-rtf): lidar presets, RTF table, half-sweep bug, measured matrix"
git push
```
