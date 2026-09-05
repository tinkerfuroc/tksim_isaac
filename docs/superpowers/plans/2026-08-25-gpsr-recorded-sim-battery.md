# GPSR Recorded Sim Battery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run 50 generated GPSR commands against the Isaac sim (40 with manipulation mocked, 10 fully live), each recorded from an arena observer camera + the head camera into a per-run contact sheet, with an aggregated report.

**Architecture:** tinker-sim gains a world-fixed ROS-published arena camera, a per-run frame recorder, a contact-sheet builder, a two-person bench scenario, and a stack bring-up script; tk25_decision gains a sim-hybrid mock config, a `--sim-feasible` corpus mode, and a `tier2` bench runner that resets the sim, records, launches a fresh orchestrator per command, and scores from telemetry.

**Tech Stack:** Python 3.10, Isaac Sim RTX sensors (warp), rclpy, PIL, pytest(<8 in tk25_decision).

**Spec:** docs/superpowers/specs/2026-08-25-gpsr-recorded-sim-battery-design.md (this repo). Parent: docs/superpowers/specs/2026-08-23-gpsr-command-variety-testing-design.md.

## Global Constraints

- Two repos. SIM = this repo's worktree `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec` (branch `worktree-gpsr-command-variety-spec`). DEC = `/home/tinker/tk25_ws/src/tk25_decision` (branch `gpsr-two-layer-orchestrator`; create task branch `gpsr-sim-battery` off it at Task 7).
- SIM test command: `cd <SIM> && ./scripts/dev-python -m pytest tests/<file> -q` (if `scripts/dev-python` is absent use `python3 -m pytest`; check once and note in the report).
- DEC test command: `cd <DEC> && PYTHONPATH=src/behavior_tree:src/gpsr_trace:src/gpsr_debug_server ROS2_PTH_WARNED=1 uv run --python .venv_decision/bin/python --no-project --with "pytest<8" python -m pytest <paths> -q -p no:cacheprovider`.
- Tests must not need ROS, Isaac, GPU, or the network. Live behaviour is exercised only in Tasks 9–10.
- Never print or copy `/home/tinker/tk25_ws/.env`. Never kill processes by name pattern. Never touch GPU processes you did not start. `ROS_DOMAIN_ID=42` for the sim stack (Tasks 9–10 only); tier2 code never sets it.
- Commit after every task, conventional messages, correct repo. Never push to main. SIM worktree has no remote — commit only.
- Shared types: `CameraStreamSpec` (SIM `simulation/tinker_sim_isaac/camera_rig.py:18` — frozen dataclass; this plan adds `mount_translation: tuple[float,float,float] | None = None` and allows `depth_topic == ""`); DEC `CorpusEntry(id, seed, template, followups, category, text, feasibility)`, `BenchResult(entry_id, template, feasibility, tier, verdict, detail="", seconds=0.0, plan=[])`, `TaskResult(slot, status, reason, steps, planner_errors, first_seen, finished_at)`.
- Telemetry task ids are 1-based; tier2 runs exactly one command per orchestrator, so its verdict is `tasks.get(1)`.

---

### Task 1: Arena camera module (pose math + spec + env gate) [SIM]

**Files:**
- Create: `simulation/tinker_sim_isaac/arena_camera.py`
- Modify: `validation/run_sim.py:232-246` (`_arena_camera_pose` moves out, thin wrapper stays)
- Test: `tests/test_arena_camera.py`

**Interfaces:**
- Produces: `arena_camera_pose(occupancy) -> (eye, target, bounds)` (moved verbatim from run_sim); `look_at_wxyz(eye, target) -> (w,x,y,z)`; `resolve_arena_camera(env: Mapping[str,str]) -> float | None` (None = disabled, else Hz); `arena_camera_spec(occupancy, *, hz) -> CameraStreamSpec`; constants `ARENA_CAMERA_ENV = "TINKER_SIM_ARENA_CAMERA"`, `ARENA_CAMERA_HZ_ENV = "TINKER_SIM_ARENA_CAMERA_HZ"`, `ARENA_CAMERA_DEFAULT_HZ = 4.0`.
- Consumes: `CameraStreamSpec` from `camera_rig` (Task 2 adds `mount_translation`; write this task against the extended field — Task 2 is committed first if you land them in one session, otherwise coordinate: the plan orders Task 2 BEFORE Task 1 at execution time; see note below).

**Execution note:** Tasks 1 and 2 are one dispatch (same area, Task 2's field is Task 1's dependency). Implement Task 2's dataclass/optional-depth changes first, then this module, then run both test files.

- [ ] **Step 1: failing tests**

```python
# tests/test_arena_camera.py
import math
import pytest
from tinker_sim_isaac.arena_camera import (
    ARENA_CAMERA_DEFAULT_HZ, arena_camera_pose, arena_camera_spec,
    look_at_wxyz, resolve_arena_camera,
)

class _Occ:
    width, height, resolution, origin_x, origin_y = 100, 80, 0.1, -5.0, -4.0

def _rotate(q, v):
    w, x, y, z = q
    # quaternion-vector rotation q v q*
    t = (2*(y*v[2]-z*v[1]), 2*(z*v[0]-x*v[2]), 2*(x*v[1]-y*v[0]))
    return (v[0]+w*t[0]+y*t[2]-z*t[1], v[1]+w*t[1]+z*t[0]-x*t[2], v[2]+w*t[2]+x*t[1]-y*t[0])

def test_look_at_points_optical_z_at_target():
    eye, target = (0.0, 0.0, 5.0), (2.0, 1.0, 0.0)
    fwd = _rotate(look_at_wxyz(eye, target), (0.0, 0.0, 1.0))  # optical +Z
    want = [t-e for t, e in zip(target, eye)]
    n = math.sqrt(sum(c*c for c in want))
    for got, exp in zip(fwd, [c/n for c in want]):
        assert got == pytest.approx(exp, abs=1e-6)

def test_look_at_rejects_degenerate():
    with pytest.raises(ValueError):
        look_at_wxyz((1.0, 1.0, 1.0), (1.0, 1.0, 1.0))

def test_resolve_disabled_by_default():
    assert resolve_arena_camera({}) is None
    assert resolve_arena_camera({"TINKER_SIM_ARENA_CAMERA": "0"}) is None

def test_resolve_enabled_and_rate_only_lowers():
    assert resolve_arena_camera({"TINKER_SIM_ARENA_CAMERA": "1"}) == ARENA_CAMERA_DEFAULT_HZ
    env = {"TINKER_SIM_ARENA_CAMERA": "1", "TINKER_SIM_ARENA_CAMERA_HZ": "2"}
    assert resolve_arena_camera(env) == 2.0
    env["TINKER_SIM_ARENA_CAMERA_HZ"] = "30"   # may only lower
    assert resolve_arena_camera(env) == ARENA_CAMERA_DEFAULT_HZ
    env["TINKER_SIM_ARENA_CAMERA_HZ"] = "junk"
    with pytest.raises(ValueError):
        resolve_arena_camera(env)

def test_spec_shape():
    spec = arena_camera_spec(_Occ, hz=4.0)
    assert spec.name == "arena_camera"
    assert spec.color_topic == "/sim/arena_camera/image_raw"
    assert spec.depth_topic == ""            # color-only stream
    assert spec.camera_info_topics == ("/sim/arena_camera/camera_info",)
    assert spec.mount_prim == "/World/ArenaCamera"
    assert spec.mount_translation is not None
    assert (spec.width, spec.height) == (960, 540)
    assert spec.tick_rate_hz == 4.0

def test_pose_matches_run_sim_contract():
    eye, target, bounds = arena_camera_pose(_Occ)
    assert bounds == [-5.0, -4.0, 5.0, 4.0]
    assert eye[2] > target[2]
```

- [ ] **Step 2: run, verify FAIL** (`ModuleNotFoundError` / missing attrs)
- [ ] **Step 3: implement**

`arena_camera.py`: move `_arena_camera_pose` body verbatim as `arena_camera_pose`. `look_at_wxyz`: build an orthonormal basis with optical +Z = normalized(target−eye), +X = normalize(cross(world_up, Z)) with `world_up=(0,0,1)` (fall back to `(0,1,0)` if nearly parallel), +Y = cross(Z, X); convert the 3×3 (columns X,Y,Z) to wxyz via the standard trace method; raise `ValueError` on zero-length direction. `resolve_arena_camera`: enabled iff env[ARENA_CAMERA_ENV] is a truthy literal ("1","true","yes" case-insensitive); rate = min(DEFAULT, parsed override); non-numeric override raises ValueError. `arena_camera_spec`: `CameraStreamSpec(name="arena_camera", color_topic="/sim/arena_camera/image_raw", depth_topic="", camera_info_topics=("/sim/arena_camera/camera_info",), frame_id="arena_camera_optical_frame", mount_prim="/World/ArenaCamera", mount_rotation_wxyz=look_at_wxyz(eye, target), mount_translation=tuple(eye), width=960, height=540, horizontal_fov_deg=70.0, tick_rate_hz=hz)` where `eye, target, _ = arena_camera_pose(occupancy)`. In `run_sim.py`, replace `_arena_camera_pose`'s body with `from tinker_sim_isaac.arena_camera import arena_camera_pose; return arena_camera_pose(occupancy)` (keep the name so line-786 callers are untouched).

- [ ] **Step 4: run tests to PASS** (both this file and Task 2's)
- [ ] **Step 5: commit** `feat(sim): world-fixed arena observer camera spec, pose math and env gate`

### Task 2: CameraRig world-fixed mounts + color-only streams [SIM]

**Files:**
- Modify: `simulation/tinker_sim_isaac/camera_rig.py` (dataclass :18-32, `initialize` :403-460, `capture` :484-554, `_pinned`/`_depth_gpu_out` sizing wherever buffers are allocated)
- Modify: `simulation/tinker_sim_isaac/ros_gateway.py` (:195-231 stream setup, `publish_cameras` :1180+)
- Test: `tests/test_camera_rig_worldfixed.py`

**Interfaces:**
- Produces: `CameraStreamSpec.mount_translation: tuple[float,float,float] | None = None` (None = robot-mounted, unchanged behaviour); `depth_topic == ""` means color-only: no depth annotator, no depth pinned buffer, `capture()` returns `(rgb, None)` for that camera, gateway creates no depth publisher and publishes color+info only.
- Consumes: existing `load_camera_specs` (must keep parsing `hardware-parity.json` unchanged — those specs get `mount_translation=None` by default).

- [ ] **Step 1: failing tests**

```python
# tests/test_camera_rig_worldfixed.py
from pathlib import Path
from tinker_sim_isaac.camera_rig import CameraStreamSpec, load_camera_specs

ROOT = Path(__file__).resolve().parents[1]

def _spec(**kw):
    base = dict(name="arena_camera", color_topic="/sim/arena_camera/image_raw",
                depth_topic="", camera_info_topics=("/sim/arena_camera/camera_info",),
                frame_id="arena_camera_optical_frame", mount_prim="/World/ArenaCamera",
                mount_rotation_wxyz=(1.0, 0.0, 0.0, 0.0), width=960, height=540,
                horizontal_fov_deg=70.0, tick_rate_hz=4.0,
                mount_translation=(1.0, 2.0, 6.0))
    base.update(kw)
    return CameraStreamSpec(**base)

def test_parity_specs_unchanged():
    specs = load_camera_specs(ROOT / "simulation/sensors/hardware-parity.json")
    assert all(s.mount_translation is None for s in specs)
    assert all(s.depth_topic for s in specs)

def test_world_fixed_flag():
    s = _spec()
    assert s.mount_translation == (1.0, 2.0, 6.0)
    assert s.depth_topic == ""

def test_color_only_helper():
    from tinker_sim_isaac.camera_rig import is_color_only
    assert is_color_only(_spec())
    assert not is_color_only(_spec(depth_topic="/x"))
```

- [ ] **Step 2: run, verify FAIL**
- [ ] **Step 3: implement**

Dataclass: add `mount_translation: tuple[float, float, float] | None = None` as the LAST field (keeps positional construction in `load_camera_specs` working). Add module-level `def is_color_only(spec) -> bool: return spec.depth_topic == ""`. `initialize()`: when `spec.mount_prim.startswith("/")`, skip the robot-prim search; `UsdGeom.Xform.Define(stage, spec.mount_prim)`, then on the xform prim: `AddTranslateOp(PrecisionDouble).Set(Gf.Vec3d(*spec.mount_translation))` — the RtxCamera child at `f"{spec.mount_prim}/rtx_camera"` then gets the same orient treatment as today (`mount_rotation_wxyz` already carries the full look-at orientation; do NOT also apply OPTICAL_TO_USD flips for world-fixed specs — `look_at_wxyz` produces the optical-convention rotation directly, so world-fixed uses the spec quaternion as-is; keep the existing per-mount behaviour for robot mounts). When `is_color_only(spec)`, create `CameraSensor(..., annotators=[COLOR_ANNOTATOR])`, allocate only the rgb pinned buffer, and in `capture()` skip the depth branch and yield `(rgb, None)`. `ros_gateway.py`: in stream setup, `depth_pub = None if is_color_only(spec) else node.create_publisher(...)`; in `publish_cameras`, guard the depth publish with `if stream["depth_pub"] is not None and depth is not None`. Do not register the arena camera in `camera_info` parity or census paths (nothing to do — those read `hardware-parity.json`).

- [ ] **Step 4: run tests to PASS**; also run the existing `tests/test_camera_publish_equivalence.py` and any camera-spec tests to prove no regression.
- [ ] **Step 5: commit** `feat(sim): CameraRig supports world-fixed color-only cameras`

### Task 3: run_sim wiring for the arena camera [SIM]

**Files:**
- Modify: `validation/run_sim.py` (~:958-1000)
- Test: `tests/test_run_sim_arena_wiring.py`

**Interfaces:**
- Consumes: `resolve_arena_camera`, `arena_camera_spec` (Task 1); `CameraRig` (Task 2).
- Produces: pure helper in `validation/run_sim.py`: `def _with_arena_camera(specs, occupancy, env) -> tuple[tuple[CameraStreamSpec, ...], float]` returning (possibly-extended specs, `robot_min_hz`) where `robot_min_hz = min(tick_rate_hz of the ORIGINAL specs)` — the arena camera must not lower the robot-camera stride.

- [ ] **Step 1: failing test**

```python
# tests/test_run_sim_arena_wiring.py
from validation.run_sim import _with_arena_camera
from tests.test_arena_camera import _Occ  # reuse fixture class

class _S:  # minimal stand-in with tick_rate_hz
    def __init__(self, hz): self.tick_rate_hz = hz

def test_disabled_returns_originals():
    specs = (_S(30.0), _S(30.0))
    out, robot_hz = _with_arena_camera(specs, _Occ, {})
    assert out == specs and robot_hz == 30.0

def test_enabled_appends_arena_and_keeps_robot_hz():
    specs = (_S(30.0),)
    out, robot_hz = _with_arena_camera(specs, _Occ, {"TINKER_SIM_ARENA_CAMERA": "1"})
    assert len(out) == 2 and out[-1].name == "arena_camera"
    assert robot_hz == 30.0
```

(If importing `validation.run_sim` at module level pulls in `isaacsim`, move `_with_arena_camera` to `tinker_sim_isaac/arena_camera.py` instead and import from there — note it in the report; the test then imports from that module.)

- [ ] **Step 2: run, verify FAIL**
- [ ] **Step 3: implement** — helper as specified; in `run_sim.py`'s camera block: compute `camera_specs, robot_min_hz = _with_arena_camera(camera_specs, occupancy, os.environ)` right after the head-aim correction (occupancy is already loaded for `_arena_camera_pose` at :786; reuse that object — if it is only available under `--livestream`, load it unconditionally when the arena camera is enabled, same loader call), and change the `camera_hz = _resolve_camera_hz(min(...))` call to use `robot_min_hz`. Print one `[sim] arena camera enabled at N Hz -> /sim/arena_camera/image_raw` line.
- [ ] **Step 4: tests PASS**
- [ ] **Step 5: commit** `feat(sim): opt-in arena observer camera wired into run_sim`

### Task 4: Run recorder [SIM]

**Files:**
- Create: `validation/gpsr_run_recorder.py`
- Test: `tests/test_gpsr_run_recorder.py`

**Interfaces:**
- Produces: pure `class FrameSink(out_dir: Path, label: str, interval_s: float, max_frames: int)` with `offer(stamp_s: float, rgb_bytes: bytes, width: int, height: int) -> Path | None` (saves JPEG and returns path when accepted — i.e. first frame or `stamp_s >= last_accepted + interval_s` and under `max_frames`; else None); `main(argv)` CLI `--out DIR --topic /a=arena --topic /b=head --interval 1.0 --max-frames 900` building one rclpy node, one subscription per `--topic topic=label` (sensor_msgs/Image, rgb8, QoS reliable depth 1), stamp from `msg.header.stamp` (sim time), SIGINT-clean shutdown writing `DIR/recorder-meta.json` `{"labels": {label: {"frames": n, "first_stamp": s, "last_stamp": s}}, "started_wall": iso, "ended_wall": iso}`.
- Files land as `DIR/frames/<label>/<seq:04d>_<int(stamp*1000)>.jpg`.

- [ ] **Step 1: failing tests**

```python
# tests/test_gpsr_run_recorder.py
from pathlib import Path
from PIL import Image
from validation.gpsr_run_recorder import FrameSink

def _rgb(w, h, val):  # solid-colour rgb8 buffer
    return bytes([val, 0, 0]) * (w * h)

def test_sink_saves_first_and_respects_interval(tmp_path):
    s = FrameSink(tmp_path, "arena", interval_s=1.0, max_frames=10)
    assert s.offer(0.0, _rgb(4, 3, 200), 4, 3) is not None
    assert s.offer(0.5, _rgb(4, 3, 200), 4, 3) is None
    p = s.offer(1.05, _rgb(4, 3, 100), 4, 3)
    assert p is not None and p.name.startswith("0001_1050")
    img = Image.open(p)
    assert img.size == (4, 3)

def test_sink_caps_frames(tmp_path):
    s = FrameSink(tmp_path, "head", interval_s=0.0, max_frames=2)
    assert s.offer(0.0, _rgb(2, 2, 1), 2, 2)
    assert s.offer(1.0, _rgb(2, 2, 1), 2, 2)
    assert s.offer(2.0, _rgb(2, 2, 1), 2, 2) is None

def test_meta_summary(tmp_path):
    s = FrameSink(tmp_path, "arena", interval_s=1.0, max_frames=10)
    s.offer(2.0, _rgb(2, 2, 1), 2, 2); s.offer(3.5, _rgb(2, 2, 1), 2, 2)
    assert s.summary() == {"frames": 2, "first_stamp": 2.0, "last_stamp": 3.5}
```

- [ ] **Step 2: run, verify FAIL**
- [ ] **Step 3: implement** — `FrameSink` pure (PIL `Image.frombytes("RGB", (w, h), buf)`, quality 85); the rclpy `main` imports rclpy lazily inside `main()` so tests never touch ROS; callback converts `msg.data`/`msg.width`/`msg.height` (assert `msg.encoding == "rgb8"`, count-and-skip otherwise) and stamp `msg.header.stamp.sec + nanosec*1e-9`.
- [ ] **Step 4: tests PASS**
- [ ] **Step 5: commit** `feat(sim): per-run GPSR frame recorder`

### Task 5: Contact sheet builder [SIM]

**Files:**
- Create: `tools/contact_sheet.py`
- Test: `tests/test_contact_sheet.py`

**Interfaces:**
- Produces: `sample_evenly(items: list, k: int) -> list` (all items if `len<=k`, else indices `round(i*(n-1)/(k-1))`, unique, order-preserving); `build_sheet(run_dir: Path, meta: dict, out: Path, columns: int = 12) -> Path`; CLI `--run-dir --meta run.json --out sheet.jpg [--columns 12]`.
- `meta` (run.json) keys: `id, text, template, feasibility, tier, verdict, detail, seconds` — tolerate missing keys.
- Layout constants: tile width 320 (height scaled by aspect), caption strip 18 px per tile (`t=<stamp>s` from the filename's `_<ms>` suffix), header band 88 px (line 1: `<id>  [<verdict>]  <seconds rounded>s  <tier>`; line 2-3: wrapped command text), verdict colours PASS `#2e7d32`, FAIL `#c62828`, TIMEOUT `#ef6c00`, ERROR `#616161`, default `#455a64`. Row order: `arena` then `head`; a label with no frames renders a 40 px grey band "no <label> frames captured". JPEG quality 80.

- [ ] **Step 1: failing tests**

```python
# tests/test_contact_sheet.py
import json
from pathlib import Path
from PIL import Image
from tools.contact_sheet import build_sheet, sample_evenly

def test_sample_evenly():
    assert sample_evenly([1, 2, 3], 5) == [1, 2, 3]
    assert sample_evenly(list(range(10)), 4) == [0, 3, 6, 9]
    assert sample_evenly([], 4) == []

def _mk_frames(d, label, n, size=(32, 18)):
    p = d / "frames" / label; p.mkdir(parents=True)
    for i in range(n):
        Image.new("RGB", size, (i * 10 % 255, 80, 80)).save(p / f"{i:04d}_{i*1000}.jpg")

def test_build_sheet_two_rows(tmp_path):
    _mk_frames(tmp_path, "arena", 20); _mk_frames(tmp_path, "head", 3)
    meta = {"id": "c001", "text": "go to the kitchen table", "verdict": "PASS",
            "seconds": 93.2, "tier": "T2"}
    out = build_sheet(tmp_path, meta, tmp_path / "sheet.jpg", columns=12)
    img = Image.open(out)
    assert img.width == 12 * 320
    assert img.height > 88 + 2 * 18  # header + two captioned rows

def test_build_sheet_missing_label_degrades(tmp_path):
    _mk_frames(tmp_path, "head", 2)
    out = build_sheet(tmp_path, {"id": "x", "verdict": "ERROR"}, tmp_path / "s.jpg")
    assert out.exists()  # no crash; arena row is a placeholder band
```

- [ ] **Step 2: run, verify FAIL**
- [ ] **Step 3: implement** — pure PIL; sort each label's files by name; default font (`ImageDraw.text`); wrap text at ~110 chars/line.
- [ ] **Step 4: tests PASS**
- [ ] **Step 5: commit** `feat(sim): contact-sheet builder for GPSR runs`

### Task 6: Bench scenario + gpsr-stack script [SIM]

**Files:**
- Create: `simulation/scenarios/gpsr-rcw2026-bench.json`, `scripts/gpsr-stack`
- Test: `tests/test_gpsr_stack_dryrun.py` (+ scenario covered by the existing scenario-schema test — find it with `grep -rl "scenarios" tests/` and add the new file to its parametrization if it enumerates files explicitly)

**Scenario:** copy `simulation/scenarios/gpsr-rcw2026.json`; add to `actors[]` a second entry `{"id": "livingroom_person", "asset_uri": <same as kitchen_person>, "pose": near the sofa — pick a free cell ≥2 m from the sofa waypoint (sofa pose is in DEC `constants.rcw2026.json` `possible_poses.sofa`), same orientation convention as kitchen_person}`. Keep everything else identical.

**scripts/gpsr-stack** (python, executable, `#!/usr/bin/env python3`): subcommands `up|down|status`, flags `--scenario gpsr-rcw2026-bench --seed 0 --manipulation {mock,live} --dry-run --log-dir gpsr_stack_logs`. Structure the command assembly as a pure function so it is testable:

```python
def stage_commands(cfg) -> list[dict]:
    """Returns ordered [{name, cmd (list[str]), env (dict), cwd, gate}] for stages 1-5.
    Stage 4 present only when cfg.manipulation == "live".
    gate = census subset name checked after the stage ("sim", "bridge", "vision", "manipulation", "nav")."""
```

Stage content = the runbook commands (docs/gpsr-sim-runbook.md, Stages 1-5), with stage-1 env `ROS_DOMAIN_ID=42 TINKER_SIM_CAMERA_HZ=12 TINKER_SIM_CONTROL_HZ=60 TINKER_SIM_ARENA_CAMERA=1 TINKER_SIM_HEAD_CAMERA_AIM=level-forward`, and stage 4 launched with `CUDA_VISIBLE_DEVICES=<the non-sim GPU>` — detect the sim GPU as the one `--dry-run` cannot know; implement as flag `--manip-gpu N` (required with `--manipulation live`). `up` runs stages in order via `subprocess.Popen(start_new_session=True)`, writes `<log-dir>/<ts>/<stage>.log` and `<stage>.pgid`, then between stages polls readiness with `tools/gpsr_interface_census.py` (invoke it as a subprocess; treat non-zero as not-ready; 180 s per stage, then abort with the census output). `down` reads the newest `<log-dir>/*/`'s pgid files, SIGINTs each PGID in REVERSE stage order with 10 s grace then SIGKILL, then prints `nvidia-smi --query-compute-apps=pid,name,used_memory --format=csv`. `status` runs the census once and prints it. NEVER `pkill`/`killall`.

- [ ] **Step 1: failing test**

```python
# tests/test_gpsr_stack_dryrun.py
import importlib.util, sys
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    "gpsr_stack", Path(__file__).resolve().parents[1] / "scripts" / "gpsr-stack")
mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)

def _cfg(**kw):
    base = dict(scenario="gpsr-rcw2026-bench", seed=0, manipulation="mock",
                manip_gpu=None, log_dir="gpsr_stack_logs")
    base.update(kw); return mod.StackConfig(**base)

def test_mock_skips_stage4():
    names = [s["name"] for s in mod.stage_commands(_cfg())]
    assert names == ["sim", "bridge", "vision", "nav_extras"]

def test_live_includes_stage4_with_gpu():
    stages = mod.stage_commands(_cfg(manipulation="live", manip_gpu=1))
    manip = [s for s in stages if s["name"] == "manipulation"][0]
    assert manip["env"]["CUDA_VISIBLE_DEVICES"] == "1"

def test_sim_stage_env():
    sim = mod.stage_commands(_cfg())[0]
    env = sim["env"]
    assert env["ROS_DOMAIN_ID"] == "42"
    assert env["TINKER_SIM_ARENA_CAMERA"] == "1"
    assert "--scenario" in sim["cmd"] and "gpsr-rcw2026-bench" in sim["cmd"]

def test_scenario_json_has_two_actors():
    import json
    data = json.loads((Path(__file__).resolve().parents[1] /
                       "simulation/scenarios/gpsr-rcw2026-bench.json").read_text())
    assert len(data["actors"]) == 2
```

- [ ] **Step 2: run, verify FAIL**
- [ ] **Step 3: implement** (read the runbook stage commands carefully, including the two manual vision gaps — the named `head_camera_server` node and the pan/tilt state publisher — as extra entries inside the "vision" stage's command list; a stage may be a list of Popens)
- [ ] **Step 4: tests PASS**
- [ ] **Step 5: commit** `feat(sim): bench scenario with second person and gpsr-stack bring-up script`

### Task 7: sim-hybrid mock config + sim-feasible corpus [DEC]

Create branch `gpsr-sim-battery` off `gpsr-two-layer-orchestrator` first.

**Files:**
- Create: `src/behavior_tree/behavior_tree/mock_config.sim-hybrid.json`
- Modify: `src/behavior_tree/behavior_tree/GPSR/bench/corpus.py`, `src/behavior_tree/behavior_tree/GPSR/gpsr_bench.py` (gen CLI)
- Test: `src/behavior_tree/test/test_gpsr_bench_sim_corpus.py`, extend `test_gpsr_bench_mock_config.py`

**Interfaces:**
- Produces: `SIM_INFEASIBLE: frozenset[str]` in corpus.py = `{"followNameFromBeacToRoom","followPrsAtLoc","followPrs","followPrsToRoom","guideNameFromBeacToBeac","guidePrsFromBeacToBeac","guideClothPrsFromBeacToBeac","guidePrsToBeacon","greetClothDscInRm","countClothPrsInRoom","putObjInTrash"}`; `generate_sim_corpus(constants_path, *, seed, count, templates=None) -> tuple[list[CorpusEntry], dict[str,int]]` — round-robins templates like `generate_corpus` but SKIPS any expansion whose template or followups intersect SIM_INFEASIBLE (re-drawing until `count` entries exist; skipped counts returned `{template_or_followup: n}` keyed by the infeasible name that triggered the skip); entry ids `s{seed}-{i:03d}-{template}`.
- CLI: `gpsr-bench gen --sim-feasible --count 40 [--templates a,b,c] --seed N --out F` → writes line 1 `{"_skipped": {...}, "_seed": N, "_mode": "sim-feasible"}` then one CorpusEntry JSON per line (readers: `load_corpus` must skip a first line starting with `{"_` — check how tier0/tier1 load the corpus and update the loader once, with a test).
- `mock_config.sim-hybrid.json`: copy `mock_config.sim.json`, set ONLY the `manipulation` subsystem `enabled: true` with all its nodes `IMMEDIATE` (copy the manipulation node list from `mock_config.bench.json`), keep the other five subsystems `enabled: false`, `keyboard_control.enabled: false`.

- [ ] **Step 1: failing tests**

```python
# src/behavior_tree/test/test_gpsr_bench_sim_corpus.py
import json
from pathlib import Path
from behavior_tree.GPSR.bench.corpus import (
    SIM_INFEASIBLE, FEASIBILITY, generate_sim_corpus, load_corpus,
)

CONSTANTS = Path(__file__).resolve().parents[1] / "behavior_tree/GPSR/constants.rcw2026.json"

def test_infeasible_names_are_known():
    assert SIM_INFEASIBLE <= set(FEASIBILITY)

def test_sim_corpus_excludes_infeasible_and_hits_count():
    entries, skipped = generate_sim_corpus(CONSTANTS, seed=2026, count=40)
    assert len(entries) == 40
    for e in entries:
        assert e.template not in SIM_INFEASIBLE
        assert not (set(e.followups) & SIM_INFEASIBLE)
    assert isinstance(skipped, dict)

def test_sim_corpus_deterministic():
    a, _ = generate_sim_corpus(CONSTANTS, seed=7, count=10)
    b, _ = generate_sim_corpus(CONSTANTS, seed=7, count=10)
    assert [e.text for e in a] == [e.text for e in b]

def test_header_line_roundtrip(tmp_path):
    entries, skipped = generate_sim_corpus(CONSTANTS, seed=7, count=5)
    out = tmp_path / "c.jsonl"
    from behavior_tree.GPSR.bench.corpus import write_corpus
    write_corpus(out, entries, header={"_skipped": skipped, "_seed": 7, "_mode": "sim-feasible"})
    loaded = load_corpus(out)
    assert [e.id for e in loaded] == [e.id for e in entries]
```

(If `write_corpus`/`load_corpus` don't exist yet under those names, find the actual write/read helpers used by `gpsr_bench.py` gen/tier0 and extend those, adjusting the test imports — the behaviour contract is what matters: a `{"_`-prefixed first line is metadata and must be skipped by every corpus reader.)

Mock-config test additions: sim-hybrid file parses; exactly one subsystem (`manipulation`) enabled; `is_full_mock_mode()` is False under it.

- [ ] **Step 2: run, verify FAIL**
- [ ] **Step 3: implement**
- [ ] **Step 4: tests PASS** (plus the whole existing `test_gpsr_bench_*` suite)
- [ ] **Step 5: commit** `feat(gpsr): sim-hybrid mock config and sim-feasible corpus generation`

### Task 8: tier2 runner + CLI [DEC]

**Files:**
- Create: `src/behavior_tree/behavior_tree/GPSR/bench/tier2.py`
- Modify: `src/behavior_tree/behavior_tree/GPSR/gpsr_bench.py` (subcommand `tier2`), `src/behavior_tree/behavior_tree/GPSR/bench/report.py` (accept sheet path in BenchResult detail — no schema change: tier2 stores `sheet=<relpath>` inside `BenchResult.detail` suffix ` | sheet=<path>` and SUMMARY.md's Runs section lists `- <id> **<verdict>** — [sheet](<relpath>)`; add `runs_section(results) -> str` helper)
- Test: `src/behavior_tree/test/test_gpsr_bench_tier2.py`

**Interfaces (consume tier1's exact patterns — read `bench/tier1.py` first):**

```python
DEFAULT_RESET_CMD = ["ros2", "service", "call", "/reset_simulation",
                     "simulation_interfaces/srv/ResetSimulation", "{}"]

def run_tier2(entries, *, mock_config: Path, constants: Path, out_dir: Path,
              timeout_s: float, tier_label: str = "T2",
              launcher=DEFAULT_LAUNCHER, reset_cmd=DEFAULT_RESET_CMD,
              recorder_cmd: list[str] | None = None,
              sheet_cmd: list[str] | None = None,
              settle_s: float = 10.0, halt_after_errors: int = 3,
              live_llm: bool = True) -> list[BenchResult]:
```

Per entry `e` (run dir `out_dir/runs/{e.id}`, absolute):
1. `subprocess.run(reset_cmd, timeout=60)`; nonzero/timeout → verdict ERROR `reset failed: ...`, continue to next entry. Sleep `settle_s`.
2. If `recorder_cmd`: substitute `{run_dir}` in each arg, `Popen(start_new_session=True)`, log to `run_dir/recorder.log`.
3. Env: reuse `tier1.bench_env` (import it) with the single command, `mock_config`, `constants`, plan_dir=`run_dir` (absolute), live_llm; launch orchestrator exactly as tier1 `run_group` does (own process group, cwd=run_dir, log `run_dir/orchestrator.log`), single slot: poll `parse_events` for `tasks.get(1)` terminal status; per-task clock from first_seen (slot 0 clock = start), timeout `timeout_s` → TIMEOUT; process exit → score what telemetry shows.
4. Stop orchestrator (reuse/extract tier1's `_stop`), stop recorder (SIGINT its process group, 15 s, then SIGKILL).
5. Write `run_dir/run.json` `{id, text, template, feasibility, tier, verdict, detail, seconds}`.
6. If `sheet_cmd`: substitute `{run_dir}` `{run_json}` `{out}` (out=`run_dir/sheet.jpg`), `subprocess.run(..., timeout=120)`; failure appends `; sheet failed` to detail but never changes the verdict. Append ` | sheet=runs/{e.id}/sheet.jpg` to detail on success.
7. Append BenchResult; if the last `halt_after_errors` results are all ERROR → write `out_dir/HALTED` with the reason and stop.

CLI: `gpsr-bench tier2 --corpus F --out DIR --mock-config M --timeout 600 [--tier-label T2+] [--recorder-cmd "..."] [--sheet-cmd "..."] [--reset-cmd "..."] [--settle 10] [--limit N] [--start K] [--offline-planner]` (`--*-cmd` strings are `shlex.split`; `--limit/--start` slice entries after loading). Report/meta via the same `report.py` writer as tier1 with `meta.tier` set from `--tier-label`.

- [ ] **Step 1: failing tests** — mirror `test_gpsr_bench_tier1.py`'s fake-launcher pattern (a fake `launcher` script that writes events.jsonl; a fake `reset_cmd` = `["true"]`; fake `recorder_cmd`/`sheet_cmd` = tiny `sh -c` writing marker files with the substituted paths). Cases: (1) PASS run invokes reset→recorder→sheet in order, run.json written, detail carries `sheet=`; (2) reset failure → ERROR, orchestrator never launched (marker absent); (3) timeout → TIMEOUT and recorder stopped (its marker-on-SIGINT written); (4) three consecutive ERRORs → HALTED file, remaining entries unscored; (5) `{run_dir}` substitution absolute.
- [ ] **Step 2: run, verify FAIL**
- [ ] **Step 3: implement** (extract shared helpers from tier1 rather than copy where trivial: `_stop`, `bench_env`, `_events_file` — import, don't duplicate)
- [ ] **Step 4: tests PASS** (whole `test_gpsr_bench_*` suite green)
- [ ] **Step 5: commit** `feat(gpsr): tier-2 sim bench runner with per-run reset, recording and contact sheets`

### Task 9: Live smoke — 2 hybrid + 1 manipulation run [BOTH, live]

No new production code; fixes discovered here go through normal fix-round dispatches. Steps:

- [ ] 1. Preflight: `nvidia-smi --query-compute-apps=pid,name --format=csv` and `ROS_DOMAIN_ID=42 ros2 node list` (expect empty / no foreign stack; if a foreign stack is up, STOP and report — do not tear down other people's processes). Identify the sim GPU (the one Isaac will use) and pick the other for `--manip-gpu`. Record `ls` of the anygrasp checkpoint path (find it: `grep -rn "checkpoint" <tk25_manipulation anygrasp package>`); note exists/missing.
- [ ] 2. `colcon build --packages-select behavior_tree --base-paths src/tk25_decision` from `/home/tinker/tk25_ws` (only that package).
- [ ] 3. Generate the smoke corpus: `gpsr-bench gen --sim-feasible --count 2 --seed 2026 --out .../bench/smoke-t2.jsonl` and a 1-entry manip corpus (`--templates takeObjFromPlcmt --count 1`).
- [ ] 4. `scripts/gpsr-stack up --scenario gpsr-rcw2026-bench --seed 0 --manipulation mock` (SIM repo). Confirm census green; confirm `/sim/arena_camera/image_raw` publishes (`ros2 topic hz --window 8`, expect ~4 Hz) and head camera ~12 Hz.
- [ ] 5. Run tier2 (DEC, env `ROS_DOMAIN_ID=42`): `gpsr-bench tier2 --corpus smoke-t2.jsonl --out .../bench/smoke-t2 --mock-config .../mock_config.sim-hybrid.json --timeout 600 --recorder-cmd "python3 <SIM>/validation/gpsr_run_recorder.py --out {run_dir} --topic /sim/arena_camera/image_raw=arena --topic /camera/color/image_raw=head --interval 1.0" --sheet-cmd "python3 <SIM>/tools/contact_sheet.py --run-dir {run_dir} --meta {run_json} --out {out}"`. Verify: both runs produce ≥5 frames per label, sheets exist and open, verdicts recorded, reset worked twice (measure reset+settle wall time; record it).
- [ ] 6. `gpsr-stack down`, then `up --manipulation live --manip-gpu <other>`; run the 1-entry manip corpus with `--tier-label T2+ --timeout 900 --mock-config mock_config.sim.json`. Record whether grasp actually executes or what blocks it (anygrasp checkpoint / cumotion). `gpsr-stack down`. Verify GPU clear.
- [ ] 7. Write the smoke report (frames rates, reset time, RTF observations, Nav2 timeout lines if any, anygrasp verdict) and commit any config-tuning changes it forced.

### Task 10: 50-run battery + report [BOTH, live, detached]

- [ ] 1. Generate the battery corpora (seed 2026): `gen --sim-feasible --count 40 --out .../bench/t2-2026/corpus.jsonl`; `gen --sim-feasible --count 10 --templates takeObjFromPlcmt,bringMeObjFromPlcmt,findObjInRoom --out .../bench/t2plus-2026/corpus.jsonl`. Commit both.
- [ ] 2. Stack up (hybrid). Launch the 40-run tier2 **fully detached** (`setsid nohup` wrapper script writing `EXIT=` marker, exactly like the Phase-1 rerun scripts) with the smoke's recorder/sheet commands, `--timeout 600`. Do not wait inside an agent; the controller monitors the marker.
- [ ] 3. On completion: stack down, stack up `--manipulation live`, launch the 10-run T2+ battery detached (`--timeout 900 --tier-label T2+`). Stack down after; verify GPU clear.
- [ ] 4. Report: run the report writer over both runs' results into `SUMMARY.md` (+ merged battery `SUMMARY.md` combining both with per-class totals); `.gitignore` gains `*/bench/t2-*/runs/`; commit corpus + report.json + SUMMARY.md for both (NOT frames/sheets). Deliver all 50 `sheet.jpg` to the user (SendUserFile by the controller, batched) and state their on-disk paths.
- [ ] 5. Final analysis in the report: per-template matrix vs T1 expectations, orchestrator/sim findings (Nav2 timeouts, reset behaviour, planner latency, manipulation blocker status), and the skipped-template table from the corpus headers.

## Execution notes

- Task pairs (1+2) and (7) suit single dispatches; 9 and 10 are controller-coordinated with fresh agents per step and detached long jobs (fresh agent per fix round — standing user instruction).
- The tinker-sim worktree has no remote: commit only. DEC branch `gpsr-sim-battery` merges back only on user instruction.
- If `validation/run_sim.py` cannot be imported in tests without Isaac (Task 3 note), keep all pure helpers in `tinker_sim_isaac/` modules.
