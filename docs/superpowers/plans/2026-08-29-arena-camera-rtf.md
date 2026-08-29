# Arena Camera RTF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Bring the simulator's real-time factor with the arena observer camera on (currently 0.21–0.24) to within ~0.1 of the arena-off figure (0.81 idle / 0.68 driving under the bench stack), without changing the hardware-parity cameras or re-opening the CUDA-700 crash.

**Architecture:** First a profiled measurement spike (Task 1–3) separates the three candidate costs — the all-products DLAA switch, a fixed per-render-product pump cost, and the arena product's own pixel count — with two throwaway-grade env hooks and a `step_profile` summariser. The result picks exactly one structural fix (Task 4a or 4b). Independent cheap wins (Task 5) and the bench policy + verification (Task 6–7) follow regardless.

**Tech Stack:** Python 3.12 (`.venv`), Isaac Sim 5 (`isaacsim.sensors.experimental.rtx` `RtxCamera`/`CameraSensor`, `carb.settings`, `omni.replicator`), Warp, ROS 2 Humble (for the real-stack verification only), pytest.

**Spec:** `docs/superpowers/specs/2026-08-29-arena-camera-rtf-design.md`

## Global Constraints

- Hardware-parity cameras (head, wrist) keep their contract resolution and rate; only the arena camera's spec may change.
- The arena rate override may only lower the default (`resolve_arena_camera`), never raise it; the same rule applies to the new size override.
- Arena-enabled sims need ≥5 GB free VRAM on their card (11 GB cards); run every sim in this plan with `CUDA_VISIBLE_DEVICES=1` and confirm with `nvidia-smi` that no other process holds >5 GB on it first. **GPU 1 is shared with the `gpsr command testing robustness` session's stack — coordinate before every live run.**
- A CUDA-poisoned sim ignores SIGINT/SIGTERM: `kill -9` the `run_sim.py` process is fine; never SIGKILL Nav2.
- Run pytest through `scripts/pytest-clean` (unsets the ROS overlay; the venv's pytest otherwise fails at collection). Worktrees have no `.venv` — the wrapper falls back to the main checkout's.
- Known pre-existing failures in `tests/test_gpsr_stack_dryrun.py` on a worktree: 8 tests raise `produced model bundle not found` (bundle lives in the main checkout only). Ignore those; Task 6 fixes the two `TINKER_SIM_ARENA_CAMERA not in env` failures.
- Fix narratives go to `docs/developer-log.md`; `docs/gpsr-sim-runbook.md` gets only final knob values.
- Commit after every task; do not push (no remote); never touch `main`.
- Subagents: Sonnet. Never Opus.

---

### Task 1: Spike env hooks — `TINKER_SIM_STABLE_AA` and `TINKER_SIM_ARENA_CAMERA_SIZE`

Two opt-in hooks the spike needs: force DLAA on with the arena off (variant D), and shrink the arena render (variant E). Both are pure-Python resolution helpers with tests; the size hook stays if Task 5 adopts it, the AA hook stays as an A/B aid.

**Files:**
- Modify: `simulation/tinker_sim_isaac/arena_camera.py` (after `resolve_arena_camera`, ~line 156; `arena_camera_spec` lines 172–199)
- Modify: `validation/run_sim.py:1042-1059` (`_with_arena_camera`, `camera_rig.initialize(...)`)
- Test: `tests/test_arena_camera.py`, `tests/test_run_sim_arena_wiring.py`

**Interfaces:**
- Produces: `arena_camera.ARENA_CAMERA_SIZE_ENV = "TINKER_SIM_ARENA_CAMERA_SIZE"`, `arena_camera.ARENA_CAMERA_DEFAULT_SIZE = (960, 540)`, `arena_camera.resolve_arena_camera_size(env) -> tuple[int, int]`, `arena_camera_spec(occupancy, *, hz, size=ARENA_CAMERA_DEFAULT_SIZE)`; `run_sim.STABLE_AA_ENV = "TINKER_SIM_STABLE_AA"`, `run_sim._stable_aa_requested(arena_enabled: bool, env: dict) -> bool`.

- [ ] **Step 1: Write the failing tests for the size override**

Append to `tests/test_arena_camera.py`:

```python
def test_resolve_size_default_and_only_lowers():
    from tinker_sim_isaac.arena_camera import (
        ARENA_CAMERA_DEFAULT_SIZE, resolve_arena_camera_size,
    )
    assert resolve_arena_camera_size({}) == ARENA_CAMERA_DEFAULT_SIZE == (960, 540)
    assert resolve_arena_camera_size({"TINKER_SIM_ARENA_CAMERA_SIZE": "640x360"}) == (640, 360)
    # may only lower: a larger request is clamped to the default
    assert resolve_arena_camera_size({"TINKER_SIM_ARENA_CAMERA_SIZE": "1920x1080"}) == (960, 540)
    for bad in ("640", "640x", "axb", "0x0", "-1x10"):
        with pytest.raises(ValueError):
            resolve_arena_camera_size({"TINKER_SIM_ARENA_CAMERA_SIZE": bad})


def test_spec_takes_size():
    spec = arena_camera_spec(_Occ, hz=2.0, size=(640, 360))
    assert (spec.width, spec.height) == (640, 360)
    assert spec.horizontal_fov_deg == 70.0   # FOV is independent of size
```

- [ ] **Step 2: Run to verify they fail**

Run: `scripts/pytest-clean tests/test_arena_camera.py -q`
Expected: 2 failed — `ImportError`/`cannot import name 'resolve_arena_camera_size'` and `TypeError: ... unexpected keyword argument 'size'`.

- [ ] **Step 3: Implement the size override**

In `simulation/tinker_sim_isaac/arena_camera.py`, after `ARENA_CAMERA_DEFAULT_HZ = 4.0` (line 34):

```python
#: Optional override for the render size, ``WIDTHxHEIGHT``; may only lower
#: either dimension (the bird's-eye view cannot resolve 10 cm objects at
#: the default size anyway, so smaller is never a fidelity loss that
#: matters — and every arena pixel is paid for on the sim's GPU).
ARENA_CAMERA_SIZE_ENV = "TINKER_SIM_ARENA_CAMERA_SIZE"
ARENA_CAMERA_DEFAULT_SIZE = (960, 540)
```

After `resolve_arena_camera` (line 155):

```python
def resolve_arena_camera_size(env: Mapping[str, str]) -> tuple[int, int]:
    """Arena render size ``(width, height)`` from ``env``.

    ``env[ARENA_CAMERA_SIZE_ENV]`` is ``WIDTHxHEIGHT`` (positive integers);
    each dimension is clamped to ``ARENA_CAMERA_DEFAULT_SIZE`` (the
    override may only lower). Unset -> the default. Malformed -> ValueError.
    """
    raw = env.get(ARENA_CAMERA_SIZE_ENV)
    if raw is None:
        return ARENA_CAMERA_DEFAULT_SIZE
    parts = raw.lower().split("x")
    if len(parts) != 2 or not all(p.isdigit() for p in parts):
        raise ValueError(
            f"{ARENA_CAMERA_SIZE_ENV} must be WIDTHxHEIGHT, got {raw!r}"
        )
    width, height = (int(p) for p in parts)
    if width <= 0 or height <= 0:
        raise ValueError(f"{ARENA_CAMERA_SIZE_ENV} dimensions must be positive")
    return (
        min(ARENA_CAMERA_DEFAULT_SIZE[0], width),
        min(ARENA_CAMERA_DEFAULT_SIZE[1], height),
    )
```

Change `arena_camera_spec`'s signature and the two literals:

```python
def arena_camera_spec(
    occupancy: object, *, hz: float, size: tuple[int, int] = ARENA_CAMERA_DEFAULT_SIZE
) -> CameraStreamSpec:
    """The world-fixed, color-only ``CameraStreamSpec`` for the arena camera."""
    eye, target, _bounds = arena_camera_pose(occupancy)
    width, height = size
    return CameraStreamSpec(
        ...  # unchanged fields
        width=width,
        height=height,
        ...
    )
```

- [ ] **Step 4: Run to verify they pass**

Run: `scripts/pytest-clean tests/test_arena_camera.py -q`
Expected: all pass (the existing `test_spec_shape` still sees 960×540 by default).

- [ ] **Step 5: Write the failing tests for the run_sim wiring**

Append to `tests/test_run_sim_arena_wiring.py`:

```python
from run_sim import _stable_aa_requested  # noqa: E402  (add to the import block)


def test_enabled_arena_honours_size_env():
    specs = (_S(30.0),)
    out, _ = _with_arena_camera(
        specs, _Occ,
        {"TINKER_SIM_ARENA_CAMERA": "1", "TINKER_SIM_ARENA_CAMERA_SIZE": "640x360"},
    )
    assert (out[-1].width, out[-1].height) == (640, 360)


def test_stable_aa_follows_arena_unless_forced():
    assert _stable_aa_requested(True, {}) is True
    assert _stable_aa_requested(False, {}) is False
    assert _stable_aa_requested(False, {"TINKER_SIM_STABLE_AA": "1"}) is True
    assert _stable_aa_requested(False, {"TINKER_SIM_STABLE_AA": "0"}) is False
    # the env can force DLAA *off* with the arena on, for the A/B only
    assert _stable_aa_requested(True, {"TINKER_SIM_STABLE_AA": "0"}) is False
```

- [ ] **Step 6: Run to verify they fail**

Run: `scripts/pytest-clean tests/test_run_sim_arena_wiring.py -q`
Expected: `ImportError: cannot import name '_stable_aa_requested'`.

- [ ] **Step 7: Implement the wiring**

In `validation/run_sim.py`, in `_with_arena_camera` (line 286–291):

```python
    from tinker_sim_isaac.arena_camera import (
        arena_camera_spec, resolve_arena_camera, resolve_arena_camera_size,
    )

    hz = resolve_arena_camera(env)
    if hz is None:
        return specs, robot_min_hz
    size = resolve_arena_camera_size(env)
    return specs + (arena_camera_spec(occupancy, hz=hz, size=size),), robot_min_hz
```

After `_arena_camera_enabled` (line 301):

```python
#: A/B aid for the RTF work: "1" forces the DLAA pin on without the arena
#: camera (isolates its per-frame cost on the parity cameras), "0" forces
#: it off with the arena camera on (only for the crash-recipe re-check;
#: see docs/superpowers/specs/2026-08-29-arena-camera-rtf-design.md).
STABLE_AA_ENV = "TINKER_SIM_STABLE_AA"


def _stable_aa_requested(arena_enabled: bool, env: dict) -> bool:
    """DLAA pin decision: the arena camera's presence unless the env forces it."""
    raw = env.get(STABLE_AA_ENV)
    if raw is None:
        return arena_enabled
    return raw.strip().lower() in _TRUTHY
```

Replace line 1059:

```python
            stable_aa = _stable_aa_requested(arena_camera_enabled, os.environ)
            if stable_aa != arena_camera_enabled:
                print(f"[sim] stable_aa forced to {stable_aa} by {STABLE_AA_ENV}", flush=True)
            camera_rig.initialize(app, stable_aa=stable_aa)
```

Also extend the `[sim] arena camera enabled` print (line 1047–1051) to include the size: `f"... at {spec.tick_rate_hz:g} Hz, {spec.width}x{spec.height} -> /sim/arena_camera/image_raw"` where `spec = camera_specs[-1]`.

- [ ] **Step 8: Run all four camera test files**

Run: `scripts/pytest-clean tests/test_arena_camera.py tests/test_run_sim_arena_wiring.py tests/test_camera_rig.py tests/test_camera_rig_worldfixed.py -q`
Expected: all pass.

- [ ] **Step 9: Commit**

```bash
git add simulation/tinker_sim_isaac/arena_camera.py validation/run_sim.py tests/test_arena_camera.py tests/test_run_sim_arena_wiring.py scripts/pytest-clean
git commit -m "feat(sim): opt-in arena camera size and stable-AA overrides for the RTF spike"
```

---

### Task 2: `step_profile` summariser

The spike produces many `{"step_profile": ...}` JSON lines per variant. A small pure module turns a log into per-bucket medians over the steady-state window so the six variants can be compared in one table.

**Files:**
- Create: `tools/step_profile_summary.py`
- Test: `tests/test_step_profile_summary.py`

**Interfaces:**
- Produces: `summarize(lines: Iterable[str], *, skip_first_s: float = 30.0) -> dict` with keys `windows`, `median_ms_per_cycle` (dict of bucket → float for physics/publish/kit_pump/cameras/spin/unaccounted/wall), `rtf_estimate` (`cycle_sim_dt / median wall`, using consecutive `sim_time` deltas), and a `main(argv)` CLI printing one Markdown table row: `| <label> | kit_pump | cameras | physics | wall | rtf |`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_step_profile_summary.py
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from step_profile_summary import summarize  # noqa: E402


def _line(wall_time, sim_time, **ms):
    buckets = {"physics": 10.0, "publish": 1.0, "kit_pump": 30.0, "cameras": 5.0,
               "spin": 1.0, "unaccounted": 3.0, "wall": 50.0}
    buckets.update(ms)
    return json.dumps({"step_profile": {"wall_time": wall_time, "sim_time": sim_time,
                                        "cycles": 10, "ms_per_cycle": buckets}})


def test_summarize_skips_warmup_and_takes_medians():
    lines = [
        "step profiling on: reporting every 10 camera cycles",   # non-JSON noise
        _line(1000.0, 0.0, kit_pump=200.0),                        # warm-up, skipped
        _line(1040.0, 8.0, kit_pump=30.0),
        _line(1041.0, 8.8, kit_pump=32.0),
        _line(1042.0, 9.6, kit_pump=34.0),
    ]
    out = summarize(lines, skip_first_s=30.0)
    assert out["windows"] == 3
    assert out["median_ms_per_cycle"]["kit_pump"] == 32.0
    assert out["median_ms_per_cycle"]["wall"] == 50.0
    # 10 cycles of sim per window: 0.8 s sim per 1.0 s wall between windows
    assert abs(out["rtf_estimate"] - 0.8) < 1e-6


def test_summarize_empty_is_explicit():
    out = summarize([], skip_first_s=0.0)
    assert out["windows"] == 0 and out["rtf_estimate"] is None
```

- [ ] **Step 2: Run to verify it fails**

Run: `scripts/pytest-clean tests/test_step_profile_summary.py -q`
Expected: `ModuleNotFoundError: No module named 'step_profile_summary'`.

- [ ] **Step 3: Implement**

```python
#!/usr/bin/env python3
"""Summarise ``TINKER_SIM_PROFILE=1`` ``step_profile`` lines from a sim log.

Usage: step_profile_summary.py LABEL LOGFILE [--skip-first-s 30]
Prints one Markdown table row (see ``main``). Pure; no Isaac imports.
"""
from __future__ import annotations

import argparse
import json
import statistics
from collections.abc import Iterable

BUCKETS = ("physics", "publish", "kit_pump", "cameras", "spin", "unaccounted", "wall")


def _records(lines: Iterable[str]) -> list[dict]:
    out = []
    for line in lines:
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "step_profile" in rec:
            out.append(rec["step_profile"])
    return out


def summarize(lines: Iterable[str], *, skip_first_s: float = 30.0) -> dict:
    recs = _records(lines)
    if not recs:
        return {"windows": 0, "median_ms_per_cycle": {}, "rtf_estimate": None}
    t0 = recs[0]["wall_time"]
    steady = [r for r in recs if r["wall_time"] - t0 >= skip_first_s]
    medians = {
        b: statistics.median(r["ms_per_cycle"][b] for r in steady) for b in BUCKETS
    } if steady else {}
    rtf = None
    pairs = [
        (b["wall_time"] - a["wall_time"], (b["sim_time"] or 0.0) - (a["sim_time"] or 0.0))
        for a, b in zip(steady, steady[1:])
        if a.get("sim_time") is not None and b.get("sim_time") is not None
    ]
    pairs = [(w, s) for w, s in pairs if w > 0]
    if pairs:
        rtf = statistics.median(s / w for w, s in pairs)
    return {"windows": len(steady), "median_ms_per_cycle": medians, "rtf_estimate": rtf}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("label")
    parser.add_argument("logfile")
    parser.add_argument("--skip-first-s", type=float, default=30.0)
    args = parser.parse_args(argv)
    with open(args.logfile, encoding="utf-8", errors="replace") as fh:
        out = summarize(fh, skip_first_s=args.skip_first_s)
    m = out["median_ms_per_cycle"]
    rtf = "n/a" if out["rtf_estimate"] is None else f"{out['rtf_estimate']:.2f}"
    cell = lambda k: f"{m[k]:.1f}" if k in m else "n/a"  # noqa: E731
    print(
        f"| {args.label} | {out['windows']} | {cell('kit_pump')} | {cell('cameras')} | "
        f"{cell('physics')} | {cell('wall')} | {rtf} |"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Run to verify it passes**

Run: `scripts/pytest-clean tests/test_step_profile_summary.py -q`
Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git add tools/step_profile_summary.py tests/test_step_profile_summary.py
git commit -m "feat(tools): step_profile summariser for RTF spikes"
```

---

### Task 3: Run the Phase 0 spike and decide

Throwaway measurement. Sim alone, no bridge, GPU 1. Record the table in the developer log and pick Task 4a or 4b.

**Files:**
- Create: `scripts/arena-rtf-spike` (bash; kept as the reproducible harness)
- Modify: `docs/developer-log.md` (new dated entry at the top of the 2026-08-29 section, create it if absent)

**Interfaces:**
- Consumes: `TINKER_SIM_STABLE_AA`, `TINKER_SIM_ARENA_CAMERA_SIZE` (Task 1), `tools/step_profile_summary.py` (Task 2).
- Produces: the decision `PHASE1=4a|4b|resolution|publish` recorded in the developer log.

- [ ] **Step 1: Confirm GPU 1 is free enough**

Run: `nvidia-smi --query-gpu=index,memory.used,memory.total,utilization.gpu --format=csv`
Expected: GPU 1 has ≥5 GB free and no other `run_sim.py` on it (`pgrep -af run_sim.py`). If the bench session's stack is up on GPU 1, STOP and coordinate (message that session) — do not run the spike on a card with another sim.

- [ ] **Step 2: Write the harness**

```bash
#!/usr/bin/env bash
# Phase 0 RTF spike: six sim-only variants, 120 s sim each, profile on.
# Usage: scripts/arena-rtf-spike [OUTDIR]   (default: outputs/rtf-spike-<date>)
# Prints a Markdown table; raw logs stay in OUTDIR.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/outputs/rtf-spike-$(date +%Y%m%d-%H%M)}"
mkdir -p "$OUT"
cd "$ROOT"
set -a; source .deployment.env; set +a
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH
export ROS_DOMAIN_ID=42 CUDA_VISIBLE_DEVICES="${SIM_GPU:-1}"
export TINKER_SIM_PROFILE=1 TINKER_SIM_PROFILE_EVERY=10
export TINKER_SIM_CAMERA_HZ=12 TINKER_SIM_CONTROL_HZ=60 TINKER_SIM_HEAD_CAMERA_AIM=level-forward

run_variant() {  # label, then env assignments
  local label="$1"; shift
  echo "== $label ==" >&2
  env "$@" ./scripts/tinker-sim launch \
    --sensor-profile sensor-rich --scenario gpsr-rcw2026-bench --seed 0 \
    --arena rcw2026 --spawn-xy=-2.0,-2.0 --ros --headless --duration 120 \
    > "$OUT/$label.log" 2>&1 || echo "variant $label exited $?" >&2
  sleep 5   # let the GPU context drain before the next variant
}

run_variant A_arena_off       TINKER_SIM_ARENA_CAMERA=0
run_variant B_arena_2hz       TINKER_SIM_ARENA_CAMERA=1 TINKER_SIM_ARENA_CAMERA_HZ=2
run_variant C_capture_skip    TINKER_SIM_ARENA_CAMERA=1 TINKER_SIM_ARENA_CAMERA_HZ=2 TINKER_SIM_CAPTURE_SKIP=arena_camera
run_variant D_dlaa_only       TINKER_SIM_ARENA_CAMERA=0 TINKER_SIM_STABLE_AA=1
run_variant E_arena_640       TINKER_SIM_ARENA_CAMERA=1 TINKER_SIM_ARENA_CAMERA_HZ=2 TINKER_SIM_ARENA_CAMERA_SIZE=640x360
run_variant F_arena_0p5hz     TINKER_SIM_ARENA_CAMERA=1 TINKER_SIM_ARENA_CAMERA_HZ=0.5

echo "| variant | windows | kit_pump ms | cameras ms | physics ms | wall ms | RTF |"
echo "|---|---|---|---|---|---|---|"
for f in "$OUT"/*.log; do
  .venv/bin/python tools/step_profile_summary.py "$(basename "$f" .log)" "$f" --skip-first-s 30
done
```

`chmod +x scripts/arena-rtf-spike`. Note `--duration 120` is simulation seconds: at RTF 0.25 a variant takes ~8 min wall; budget ~40 min total.

- [ ] **Step 3: Run it**

Run: `scripts/arena-rtf-spike 2>&1 | tee outputs/rtf-spike-latest.txt`
Expected: six log files, each with ≥20 `step_profile` lines after the 30 s warm-up, and a seven-row table. Check every log's tail for `error 700` / `cudaErrorMemoryAllocation`; a crashed variant is a finding (record it), not a reason to retry silently.

- [ ] **Step 4: Apply the decision rule and write it up**

Add to `docs/developer-log.md` an entry `## 2026-08-29 — Arena camera RTF: Phase 0 measurement` containing the table verbatim, the GPU/host conditions (`nvidia-smi` line, load average), and one paragraph applying the spec's rule:

- `D − A` ≥ 60 % of `B − A` (in `kit_pump` ms) → **Task 4a** (DLAA is the tax).
- `F ≈ B` (within 15 %) and `D` small → **Task 4b** (fixed per-product cost).
- `E` recovers ≥ 60 % of `B − A` → resolution is the lever: Task 5 only, then Task 6/7.
- `C ≈ A` → the cost is in `publish_cameras`, not the render: STOP, report; the spec's Phase 1 does not apply and a new bounded task (arena publish path) replaces it.

Record the decision as a literal line: `Decision: Task 4a` (or 4b / 5-only / stop).

- [ ] **Step 5: Commit**

```bash
git add scripts/arena-rtf-spike docs/developer-log.md
git commit -m "spike: arena camera RTF phase-0 measurements and decision"
```

---

### Task 4a: Scope the DLAA pin (only if Task 3 decided 4a)

Two sub-approaches, tried in order; stop at the first that passes the crash recipe.

**Files:**
- Modify: `simulation/tinker_sim_isaac/camera_rig.py:501-537` (`initialize`, `stable_aa` block)
- Modify: `validation/run_sim.py` (`_stable_aa_requested` semantics unchanged; only the log line)
- Test: `tests/test_camera_rig_worldfixed.py`

**Interfaces:**
- Consumes: `CameraRig.initialize(app, *, stable_aa: bool)`.
- Produces: `CameraRig.initialize(app, *, stable_aa: bool, stable_aa_cameras: frozenset[str] | None = None)` — when `stable_aa_cameras` is given, DLAA is applied per render product for those names only; `None` keeps today's global behaviour.

- [ ] **Step 1: Probe whether AA is per-render-product (5-minute throwaway, sim GPU 1)**

Run a sensor-rich sim with `TINKER_SIM_ARENA_CAMERA=1 TINKER_SIM_CAMERA_DEBUG=1`, and from a second terminal is not possible (headless) — instead add a temporary print in `initialize` after the sensors are created:

```python
            # TEMP PROBE (remove before commit)
            rp = self._sensors[spec.name].render_product
            print("[probe]", spec.name, rp.GetPath(), [a.GetName() for a in rp.GetPrim().GetAttributes() if "aa" in a.GetName().lower() or "dlss" in a.GetName().lower()], flush=True)
```

Expected: either the `UsdRender.Product` prim lists an AA-related attribute (e.g. `rtx:post:aa:op` authored per product — then approach (i) is possible) or it lists none (global-only → approach (ii)). Record the printed lines in the developer-log entry. Remove the probe.

- [ ] **Step 2 (approach i, per-product attribute exists): write the failing test**

```python
# tests/test_camera_rig_worldfixed.py
def test_initialize_accepts_stable_aa_cameras_keyword():
    import inspect
    from tinker_sim_isaac.camera_rig import CameraRig
    params = inspect.signature(CameraRig.initialize).parameters
    assert "stable_aa_cameras" in params
    assert params["stable_aa_cameras"].default is None
    assert params["stable_aa_cameras"].kind is inspect.Parameter.KEYWORD_ONLY
```

Run: `scripts/pytest-clean tests/test_camera_rig_worldfixed.py -q` — expected: 1 failed (`KeyError: 'stable_aa_cameras'`).

- [ ] **Step 3 (approach i): implement**

In `initialize`: add `stable_aa_cameras: frozenset[str] | None = None` after `stable_aa`; keep the global `set_int` only when `stable_aa and stable_aa_cameras is None`; after each `CameraSensor` is created, when `stable_aa and stable_aa_cameras is not None and spec.name in stable_aa_cameras`, author the attribute found in Step 1 on `self._sensors[spec.name].render_product.GetPrim()` with value `AA_OP_DLAA`. In `run_sim.py:1059` pass `stable_aa_cameras=frozenset({"arena_camera"}) if stable_aa else None`.

Run the test file — expected pass. Then run the crash recipe (Step 5).

- [ ] **Step 4 (approach ii, AA is global-only): re-test the crash recipe with DLAA off**

No code change yet. Run the full stack with DLAA forced off:

```bash
# main checkout, GPU 1 free, bench session coordinated
TINKER_SIM_STABLE_AA=0 ./scripts/gpsr-stack up --scenario gpsr-rcw2026-bench --sim-gpu 1
```

Note: `gpsr-stack` builds the sim env from `_preamble_env`, which is merged over the real environment (`_full_env`), so the exported variable reaches the sim. Watch `gpsr_stack_logs/<run>/sim.log` for 5 minutes for `error 700` / `illegal memory access`. Repeat 3×. Tear down with `./scripts/gpsr-stack down`.

- Clean 3/3 → the a9fa951 fresh-buffer gate made DLAA redundant: change `_stable_aa_requested`'s default to `False` (return `False` when the env is unset), update `test_stable_aa_follows_arena_unless_forced` accordingly (`assert _stable_aa_requested(True, {}) is False`), and rewrite the `run_sim.py:1053-1058` comment to say DLAA is now opt-in via `TINKER_SIM_STABLE_AA=1` and why.
- Any crash → approach (ii) fails; DLAA stays; go to Task 4b as the structural fix and record that in the developer log.

- [ ] **Step 5: Crash recipe for whichever approach was implemented**

Same `gpsr-stack up` recipe as Step 4 (without the env override for approach i), 3 runs × 5 min, zero `error 700`. Then a sim-only RTF re-measure: `scripts/arena-rtf-spike` variants A and B only (comment the others out temporarily, or run the two `run_variant` lines by hand) — expected `B − A` in `kit_pump` ms to shrink by the amount `D − A` measured in Task 3.

- [ ] **Step 6: Commit**

```bash
git add simulation/tinker_sim_isaac/camera_rig.py validation/run_sim.py tests/test_camera_rig_worldfixed.py tests/test_run_sim_arena_wiring.py docs/developer-log.md
git commit -m "perf(sim): scope the DLAA pin so the arena camera stops taxing the parity cameras"
```

---

### Task 4b: Render the arena product only on its own stride (only if Task 3 decided 4b, or 4a failed)

Keep the arena `CameraSensor`'s render product alive only around the Kit pumps that fall on the arena stride.

**Files:**
- Modify: `simulation/tinker_sim_isaac/camera_rig.py` (new `set_product_active`, `product_active`)
- Modify: `validation/run_sim.py:1169-1186` (arena stride around the pump)
- Modify: `simulation/tinker_sim_isaac/ros_gateway.py:1195-1264` (`publish_cameras` skips inactive cameras)
- Test: `tests/test_camera_rig.py`, `tests/test_run_sim_arena_wiring.py`

**Interfaces:**
- Produces: `CameraRig.set_product_active(name: str, active: bool) -> None` (idempotent; inactive = annotators detached and hydra texture destroyed via the sensor's `_invalidate_sensor()`, active = `_initialize_sensor(annotators)` re-run and the `(name, kind)` entries dropped from `_consumed_ptrs`), `CameraRig.product_active(name) -> bool`; `run_sim._arena_stride(camera_stride: int, camera_hz: float, arena_hz: float) -> int` (pure: number of camera cycles between arena renders, `max(1, round(camera_hz / arena_hz))`).
- Note: `set_updates_enabled` was not found on this Isaac build's replicator (`grep -rl set_updates_enabled extscache/` is empty), so destroy/recreate is the mechanism. Cost of a recreate must be measured (Step 5) — if it exceeds the saving, this task is abandoned and the log says so.

- [ ] **Step 1: Write the failing pure test for the stride**

```python
# tests/test_run_sim_arena_wiring.py
from run_sim import _arena_stride  # noqa: E402


def test_arena_stride_in_camera_cycles():
    assert _arena_stride(5, 12.0, 2.0) == 6
    assert _arena_stride(5, 12.0, 4.0) == 3
    assert _arena_stride(5, 12.0, 12.0) == 1
    assert _arena_stride(5, 12.0, 0.5) == 24
```

Run: `scripts/pytest-clean tests/test_run_sim_arena_wiring.py -q` — expected `ImportError`.

- [ ] **Step 2: Implement `_arena_stride`**

After `_arena_camera_enabled` in `run_sim.py`:

```python
def _arena_stride(camera_stride: int, camera_hz: float, arena_hz: float) -> int:
    """Camera cycles between arena renders (``camera_stride`` is physics
    frames per camera cycle and is unused here; kept so callers pass the
    same triple they log)."""
    if camera_hz <= 0.0 or arena_hz <= 0.0:
        raise ValueError("camera_hz and arena_hz must be positive")
    return max(1, round(camera_hz / arena_hz))
```

Run the test — expected pass.

- [ ] **Step 3: Write the failing rig test**

```python
# tests/test_camera_rig.py
def test_set_product_active_is_tracked_per_camera():
    import inspect
    from tinker_sim_isaac.camera_rig import CameraRig
    assert "set_product_active" in dir(CameraRig)
    sig = inspect.signature(CameraRig.set_product_active)
    assert list(sig.parameters)[1:] == ["name", "active"]
```

Run — expected fail (`AssertionError`).

- [ ] **Step 4: Implement on `CameraRig`**

In `__init__` add `self._inactive: set[str] = set()`. Add:

```python
    def product_active(self, name: str) -> bool:
        return name not in self._inactive

    def set_product_active(self, name: str, active: bool) -> None:
        """Destroy (``active=False``) or recreate (``True``) one camera's RTX
        render product. Inactive products cost Kit nothing per pump; the
        arena camera is only active around the pumps on its own stride.
        Idempotent. Drops the camera's consumed-pointer records on
        recreate so the next capture() treats its buffers as fresh."""
        sensor = self._sensors[name]
        if active and name in self._inactive:
            annotators = [COLOR_ANNOTATOR] if name in self._color_only else [COLOR_ANNOTATOR, DEPTH_ANNOTATOR]
            sensor._initialize_sensor(annotators)
            self._consumed_ptrs.pop((name, "rgb"), None)
            self._consumed_ptrs.pop((name, "depth"), None)
            self._inactive.discard(name)
        elif not active and name not in self._inactive:
            sensor._invalidate_sensor()
            self._inactive.add(name)
```

In `capture()` (line ~715, before the `TINKER_SIM_CAPTURE_SKIP` check): `if name in self._inactive: results[name] = (None, None); continue` using whatever the skip path already returns for an unconsumed camera. In `ros_gateway.publish_cameras` the `(None, None)` result already means "nothing to publish" for the skip path — verify by reading lines 1215–1230 and reuse that branch.

- [ ] **Step 5: Wire the stride into the loop and measure**

In `run_sim.py` before the loop: `arena_stride = _arena_stride(camera_stride, camera_hz, camera_specs[-1].tick_rate_hz) if arena_camera_enabled else 0`, `camera_cycle = 0`. Inside `if camera_frame_index % camera_stride == 0:`:

```python
                            camera_cycle += 1
                            arena_now = arena_camera_enabled and camera_cycle % arena_stride == 0
                            if arena_camera_enabled:
                                camera_rig.set_product_active("arena_camera", arena_now)
                            _pump_streaming_app_update(app, kit_settings)
                            gateway.publish_cameras()
                            if arena_camera_enabled and arena_now:
                                camera_rig.set_product_active("arena_camera", False)
```

Then run `scripts/arena-rtf-spike` variants A and B. Expected: `B.kit_pump` within 10 % of `A.kit_pump` on the 5-of-6 non-arena cycles (the median will show it). If the recreate cost makes the arena cycle stall > 200 ms, or `error 700` appears, revert this task's loop change and record the numbers — the destroy/recreate mechanism is then ruled out, and Task 5's resolution drop becomes the fix.

- [ ] **Step 6: Crash recipe**

Full `gpsr-stack up --scenario gpsr-rcw2026-bench --sim-gpu 1`, 3 runs × 5 min, zero `error 700`, arena frames still arriving on `/sim/arena_camera/image_raw` at ~2 Hz (`ros2 topic hz --no-daemon /sim/arena_camera/image_raw`).

- [ ] **Step 7: Commit**

```bash
git add simulation/tinker_sim_isaac/camera_rig.py simulation/tinker_sim_isaac/ros_gateway.py validation/run_sim.py tests/test_camera_rig.py tests/test_run_sim_arena_wiring.py docs/developer-log.md
git commit -m "perf(sim): keep the arena render product alive only on its own stride"
```

---

### Task 5: Cheap wins — arena default 640×360 and 2 Hz

Independent of Task 4; do it even if Task 4 landed.

**Files:**
- Modify: `simulation/tinker_sim_isaac/arena_camera.py:34` (`ARENA_CAMERA_DEFAULT_HZ`), the new `ARENA_CAMERA_DEFAULT_SIZE`
- Modify: `tools/contact_sheet.py` (only if it assumes the 960 width — `grep -n "960" tools/contact_sheet.py`)
- Test: `tests/test_arena_camera.py`, `tests/test_contact_sheet.py`

- [ ] **Step 1: Update the tests first**

In `tests/test_arena_camera.py`: `test_resolve_enabled_and_rate_only_lowers` — `ARENA_CAMERA_DEFAULT_HZ` stays symbolic; add `assert ARENA_CAMERA_DEFAULT_HZ == 2.0`; the `"30"` override still clamps to the default. `test_spec_shape`: `assert (spec.width, spec.height) == (640, 360)`. `test_resolve_size_default_and_only_lowers`: default `== (640, 360)`; `"960x540"` now clamps to `(640, 360)`.

Run: `scripts/pytest-clean tests/test_arena_camera.py -q` — expected: 3 failed.

- [ ] **Step 2: Change the two defaults**

`ARENA_CAMERA_DEFAULT_HZ = 2.0`, `ARENA_CAMERA_DEFAULT_SIZE = (640, 360)`; update the comment above `ARENA_CAMERA_HZ_ENV` to cite the 2026-08-29 measurement.

Run the arena tests — expected pass. Run `scripts/pytest-clean tests/test_contact_sheet.py -q` — if a width assumption fails, make the sheet read the frame's actual width (the message's `width` field) instead of a literal.

- [ ] **Step 3: Visual check of the smaller frame**

Sim-only on GPU 1 with `TINKER_SIM_ARENA_CAMERA=1`, grab one frame: `ros2 topic echo --no-daemon --once /sim/arena_camera/image_raw --field width` must print `640`; save a frame with the bench session's existing grab path (`tk25_decision bench/debug` tooling) or `ros2 run image_view image_saver` and confirm a person and a table are recognisable. Attach the path in the developer log.

- [ ] **Step 4: Commit**

```bash
git add simulation/tinker_sim_isaac/arena_camera.py tests/test_arena_camera.py tools/contact_sheet.py tests/test_contact_sheet.py docs/developer-log.md
git commit -m "perf(sim): arena camera defaults to 640x360 at 2 Hz"
```

---

### Task 6: Bench policy — arena camera only on evidence runs

`gpsr-stack` gets an `--evidence` flag; without it the arena env pair is not set. This also makes the two stale dry-run tests true again.

**Files:**
- Modify: `scripts/gpsr-stack:211-219` (`StackConfig`), `:290-300` (arena env pair), `:730-755` (parser + `main`)
- Test: `tests/test_gpsr_stack_dryrun.py:41-65`

**Interfaces:**
- Produces: `StackConfig.evidence: bool = False`; `gpsr-stack up --evidence`.

- [ ] **Step 1: Write the failing tests**

Replace `test_sim_stage_no_camera_env_vars` (lines 50–65) with:

```python
# --- Arena camera is an evidence-run opt-in (RTF: 0.24 with it, 0.68+ without) ---

def test_sim_stage_arena_off_by_default():
    for cfg in (_cfg(manipulation="mock"), _cfg(manipulation="live", manip_gpu=1)):
        env = mod.stage_commands(cfg)[0]["env"]
        assert "TINKER_SIM_ARENA_CAMERA" not in env
        assert "TINKER_SIM_ARENA_CAMERA_HZ" not in env
        assert "TINKER_SIM_DISABLE_WRIST_CAMERA" not in env


def test_sim_stage_arena_on_for_evidence_runs():
    env = mod.stage_commands(_cfg(evidence=True))[0]["env"]
    assert env["TINKER_SIM_ARENA_CAMERA"] == "1"
    assert env["TINKER_SIM_ARENA_CAMERA_HZ"] == "2"


def test_cli_evidence_flag_reaches_config(monkeypatch):
    seen = {}
    monkeypatch.setattr(mod, "_run_up", lambda cfg, dry_run: seen.update(cfg=cfg) or 0)
    assert mod.main(["up", "--evidence", "--dry-run"]) == 0
    assert seen["cfg"].evidence is True
    assert mod.main(["up", "--dry-run"]) == 0
    assert seen["cfg"].evidence is False
```

`_cfg` must accept `evidence` — it forwards `**kw` to `StackConfig`, so no change there. Run: `scripts/pytest-clean tests/test_gpsr_stack_dryrun.py -q -k "arena or evidence or sim_stage_env"` — expected: the new tests fail (`TypeError: unexpected keyword 'evidence'`), `test_sim_stage_env` still fails.

- [ ] **Step 2: Implement**

`StackConfig`: add `evidence: bool = False` as the last field. In `stage_commands`, replace the two literal lines with a conditional after the dict is built:

```python
    sim_env = {
        "CUDA_VISIBLE_DEVICES": str(cfg.sim_gpu),
        ...  # existing entries, minus the two TINKER_SIM_ARENA_CAMERA* lines
    }
    if cfg.evidence:
        # Arena observer only for evidence runs: it costs ~0.4 RTF even at
        # 2 Hz (docs/developer-log.md 2026-08-29), so pass/fail batteries
        # run without it and keep their 600 s wall cap.
        sim_env["TINKER_SIM_ARENA_CAMERA"] = "1"
        sim_env["TINKER_SIM_ARENA_CAMERA_HZ"] = "2"
    stages.append({..., "env": _preamble_env(sim_env), ...})
```

Rewrite the old "Arena observer re-enabled for the recorded battery" comment to the new one above. Parser: `parser.add_argument("--evidence", action="store_true", help="enable the arena observer camera (contact-sheet evidence runs; ~0.4 RTF cost)")`; `main`: `evidence=args.evidence`.

Run: `scripts/pytest-clean tests/test_gpsr_stack_dryrun.py -q` — expected: only the 8 `model bundle not found` environmental failures remain (0 arena-related).

- [ ] **Step 3: Runbook knob line**

In `docs/gpsr-sim-runbook.md`, in the gpsr-stack usage block near line 26, add: `--evidence   # arena observer camera on (contact sheets); costs RTF, off for batteries`. No narrative.

- [ ] **Step 4: Commit**

```bash
git add scripts/gpsr-stack tests/test_gpsr_stack_dryrun.py docs/gpsr-sim-runbook.md
git commit -m "feat(gpsr-stack): --evidence opt-in for the arena camera; batteries run without it"
```

---

### Task 7: Verify on the real stack and write up

**Files:**
- Modify: `docs/developer-log.md` (extend the 2026-08-29 entry), `docs/gpsr-sim-runbook.md` (RTF table row only)
- Modify: `tests/test_run_sim_arena_wiring.py` (regression guard for the Task 4 choice)

- [ ] **Step 1: Regression guard**

If Task 4a landed with approach (ii): `test_stable_aa_follows_arena_unless_forced` already pins `_stable_aa_requested(True, {}) is False`. If approach (i): add `test_stable_aa_cameras_is_arena_only` asserting the frozenset passed is `{"arena_camera"}` by exposing it as a module constant `STABLE_AA_CAMERAS = frozenset({"arena_camera"})` in `run_sim.py` and asserting on it. If Task 4b: `test_arena_stride_in_camera_cycles` is the guard. Run the four camera test files — all pass.

- [ ] **Step 2: Before/after on the bench stack (coordinate GPU 1 with the bench session)**

From the main checkout after merging this branch (or from this worktree with `TINKER_SIM_ROOT` pointing at it — the bridge stage exports it, commit 1148353):

```bash
./scripts/gpsr-stack up --scenario gpsr-rcw2026-bench --sim-gpu 1 --evidence
# wait for all gates, then 3 samples 2 min apart:
timeout 20 ros2 topic echo --no-daemon /clock --field clock | grep sec | sed -n '1p;$p'; date +%s.%N
./scripts/gpsr-stack down
```

RTF = (last sec − first sec) / wall elapsed. Expected: ≥0.6 with `--evidence` (was 0.21–0.24), and the non-evidence stack unchanged at ~0.7–0.8. Also `nvidia-smi` utilisation on GPU 1 during the evidence run (was 98 %).

- [ ] **Step 3: Write-up**

Extend the developer-log entry with: the before/after table, which of 4a/4b was taken and why the other was not, the crash-recipe result (runs × minutes, zero 700), the one-line policy (`--evidence`), and a "follow-up, not done" line for the spec's Phase 2 item 3 (gating the head camera's 12 Hz on having a subscriber) with the Phase 0 `A` row as the reason it was or was not worth it. Update the runbook RTF table (line ~210) with one new row `arena camera on (--evidence)` giving the measured figure. Nothing else in the runbook.

- [ ] **Step 4: Commit and hand back**

```bash
git add docs/developer-log.md docs/gpsr-sim-runbook.md tests/test_run_sim_arena_wiring.py
git commit -m "docs: arena camera RTF — measurements, fix, and the --evidence policy"
```

Report the branch (`worktree-rtf-arena-camera`) and the merge command for the user to run themselves (or delegate): `git -C /home/tinker/tinker-sim/6.0.1 merge --ff-only worktree-rtf-arena-camera` after confirming the main checkout's dirty bridge files are untouched by this branch (`git diff --stat task50-stage-a-repair...worktree-rtf-arena-camera -- ros2_ws/` must be empty).
