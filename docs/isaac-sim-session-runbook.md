# Isaac Sim session start runbook (tinker-sim 6.0.1)

How to start Isaac Sim sessions on this machine — reliably, and without tripping the
aborts that have bitten us before. Grounded in the actual launch code
(`tools/tinker_sim_deploy/cli.py`, `validation/run_sim.py`, `tools/tinker_sim_deploy/ros_boundary.py`)
and the two 2-GPU RTX 2080 Ti setup on this host.

> **The one thing to internalize:** Isaac Sim has **no** built-in "another instance is
> running" lock. Every abort we've seen with that flavor came from *our own* guards — a
> stale PID lock or the manual "GPU already occupied" runbook rule — not from Isaac. So the
> fix is always preflight + (for concurrency) per-instance parameterization, never "Isaac
> won't allow it."

---

## 0. Decision: which path do you want?

| You want… | Path | Status today |
|---|---|---|
| One normal streamed session (view in browser) | **§2 Standard streaming session** | ✅ Works out of the box |
| One headless validation/smoke run | **§2 Headless variant** | ✅ Works out of the box |
| Recover after an abort / crash | **§3 Recovery** | ✅ Just cleanup |
| Two sessions at once, both headless (2 dev loops) | **§4a** | ✅ Works today with 2 env exports |
| Two sessions at once, at least one streamed | **§4b** | ⚠️ Needs the small code patch in §4b |

---

## 1. Preflight (ALWAYS run these first)

Two checks, every time. Skipping them is what caused the past aborts.

```bash
# (a) GPU must be free of stray Isaac/cuMotion processes.
#     This is the check docs/gpsr-sim-runbook.md:19 requires — it must print NOTHING
#     (for a single-session launch). If it lists PIDs, see §3.
nvidia-smi --query-compute-apps=pid,used_memory,process_name --format=csv,noheader

# (b) No stale streaming-viewer lock. If this prints a path, a streamed session thinks
#     it's still running. The code self-heals a DEAD pid, but check anyway (see §3).
ls -l .cache/streaming-viewer.lock 2>/dev/null || echo "no lock (good)"
```

Why both: a run can hold memory on **both** GPUs even though physics is CPU-only
(a recorded block on 2026-08-19 showed 2276 MiB on GPU 0 *and* 1032 MiB on GPU 1 from one
`run_sim.py` + cuMotion). So "GPU 1 looks idle" is not a safe assumption — check the actual
process list, not just `nvidia-smi` memory.

---

## 2. Standard single session (the safe default)

### Streaming session (interactive, view in browser via WebRTC)

```bash
cd /home/tinker/tinker-sim/6.0.1
./scripts/launch-arena-streaming        # arena scene, sources .deployment.env, --livestream
# or ./scripts/launch-streaming         # LAN DDS profile variant
```

What this does under the hood:
- Sources `.deployment.env` → `ROS_DOMAIN_ID=25`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
  `TINKER_ACCEPT_OMNIVERSE_EULA=Y`, `TINKER_WS=/home/tinker/tk25_ws`.
- Takes the streaming-viewer PID lock at `.cache/streaming-viewer.lock`.
- Binds WebRTC **signal 49100 / media 47998** on `127.0.0.1` (loopback only).
- DDS profile: `local` (shared-memory Fast DDS) for arena-streaming, `lan`
  (`config/fastdds-lan.xml`, UDP-only) for `launch-streaming`. Override with
  `TINKER_SIM_DDS_PROFILE=local|lan`.

Then connect the viewer (separate terminal):

```bash
./scripts/connect-arena-streaming       # tunnels signal 49100 / media 47998
```

### Headless variant (validation / smoke, no viewer)

```bash
cd /home/tinker/tinker-sim/6.0.1
uv run --frozen --no-sync python validation/run_sim.py \
    --sensor-profile navigation-parity --seed 0
```

Headless runs **do not** take the streaming-viewer lock and **do not** bind the WebRTC
ports — so they're immune to the two most common concurrency collisions. This is the path
to prefer when you just need the sim running for a test.

### Physics note (applies to every path)

Physics is **hard-pinned to CPU** by design (`backend.py` sets `device="cpu"`; every launch
passes `--/physics/useGpu=false --/physics/cudaDevice=-1`, and the backend *raises* if Isaac
picks GPU physics). The GPUs do rendering only. Consequence: VRAM is not the constraint
(11 GB/instance is ample) — **CPU is**. Two concurrent sessions double CPU physics load.

---

## 3. Recovery (after an abort, crash, or "already active")

```bash
# A) "one streaming viewer is already active (simulator pid N)"
#    -> our PID lock. If pid N is dead the code auto-clears it; if you're sure nothing is
#       running, remove it:
cat .cache/streaming-viewer.lock          # see which pid it claims
ps -p "$(cat .cache/streaming-viewer.lock)" 2>/dev/null || rm -f .cache/streaming-viewer.lock

# B) GPU still occupied by a stray process (preflight (a) listed PIDs)
nvidia-smi --query-compute-apps=pid,process_name --format=csv,noheader
kill <pid>                                # or kill -9 if it won't die; then re-check

# C) Kit died mid-shutdown (the "Destroying busy TaskGroup / Press ABORT" assertion we've
#    logged). Harmless race on exit — just relaunch after confirming (a) and (b) are clean.
```

The OV/uv cache locks you may see (`.cache/isaac/6.0.1/ov/_cache.lock`,
`~/.local/share/ov/.../registry.lock`, `.cache/uv/**/.lock`) are **registry/cache write
locks, not startup gates** — do not delete them to "fix" a launch; they are not the cause.

---

## 4. Two concurrent sessions

Physically supported: 2× RTX 2080 Ti. Isaac 6.0 is the first version where concurrent
streaming is even possible (5.x had fixed internal ports). But nothing in the repo
parameterizes per-instance settings yet, so follow the right sub-path.

### 4a. Two HEADLESS sessions (works today)

Only two things must differ per instance: the **ROS domain** and the **GPU**. Both are just
env exports — no code change.

```bash
# --- Terminal 1 : instance A on GPU 0, ROS domain 25 ---
cd /home/tinker/tinker-sim/6.0.1
CUDA_VISIBLE_DEVICES=0 ROS_DOMAIN_ID=25 \
  uv run --frozen --no-sync python validation/run_sim.py --sensor-profile navigation-parity --seed 0

# --- Terminal 2 : instance B on GPU 1, ROS domain 26 ---
cd /home/tinker/tinker-sim/6.0.1
CUDA_VISIBLE_DEVICES=1 ROS_DOMAIN_ID=26 \
  uv run --frozen --no-sync python validation/run_sim.py --sensor-profile navigation-parity --seed 1
```

- `CUDA_VISIBLE_DEVICES` sandboxes each process to one GPU, so the renderer (and any cuMotion
  spill) can only touch that card — this is what prevents the "run bled onto GPU 1" problem.
  Prefer it over `--/renderer/activeGpu` because it constrains *every* CUDA subsystem, not
  just the renderer.
- `ROS_DOMAIN_ID` is respected by both the Isaac side (`ros_boundary.py:85` fallback) and the
  Nav2/Humble side (`launch-humble:22`). **Different IDs are mandatory** — everything defaults
  to 25, so without the override the two stacks share a domain and cross-talk.
- Nav2 side for each: launch its Humble stack in a matching-domain shell, e.g.
  `ROS_DOMAIN_ID=25 ./scripts/launch-humble …` for A, `26` for B.

That's it for headless — no port/lock conflicts because headless takes neither.

### 4b. Two sessions with streaming (needs a small patch first)

Streaming adds three shared resources that collide. Until these are parameterized, a second
*streamed* instance will abort or fail to bind. The change is small and localized:

| Collision | Where it's hardcoded | What instance B needs |
|---|---|---|
| WebRTC signal/media ports | `validation/run_sim.py:15-16` (`49100`/`47998`) | e.g. `49200`/`48100` |
| Single-viewer PID lock | `cli.py:126-140`, gated at `cli.py:212-213` | per-instance lock filename |
| Stream ready-file | `/tmp/tinker-sim-arena-streaming.ready.json` | already overridable → `TINKER_SIM_STREAM_READY_FILE` |
| Kit / OV shader cache | `.cache/isaac/6.0.1` via `process.py:55-65` | per-instance cache root (avoid shader-cache contention) |

Recommended implementation (one contained change): introduce `TINKER_SIM_INSTANCE=0|1` and
derive everything from it —
- `CUDA_VISIBLE_DEVICES = n`
- `ROS_DOMAIN_ID = 25 + n`
- signal port `= 49100 + 100*n`, media port `= 47998 + 102*n`
- lock `= .cache/streaming-viewer.<n>.lock`
- `TINKER_SIM_STREAM_READY_FILE = /tmp/tinker-sim-arena-streaming.<n>.ready.json`
- cache root `= .cache/isaac/6.0.1-<n>`

Injection points: the `env` dict in `cli.py::_launch` (~line 206) for the env-derived values,
the port constants + `_streaming_lock()` path in `cli.py`/`run_sim.py` for the rest. The
first streamed launch on the new cache root pays a one-time shader-recompile cost.

Until that lands: run **at most one streamed** session; make the second session **headless**
(§4a). That combination works today with zero code changes.

---

## 5. Quick reference

### Env vars that matter

| Var | Default | Effect |
|---|---|---|
| `ROS_DOMAIN_ID` | `25` | DDS domain; must differ per concurrent instance |
| `CUDA_VISIBLE_DEVICES` | *(unset → GPU 0)* | Pin instance to a GPU; **not set by repo today** |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | DDS impl |
| `TINKER_SIM_DDS_PROFILE` | `local` (arena) / `lan` (streaming) | `local`=shared-mem, `lan`=`config/fastdds-lan.xml` |
| `TINKER_SIM_STREAM_READY_FILE` | `/tmp/tinker-sim-arena-streaming.ready.json` | Per-instance readiness handshake file |
| `TINKER_ACCEPT_OMNIVERSE_EULA` | `Y` (in `.deployment.env`) | EULA gate |
| `TINKER_WS` | `/home/tinker/tk25_ws` | External Humble/Nav2 workspace |

### Ports & files

- WebRTC signal **49100/TCP**, media **47998/UDP** — `127.0.0.1` (hardcoded in `run_sim.py:15-16`).
- Streaming-viewer lock: `.cache/streaming-viewer.lock` (PID file, streaming/`--livestream` only).
- Shared Kit/OV cache: `.cache/isaac/6.0.1` (`XDG_CACHE_HOME`/`OV_CACHE_ROOT`/`ISAACSIM_CACHE_PATH`).

### Launch chain (for debugging)

`scripts/launch-{arena-,}streaming` → `scripts/launch-isaac` → `scripts/tinker-sim` →
`tools/deploy.py` → `tools/tinker_sim_deploy/cli.py::_launch` →
(`isaacsim …streaming` **or** `python validation/run_sim.py …`) → `SimulationApp(...)` at
`run_sim.py:585`.

---

## 6. Golden rules

1. **Always preflight** (§1). Most past aborts were a stale lock or an occupied GPU, not Isaac.
2. **Pin the GPU** for any concurrent launch — runs can spill onto the "idle" card.
3. **Distinct `ROS_DOMAIN_ID`** per concurrent instance, or they cross-talk on domain 25.
4. **One streamed session at a time** until §4b lands; make additional sessions headless.
5. A cache/registry `.lock` is **not** a startup gate — don't delete it to fix a launch.
