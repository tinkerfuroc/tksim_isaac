# Map Check

Map Check turns a Nav2 2D map into a 3D sim arena, then renders Tinker standing at
the poses you saved in Map Studio, so you can see whether each pose is a sane place for
the robot to be before anyone runs a full sim. It is a localhost FastAPI server with a
React UI.

## Start it

From the repo root:

```bash
npm --prefix tools/map_check/web ci && npm --prefix tools/map_check/web run build
env -u PYTHONPATH -u AMENT_PREFIX_PATH uv run --project tools/map_check python -m map_check
```

The `env -u ...` prefix is needed because a ROS-sourced shell sets `PYTHONPATH` and
`AMENT_PREFIX_PATH`, which pull ROS packages and pytest plugins into the tool's own
environment and break it. The build step only needs repeating when the web code changes.

Then open <http://127.0.0.1:9081>. `--port N` (or `MAP_CHECK_PORT`) changes the port.
Only one Map Check server can use a given state folder at a time.

### From another machine

Map Check binds to loopback only and refuses to start on any other host
(`MAP_CHECK_HOST` must be `127.0.0.1`, `::1` or `localhost`). Browser requests whose
Origin is not a loopback address on the same host are also refused. To use it remotely,
tunnel the port and open the same URL locally:

```bash
ssh -N -L 9081:127.0.0.1:9081 <host>
```

## The two screens

- **Map → arena**: pick a Map Studio map or upload a new map (yaml + image). The tool
  finds the closed outlines, and you set the wall height, whether each outline is filled,
  each shape's height, and erase stray wall cells. Saving writes a 3D arena the sim can use.
- **Check poses**: pick a map that has a saved arena, tick the poses to check (from its
  Map Studio `<map>.poses.yaml`, read only), choose views (including behind views and
  head pan/tilt), choose an arm pose, and render. Poses are flagged when the robot's
  clearance is under 2 cm (for example "arm (link3) overlaps wall by 3 cm"). You get one
  image per view plus an overview of the arena.

## Where files go

| What | Where |
| --- | --- |
| Uploaded map (new `<name>.yaml` + image) | the nav maps dir (`MAP_CHECK_MAPS_DIR`) |
| Saved arena | `artifacts/arena/<name>/<version>/` (`arena.usd`, `map.yaml`, image, `boxes.json`, `manifest.json`) |
| Latest arena version | `artifacts/arena/<name>/current.json`, replaced atomically on each save |
| Renders, job files, bench state, robot cache | `outputs/map_check/` |

Never touched: Map Studio's `*.poses.yaml` files are only read. Existing maps are never
overwritten, and neither is an existing arena version directory; saving again makes a new
version. A name already used by a map or an arena is refused. Map names match
`[A-Za-z0-9_][A-Za-z0-9_.-]{0,99}`.

The sim uses a saved arena by name: `validation/run_sim.py --arena <name>` calls
`resolve_arena_artifact`, which follows `artifacts/arena/<name>/current.json` to the
manifest's directory and needs `arena.usd` and `map.yaml` there. It only looks in
`<repo>/artifacts/arena`: an arena saved under another `MAP_CHECK_ARENA_ROOT` (its
`current.json` then holds an absolute manifest path) is invisible to `--arena <name>` until
that directory is linked or copied there.

Saving a picked Map Studio map is refused when an arena of that name exists that Map Check
did not build (for example a hand-made sim arena), so its `current.json` is never repointed.
Uploaded map images must be `.pgm` or `.png`.

### After uploading a new map

The new yaml and image are only in the source maps folder. Rebuild `navigation_bringup`
before Nav2 can load the map; Map Check does not do this for you.

## GPU busy

Renders use Isaac Sim, one process per job, which exits when the job is done. Map Check
will not start a render while the GPU looks busy: any `nvidia-smi` compute app on either
GPU, or a running process whose command is `run_sim.py`, `gpsr-stack`, `isaac-sim`,
`omni.kit`, `isaacsim` or `isaac_render` (another Map Check's render; this server's own job
does not count). The UI says what is using it. If you know it is safe, the override is one
explicit click per render; it is not remembered. A map with a render already queued or
running refuses another until that one finishes. When both
GPUs are free, GPU 1 is used. The render sees only that GPU (`CUDA_VISIBLE_DEVICES`, as
`scripts/gpsr-stack` pins its stages). Each job has a hard timeout of
`min(1800, 120 + 20 x views)` seconds, after which its whole process group is killed. The
render also stops itself if its deadline passes or the server dies without stopping it
(killed, out of memory), so a hung Isaac cannot hold the GPU. A dropped ssh session
(SIGHUP) stops the server and its render like Ctrl+C. The UI estimate is
`40 + 3 x views` seconds. Check with whoever owns the GPU before overriding.

## Settings

Every setting can be overridden with a `MAP_CHECK_<FIELD>` environment variable.
Path defaults that derive from the repo follow `MAP_CHECK_REPO_ROOT`.

| Variable | Default |
| --- | --- |
| `MAP_CHECK_HOST` | `127.0.0.1` (loopback only) |
| `MAP_CHECK_PORT` | `9081` |
| `MAP_CHECK_MAPS_DIR` | `/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps` |
| `MAP_CHECK_ARENA_ROOT` | `<repo>/artifacts/arena` |
| `MAP_CHECK_ROBOT_POINTER` | `<repo>/artifacts/robot/tinker2/current.json` |
| `MAP_CHECK_ARM_POSES_JSON` | `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/HRI/constants.json` |
| `MAP_CHECK_STATE_DIR` | `<repo>/outputs/map_check` |
| `MAP_CHECK_REPO_ROOT` | the checkout containing `tools/` |
| `MAP_CHECK_ISAAC_PYTHON` | `<repo>/.venv/bin/python` |
| `MAP_CHECK_OVERLAP_MARGIN` | `0.02` (m) |
| `MAP_CHECK_DEFAULT_WALL_HEIGHT` | `0.6` (m) |
| `MAP_CHECK_DEFAULT_SHAPE_HEIGHT` | `0.6` (m) |
| `MAP_CHECK_DEFAULT_FILL_AREA` | `4.0` (m2, fill closed outlines up to) |

## Tests

```bash
uv sync --project tools/map_check
env -u PYTHONPATH -u AMENT_PREFIX_PATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  uv run --project tools/map_check pytest tools/map_check/tests -q
npm --prefix tools/map_check/web test
```

Tests that need real data (the measured map, the robot artifact) read it from
`MAP_CHECK_REAL_ROOT` (default `/home/tinker/tinker-sim/6.0.1`) and skip when it is absent.
