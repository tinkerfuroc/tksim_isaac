# Sim-only static+inflation global costmap — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the GPSR-only Nav2 parameter overlay emit a global costmap with `static_layer` + `inflation_layer` only, so the simulator's truth-pose lidar raycast can no longer inject phantom obstacle marks that transiently disconnect the global planner.

**Architecture:** One pure function, `prior_map_costmap_overlay` in `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/nav_params_overlay.py`, rewrites a deep copy of hardware's `nav2_dwb_params.yaml` at launch time (`gpsr.launch.py` is its only caller). The change edits the plugin list constant, deletes the orphaned `obstacle_layer` block from the global section, and documents why. Tests are plain `unittest` in `tests/test_nav_costmap_profile.py` (a hand-built fixture plus the real upstream file when `tk25_ws` is present).

**Tech Stack:** Python 3.10 (system `python3`), `yaml`, `unittest` via pytest; ROS 2 Humble Nav2 parameter YAML (no ROS import needed for the tests).

**Spec:** `docs/superpowers/specs/2026-09-06-nav-global-costmap-static-only-design.md`

## Global Constraints

- Work only in the worktree `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/nav-global-costmap-static-only` (branch `nav-global-costmap-static-only`, off `origin/main` @ bbeaa16). Never touch `/home/tinker/tinker-sim/6.0.1` (the user's checkout) or `/home/tinker/tk25_ws` (hardware's nav repo; read-only).
- Hardware's `nav2_dwb_params.yaml` is never edited; the overlay mutates a deep copy only (`test_input_is_not_mutated` must keep passing).
- Only the GLOBAL costmap changes. The local costmap dict must come out of the overlay exactly as it went in, apart from the pre-existing footprint/inflation rewrites (which the fixture does not exercise).
- Emitted global plugins are exactly `["static_layer", "inflation_layer"]` and the global section must carry no `obstacle_layer` key.
- Test command (run from the worktree root): `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_nav_costmap_profile.py -q`
- Commit trailer on every commit:
  `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_01MGJG2UwdZHoLge2H9szeny`
- No new dependencies, no new files outside the ones listed per task.

---

### Task 1: Drop the global obstacle_layer in the overlay (TDD)

**Files:**
- Modify: `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/nav_params_overlay.py:1-58`
- Test: `tests/test_nav_costmap_profile.py`

**Interfaces:**
- Consumes: `prior_map_costmap_overlay(params: Mapping[str, Any]) -> dict` (existing).
- Produces: same signature; new behaviour — `PRIOR_MAP_PLUGINS == ["static_layer", "inflation_layer"]`; the returned global `ros__parameters` has no `obstacle_layer` key. Task 2 cites these facts in docs.

- [ ] **Step 1: Extend the fixture so the local costmap carries a scan source**

In `tests/test_nav_costmap_profile.py` replace the `local_costmap` block of `SLAM_PROFILE` (lines 57-66) with:

```python
    "local_costmap": {
        "local_costmap": {
            "ros__parameters": {
                "global_frame": "odom",
                "rolling_window": True,
                "width": 5,
                "height": 5,
                "plugins": ["voxel_layer", "inflation_layer"],
                "voxel_layer": {
                    "plugin": "nav2_costmap_2d::VoxelLayer",
                    "observation_sources": "scan",
                    "scan": {"topic": "/scan", "data_type": "LaserScan"},
                },
                "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer"},
            }
        }
    },
```

Then update `test_local_costmap_is_untouched` (lines 101-105) so it still compares the whole dict but says why the scan source matters:

```python
    def test_local_costmap_is_untouched(self):
        """The odom-frame rolling window is correct and stays as it is.

        In particular its ``/scan`` source survives: the local costmap is
        the controller's only reactive layer, and it lives in ``odom`` so
        AMCL corrections never offset its marks (unlike the global map).
        """
        self.assertEqual(
            self.result["local_costmap"], SLAM_PROFILE["local_costmap"]
        )
```

Note: the fixture's local `inflation_layer` has no `inflation_radius` key and no `footprint`, so the overlay's inflation/footprint loops leave it alone and the equality still holds.

- [ ] **Step 2: Write the failing tests for the global costmap**

Replace `test_static_layer_runs_first` (lines 84-89) with:

```python
    def test_static_layer_runs_first(self):
        """StaticLayer sizes the master grid to the arena map."""
        self.assertEqual(
            _global(self.result)["plugins"],
            ["static_layer", "inflation_layer"],
        )

    def test_global_costmap_drops_the_obstacle_layer(self):
        """The sim lidar raycasts the same PGM the static layer loads.

        The obstacle layer can add nothing to the global map; projected with
        the AMCL/EKF estimate instead of the true pose it only draws
        displaced wall copies whose inscribed rings disconnected the planner
        for single ticks (run 20260906T041154: 42 refusal streaks, median
        0 s, 67 % during plain driving).
        """
        section = _global(self.result)
        self.assertNotIn("obstacle_layer", section)
        self.assertNotIn("obstacle_layer", section["plugins"])
        self.assertEqual(section["plugins"][-1], "inflation_layer")
        self.assertEqual(
            section["inflation_layer"]["plugin"],
            "nav2_costmap_2d::InflationLayer",
        )
```

Replace `UpstreamParamsTest.test_overlay_puts_the_real_file_in_prior_map_mode` (lines 157-168) with:

```python
    def test_overlay_puts_the_real_file_in_prior_map_mode(self):
        result = prior_map_costmap_overlay(self.params)
        section = _global(result)
        self.assertFalse(section["rolling_window"])
        self.assertTrue(section["track_unknown_space"])
        self.assertEqual(section["plugins"], ["static_layer", "inflation_layer"])
        # The global map is static-only in simulation (see
        # nav_params_overlay.py); the upstream obstacle block is dropped
        # from the copy rather than left as an unreferenced parameter set.
        self.assertNotIn("obstacle_layer", section)
        # The local costmap's live scan source is carried through untouched.
        local = result["local_costmap"]["local_costmap"]["ros__parameters"]
        upstream_local = self.params["local_costmap"]["local_costmap"]["ros__parameters"]
        self.assertEqual(local["voxel_layer"], upstream_local["voxel_layer"])
        self.assertIn("voxel_layer", local["plugins"])
```

- [ ] **Step 3: Run the tests to verify they fail**

Run (from the worktree root):
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_nav_costmap_profile.py -q`

Expected: 3 failures — `test_static_layer_runs_first` (list has `obstacle_layer` in position 1), `test_global_costmap_drops_the_obstacle_layer` (`obstacle_layer` present), `test_overlay_puts_the_real_file_in_prior_map_mode` (plugins list has three entries). `test_local_costmap_is_untouched` must already PASS. If `UpstreamParamsTest` reports SKIPPED instead of FAIL, `tk25_ws` is missing on this host — stop and report; it must be present (`/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/params/nav2_dwb_params.yaml`).

- [ ] **Step 4: Implement the overlay change**

In `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/nav_params_overlay.py`:

Replace line 30:

```python
PRIOR_MAP_PLUGINS = ["static_layer", "obstacle_layer", "inflation_layer"]
```

with:

```python
# Static + inflation ONLY.  In simulation the "lidar" is
# tinker_sim_isaac.ros_gateway._development_point_cloud: a raycast of the very
# arena PGM that map_server hands the static layer, taken from the robot's TRUE
# pose.  Nav2 projects those returns with the AMCL/EKF ESTIMATE, so every pose
# error draws a displaced copy of nearby walls into an obstacle layer, and the
# copy's 0.21 m inscribed ring (cost 253, an obstacle to NavFn) momentarily
# disconnects narrow passages.  Run 20260906T041154: 95 "failed to create plan"
# refusals in 42 streaks, median duration 0 s, 67 % starting during plain
# driving, zero collisions; the static map alone was fully connected.  Offline
# model (reports/nav-global-costmap-2026-09-06/root-cause-verification.md):
# a single scan displaced 0.15-0.20 m (the documented AMCL 1-sigma) disconnects
# the route; marks left behind by a map->odom correction do so at 0.025 m;
# static + inflation only: 0 disconnections in 17,952 trials.  The layer carried
# no information the static map lacked, so it is dropped rather than tuned.
# The LOCAL costmap keeps /scan: it lives in odom, where AMCL corrections do
# not offset its marks, and it is the controller's only reactive layer.
PRIOR_MAP_PLUGINS = ["static_layer", "inflation_layer"]
```

Replace lines 55-58 (the four assignments after the "StaticLayer resizes the master grid" comment):

```python
    section["rolling_window"] = False
    section["track_unknown_space"] = True
    section["plugins"] = list(PRIOR_MAP_PLUGINS)
    section["static_layer"] = dict(STATIC_LAYER)
```

with:

```python
    section["rolling_window"] = False
    section["track_unknown_space"] = True
    section["plugins"] = list(PRIOR_MAP_PLUGINS)
    section["static_layer"] = dict(STATIC_LAYER)
    # Drop the upstream block with the plugin, so the emitted YAML does not
    # declare a layer the plugin list no longer loads (see PRIOR_MAP_PLUGINS).
    section.pop("obstacle_layer", None)
```

Append this paragraph to the module docstring, before the closing `"""` (after "The upstream file is hardware's, and is left untouched."):

```
The global costmap is also reduced to ``static_layer`` + ``inflation_layer``:
the simulated lidar is a raycast of the same PGM, so an obstacle layer on the
global map can only add pose-estimate error as phantom walls (see
``PRIOR_MAP_PLUGINS``).  The local costmap keeps its live ``/scan`` source.
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_nav_costmap_profile.py -q`

Expected: all tests PASS, 0 skipped (the upstream class runs on this host). Also run
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_navigation_launch_map.py -q` — must still pass (it does not import the overlay; this guards the launch-file neighbourhood).

- [ ] **Step 6: Sanity-check the emitted YAML on the real upstream file**

Run from the worktree root:

```bash
PYTHONPATH=ros2_ws/src/tinker_sim_bridge python3 - <<'PY'
import yaml
from tinker_sim_bridge.nav_params_overlay import prior_map_costmap_overlay
p = yaml.safe_load(open("/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/params/nav2_dwb_params.yaml"))
g = prior_map_costmap_overlay(p)["global_costmap"]["global_costmap"]["ros__parameters"]
print(g["plugins"], "obstacle_layer" in g, sorted(k for k in g if k.endswith("_layer")))
PY
```

Expected output: `['static_layer', 'inflation_layer'] False ['inflation_layer', 'static_layer']`

- [ ] **Step 7: Commit**

```bash
git add ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/nav_params_overlay.py tests/test_nav_costmap_profile.py
git commit -m "fix(nav overlay): static+inflation-only global costmap in simulation

The simulated lidar raycasts the same arena PGM the static layer loads,
from the robot's true pose; Nav2 projects it with the AMCL/EKF estimate.
The global obstacle_layer therefore only ever added displaced wall copies
whose inscribed rings disconnected NavFn for single ticks (run
20260906T041154: 42 refusal streaks, median 0 s, 67 % during plain
driving; static map fully connected). Drop the layer from the generated
sim-only params; the local costmap keeps /scan.

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01MGJG2UwdZHoLge2H9szeny"
```

---

### Task 2: Documentation — CHANGELOG and developer log

**Files:**
- Modify: `ros2_ws/src/tinker_sim_bridge/CHANGELOG.md:8-9` (insert a new subsection directly under `## [Unreleased]`, above the existing `### Fixed (Task 50 ...)`)
- Modify: `docs/developer-log.md:6` (insert a new dated entry directly after the intro paragraph, before `## 2026-09-06 — Task #25 ...`)

**Interfaces:**
- Consumes: the facts from Task 1 (plugin list, dropped block) and the spec's evidence numbers.
- Produces: nothing code-facing.

- [ ] **Step 1: Add the CHANGELOG entry**

Insert immediately after the line `## [Unreleased]` (and its following blank line) in `ros2_ws/src/tinker_sim_bridge/CHANGELOG.md`:

```markdown
### Changed (nav overlay: static+inflation-only global costmap)

- `nav_params_overlay.prior_map_costmap_overlay` now emits the GPSR global
  costmap with `plugins: [static_layer, inflation_layer]` and drops the
  upstream `obstacle_layer` block from the generated copy.  The simulated
  lidar (`ros_gateway._development_point_cloud`) raycasts the same arena PGM
  the static layer loads, from the robot's true pose, while Nav2 projects the
  returns with the AMCL/EKF estimate; the global obstacle layer therefore
  only ever contributed displaced wall copies whose inscribed rings
  disconnected NavFn for single ticks (run 20260906T041154: 95 refusals in
  42 streaks, median 0 s, 67 % during plain driving, zero collisions).  The
  local costmap keeps its live `/scan` source.  Hardware's
  `nav2_dwb_params.yaml` and `navigation.launch.py` are unchanged.  Design:
  `docs/superpowers/specs/2026-09-06-nav-global-costmap-static-only-design.md`.

```

- [ ] **Step 2: Add the developer-log entry**

Insert in `docs/developer-log.md` immediately before the line
`## 2026-09-06 — Task #25: bound \`controller_reconciler\`'s post-success teardown so it cannot wedge the launch chain`:

```markdown
## 2026-09-06 — GPSR nav: planner refusals were phantom obstacle marks, not inflation

**Symptom.** Full-stack GPSR runs (`gpsr_stack_logs/20260906T041154`, scenarios
s2026-000 laundry_desk and s2026-019 side_table_02) lost 8 of 10 gotos:
`GridBased: failed to create plan with tolerance 0.60` → `Invalid path, Path is
empty` → spin/wait/backup recoveries → `Goal failed`. The first hypothesis
(global `inflation_radius` 0.45 too large for waypoints 0.2–0.4 m from
furniture) was wrong: the 253 ring is the footprint's inscribed radius
(0.21 m) regardless of `inflation_radius`, and a flood fill over the static
arena costmap reaches both goal boxes from every attempt start (goal cells cost
0 / 48).

**Measured** (evidence copied to `reports/nav-global-costmap-2026-09-06/`,
primary capture in the s2026-019 run dir's `nav-evidence/`): 95 refusals in
42 streaks, median streak duration 0 s (single 1 Hz tick, max 40 s); 67 % of
streaks begin during plain driving with no recovery in the preceding 30 s;
attempts 2–5 and 8 logged zero controller collisions; 45 % of refusals fall
12–106 ms after `ClearEntireCostmap`, i.e. the BT replanned before the 10 Hz
master grid was rebuilt and escalated the blip into the recovery round-robin.
No safety-supervisor or gateway stop ever fired. The doorway at x=0 is 0.95 m
in both USD and PGM. Humble RPP's `inCollision` threshold is `LETHAL_OBSTACLE`
(254), not inscribed.

**Ruled out.** Inflation radius (above); waypoint placement (goals connected);
physical blockage / safety stop (none); a chain defect — the truth-pose
raycast origin equals the `base_link→livox360` static TF, `pointcloud_to_laserscan`
does no TF lookup (`target_frame == cloud frame`), all stamps are sim time;
nothing in the chain can displace a return.

**Root cause.** The simulated lidar is a raycast of the arena PGM from the
robot's TRUE pose; Nav2 projects it with the AMCL/EKF ESTIMATE (wheel odometry
only). Every pose error draws a displaced copy of nearby walls into the global
`obstacle_layer`; the copy's inscribed ring momentarily disconnects narrow
passages, and marks made just before a map→odom correction stay offset until
later rays raytrace through them. Offline model reusing the real
`OccupancyMap.raycast_many` and the gateway's 181-ray set, NavFn as BFS over
< 253 with the 0.6 m tolerance box: zero error → 100 % connected (132/132
route poses); a single scan displaced by 0.15 m (side_table_02 route) or
0.20 m (laundry_desk) — the documented ~0.2 m AMCL 1σ — disconnects; the
jump model (last 5 scans offset, robot at truth) disconnects at 0.025 m / 2°;
pure yaw error alone is weak (1/66 poses, ~15.5°). With `static_layer` +
`inflation_layer` only: 0 disconnections in 17,952 trials. A true-body
footprint (inscribed 0.13 m) only doubles the tolerable error (0.15 → 0.275 m).

**Fix.** `nav_params_overlay.py` emits the GPSR global costmap with
`plugins: [static_layer, inflation_layer]` and drops the `obstacle_layer`
block from the generated copy: the layer carried no information the static
map lacked, so it is removed rather than tuned. Local costmap unchanged (odom
frame; the controller's only reactive layer). `inf_is_valid` (a no-op here —
every ray hits a wall or the map edge within 40 m) and a BT retry around
`ComputePathToPose` were considered and left for follow-up. Live gate before
merge: rerun s2026-019 with `nav_monitor` (`map_to_odom_jumps.csv`),
`/sim/internal/physics_truth` vs `/amcl_pose`, and a bridge-log grep — success is
zero `failed to create plan` during driving and the goal reached. Residual
static-map fragility noted: a 0.50 m free channel at (-1.82, -0.06) near
`wall_0012`/TV stand on the laundry_desk route.

```

- [ ] **Step 3: Check the Markdown renders sanely**

Run: `grep -n '^## 2026-09-06' docs/developer-log.md | head -3` — expected: the new entry's heading on the first line of output, the Task #25 heading second.
Run: `sed -n '/^## \[Unreleased\]/,/^### Fixed/p' ros2_ws/src/tinker_sim_bridge/CHANGELOG.md | head -20` — expected: the new `### Changed` subsection appears before `### Fixed`.

- [ ] **Step 4: Commit**

```bash
git add ros2_ws/src/tinker_sim_bridge/CHANGELOG.md docs/developer-log.md
git commit -m "docs: record the phantom-mark root cause and the static-only global costmap fix

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01MGJG2UwdZHoLge2H9szeny"
```

---

### Task 3: Verification, push, draft PR

**Files:**
- None modified (verification and git only).

**Interfaces:**
- Consumes: commits from Tasks 1 and 2 on branch `nav-global-costmap-static-only`.
- Produces: the branch on `origin`, a draft PR, and the exact changed-file list + SHA for the validating session.

- [ ] **Step 1: Run the unit suites that touch the nav launch neighbourhood**

Run from the worktree root:
```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_nav_costmap_profile.py tests/test_navigation_launch_map.py -q
```
Expected: all PASS, 0 skipped.

- [ ] **Step 2: Confirm the diff is confined to the planned files**

Run: `git diff --stat origin/main..HEAD`
Expected files only: `docs/superpowers/specs/2026-09-06-nav-global-costmap-static-only-design.md`, `docs/superpowers/plans/2026-09-06-nav-global-costmap-static-only.md`, `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/nav_params_overlay.py`, `tests/test_nav_costmap_profile.py`, `ros2_ws/src/tinker_sim_bridge/CHANGELOG.md`, `docs/developer-log.md`. Anything else is a defect — stop and report.

- [ ] **Step 3: Push the branch and open a draft PR**

```bash
git push -u origin nav-global-costmap-static-only
gh pr create --draft --base main --title "fix(nav overlay): static+inflation-only global costmap in simulation" --body "$(cat <<'BODY'
## Summary
- GPSR-only Nav2 overlay now emits the global costmap with `plugins: [static_layer, inflation_layer]` and drops the upstream `obstacle_layer` block from the generated copy.
- Root cause: the simulated lidar raycasts the same arena PGM from the TRUE pose while Nav2 projects it with the AMCL/EKF estimate, so the global obstacle layer only ever drew displaced wall copies whose inscribed rings disconnected NavFn for single ticks (run 20260906T041154: 95 refusals / 42 streaks, median 0 s, 67 % during plain driving, zero collisions; static map fully connected). Offline model: 0.15–0.20 m single-scan / 0.025 m jump-model disconnects; 0/17,952 with static+inflation only.
- Spec: `docs/superpowers/specs/2026-09-06-nav-global-costmap-static-only-design.md`; developer-log entry included.

## Test plan
- [x] `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_nav_costmap_profile.py tests/test_navigation_launch_map.py -q`
- [ ] Live gate (GPSR testing session, GPU boot user-gated): rerun s2026-019 with nav_monitor + physics_truth vs /amcl_pose; success = zero `failed to create plan` during driving and goal reached.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

https://claude.ai/code/session_01MGJG2UwdZHoLge2H9szeny
BODY
)"
```
Expected: push succeeds; `gh` prints the PR URL. Record the PR URL and `git rev-parse HEAD`.

- [ ] **Step 4: Report**

Return: HEAD SHA, PR URL, and the exact file list from Step 2 (the validating session checks out precisely these files).
