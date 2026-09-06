# Sim-only Nav2 global costmap: static + inflation only — design

Date: 2026-09-06. Status: approved for implementation (user instruction 2026-09-06),
pending live validation before merge.

## Problem

Full-stack GPSR runs in the Isaac simulation fail their navigation legs: `navigate_to_pose`
to near-furniture waypoints aborts repeatedly (`GridBased: failed to create plan with
tolerance 0.60` → `Invalid path, Path is empty` → recovery churn → goal failed after the
recovery budget). In run `20260906T041154` (scenarios s2026-000 laundry_desk and s2026-019
side_table_02) 8 of 10 gotos failed this way; the arm never got a turn.

Evidence (durable copy: `reports/nav-global-costmap-2026-09-06/`, gitignored local
evidence like every other `reports/` dir; primary capture in
`tk25_decision/.../GPSR/gpsr_runs/bench/t2-2026/runs/s2026-019-takeObjFromPlcmt/nav-evidence/`):

- The static arena costmap is fully connected: a flood fill over NavFn-traversable cells
  (< 253) reaches both goal boxes from every attempt start except one (attempt 9 started in
  a 253 cell after a BackUp recovery). Goal cells: side_table_02 cost 0, laundry_desk 48.
- The 95 planner refusals form 42 streaks with **median duration 0 s** (single 1 Hz tick,
  max 40 s); **67 %** begin during plain driving with no recovery in the preceding 30 s;
  attempts 2–5 and 8 had zero controller collision warnings. 45 % of refusals land
  12–106 ms after `ClearEntireCostmap`, i.e. the BT replanned before the 10 Hz master grid
  was rebuilt — the recovery tree amplifies each blip into spin/wait/backup.
- No safety-supervisor or command-gateway stop ever fired; the x=0 doorway is 0.95 m in
  both USD and PGM; mesh furniture is rasterized (an intermediate report claiming otherwise
  was checked and is wrong — see `ERRATA.md`).
- The scan chain (truth-pose raycast → `livox360` → `pointcloud_to_laserscan` → obstacle
  layer) is geometrically consistent; nothing in it can displace a return. Only the
  AMCL/EKF pose estimate can.

## Root cause

The simulator's lidar is not a sensor: `ros_gateway._development_point_cloud` raycasts the
arena PGM from the robot's **true** pose. Nav2 projects those returns into the map with the
**estimated** pose (AMCL map→odom ∘ EKF odom→base_link, wheel odometry only). Every pose
error draws a displaced copy of nearby walls into the global costmap's `obstacle_layer`;
the copy's 0.21 m inscribed ring (cost 253, an obstacle to NavFn) momentarily disconnects
narrow passages. Marks made just before a map→odom correction stay offset until later rays
happen to raytrace through them.

Offline verification (`root-cause-verification.md`, model reuses the real
`OccupancyMap.raycast_many` and the gateway's exact ray set; NavFn modelled as BFS over
< 253 cells with the 0.6 m tolerance box; 38 + 28 route poses):

| Experiment | Result |
|---|---|
| Zero error, both routes | 100 % connected (132/132) |
| Single displaced scan, min disconnecting translation | 0.15 m (side_table_02 route), 0.20 m (laundry_desk) — at the documented ~0.2 m AMCL 1σ |
| Single displaced scan, pure yaw | weak: 1/66 poses, only at ~15.5° |
| Jump model (last 5 scans offset, robot at truth) | 0.025 m / 2.0° — inside normal noise |
| Static + inflation only, all sweeps | 0 disconnections / 17,952 trials |
| Smaller footprint (inscribed 0.13 m) | min error 0.15 → 0.275 m — helps, does not eliminate |

The single-scan model reproduces the observed single-tick blip; the jump model the rarer
multi-second streaks.

## Design

Because the synthetic lidar raycasts the very PGM the `static_layer` loads, the global
costmap's `obstacle_layer` can never add information in simulation — it can only inject
phantoms. Remove it from the generated sim-only Nav2 parameters.

Change, confined to `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/nav_params_overlay.py`
(`prior_map_costmap_overlay`, the GPSR-only overlay applied to a *copy* of hardware's
`nav2_dwb_params.yaml`):

1. `PRIOR_MAP_PLUGINS = ["static_layer", "inflation_layer"]` (was `[static, obstacle,
   inflation]`).
2. Delete the `obstacle_layer` mapping from `global_costmap.global_costmap.ros__parameters`
   in the copy, so the emitted YAML does not carry an unreferenced layer block.
3. Module docstring / inline comment: state the zero-information argument, the evidence
   numbers above, and that the local costmap keeps `/scan` on purpose.

Everything else in the overlay (static layer, non-rolling window, tolerances, RPP, footprint,
inflation 0.45 on both costmaps, progress checker) is unchanged. The **local** costmap is
untouched: it lives in `odom`, so AMCL corrections do not offset its marks, and it is the
only reactive layer the controller has.

Emitted global costmap after the change (from the upstream file):

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      rolling_window: false
      track_unknown_space: true
      plugins: [static_layer, inflation_layer]
      static_layer: {plugin: nav2_costmap_2d::StaticLayer, map_subscribe_transient_local: true}
      inflation_layer: {plugin: nav2_costmap_2d::InflationLayer, cost_scaling_factor: 5.0, inflation_radius: 0.45}
      footprint: "[ [0.20, 0.27], [0.20, -0.27], [-0.45, -0.27], [-0.45, 0.27] ]"
      # ... unchanged keys (frames, resolution, update rates)
```

## Alternatives considered and rejected

- **Lower global `inflation_radius` 0.45 → 0.22** (first hypothesis): the 253 ring is the
  footprint's inscribed radius (0.21 m), independent of `inflation_radius`; no effect.
- **Move waypoints away from furniture**: goals are connected in the static map; not the
  cause. (Also authored in `tk25_decision`, user-owned.)
- **`inf_is_valid: true` on the scan sources**: in this arena every ray hits a wall or the map
  edge within 40 m, so `inf` never occurs — a no-op. Follow-up only if a boundless arena appears.
- **BT `RetryUntilSuccessful` around `ComputePathToPose`**: masks blips instead of removing
  their cause, and delays recoveries for genuinely blocked goals. Follow-up only if the live
  run still shows refusals during driving.
- **Smaller (true-body) footprint**: secondary lever (halves the sensitivity); keep as a
  follow-up, not bundled, so the live run isolates one variable.
- **Truth-based odometry**: violates the hardware-parity contract (`truth_odometry: false`).

## Scope and non-goals

- GPSR path only (`gpsr.launch.py` is the sole caller of the overlay).
  `navigation.launch.py` runs hardware's rolling SLAM-style profile unchanged.
- Hardware's `nav2_dwb_params.yaml` is never edited.
- Not addressed: RPP "collision ahead" from local-costmap marks under odometry yaw slip,
  the BT clear→replan race, and the one static-map-marginal corridor (0.50 m free channel
  at (-1.82, -0.06) near `wall_0012`/TV stand on the laundry_desk route).

## Testing

Unit (`tests/test_nav_costmap_profile.py`, run as
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_nav_costmap_profile.py -q`):

- `test_static_layer_runs_first` → global `plugins == ["static_layer", "inflation_layer"]`.
- New `test_global_costmap_drops_the_obstacle_layer` → no `obstacle_layer` key in the global
  section; `inflation_layer` and `static_layer` present.
- `test_local_costmap_is_untouched` → unchanged assertion (still byte-identical; the fixture
  gains a `voxel_layer.scan` block so the test proves the local scan source survives).
- `UpstreamParamsTest.test_overlay_puts_the_real_file_in_prior_map_mode` → asserts
  `obstacle_layer` is absent from the real file's overlaid global section and that the
  real file's local `voxel_layer.scan` observation source is carried through untouched.
- `test_input_is_not_mutated`, `WritePriorMapParamsTest` → unchanged; must still pass.

Live (owned by the GPSR testing session, GPU boot gated on the user): rerun s2026-019 with
`nav_monitor` (sim-time params; `map_to_odom_jumps.csv`, `rates.csv`), `/sim/internal/physics_truth`
vs `/amcl_pose` echoes, `/global_costmap/costmap_updates` capture, and a post-run grep of the
bridge log. Success: **zero** `failed to create plan` during driving, goal reached.
Merge is gated on that run.

## Documentation

- `nav_params_overlay.py` docstring/comment (the file is the only prose that documents the overlay).
- `ros2_ws/src/tinker_sim_bridge/CHANGELOG.md` `[Unreleased]`.
- `docs/developer-log.md` dated entry: symptom, measurements, ruled-out hypotheses, fix shape
  (fix narratives go there, not the runbook).

## Rollout

Branch `nav-global-costmap-static-only` (worktree off `origin/main` @ bbeaa16), pushed to
origin; draft PR; the validating session checks out the exact changed files onto its
provisioned tree and rebuilds `tinker_sim_bridge`; merge after the live gate passes.
