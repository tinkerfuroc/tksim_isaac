# GPSR sim navigation bug report

Source: past GPSR bench runs served by the trial viewer on `127.0.0.1:18765`
(`tk25_decision/src/behavior_tree/behavior_tree/GPSR/bench/serve_trials.py`), sessions 2026-10-01 to 2026-10-07.
Findings were each checked by two independent reviewers; corrections are folded in. No project code was changed; the analysis scripts are in [`2026-10-08-gpsr-nav-tools/`](2026-10-08-gpsr-nav-tools/).

Four sections, one per area. Each can be debugged in its own session.

| # | Area | Repo | Priority |
|---|------|------|----------|
| 1 | Nav2 config in the sim stack | `tinker-sim/6.0.1` (+ `tk25_ws` install) | High — causes the spawn spinning |
| 2 | Approach planner | `tk25_ws/src/tk26_navigation/src/approach_planner` | High — causes most approach aborts |
| 3 | Bench harness: trial reset | `tk25_decision/.../GPSR/bench`, `tinker-sim/6.0.1/scripts` | Medium |
| 4 | Person detection position | vision stack | Medium |

## Shared context (read first)

- **Trial runs:** `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/<batch>/runs/<entry>/` — `run.json` (verdicts), `orchestrator.log`, `logs/cmd_*.log`, `debug/*/events.jsonl`, `scene-plan.json`, `spawned.json`, `rosbag/rosbag_0.db3.zstd` (only `/tf`, `/joint_states`, camera images; no `cmd_vel`/`odom`).
- **Stack logs:** `/home/tinker/tinker-sim/6.0.1/gpsr_stack_logs/<YYYYMMDDTHHMMSS>/`. `02-bridge.log` has Nav2 (`bt_navigator`, `controller_server` = RPP, `planner_server` = GridBased, `behavior_server`). `05-nav_extras-0.log` has `approach_planner`. Lines carry epoch timestamps: `[controller_server-22] [WARN] [1791027951.2] [controller_server]: ...`
- **Batch → session:** `grep "stack up. logs" bench/<batch>/gpsr-stack-up.log`.
- **Nav2 params actually generated:** `/tmp/nav2_dwb_params.prior_map.uid1002.domain<N>.yaml`, written by `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/nav_params_overlay.py` from `gpsr.launch.py:~154`.
- **Geometry:** spawn ≈ (−1.93, −2.00, yaw −1.3°). Command point = (−2.00, −2.00, yaw 0°).

### Tools and pose traces

Scripts used for this report live in [`2026-10-08-gpsr-nav-tools/`](2026-10-08-gpsr-nav-tools/), next to this file. Run them from that folder (they import `navtrace.py`); each prints usage with `-h`. All default to the paths above.

| Script | Purpose | Used for |
|---|---|---|
| `extract_tf_pose.py <run_dir> <out.csv>` | One rosbag → pose CSV | all trace work |
| `extract_traces.sh [-o DIR] [-j JOBS] [BATCH_GLOB]` | Many rosbags → CSVs | all trace work |
| `pose_trace.py <csv>` | Map-frame pose over time, distance to goal, cumulative rotation | 1a |
| `start_motion.py [DIR or CSVs]` | How each trial leaves its start: dwell, rotation, reversals, spin windows | 1a, 3a |
| `trace_goals.py <csv>` | Nav2 goals during a traced run, with duration, collision-ahead count, rotation | 1a, 1c |
| `nav_goal_census.py [--goal -2,-2]` | Every goal to a point across all sessions, bucketed by start distance (log-only, no bags) | 1a, 1c |
| `nav_timeline.py <session> <t0> <t1>` | Condensed Nav2 + approach_planner log timeline | all |
| `nav_warn_tally.py [--node planner_node]` | Warning/error counts across sessions | all |
| `approach_aborts.py [SESSION ...]` | Each `GoToApproach ABORTED` → run entry and verdict | 2 |
| `costmap_check.py --pose=x,y[,yaw] --reach=sx,sy:gx,gy` | Static-map costmap check, hardware vs sim footprint | 1b, 1c, 2b |

**Pose traces are not kept; regenerate them into `/tmp/gpsr-nav-traces/`.** 52 traces were left in `/tmp/gpsr-nav-traces/` on 2026-10-08, but `/tmp` may have been cleared since. They are produced from the bench rosbags:

```bash
cd docs/issue_reports/2026-10-08-gpsr-nav-tools
./extract_traces.sh -j 2 't2-five-diverse-gpu1-20261007-rerun2'   # one batch
./extract_traces.sh -j 2 't2-20-live-rerun14'
./extract_traces.sh -j 2                                          # all t2-* batches: hours
python3 start_motion.py                                           # reads /tmp/gpsr-nav-traces
```

- Each CSV is named `<batch>__<entry>.csv`, with columns `t,parent,child,x,y,yaw`: raw `/tf` rows for `map→odom` and `odom→base_link`. `navtrace.load_trace()` composes the map-frame pose and drops base_link rows that come before the first `map→odom`. Measure rotation with the odom yaw, because map-frame yaw jumps on AMCL corrections.
- Each bag is decompressed to a temporary `.db3` of 1–15 GB in `/dev/shm/gpsr-nav` and deleted afterwards, so `-j` × 15 GB of RAM must be free. Expect about 3–6 bags per 10 minutes. For questions that span many runs, use the log-only `nav_goal_census.py`.
- The bags hold only `/tf`, `/joint_states` and camera images: no `cmd_vel` and no `odom`.
- Three bags are corrupt (zstd fails): rerun14 `005` and `016`, and diverse15-rerun15 `div15-06`.
- Traces extracted for this report (53):
  - `t2-15-distinct-rerun13…27` (12 batches, one entry each);
  - `t2-20-live-rerun11`, `12`, `13`, `15`, and `100…112` (one entry each);
  - `t2-20-live-rerun14` entries `000`, `001`, `002`, `003`, `004`, `006`, `007`, `008`, `010`, `011`, `013`, `015`, `017`;
  - `t2-diverse15-rerun5`, `9`, `25`, `30`, `35`, `36`;
  - all 5 entries of `t2-five-diverse-gpu1-20261007-rerun2`.
  
  Regenerate by batch glob as shown above.

---

## 1. Nav2 config in the sim stack

Three problems in how the sim configures Nav2. 1a is the reported "robot spins near spawn when a task starts."

### 1a. RPP rotates in place for minutes when the goal is 0.1–0.5 m away

**Symptom.** A Nav2 goal to the command point from 0.1–0.5 m away turns back and forth without driving. It succeeds by chance or is cancelled.

| Start distance to (−2, −2) | Goals | Median time | > 30 s | Outcome |
|---|---|---|---|---|
| < 0.1 m | 146 | 0.1 s | 0 | all succeeded |
| 0.1–0.5 m | 57 | 64 s (max 394 s) | 38 | 39 ok, 17 cancelled, 1 failed; 22 had "Failed to make progress"; 0 had "collision ahead" |

Worst case: batch `t2-five-diverse-gpu1-20261007-rerun2`, entry `div15-s20261008-div15-11-count-countPrsInRoom`, session `20261007T030006`, epoch 1791357966–1791358068. Started 0.26 m from the goal. Rotated 1511° with 9 reversals over 134 s and never moved more than 0.31 m.

**Root cause.**
- The goal is behind the robot. `allow_reversing: false`, so RPP must turn ~180° to face it.
- The path is shorter than `min_lookahead_dist` (0.25), so the carrot is the goal itself.
- Humble RPP's `shouldRotateToGoalHeading` compares carrot distance with the goal checker's `xy_goal_tolerance` (0.2). Inside 0.2 m it turns to the goal yaw (0°). Outside it turns toward the path (~180°). Both branches command zero linear speed.
- The base does not pivot about `base_link`: a ~160° turn shifts it 0.07–0.1 m. Turning to the goal yaw pushes it back across the 0.2 m line, which flips the branch again. This repeats about every 21 s.
- The progress checker counts xy only (0.15 m / 30 s), so it fires "Failed to make progress" and runs a recovery spin.
- From the true spawn the goal is 0.07 m away and yaw error is 0.02 rad, so the goal checker succeeds immediately.

The branch is inferred from the pose trace, since the bags have no `cmd_vel`.

**Fix.**
- In `nav_params_overlay.py`, raise `general_goal_checker.xy_goal_tolerance` from 0.2 to about 0.4. RPP reads the same value, so this fixes both the done test and the branch flip. Optionally raise `yaw_goal_tolerance` to about 0.4.
- Alternative: send the command-point goal with the robot's current yaw, or skip it when within 0.4 m.

**Done when.** A goal to (−2, −2) from 0.2–0.4 m away with the robot facing away finishes in under 15 s with no reversals.

### 1b. Costmaps use the 0.95 m hardware footprint, not the sim one

**Symptom.** `approach_planner` logs `adopted live footprint ... [(0.26, 0.26), (0.26, -0.26), (-0.71, -0.26), (-0.71, 0.26)]` in all 168 sessions since 2026-09-25. The overlay sets the 0.65 m sim footprint `[[0.20,0.27],[0.20,-0.27],[-0.45,-0.27],[-0.45,0.27]]`.

**Root cause.**
- The installed launch `/home/tinker/tk25_ws/install/navigation_bringup/share/navigation_bringup/launch/navigation_dwb_launch.py` (dated Sep 20) adds `'footprint': RobotField('base.footprint')` to its `RewrittenYaml` substitutions. This replaces every `footprint` key in both costmaps after the overlay has run.
- `RobotField` reads `tinker_robot_config/robots/_common/robot.yaml:9`, which holds the hardware footprint. Nav2's default 0.01 padding gives the logged values.
- Proof: the RewrittenYaml temp file for session `20261007T030006` has the hardware footprint at lines 427 and 466, while the overlay output `/tmp/nav2_dwb_params...domain142.yaml` has the sim one.
- `gpsr.launch.py:525` hard-sets `ROBOT_NAME=tinker2`, so changing the environment does nothing.
- The `tk26_navigation` source checkout is on branch `map-studio`, which lacks this substitution. `main` (commit `06c7c36`) has it, so rebuilding from `main` keeps the problem.

**Fix (pick one).**
- A sim-only robot profile, e.g. a copy of `tinker2` plus `base.footprint`, selected by the sim launch. Do not change `_common` or `tinker2`: that changes the real robot.
- Re-apply the sim footprint after the rewrite: a runtime param set on both costmaps, or a sim-side copy of the nav launch.

`approach_planner` follows `/global_costmap/published_footprint`, so it picks up the change automatically.

**Done when.** `ros2 topic echo /global_costmap/published_footprint` matches the sim footprint plus padding, and the approach planner logs it.

### 1c. Long collision-ahead loops when starting near furniture

**Symptom.** 77 goals from ≥ 2 m away. 46 hit RPP "detected collision ahead!" (up to 365 times) followed by "Controller patience exceeded", a costmap clear and a retry. 35 were cancelled and 23 failed.

| Case | Session, epoch | Start | Notes |
|---|---|---|---|
| rerun14 `006` | `20261003T062109`, 1791027932 | (3.33, −2.25) | 0.45 m from the kitchen table cluster; 174 warnings, 2 patience aborts |
| rerun14 `001` | same, ~1791023153 | (−0.70, −4.28) | stuck ~40 s |
| diverse15-rerun26 `navigation-goToLoc` | `20261006T100442` | (−4.29, 0.63) | 0.10 m from the shelf; planner fails and every spin/backup reports "Collision Ahead" |

**Root cause.**
- The start pose is inside furniture inflation (0.45 m). Rotation only escapes when the heading happens to point away.
- The larger footprint (1b) matters in one case. A rotation sweep at `001` finds 48 of 72 headings lethal with the hardware footprint versus 9 with the sim one.
- At `006` and the shelf case, the sim footprint would not help.

Method: a static-map costmap reconstruction. Reproduce it with `costmap_check.py --pose=3.33,-2.25,-125 --pose=-0.70,-4.28 --pose=-2.03,-4.55 --reach=-4.29,0.63:-3.69,1.0`. It was not confirmed with a live Nav2 run.

**Fix.**
- Fix 1b, then rerun these cases.
- Keep trial start poses at least 0.6 m from furniture (see section 3).

**Done when.** Starts at the poses above leave without patience aborts under the sim footprint, or the harness no longer produces such starts.

---

## 2. Approach planner

`GoToApproach` problems. Installed copy is authoritative: `/home/tinker/tk25_ws/install/approach_planner/lib/python3.10/site-packages/approach_planner/`. The `src/` copy is older; for example it has no `action_server.py` or `barrier_check.py`. Port fixes to whichever is built.

Totals: 39 `GoToApproach ABORTED` lines in 16 sessions.

### 2a. "No path" is treated as "arrived", so failed attempts run for about 60 s

**Root cause.**
- Nav2 reports `distance_remaining = 0` when it has no path. `nav_client.py:264` computes `arrived = d_rem <= arrival_threshold_m (0.30)`, which skips the no-progress watchdog.
- `progress.py` `ProgressTracker` counts base motion (0.05 m / 0.15 rad) as progress, so Nav2 recovery spins and backups count. The 60 s first-attempt cap then logs "first-attempt cap reached but goal is progressing; continuing".
- `planner_node.py:218` `enable_recompute_attempt=False`, so no other stand pose is tried. The action aborts with status 9 and logs `last_d=0.00m stalled=False`.
- Exits seen: the 60 s soft cap after about 8 s of no motion (2 cases); Nav2's own BT abort after 6 recovery retries (1 case, `20261005T024243`).

Example: session `20261003T062109`, epoch 1791025587–1791025655. 12 planner failures, spin, wait and backup for 65 s.

**Fix.**
- Treat `d <= 0` as "no path / unknown", not arrival.
- Do not count motion during Nav2 recovery behaviors as progress.
- Consider enabling the recompute attempt.

**Done when.** An unplannable stand pose is given up within the no-progress timeout (8 s) and an alternative is tried.

### 2b. Blind offset stand pose can be inside furniture

**Symptom.** Session `20261003T062109`, ~1791025587. Target (−2.00, −5.15) on the sofa; stand pose (−2.03, −4.55). The sofa bounding box is x −3.24…−1.13, y −5.29…−4.39, so the pose is inside it and on occupied static-map cells. GridBased failed all 12 tries. Similar: `20261004T100658`, `20261005T024243`.

**Root cause.**
- `action_server.py` (~l.335–370) builds the pose with `make_offset_candidate` and checks only line of sight via `planner_node._offset_pose_los_ok` (~l.957–977). There is no costmap or footprint check.
- `pose_has_clear_access` in `surface.py` returns "clear" when the pose is still inside the target's own blocking run (the "run never exited" branch, depth 0.6 m ≤ `max_depth` 1.0 m).

**Fix.** Before sending, check the blind pose's footprint cost against the costmap. Reject poses inside the target's own obstacle run.

**Done when.** Replaying the sofa target produces a stand pose outside the sofa, and Nav2 plans to it.

### 2c. "No free candidate around target" next to walls

**Symptom.** 18 aborts in 9 sessions:
- 12 are the living-room person detected at (−4.23…−4.43, −5.16…−5.32), on or against the south-west corner walls (cell value 99/100). Sessions `213429`, `004721`, `014521`, `052211`, `085442`. Each is logged twice because the BT retries after about 1 s.
- 2 are the same person detected outside the arena at (−2.89, −5.80) (`20261004T204512`).
- 4 are object approaches near furniture (`20261001T055739`, `20261006T060508` ×2, `20261007T030006`).

**Root cause.**
- In `algorithm.py` (~l.428–485) each ring pose goes through the footprint raster `_pose_mean_cost` (lethal threshold 99, live 0.97 m footprint + 0.04 m margin) before `pose_has_clear_access`.
- The robot does not fit in the pocket formed by the west wall, the south wall, a small static blob at x −4.10…−3.90, y −5.10…−4.85, and the block at x ≥ −3.25.
- The distance band matters: with [0.45, 1.60] some targets get a candidate (a live run found one at 1.50 m for (−4.36, −5.32)).
- Stale latched costmaps (logs say 124–1539 s old) may also block wider-band candidates.

The detection position itself is section 4.

**Fix.**
- Fix 1b (the smaller footprint may fit).
- Widen the band automatically on this error.
- For person targets, snap or reject targets that fall inside static obstacles.
- Investigate the stale costmap snapshot.

**Done when.** Replaying these detections either finds a candidate or rejects the target as invalid with a clear reason.

### 2d. Barrier check flags the target person's own body

**Symptom.** Session `20261003T062109`: 6 aborts with "every one of N candidate(s) approaches the target through the barrier the lidar saw" or "barrier seen ... giving up". Targets (−3.68, −4.18), (−3.77, −4.46), (−0.69, 3.82). Barrier points are 0.18–0.24 m from the target, up to 1.74–1.80 m high (the 1.8 m band cap).

**Root cause.** `barrier_check.py` (~l.120–138) uses the 0.5 m person clear radius only when `target_z <= 0.05`. Otherwise it uses `min(0.25, 0.3 × corridor) = 0.18 m`, so the person's upper body counts as a barrier. `target_z > 0.05` is inferred, since the log does not print it.

The fit is weak for (−3.77, −4.46): the points sit 0.14–0.18 m outside the person's collision box. Possible lidar or TF offset.

**Fix.** Use the person clear radius for any person target, regardless of z.

**Done when.** Replaying these targets yields no barrier abort.

---

## 3. Bench harness: trial reset

### 3a. The base is never reset between trials

**Symptom.** In multi-trial sessions each trial starts where the robot was left. For example, rerun14 `001` starts at (−0.70, −4.28), exactly where `000` ended. These off-spawn starts trigger 1a (0.1–0.5 m off) or 1c (far, near furniture).

**Root cause.**
- `bench/tier2.py:~739` runs `--reset-cmd`, set by every `battery*.sh` to `tinker-sim/6.0.1/scripts/gpsr-run-reset`.
- That script only zeroes the xarm and head joints and sleeps 9 s. Its header says it "Deliberately does NOT touch the base pose". Commit `5fa9186` removed the old teleport plus re-localize because "SetEntityState teleports are dead USD writes".
- Nothing cancels Nav2 goals. When an orchestrator is killed on TIMEOUT, its last `NavigateToPose` keeps driving through reset, settle and spawn. Example: five-diverse rerun2 trial `09` ended at (1.49, −3.72); the orphaned goal drove the robot to (−1.71, −2.00) before trial `11` began. That is the 0.26 m start in 1a.
- An unexplained yaw change (24.5° → 101°) between five-diverse trials `00` and `03` happened with no Nav2 goal in between.

**Uncommitted change to account for.** `tk25_decision` work tree: `gpsr_orchestrator.py` (~l.145–175) and `bench/tier2.py:799` (`start_outside_arena=False`). In sim this skips the command-point goal; the comment cites the spinning. This hides 1a at task start, but:
- Future multi-trial batches will start each trial wherever the robot is, with no return to (−2, −2).
- Mid-task returns to (−2, −2) still hit 1a.
- `gpsr-run-reset`'s comment about the orchestrator driving back is now stale.

**Fix.**
- Cancel all Nav2 goals (`navigate_to_pose`, `go_to_approach`) in the reset step.
- Restore the base pose with a method that works on the Fabric articulation, then re-localize AMCL. Or run one trial per stack session.

**Done when.** Every trial's first pose is within 0.05 m and 2° of spawn, and no Nav2 goal is active when a trial starts.

**Status (2026-10-10).** Fixed in tinker-sim `dev` (`01f49a6`..`656ee6b`, plus docs). `scripts/gpsr-run-reset` now runs `tools/gpsr_run_reset.py`. It cancels every action goal, restores the base with the sim's scope-2 STATE reset (a PhysX-view write), re-seeds AMCL from truth, and gates each trial start at 0.05 m / 2°.
- Batch `t2-five-diverse-reset-20261008`: 5 of 5 starts at (−2.0002, −2.0000, −0.08°), a spread of 0.000 m.
- Orphaned-goal check (2026-10-10): the in-flight goals were cancelled, and the base came back 3 m to spawn.

The true spawn is the scenario pose (−2.000, −2.000, ~0°), not the (−1.93, −2.00) given above. Now that every trial starts 0 m from the command point, the uncommitted skip-command-point change in `tk25_decision` is no longer needed at task start. Mid-task returns still need §1a.

---

## 4. Person detection position

**Status (2026-10-11): fixed and verified end to end.**
- The fix is tk26_vision PR #8: person boxes use the YOLO-seg person mask instead of SAM, and boxes with no matching YOLO person are dropped.
- Sim check `t2-sec4-person-20261011` (tk25_decision `gpsr_runs/bench/`):
  - The BT approached the living-room person at (−3.66, −3.94), 0.14 m from the spawned (−3.8, −3.95), and SUCCEEDED.
  - The count trial counted 1 of 1.
  - Every box was `yolo_person`.
- Known gap: in a downward-looking head frame, YOLO-seg can label the robot's own arm as a person.

**Symptom.** Deliver-to-person trials in the living room (batches `t2-20-live-rerun57`, `59`, `64`, `68`, `79`, `92`):
- The person is spawned at (−3.8, −3.95) (`spawned.json`: `cmd_person_living_room`, pose `pointing_left`).
- The BT's picked detection drifts across retries: (−3.7, −4.3), (−4.0, −4.4), then (−4.23…−4.43, −5.16…−5.32), inside or against the south-west walls.
- One session picked (−2.89, −5.80), outside the arena beyond the south wall (y −5.5…−5.35).
- This drives 2c. Trials end FAIL on `delivered(bowl, person_pointing_left)`. FAIL is not caused by the abort alone: later retries ended 0.42–1.55 m from the person, and in rerun57 the delivery check passed.

**Not yet investigated.** Why the 3D person position lands about 1.3 m behind the real person. Candidates: depth sampling hitting the wall behind the person, camera-to-map TF timing, head pan/tilt state. Start from `orchestrator.log` "Picked detection at" lines in those batches and the matching head-camera frames under `runs/<entry>/frames/head/`.

**Fix (downstream guard, independent of the vision fix).** Reject or re-project person positions that fall inside static-map obstacles or outside the arena before approaching.

**Done when.** The picked position is within about 0.5 m of the spawned person in these scenarios.
