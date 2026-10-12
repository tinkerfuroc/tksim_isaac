# GPSR full battery: collision-ahead loops and barrier abort (2026-10-11)

Source: the full GPSR battery of 2026-10-10.

- Battery directory: `~/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t2-full-20261010/`. See `RESULTS.md` there.
- Stack logs: `~/tinker-sim/6.0.1/gpsr_stack_logs_d217/20261010T102014/`. Nav2 is in `02-bridge.log`, the approach planner in `05-nav_extras-0.log`, and the generalist detector in `03-vision-0.log`.
- Code under test:
  - tinker-sim `c7359f2`, with nav 1a/1b and the surface-map AMCL; the alphas commit 37931d5 was not in the stack
  - tk26_navigation `1a9ce0a`
  - tk25_decision `b625af6`
  - sim footprint `tinker2_sim` `[[0.20,0.27],[0.20,-0.27],[-0.45,-0.27],[-0.45,0.27]]`

Timestamps below are epoch seconds minus 1791640000.

---

## Bug A: RPP "collision ahead" loops at stand poses next to furniture

**Owners:** Nav2 config (debug-gpsr-nav-issue) for the controller and recovery side; approach planner (gpsr-issue-2-approach-planner) for stand-pose selection; decision (a new GPSR session, to be started by the user) for the fixed furniture nav poses.

### Symptom

Short goals that start or end next to furniture throw `RegulatedPurePursuitController detected collision ahead!` many times in a row. The pattern repeats: the controller aborts once `failure_tolerance` (1.0 s at 10 Hz, about 10 warnings per abort) runs out, the Nav2 BT recovers (clear costmaps, spin 1.57, wait 5, back up 0.30), and the robot retries.

Nav2 goals with collision-ahead warnings in the battery (from `02-bridge.log`):

| Time | From → to | Duration | End | collision-ahead |
|---|---|---|---|---|
| 10:39:37 | (−0.56, −4.28) → (−1.99, −2.00) | 63 s | succeeded | 74 |
| 11:27:50 | (3.37, −2.13) → (3.26, −3.85) | 30 s | canceled (approach aborted) | 40 |
| 11:28:22 | (3.85, −3.60) → (3.37, −2.17) | 51 s | succeeded | 78 |
| 11:44:29 | (3.49, −2.28) → (3.34, −3.85) | 38 s | succeeded | 4 |
| 11:53:33 | (1.04, 1.91) → (1.17, 1.75) | **120.5 s** | **canceled** by the decision BT's 120 s goto timeout | **98** |
| 11:55:34 | (0.99, 1.87) → (1.17, 1.75) | 18 s | succeeded (third try) | 34 |

The 11:53 row is a 0.2 m move that could not be completed in 120 s. In the last trial, that cost the trip to side_table_02 about 260 s, and the trial timed out.

**Correction (2026-10-11, found by gpsr-bug-a-bug-b): the 11:53 retry only existed because the decision cancelled a robot that was still moving.**
- The goal before it, (−2, −2) → (1.17, 1.75), had zero controller warnings, and the robot was still moving in the arena frames.
- The decision's `BtNode_GotoAction` cancelled it at 120.5 s mid-approach, at (1.04, 1.91), 0.45 m from the table and facing away.
- The cause: the 120 s timeout is measured in **wall-clock** time (`time.monotonic`, `TemplateNodes/Navigation.py`), and the sim ran at RTF about 0.3 (about 40 sim-s per 127 wall-s). So 120 s of wall time is about 36 s of sim time.
- Three goals in the battery were cancelled this way at about 120 s, and every bedroom trip took 110–120 s of wall time.
- The pivot and collision-ahead mechanism below is still what made the *retry* loop. The trigger was the premature cancel.
- Fix (decision): budget the goto on Nav2's feedback `navigation_time` (sim time), plus a wall-clock no-feedback stall check and a wall backstop.

Long goals across open floor had none.

### Root cause (strongly supported; the rotation itself was not observed because cmd_vel was not recorded)

1. **The footprint is very asymmetric around the rotation centre.** `base_link` sits 0.20 m from the front and 0.45 m from the rear, so the circumscribed radius is √(0.45² + 0.27²) = **0.525 m**. Any obstacle closer than about 0.53 m blocks some in-place headings.

2. **The stuck poses are all 0.40–0.60 m from furniture.** Output of `docs/issue_reports/2026-10-08-gpsr-nav-tools/costmap_check.py` (static map, local inflation 0.45, scaling 8):

   | Pose | Nearest map obstacle | SIM footprint: headings (of 72) that hit lethal / inscribed |
   |---|---|---|
   | (1.04, 1.91) next to side_table_02 | 0.45 m | 21 / 65 |
   | (0.99, 1.87) | 0.40 m | 25 / 72 |
   | (1.17, 1.75) side_table_02 nav pose | 0.60 m | 0 / 47 |
   | (3.37, −2.13) kitchen_table nav pose | 0.50 m | 19 / 63 |
   | (3.31, −2.29) approach pose, 0.05 m fallback | 0.45 m | 35 / 72 |
   | (3.26, −3.85) approach pose, 0.05 m fallback | 0.45 m | 30 / 72 |
   | (3.85, −3.60) | 0.49 m | 31 / 72 |
   | (−0.56, −4.28) | 0.55 m | 10 / 68 |

   The kitchen_table also has four chairs around it, which the lidar sees (image below), so the live costmap is tighter than this static check.

3. **Near a goal, RPP's only manoeuvres are in-place rotations**, and those are collision-checked:
   - `use_rotate_to_heading` with `rotate_to_heading_min_angle` 0.785 pivots when the path heading is more than 45° off. On a 0.1–0.3 m goal, the direction to the goal is arbitrary relative to the robot's heading.
   - It also pivots to the final goal yaw.
   - In Humble, `isCollisionImminent()` first checks the footprint at the current pose, then projects the rotation for `max_allowed_time_to_collision_up_to_carrot` (default 1.0 s, i.e. 0.6 rad at 0.6 rad/s) against lethal cells in the local costmap (voxel layer from `/scan`).
   - The rear overhang sweeps into the furniture or chairs, so every control cycle throws "collision ahead".

   Example: at 11:53 the robot was at (1.04, 1.91), facing roughly west toward the table (truth: final yaw −169°). The goal (1.17, 1.75) lies at −51°, a 118° heading error, so RPP must pivot with the 0.45 m rear sweeping at 0.40–0.45 m clearance. It never moved the last 0.2 m.

4. **Recovery doesn't break the loop.** Clearing the local costmap lets RPP move until the scan re-marks the obstacle. `Spin` runs into the same rotation problem (the behaviour server's `simulate_ahead_time` is 2.0 s). `BackUp` 0.30 m at 0.05 m/s does help, but it is the fourth action in the round-robin, and `allow_reversing` is false in RPP.

5. **Who picks the poses:**
   - The approach planner checks only the footprint at the final yaw (the 0.15 m preferred / 0.05 m fallback clearance). It does not check whether the robot can turn there, or turn from its arrival heading into the final yaw. Both kitchen_table approach poses were 0.05 m-fallback poses with 30–35 of 72 headings blocked.
   - The decision's fixed nav poses (kitchen_table about (3.41, −2.24), side_table_02 x = 1.172) are also inside 0.53 m of furniture or chairs.

### Live diagnosis and fix (2026-10-11, debug-gpsr-nav-issue, tinker-sim `d30439d`)

This section supersedes parts of the root cause above. It comes from a live repro with truth, AMCL and `/cmd_vel` recorded at 10 Hz.

- **Main driver: a near-goal pivot limit cycle. The collision checks are a symptom of it.**
  1. RPP stops translating as soon as the AMCL estimate is inside `xy_goal_tolerance`, so the robot parks at the boundary (0.195 m measured against 0.2).
  2. It then pivots to the goal yaw. The true base_link stayed fixed to 1 mm, but each AMCL update during the pivot (`update_min_a` 0.1 rad) moved the estimate 1–1.5 cm with heading. That put the estimate back outside the tolerance.
  3. Outside the tolerance, RPP pivots about 140° to the path heading. Back inside, it pivots back again.
  4. On the 0.25 m side_table_02 goal (1.02, 2.12) → (1.17, 1.75, 180°), the robot rotated 446°, 697°, 562° and 199° in four runs, and three of the four looped. The rotation sweep next to the table produces the "collision ahead" warnings.
  5. The 11:53 battery case (98 warnings) is this loop: the robot sat 0.206 m from the goal against a 0.2 tolerance.
- **Fix `d30439d`: AMCL `update_min_a` 0.1 → 3.0** in `nav_params_overlay.py`, plus a test.
  - This is safe in the sim because odometry is exact in pure rotation, and 5 cm of travel still triggers updates.
  - On the same repro: 208° and 192° of rotation, 8.4 s and 7.3 s, no loop (2 of 2).
  - The remaining 19–34 warnings per goal come from the one necessary pivot.
  - Open: small N, and the AMCL route-accuracy dataset has not been re-run with this setting. Check it in the next battery.
- **Corrections to points 3–4 and the table:**
  - The BT recovery round-robin barely ran in the battery. The whole log has 5 `Controller patience exceeded`, 1 `Failed to make progress`, 1 spin and 0 backups. "Recovery doesn't break the loop" (point 4) was inferred and is not supported by the log.
  - The 11:51 goal (−1.99, −2.00) → (1.17, 1.75) was cancelled at 120 s with zero collision warnings. This is the wall-clock goto timeout described in the correction under Symptom, not Nav2.
- **Footprint:** the measured body is x [−0.38, 0.13], y ±0.205, radius 0.43, against the footprint's 0.525.
  - A static check with the body plus 3 cm cuts blocked headings at the battery poses: kitchen (3.37, −2.13) 19 → 6, (3.85, −3.60) 31 → 8, (−0.56, −4.28) 10 → 0.
  - The live test was inconclusive because `set_parameters` on the local_costmap footprint timed out.
  - Not done: this needs the tinker2_sim profile in tk25_basic (`feat/tinker2-sim-profile`), and merging that is the user's call.
- **Approach planner footprint: withdrawn, not a problem.** The startup line `[(0.25,±0.25),(−0.3,±0.25)]` is only the YAML fallback, which is the hardware footprint. With `use_published_footprint: true`, the planner switches to `/global_costmap/published_footprint` about 2.4 s after startup and logs `adopted live footprint ... [(0.21, ±0.28), (−0.46, ±0.28)]`. That is tinker2_sim plus about 1 cm of costmap padding, adopted long before the first goal in both the d217 battery and the d219 run (verified in `05-nav_extras-0.log` and `04-nav_extras-0.log`).
- **Config drift:** HEAD has `xy_goal_tolerance` 0.3, but the working tree carries an uncommitted 0.2 from another session, and stacks run with 0.2. `d30439d` doesn't depend on either value.

### Live check of the approach-planner fix (2026-10-11, tk26_navigation PR #11, merged as `f82e397` and installed in `~/tk25_ws/install`)

Run by gpsr-issue-2-approach-planner. Setup: GPU 0, domain 219, mock manipulation, the PR #11 overlay build, and the dev tree including `d30439d` (the bridge is a develop install, so the overlay is loaded from source). Logs: `gpsr_stack_logs_d219/20261011T094019/` (planner `04-nav_extras-0-pr11.log`, Nav2 `02-bridge.log`).

| Trial | Target | Chosen stand pose | Outcome |
|---|---|---|---|
| K1 | kitchen_table (2.83, −3.11) | (3.44, −2.32), turn ok, 39/72 free, gap 0.26 m, clr 0.15 | SUCCEEDED attempt 1, 1.10 m from target |
| K2 | kitchen_table (2.70, −3.15) | (3.31, −2.36), turn ok, 29/72 free, gap 0.37 m, clr 0.05 | SUCCEEDED attempt 1, 1.19 m |
| K2b | same, after a full nav leg from side_table_02 | same | SUCCEEDED attempt 1, 1.17 m |
| S1 | side_table_02 (0.48, 1.65) | (1.08, 1.73), turn ok, 59/72 free | SUCCEEDED attempt 1, 0.60 m |
| S2 | same | (1.08, 1.70), turn ok, 49/72 free | SUCCEEDED attempt 1, 0.59 m |

- **Collision-ahead warnings:** 0 inside any GoToApproach window. There were 121 on the plain NavigateToPose legs to the decision's fixed nav poses: S1 34, K2b (side table → kitchen) 85, S2 2.
- **Remaining issue 1, corrected (debug-gpsr-nav-issue, log analysis):** the 85 K2b warnings came from one blocked *departure* pivot at side_table_02, not from the kitchen pose or a leftover limit cycle.
  - All 85 fall in the leg's first ~40 s and end with `Controller patience exceeded` plus a local costmap clear. The arrival at the kitchen_table had 0 warnings.
  - Start pose (1.06, 1.75), facing the table, 0.50 m away: 22/72 headings are blocked (lethal arcs 5–55° and 305–355°).
  - RPP pivots counter-clockwise toward about 285°, the point within 45° of the path heading. Its 1.0 s rotation projection (about 34°) reaches the lethal arc from about 271° on, so it deadlocks until the patience abort.
  - The costmap clear then wipes the table cells behind the robot, which the front-only lidar cannot re-mark, and the pivot completes.
  - Physically it was safe: body radius 0.43 m against 0.50 m of clearance.
  - Levers:
    - Trim the footprint to the body + 3 cm: 10/72 blocked and the projection clears. Body + 2 cm: 0/72. Needs the tk25_basic tinker2_sim profile.
    - Or lower `max_allowed_time_to_collision_up_to_carrot`. Humble shares that value with forward motion.
- **Remaining issue 2:** with the chairs, every kitchen_table stand pose sits at the 1.0 m stand-off maximum (1.10–1.19 m from the target, vs about 0.95 m before the PR). Manipulation should check whether that is within grasp reach.
- **Misleading warning:** "no usable stand pose with 0.15 m clearance" in K2 actually means a 0.15 m candidate existed but failed TURN. Reworded in PR #11 `054d8a7` to "chosen pose (x, y) has only 0.05 m footprint clearance (preferred 0.15 m): …"; grep for "has only" in future logs.

### Related: wheels kept driving after `/cmd_vel` went to zero (sim base drive, open)

- **What happened:** in about 1 of 4 route runs (kitchen approach near (3.3, 0.15)), RPP correctly detected a collision ahead and `/cmd_vel` went to 0. The wheels (`/isaac_joint_states` → odom) kept 0.35 m/s for 3.4 sim-s, and the robot drove into furniture: centre 0.21 m from the obstacle, body radius 0.43.
- **Effects:** truth spun +70°, and AMCL lost 0.6 m and 60°. The BT spin and backup recoveries then failed with "Collision Ahead".
- **Probe so far:** a per-layer probe (`cmd_vel` → facade wheel command → `/isaac_joint_commands` → measured) over two later route runs saw no recurrence.
- **Logs:** `gpsr_stack_logs/20261011T081535`, and `/tmp/claude-1002/bugA/base.jsonl` at sim t+64 s.
- **Owner:** stop-command-wheels-sim.
- **Root cause (stop-command-wheels-sim, 2026-10-11):** the zero `/cmd_vel` never reached the wheels.
  - The wheels froze at v = 0.350, wz = +0.145 from sim t+62.6, about 1.7 s *before* the zero `/cmd_vel`, right after a ~1 wall-s sim stall.
  - The stall overflowed the depth-50 `/isaac_joint_commands` reader. The Isaac gateway's strict snapshot contiguity then rejected every later packet.
  - The backend held its last wheel velocity until the 5 s command-stream timeout fired (`command_stream_lost` in `01-sim.log`, wall_age 19.47 s). That matches odom dropping to 0 at t+67.60.
  - Separately, the wheel slew wrote its slewed value back into the command buffer, so a zero command braked by only one slew step per packet.
- **Fix:** branch `fix/wheel-overrun-command-gap` (`b284a7e`, not merged).
  - A forward snapshot gap now stops the robot immediately; it recovers on the next 0.25 s safety heartbeat.
  - The slew no longer overwrites the command buffer.
  - 7 new unit tests. Not yet re-run live.

### Suggested fixes (pick per owner)

- **Approach planner:**
  - Rank, or require, *turnable* candidates: footprint clear over the arc from the expected arrival heading to the final yaw, or at least enough free headings at that cell.
  - Or prefer candidates with circumscribed-radius clearance and use the 0.05 m fallback only when none exist.
  - Add the arrival-direction check to the reachability probe.
- **Nav2 config (sim overlay):**
  - Make the goal yaw optional where the caller does not need it (the yaw-free approach for `yaw_mode` NaN).
  - Raise `rotate_to_heading_min_angle` so short goals curve or creep instead of pivoting.
  - Consider reordering the recovery round-robin so `BackUp` comes before `Spin` near obstacles.
  - Or lower `max_allowed_time_to_collision_up_to_carrot` for rotations. Humble has one value; check its side effects on forward motion.
  - Optionally trim the footprint rear from −0.45 to the measured body −0.38 plus a smaller margin. Each 1 cm of rear cuts the sweep radius.
- **Decision:** move the kitchen_table and side_table_02 nav poses out to at least 0.55–0.60 m from furniture and chairs, measured from the centre with this footprint. Or give those goals a free final yaw.

### How to reproduce or check

- Run `python3 docs/issue_reports/2026-10-08-gpsr-nav-tools/costmap_check.py --pose=1.04,1.91,180 --pose=3.31,-2.29,-120 --inflation 0.45 --scaling 8`.
- Live: send NavigateToPose from (1.04, 1.91, yaw −169°) to (1.17, 1.75) and count `detected collision ahead` in `02-bridge.log`. Record `/cmd_vel` and `/local_costmap/costmap` to confirm the pivot.

---

## Bug B: approach "barrier seen" abort caused by a mis-estimated target (wrong object, z 0.26 m too low)

**Owners:** approach planner barrier rule (gpsr-issue-2-approach-planner). Root cause of the low z still open (sim/perception). Vision logging and frame freshness (section-4 session / tk26_vision). Decision: settle wait and the surface-height sanity check (a new GPSR session, to be started by the user).

### Symptom

In trial s70718-004 ("navigate to the kitchen_table then look for a food and grasp it and deliver it to me"), the grasp drive-up GoToApproach aborted with status 12 (`05-nav_extras-0.log`, lines 20–45):

```
6069.5 GoToApproach start: target=(2.83, -3.11, z=0.46) ... target_is_person=False
6069.6 nav_attempt_1: no usable stand pose with 0.15 m footprint clearance; chosen pose (3.31, -2.29) has 0.05 m
6069.7 corrective drive +0.17 failed: aborted by behavior server (status 6)      <- "Collision Ahead - Exiting DriveOnHeading" (Bug A)
6070.7 access check: checkable=True blocked=True n_points=85 examined=[0.37,0.87]m min_height=0.56m along=[0.65,0.85]m nearest_to_target=0.30m max_height=0.64m
6070.7 nav_attempt_1 arrived but the access check found a barrier ...; snapshotting costmap   (snapshot 1188 s old, N1)
6070.9 nav_attempt_2: chosen pose (3.26, -3.85) has 0.05 m
6101.3 approach check: barrier seen ... (14 pts, along=[0.32,0.53]m nearest_to_target=0.33m max_height=0.64m, confirmed over 2 consecutive fresh evaluation(s))
6101.4 ABORTED status=12 phase=nav_attempt_2 msg='barrier seen between the planned stand pose and the target while approaching (second attempt); giving up'
```

### Root cause

1. **The target came from a bad detection.**
   - `03-vision-0.log` 6068.7: YOLO-World returned 1 box (conf **0.236**) for the six-food prompt, giving `nearest at (2.83, -3.11, 0.46)`.
   - The overlay `vision_log/20260822_081720/generalist_detection_node_yolo_world_overlay_20261010_112748_734.jpg` shows the box on the **orange mug**, which is not a food.
   - Truth: mug at (2.70, −3.15, **0.72**). The estimate is 0.14 m off in xy and **0.26 m low in z**: below the 0.734 m table top, which is physically impossible for an object resting on it.
   - All three detections discussed here (mug 6068.5 and 7065.2, bowl 7786.1) used `camera=orbbec` (the head) with `depth_source=native`. (Corrected 2026-10-11: an earlier version blamed the RealSense fallback; that log line belongs to a different call.)
   - The BT did get a per-class label (`['red tomato soup can']` in the orchestrator log). Only the vision-log JSON writes the whole prompt as `cls_name`, which is a logging bug in the generalist's `_log_debug` for the yolo_world branch. The section-4 vision session is fixing it.
2. **The barrier rule depends on target z.** In `barrier_check.py`, `min_height = target_z + height_margin_m` (0.10). With z = 0.46 that gives min_height = 0.56 m. The chairs around the round table (seat about 0.45 m, backs and edges at 0.56–0.64 m in the cloud: `max_height=0.64m`, 0.30–0.33 m from the target) then count as a barrier from both sides of the table.
3. **The same table with a correct z passed.**
   - Later in the same trial (`03-vision-0.log` 7065.2), a detection at (2.67, −3.18, **0.70**) gave min_height 0.80.
   - The GoToApproach at 7069.3 logged `access check: checkable=True blocked=False` and SUCCEEDED.

### Suggested fixes

- **Approach planner (robustness):**
  - When the target is on a known surface (or `z` is below the support surface implied by the costmap), clamp `target_z` to at least the surface height before the barrier rule.
  - Or ignore barrier points lower than the surface top: here 0.56–0.64 < 0.734.
  - Or require barrier points to be higher than the target **and** farther than about 0.4 m from it.
  - Log the clamp.
- **Sim / perception: root cause of the low z still OPEN.**
  - The SAM masks sit tightly on the mug and the bowl, so this is not a mask problem.
  - Both low-z detections came right after the head settled at tilt +45°:
    - mug: z 0.46, about 10° low at 1.69 m (`s70718-004` orchestrator, around lines 495–510)
    - bowl: z 0.48 vs actual 0.60, at 1.35 m (`s20261002-002`, around line 365)
  - The good mug detection (z 0.70) came when the tilt did not settle (`tilt_err=25.04°`).
  - **Head-move stamp-latency hypothesis: refuted** by the section-4 vision session's live test (2026-10-11; GPU 1, domain 219, RTF about 0.2, robot at spawn, tilt 20 → 45):
    - Floor z in the map frame right after the move: −0.097 / −0.08 at 0–0.5 s, −0.076 from 1 s on. The transient is about 2 cm (about 0.6°) and gone within 2–3 frames.
    - The generalist's post-move error on a chair was at most about 0.2 m at 0 s, under 1 cm from 2 s, and in the opposite direction (higher and closer).
    - A settled tilt sweep from 0° to 45° gave the same floor z at every tilt, so there is no tilt-dependent render-vs-TF mismatch either.
  - **Part of the error is now explained: sim base_link height (section-4 vision session, 2026-10-11).**
    - In physics, `base_link` rests 0.0775 m above the floor. The wheel bottoms are at z = −0.0775 in `base_link` (artifact `robot.urdf`; `tk26_sim` `tinker_full.urdf.xacro` says so explicitly).
    - Odom → `base_link` has z = 0, and localization is 2D. So every map-frame z is measured from the `base_link` plane, not the floor, and every map-frame point from the head camera reads 7.75 cm too low. The measured floor read −0.075 to −0.076 at every tilt.
    - Corrected for this, the mug reads 0.54 vs 0.72 (**about 18 cm still unexplained**) and the bowl 0.56 vs 0.60 (about 4 cm left).
    - **Correction (debug-gpsr-nav-issue, 2026-10-11): the odom → `base_link` TF does not come from `base_facade.py`**, which publishes only the `/tracer/odom` message. The TF comes from robot_localization's `ekf_filter_node` (`navigation_bringup/params/ekf.yaml`: `two_d_mode: true`, `publish_tf: true`), which zeroes z whatever the input says. So "publish z = 0.0775 in base_facade" would not work.
    - **This is hardware parity, not a sim-only bug.** The real robot runs the same EKF with `two_d_mode`, and its `base_link` is also the xArm mount point on the chassis, not the floor. Its height there is not in any file. A sim-only fix would make sim and hardware diverge.
    - **Affected:** anything that compares map-frame z with floor-referenced heights.
      - Bug B object z
      - `placement.json` surface heights (kitchen_table top 0.724, …)
      - truth_evaluator and spawned-object z
      - the approach planner's barrier heights (`max_height` 0.64 m)
    - **Not affected:**
      - pointcloud_to_laserscan (livox frame)
      - AMCL (2D)
      - MoveIt (base_link-relative)
      - the voxel layer (obstacles shift 7.75 cm down; no practical effect)
    - **Options (the user's call; option a touches hardware):**
      - a) Robot-wide: add a `base_footprint` frame on the floor, joined to `base_link` at z = h (0.0775 in sim; hardware needs measuring). Point the EKF's `base_link_frame` and AMCL `base_frame_id` at `base_footprint`. Map z is then floor height on both platforms.
      - b) Consumer-side: keep the TF tree and add a per-robot h wherever map z is compared with floor heights (placement surfaces, truth evaluator, Bug B check, barrier heights).
    - **Decision and fix (2026-10-11): the user chose a floor-level child link. tinker-sim `c6cb3d6` (debug-gpsr-nav-issue).**
      - Every sim robot_description has a fixed joint `base_link` → `base_footprint` at z = −0.0775. The value is derived from the URDF's wheel collision bottoms, and all wheels must agree within 1 mm.
      - Unchanged: EKF, AMCL, Nav2, physics and the robot artifact. Map z is still measured from the `base_link` plane.
      - Verified: robot_state_publisher publishes (0, 0, −0.0775), and the livox comes out 0.2725 m above the floor.
      - Still to do:
        - Consumers that need floor height (Bug B object z, placement surfaces, truth evaluator, barrier heights) should look up `base_footprint` in the map frame instead of using a constant. None of them has been changed yet.
        - Hardware needs the same joint in tk25_basic `tinker_urdf`, with the real `base_link` height measured.
      - Follow-up (user: "derived for sim, 30 cm for the real robot"):
        - tinker-sim `8d49e0a`: `with_base_footprint` moves an existing `base_footprint` joint to the wheel-derived height, so the sim stays at −0.0775 even when the hardware xacro carries its own link.
        - tk25_basic `597bcd7` on new branch `feat/base-footprint`, not pushed: `tracer_mini_manipulator.urdf.xacro` gains `base_footprint` at xacro arg `base_footprint_z`, default −0.30, a placeholder until the real height is measured.
        - Whoever next re-renders the sim artifact will pull this massless link into `robot.usd`. The TF stays correct because of `8d49e0a`.
    - **Related latent bug (lidar slice height):** `arena_map.livox_scan_height()` slices the static map at the livox joint's 0.195 m, but the real scan plane is 0.195 + 0.0775 = 0.2725 m above the floor.
      - No collider in the rcw2026 `arena.usd` (60 prims checked) changes coverage between those heights, so the current map is unaffected.
      - The correct form is joint z + rest height, or better, the union of footprints over [sensor z, sensor z + 2.0] to match pointcloud_to_laserscan.
      - debug-gpsr-nav-issue offers to implement this in `tools/tinker_sim_deploy/arena_map.py` if the user wants it.
  - **Ruled out: the objects had moved.** The truth probe sampled the mug about 25,700 times in s70718-004, with 0.00 m displacement. It stayed at (2.70, −3.15, 0.72) throughout.
  - **Still to check under battery conditions** (live manipulation on both GPUs, arm in the "orbbec look" pose, after a corrective drive or collision):
    - whether the head's actual (rendered) pitch differs from the joint state used for TF while under load
    - base pitch or tilt in physics
    - AMCL pose at that moment
  - Reproducing it needs the battery-like setup, not the spawn test.
  - **Not recommended:** a confidence floor. The mug scored 0.236 as "red tomato soup can", but the real soup can scored 0.095 in the good frame.
- **Decision:**
  - Reject detections whose z is below the target surface height, and re-look instead of driving up to them.
  - After a pan/tilt settles, wait a couple of frames before scanning.

### How to reproduce or check

- Unit-level: run `find_barrier` on the logged numbers. Use target (2.83, −3.11, 0.46) vs (2.83, −3.11, 0.72), with points at 0.56–0.64 m about 0.3 m from the target along the corridor. It flips from blocked to clear.
- Live: kitchen_table scene from `t2-full-20261010/runs/s70718-004-goToLoc/scene-plan.json`; send GoToApproach to (2.70, −3.15, 0.46) and to (2.70, −3.15, 0.72).
