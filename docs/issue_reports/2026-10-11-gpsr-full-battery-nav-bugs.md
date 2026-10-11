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

**Owners:** approach planner barrier rule (gpsr-issue-2-approach-planner). Sim head-camera stamp timing (tinker-sim). Vision logging and frame freshness (section-4 session / tk26_vision). Decision: settle wait and the surface-height sanity check (a new GPSR session, to be started by the user).

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
- **Sim / timing (likely root cause of the low z; analysis by the section-4 vision session, 2026-10-11):**
  - The SAM masks sit tightly on the mug and the bowl in the overlays. The bad estimates are not a mask problem.
  - Both low-z detections came right after the head settled at tilt +45°:
    - mug: z 0.46, about 10° off at 1.69 m (`s70718-004` orchestrator, around lines 495–510)
    - bowl: z 0.48 vs 0.612, about 8.5° off at 1.35 m (`s20261002-002`, around line 365)
  - The good mug detection (z 0.70) came when the tilt did *not* settle (`tilt_err=25.04°`), so the head never moved.
  - An older person scan (10-04 rerun57) showed the same thing: 0.47 m off right after a +45 settle, within about 0.1 m long after.
  - Hypothesis: right after a head move, the rendered head image carries a stamp later than the head pose it actually shows. camera_server looks up TF at the image stamp, gets the settled +45° pose, and deprojects the points about 10° too far down: low and far.
  - **Sim owner:** measure the render-to-stamp latency of the head colour/depth against `/isaac_joint_states` during a tilt move.
  - **Vision mitigation (after the sim check):** accept only frames stamped after the detect request (tighten `captured_after`).
  - **Not recommended:** a confidence floor. The mug scored 0.236 as "red tomato soup can", but the real soup can scored 0.095 in the good frame, so any floor that rejects the mug also loses true hits.
- **Decision:**
  - Reject detections whose z is below the target surface height, and re-look instead of driving up to them.
  - After a pan/tilt settles, wait a couple of frames before scanning.

### How to reproduce or check

- Unit-level: run `find_barrier` on the logged numbers. Use target (2.83, −3.11, 0.46) vs (2.83, −3.11, 0.72), with points at 0.56–0.64 m about 0.3 m from the target along the corridor. It flips from blocked to clear.
- Live: kitchen_table scene from `t2-full-20261010/runs/s70718-004-goToLoc/scene-plan.json`; send GoToApproach to (2.70, −3.15, 0.46) and to (2.70, −3.15, 0.72).
