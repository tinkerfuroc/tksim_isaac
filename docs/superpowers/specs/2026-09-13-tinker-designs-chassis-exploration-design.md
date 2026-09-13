# tinker_designs — chassis design-space exploration in simulation

Date: 2026-09-13
Status: draft for review
Scope: milestones M1 + M2 below. M3 (non-differential wheel topologies) is
explicitly out of scope but nothing here may block it.

## 1. Goal

Let the team design candidate robot chassis (wheeled base, arm mount, sensor
mounts) as parameter sets, build each into a simulation-ready robot artifact
with one command, run the existing sim batteries against any candidate, and
compare candidates on a fixed set of chassis metrics — before any hardware
is committed to.

Non-goals

* A GUI. URDF Studio (OpenLegged, urdf.enkeebot.com) is the sketchpad; its
  output is read off by hand into a design file. No import-from-URDF-Studio
  path in M1/M2.
* Visual fidelity. Chassis bodies are primitives with correct mass/inertia;
  meshes are an optional overlay later.
* Mecanum / omni / tricycle kinematics (M3).
* Changing the real-robot description in `tk25_ws`.

## 2. Current state (facts, with references)

* The sim is a Tinker2 simulator, not robot-agnostic. Identity is hard-coded
  in four independent places:
  * import: `tools/tinker_sim_deploy/workspace.py` canonicalizer requires
    `base_link`, `drive_joint`, `joint1..joint7`, and an arm mount at the
    literal `_ARM_MOUNT_ORIGIN = (-0.03, 0.0, 0.527)` (line 45); the manifest
    kinematics block (lines 723-727) is typed constants, and `"robot":
    "tinker2"` is asserted throughout.
  * backend: `simulation/tinker_sim_isaac/backend.py` — prim `/World/Tinker`,
    wheel regexes `front_.*_wheel_joint` (drive) and `rear_.*_swivel_joint` /
    `rear_.*_wheel_joint` (casters) at lines 76/87, xArm chain and gripper
    mimic names at 845-861 / 1850-1861.
  * kinematics: `simulation/tinker_sim_core/base.py` is diff-drive only;
    `calibration.py:48-49` defaults wheel radius/track as literals.
  * ROS side: joint lists repeated in `ros2_ws/src/tinker_sim_bridge/config/
    {base_facade,command_gateway,controllers}.yaml` and the ros2_control
    xacro; `nav_params_overlay.py:138` carries a footprint
    `[[0.20,0.27],[0.20,-0.27],[-0.45,-0.27],[-0.45,0.27]]` that disagrees
    with the manifest's `[[0.15,0.25],...,[-0.35,0.25]]`.
* Consumers of the active robot read `artifacts/robot/tinker2/current.json`:
  `validation/run_sim.py:335,1106,1291`, `validation/manipulation_qualification.py:431`,
  `validation/gripper_close_probe.py:143`, `tools/arena_import.py:369`, and the
  bridge via `tinker_sim_bridge/current_artifact.py` →
  `tinker_sim_deploy.runtime.resolve_current_artifact`.
* Upstream `tk25_ws/src/tk26_sim/src/isaac_bringup` already has the full
  render chain: parametric `urdf/tinker_full.urdf.xacro` (wheel radius, track,
  axle X, caster trail, ballast as `xacro:property`), `scripts/render_for_isaac.sh`
  (xacro → strip `<gazebo>` → resolve `package://` → xmllint) and
  `scripts/verify_in_isaac.py` (`isaacsim.asset.importer.urdf` `ImportConfig`
  → USD). That chain is the reference for the in-repo pipeline; it is not
  reused directly because the user chose a new in-repo package.
* This machine: Isaac Sim 6.0.1 via `.venv/bin/python` (uv) with
  `isaacsim.asset.importer.urdf` installed; `/opt/ros/humble/bin/xacro`;
  `tinker_urdf`, `xarm_description`, `realsense2_description`,
  `orbbec_description` resolve from `tk25_ws/install`.
* Existing pattern to copy: `tools/arena_import.py` + `config/arena-import.json`
  → `artifacts/arena/<id>/<hash>/` + `current.json`, with every Kit call
  behind injectable `ConverterHooks` so the pipeline unit-tests without a GPU.

## 3. Architecture

Four units. A and B are M1; C and D are M2.

```
designs/<name>/design.json ──► tools/design_import.py ──► artifacts/robot/<name>/<hash>/
        (A: source)                 (B: pipeline)              robot.urdf, robot.usd,
                                                               robot-profile.yaml,
                                                               manifest.json, source-lock.json
                                                               + current.json
                                                                        │
     TINKER_SIM_ROBOT=<name> ──► runtime resolves current.json ◄────────┘
                                 (C: profile-driven backend/bridge/nav)
                                            │
                                 run.json + chassis_metrics.json (D)
                                            │
                                 tools/design_compare.py → table across runs
```

### A. `tinker_designs` package — design source

Location: `tinker_designs/` at repo root (Python package + xacro assets), the
same level as `simulation/` and `tools/`.

```
tinker_designs/
  __init__.py
  schema.py            # Design dataclass, load/validate design.json
  render.py            # design → xacro args → clean URDF (no Isaac)
  contract.py          # structural contract check (see §5)
  derive.py            # kinematics/footprint/CoG derived from the URDF
  xacro/
    design.urdf.xacro  # top level: chassis + wheel rig + arm + sensors
    chassis_box.xacro  # parametric box body, ballast, ground clearance
    wheel_rig_diff.xacro   # 2 driven front wheels + 2 rear swivel casters
    mounts.xacro       # arm mount, sensor mount macros (livox, head cam, mast)
designs/
  tinker2_ref/design.json   # reproduces tinker_full — the parity design
  <candidate>/design.json
```

`design.json` is the single source of truth for a candidate. All lengths in
metres, masses in kg, angles in radians, frames in `base_link`
(tracer_mini convention: +X forward, +Y left, +Z up, `base_link` at the arm
mount level).

```json
{
  "name": "wide_track_a",
  "kinematics": "diff_drive",
  "chassis": {"size": [0.55, 0.42, 0.30], "mass": 22.0, "cog_offset": [-0.05, 0, -0.10],
              "ground_clearance": 0.05},
  "wheels": {"radius": 0.0525, "width": 0.063, "mass": 1.0, "track": 0.30, "front_x": 0.0},
  "casters": {"radius": 0.030, "width": 0.025, "mass": 0.5, "rear_x": -0.30, "trail": 0.025},
  "ballast": {"mass": 20.0, "xyz": [-0.15, 0, -0.05]},
  "arm_mount": {"xyz": [-0.03, 0, 0.527], "rpy": [0, 0, 0]},
  "sensors": [
    {"type": "livox_mid360", "name": "livox", "xyz": [0.20, 0, 0.35], "rpy": [0, 0, 0]},
    {"type": "head_camera",  "name": "head",  "xyz": [0.0, 0, 1.45], "rpy": [0, 0.35, 0]}
  ]
}
```

Fixed in M1/M2 (not parameters): the arm (xArm7 + xarm_gripper + wrist D435,
included from `tinker_urdf`/`xarm_description` exactly as `tinker_full` does),
the pan-tilt head and its camera model, and joint naming (§5). The wheel-rig
xacro is a port of `tinker_full`'s wheel block with the same joint names and
axes, so the tinker2 backend drives a candidate unchanged.

### B. `tools/design_import.py` — pipeline CLI

Mirrors `arena_import.py`. Stages, each a pure function with a file in/out:

1. **render**: `xacro design.urdf.xacro <args from design.json>` under the
   sourced ROS env; strip `<gazebo>`; rewrite `package://` to `file://`
   through `ament_index_python`; xmllint. Ported from `render_for_isaac.sh`.
2. **contract**: run the structural check (§5) and inertia sanity: every link
   has mass > 0 and a positive-definite inertia tensor; every wheel/caster
   ground contact is at the same z (wheel bottom = `ground_z`); the footprint
   polygon is simple and contains the projected CoG.
3. **import**: headless `SimulationApp`, `isaacsim.asset.importer.urdf`
   `ImportConfig` with `fix_base=False`, `merge_fixed_joints=False`, mimic
   parsing ON (matches the tinker2 artifact), convex decomposition OFF for
   primitives; export USD. Behind a `ConverterHooks` protocol; a stub hook
   writes a marker file so everything else tests on system Python.
4. **derive** (`tinker_designs/derive.py`): from the clean URDF compute the
   manifest `kinematics` block — driven joint names, wheel radius from the
   wheel cylinder, track from the two driven-wheel origins, footprint as the
   convex hull of the chassis box + wheel envelopes projected to the ground
   plane, total mass and CoG. Nothing in the manifest is typed by hand.
5. **publish**: content-addressed `artifacts/robot/<name>/<hash>/` with the
   same file set as tinker2 (`robot.urdf`, `robot.usd`, `robot-profile.yaml`,
   `manifest.json`, `source-lock.json`) and `current.json`, through the
   existing publisher in `workspace.py` refactored so `robot` is a parameter
   instead of the literal `"tinker2"`. `source-lock.json` hashes
   `design.json`, every xacro in `tinker_designs/xacro/`, and the upstream
   included xacros/meshes it resolved. `map.yaml`/`map.pgm` are not robot
   properties; they stay with the arena artifact and the tinker2 importer
   keeps copying them for backward compatibility only.

`robot-profile.yaml` is generated (see §3C) — it is the file the runtime reads
in M2, so M1 already writes it.

CLI: `./.venv/bin/python tools/design_import.py --design designs/<name>/design.json
[--no-import] [--stub-converter]`. `--no-import` stops after stage 2 for the
fast xacro-edit loop. Exit codes distinguish render / contract / import /
publish failures.

### C. Robot selection in the runtime (M2)

* `TINKER_SIM_ROBOT` (default `tinker2`) names the artifact family.
  `tinker_sim_deploy.runtime.resolve_current_artifact(root, robot=...)` reads
  `artifacts/robot/<robot>/current.json`; the seven call sites in §2 go
  through it instead of the literal path. `/sim/status/isaac` and the run
  report carry the robot name and artifact id.
* **Robot profile** (`robot-profile.yaml` in the artifact) is the one place
  the sim learns joint names and geometry:

  ```yaml
  robot: wide_track_a
  kinematics: diff_drive
  base_frame: base_link
  wheels: {driven: [front_left_wheel_joint, front_right_wheel_joint],
           caster_swivel: [rear_left_swivel_joint, rear_right_swivel_joint],
           caster_wheel:  [rear_left_wheel_joint, rear_right_wheel_joint],
           radius_m: 0.0525, track_m: 0.30}
  arm:   {joints: [joint1, …, joint7], base_link: link_base, tcp_link: link_tcp}
  gripper: {drive: drive_joint, mimics: [left_finger_joint, right_finger_joint]}
  pan_tilt: {joints: [pan_joint, tilt_joint]}
  footprint: [[x, y], …]
  mass_kg: 64.2
  cog_base_link: [x, y, z]
  ```

  For tinker2 the file is generated by the tinker2 importer with today's
  values, so tinker2 behaviour is unchanged and covered by an equality test.
  (Today `robot-profile.yaml` in the tinker2 artifact is a verbatim copy of
  the upstream `tinker_robot_config/robots/tinker2/robot.yaml`; the generated
  profile replaces it and the upstream file's content, if still needed, is
  carried as a nested `upstream:` block.)
* Consumers:
  * `backend.py`: wheel/caster patterns, arm chain, gripper names come from
    the profile (module constants become the tinker2 defaults used only when
    no profile is present). The prim path stays `/World/Tinker` — it is the
    sim's handle, not a robot property.
  * `calibration.py`: `development_default()` takes `wheel_radius_m` /
    `wheel_track_m` from the profile; a calibration file still overrides.
  * bridge: `manipulation.launch.py` / the base launch pass profile joint
    lists as parameter overrides to `base_facade`, `command_gateway`,
    `controllers` — the YAMLs keep tinker2 values as defaults.
  * `nav_params_overlay.py`: footprint and inscribed radius read from the
    profile; the literal at line 138 is deleted. (This also fixes the
    existing tinker2 inconsistency — the tinker2 profile footprint is derived
    from its URDF and both sites read it.)
* `run_sim.py --robot <name>` sets the env var for the launched stack.

### D. Chassis metrics recorder (M2)

A recorder node in `validation/` (same hook as the truth evaluator, wired via
the existing `--recorder-cmd`) that writes `chassis_metrics.json` beside
`run.json` and merges a `chassis` block into it:

| metric | source |
|---|---|
| `nav_success`, `time_to_goal_s`, `path_length_m` | Nav2 result + `/sim/truth` pose |
| `max_abs_roll_rad`, `max_abs_pitch_rad` | truth base orientation |
| `wheel_lift_events` | truth contact: a driven wheel with zero normal force for > 100 ms |
| `slip_ratio_mean` | (rim speed − truth body speed) / rim speed, driven wheels |
| `min_clearance_m` | min over the run of distance from the profile footprint (at truth pose) to occupied cells of the arena map |
| `cmd_tracking_rmse` | commanded vs truth twist |
| `rtf_mean` | sim clock vs wall clock |

All inputs already exist (`/sim/truth/*`, arena map artifact, Nav2 action
result); the recorder adds no new sim probes. `tools/design_compare.py`
prints a table from N `run.json` files, one row per (robot, scenario).

## 4. Data flow of a candidate, end to end

1. Sketch in URDF Studio; read dimensions into `designs/<name>/design.json`.
2. `design_import.py --no-import` until render + contract pass (seconds).
3. `design_import.py` full → artifact + `current.json` (one Kit boot).
4. `TINKER_SIM_ROBOT=<name> run_sim.py …` with an existing battery.
5. `design_compare.py runs/*/run.json` → decision table.

## 5. Structural contract (what a design must satisfy in M1/M2)

Shared by the tinker2 canonicalizer and `tinker_designs/contract.py` — one
implementation, the tinker2 literals become defaults:

* root link `base_link`; a fixed joint `base_link → link_base` whose origin
  equals `design.arm_mount` (replaces `_ARM_MOUNT_ORIGIN`);
* arm joints exactly `joint1..joint7`; gripper `drive_joint` with the two
  finger mimics; `pan_joint`, `tilt_joint`;
* driven wheels match `front_.*_wheel_joint` (continuous, axis +Y); casters
  `rear_.*_swivel_joint` (continuous, axis +Z) with child `rear_.*_wheel_joint`;
* `kinematics: diff_drive` — the only value accepted until M3.

M3 will lift the wheel clauses by adding profile kinematics types; nothing in
C reads wheel names except through the profile, which is the seam.

## 6. Error handling

* render: xacro/xmllint failure → exit 2 with the tool's stderr; unresolved
  `package://` is an error, not a warning (Isaac fails later with
  "Used null prim").
* contract/inertia: exit 3 listing every violation, not the first.
* import: Kit exceptions → exit 4; the stage directory is removed, no partial
  artifact, `current.json` untouched (same atomic stage+rename as tinker2).
* runtime: unknown `TINKER_SIM_ROBOT` or missing `current.json` → the launcher
  refuses to start with the path it looked for; a profile missing a required
  field is a startup error, never a fallback to tinker2 values.

## 7. Testing

* Unit (system Python, no GPU): schema load/validate; render golden test —
  `designs/tinker2_ref` renders to a URDF whose canonical form equals the
  current tinker2 artifact's `robot.urdf` (the parity gate for the xacro
  port); contract violations each have a failing fixture; `derive.py`
  against hand-computed values; publisher with the stub converter; profile
  parsing; `resolve_current_artifact` with two robots.
* Runtime (existing fake-view test pattern): backend reads wheel/arm names
  from a profile; `calibration.py` default from profile; nav overlay reads
  footprint from profile; tinker2 profile produces the exact constants used
  today.
* Live (one Kit boot each, on the go-ahead per the standing GPU rule): import
  `tinker2_ref`, run the existing physics smoke with
  `TINKER_SIM_ROBOT=tinker2_ref`, compare its `chassis_metrics.json` against
  the same smoke on `tinker2` — this is the M2 acceptance. Then one
  candidate design through the same smoke.

## 8. Milestones

* **M1 (A + B)**: package, xacro port, pipeline, `tinker2_ref` parity gate,
  publisher parameterised by robot, contract shared with the canonicalizer.
  Deliverable: `design_import.py` produces `artifacts/robot/<name>/…` for
  any diff-drive design.
* **M2 (C + D)**: `TINKER_SIM_ROBOT`, profile-driven backend/bridge/nav,
  metrics recorder, compare tool, live parity run.
* **M3 (out of scope)**: `Twist2D.linear_y`, per-profile wheel Jacobian in
  `BaseParityModel`, N-driven-wheel backend grouping, Nav2 holonomic flag.

## 9. Decisions taken in this spec

* New in-repo `tinker_designs` package rather than upstream `isaac_bringup`
  (user decision, 2026-09-13).
* Chassis body is a parametric primitive box (user decision).
* The tinker2 import path is refactored (publisher and contract
  parameterised) rather than duplicated, so there is one artifact format
  and one validator.
* Prim path `/World/Tinker` stays fixed.
* `map.*` files stay a tinker2-importer legacy, not a robot property.
* The wheel rig is ported verbatim from `tinker_full` (names, axes, caster
  trail) so M1 needs no backend change.
