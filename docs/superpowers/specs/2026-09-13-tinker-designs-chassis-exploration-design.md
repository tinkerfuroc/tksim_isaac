# tinker_designs — chassis design-space exploration in simulation

Date: 2026-09-13 (rev 2: multi-arm designs, GUI-authored URDF as source)
Status: draft for review
Scope: milestones M1 + M2 below. M3 (non-differential wheel topologies) is
explicitly out of scope but nothing here may block it.

## 1. Goal

Let the team design candidate robots (wheeled chassis, zero or more arms of
any model, sensor mounts) in a GUI, build each into a simulation-ready robot
artifact with one command, run the existing sim batteries against any
candidate, and compare candidates on a fixed set of chassis metrics — before
any hardware is committed to.

Non-goals

* Writing a GUI. URDF Studio (OpenLegged, Apache-2.0, urdf.enkeebot.com) is
  the authoring tool; the pipeline consumes what it exports.
* Visual fidelity. Chassis bodies are primitives with correct mass/inertia;
  meshes are an optional overlay.
* Mecanum / omni / tricycle kinematics (M3).
* Motion planning or the GPSR manipulation stack on a non-xArm arm. M2 gives
  joint-level control, truth and contacts for every declared arm; planner
  and gripper-facade integration is per arm model and is scheduled only when
  a candidate arm becomes the frontrunner.
* Changing the real-robot description in `tk25_ws`.

## 2. Current state (facts, with references)

* The sim is a Tinker2 simulator, not robot-agnostic. Identity is hard-coded
  in four independent places:
  * import: `tools/tinker_sim_deploy/workspace.py` canonicalizer requires
    `base_link`, `drive_joint`, `joint1..joint7`, and an arm mount at the
    literal `_ARM_MOUNT_ORIGIN = (-0.03, 0.0, 0.527)` (line 45); the manifest
    kinematics block (lines 723-727) is typed constants, and `"robot":
    "tinker2"` is asserted throughout.
  * backend: `simulation/tinker_sim_isaac/backend.py` — prim `/World/Tinker`;
    Isaac Lab actuator groups are per-name regexes (`joint[1-7]`,
    `pan_joint`/`tilt_joint`, `drive_joint`, `.*finger.*`, casters, wheels at
    lines 1484-1596); the gripper mimic mirror assumes the xArm finger chain
    (845-861, 1850-1861). The articulation joint list itself is generic
    (`joint_names` from the view, line 2263).
  * kinematics: `simulation/tinker_sim_core/base.py` is diff-drive only;
    `calibration.py:48-49` defaults wheel radius/track as literals.
  * ROS side: `command_gateway.py:80` takes `arm_joints` as a ROS parameter
    (default `joint1..7`) and `/isaac_joint_commands` is mapped by joint
    name (`ros_gateway.py:468`), so the command path is name-generic;
    `controllers.yaml` hard-codes `xarm7_traj_controller`; `base_facade.yaml`
    hard-codes wheel joint names; `nav_params_overlay.py:138` carries a
    footprint `[[0.20,0.27],…,[-0.45,0.27]]` that disagrees with the
    manifest's `[[0.15,0.25],…,[-0.35,0.25]]`.
* Consumers of the active robot read `artifacts/robot/tinker2/current.json`:
  `validation/run_sim.py:335,1106,1291`, `validation/manipulation_qualification.py:431`,
  `validation/gripper_close_probe.py:143`, `tools/arena_import.py:369`, and the
  bridge via `tinker_sim_bridge/current_artifact.py` →
  `tinker_sim_deploy.runtime.resolve_current_artifact`.
* `robot-profile.yaml` in the tinker2 artifact is a verbatim copy of the
  upstream `tinker_robot_config/robots/tinker2/robot.yaml`; nothing at
  runtime reads it today.
* Upstream `tk25_ws/src/tk26_sim/src/isaac_bringup` has the reference render
  chain: `scripts/render_for_isaac.sh` (xacro → strip `<gazebo>` → resolve
  `package://` → xmllint) and `scripts/verify_in_isaac.py`
  (`isaacsim.asset.importer.urdf` `ImportConfig` → USD), plus the
  `tinker_full.urdf.xacro` wheel rig (2 driven front wheels, 2 rear swivel
  casters) that `tinker2_ref` must reproduce.
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
URDF Studio ──export──► designs/<name>/robot.urdf (+ meshes/, design.yaml)
                                   │  (A: source)
                                   ▼
                        tools/design_import.py (B)
                                   │  render/clean → contract → Isaac import → derive → publish
                                   ▼
                        artifacts/robot/<name>/<hash>/  robot.urdf, robot.usd,
                                                        robot-profile.yaml, manifest.json,
                                                        source-lock.json  + current.json
                                   │
     TINKER_SIM_ROBOT=<name> ──► runtime resolves current.json (C: profile-driven
                                 backend actuator groups / bridge / nav)
                                   │
                        run.json + chassis_metrics.json (D) ──► tools/design_compare.py
```

### A. Design source — GUI-authored URDF plus a role sidecar

Location: `designs/<name>/` at repo root, one directory per candidate:

```
designs/<name>/
  robot.urdf | robot.urdf.xacro   # exported from URDF Studio (or any tool)
  meshes/                         # optional; vendor arm meshes, referenced relatively
  design.yaml                     # roles: what each joint/link IS (below)
```

The URDF is the geometry/mass/inertia source of truth. `design.yaml` is the
only hand-maintained file, and it is short — it declares roles the pipeline
cannot infer from geometry alone:

```yaml
name: dual_arm_a
kinematics: diff_drive             # only value accepted until M3
base_frame: base_link
wheels:
  driven:        [front_left_wheel_joint, front_right_wheel_joint]
  caster_swivel: [rear_left_swivel_joint, rear_right_swivel_joint]
  caster_wheel:  [rear_left_wheel_joint, rear_right_wheel_joint]
arms:
  - name: left
    mount: left_arm_base_link      # link fixed to the chassis
    joints: [l_j1, l_j2, l_j3, l_j4, l_j5, l_j6]
    gripper: {drive: l_grip, mimics: [l_finger_l, l_finger_r]}   # or null
    drive: {stiffness: 400.0, damping: 40.0}                       # position drive gains
  - name: right
    mount: right_arm_base_link
    joints: [r_j1, r_j2, r_j3, r_j4, r_j5, r_j6]
    gripper: null
    drive: {stiffness: 400.0, damping: 40.0}
pan_tilt: {joints: [pan_joint, tilt_joint]}     # or null
sensors:
  - {type: livox_mid360, frame: livox_frame}
  - {type: head_camera,  frame: head_camera_link}
```

`design_import.py --init` writes a first `design.yaml` by naming heuristics
(`*wheel_joint`, `*swivel*`, continuous joints on ±Y under `base_link`,
serial revolute chains off a fixed mount) for the user to confirm; it never
guesses silently at import time — an unlisted movable joint is a contract
error (§5).

Authoring workflow: build the chassis from primitives in URDF Studio; import
a vendor arm URDF and attach it with the merge-URDF feature, once per arm;
place sensor frames; export. Arms without a vendor URDF are sketched as
primitives too, with reach and mass as estimates — flagged in `design.yaml`
with `estimated: true` so it shows in the comparison table.

`designs/tinker2_ref/` is the parity design: the current tinker2 URDF and a
`design.yaml` naming its existing joints. Its artifact must equal the
tinker2 artifact's canonical URDF and derived profile.

Parameter sweeps are not in M1/M2. When one is needed, a small URDF
transform tool (widen track, move a mount) applied to a GUI-authored base
design is the planned mechanism — the pipeline is unchanged because it
consumes URDF.

### B. `tools/design_import.py` — pipeline CLI

Mirrors `arena_import.py`. Stages, each a pure function with a file in/out:

1. **render/clean**: if the source is xacro, expand it under the sourced ROS
   env, then canonicalise — that canonical form, with `<gazebo>` blocks and
   `package://` URIs intact, is the published `robot.urdf` (the tinker2
   artifact keeps both, and the parity gate is byte-exact). A separate,
   transient copy for the importer has `<gazebo>` stripped and `package://`
   / relative `meshes/` paths resolved to `file://` (via `AMENT_PREFIX_PATH`
   share directories). Ported from `render_for_isaac.sh`, whose strip and
   resolve stages likewise only ever fed Isaac.
2. **contract**: structural check (§5) plus inertia sanity: every link that
   declares `<inertial>` has mass > 0 and a positive-definite inertia tensor
   (massless frame links are allowed; the Isaac importer ghosts them, see
   the developer log); every wheel/caster ground contact is at the same z;
   the footprint polygon is simple and contains
   the projected CoG. The CoG checked here is the arms-at-zero `cog_base_link`;
   `cog_arms_extended` is not gated at import time -- it feeds M2's
   `tipover_margin_m` tip-over metric instead.
3. **import**: headless `SimulationApp`, Isaac Sim 6.0.1's
   `isaacsim.asset.importer.urdf` `URDFImporter`/`URDFImporterConfig` (the
   old `_urdf.ImportConfig` binding is removed on this version) with
   `fix_base=False`, `merge_fixed_joints=False`; the bundled
   `urdf_usd_converter` 0.1.3 assigns convex-hull approximation to every
   mesh collider unconditionally (`_impl/geometry.py`), with no config knob
   for anything else on this importer version; massless fixed-joint "ghost"
   links (mount/sensor frames with no inertial/visual/collision) are kept as
   plain `Xform` prims, not `RigidBodyAPI`/`Joint` prims; export USD. Behind
   a `ConverterHooks` protocol; a stub hook writes a marker file so
   everything else tests on system Python.
4. **derive** (`tinker_designs/derive.py`): from the clean URDF + roles
   compute the profile — wheel radius from the driven-wheel cylinder, track
   from the driven-wheel origins, footprint as the convex hull of every
   primitive (box/cylinder) collision under `base_frame`'s fixed subtree plus
   the wheel envelopes, projected to the ground plane, total mass and CoG,
   per-arm reach (max distance from mount to last link origin over the joint
   limits' corner set) and CoG with every arm at its reach pose. Nothing in
   the profile is typed by hand except `design.yaml` roles and drive gains,
   with one exception: a chassis whose collision is a mesh (tinker2's is)
   has no primitive to derive a footprint from, so `design.yaml` may carry a
   `footprint:` override; deriving is an error when neither exists.

   The published `robot.urdf` keeps `package://` mesh URIs exactly as the
   tinker2 artifact does; the `file://`-resolved copy is a transient input
   to stage 3 only. This is what lets the `tinker2_ref` parity test be
   byte-exact.
5. **publish**: content-addressed `artifacts/robot/<name>/<hash>/` with the
   tinker2 file set (`robot.urdf`, `robot.usd`, `robot-profile.yaml`,
   `manifest.json`, `source-lock.json`, plus `meshes/` when present) and
   `current.json`, through the existing publisher in `workspace.py`
   refactored so `robot` is a parameter instead of the literal `"tinker2"`.
   `source-lock.json` hashes every file under `designs/<name>/` and every
   upstream file `package://` resolved to. `map.yaml`/`map.pgm` are not
   robot properties; the tinker2 importer keeps copying them for backward
   compatibility only.

CLI: `./.venv/bin/python tools/design_import.py --design designs/<name>
[--init] [--no-import] [--stub-converter]`. `--no-import` stops after
stage 2 for the fast edit loop. Exit codes distinguish render / contract /
import / publish failures.

### C. Robot selection in the runtime (M2)

* `TINKER_SIM_ROBOT` (default `tinker2`) names the artifact family.
  `tinker_sim_deploy.runtime.resolve_current_artifact(root, robot=...)` reads
  `artifacts/robot/<robot>/current.json`; the call sites in §2 go through
  it. `/sim/status/isaac` and the run report carry robot name + artifact id.
* **Robot profile** (`robot-profile.yaml`, generated by stage 4) is the one
  place the sim learns joint names and geometry. It is `design.yaml`'s roles
  plus derived values:

  ```yaml
  robot: dual_arm_a
  kinematics: diff_drive
  base_frame: base_link
  wheels: {driven: […], caster_swivel: […], caster_wheel: […],
           radius_m: 0.0525, track_m: 0.30}
  arms:  [{name: left, mount: …, joints: […], gripper: {…}|null,
           drive: {stiffness, damping}, reach_m: 0.86, estimated: false}, …]
  pan_tilt: {joints: […]} | null
  footprint: [[x, y], …]
  mass_kg: 71.4
  cog_base_link: [x, y, z]
  cog_arms_extended: [x, y, z]
  ```

  For tinker2 the profile is generated by the tinker2 importer from
  `designs/tinker2_ref/design.yaml` and today's URDF, and an equality test
  pins every constant the runtime uses today (wheel names, radius, track,
  `joint1..7`, `drive_joint` + finger mimics). The upstream `robot.yaml`
  content is carried under an `upstream:` key.
* Consumers:
  * `backend.py`: Isaac Lab actuator groups are built from the profile —
    one velocity group for driven wheels, one passive group for casters,
    one position group per arm with that arm's gains, one for `pan_tilt`,
    one per gripper drive, one for gripper mimics. Today's literal groups
    become the tinker2 defaults used only when no profile is present. The
    xArm mimic mirror runs only for arms whose `gripper.mimics` is
    non-empty. The prim path stays `/World/Tinker`.
  * `calibration.py`: `development_default()` takes wheel radius/track from
    the profile; a calibration file still overrides.
  * bridge: `command_gateway` `arm_joints` = union of all arm joints plus
    gripper drives; `controllers.yaml` is generated — one
    `JointTrajectoryController` per arm named `<arm>_traj_controller`
    (`xarm7_traj_controller` is what tinker2's profile generates, so nothing
    downstream renames); `base_facade` wheel names from the profile.
  * `nav_params_overlay.py`: footprint and inscribed radius from the
    profile; the literal at line 138 is deleted (and the tinker2 footprint
    inconsistency with it).
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
| `tipover_margin_m` | min over the run of distance from the truth-pose CoG projection to the support polygon edge, using `cog_base_link` (arms stowed) or `cog_arms_extended` when any arm is beyond its stow pose |
| `rtf_mean` | sim clock vs wall clock |

All inputs already exist (`/sim/truth/*`, arena map artifact, Nav2 action
result, joint states); the recorder adds no new sim probes.
`tools/design_compare.py` prints a table from N `run.json` files, one row per
(robot, scenario), with `estimated` arms marked.

An "arms extended" scenario (scripted joint trajectory to each arm's
declared extended pose while driving the nav smoke) is added to the battery
so `tipover_margin_m` is exercised on every design.

## 4. Data flow of a candidate, end to end

1. Build chassis + attach arms + place sensors in URDF Studio; export to
   `designs/<name>/robot.urdf` (+ `meshes/`).
2. `design_import.py --design designs/<name> --init` → confirm `design.yaml`.
3. `design_import.py … --no-import` until clean + contract pass (seconds).
4. `design_import.py …` full → artifact + `current.json` (one Kit boot).
5. `TINKER_SIM_ROBOT=<name> run_sim.py …` with an existing battery plus the
   arms-extended scenario.
6. `design_compare.py runs/*/run.json` → decision table.

## 5. Structural contract (what a design must satisfy in M1/M2)

Implemented once, in `tinker_designs/contract.py`, as a role-driven check.
The tinker2 canonicalizer in `workspace.py` is a different thing — an
xArm-specific *canonical form* step (ros2_control munging, `world` link
insertion, `drive_joint` provider rules) — and stays; its only change is
that the arm-mount origin literal becomes a `mount_origin` parameter. The
tinker2 export additionally runs the generic contract with
`designs/tinker2_ref/design.yaml`, so tinker2 is checked by the same rules
as every candidate. Rules:

* root link is `base_frame`; every arm's `mount` link is connected to
  `base_frame` by fixed joints only;
* every movable joint in the URDF is claimed by exactly one role in
  `design.yaml` (driven wheel, caster swivel, caster wheel, an arm's
  `joints`, a gripper drive or mimic, pan_tilt); unclaimed movable joints
  are an error;
* driven wheels are continuous joints on a common ±Y axis; casters are a
  continuous ±Z swivel with a continuous wheel child;
* each arm is a serial chain from `mount` through `joints`; `gripper.drive`
  is a descendant of the chain's last link; mimics carry `<mimic>` tags
  pointing at `drive`;
* `kinematics: diff_drive` — the only value accepted until M3.

M3 will lift the wheel clauses by adding profile kinematics types; nothing in
C reads wheel or arm names except through the profile, which is the seam.

## 6. Error handling

* render: xacro/xmllint failure → exit 2 with the tool's stderr; unresolved
  `package://` or mesh path is an error, not a warning (Isaac fails later
  with "Used null prim").
* contract/inertia: exit 3 listing every violation, not the first.
* import: Kit exceptions → exit 4; the stage directory is removed, no partial
  artifact, `current.json` untouched (same atomic stage+rename as tinker2).
* runtime: unknown `TINKER_SIM_ROBOT` or missing `current.json` → the launcher
  refuses to start with the path it looked for; a profile missing a required
  field is a startup error, never a fallback to tinker2 values.

## 7. Testing

* Unit (system Python, no GPU): `design.yaml` load/validate and `--init`
  heuristics on fixtures (tinker2 URDF, a two-arm primitive fixture);
  contract violations each have a failing fixture (unclaimed joint, arm not
  on a fixed mount, wheels on different axes); `derive.py` against
  hand-computed values incl. reach and extended CoG; publisher with the stub
  converter; profile parsing; `resolve_current_artifact` with two robots;
  parity — `designs/tinker2_ref` produces a canonical URDF and profile equal
  to the tinker2 artifact's.
* Runtime (existing fake-view test pattern): actuator groups built from a
  two-arm profile and from the tinker2 profile (the latter must equal
  today's literal groups); `calibration.py` default from profile; generated
  `controllers.yaml` for one and two arms; nav overlay footprint from
  profile.
* Live (one Kit boot each, on the go-ahead per the standing GPU rule):
  import `tinker2_ref`; run the physics smoke with
  `TINKER_SIM_ROBOT=tinker2_ref` and compare `chassis_metrics.json` against
  the same smoke on `tinker2` — M2 acceptance. Then a two-arm primitive
  design through the smoke + arms-extended scenario.

## 8. Milestones

* **M1 (A + B)**: verify URDF Studio runs locally; `designs/` layout,
  `design.yaml` schema + `--init`; pipeline; `tinker2_ref` parity gate;
  publisher parameterised by robot; contract shared with the canonicalizer.
  Deliverable: `design_import.py` produces `artifacts/robot/<name>/…` for
  any diff-drive design with any number of arms.
* **M2 (C + D)**: `TINKER_SIM_ROBOT`; profile-driven actuator groups, bridge
  controllers, calibration, nav footprint; metrics recorder incl. tip-over;
  arms-extended scenario; compare tool; live parity run.
* **M2.5 (scheduled per arm model, not now)**: planner + gripper facade
  integration for a frontrunner non-xArm arm.
* **M3 (out of scope)**: `Twist2D.linear_y`, per-profile wheel Jacobian in
  `BaseParityModel`, N-driven-wheel backend grouping, Nav2 holonomic flag.

## 9. Decisions taken in this spec

* New in-repo `designs/` + `tinker_designs` package rather than upstream
  `isaac_bringup` (user decision, 2026-09-13).
* Chassis body is a parametric primitive box (user decision).
* Designs are authored in a GUI (URDF Studio) and the source of truth is the
  exported URDF plus a short role sidecar — not a hand-written parametric
  file (user preference, 2026-09-13). Sweeps are a later URDF-transform
  layer.
* Arms are design components: zero or more, any model, declared in
  `design.yaml`; the xArm is just tinker2's arm (user requirement: two
  non-xArm arms possible).
* The tinker2 import path is refactored (publisher and contract
  parameterised) rather than duplicated, so there is one artifact format
  and one validator.
* Prim path `/World/Tinker` stays fixed.
* `map.*` files stay a tinker2-importer legacy, not a robot property.
* Assumption: candidate arms have vendor URDFs; if not, primitives with
  `estimated: true`.
