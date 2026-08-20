# GPSR in Simulation — Design

**Date:** 2026-08-20
**Status:** approved design, pending implementation plan
**Scope:** run `tk25_decision`'s GPSR two-layer orchestrator against the tinker-sim 6.0.1 rcw2026 arena with **real (non-mocked) vision and manipulation**.

## Context

GPSR (`tk25_decision`, branch `gpsr-two-layer-orchestrator`) is today only exercisable
in full-mock mode or on the real robot. Mock mode proves behaviour-tree shape but nothing
about perception, planning, or grasping; the real robot is scarce and slow to iterate on.
Running GPSR against the arena simulation closes that gap — provided vision and
manipulation are genuinely exercised rather than stubbed, which is the explicit
requirement here.

A survey of all three repos (2026-08-20) found the integration is far cheaper than
expected: **every ROS interface GPSR needs is already served by something real.** The
work is wiring, coordinate data, and two genuine bug fixes — not new subsystems.

## Findings that shape the design

1. **Camera parity already exists.** `simulation/sensors/hardware-parity.json` publishes
   `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info` and the
   wrist `/camera/xarm_camera/{color,aligned_depth_to_color}/*` set — byte-identical to the
   topics `tk26_vision`'s `camera_server_node` subscribes to, with a QoS block whose own
   rationale cites tk26_vision's config. Real vision needs no shim.
2. **The gripper gap is already filled.** `joint_move_action` is served by the real
   `pick_and_place` node, which supports `execution_profile ∈ {hardware, sim_ompl, sim_cumotion}`.
   `start_grasp` is served by `arm_api/grasp_action`, a pure ROS orchestrator. Only
   `/xarm_gripper/gripper_action` requires a real xArm IP — and tinker-sim's `gripper_facade`
   already serves exactly that action, which is the `/tinker_sim_gripper_facade` stand-in
   `tk25_manipulation`'s comments reference but never implement.
3. **Pan/tilt is already replaced.** The real `pan_tilt_controller` opens `/dev/ttyUSB0`
   unguarded and crashes without hardware; tinker-sim's `pan_tilt_facade` serves the same
   `/pan_tilt_controller/cmd` + `/state` interface with no serial device.
4. **A real cross-repo bug blocks two GPSR actions.** GPSR calls `feature_extraction_service`
   and `detect_waving_persons` as ROS **services** (`ServiceHandler` → `create_client`,
   `BaseBehaviors.py:238`), but tk26_vision serves both as **ActionServers**
   (`feature_recognition.py:266`, `waving_person_server.py:249`). The `wait_for_service`
   loop never returns, so `describe_person` and waving detection hang — on the real robot too.
5. **GPSR's world model is bound to a different arena.** `GPSR/constants.json` hardcodes 23
   waypoints for the RoboCup Incheon 2026 map. The planner LLM is prompted to emit only names
   from this file, so an arena remap is data, not code.
6. **Textured YCB objects are already published in this repo** (see Components §4), so real
   vision has recognizable targets without any asset work.
7. **Open-vocabulary recognition is available.** `object_detection_generalist` chains
   YOLO-World + MobileSAM + a Gemini/Qwen VLM, so object recognition is not limited to COCO
   classes — but it still needs assets that read as the object they represent.

## Architecture

Six stacks on one machine, `ROS_DOMAIN_ID=42`:

| Layer | Process | Provides to GPSR |
| --- | --- | --- |
| Sim | `run_sim.py --arena rcw2026 --spawn-xy=-2.0,-2.0`, hardware-parity cameras | real-named camera topics + QoS |
| Sim bridge | `gripper_facade`, `pan_tilt_facade`, `audio_fixtures`, `ros2_control` | `/xarm_gripper/gripper_action`, `/pan_tilt_controller/*`, `announce` |
| Nav | tinker-sim `navigation.launch.py` (Nav2 + AMCL on the derived arena map) | `navigate_to_pose`, `map`→`base_link` TF for `START_POSE` |
| tk26_navigation | `approach_planner`, `orientation_angle_service` | `go_to_approach`, `find_approach_pose`, `orientation_angle_service` |
| tk26_vision | `vision_bringup.launch.py enable_gpsr:=true` + `camera_server_node` | all five detection services (real YOLO/SAM/VLM) |
| tk25_manipulation | `manipulation_planning_task_only.launch.py execution_profile:=sim_cumotion`, `arm_api/grasp_action`, `anygrasp_ros2` | `joint_move_action`, `start_grasp` |

`mock_config.json` for sim runs: `vision=false`, `manipulation=false`, `navigation=false`
(real Nav2, not the shipped default), `announcement=false` (sim `announce`),
`audio_input=true` — audio input is the single mocked subsystem, since command intake uses
`BT_GPSR_CMD` injection and no simulated ASR exists.

## Components

### 1. Vision transport fix (`tk25_decision`)
`BtNode_FeatureExtraction` (`TemplateNodes/Vision.py:608`) and `BtNode_ScanForWavingPersonNew`
(`GPSR/custom_nodes.py:880`) change base class `ServiceHandler` → `ActionHandler`, matching
the action types tk26_vision actually serves. This fixes real-robot behaviour as well; it is
not a sim workaround.

### 2. Arena world model (`tk25_decision`)
New `GPSR/constants.rcw2026.json`, selected by `GPSR_CONSTANTS_FILE` (unset = the competition
`constants.json`), so the competition file is never overwritten. Every waypoint is verified free on the
derived arena map using `OccupancyMap.free_with_clearance(x, y, 0.35)` from
`simulation/tinker_sim_core/occupancy.py`, and each name maps to real arena furniture
(`kitchen_table`, `side_table_02`, shelves) rather than invented poses.

### 3. Composite launch (`tinker-sim`)
GPSR needs cameras, arm, and Nav2 simultaneously; today those live in separate profiles.
A `gpsr.launch.py` composes the manipulation bridge and navigation stack over one
hardware-parity sim instance.

### 4. Scenario and objects (`tinker-sim`)
A `gpsr-rcw2026` scenario places graspable objects on arena furniture, plus a `listen_action`
fixture beside `audio_fixtures` so `ask_person` does not hang.

**Object assets already exist — nothing to source or author.** A teammate's YCB importer
(`tools/ycb_import.py`, pinned allowlist `config/ycb-import.json`) has already published ten
textured, physics-ready YCB objects as a content-addressed artifact under
`artifacts/objects/ycb/d2d5ccd2.../`: cheez-it, sugar box, spam, mustard bottle, pudding box,
tomato soup can, banana, bleach cleanser, bowl, mug. Each is an `object.usd` with a texture
directory, referenced from a scenario as a repo-root-relative `asset_uri`
(resolved by `_uri(root, asset_uri)`, `simulation/tinker_sim_core/orchestration.py:59`).
Recognition rides on `object_detection_generalist`'s open-vocabulary VLM path, which suits
scanned YCB objects well: they are real photographed products, so a VLM prompted for
"tomato soup can" or "mug" has a genuine chance where a COCO classifier would not.

## Verification

First milestone is **one command end-to-end**: a single injected command
(`BT_GPSR_CMD="go to the kitchen and find a person"`) driven through split → plan → execute
against the live sim, with evidence captured per stage:

1. **Interface census** — every interface in the table above answers (`ros2 service list`,
   `ros2 action list`, one probe each). Fails fast if a stack is missing.
2. **Perception** — `object_detection_generalist` returns a non-empty detection against sim
   camera frames, with the centroid transformed into `map`.
3. **Planning** — the two-layer orchestrator's split and per-target plans are logged and
   contain only names from the arena constants file.
4. **Execution** — the robot reaches the commanded waypoint under Nav2 (pose within tolerance
   of the arena's `kitchen` waypoint) and `START_POSE` capture succeeds.
5. **Teardown** — explicit-PID, GPU verified clear.

Evidence lands under `reports/gpsr-sim-<date>/`. Broadening to the full RoboCup ten-command
set follows only once this passes.

## Out of scope

- Simulated ASR (`listen_action` is a fixture; command intake is injected).
- The GPSR mission supervisor (`GPSR_SUPERVISION_MODE`), which needs dual-camera production context.
- Any change to `tk25_decision`'s competition `constants.json`.
- Credential handling of any kind — `.env` keys are consumed as-is and never read, printed, or moved.
