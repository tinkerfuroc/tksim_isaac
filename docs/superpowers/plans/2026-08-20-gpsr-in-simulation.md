# GPSR in Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run `tk25_decision`'s GPSR two-layer orchestrator against the tinker-sim 6.0.1 rcw2026 arena with real (non-mocked) vision and manipulation, proven by one natural-language command executing end-to-end.

**Architecture:** No new subsystems. Every ROS interface GPSR needs is already served by something real — the sim publishes hardware-parity camera topics, `gripper_facade` serves `/xarm_gripper/gripper_action`, `pan_tilt_facade` replaces the serial gimbal, `audio_fixtures` serves the whole speech surface, and tk25_manipulation's `pick_and_place`/`grasp_action` run against sim via `execution_profile:=sim_cumotion`. The work is: two transport bug fixes in decision, an arena world-model file, a de-duplicating composite launch, a scenario using already-published YCB assets, and a live bring-up.

**Tech Stack:** ROS 2 Humble, Python 3.10 (tk25_ws nodes) / 3.12 (tinker-sim venv), Isaac Sim, py_trees, Nav2, MoveIt/cuMotion.

**Spec:** `docs/superpowers/specs/2026-08-20-gpsr-in-simulation-design.md`

## Global Constraints

- Vision and manipulation are **never mocked**: `mock_config.json` for sim runs sets `vision`, `manipulation`, `navigation`, `announcement`, `audio_input` all `enabled=false`.
- Credentials are never read, printed, copied, or moved. `/home/tinker/tk25_ws/.env` is consumed by the tools that already read it; no task inspects its values.
- `--sensor-profile` accepts exactly `physics-only | sensor-rich | navigation-parity | manipulation-core`. GPSR requires `sensor-rich` (the only branch that loads `simulation/sensors/hardware-parity.json`).
- `--spawn-xy` must use the `=` form: `--spawn-xy=-2.0,-2.0` (argparse rejects the space form for negative values).
- Live runs use `ROS_DOMAIN_ID=42`; launch env pattern: `set -a; source .deployment.env; set +a; unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH`.
- Never launch Isaac while another Isaac holds the GPU; verify `nvidia-smi --query-compute-apps=pid --format=csv,noheader` is empty first. Tear down by explicit PID (SIGINT, wait, then SIGKILL) — never `pkill` a pattern that can match the running shell.
- Arena artifact: `artifacts/arena/rcw2026/d2b559b43207c8d54ae2609f638dca1cc36ee8b7adc7e4d94aee86e7fb56729c/`.
- YCB object artifact: `artifacts/objects/ycb/d2d5ccd2c098b68f39737f8f0490358b7fd6cbfa8080b604851e073fc758acda/`.
- Commit messages end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Tasks 1–2 modify `/home/tinker/tk25_ws/src/tk25_decision` (branch `gpsr-two-layer-orchestrator`); Tasks 3–5 modify `/home/tinker/tinker-sim/6.0.1`. Commit in the repo you changed; never mix repos in one commit.

---

### Task 1: Convert the two vision nodes from service to action clients

GPSR calls `feature_extraction_service` and `detect_waving_persons` as ROS **services**, but tk26_vision serves both as **ActionServers**. `ServiceHandler.setup()` loops `while not self.client.wait_for_service(timeout_sec=1.0)` forever, so `describe_person` and waving detection hang. This is a real-robot bug, not a sim artifact.

**Files:**
- Modify: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/TemplateNodes/Vision.py:608-689` (`BtNode_FeatureExtraction`)
- Modify: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/custom_nodes.py:880-932` (`BtNode_ScanForWavingPersonNew`)

**Interfaces:**
- Consumes: `ActionHandler` from `TemplateNodes/ActionBase.py:98` — constructor `(name, action_type, action_name, key, wait_for_server_timeout_sec=-3.0, action_timeout_ticks=0)`; override `send_goal()` and `process_result()`.
- Produces: both classes keep their existing public constructor signatures and blackboard keys, so no call site changes.

The goal/result fields are byte-identical between the `.srv` and `.action` forms, so this is a transport change only — no field remapping.

- [ ] **Step 1: Confirm the type identity before changing anything**

```bash
cd /home/tinker/tk25_ws/src/tk26_vision/src/tinker_vision_msgs_26
diff <(sed -n '1,/^---$/p' action/FeatureExtraction.action) <(sed -n '1,/^---$/p' srv/FeatureExtraction.srv)
diff <(sed -n '1,/^---$/p' action/DetectWaving.action) <(sed -n '1,/^---$/p' srv/DetectWaving.srv)
```
Expected: no differences in the goal/request halves. If they differ, STOP and report — the conversion below assumes identical fields.

- [ ] **Step 2: Convert `BtNode_FeatureExtraction`**

Change the base class and replace `initialise`/`update` with the action lifecycle. Keep the constructor signature, blackboard keys, and `self.camera` logic exactly as they are; only the marked parts change.

```python
class BtNode_FeatureExtraction(ActionHandler):
    def __init__(self,
                 name: str,
                 bb_dest_key: str,
                 bb_image_key: str,
                 service_name: str = "feature_extraction_service",
                 use_orbbec=True,
                 ):
        # key=None: this node builds its own goal in send_goal(), it does not
        # read a goal off the blackboard.
        super(BtNode_FeatureExtraction, self).__init__(
            name, FeatureExtraction, service_name, None,
            wait_for_server_timeout_sec=-3,
        )
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.key = bb_dest_key
        self.blackboard.register_key(
            key="features",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_dest_key)
        )
        self.blackboard.register_key(
            key="comparison_image",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_image_key)
        )
        self.camera = "orbbec" if use_orbbec else "realsense"

    def send_goal(self):
        if self.mock_mode:
            from sensor_msgs.msg import Image
            self.feedback_message = "MOCK: Feature extraction completed"
            self.blackboard.features = "[mock features]"
            self.blackboard.comparison_image = Image()

            class MockFuture:
                def done(self):
                    return True

            self.send_goal_future = MockFuture()
            return
        goal = FeatureExtraction.Goal()
        goal.camera = self.camera
        self.send_goal_request(goal)
        self.feedback_message = "Sent feature extraction goal"

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = f"Feature extraction aborted: {self.result_status_string}"
            return pytree.common.Status.FAILURE
        result = self.result_message.result
        if result.status != 0:
            self.feedback_message = (
                f"Feature extraction failed with error code {result.status}: {result.error_msg}"
            )
            return pytree.common.Status.FAILURE
        self.blackboard.features = result.feature
        self.blackboard.comparison_image = result.comparison_image
        img = result.comparison_image
        self.feedback_message = f"Features: {result.feature} | image: {img.width}x{img.height}"
        return pytree.common.Status.SUCCESS
```

Add the imports this needs at the top of `Vision.py` if absent: `from behavior_tree.TemplateNodes.ActionBase import ActionHandler` and `import action_msgs.msg as action_msgs` (match how `Manipulation.py` imports them — copy those import lines verbatim rather than inventing new ones).

Delete the old `initialise()` and `update()` from this class: `ActionHandler` provides both, and its `update()` already routes mock ticks through `wait_for_keypress_in_mock()`.

- [ ] **Step 3: Convert `BtNode_ScanForWavingPersonNew`**

```python
class BtNode_ScanForWavingPersonNew(ActionHandler):
    def __init__(self,
                 name: str,
                 bb_key_all_persons: str,
                 bb_key_closest_person: str,
                 threshold_meters: float,
                 service_name: str = "detect_waving_persons",
                 target_frame: str = "map"
                 ):
        super().__init__(name, DetectWaving, service_name, None,
                         wait_for_server_timeout_sec=-3)
        self.bb_key_all_persons = bb_key_all_persons
        self.bb_key_closest_person = bb_key_closest_person
        self.threshold_meters = threshold_meters
        self.target_frame = target_frame
        self.bb_write_client = None

    def setup(self, **kwargs):
        ActionHandler.setup(self, **kwargs)
        self.bb_write_client = self.attach_blackboard_client(name="ScanForWavingPersonNew")
        self.bb_write_client.register_key(self.bb_key_all_persons, access=py_trees.common.Access.WRITE)
        self.bb_write_client.register_key(self.bb_key_closest_person, access=py_trees.common.Access.WRITE)

    def send_goal(self):
        if self.mock_mode:
            self.feedback_message = "MOCK: waving scan goal sent"

            class MockFuture:
                def done(self):
                    return True

            self.send_goal_future = MockFuture()
            return
        goal = DetectWaving.Goal()
        goal.threshold_meters = self.threshold_meters
        goal.target_frame = self.target_frame
        self.send_goal_request(goal)
        self.feedback_message = "Sent waving-person detection goal"

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = f"Waving scan aborted: {self.result_status_string}"
            return py_trees.common.Status.FAILURE
        result = self.result_message.result
        if result.status != 0:
            self.feedback_message = f"Waving scan failed with status {result.status}: {result.error_msg}"
            return py_trees.common.Status.FAILURE
        if not result.waving_persons:
            self.feedback_message = "Scan succeeded, but no waving person found."
            return py_trees.common.Status.FAILURE
        self.bb_write_client.set(self.bb_key_all_persons, result.waving_persons)
        self.bb_write_client.set(self.bb_key_closest_person, result.waving_persons[0])
        closest = result.waving_persons[0]
        self.feedback_message = (
            f"Found {len(result.waving_persons)} waving person(s). Closest at "
            f"({closest.point.x:.4f}, {closest.point.y:.4f}, {closest.point.z:.4f}) "
            f"in {closest.header.frame_id}"
        )
        return py_trees.common.Status.SUCCESS
```

Note: `goal.min_waving_persons` is deliberately left at its default. The service version never set it either; changing that default is a behaviour change outside this task's scope. Record it as a concern in your report.

- [ ] **Step 4: Verify both classes import and instantiate**

```bash
cd /home/tinker/tk25_ws && colcon build --packages-select behavior_tree --symlink-install 2>&1 | tail -5
```
Expected: `Finished <<< behavior_tree`. Then confirm the base class actually changed:
```bash
source install/setup.bash
python3 -c "
from behavior_tree.TemplateNodes.Vision import BtNode_FeatureExtraction
from behavior_tree.GPSR.custom_nodes import BtNode_ScanForWavingPersonNew
from behavior_tree.TemplateNodes.ActionBase import ActionHandler
assert issubclass(BtNode_FeatureExtraction, ActionHandler), 'FeatureExtraction not converted'
assert issubclass(BtNode_ScanForWavingPersonNew, ActionHandler), 'ScanForWaving not converted'
print('both are ActionHandler subclasses')
"
```
Expected: `both are ActionHandler subclasses`.

- [ ] **Step 5: Run the repo's own GPSR tests for regressions**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision && python3 -m pytest src/behavior_tree/test -q -k "not live" 2>&1 | tail -15
```
Record the pass/fail set. Any test that failed **before** your change must be reported as pre-existing, not fixed silently — run the same command on a clean checkout first (`git stash`, run, `git stash pop`) to get the baseline.

- [ ] **Step 6: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/TemplateNodes/Vision.py src/behavior_tree/behavior_tree/GPSR/custom_nodes.py
git commit -m "fix: call feature_extraction and detect_waving as actions, not services

tk26_vision serves both as ActionServers; the ServiceHandler clients spun
forever in wait_for_service, hanging describe_person and waving detection.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: Arena world model for GPSR

GPSR's `constants.json` hardcodes 23 waypoints for the RoboCup Incheon map. The planner LLM is prompted to emit only names from this file, so retargeting is data, not code. The competition file must stay untouched.

**Files:**
- Create: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json`
- Modify: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_full.py:45-48` (`CONSTANTS_PATH`)

**Interfaces:**
- Consumes: `OccupancyMap.free_with_clearance(x, y, clearance_m)` and `nearest_free_world(x, y, clearance_m, max_radius_m=5.0)` from `/home/tinker/tinker-sim/6.0.1/simulation/tinker_sim_core/occupancy.py`.
- Produces: env var `GPSR_CONSTANTS_PATH` selecting the constants file; unset keeps the competition default.

- [ ] **Step 1: Compute standing poses from the arena's real furniture**

The arena's seven placement surfaces are furniture *tops*; the robot must stand *near* them, not on them. This script picks, for each surface, the nearest cell with 0.35 m clearance and a yaw facing the surface centre. Run it from the tinker-sim repo:

```bash
python3 - <<'EOF'
import json, math, sys
sys.path.insert(0, "simulation")
from pathlib import Path
from tinker_sim_core.occupancy import OccupancyMap

ART = Path("artifacts/arena/rcw2026/d2b559b43207c8d54ae2609f638dca1cc36ee8b7adc7e4d94aee86e7fb56729c")
occ = OccupancyMap.from_pgm(ART / "map.pgm", resolution=0.05, origin_x=-5.05, origin_y=-6.0)
surfaces = json.loads((ART / "placement.json").read_text())["surfaces"]

out = {}
for s in surfaces:
    cx, cy, _ = s["center_xyz"]
    stand = None
    # Try progressively further standoffs on 16 headings; take the first clear one.
    for radius in (0.65, 0.75, 0.85, 1.0, 1.2):
        for i in range(16):
            a = 2 * math.pi * i / 16
            x, y = cx + radius * math.cos(a), cy + radius * math.sin(a)
            if occ.free_with_clearance(x, y, 0.35):
                stand = (round(x, 3), round(y, 3))
                break
        if stand:
            break
    if not stand:
        print(f"!! no standing pose for {s['surface_id']}", file=sys.stderr)
        continue
    yaw = math.atan2(cy - stand[1], cx - stand[0])   # face the surface
    out[s["surface_id"]] = {"stand": stand, "yaw": round(yaw, 4), "surface": [cx, cy]}
    print(f"{s['surface_id']:24s} stand={stand} yaw={yaw:+.3f}")
Path("/tmp/gpsr_arena_poses.json").write_text(json.dumps(out, indent=1))
print("\nwrote /tmp/gpsr_arena_poses.json")
EOF
```
Expected: a standing pose for every surface, none reported as failing. If any surface reports `!!`, widen the radius list and re-run; record what you changed.

- [ ] **Step 2: Author `constants.rcw2026.json`**

Use the exact schema of the competition file: `possible_poses` maps a name to `{"point": {x,y,z}, "orientation": {x,y,z,w}}`, `possible_objects` maps short name to a descriptive VLM prompt, `default_locations` maps object short name to a `possible_poses` key, `search_spots` maps a room name to an ordered list of pose names. Quaternion from yaw is `z=sin(yaw/2), w=cos(yaw/2)`, `x=y=0`.

Copy `arm_pos_navigating`, `arm_pos_scan`, `arm_pos_orbbec_look`, `arm_pos_scan_original` **verbatim** from `constants.json` — they are joint angles, not map data, and must not change.

Name the poses after the arena's furniture so the planner's vocabulary matches reality: `kitchen_table`, `laundry_desk`, `shelf`, `shelf_02`, `side_table`, `side_table_02`, plus the seventh surface's name as it appears in `placement.json`. Add `command_point` at the verified-free robot spawn `(-2.0, -2.0)` with yaw `0.0`.

`possible_objects` must name the YCB objects actually spawnable in the arena, with prompts a VLM can match:

```json
"possible_objects": {
    "soup": "red tomato soup can",
    "mug": "red ceramic mug",
    "banana": "yellow banana",
    "mustard": "yellow mustard bottle",
    "sugar_box": "yellow sugar box",
    "spam": "blue rectangular spam can",
    "cheez_it": "red cracker box",
    "pudding_box": "brown pudding box",
    "bowl": "red bowl",
    "bleach": "white bleach cleanser bottle"
}
```

`default_locations` maps each of those to a pose name (e.g. `"soup": "kitchen_table"`). `search_spots` gives each room an ordered sweep list of pose names. Every value in `default_locations` and `search_spots` MUST be a key of `possible_poses` — `load_knowledge_from_constants` will silently produce an unreachable plan otherwise.

- [ ] **Step 3: Add the env-var hook**

In `gpsr_full.py`, replace the hardcoded constant (currently lines 45-48):

```python
import os

CONSTANTS_PATH = os.environ.get(
    "GPSR_CONSTANTS_PATH",
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/"
    "behavior_tree/GPSR/constants.json",
)
```

Add `import os` at the top if it is not already imported. This is the single canonical definition — `gpsr_orchestrator.py:102` and six other call sites import it from here. Note in your report that `codegen.py` computes its own path from `__file__` and four legacy scripts (`gpsr_new.py`, `gpsr_secondcall.py`, `gpsr_2ndcall.py`, `egpsr.py`) hardcode the path, so they are unaffected by this override — that is acceptable because the two-layer orchestrator is the only entry point this plan uses.

- [ ] **Step 4: Verify both files load and every reference resolves**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree && python3 - <<'EOF'
import json, sys
from pathlib import Path
comp = json.loads(Path("GPSR/constants.json").read_text())
arena = json.loads(Path("GPSR/constants.rcw2026.json").read_text())
poses = set(arena["possible_poses"])
objs = set(arena["possible_objects"])
bad = []
for k, v in arena.get("default_locations", {}).items():
    if k.startswith("_"): continue
    if k not in objs: bad.append(f"default_locations key {k!r} not in possible_objects")
    if v not in poses: bad.append(f"default_locations[{k}]={v!r} not in possible_poses")
for room, spots in arena.get("search_spots", {}).items():
    if room.startswith("_"): continue
    for s in spots:
        if s not in poses: bad.append(f"search_spots[{room}] names unknown pose {s!r}")
for arm_key in ("arm_pos_navigating", "arm_pos_scan", "arm_pos_orbbec_look", "arm_pos_scan_original"):
    if arena.get(arm_key) != comp.get(arm_key):
        bad.append(f"{arm_key} differs from the competition file — must be copied verbatim")
print("\n".join(bad) if bad else f"OK: {len(poses)} poses, {len(objs)} objects, all references resolve")
sys.exit(1 if bad else 0)
EOF
```
Expected: the `OK:` line and exit 0.

- [ ] **Step 5: Verify every arena pose is actually free on the map**

```bash
cd /home/tinker/tinker-sim/6.0.1 && python3 - <<'EOF'
import json, sys
sys.path.insert(0, "simulation")
from pathlib import Path
from tinker_sim_core.occupancy import OccupancyMap
ART = Path("artifacts/arena/rcw2026/d2b559b43207c8d54ae2609f638dca1cc36ee8b7adc7e4d94aee86e7fb56729c")
occ = OccupancyMap.from_pgm(ART / "map.pgm", resolution=0.05, origin_x=-5.05, origin_y=-6.0)
c = json.loads(Path("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json").read_text())
bad = [n for n, p in c["possible_poses"].items()
       if not occ.free_with_clearance(p["point"]["x"], p["point"]["y"], 0.35)]
print("obstructed poses:", bad if bad else "none")
sys.exit(1 if bad else 0)
EOF
```
Expected: `obstructed poses: none`, exit 0. Any obstructed pose must be moved using `nearest_free_world` before proceeding.

- [ ] **Step 6: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json src/behavior_tree/behavior_tree/GPSR/gpsr_full.py
git commit -m "feat: rcw2026 arena world model for GPSR, selected by GPSR_CONSTANTS_PATH

Waypoints derived from the arena placement manifest and verified free at
0.35 m clearance on the derived map. Competition constants.json untouched.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: Composite GPSR launch

GPSR needs cameras, arm, and Nav2 at once. `manipulation.launch.py` and `navigation.launch.py` both unconditionally start same-named `command_gateway`, `safety_supervisor`, `contract_guard`, and `robot_state_publisher` with divergent parameters, so including both verbatim collides. This must be an explicit de-duplicating merge.

**Files:**
- Create: `/home/tinker/tinker-sim/6.0.1/ros2_ws/src/tinker_sim_bridge/launch/gpsr.launch.py`
- Test: `/home/tinker/tinker-sim/6.0.1/tests/test_gpsr_launch.py`

**Interfaces:**
- Produces: launch file `gpsr.launch.py` with arguments `project_root`, `tinker_workspace`, `scenario` (default `"gpsr-rcw2026"`), `map_yaml`, `seed`.

- [ ] **Step 1: Write the failing test**

```python
from __future__ import annotations

import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "ros2_ws/src/tinker_sim_bridge/launch/gpsr.launch.py"


def _executables(tree: ast.AST) -> list[str]:
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "Node":
            for kw in node.keywords:
                if kw.arg == "executable" and isinstance(kw.value, ast.Constant):
                    found.append(kw.value.value)
    return found


class GpsrLaunchTest(unittest.TestCase):
    def setUp(self):
        self.assertTrue(LAUNCH.is_file(), f"{LAUNCH} does not exist")
        self.tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
        self.executables = _executables(self.tree)

    def test_singleton_nodes_are_not_duplicated(self):
        for name in ("command_gateway", "safety_supervisor", "contract_guard",
                     "robot_state_publisher"):
            self.assertEqual(
                self.executables.count(name), 1,
                f"{name} must appear exactly once; found {self.executables.count(name)}",
            )

    def test_manipulation_side_is_present(self):
        for name in ("ros2_control_node", "xarm_facade", "gripper_facade",
                     "pan_tilt_facade", "audio_fixtures"):
            self.assertIn(name, self.executables)

    def test_navigation_side_is_present(self):
        for name in ("base_facade", "initial_pose", "pointcloud_to_laserscan_node"):
            self.assertIn(name, self.executables)

    def test_supervisor_manages_controllers_for_the_arm(self):
        source = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("required_sources", source)
        self.assertNotIn('"manage_controllers": False', source,
                         "the composite owns the arm, so controllers must be managed")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gpsr_launch.py -q`
Expected: FAIL — `gpsr.launch.py does not exist`.

- [ ] **Step 3: Write the composite launch**

Build it by reading both existing launch files and merging, not by including them. Rules:
- Exactly one `robot_state_publisher`. Use manipulation's `topic_control_description(resolved_artifact.robot_urdf)` form — it is a superset (control-topic-augmented) of navigation's raw URDF read.
- Exactly one `command_gateway`, one `contract_guard` (`{"profile": "manipulation"}`, since the arm contract is the stricter one), one `safety_supervisor` with `manage_controllers` left at its default `True` (the composite starts `ros2_control_node`, so controllers exist to manage) and `required_sources` covering both use cases: `["xarm", "collision"]`.
- Keep from manipulation: `ros2_control_node`, the `joint_state_broadcaster` and `xarm7_traj_controller` reconcilers, `xarm_facade`, `gripper_facade`, `pan_tilt_facade`, `truth_evaluator`, `audio_fixtures` (pass `scenario_file` so dialogue is loaded), `scenario_runner`.
- Keep from navigation: `base_facade`, `initial_pose`, the `livox360_static_tf` static transform, `pointcloud_to_laserscan`, the `localization_no_ekf_launch.py` include (AMCL), the `ekf_filter_node`, and the `navigation_dwb_launch.py` include — all parameterised by `map_yaml`.
- Preserve the `env` dict pattern both files use for `additional_env`, including `PYTHONPATH` pointing at `<root>/simulation`.

- [ ] **Step 4: Run the test to verify it passes**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gpsr_launch.py tests/test_navigation_launch_map.py -q`
Expected: PASS, output pristine.

- [ ] **Step 5: Build the overlay**

```bash
cd /home/tinker/tinker-sim/6.0.1 && TINKER_WS=/home/tinker/tk25_ws ./scripts/build-humble-overlay 2>&1 | tail -3
```
Expected: `Summary: 2 packages finished`. Do NOT run this while another session holds `ros2_ws/install` — check `pgrep -f 'ros2 launch tinker_sim[_]bridge'` is empty first.

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/tinker_sim_bridge/launch/gpsr.launch.py tests/test_gpsr_launch.py
git commit -m "feat: composite GPSR launch merging navigation and manipulation stacks

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: GPSR arena scenario with YCB objects

**Files:**
- Create: `/home/tinker/tinker-sim/6.0.1/simulation/scenarios/gpsr-rcw2026.json`
- Test: `/home/tinker/tinker-sim/6.0.1/tests/test_gpsr_scenario.py`

**Interfaces:**
- Consumes: YCB artifact objects as repo-root-relative `asset_uri` (resolved by `_uri(root, asset_uri)`, `simulation/tinker_sim_core/orchestration.py:59`); scenario schema exactly as in `simulation/scenarios/pick-deliver-place-rcw2026.json`.
- Produces: scenario id `gpsr-rcw2026` (must equal the filename stem; must NOT join `QUALIFICATION_SCENARIO_NAMES`).

- [ ] **Step 1: Confirm the YCB object paths exist**

```bash
cd /home/tinker/tinker-sim/6.0.1
Y=artifacts/objects/ycb/d2d5ccd2c098b68f39737f8f0490358b7fd6cbfa8080b604851e073fc758acda
for o in ycb_010_tomato_soup_can ycb_025_mug ycb_011_banana ycb_024_bowl; do
  test -f "$Y/$o/object.usd" && echo "OK $o" || echo "MISSING $o"
done
```
Expected: four `OK` lines. If any is missing, the artifact needs re-materialising via `tools/ycb_import.py --config config/ycb-import.json` — report rather than improvising.

- [ ] **Step 2: Write the failing test**

```python
from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

SCENARIO = ROOT / "simulation/scenarios/gpsr-rcw2026.json"
ARENA = ROOT / "artifacts/arena/rcw2026/d2b559b43207c8d54ae2609f638dca1cc36ee8b7adc7e4d94aee86e7fb56729c"


class GpsrScenarioTest(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SCENARIO.is_file(), f"{SCENARIO} does not exist")
        self.raw = json.loads(SCENARIO.read_text(encoding="utf-8"))

    def test_id_matches_filename_stem(self):
        self.assertEqual(self.raw["id"], SCENARIO.stem)

    def test_targets_the_arena(self):
        self.assertEqual(self.raw["world"], {"mode": "arena", "arena": "rcw2026"})

    def test_objects_reference_existing_ycb_assets(self):
        self.assertTrue(self.raw["objects"], "scenario must spawn at least one object")
        for record in self.raw["objects"]:
            uri = record["asset_uri"]
            self.assertIn("artifacts/objects/ycb/", uri)
            self.assertTrue((ROOT / uri).is_file(), f"missing asset: {uri}")

    @unittest.skipUnless((ARENA / "map.pgm").is_file(),
                         "rcw2026 arena artifact not present in this checkout")
    def test_robot_spawn_is_free(self):
        from tinker_sim_core.occupancy import OccupancyMap
        occupancy = OccupancyMap.from_pgm(
            ARENA / "map.pgm", resolution=0.05, origin_x=-5.05, origin_y=-6.0
        )
        x, y, _ = self.raw["robot"]["initial_pose"]
        self.assertTrue(occupancy.free_with_clearance(x, y, 0.35))

    def test_objects_sit_on_declared_surfaces(self):
        surfaces = {
            s["surface_id"]: s
            for s in json.loads((ARENA / "placement.json").read_text())["surfaces"]
        } if (ARENA / "placement.json").is_file() else {}
        if not surfaces:
            self.skipTest("arena artifact not present")
        for record in self.raw["objects"]:
            x, y, z = record["pose"]["xyz"]
            on = [
                s for s in surfaces.values()
                if abs(x - s["center_xyz"][0]) <= s["size_xy"][0] / 2
                and abs(y - s["center_xyz"][1]) <= s["size_xy"][1] / 2
                and abs(z - s["center_xyz"][2]) < 0.05
            ]
            self.assertTrue(on, f"object {record['id']} rests on no declared surface")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gpsr_scenario.py -q`
Expected: FAIL — scenario file does not exist.

- [ ] **Step 4: Write the scenario**

Place four YCB objects on real arena surfaces, spawning each 0.01 m above its tabletop so it settles rather than interpenetrates. Tabletop heights come from `placement.json`: `kitchen_table#top` z=0.724, `side_table_02#top` z=0.602, `shelf_02#plate` z=1.06.

```json
{
    "schema_version": 2,
    "id": "gpsr-rcw2026",
    "world": {"mode": "arena", "arena": "rcw2026"},
    "robot": {"id": "tinker2", "initial_pose": [-2.0, -2.0, 0.0]},
    "actors": [],
    "objects": [
        {
            "id": "soup",
            "asset_uri": "artifacts/objects/ycb/d2d5ccd2c098b68f39737f8f0490358b7fd6cbfa8080b604851e073fc758acda/ycb_010_tomato_soup_can/object.usd",
            "pose": {"xyz": [2.5, -3.0, 0.734], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
        },
        {
            "id": "mug",
            "asset_uri": "artifacts/objects/ycb/d2d5ccd2c098b68f39737f8f0490358b7fd6cbfa8080b604851e073fc758acda/ycb_025_mug/object.usd",
            "pose": {"xyz": [2.7, -3.15, 0.734], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
        },
        {
            "id": "banana",
            "asset_uri": "artifacts/objects/ycb/d2d5ccd2c098b68f39737f8f0490358b7fd6cbfa8080b604851e073fc758acda/ycb_011_banana/object.usd",
            "pose": {"xyz": [0.43, 1.755, 0.612], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
        },
        {
            "id": "bowl",
            "asset_uri": "artifacts/objects/ycb/d2d5ccd2c098b68f39737f8f0490358b7fd6cbfa8080b604851e073fc758acda/ycb_024_bowl/object.usd",
            "pose": {"xyz": [0.312, -0.57, 1.07], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}
        }
    ],
    "regions": [],
    "events": [],
    "dialogue": [
        {"endpoint": "wait_for_start", "outcome": "start"},
        {"endpoint": "listen_action", "outcome": "go to the kitchen table and find a person"},
        {"endpoint": "get_confirmation_service", "outcome": "yes"}
    ],
    "postconditions": [
        {"name": "no safety stop", "path": "robot.safety_stop", "operator": "equals", "value": false}
    ]
}
```

Adjust each object's x/y so `test_objects_sit_on_declared_surfaces` passes — the values above must land inside the surface footprint given in `placement.json` (`kitchen_table#top` is 0.86×0.87 centred at [2.587, −3.058]).

- [ ] **Step 5: Run the tests to verify they pass**

Run: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_gpsr_scenario.py tests/test_orchestration.py -q`
Expected: PASS, output pristine. `test_orchestration.py` globs every scenario and will reject a malformed one.

- [ ] **Step 6: Commit**

```bash
git add simulation/scenarios/gpsr-rcw2026.json tests/test_gpsr_scenario.py
git commit -m "feat: gpsr-rcw2026 scenario placing YCB objects on arena surfaces

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 5: Sim mock profile and bring-up runbook

**Files:**
- Create: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/mock_config.sim.json`
- Create: `/home/tinker/tinker-sim/6.0.1/docs/gpsr-sim-runbook.md`

**Interfaces:**
- Produces: a mock config where every subsystem is `enabled: false`, selected at runtime by `BT_MOCK_CONFIG`.

- [ ] **Step 1: Write the sim mock config**

Copy `mock_config.json`'s exact structure, setting `enabled: false` for all six subsystems (`vision`, `manipulation`, `navigation`, `audio_input`, `announcement`, `mock_controls`). Read the shipped file first and preserve every other key — only the `enabled` flags change. The shipped default mocks `navigation`, which is wrong for this plan: Nav2 is real here.

- [ ] **Step 2: Verify nothing reports as mocked**

```bash
cd /home/tinker/tk25_ws && source install/setup.bash
BT_MOCK_CONFIG=$PWD/src/tk25_decision/src/behavior_tree/behavior_tree/mock_config.sim.json \
python3 -c "
from behavior_tree.config import is_subsystem_mocked, is_full_mock_mode
subs = ['vision','manipulation','navigation','audio_input','announcement','mock_controls']
bad = [s for s in subs if is_subsystem_mocked(s)]
assert not bad, f'still mocked: {bad}'
assert not is_full_mock_mode(), 'full mock mode must be off'
print('nothing mocked')
"
```
Expected: `nothing mocked`.

- [ ] **Step 3: Write the runbook**

Document the exact bring-up order, one command block per stack, with the env preamble from Global Constraints repeated in each: (1) sim, (2) composite bridge launch, (3) tk26_vision `vision_bringup.launch.py enable_gpsr:=true` plus `camera_server_node`, (4) tk25_manipulation `manipulation_planning_task_only.launch.py execution_profile:=sim_cumotion` plus `arm_api grasp_action` plus `anygrasp_ros2`, (5) tk26_navigation `approach_planner` + `orientation_angle_service`, (6) GPSR itself. Include the teardown discipline and the GPU pre-flight check verbatim. State that `camera_backend` defaults to `service`, so either `camera_server_node` runs or every vision node needs `-p camera_backend:=subscription`.

- [ ] **Step 4: Commit (two repos, two commits)**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/mock_config.sim.json
git commit -m "feat: sim mock profile with every subsystem real

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
cd /home/tinker/tinker-sim/6.0.1
git add docs/gpsr-sim-runbook.md
git commit -m "docs: GPSR-in-simulation bring-up runbook

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 6: Interface census against the live stack

The first live milestone. Proves the topology holds before any behaviour is attempted, and fails fast and specifically when a stack is missing.

**Files:**
- Create: `/home/tinker/tinker-sim/6.0.1/tools/gpsr_interface_census.py`

**Interfaces:**
- Produces: CLI `python3 tools/gpsr_interface_census.py [--json OUT]`, exit 0 when every required interface is present, exit 1 listing what is missing.

- [ ] **Step 1: Write the census tool**

It must check, by name and type, every interface GPSR needs, grouped by owning stack so a failure names the stack to start:

```python
#!/usr/bin/env python3
"""Check that every ROS interface GPSR needs is being served.

Run with the ROS overlay sourced and ROS_DOMAIN_ID matching the stack.
Exit 0 when everything GPSR calls exists; exit 1 with a per-stack breakdown.
"""
from __future__ import annotations

import argparse
import json
import sys

import rclpy
from rclpy.node import Node

SERVICES = {
    "sim bridge": [("announce", "tinker_audio_msgs/srv/TextToSpeech")],
    "tk26_vision": [
        ("object_detection_generalist", "tinker_vision_msgs_26/srv/ObjectDetectionGeneralist"),
        ("object_detection_yolo", "tinker_vision_msgs_26/srv/ObjectDetection"),
        ("door_detection_srv", "tinker_vision_msgs_26/srv/DoorDetection"),
    ],
    "tk26_navigation": [
        ("find_approach_pose", "tinker_nav_msgs/srv/FindApproachPose"),
        ("orientation_angle_service", "tinker_nav_msgs/srv/OrientationAngle"),
    ],
}

ACTIONS = {
    "sim bridge": ["listen_action", "/xarm_gripper/gripper_action"],
    "nav2": ["navigate_to_pose"],
    "tk26_navigation": ["go_to_approach"],
    "tk26_vision": ["feature_extraction_service", "detect_waving_persons"],
    "tk25_manipulation": ["joint_move_action", "start_grasp"],
}

TOPICS = {
    "sim cameras": [
        ("/camera/color/image_raw", "sensor_msgs/msg/Image"),
        ("/camera/depth/image_raw", "sensor_msgs/msg/Image"),
        ("/camera/color/camera_info", "sensor_msgs/msg/CameraInfo"),
        ("/camera/xarm_camera/color/image_raw", "sensor_msgs/msg/Image"),
    ],
    "sim bridge": [("/pan_tilt_controller/state", "tinker_vision_msgs_26/msg/PanTiltState")],
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", default=None)
    args = parser.parse_args()

    rclpy.init()
    node = Node("gpsr_interface_census")
    # Let discovery settle; a bare graph query right after init under-reports.
    end = node.get_clock().now().nanoseconds + 5_000_000_000
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    have_services = {name: types for name, types in node.get_service_names_and_types()}
    have_topics = {name: types for name, types in node.get_topic_names_and_types()}
    # Actions surface as a /_action/send_goal service per action name.
    have_actions = {
        name[: -len("/_action/send_goal")]
        for name in have_services
        if name.endswith("/_action/send_goal")
    }

    missing: dict[str, list[str]] = {}

    def miss(stack: str, what: str) -> None:
        missing.setdefault(stack, []).append(what)

    for stack, entries in SERVICES.items():
        for name, type_name in entries:
            found = have_services.get(name) or have_services.get("/" + name)
            if not found:
                miss(stack, f"service {name}")
            elif type_name not in found:
                miss(stack, f"service {name} has type {found}, expected {type_name}")

    for stack, names in ACTIONS.items():
        for name in names:
            if name not in have_actions and name.lstrip("/") not in have_actions:
                miss(stack, f"action {name}")

    for stack, entries in TOPICS.items():
        for name, type_name in entries:
            found = have_topics.get(name)
            if not found:
                miss(stack, f"topic {name}")
            elif type_name not in found:
                miss(stack, f"topic {name} has type {found}, expected {type_name}")

    node.destroy_node()
    rclpy.shutdown()

    report = {"missing": missing, "ok": not missing}
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=1, sort_keys=True)
    if missing:
        for stack, items in sorted(missing.items()):
            print(f"MISSING [{stack}]")
            for item in items:
                print(f"  - {item}")
        return 1
    print("all GPSR interfaces present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Verify it fails cleanly with nothing running**

```bash
cd /home/tinker/tinker-sim/6.0.1
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=99 timeout 60 python3 tools/gpsr_interface_census.py; echo "exit=$?"
```
Expected: `exit=1` and a `MISSING [...]` block naming every stack. A crash or a hang is a failure of the tool, not a valid result.

- [ ] **Step 3: Bring up the full stack and run the census**

Pre-flight: `nvidia-smi --query-compute-apps=pid --format=csv,noheader` MUST be empty. If it is not, wait — do not contend for the GPU.

Follow `docs/gpsr-sim-runbook.md` in order, recording the PID of every process you start. Then:
```bash
ROS_DOMAIN_ID=42 python3 tools/gpsr_interface_census.py --json reports/gpsr-sim-2026-08-20/census.json
```
Expected: `all GPSR interfaces present`, exit 0. Any missing entry names the stack that failed to start — fix the bring-up, do not weaken the census.

- [ ] **Step 4: Tear down and commit**

SIGINT every recorded PID, wait 10 s, SIGKILL survivors, then confirm the GPU is clear.

```bash
git add tools/gpsr_interface_census.py
git commit -m "feat: GPSR interface census tool for sim bring-up verification

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 7: One command end-to-end

**Files:** evidence under `reports/gpsr-sim-2026-08-20/` (gitignored). No code changes expected — a failure here reopens the task that owns the broken piece.

- [ ] **Step 1: Bring up the stack and confirm the census passes**

Repeat Task 6 Step 3. Do not proceed on a partial census.

- [ ] **Step 2: Prove perception against sim frames**

```bash
source /opt/ros/humble/setup.bash && source /home/tinker/tk25_ws/install/setup.bash
ROS_DOMAIN_ID=42 ros2 service call /object_detection_generalist \
  tinker_vision_msgs_26/srv/ObjectDetectionGeneralist \
  "{camera: 'orbbec', prompt: 'red tomato soup can', target_frame: 'map'}" \
  2>&1 | tee reports/gpsr-sim-2026-08-20/perception.log
```
Assert: `status: 0` and at least one object returned. Save the response verbatim. If detection returns empty, capture one camera frame to `reports/gpsr-sim-2026-08-20/head_frame.png` and report — an empty result here is a genuine finding about asset realism, not a reason to mock vision.

- [ ] **Step 3: Run one command end-to-end**

```bash
cd /home/tinker/tk25_ws && source install/setup.bash
export ROS_DOMAIN_ID=42
export GPSR_CONSTANTS_PATH=$PWD/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json
export BT_MOCK_CONFIG=$PWD/src/tk25_decision/src/behavior_tree/behavior_tree/mock_config.sim.json
export BT_GPSR_CMD="go to the kitchen table and find a person"
export BT_GPSR_NUM_COMMANDS=1
set -a; source /home/tinker/tk25_ws/.env; set +a
ros2 run behavior_tree gpsr-orchestrator 2>&1 | tee reports/gpsr-sim-2026-08-20/orchestrator.log
```

Assert, from the log:
1. The top-layer split produced at least one target.
2. The per-target plan names only actions from `ACTION_FACTORIES` and only locations present in `constants.rcw2026.json`.
3. `BtNode_CaptureCurrentPose` succeeded (proves `map`→`base_link` TF is alive).
4. A `navigate_to_pose` goal was accepted and reached its waypoint.

- [ ] **Step 4: Record the robot's achieved pose**

```bash
ROS_DOMAIN_ID=42 timeout 20 ros2 topic echo /amcl_pose --once \
  | tee reports/gpsr-sim-2026-08-20/final_pose.txt
```
Assert the pose is within 0.5 m of the `kitchen_table` waypoint in `constants.rcw2026.json`.

- [ ] **Step 5: Write the evidence summary**

Create `reports/gpsr-sim-2026-08-20/SUMMARY.md` with one row per assertion above: what ran, the measured value, the expected value, PASS/FAIL, and the evidence file. Include the census JSON and every PID torn down.

- [ ] **Step 6: Tear down**

SIGINT every recorded PID in reverse bring-up order, wait, SIGKILL survivors, verify `nvidia-smi --query-compute-apps=pid --format=csv,noheader` is empty.

- [ ] **Step 7: Report** — summarise per-assertion results, every deviation, and any interface that behaved differently from the census.

## Self-review notes

- **Spec coverage:** the spec's four components map to Tasks 1 (transport fix), 2 (arena world model), 3 (composite launch), 4 (scenario/objects); Task 5 covers the mock profile and runbook the topology section implies; Tasks 6–7 implement the spec's five-stage verification ladder. The spec's "out of scope" items (simulated ASR, mission supervisor, competition constants, credentials) are not touched by any task.
- **Type consistency:** `ActionHandler(name, action_type, action_name, key, wait_for_server_timeout_sec)` is used identically in Task 1's two conversions and matches `BtNode_Grasp`'s call. `free_with_clearance(x, y, clearance_m)` is used identically in Tasks 2 and 4. `GPSR_CONSTANTS_PATH` is defined in Task 2 Step 3 and consumed in Task 7 Step 3. `BT_MOCK_CONFIG` is defined in Task 5 and consumed in Task 7. Scenario id `gpsr-rcw2026` in Task 4 matches the launch default in Task 3.
- **Known judgment points (verify, don't assume):** the seventh arena surface's name (read `placement.json`, Task 2 Step 1); the exact import lines for `ActionHandler`/`action_msgs` in `Vision.py` (copy from `Manipulation.py`, Task 1 Step 2); whether `anygrasp_ros2` needs its own weights present (Task 5 Step 3); and whether `get_confirmation_action` (an action GPSR calls) is satisfied by the sim's `get_confirmation_service` (a service) — if `create_ask_person` hangs in Task 7, that mismatch is the cause and should be reported, not patched around.
