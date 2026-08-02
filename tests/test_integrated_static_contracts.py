"""Task 2: integrated Gate B static contract checks (F1-F5).

The tests build a self-consistent fixture from scratch:

* a real production Git repository whose immutable files (SRDF, controllers.yaml,
  C++ sources, launch, .action schemas) are committed at a real
  ``implementation_head``;
* a simulator tree whose config, 17 configured scenario declarations, overlay
  contract, model bundle and provider manifest are internally consistent
  (hashes recomputed in the same canonical way the checker expects);
* a produced three-entry source-lock manifest referencing the production
  implementation head.

Mutations are baked into the fixture so the immutable blob at the inspected
commit carries the mutated bytes; the checker must fail the corresponding
semantic check (never a comment/path-only shortcut).
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

from integrated_static_contracts import (  # noqa: E402
    ARM_JOINTS,
    GRIPPER_JOINT,
    PROD_ACTION_DIR_REL,
    PROD_ACTION_EXECUTION_CPP_REL,
    PROD_ACTION_RUNTIME_CPP_REL,
    PROD_GRIPPER_CONTROLLERS_REL,
    PROD_LAUNCH_REL,
    PROD_PACKAGE_UTILS_CPP_REL,
    PROD_PICK_AND_PLACE_CPP_REL,
    PROD_SCENE_OWNERSHIP_CPP_REL,
    PROD_SRDF_REL,
    PROD_XARM7_CONTROLLERS_REL,
    RUNTIME_MAPPING_KEYS,
    TOUCH_LINKS,
    validate_static_contracts,
)

FIXED_COMMIT_DATE = "2026-07-01T00:00:00Z"
CONFIG_REL = "simulation/qualification/integrated-ompl.json"
MANIFEST_NAME = "source-lock-manifest.json"
REAL_CONFIG = ROOT / CONFIG_REL

SCENARIO_OWNED = {
    "qualification-moveit-plan-joint": ["sim_fixture/pedestal", "sim_fixture/public_target"],
    "qualification-moveit-plan-pose": ["sim_fixture/pedestal", "sim_fixture/public_target"],
    "qualification-moveit-plan-blocked": ["sim_fixture/pedestal", "sim_fixture/public_target", "sim_fixture/plan_blocker"],
    "qualification-moveit-execute-joint": ["sim_fixture/pedestal", "sim_fixture/public_target"],
    "qualification-moveit-execute-pose": ["sim_fixture/pedestal", "sim_fixture/public_target"],
    "qualification-moveit-cartesian-retreat": ["sim_fixture/pedestal", "sim_fixture/public_target"],
    "qualification-moveit-gripper": ["sim_fixture/pedestal", "sim_fixture/public_target"],
    "qualification-moveit-cancel": ["sim_fixture/pedestal", "sim_fixture/public_target"],
    "qualification-moveit-safety": ["sim_fixture/pedestal", "sim_fixture/public_target"],
    "qualification-pick-place-positive": ["sim_fixture/pedestal", "sim_fixture/qualification_cube", "sim_fixture/place_pedestal"],
    "qualification-pick-place-blocked-approach": ["sim_fixture/pedestal", "sim_fixture/qualification_cube", "sim_fixture/place_pedestal", "sim_fixture/plan_blocker"],
    "qualification-pick-place-unreachable-grasp": ["sim_fixture/pedestal", "sim_fixture/qualification_cube", "sim_fixture/place_pedestal"],
    "qualification-pick-place-malformed-back": ["sim_fixture/pedestal", "sim_fixture/qualification_cube", "sim_fixture/place_pedestal"],
    "qualification-pick-place-cancel-approach": ["sim_fixture/pedestal", "sim_fixture/qualification_cube", "sim_fixture/place_pedestal"],
    "qualification-pick-place-cancel-transport": ["sim_fixture/pedestal", "sim_fixture/qualification_cube", "sim_fixture/place_pedestal"],
    "qualification-pick-place-safety-transport": ["sim_fixture/pedestal", "sim_fixture/qualification_cube", "sim_fixture/place_pedestal"],
    "qualification-pick-place-occupied-place": ["sim_fixture/pedestal", "sim_fixture/qualification_cube", "sim_fixture/place_pedestal", "sim_fixture/place_occupant"],
}
TARGET_BY_SCENARIO = {
    sid: ("sim_fixture/public_target" if sid.startswith("qualification-moveit-") else "sim_fixture/qualification_cube")
    for sid in SCENARIO_OWNED
}


def _canonical(obj: object) -> bytes:
    return json.dumps(obj, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _sha256(obj: object) -> str:
    return hashlib.sha256(_canonical(obj)).hexdigest()


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _deep_merge(base: Any, over: Any) -> Any:
    if isinstance(base, dict) and isinstance(over, dict):
        merged = dict(base)
        for key, value in over.items():
            if key in merged:
                merged[key] = _deep_merge(merged[key], value)
            else:
                merged[key] = value
        return merged
    return over


def _write_json_canonical(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(_canonical(value))


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _write(root: Path, relative: str, content: bytes | str) -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content if isinstance(content, bytes) else content.encode("utf-8"))
    return path


def _git(root: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(root), *args],
        capture_output=True,
        text=True,
        env={**os.environ, "LC_ALL": "C", "GIT_AUTHOR_DATE": FIXED_COMMIT_DATE, "GIT_COMMITTER_DATE": FIXED_COMMIT_DATE},
        check=False,
    )


def _git_init(root: Path) -> None:
    root.mkdir(parents=True, exist_ok=True)
    _git(root, "init", "-q")
    _git(root, "config", "user.email", "fixture@example.invalid")
    _git(root, "config", "user.name", "Fixture")


def _git_head(root: Path) -> str:
    proc = _git(root, "rev-parse", "HEAD")
    assert proc.returncode == 0, proc.stderr
    return proc.stdout.strip()


def _commit(root: Path, message: str) -> None:
    _git(root, "add", "-A")
    proc = _git(root, "commit", "-q", "-m", message)
    assert proc.returncode == 0, proc.stderr


# ---------------------------------------------------------------------------
# default immutable production files
# ---------------------------------------------------------------------------
DEFAULT_SRDF = """<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="xarm7_srdf">
  <xacro:macro name="xarm7_macro_srdf" params="prefix='' add_gripper='false' add_vacuum_gripper='false' add_bio_gripper='false' add_other_geometry='false' ">
    <group name="${prefix}xarm7">
      <joint name="${prefix}world_joint" />
      <joint name="${prefix}joint1" />
      <joint name="${prefix}joint2" />
      <joint name="${prefix}joint3" />
      <joint name="${prefix}joint4" />
      <joint name="${prefix}joint5" />
      <joint name="${prefix}joint6" />
      <joint name="${prefix}joint7" />
      <joint name="${prefix}joint_eef" />
      <xacro:if value="${add_gripper}">
        <joint name="${prefix}gripper_fix" />
        <joint name="${prefix}joint_tcp" />
      </xacro:if>
    </group>
    <xacro:if value="${add_gripper}">
      <group name="${prefix}xarm_gripper">
        <link name="${prefix}xarm_gripper_base_link" />
        <link name="${prefix}left_outer_knuckle" />
        <link name="${prefix}left_finger" />
        <link name="${prefix}left_inner_knuckle" />
        <link name="${prefix}right_inner_knuckle" />
        <link name="${prefix}right_outer_knuckle" />
        <link name="${prefix}right_finger" />
        <link name="${prefix}link_tcp" />
        <joint name="${prefix}drive_joint" />
      </group>
      <end_effector name="${prefix}xarm_gripper" parent_link="${prefix}link_tcp" group="${prefix}xarm_gripper" />
    </xacro:if>
  </xacro:macro>
</robot>
"""

DEFAULT_XARM7_CONTROLLERS = """controller_names:
  - xarm7_traj_controller

xarm7_traj_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
    - joint7
"""

DEFAULT_GRIPPER_CONTROLLERS = """controller_names:
  - xarm_gripper

xarm_gripper:
  action_ns: gripper_action
  type: GripperCommand
  default: true
  joints:
    - drive_joint
"""

DEFAULT_JOINT_LIMITS_XARM7 = """joint1:
  min_position: -6.283185307179586
  max_position: 6.283185307179586
"""

DEFAULT_JOINT_LIMITS_GRIPPER = """drive_joint:
  min_position: 0.0
  max_position: 0.85
"""

DEFAULT_KINEMATICS = """xarm7:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  tip_link: link_tcp
"""

DEFAULT_PICK_AND_PLACE_CPP = """#include <chrono>
#include <string>
#include <thread>

// Deterministic teardown: bounded runtime shutdown then joined executor thread.
void GraspNode::shutdown_teardown() {
  (void)motion_runtime_.shutdown(std::chrono::seconds(3));
  if (executor_thread_.joinable()) executor_thread_.join();
}

// The destructor establishes a bounded deadline, shuts the runtime down, joins
// the executor thread and resets the state-validity client in this exact body.
GraspNode::~GraspNode() {
  const auto shutdown_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  motion_runtime_.shutdown(shutdown_deadline);
  if (executor_thread_.joinable()) executor_thread_.join();
  check_state_validity_client_.reset();
}

// Straight slide forwards the avoid_collisions boolean to the typed request.
StageResult GraspNode::move_straight(TransactionContext &ctx, const geometry_msgs::msg::Pose &target_pose,
                                     bool avoid_collisions, int16_t stage, std::chrono::milliseconds timeout) {
  moveit_msgs::srv::GetCartesianPath::Request request;
  request.group_name = move_group->getName();
  request.link_name = move_group->getEndEffectorLink();
  request.avoid_collisions = avoid_collisions;
  return {ResultStatus::Success, 0, ""};
}
"""

DEFAULT_ACTION_EXECUTION_CPP = """#include <cstdint>
#include <string>

// Terminal result-field writes shared by the task action servers.
void write_task_result(Result *result, ResultStatus status, int16_t stage, std::string message) {
  result->success = status == ResultStatus::Success;
  result->status = static_cast<int16_t>(status);
  result->stage = status == ResultStatus::Success ? 0 : stage;
  result->error_msg = std::move(message);
}

// Each task builder writes exactly the fields its .action result schema
// declares.  A write in a different builder never satisfies another builder's
// required fields (F2.1).
void make_pick_result(Result *result, ResultStatus status, int16_t stage, std::string message) {
  result->stage = status == ResultStatus::Success ? 0 : stage;
  result->status = static_cast<int16_t>(status);
  result->error_msg = std::move(message);
}

void make_place_result(Result *result, ResultStatus status, int16_t stage, std::string message) {
  result->status = static_cast<int16_t>(status);
  result->error_msg = std::move(message);
  result->stage = status == ResultStatus::Success ? 0 : stage;
}

void make_cartesian_result(Result *result, ResultStatus status, int16_t stage, std::string message) {
  result->success = status == ResultStatus::Success;
  result->stage = status == ResultStatus::Success ? 0 : stage;
  result->status = static_cast<int16_t>(status);
  result->error_msg = std::move(message);
}

void make_joint_result(Result *result, ResultStatus status, int16_t stage, std::string message) {
  result->success = status == ResultStatus::Success;
  result->stage = status == ResultStatus::Success ? 0 : stage;
  result->status = static_cast<int16_t>(status);
  result->error_msg = std::move(message);
}

void make_fold_result(Result *result, ResultStatus status, int16_t stage, std::string message) {
  result->success = status == ResultStatus::Success;
  result->stage = status == ResultStatus::Success ? 0 : stage;
  result->status = static_cast<int16_t>(status);
  result->error_msg = std::move(message);
}
"""

DEFAULT_ACTION_RUNTIME_CPP = """#include <chrono>
#include <thread>

// The coordinator spawns a managed worker per transaction and joins it; the
// runtime destructor joins the coordinator with a bounded shutdown.  No
// detached thread is ever used.
void coordinator_main(MotionRuntime *runtime) {
  for (;;) {
    Transaction *transaction = runtime->next_transaction();
    if (transaction == nullptr) break;
    transaction->worker = std::thread([transaction]() { transaction->run(); });
    if (transaction->worker.joinable()) transaction->worker.join();
  }
}

MotionRuntime::~MotionRuntime() {
  if (coordinator_.joinable()) coordinator_.join();
  shutdown(std::chrono::seconds(3));
}
"""

DEFAULT_PACKAGE_UTILS_CPP = """#include <string>
#include <vector>

// Strict simulator ownership is scoped to PlanningSceneRuntime.  The SimOmpl
// branch returns before the hardware namespaced cleanup below.
void GraspNode::clean_planning_scene() {
  if (execution_profile_ == ExecutionProfile::SimOmpl) {
    if (keepout_enabled_.load()) apply_floor_keepout(true);
    return;
  }
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  const auto remove_ids = task_cleanup_remove_ids(
      scene_state_, known_objects, attached_ids, /*explicit_teardown=*/false);
  planning_scene_interface->applyPlanningScene(planning_scene_msg);
}
"""

DEFAULT_SCENE_OWNERSHIP_CPP = """#include <chrono>

// SimOmpl requires the native close to confirm obstruction; the hardware
// compatibility branch remains separately guarded with collision-disabled lift.
StageResult run_post_close_pick(TransactionContext &ctx, ExecutionProfile profile, TaskSceneState &state,
                                const GripperExecutionResult &close_result, const TargetSceneObject &target,
                                bool stay, SceneBackend &backend, AttachmentReconciler &attachments,
                                std::chrono::milliseconds timeout) {
  if (profile == ExecutionProfile::SimOmpl && !close_result.confirms_obstruction()) {
    return {ResultStatus::PostconditionFailed, close_result.stage_result.stage,
            "native close reached the free-space goal without obstruction"};
  }
  StageResult result;
  if (profile == ExecutionProfile::Hardware) {
    result = backend.execute_lift(ctx, false, post_close_stage, timeout);
    if (!result.ok()) return result;
  }
  result = attachments.reconcile(ctx, state, request, std::chrono::steady_clock::now() + timeout);
  if (profile == ExecutionProfile::SimOmpl) {
    result = backend.execute_lift(ctx, true, post_close_stage, timeout);
    if (!result.ok()) return result;
  }
  return {ResultStatus::Success, 0, ""};
}
"""

DEFAULT_GRASP_NODE_HPP = """#include <thread>
#include <chrono>

class MotionRuntime {
public:
  void shutdown(std::chrono::milliseconds budget) {}
};

class GraspNode {
  std::thread executor_thread_;
  MotionRuntime motion_runtime_;
  bool keepout_enabled_;
  ExecutionProfile execution_profile_;
};
"""

DEFAULT_PICK_ACTION = """# ============ Goal ============
geometry_msgs/Pose target_pose

---

# ============ Result ============
int16 stage
int16 status
string error_msg

---

# ============ Feedback ============
"""

DEFAULT_PLACE_ACTION = """# ============ Goal ============
geometry_msgs/Pose target_pose

---

# ============ Result ============
int16 status
string error_msg
int16 stage

---

# ============ Feedback ============
"""

DEFAULT_CARTESIAN_ACTION = """# ============ Goal ============
geometry_msgs/Pose target_pose

---

# ============ Result ============
bool success
int16 stage
int16 status
string error_msg

---

# ============ Feedback ============
"""

DEFAULT_JOINT_MOVE_ACTION = """# ============ Goal ============
float32[] joint_positions

---

# ============ Result ============
bool success
int16 stage
int16 status
string error_msg

---

# ============ Feedback ============
"""

DEFAULT_FOLD_ACTION = """# ============ Goal ============
string mode

---

# ============ Result ============
bool success
int16 stage
int16 status
string error_msg

---

# ============ Feedback ============
"""

DEFAULT_LAUNCH = """from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([])
"""

PRODUCTION_FILES: dict[str, str] = {
    PROD_SRDF_REL: DEFAULT_SRDF,
    PROD_XARM7_CONTROLLERS_REL: DEFAULT_XARM7_CONTROLLERS,
    PROD_GRIPPER_CONTROLLERS_REL: DEFAULT_GRIPPER_CONTROLLERS,
    PROD_LAUNCH_REL: DEFAULT_LAUNCH,
    PROD_PICK_AND_PLACE_CPP_REL: DEFAULT_PICK_AND_PLACE_CPP,
    PROD_ACTION_EXECUTION_CPP_REL: DEFAULT_ACTION_EXECUTION_CPP,
    PROD_ACTION_RUNTIME_CPP_REL: DEFAULT_ACTION_RUNTIME_CPP,
    PROD_PACKAGE_UTILS_CPP_REL: DEFAULT_PACKAGE_UTILS_CPP,
    PROD_SCENE_OWNERSHIP_CPP_REL: DEFAULT_SCENE_OWNERSHIP_CPP,
    "src/pick_and_place/include/grasp_node.hpp": DEFAULT_GRASP_NODE_HPP,
    PROD_ACTION_DIR_REL + "/Pick.action": DEFAULT_PICK_ACTION,
    PROD_ACTION_DIR_REL + "/Place.action": DEFAULT_PLACE_ACTION,
    PROD_ACTION_DIR_REL + "/CartesianMove.action": DEFAULT_CARTESIAN_ACTION,
    PROD_ACTION_DIR_REL + "/JointMove.action": DEFAULT_JOINT_MOVE_ACTION,
    PROD_ACTION_DIR_REL + "/Fold.action": DEFAULT_FOLD_ACTION,
    "src/xarm_ros2/xarm_moveit_config/config/xarm7/joint_limits.yaml": DEFAULT_JOINT_LIMITS_XARM7,
    "src/xarm_ros2/xarm_moveit_config/config/xarm_gripper/joint_limits.yaml": DEFAULT_JOINT_LIMITS_GRIPPER,
    "src/xarm_ros2/xarm_moveit_config/config/xarm7/kinematics.yaml": DEFAULT_KINEMATICS,
}


def _scenario_doc(sid: str) -> dict[str, Any]:
    owned = SCENARIO_OWNED[sid]
    target = TARGET_BY_SCENARIO[sid]
    revision = "fixture-{}".format(sid)
    planning_scene = {
        "revision": revision,
        "frame_id": "base_link",
        "target_source_id": target,
        "target_handoff": "pick_and_place/object_mesh",
        "objects": [
            {"id": oid, "class": "target" if oid == target else "static", "primitive": {"type": "box"}}
            for oid in owned
        ],
    }
    planning_scene["revision_digest"] = _sha256(
        {k: v for k, v in planning_scene.items() if k != "revision_digest"}
    )
    stage = "C" if sid.startswith("qualification-moveit-plan") else "D" if sid.startswith("qualification-moveit-") else "E"
    return {
        "schema_version": 2,
        "id": sid,
        "qualification_gate": sid,
        "seed": 7,
        "planning_scene": planning_scene,
        "integrated": {
            "stage": stage,
            "execution_profile": "sim_ompl",
            "expected_scene": {"owned_ids": owned, "attached_ids": [], "task_target_id": "pick_and_place/object_mesh"},
            "forbidden_endpoints": ["/isaac_joint_commands"],
            "terminal_policy": "verified by independent raw-truth replay",
        },
    }


def _scenario_declaration_sha(sid: str, raw: Mapping[str, Any]) -> str:
    decl = {k: v for k, v in raw.items() if k not in ("id", "seed")}
    return _sha256({"id": sid, "seed": raw.get("seed"), "declaration": decl})


def _build_overlay_contract(simulator_root: Path, production_root: Path, impl_head: str) -> dict[str, Any]:
    scenarios: dict[str, Any] = {}
    for sid in SCENARIO_OWNED:
        raw = _read_json(simulator_root / "simulation/scenarios" / (sid + ".json"))
        scenarios[sid] = {
            "scenario_declaration_sha256": _scenario_declaration_sha(sid, raw),
            "planning_scene": {
                "owned_ids": list(SCENARIO_OWNED[sid]),
                "revision": raw["planning_scene"]["revision"],
                "revision_digest": raw["planning_scene"]["revision_digest"],
                "frame_id": "base_link",
                "target_source_id": raw["planning_scene"]["target_source_id"],
                "target_handoff": "pick_and_place/object_mesh",
            },
        }

    typed_contract = {
        "report_revision": "integrated-manipulation-v1",
        "actions": {
            "/pickup_action": {"source": "/pick_and_place", "type": "tinker_arm_msgs/action/Pick"},
            "/place_action": {"source": "/pick_and_place", "type": "tinker_arm_msgs/action/Place"},
            "/cartesian_move_action": {"source": "/pick_and_place", "type": "tinker_arm_msgs/action/CartesianMove"},
            "/joint_move_action": {"source": "/pick_and_place", "type": "tinker_arm_msgs/action/JointMove"},
            "/fold_action": {"source": "/pick_and_place", "type": "tinker_arm_msgs/action/Fold"},
            "/xarm7_traj_controller/follow_joint_trajectory": {"source": "controller_resource:xarm7_traj_controller", "type": "control_msgs/action/FollowJointTrajectory"},
            "/xarm_gripper/gripper_action": {"source": "/tinker_sim_gripper_facade", "type": "control_msgs/action/GripperCommand"},
        },
        "services": {
            "/apply_planning_scene": {"source": "/move_group", "type": "moveit_msgs/srv/ApplyPlanningScene"},
            "/get_planning_scene": {"source": "/move_group", "type": "moveit_msgs/srv/GetPlanningScene"},
        },
        "publishers": {
            "/isaac_joint_commands": {"cardinality": 1, "depth": 50, "durability": "VOLATILE", "reliability": "RELIABLE", "source": "/tinker_sim_command_gateway", "type": "sensor_msgs/msg/JointState"},
            "/joint_states": {"cardinality": 1, "depth": 10, "durability": "VOLATILE", "reliability": "RELIABLE", "source": "/controller_manager", "type": "sensor_msgs/msg/JointState"},
        },
        "joint_names": list(ARM_JOINTS) + [GRIPPER_JOINT],
        "touch_links": list(TOUCH_LINKS),
        "tf": {"parent": "base_link", "child": "link_tcp"},
        "controller_resources": {"joint_state_broadcaster": "active", "xarm7_traj_controller": "active"},
        "final_simulation_state": "STATE_PLAYING",
    }
    runtime_sha = _sha256({key: typed_contract[key] for key in RUNTIME_MAPPING_KEYS})
    typed_contract["runtime_contract_sha256"] = runtime_sha
    typed_contract["public_report_separation"] = {
        "public_integrated": {"execution_profile": "sim_ompl"},
        "public_integrated_sha256": _sha256({"execution_profile": "sim_ompl"}),
    }

    bundle = _read_json(simulator_root / "outputs/ompl-overlay/model-bundle-r2/model-bundle.json")
    provider = _read_json(simulator_root / "ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json")
    provider_raw = (simulator_root / "ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json").read_bytes()

    return {
        "contract_id": "simulator-ompl-overlay-acceptance",
        "schema_version": 1,
        "scenarios": scenarios,
        "typed_contract": typed_contract,
        "evidence": {"task6": {"runtime_contract_sha256": runtime_sha, "public_integrated_sha256": _sha256({"execution_profile": "sim_ompl"})}},
        "fixture_contract": {
            "target_handoff": "pick_and_place/object_mesh",
            "target_source_id": "sim_fixture/public_target",
            "task_owned_lifecycle": "pick_and_place creates and owns pick_and_place/object_mesh itself",
        },
        "model_bundle": {
            "structural_fingerprint": bundle["structural_fingerprint"],
            "manifest_sha256": _sha256_bytes((simulator_root / "outputs/ompl-overlay/model-bundle-r2/model-bundle.json").read_bytes()),
            "artifacts": {key: {"path_relative": value.get("path_relative"), "sha256": value["sha256"]} for key, value in bundle.get("artifacts", {}).items()},
            "production_source_commits": {
                "arm_joint_limits": {"commit": impl_head, "path_relative": "src/xarm_ros2/xarm_moveit_config/config/xarm7/joint_limits.yaml", "repo_path": str(production_root), "sha256": _sha256_bytes(DEFAULT_JOINT_LIMITS_XARM7.encode("utf-8"))},
                "gripper_joint_limits": {"commit": impl_head, "path_relative": "src/xarm_ros2/xarm_moveit_config/config/xarm_gripper/joint_limits.yaml", "repo_path": str(production_root), "sha256": _sha256_bytes(DEFAULT_JOINT_LIMITS_GRIPPER.encode("utf-8"))},
                "kinematics": {"commit": impl_head, "path_relative": "src/xarm_ros2/xarm_moveit_config/config/xarm7/kinematics.yaml", "repo_path": str(production_root), "sha256": _sha256_bytes(DEFAULT_KINEMATICS.encode("utf-8"))},
            },
            "normalization": {
                "groups": {"arm": "xarm7", "gripper": "xarm_gripper"},
                "ordered_joints": list(ARM_JOINTS) + [GRIPPER_JOINT],
            },
        },
        "production_overlay": {
            "launch_file": "manipulation_planning_task_only.launch.py",
            "sim_compatibility_parameters_literal_false": {
                "use_cumotion_goalset": False,
                "use_cumotion_object_attachment": False,
                "use_cumotion_straight_approach": False,
                "esdf_freshness_wait_enabled": False,
            },
            "simulator_overlay_provider_set": {
                "executables": sorted(provider_executables(provider)),
            },
            "task_owned_lifecycle": "pick_and_place creates and owns pick_and_place/object_mesh itself",
        },
        "provider_manifest": {
            "canonical_self_hash": provider.get("provider_manifest_sha256"),
            "raw_sha256": _sha256_bytes(provider_raw),
        },
        "repositories": {
            "production": {
                "implementation_identity": impl_head,
                "path": str(production_root),
                "dirty_policy": "read-only runtime input",
            },
            "simulator": {
                "implementation_identity": "f34de5f4cd472e2dbb50d65eb53e089bb1c84891",
                "path": str(simulator_root),
            },
        },
        "ros_policy": {
            "distro": "humble",
            "rmw_implementation": "rmw_fastrtps_cpp",
            "domain_id": 25,
            "dds_profiles": {"local": "Fast DDS shared memory enabled (no profile override)", "lan": "config/fastdds-lan.xml"},
        },
    }


def provider_executables(provider: Mapping[str, Any]) -> list[str]:
    executables: set[str] = set()
    for section in ("persistent_nodes", "one_shot_processes"):
        for entry in provider.get(section, []) or []:
            if isinstance(entry, dict) and isinstance(entry.get("executable"), str):
                executables.add(entry["executable"])
    return sorted(executables)


def _build_model_bundle(production_root: Path, impl_head: str) -> dict[str, Any]:
    contract = {
        "planning_frame": "base_link",
        "tcp_link": "link_tcp",
        "gripper_joint": GRIPPER_JOINT,
        "touch_links": list(TOUCH_LINKS),
        "arm_joints": list(ARM_JOINTS),
        "groups": {
            "xarm7": {"joints": ["world_joint", "base_to_arm_joint"] + list(ARM_JOINTS) + ["joint_eef", "gripper_fix", "joint_tcp"], "links": ["world", "base_link"] + list(TOUCH_LINKS)},
            "xarm_gripper": {"joints": [GRIPPER_JOINT], "links": list(TOUCH_LINKS)},
        },
        "end_effector": {"group": "xarm_gripper", "parent_link": "link_tcp"},
        "kinematics": {"xarm7": {"base_link": "base_link", "tip_link": "link_tcp", "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin"}},
    }
    return {
        "schema_version": 1,
        "producer": {"name": "tinker_sim_bridge.model_bundle", "version": "1"},
        "structural_fingerprint": _sha256(contract),
        "contract": contract,
        "artifacts": {
            "joint_limits": {"path_relative": "outputs/ompl-overlay/model-bundle-r2/joint_limits.yaml", "sha256": _sha256_bytes(b"fixture joint limits\n")},
            "simulator_full_urdf": {"path_relative": "artifacts/robot/tinker2/36ac0317025d20a5/robot.urdf", "sha256": _sha256_bytes(b"fixture full urdf\n")},
        },
        "normalization": {
            "groups": {"arm": "xarm7", "gripper": "xarm_gripper"},
            "ordered_joints": list(ARM_JOINTS) + [GRIPPER_JOINT],
            "mount": {"parent": "world", "child": "base_link"},
        },
        "production_source_commits": {
            "arm_joint_limits": {"commit": impl_head, "path_relative": "src/xarm_ros2/xarm_moveit_config/config/xarm7/joint_limits.yaml", "repo_path": str(production_root), "sha256": _sha256_bytes(DEFAULT_JOINT_LIMITS_XARM7.encode("utf-8"))},
            "gripper_joint_limits": {"commit": impl_head, "path_relative": "src/xarm_ros2/xarm_moveit_config/config/xarm_gripper/joint_limits.yaml", "repo_path": str(production_root), "sha256": _sha256_bytes(DEFAULT_JOINT_LIMITS_GRIPPER.encode("utf-8"))},
            "kinematics": {"commit": impl_head, "path_relative": "src/xarm_ros2/xarm_moveit_config/config/xarm7/kinematics.yaml", "repo_path": str(production_root), "sha256": _sha256_bytes(DEFAULT_KINEMATICS.encode("utf-8"))},
        },
    }


def _build_provider_manifest() -> dict[str, Any]:
    manifest = {
        "schema_version": 1,
        "owner": "tinker_sim_bridge",
        "persistent_nodes": [
            {"cardinality": 1, "executable": "command_gateway", "key": "command_gateway", "package": "tinker_sim_bridge"},
            {"cardinality": 1, "executable": "contract_guard", "key": "contract_guard", "package": "tinker_sim_bridge"},
            {"cardinality": 1, "executable": "fixture_planning_scene", "key": "fixture_planning_scene", "package": "tinker_sim_bridge"},
            {"cardinality": 1, "executable": "gripper_facade", "key": "gripper_facade", "package": "tinker_sim_bridge"},
            {"cardinality": 1, "executable": "integrated_readiness", "key": "integrated_readiness", "package": "tinker_sim_bridge"},
            {"cardinality": 1, "executable": "move_group", "key": "move_group", "package": "moveit_ros_move_group"},
            {"cardinality": 1, "executable": "pan_tilt_facade", "key": "pan_tilt_facade", "package": "tinker_sim_bridge"},
            {"cardinality": 1, "executable": "physics_ready_gate", "key": "physics_ready_gate", "package": "tinker_sim_bridge"},
            {"cardinality": 1, "executable": "pick_and_place", "key": "pick_and_place", "package": "pick_and_place"},
            {"cardinality": 1, "executable": "robot_state_publisher", "key": "robot_state_publisher", "package": "robot_state_publisher"},
            {"cardinality": 1, "executable": "ros2_control_node", "key": "controller_manager", "package": "controller_manager"},
            {"cardinality": 1, "executable": "safety_supervisor", "key": "safety_supervisor", "package": "tinker_sim_bridge"},
            {"cardinality": 1, "executable": "truth_evaluator", "key": "truth_evaluator", "package": "tinker_sim_bridge"},
            {"cardinality": 1, "executable": "xarm_facade", "key": "xarm_facade", "package": "tinker_sim_bridge"},
        ],
        "one_shot_processes": [
            {"cardinality": 1, "executable": "scenario_runner", "key": "scenario_runner", "package": "tinker_sim_bridge"},
            {"cardinality": 1, "executable": "controller_reconciler", "key": "controller_reconciler", "package": "tinker_sim_bridge"},
        ],
        "controller_resources": [
            {"cardinality": 1, "resource_name": "joint_state_broadcaster", "controller_type": "joint_state_broadcaster/JointStateBroadcaster"},
            {"cardinality": 1, "resource_name": "xarm7_traj_controller", "controller_type": "joint_trajectory_controller/JointTrajectoryController"},
        ],
        "publishers": [
            {"cardinality": 1, "topic": "/joint_states", "type": "sensor_msgs/msg/JointState"},
            {"cardinality": 1, "topic": "/isaac_joint_commands", "type": "sensor_msgs/msg/JointState"},
        ],
    }
    manifest["provider_manifest_sha256"] = _sha256({k: v for k, v in manifest.items() if k != "provider_manifest_sha256"})
    return manifest


@dataclass(frozen=True)
class StaticContractFixture:
    simulator_root: Path
    production_root: Path
    source_lock_manifest: Path
    config: dict[str, object]
    impl_head: str


def make_static_contract_fixture(
    tmp_path: Path,
    *,
    production_file_overrides: Mapping[str, str] | None = None,
    manifest_status: str = "verified-pass",
    production_status: str = "verified-pass",
    with_qualification_policy: bool = True,
    scenario_overrides: Mapping[str, Mapping[str, Any]] | None = None,
    bundle_overrides: Mapping[str, Any] | None = None,
    provider_overrides: Mapping[str, Any] | None = None,
    overlay_overrides: Mapping[str, Any] | None = None,
    config_overrides: Mapping[str, Any] | None = None,
) -> StaticContractFixture:
    simulator_root = (tmp_path / "simulator").resolve()
    production_root = (tmp_path / "production").resolve()

    # ---- production repo ---------------------------------------------------
    _git_init(production_root)
    files = dict(PRODUCTION_FILES)
    if production_file_overrides:
        files.update(production_file_overrides)
    for rel, content in files.items():
        _write(production_root, rel, content)
    _write(production_root, "README.md", "production fixture\n")
    _commit(production_root, "feat: production planning task-only launch")
    impl_head = _git_head(production_root)

    # ---- simulator tree -----------------------------------------------------
    config = _read_json(REAL_CONFIG)
    config["source_lock_policies"] = {
        "simulator_overlay": "integration/source-locks.json",
        "production": str(production_root / "integration/source-locks.json"),
        "qualification_tooling": "integration/integrated-qualification-source-lock.json",
    }
    if config_overrides:
        for key, value in config_overrides.items():
            config[key] = value
    _write_json_canonical(simulator_root / CONFIG_REL, config)

    scenario_files: dict[str, Any] = {sid: _scenario_doc(sid) for sid in SCENARIO_OWNED}
    if scenario_overrides:
        for sid, over in scenario_overrides.items():
            scenario_files[sid] = _deep_merge(scenario_files[sid], over)
    for sid, raw in scenario_files.items():
        _write_json_canonical(simulator_root / "simulation/scenarios" / (sid + ".json"), raw)
    # A non-integrated scenario legitimately coexists (no planning_scene, not in
    # the configured set) and must never fail the fixture-ownership check.
    _write_json_canonical(
        simulator_root / "simulation/scenarios" / "qualification-free-space.json",
        {"schema_version": 2, "id": "qualification-free-space", "qualification_gate": "free-space", "seed": 7, "world": {"mode": "current"}},
    )
    _write_json_canonical(
        simulator_root / "simulation/scenarios" / "reception-seat-assignment.json",
        {"schema_version": 2, "id": "reception-seat-assignment", "qualification_gate": None, "seed": 3},
    )

    provider = _build_provider_manifest()
    if provider_overrides:
        provider = _deep_merge(provider, provider_overrides)
        if "provider_manifest_sha256" not in provider_overrides:
            provider["provider_manifest_sha256"] = _sha256({k: v for k, v in provider.items() if k != "provider_manifest_sha256"})
    _write_json_canonical(
        simulator_root / "ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json", provider
    )

    bundle = _build_model_bundle(production_root, impl_head)
    if bundle_overrides:
        bundle.update(bundle_overrides)
        if "contract" in bundle_overrides:
            bundle["structural_fingerprint"] = _sha256(bundle["contract"])
    _write_json_canonical(simulator_root / "outputs/ompl-overlay/model-bundle-r2/model-bundle.json", bundle)
    _write(simulator_root, "outputs/ompl-overlay/model-bundle-r2/joint_limits.yaml", b"fixture joint limits\n")
    _write(simulator_root, "artifacts/robot/tinker2/36ac0317025d20a5/robot.urdf", b"fixture full urdf\n")

    overlay = _build_overlay_contract(simulator_root, production_root, impl_head)
    if overlay_overrides:
        for key, value in overlay_overrides.items():
            overlay[key] = value
    _write_json_canonical(simulator_root / "integration/ompl-overlay-contract.json", overlay)

    # ---- source-lock manifest ------------------------------------------------
    policies = config["source_lock_policies"]
    records: dict[str, object] = {
        "simulator_overlay": {
            "repository": "simulator_overlay",
            "status": "verified-pass",
            "policy_path": "integration/source-locks.json",
            "implementation_head": "490f907831d9f6f06242e0d151ac014547973d6e",
            "resolved_policy_commit": "ab8cf7e9645b1e019aba81e2c7923177ba13d1ac",
        },
        "production": {
            "repository": "production",
            "status": production_status,
            "policy_path": "integration/source-locks.json",
            "implementation_head": impl_head,
            "resolved_policy_commit": impl_head,
        },
        "qualification_tooling": {
            "repository": "qualification_tooling",
            "status": "verified-pass" if with_qualification_policy else "evidence-invalid",
            "policy_path": "integration/integrated-qualification-source-lock.json",
            "implementation_head": "fa79ef40999d5251d75e71672db325f4874c5243",
            "resolved_policy_commit": "fa79ef40999d5251d75e71672db325f4874c5243",
        },
    }
    manifest = {
        "schema_version": 1,
        "status": manifest_status,
        "repositories": sorted(policies),
        **records,
    }
    manifest_path = tmp_path / MANIFEST_NAME
    _write_json_canonical(manifest_path, manifest)

    return StaticContractFixture(
        simulator_root=simulator_root,
        production_root=production_root,
        source_lock_manifest=manifest_path,
        config=config,
        impl_head=impl_head,
    )


def _run_static_fixture(tmp_path: Path, **kwargs: Any):
    fixture = make_static_contract_fixture(tmp_path, **kwargs)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    return report, fixture


def _failed_reasons(report) -> list[str]:
    return [reason for check in report.checks for reason in check.reasons]


def _check(report, name: str):
    return next(check for check in report.checks if check.name == name)


# ---------------------------------------------------------------------------
# positive path
# ---------------------------------------------------------------------------
def test_all_static_contracts_pass(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    assert report.status == "verified-pass"
    assert all(check.passed for check in report.checks)
    assert set(report.source_identities) == {"simulator_overlay", "production", "qualification_tooling"}
    names = [check.name for check in report.checks]
    assert names == [
        "model-fingerprint",
        "controller-mapping",
        "selected-launch",
        "provider-cardinality",
        "fixture-ownership",
        "action-lifecycle",
        "scene-and-collision-safety",
        "source-identities",
        "transport-contract",
    ]


def test_real_root_evidence_invalid_only_source_identities_fail(tmp_path):
    """F4: a manifest whose only deficit is the absent qualification-tooling
    policy yields aggregate evidence-invalid; the eight non-source-identity
    checks still pass."""
    report, fixture = _run_static_fixture(
        tmp_path,
        manifest_status="evidence-invalid",
        with_qualification_policy=False,
        production_status="verified-pass",
    )
    assert report.status == "evidence-invalid"
    for check in report.checks:
        if check.name == "source-identities":
            assert not check.passed
        else:
            assert check.passed, "{} failed: {}".format(check.name, check.reasons)


def test_manifest_missing_is_evidence_invalid(tmp_path):
    fixture = make_static_contract_fixture(tmp_path)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=tmp_path / "does-not-exist.json",
        config=fixture.config,
    )
    assert report.status == "evidence-invalid"


# ---------------------------------------------------------------------------
# F1 model identity mutations
# ---------------------------------------------------------------------------
def test_srdf_touch_link_reorder_fails(tmp_path):
    mutated = DEFAULT_SRDF.replace(
        '        <link name="${prefix}left_finger" />\n        <link name="${prefix}left_inner_knuckle" />',
        '        <link name="${prefix}left_inner_knuckle" />\n        <link name="${prefix}left_finger" />',
    )
    assert mutated != DEFAULT_SRDF
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_SRDF_REL: mutated})
    assert not _check(report, "model-fingerprint").passed
    assert any("touch-link order" in reason for reason in _check(report, "model-fingerprint").reasons)


def test_srdf_touch_link_missing_fails(tmp_path):
    mutated = DEFAULT_SRDF.replace('        <link name="${prefix}link_tcp" />\n', "")
    assert mutated != DEFAULT_SRDF
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_SRDF_REL: mutated})
    assert not _check(report, "model-fingerprint").passed
    assert any("touch-link order" in reason for reason in _check(report, "model-fingerprint").reasons)


def test_srdf_end_effector_parent_wrong_fails(tmp_path):
    mutated = DEFAULT_SRDF.replace(
        'parent_link="${prefix}link_tcp"', 'parent_link="${prefix}link7"'
    )
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_SRDF_REL: mutated})
    assert any("parent_link" in reason for reason in _check(report, "model-fingerprint").reasons)


def test_bundle_structural_fingerprint_drift_fails(tmp_path):
    report, _ = _run_static_fixture(tmp_path, bundle_overrides={"structural_fingerprint": "f" * 64})
    assert not _check(report, "model-fingerprint").passed
    assert any("fingerprint" in reason.lower() for reason in _check(report, "model-fingerprint").reasons)


def test_bundle_artifact_hash_drift_fails(tmp_path):
    report, _ = _run_static_fixture(
        tmp_path,
        bundle_overrides={"artifacts": {"joint_limits": {"path_relative": "outputs/ompl-overlay/model-bundle-r2/joint_limits.yaml", "sha256": "e" * 64}}},
    )
    assert any("sha256 mismatch" in reason for reason in _check(report, "model-fingerprint").reasons)


def test_bundle_touch_links_permutation_fails(tmp_path):
    contract = {
        "planning_frame": "base_link",
        "tcp_link": "link_tcp",
        "gripper_joint": GRIPPER_JOINT,
        "touch_links": ["link_tcp"] + list(TOUCH_LINKS[:-1]),
        "arm_joints": list(ARM_JOINTS),
        "groups": {"xarm7": {"joints": ["world_joint"] + list(ARM_JOINTS) + ["joint_eef"]}, "xarm_gripper": {"joints": [GRIPPER_JOINT]}},
        "end_effector": {"group": "xarm_gripper", "parent_link": "link_tcp"},
        "kinematics": {"xarm7": {"base_link": "base_link", "tip_link": "link_tcp"}},
    }
    report, _ = _run_static_fixture(tmp_path, bundle_overrides={"contract": contract, "structural_fingerprint": _sha256(contract)})
    assert any("touch_links" in reason for reason in _check(report, "model-fingerprint").reasons)


# ---------------------------------------------------------------------------
# F2.2 authoritative overlay source-commit binding mutations
# ---------------------------------------------------------------------------
def test_overlay_source_commits_empty_fails(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    overlay["model_bundle"]["production_source_commits"] = {}
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert not _check(report, "model-fingerprint").passed
    assert any("non-empty" in reason for reason in _check(report, "model-fingerprint").reasons)


def test_overlay_source_commit_malformed_entry_fails(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    del overlay["model_bundle"]["production_source_commits"]["arm_joint_limits"]["sha256"]
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert not _check(report, "model-fingerprint").passed
    assert any("sha256" in reason for reason in _check(report, "model-fingerprint").reasons)


def test_overlay_source_commit_wrong_digest_fails(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    overlay["model_bundle"]["production_source_commits"]["arm_joint_limits"]["sha256"] = "f" * 64
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert not _check(report, "model-fingerprint").passed
    assert any("sha256 mismatch" in reason for reason in _check(report, "model-fingerprint").reasons)


def test_overlay_source_commit_missing_blob_fails(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    overlay["model_bundle"]["production_source_commits"]["arm_joint_limits"]["path_relative"] = (
        "src/xarm_ros2/xarm_moveit_config/config/xarm7/nonexistent.yaml"
    )
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert not _check(report, "model-fingerprint").passed
    assert any("not found" in reason for reason in _check(report, "model-fingerprint").reasons)


def test_overlay_source_commit_wrong_repo_fails(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    overlay["model_bundle"]["production_source_commits"]["arm_joint_limits"]["repo_path"] = str(
        fixture.simulator_root
    )
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert not _check(report, "model-fingerprint").passed
    assert any("does not exist" in reason for reason in _check(report, "model-fingerprint").reasons)


def test_overlay_source_commit_non_ancestor_fails(tmp_path):
    """A real commit that exists in the production repo but is not an ancestor
    of the manifest implementation head fails the pinned-ancestry check."""
    report, fixture = _run_static_fixture(tmp_path)
    prod = fixture.production_root
    _git(prod, "checkout", "-q", "--orphan", "fixture-orphan")
    _write(prod, "README.md", "orphan drift\n")
    _git(prod, "add", "-A")
    proc = _git(prod, "commit", "-q", "-m", "chore: orphan non-ancestor commit")
    assert proc.returncode == 0, proc.stderr
    orphan = _git_head(prod)
    _git(prod, "checkout", "-q", "-f", fixture.impl_head)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    overlay["model_bundle"]["production_source_commits"]["arm_joint_limits"]["commit"] = orphan
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert not _check(report, "model-fingerprint").passed
    assert any("not an ancestor" in reason for reason in _check(report, "model-fingerprint").reasons)


def test_overlay_source_commit_working_tree_drift_ignored(tmp_path):
    """F2.2: production working-tree drift must NOT affect an immutable
    ``git show <commit>:<path>`` binding.  The check still passes."""
    report, fixture = _run_static_fixture(tmp_path)
    _write(
        fixture.production_root,
        "src/xarm_ros2/xarm_moveit_config/config/xarm7/joint_limits.yaml",
        "# drifted working copy, never committed\n",
    )
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert _check(report, "model-fingerprint").passed
    assert _check(report, "source-identities").passed


# ---------------------------------------------------------------------------
# F1/F2 controller mapping mutations
# ---------------------------------------------------------------------------
def test_controller_endpoint_wrong_fails(tmp_path):
    mutated = DEFAULT_XARM7_CONTROLLERS.replace("follow_joint_trajectory", "wrong_endpoint")
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_XARM7_CONTROLLERS_REL: mutated})
    assert not _check(report, "controller-mapping").passed
    assert any("action_ns" in reason for reason in _check(report, "controller-mapping").reasons)


def test_controller_joint_list_wrong_fails(tmp_path):
    mutated = DEFAULT_XARM7_CONTROLLERS.replace("    - joint7\n", "    - joint6\n")
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_XARM7_CONTROLLERS_REL: mutated})
    assert any("seven-joint" in reason for reason in _check(report, "controller-mapping").reasons)


def test_gripper_controller_joint_wrong_fails(tmp_path):
    mutated = DEFAULT_GRIPPER_CONTROLLERS.replace("drive_joint", "gripper_joint")
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_GRIPPER_CONTROLLERS_REL: mutated})
    assert any("drive_joint" in reason for reason in _check(report, "controller-mapping").reasons)


# ---------------------------------------------------------------------------
# F1.3 action lifecycle mutations
# ---------------------------------------------------------------------------
def test_detached_thread_fails(tmp_path):
    mutated = DEFAULT_PICK_AND_PLACE_CPP + "\n// detached worker\nstd::thread().detach();\n"
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_PICK_AND_PLACE_CPP_REL: mutated})
    assert not _check(report, "action-lifecycle").passed
    assert any("detached" in reason for reason in _check(report, "action-lifecycle").reasons)


def test_missing_runtime_shutdown_fails(tmp_path):
    # The destructor body must call motion_runtime_.shutdown(...); removing it
    # from the actual destructor (not a helper) fails the structural binding.
    mutated = DEFAULT_PICK_AND_PLACE_CPP.replace("  motion_runtime_.shutdown(shutdown_deadline);\n", "")
    assert mutated != DEFAULT_PICK_AND_PLACE_CPP
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_PICK_AND_PLACE_CPP_REL: mutated})
    assert any("motion_runtime_.shutdown" in reason for reason in _check(report, "action-lifecycle").reasons)


def test_unjoined_executor_thread_fails(tmp_path):
    mutated = DEFAULT_PICK_AND_PLACE_CPP.replace("if (executor_thread_.joinable()) executor_thread_.join();", "")
    assert mutated != DEFAULT_PICK_AND_PLACE_CPP
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_PICK_AND_PLACE_CPP_REL: mutated})
    assert any("join" in reason for reason in _check(report, "action-lifecycle").reasons)


def test_missing_result_field_write_fails(tmp_path):
    mutated = DEFAULT_ACTION_EXECUTION_CPP.replace("result->stage =", "result->unused_stage =")
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={"src/pick_and_place/src/action_execution.cpp": mutated})
    assert any("result->stage" in reason for reason in _check(report, "action-lifecycle").reasons)


def test_missing_action_specific_success_fails(tmp_path):
    mutated = DEFAULT_ACTION_EXECUTION_CPP.replace("result->success =", "result->finished =")
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={"src/pick_and_place/src/action_execution.cpp": mutated})
    assert any("result->success" in reason for reason in _check(report, "action-lifecycle").reasons)


# ---------------------------------------------------------------------------
# F1.4 scene / collision safety mutations
# ---------------------------------------------------------------------------
def test_sim_cleanup_early_return_removed_fails(tmp_path):
    mutated = DEFAULT_PACKAGE_UTILS_CPP.replace(
        "  if (execution_profile_ == ExecutionProfile::SimOmpl) {\n    if (keepout_enabled_.load()) apply_floor_keepout(true);\n    return;\n  }\n",
        "",
    )
    assert mutated != DEFAULT_PACKAGE_UTILS_CPP
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_PACKAGE_UTILS_CPP_REL: mutated})
    assert not _check(report, "scene-and-collision-safety").passed
    assert any("ExecutionProfile::SimOmpl" in reason for reason in _check(report, "scene-and-collision-safety").reasons)


def test_sim_lift_collision_disabled_fails(tmp_path):
    mutated = DEFAULT_SCENE_OWNERSHIP_CPP.replace("backend.execute_lift(ctx, true,", "backend.execute_lift(ctx, false,")
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_SCENE_OWNERSHIP_CPP_REL: mutated})
    assert not _check(report, "scene-and-collision-safety").passed
    assert any("execute_lift(ctx, true" in reason for reason in _check(report, "scene-and-collision-safety").reasons)


def test_missing_avoid_collisions_forwarding_fails(tmp_path):
    mutated = DEFAULT_PICK_AND_PLACE_CPP.replace("request.avoid_collisions = avoid_collisions;", "request.avoid_collisions = true;")
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_PICK_AND_PLACE_CPP_REL: mutated})
    assert any("avoid_collisions" in reason for reason in _check(report, "scene-and-collision-safety").reasons)


def test_missing_obstruction_gate_fails(tmp_path):
    mutated = DEFAULT_SCENE_OWNERSHIP_CPP.replace("!close_result.confirms_obstruction()", "true")
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_SCENE_OWNERSHIP_CPP_REL: mutated})
    assert any("confirms_obstruction" in reason for reason in _check(report, "scene-and-collision-safety").reasons)


# ---------------------------------------------------------------------------
# F2.1 structural C++ mutation coverage (branch swap / spoof / move)
# ---------------------------------------------------------------------------
def test_sim_hw_execute_lift_branch_swap_fails(tmp_path):
    """Swapping collision-aware/collision-disabled lift across the Sim/Hardware
    branches must fail (the Sim branch is now collision-disabled)."""
    marker_hw = "__SWAP_HW__"
    marker_sim = "__SWAP_SIM__"
    mutated = DEFAULT_SCENE_OWNERSHIP_CPP
    mutated = mutated.replace(
        "result = backend.execute_lift(ctx, false, post_close_stage, timeout);", marker_hw
    )
    mutated = mutated.replace(
        "result = backend.execute_lift(ctx, true, post_close_stage, timeout);", marker_sim
    )
    mutated = mutated.replace(
        marker_hw, "result = backend.execute_lift(ctx, true, post_close_stage, timeout);"
    )
    mutated = mutated.replace(
        marker_sim, "result = backend.execute_lift(ctx, false, post_close_stage, timeout);"
    )
    assert "backend.execute_lift(ctx, true, post_close_stage, timeout);" in mutated
    assert "backend.execute_lift(ctx, false, post_close_stage, timeout);" in mutated
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_SCENE_OWNERSHIP_CPP_REL: mutated})
    assert not _check(report, "scene-and-collision-safety").passed
    assert any("must not call collision-aware" in reason for reason in _check(report, "scene-and-collision-safety").reasons)
    assert any("must not call collision-disabled" in reason for reason in _check(report, "scene-and-collision-safety").reasons)


def test_sim_lift_string_literal_spoof_fails(tmp_path):
    """A string literal carrying the execute_lift token must not satisfy the
    SimOmpl lift binding (literals are sanitized)."""
    mutated = DEFAULT_SCENE_OWNERSHIP_CPP.replace(
        "result = backend.execute_lift(ctx, true, post_close_stage, timeout);",
        'const char *spoof = "result = backend.execute_lift(ctx, true, post_close_stage, timeout);";',
    )
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_SCENE_OWNERSHIP_CPP_REL: mutated})
    assert not _check(report, "scene-and-collision-safety").passed
    assert any("must call collision-aware execute_lift" in reason for reason in _check(report, "scene-and-collision-safety").reasons)


def test_sim_lift_raw_string_spoof_fails(tmp_path):
    """A raw string literal carrying the execute_lift token must not satisfy the
    SimOmpl lift binding."""
    mutated = DEFAULT_SCENE_OWNERSHIP_CPP.replace(
        "result = backend.execute_lift(ctx, true, post_close_stage, timeout);",
        'const char *spoof = R"(result = backend.execute_lift(ctx, true, post_close_stage, timeout);)";',
    )
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_SCENE_OWNERSHIP_CPP_REL: mutated})
    assert not _check(report, "scene-and-collision-safety").passed
    assert any("must call collision-aware execute_lift" in reason for reason in _check(report, "scene-and-collision-safety").reasons)


def test_sim_lift_if0_spoof_fails(tmp_path):
    """Parking the required execute_lift(ctx, true, ...) call inside a dead
    #if 0 block must not satisfy the SimOmpl lift binding."""
    mutated = DEFAULT_SCENE_OWNERSHIP_CPP.replace(
        "result = backend.execute_lift(ctx, true, post_close_stage, timeout);",
        "#if 0\nresult = backend.execute_lift(ctx, true, post_close_stage, timeout);\n#endif",
    )
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_SCENE_OWNERSHIP_CPP_REL: mutated})
    assert not _check(report, "scene-and-collision-safety").passed
    assert any("must not contain conditional-preprocessor" in reason for reason in _check(report, "scene-and-collision-safety").reasons)


def test_clean_planning_scene_string_spoof_fails(tmp_path):
    """A 'return' string literal before the hardware cleanup in the Sim branch
    must not satisfy the early-return binding (it is sanitized)."""
    mutated = DEFAULT_PACKAGE_UTILS_CPP.replace(
        "    return;\n",
        '    ROS_INFO("sim return early");\n    applyPlanningScene(planning_scene_msg);\n',
    )
    assert mutated != DEFAULT_PACKAGE_UTILS_CPP
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_PACKAGE_UTILS_CPP_REL: mutated})
    assert not _check(report, "scene-and-collision-safety").passed


def test_clean_planning_scene_if0_spoof_fails(tmp_path):
    """A dead #if 0 block carrying the SimOmpl early return must not satisfy the
    early-return binding (the whole load-bearing body is rejected)."""
    mutated = DEFAULT_PACKAGE_UTILS_CPP.replace(
        "  if (execution_profile_ == ExecutionProfile::SimOmpl) {\n    if (keepout_enabled_.load()) apply_floor_keepout(true);\n    return;\n  }\n",
        "  #if 0\n  if (execution_profile_ == ExecutionProfile::SimOmpl) {\n    return;\n  }\n  #endif\n",
    )
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_PACKAGE_UTILS_CPP_REL: mutated})
    assert not _check(report, "scene-and-collision-safety").passed


def test_move_straight_required_assignment_moved_fails(tmp_path):
    """The avoid_collisions assignment must live in GraspNode::move_straight's
    own body; moving it to a helper function must not satisfy the binding."""
    mutated = DEFAULT_PICK_AND_PLACE_CPP.replace(
        "  request.avoid_collisions = avoid_collisions;\n",
        "  forward_avoid_collisions(request, avoid_collisions);\n",
    ) + """
void forward_avoid_collisions(moveit_msgs::srv::GetCartesianPath::Request &request, bool avoid_collisions) {
  request.avoid_collisions = avoid_collisions;
}
"""
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_PICK_AND_PLACE_CPP_REL: mutated})
    assert not _check(report, "scene-and-collision-safety").passed
    assert any("avoid_collisions" in reason for reason in _check(report, "scene-and-collision-safety").reasons)


def test_missing_destructor_state_validity_reset_fails(tmp_path):
    """The destructor body must reset the state-validity client; removing that
    reset from the actual destructor fails the structural binding."""
    mutated = DEFAULT_PICK_AND_PLACE_CPP.replace("  check_state_validity_client_.reset();\n", "")
    assert mutated != DEFAULT_PICK_AND_PLACE_CPP
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_PICK_AND_PLACE_CPP_REL: mutated})
    assert not _check(report, "action-lifecycle").passed
    assert any("state-validity client" in reason for reason in _check(report, "action-lifecycle").reasons)


def test_destructor_shutdown_string_spoof_fails(tmp_path):
    """The destructor's motion_runtime_.shutdown(...) must be executable code;
    a string literal in the destructor body must not satisfy the binding."""
    mutated = DEFAULT_PICK_AND_PLACE_CPP.replace(
        "  motion_runtime_.shutdown(shutdown_deadline);\n",
        '  const char *spoof = "motion_runtime_.shutdown(shutdown_deadline);";\n',
    )
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_PICK_AND_PLACE_CPP_REL: mutated})
    assert not _check(report, "action-lifecycle").passed
    assert any("motion_runtime_.shutdown" in reason for reason in _check(report, "action-lifecycle").reasons)


def test_builder_missing_required_field_fails(tmp_path):
    """One builder missing a required result field fails its schema binding
    (the write in a different builder does not count)."""
    mutated = DEFAULT_ACTION_EXECUTION_CPP.replace(
        "void make_fold_result(Result *result, ResultStatus status, int16_t stage, std::string message) {\n  result->success = status == ResultStatus::Success;\n  result->stage = status == ResultStatus::Success ? 0 : stage;\n  result->status = static_cast<int16_t>(status);\n  result->error_msg = std::move(message);\n}",
        "void make_fold_result(Result *result, ResultStatus status, int16_t stage, std::string message) {\n  result->success = status == ResultStatus::Success;\n  result->status = static_cast<int16_t>(status);\n}",
    )
    assert mutated != DEFAULT_ACTION_EXECUTION_CPP
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_ACTION_EXECUTION_CPP_REL: mutated})
    assert not _check(report, "action-lifecycle").passed
    assert any("make_fold_result" in reason and "error_msg" in reason for reason in _check(report, "action-lifecycle").reasons)


def test_builder_required_field_moved_to_helper_fails(tmp_path):
    """A required result-field write moved out of the builder into a shared
    helper must not satisfy the builder's schema binding."""
    mutated = DEFAULT_ACTION_EXECUTION_CPP.replace(
        "void make_cartesian_result(Result *result, ResultStatus status, int16_t stage, std::string message) {\n  result->success = status == ResultStatus::Success;\n  result->stage = status == ResultStatus::Success ? 0 : stage;\n  result->status = static_cast<int16_t>(status);\n  result->error_msg = std::move(message);\n}",
        "void make_cartesian_result(Result *result, ResultStatus status, int16_t stage, std::string message) {\n  write_task_result(result, status, stage, std::move(message));\n}",
    )
    assert mutated != DEFAULT_ACTION_EXECUTION_CPP
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_ACTION_EXECUTION_CPP_REL: mutated})
    assert not _check(report, "action-lifecycle").passed
    assert any("make_cartesian_result" in reason for reason in _check(report, "action-lifecycle").reasons)


def test_missing_coordinator_worker_join_fails(tmp_path):
    """The coordinator must join the per-transaction worker; dropping the join
    from coordinator_main fails the managed-runtime binding."""
    mutated = DEFAULT_ACTION_RUNTIME_CPP.replace(
        "    if (transaction->worker.joinable()) transaction->worker.join();\n",
        "    (void)transaction;\n",
    )
    assert mutated != DEFAULT_ACTION_RUNTIME_CPP
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_ACTION_RUNTIME_CPP_REL: mutated})
    assert not _check(report, "action-lifecycle").passed
    assert any("coordinator_main" in reason for reason in _check(report, "action-lifecycle").reasons)


def test_detach_string_literal_spoof_passes(tmp_path):
    """A .detach() inside a string literal is sanitized and must NOT trip the
    forbidden-token scan (literals cannot satisfy or trigger semantic checks)."""
    mutated = DEFAULT_ACTION_RUNTIME_CPP.replace(
        "    if (transaction->worker.joinable()) transaction->worker.join();\n",
        '    const char *spoof = "std::thread().detach();";\n    if (transaction->worker.joinable()) transaction->worker.join();\n',
    )
    assert mutated != DEFAULT_ACTION_RUNTIME_CPP
    report, _ = _run_static_fixture(tmp_path, production_file_overrides={PROD_ACTION_RUNTIME_CPP_REL: mutated})
    assert _check(report, "action-lifecycle").passed


# ---------------------------------------------------------------------------
# F2 fixture ownership mutations
# ---------------------------------------------------------------------------
def test_non_integrated_scenario_coexists_without_failure(tmp_path):
    report, _ = _run_static_fixture(tmp_path)
    assert _check(report, "fixture-ownership").passed


def test_configured_scenario_missing_fails(tmp_path):
    sid = "qualification-moveit-plan-joint"
    report, fixture = _run_static_fixture(tmp_path)
    (fixture.simulator_root / "simulation/scenarios" / (sid + ".json")).unlink()
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert any("configured scenario missing" in reason for reason in _check(report, "fixture-ownership").reasons)


def test_extra_configured_scenario_fails(tmp_path):
    # Inject a genuinely new scenario into stage C so the unique count exceeds 17.
    fixture = make_static_contract_fixture(tmp_path)
    extra_id = "qualification-moveit-plan-extra"
    config = dict(fixture.config)
    config["stages"] = json.loads(_canonical(fixture.config["stages"]))
    config["stages"]["C"]["scenarios"] = list(config["stages"]["C"]["scenarios"]) + [extra_id]
    raw = _scenario_doc("qualification-moveit-plan-joint")
    raw["id"] = extra_id
    raw["qualification_gate"] = extra_id
    _write_json_canonical(fixture.simulator_root / "simulation/scenarios" / (extra_id + ".json"), raw)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=config,
    )
    assert any("exactly 17" in reason for reason in _check(report, "fixture-ownership").reasons)


def test_overlay_scenario_set_missing_fails(tmp_path):
    """F2.3: a configured C/D/E scenario absent from the overlay scenarios map
    fails the exact set symmetry."""
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    del overlay["scenarios"]["qualification-pick-place-positive"]
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert not _check(report, "fixture-ownership").passed
    assert any("missing:" in reason for reason in _check(report, "fixture-ownership").reasons)


def test_overlay_scenario_set_extra_fails(tmp_path):
    """F2.3: an overlay scenario entry that is not configured fails the exact
    set symmetry before per-scenario comparison."""
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    overlay["scenarios"]["qualification-overlay-extra"] = {
        "scenario_declaration_sha256": "0" * 64,
        "planning_scene": {
            "owned_ids": ["sim_fixture/extra"],
            "revision": "extra-revision",
            "revision_digest": "1" * 64,
            "frame_id": "base_link",
            "target_source_id": "sim_fixture/extra",
            "target_handoff": "pick_and_place/object_mesh",
        },
    }
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert not _check(report, "fixture-ownership").passed
    assert any("extra:" in reason for reason in _check(report, "fixture-ownership").reasons)


def test_ownership_drift_fails(tmp_path):
    sid = "qualification-pick-place-positive"
    owned = list(SCENARIO_OWNED[sid])
    over = {"planning_scene": {"objects": [{"id": oid, "class": "static", "primitive": {"type": "box"}} for oid in owned[:-1]]}}
    report, _ = _run_static_fixture(tmp_path, scenario_overrides={sid: over})
    assert any("owned ids != integrated.expected_scene.owned_ids" in reason for reason in _check(report, "fixture-ownership").reasons)


def test_target_not_in_owned_set_fails(tmp_path):
    sid = "qualification-moveit-plan-joint"
    over = {"planning_scene": {"target_source_id": "sim_fixture/foreign_target"}}
    report, _ = _run_static_fixture(tmp_path, scenario_overrides={sid: over})
    assert any("not in the owned set" in reason for reason in _check(report, "fixture-ownership").reasons)


def test_scenario_declaration_sha256_drift_fails(tmp_path):
    sid = "qualification-moveit-plan-joint"
    report, fixture = _run_static_fixture(tmp_path)
    raw = _read_json(fixture.simulator_root / "simulation/scenarios" / (sid + ".json"))
    raw["qualification_gate"] = "renamed-gate"
    raw["planning_scene"]["revision_digest"] = _sha256({k: v for k, v in raw["planning_scene"].items() if k != "revision_digest"})
    _write_json_canonical(fixture.simulator_root / "simulation/scenarios" / (sid + ".json"), raw)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert any("declaration sha256 differs" in reason for reason in _check(report, "fixture-ownership").reasons)


def test_revision_digest_non_canonical_fails(tmp_path):
    sid = "qualification-moveit-plan-joint"
    over = {"planning_scene": {"revision_digest": "f" * 64}}
    report, _ = _run_static_fixture(tmp_path, scenario_overrides={sid: over})
    assert any("revision_digest is not canonical" in reason for reason in _check(report, "fixture-ownership").reasons)


# ---------------------------------------------------------------------------
# F2 provider manifest / transport mutations
# ---------------------------------------------------------------------------
def test_provider_canonical_hash_drift_fails(tmp_path):
    report, _ = _run_static_fixture(tmp_path, provider_overrides={"provider_manifest_sha256": "d" * 64})
    assert any("canonical self-hash" in reason for reason in _check(report, "provider-cardinality").reasons)


def test_provider_overlay_hash_drift_fails(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    overlay["provider_manifest"]["canonical_self_hash"] = "c" * 64
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert any("canonical_self_hash" in reason for reason in _check(report, "provider-cardinality").reasons)


def test_provider_set_drift_fails(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    overlay["production_overlay"]["simulator_overlay_provider_set"]["executables"].append("extra_provider")
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert any("provider executable set differs" in reason for reason in _check(report, "provider-cardinality").reasons)


def test_runtime_digest_stale_nested_location_fails(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    # Move the digest back into the stale nested public_report_separation location.
    nested = overlay["typed_contract"].pop("runtime_contract_sha256")
    overlay["typed_contract"]["public_report_separation"]["runtime_contract_sha256"] = nested
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert any("must not be nested" in reason for reason in _check(report, "transport-contract").reasons)


def test_runtime_digest_wrong_value_fails(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    overlay["typed_contract"]["runtime_contract_sha256"] = "a" * 64
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert any("runtime_contract_sha256" in reason for reason in _check(report, "transport-contract").reasons)


def test_isaac_joint_commands_qos_drift_fails(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    overlay["typed_contract"]["publishers"]["/isaac_joint_commands"]["depth"] = 10
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert any("depth must be 50" in reason for reason in _check(report, "transport-contract").reasons)


def test_public_one_key_mapping_preserved(tmp_path):
    report, _ = _run_static_fixture(tmp_path)
    assert _check(report, "transport-contract").passed
    details = _check(report, "transport-contract").details
    assert details.get("ros_policy", {}).get("domain_id") == 25


# ---------------------------------------------------------------------------
# F3 / F5 source identity + lock commit mutations
# ---------------------------------------------------------------------------
def test_lock_commit_carrying_source_payload_fails_static(tmp_path):
    """A non-pass source-lock manifest (the observer-level consequence of a lock
    commit carrying a source payload) propagates to aggregate evidence-invalid
    while the structural checks still report their own pass/fail."""
    report, _ = _run_static_fixture(
        tmp_path,
        production_status="verified-fail",
        manifest_status="verified-fail",
    )
    assert report.status == "evidence-invalid"
    assert not _check(report, "source-identities").passed


def test_production_identity_mismatch_fails(tmp_path):
    report, fixture = _run_static_fixture(tmp_path)
    overlay = _read_json(fixture.simulator_root / "integration/ompl-overlay-contract.json")
    overlay["repositories"]["production"]["implementation_identity"] = "0" * 40
    _write_json_canonical(fixture.simulator_root / "integration/ompl-overlay-contract.json", overlay)
    report = validate_static_contracts(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        source_lock_manifest=fixture.source_lock_manifest,
        config=fixture.config,
    )
    assert any("implementation_head differs" in reason for reason in _check(report, "source-identities").reasons)


def test_selected_launch_cumotion_token_fails(tmp_path):
    report, _ = _run_static_fixture(
        tmp_path, production_file_overrides={PROD_LAUNCH_REL: "ros2 launch cumotion.launch.py\n"}
    )
    assert any("cumotion" in reason.lower() for reason in _check(report, "selected-launch").reasons)


def test_unknown_manifest_status_is_evidence_invalid(tmp_path):
    report, _ = _run_static_fixture(tmp_path, manifest_status="bogus")
    assert report.status == "evidence-invalid"


def test_cli_emits_atomic_evidence_files(tmp_path):
    """F4: the CLI writes canonical finite JSON evidence files atomically."""
    import integrated_static_contracts as module

    fixture = make_static_contract_fixture(tmp_path)
    output_dir = tmp_path / "evidence"
    argv = [
        "--simulator-root", str(fixture.simulator_root),
        "--production-root", str(fixture.production_root),
        "--source-lock-manifest", str(fixture.source_lock_manifest),
        "--config", str(fixture.simulator_root / CONFIG_REL),
        "--output", str(output_dir),
    ]
    exit_code = module.main(argv)
    assert exit_code == 0
    for name in ("static-contract.json", "model-fingerprint.json", "source-identities.json"):
        path = output_dir / name
        assert path.is_file()
        data = json.loads(path.read_text(encoding="utf-8"))
        assert isinstance(data, dict)
        assert "schema_version" in data
    static = json.loads((output_dir / "static-contract.json").read_text(encoding="utf-8"))
    assert static["status"] == "verified-pass"
    assert len(static["checks"]) == 9
    # No stray temp files survive an atomic replace.
    leftovers = [p.name for p in output_dir.iterdir() if ".tmp" in p.name or p.name.endswith("~")]
    assert leftovers == []
