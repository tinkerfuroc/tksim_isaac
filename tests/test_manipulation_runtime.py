from __future__ import annotations

import ast
import json
import os
import queue
import re
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "validation"))

from tinker_sim_core.command_mux import (
    JointCommand,
    encode_command_epoch,
    encode_command_frame,
    encode_snapshot_packet,
)
from tinker_sim_isaac.backend import IsaacWholeRobotBackend
from tinker_sim_isaac.target_write_gate import TargetWriteGate
from tinker_sim_isaac.ros_gateway import PhysicsTruthJsonlWriter, RosStandardGateway
from manipulation_qualification import QualificationManifest, QualificationRunner
from run_sim import _content_addressed_tinker_usd, _expected_scenario_objects


class _FakeRobot:
    device = "cpu"
    num_base_dofs = 2

    def __init__(self) -> None:
        self.is_initialized = True
        self.root_view = object()
        self.data = SimpleNamespace(
            joint_names=("drive_joint", "joint1"),
            joint_pos=torch.tensor([[0.25, -0.4]], dtype=torch.float32),
            joint_vel=torch.tensor([[0.1, -0.2]], dtype=torch.float32),
            joint_stiffness=torch.tensor([[200.0, 20000.0]], dtype=torch.float32),
            joint_damping=torch.tensor([[20.0, 1500.0]], dtype=torch.float32),
            joint_effort_limits=torch.tensor([[12.0, 30.0]], dtype=torch.float32),
            gravity_compensation_forces=torch.tensor(
                [[0.1, 0.2, 0.3, 1.5]], dtype=torch.float32
            ),
            applied_torque=torch.tensor([[1.5, -1.0]], dtype=torch.float32),
            root_pos_w=torch.tensor([[0.0, 0.0, 0.4]], dtype=torch.float32),
            root_quat_w=torch.tensor([[0.1, 0.2, 0.3, 0.4]], dtype=torch.float32),
            root_lin_vel_w=torch.tensor([[0.5, 0.6, 0.7]], dtype=torch.float32),
            root_ang_vel_w=torch.tensor([[0.8, 0.9, 1.0]], dtype=torch.float32),
            body_names=("base", "link_tcp"),
            body_pos_w=torch.tensor(
                [[[0.0, 0.0, 0.4], [1.0, 2.0, 3.0]]], dtype=torch.float32
            ),
            body_quat_w=torch.tensor(
                [[[0.1, 0.2, 0.3, 0.4], [0.4, 0.3, 0.2, 0.1]]], dtype=torch.float32
            ),
        )
        # Isaac Lab articulations own actuator models keyed by group name. Each
        # ImplicitActuator keeps its own effort_limit tensor that is re-applied on
        # reset/reinit; the runtime effort-limit writer must keep this in sync
        # (Isaac Lab issue #128).
        self.actuators = {
            "gripper": SimpleNamespace(
                joint_names=("drive_joint",),
                effort_limit=torch.tensor([[12.0]], dtype=torch.float32),
            ),
            "arm": SimpleNamespace(
                joint_names=("joint1",),
                effort_limit=torch.tensor([[30.0]], dtype=torch.float32),
                stiffness=torch.tensor([[20000.0]], dtype=torch.float32),
                damping=torch.tensor([[1500.0]], dtype=torch.float32),
            ),
        }
        self.limit_calls: list[dict[str, object]] = []
        self.gain_calls: list[tuple[str, dict[str, object]]] = []
        self.target_calls: list[tuple[str, torch.Tensor]] = []
        self.write_count = 0

    def write_joint_effort_limit_to_sim_index(self, **kwargs: object) -> None:
        self.limit_calls.append(kwargs)

    def write_joint_stiffness_to_sim_index(self, **kwargs: object) -> None:
        self.gain_calls.append(("stiffness", kwargs))

    def write_joint_damping_to_sim_index(self, **kwargs: object) -> None:
        self.gain_calls.append(("damping", kwargs))

    def set_joint_position_target(self, target: torch.Tensor) -> None:
        self.target_calls.append(("position", target.clone()))

    def set_joint_velocity_target(self, target: torch.Tensor) -> None:
        self.target_calls.append(("velocity", target.clone()))

    def set_joint_effort_target(self, target: torch.Tensor) -> None:
        self.target_calls.append(("effort", target.clone()))

    def write_data_to_sim(self) -> None:
        self.write_count += 1

    def update(self, _dt: float) -> None:
        pass


class _ArrayValue:
    def __init__(self, values: object) -> None:
        self._values = values

    def tolist(self) -> object:
        return self._values


class _FakeRigidView:
    count = 1

    def __init__(self) -> None:
        self._transforms = _ArrayValue(
            [[0.65, -0.1, 0.8, 0.1, 0.2, 0.3, 0.4]]
        )
        self._velocities = _ArrayValue(
            [[0.01, 0.02, 0.03, 0.04, 0.05, 0.06]]
        )

    def get_transforms(self) -> _ArrayValue:
        return self._transforms

    def get_velocities(self) -> _ArrayValue:
        return self._velocities


def _backend() -> IsaacWholeRobotBackend:
    backend = object.__new__(IsaacWholeRobotBackend)
    backend._torch = torch
    backend._robot = _FakeRobot()
    backend._joint_index = {"drive_joint": 0, "joint1": 1}
    backend.joint_names = ("drive_joint", "joint1")
    backend._position_targets = torch.tensor([[1.0, -1.0]], dtype=torch.float32)
    backend._velocity_targets = torch.tensor([[2.0, 3.0]], dtype=torch.float32)
    backend._effort_targets = torch.tensor([[4.0, 5.0]], dtype=torch.float32)
    backend._safety_stopped = False
    backend._safety_snapshot = None
    backend._safety_joint_ids = (1,)
    backend._safety_nominal_stiffness = (20000.0,)
    backend._safety_nominal_damping = (1500.0,)
    backend._safety_nominal_effort_limits = (30.0,)
    backend._safety_gains_applied = False
    backend._safety_hold_steps = 0
    backend._pending_snapshot_id = None
    backend._pending_snapshot_count = 0
    backend._pending_snapshot_index = 0
    backend._pending_snapshot_commands = []
    backend._default_gripper_effort_limit = 12.0
    backend._gripper_effort_limit = 12.0
    backend._expected_objects = {}
    backend._contact_pairs_by_key = {}
    backend._robot_view_identity = id(backend._robot.root_view)
    backend._clock_step_origin = 0
    backend._clock_elapsed_steps = 0
    backend._sim = SimpleNamespace(
        get_physics_step_count=lambda: 0,
        step=lambda render=False: None,
    )
    backend.render = False
    backend.dt = 1.0 / 120.0
    backend.physics_dt = 1.0 / 120.0
    backend.physics_hz = 120.0
    backend.control_hz = 120.0
    backend.physics_substeps = 1
    backend._target_write_gate = TargetWriteGate()
    backend.step_profile = {"enabled": False, "target_writes": 0}
    backend.chassis_ballast_mass_kg = 0.0
    backend._object_views = {}
    backend._refresh_object_views = lambda: None
    return backend


def _protocol_gateway(epoch: int = 11) -> RosStandardGateway:
    backend = _backend()
    gateway = object.__new__(RosStandardGateway)
    gateway.backend = backend
    gateway._incoming_events = queue.SimpleQueue()
    gateway._last_command_error = None
    gateway._safety_active = False
    gateway._session_protocol_enabled = True
    gateway._command_epoch = None
    gateway._retired_command_epochs = set()
    gateway._last_logical_snapshot_id = -1
    gateway._last_snapshot_packet_count = 0
    gateway._last_snapshot_packet_index = 0
    gateway._last_snapshot_id = -1
    gateway._command_stream_timeout_s = 0.5
    gateway._last_command_received_at = None
    gateway._command_stream_lost = True
    gateway._command_loss_at = 0.0
    gateway._safety_sample_sequence = 1
    gateway._last_safety_clear_sequence = 1
    gateway._last_epoch_adoption_clear_sequence = -1
    gateway._command_loss_safety_sequence = 0
    gateway._safety_timeout_s = 1.0
    gateway._safety_last_sample_at = time.monotonic()
    gateway._last_safety_clear_at = gateway._safety_last_sample_at
    return gateway


def _spawn_usd_file_cfg_source(backend_source: str) -> str:
    """Return the source text of the ``UsdFileCfg`` used as the Articulation spawn.

    The backend builds ``ArticulationCfg(...)`` with a ``spawn=sim_utils.UsdFileCfg(...)``
    keyword.  This extracts exactly that spawn call so the test can assert the
    spawn-time joint-drive contract without importing Isaac or inspecting USD.
    """
    tree = ast.parse(backend_source)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not isinstance(func, ast.Name) or func.id != "ArticulationCfg":
            continue
        for keyword in node.keywords:
            if keyword.arg != "spawn":
                continue
            value = keyword.value
            if not isinstance(value, ast.Call):
                continue
            value_func = value.func
            if not (
                isinstance(value_func, ast.Attribute)
                and value_func.attr == "UsdFileCfg"
            ):
                continue
            return ast.get_source_segment(backend_source, value) or ""
    return ""


class ManipulationRuntimeTest(unittest.TestCase):
    def _runner_for_gpu(self, command_runner):
        runner = object.__new__(QualificationRunner)
        runner._command_runner = command_runner
        runner._owned_pids = set()
        return runner

    @staticmethod
    def _nvidia_result(stdout: str = "", returncode: int = 0, stderr: str = ""):
        return SimpleNamespace(stdout=stdout, stderr=stderr, returncode=returncode)

    def test_gpu_cleanup_detects_graphics_only_process_and_memory_leak(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt_dir = Path(directory)
            manifest = QualificationManifest("attempt-1", attempt_dir, {})
            outputs = [
                self._nvidia_result("0, GPU-abc, 100\n"),
                self._nvidia_result("GPU-abc, 101, python, 12\n"),
                self._nvidia_result("# gpu pid type sm mem enc dec command\n 0 101 G 0 0 0 0 viewer\n"),
            ]
            outputs.extend(
                [
                    self._nvidia_result("0, GPU-abc, 180\n"),
                    self._nvidia_result("\n"),
                    self._nvidia_result("# gpu pid type sm mem enc dec command\n 0 222 G 0 0 0 0 compositor\n"),
                ]
                * 5
            )
            runner = self._runner_for_gpu(lambda *args, **kwargs: outputs.pop(0))
            runner._owned_pids = {222}

            baseline = runner._gpu_processes()
            self.assertTrue(baseline["available"])
            self.assertFalse(
                runner._write_resource_evidence(manifest, baseline)
            )
            evidence = json.loads((attempt_dir / "resource-cleanup.json").read_text())
            self.assertEqual(evidence["attempt_owned_gpu_survivors"][0]["pid"], 222)
            self.assertEqual(evidence["unexplained_gpu_memory"][0]["final_memory_used_mib"], 180)
            self.assertEqual(evidence["memory_tolerance_mib"], 32)

    def test_gpu_cleanup_accepts_clean_return_to_baseline(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt_dir = Path(directory)
            manifest = QualificationManifest("attempt-1", attempt_dir, {})
            outputs = [
                self._nvidia_result("0, GPU-abc, 100\n"),
                self._nvidia_result("GPU-abc, 101, python, 12\n"),
                self._nvidia_result("# gpu pid type sm mem enc dec command\n 0 101 C 0 0 0 0 python\n"),
                self._nvidia_result("0, GPU-abc, 100\n"),
                self._nvidia_result("GPU-abc, 101, python, 12\n"),
                self._nvidia_result("# gpu pid type sm mem enc dec command\n 0 101 C 0 0 0 0 python\n"),
            ]
            runner = self._runner_for_gpu(lambda *args, **kwargs: outputs.pop(0))
            baseline = runner._gpu_processes()
            self.assertTrue(runner._write_resource_evidence(manifest, baseline))
            evidence = json.loads((attempt_dir / "resource-cleanup.json").read_text())
            self.assertTrue(evidence["clean"])
            self.assertEqual(len(evidence["settle_attempts"]), 1)

    def test_gpu_cleanup_preserves_unrelated_preexisting_allocation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt_dir = Path(directory)
            manifest = QualificationManifest("attempt-1", attempt_dir, {})
            outputs = [
                self._nvidia_result("0, GPU-abc, 2048\n"),
                self._nvidia_result("GPU-abc, 77, compositor, 2048\n"),
                self._nvidia_result("# gpu pid type sm mem enc dec command\n 0 77 G 0 0 0 0 compositor\n"),
                self._nvidia_result("0, GPU-abc, 2048\n"),
                self._nvidia_result("GPU-abc, 77, compositor, 2048\n"),
                self._nvidia_result("# gpu pid type sm mem enc dec command\n 0 77 G 0 0 0 0 compositor\n"),
            ]
            runner = self._runner_for_gpu(lambda *args, **kwargs: outputs.pop(0))
            baseline = runner._gpu_processes()
            self.assertTrue(runner._write_resource_evidence(manifest, baseline))
            evidence = json.loads((attempt_dir / "resource-cleanup.json").read_text())
            self.assertEqual(evidence["final"]["processes"][0]["pid"], 77)
            self.assertEqual(evidence["attempt_owned_gpu_survivors"], [])

    def test_gpu_cleanup_fails_closed_when_snapshot_query_is_unavailable(self) -> None:
        runner = self._runner_for_gpu(
            lambda *args, **kwargs: self._nvidia_result(returncode=1, stderr="denied")
        )
        with patch("manipulation_qualification.shutil.which", return_value="nvidia-smi"):
            snapshot = runner._gpu_processes()
        self.assertFalse(snapshot["available"])
        self.assertEqual(snapshot["gpus"], [])

    def test_gate_window_preserves_evidence_presence_and_size(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt_dir = Path(directory)
            (attempt_dir / "physics_truth.jsonl").write_text(
                '{"frame": 0}\n\nnot-json\n{"frame": 1}\n',
                encoding="utf-8",
            )
            (attempt_dir / "evaluator.jsonl").write_text(
                '{"frame": 0}\n\n',
                encoding="utf-8",
            )
            manifest = QualificationManifest(
                attempt_id="attempt-1",
                attempt_dir=attempt_dir,
                data={},
            )
            runner = QualificationRunner(root=attempt_dir, gate="all")

            runner._write_gate_window("safety-stop", manifest)

            window = json.loads(
                (attempt_dir / "gate-window.json").read_text(encoding="utf-8")
            )
            self.assertEqual(window["schema_version"], 1)
            self.assertEqual(window["gate"], "safety-stop")
            self.assertEqual(window["attempt_id"], "attempt-1")
            self.assertEqual(window["raw_start_index"], 2)
            self.assertEqual(window["evaluator_start_index"], 1)
            self.assertTrue(window["wall_timestamp"])
            self.assertFalse((attempt_dir / "gate-window.json.tmp").exists())

            gate_window = attempt_dir / "gate-window.json"
            self.assertTrue(gate_window.is_file())
            self.assertGreater(gate_window.stat().st_size, 0)

    def test_backend_is_stopped_until_explicit_clear_sample(self) -> None:
        backend = _backend()
        backend._safety_stopped = True
        backend._safety_snapshot = backend._robot.data.joint_pos.clone()

        self.assertFalse(
            backend.command_joints(JointCommand(("joint1",), positions=(0.75,)))
        )
        backend.set_safety_stop(False)
        self.assertTrue(
            backend.command_joints(JointCommand(("joint1",), positions=(0.75,)))
        )

    def test_safety_stop_holds_measured_position_and_invalidates_commands(self) -> None:
        backend = _backend()
        backend.set_safety_stop(True)

        self.assertTrue(backend.safety_stopped)
        self.assertEqual(backend._position_targets.shape, (1, 2))
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), 0.25)
        self.assertAlmostEqual(float(backend._position_targets[0, 1]), -0.4)
        self.assertEqual(backend._velocity_targets.tolist(), [[0.0, 0.0]])
        self.assertEqual(backend._effort_targets.tolist(), [[0.0, 0.0]])
        self.assertFalse(backend.command_joints(JointCommand(("joint1",), positions=(9.0,))))

        backend._robot.data.joint_pos = torch.tensor([[0.3, -0.2]], dtype=torch.float32)
        backend.set_safety_stop(False)
        self.assertFalse(backend.safety_stopped)
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), 0.3)
        self.assertAlmostEqual(float(backend._position_targets[0, 1]), -0.2)
        self.assertEqual(backend._velocity_targets.tolist(), [[0.0, 0.0]])

    def test_safety_stop_reapplies_explicit_hold_without_reviving_old_epoch(self) -> None:
        backend = _backend()
        backend.begin_command_snapshot(7)
        backend.command_joints(JointCommand(("joint1",), velocities=(1.25,)))
        backend.set_safety_stop(True)

        self.assertFalse(
            backend.command_joints(JointCommand(("joint1",), positions=(9.0,)))
        )
        backend._robot.data.joint_pos = torch.tensor(
            [[0.25, -0.401]], dtype=torch.float32
        )
        backend._robot.data.joint_vel = torch.tensor([[0.1, -0.02]], dtype=torch.float32)
        backend.step()
        backend.step()

        self.assertEqual(backend._velocity_targets.tolist(), [[0.0, 0.0]])
        self.assertAlmostEqual(float(backend._effort_targets[0, 0]), 0.0)
        # Gravity feed-forward only (fake gravity 1.5 at base-DoF-offset index 3);
        # the position/velocity correction is PhysX's drive.
        self.assertAlmostEqual(float(backend._effort_targets[0, 1]), 1.5, places=4)
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), 0.25)
        self.assertAlmostEqual(float(backend._position_targets[0, 1]), -0.4)
        # The measured state did not change between the two steps, so the
        # recomputed effort target is identical and the write gate skips the
        # second push (PhysX holds the target it already has).
        self.assertEqual(backend._robot.write_count, 1)
        # The hold's drive configuration (ceiling, stiffness 0, damping 0) is
        # constant for the whole hold and is written once on entry; each of
        # those is a PhysX model-property write (measured 2.9 ms per step when
        # repeated). The effort target is still recomputed and pushed every
        # step, see the target_calls assertions below.
        self.assertEqual(
            [name for name, _ in backend._robot.gain_calls],
            ["stiffness", "damping"],
        )
        self.assertEqual(
            [call["limits"].tolist() for call in backend._robot.limit_calls[-1:]],
            [[[100.0]]],
        )
        self.assertEqual(
            [call["stiffness"] for name, call in backend._robot.gain_calls if name == "stiffness"],
            [IsaacWholeRobotBackend.SAFETY_HOLD_STIFFNESS],
        )
        self.assertEqual(
            [call["damping"] for name, call in backend._robot.gain_calls if name == "damping"],
            [IsaacWholeRobotBackend.SAFETY_HOLD_DAMPING],
        )
        self.assertEqual(
            [kind for kind, _ in backend._robot.target_calls[-3:]],
            ["position", "velocity", "effort"],
        )
        self.assertEqual(
            [kind for kind, _ in backend._robot.target_calls],
            ["position", "velocity", "effort"],
        )
        effort_targets = [
            target for kind, target in backend._robot.target_calls if kind == "effort"
        ]
        self.assertEqual(len(effort_targets), 1)
        for target in effort_targets:
            self.assertAlmostEqual(float(target[0, 0]), 0.0)
            self.assertAlmostEqual(float(target[0, 1]), 1.5, places=4)

        backend.set_safety_stop(False)
        self.assertEqual(backend._velocity_targets.tolist(), [[0.0, 0.0]])
        self.assertEqual(backend._command_snapshot_id, None)
        self.assertEqual(
            [name for name, _ in backend._robot.gain_calls[-2:]],
            ["stiffness", "damping"],
        )
        self.assertEqual(
            backend._robot.gain_calls[-1][1]["damping"].tolist(),
            [[1500.0]],
        )
        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[30.0]])
        self.assertTrue(
            backend.command_joints(JointCommand(("joint1",), positions=(0.75,)))
        )

    def test_safety_hold_uses_physx_drive_and_restores_nominal_gain(self) -> None:
        """The hold is PhysX's drive: hold gains on entry, gravity fed forward."""
        backend = _backend()
        backend.set_safety_stop(True)
        backend.step()
        self.assertEqual(IsaacWholeRobotBackend.SAFETY_HOLD_STIFFNESS, 600.0)
        self.assertEqual(IsaacWholeRobotBackend.SAFETY_HOLD_DAMPING, 80.0)

        self.assertEqual(
            [call["stiffness"] for name, call in backend._robot.gain_calls if name == "stiffness"],
            [IsaacWholeRobotBackend.SAFETY_HOLD_STIFFNESS],
        )
        self.assertEqual(
            [call["damping"] for name, call in backend._robot.gain_calls if name == "damping"],
            [IsaacWholeRobotBackend.SAFETY_HOLD_DAMPING],
        )
        self.assertEqual(backend._robot.target_calls[-1][0], "effort")
        self.assertAlmostEqual(float(backend._robot.target_calls[-1][1][0, 1]), 1.5)
        self.assertAlmostEqual(float(backend._robot.target_calls[-1][1][0, 0]), 0.0)

        backend.set_safety_stop(False)
        self.assertEqual(
            [name for name, _ in backend._robot.gain_calls[-2:]],
            ["stiffness", "damping"],
        )
        self.assertEqual(
            backend._robot.gain_calls[-1][1]["damping"].tolist(),
            [[IsaacWholeRobotBackend.NOMINAL_ARM_DAMPING]],
        )
        self.assertEqual(
            backend._robot.gain_calls[-2][1]["stiffness"].tolist(),
            [[IsaacWholeRobotBackend.NOMINAL_ARM_STIFFNESS]],
        )
        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[30.0]])
        self.assertEqual(backend._effort_targets.tolist(), [[0.0, 0.0]])

    def test_safety_effort_uses_proxy_gravity_with_base_dof_offset_and_sign(self) -> None:
        backend = _backend()
        backend._robot.data.gravity_compensation_forces = SimpleNamespace(
            torch=torch.tensor([[9.0, 8.0, 7.0, -0.5]], dtype=torch.float64)
        )
        backend.set_safety_stop(True)
        backend._robot.data.joint_pos = torch.tensor([[0.25, -0.39]], dtype=torch.float32)
        backend._robot.data.joint_vel = torch.tensor([[0.0, 0.01]], dtype=torch.float32)
        backend.step()

        # The arm joint is data index 1, and gravity index 1 + num_base_dofs.
        # Only gravity (-0.5) is fed forward; the -0.01 rad error and the
        # +0.01 rad/s velocity are corrected by PhysX's drive at 600 / 80.
        self.assertAlmostEqual(float(backend._effort_targets[0, 0]), 0.0)
        self.assertAlmostEqual(float(backend._effort_targets[0, 1]), -0.5, places=4)

    def test_safety_effort_clips_to_100_nm_ceiling_and_zeroes_non_arm_joints(self) -> None:
        backend = _backend()
        backend._robot.data.gravity_compensation_forces = torch.tensor(
            [[1000.0, 1000.0, 1000.0, 1000.0]], dtype=torch.float32
        )
        backend.set_safety_stop(True)
        backend.step()

        self.assertEqual(backend._effort_targets.tolist(), [[0.0, 100.0]])
        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[100.0]])

        backend._robot.data.gravity_compensation_forces = torch.tensor(
            [[-1000.0, -1000.0, -1000.0, -1000.0]], dtype=torch.float32
        )
        # The feed-forward is refreshed every SAFETY_HOLD_GRAVITY_REFRESH_STEPS
        # control steps; until then the entry value is held.
        self.assertEqual(backend._effort_targets.tolist(), [[0.0, 100.0]])
        for _ in range(IsaacWholeRobotBackend.SAFETY_HOLD_GRAVITY_REFRESH_STEPS):
            backend.step()
        self.assertEqual(backend._effort_targets.tolist(), [[0.0, -100.0]])
        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[100.0]])

        backend.set_safety_stop(False)
        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[30.0]])

    def test_safety_ceiling_covers_all_arm_joints_and_clear_restores_each_nominal(self) -> None:
        backend = _backend()
        backend._safety_joint_ids = tuple(range(1, 8))
        backend._safety_nominal_effort_limits = tuple(
            float(limit) for limit in (11, 22, 33, 44, 55, 66, 77)
        )

        self.assertEqual(
            backend._safety_effort_limits(),
            (100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0),
        )

        backend._safety_gains_applied = True
        backend._restore_safety_actuator_gains()
        self.assertEqual(
            backend._robot.limit_calls[-1]["limits"].tolist(),
            [[11.0, 22.0, 33.0, 44.0, 55.0, 66.0, 77.0]],
        )

    def test_safety_hold_mirrors_gains_into_the_actuator_model(self) -> None:
        """applied_torque telemetry and reset re-sync come from the actuator
        model's own tensors (Isaac Lab issue #128), so the hold gains and the
        nominal restore must be mirrored there, not only written to PhysX."""
        backend = _backend()
        arm = backend._robot.actuators["arm"]
        backend.set_safety_stop(True)
        backend.step()
        self.assertEqual(float(arm.stiffness[0, 0]), IsaacWholeRobotBackend.SAFETY_HOLD_STIFFNESS)
        self.assertEqual(float(arm.damping[0, 0]), IsaacWholeRobotBackend.SAFETY_HOLD_DAMPING)
        self.assertEqual(float(arm.effort_limit[0, 0]), 100.0)
        # The gripper group owns drive_joint only; it must be untouched.
        self.assertEqual(float(backend._robot.actuators["gripper"].effort_limit[0, 0]), 12.0)

        backend.set_safety_stop(False)
        self.assertEqual(float(arm.stiffness[0, 0]), 20000.0)
        self.assertEqual(float(arm.damping[0, 0]), 1500.0)
        self.assertEqual(float(arm.effort_limit[0, 0]), 30.0)

    def test_safety_hold_writes_drive_config_once_per_hold(self) -> None:
        """Ceiling/stiffness/damping are constant for a hold: written on entry only."""
        backend = _backend()
        backend.set_safety_stop(True)
        for index in range(5):
            # A moving measured state is PhysX's business now: the drive
            # corrects it at every substep without a Python push.
            backend._robot.data.joint_vel = torch.tensor(
                [[0.1, -0.2 + 0.01 * index]], dtype=torch.float32
            )
            backend.step()

        self.assertEqual(len(backend._robot.gain_calls), 2)
        self.assertEqual(
            [call["limits"].tolist() for call in backend._robot.limit_calls],
            [[[100.0]]],
        )
        # One push on entry (latched pose, zero velocity, gravity feed-forward),
        # then nothing while the hold is steady.
        self.assertEqual(backend._robot.write_count, 1)
        self.assertEqual(
            [kind for kind, _ in backend._robot.target_calls].count("effort"), 1
        )
        # The gravity feed-forward is refreshed every
        # SAFETY_HOLD_GRAVITY_REFRESH_STEPS control steps.
        backend._robot.data.gravity_compensation_forces = torch.tensor(
            [[0.1, 0.2, 0.3, 2.5]], dtype=torch.float32
        )
        refresh = IsaacWholeRobotBackend.SAFETY_HOLD_GRAVITY_REFRESH_STEPS
        for _ in range(refresh - 5):
            backend.step()
        # Still the entry value one step before the refresh boundary...
        self.assertEqual(backend._robot.write_count, 1)
        self.assertAlmostEqual(float(backend._effort_targets[0, 1]), 1.5, places=4)
        backend.step()
        # ...and refreshed (one push) on it.
        self.assertEqual(backend._robot.write_count, 2)
        self.assertAlmostEqual(float(backend._effort_targets[0, 1]), 2.5, places=4)

        # Clearing restores nominal gains exactly once; a second hold writes
        # the hold configuration again.
        backend.set_safety_stop(False)
        self.assertEqual(
            [name for name, _ in backend._robot.gain_calls[-2:]],
            ["stiffness", "damping"],
        )
        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[30.0]])
        backend.set_safety_stop(True)
        backend.step()
        backend.step()
        self.assertEqual(len(backend._robot.gain_calls), 6)
        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[100.0]])

    def test_safety_hold_rewrites_drive_config_after_view_refresh(self) -> None:
        """A re-resolved articulation view carries the actuator model's nominal
        gains, so the hold configuration must be pushed again after one."""
        backend = _backend()
        backend.set_safety_stop(True)
        backend.step()
        backend.step()
        self.assertEqual(len(backend._robot.gain_calls), 2)

        # Standard STOP -> PLAY: Isaac Lab recreates the root view.
        backend._robot.root_view = object()
        backend.step()
        self.assertEqual(len(backend._robot.gain_calls), 4)
        self.assertEqual(
            [name for name, _ in backend._robot.gain_calls[-2:]],
            ["stiffness", "damping"],
        )
        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[100.0]])
        backend.step()
        self.assertEqual(len(backend._robot.gain_calls), 4)

    def test_safety_effort_rejects_missing_nonfinite_and_malformed_gravity(self) -> None:
        for gravity in (
            None,
            torch.tensor([[0.0, 0.0, 0.0, float("nan")]], dtype=torch.float32),
            torch.zeros((1, 3), dtype=torch.float32),
        ):
            backend = _backend()
            backend._robot.data.gravity_compensation_forces = gravity
            backend.set_safety_stop(True)
            with self.subTest(gravity=gravity):
                with self.assertRaises(RuntimeError):
                    backend.step()

    def test_drive_joint_effort_is_runtime_bounded_and_zero_uses_default(self) -> None:
        backend = _backend()
        backend._set_gripper_effort_limit(100.0)
        self.assertEqual(backend.gripper_effort_limit, 12.0)
        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[12.0]])

        backend._set_gripper_effort_limit(0.0)
        self.assertEqual(backend.gripper_effort_limit, 12.0)
        with self.assertRaises(ValueError):
            backend._set_gripper_effort_limit(-1.0)

    def test_runtime_dtype_proxy_falls_back_to_torch_dtype_for_effort_limit(self) -> None:
        backend = _backend()
        backend._robot.data.joint_pos = SimpleNamespace(dtype=float)

        backend._set_gripper_effort_limit(6.0)

        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[6.0]])

    def test_gripper_effort_limit_updates_implicit_actuator_model_entry(self) -> None:
        """RED runtime contract: _set_gripper_effort_limit must update the actuator model.

        Isaac Lab issue #128: write_joint_effort_limit_to_sim_index writes only the
        PhysX/shared buffer. The owning ImplicitActuator keeps its own effort_limit
        tensor, which is re-applied to the simulator on reset/reinit. If the runtime
        call does not update that entry, the actuator re-sync clobbers the request.
        A zero request restores the existing default and a negative request still
        rejects without touching either writer or model entry.
        """
        backend = _backend()
        gripper = backend._robot.actuators["gripper"]
        self.assertEqual(float(gripper.effort_limit[0, 0]), 12.0)

        backend._set_gripper_effort_limit(6.0)
        self.assertEqual(backend.gripper_effort_limit, 6.0)
        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[6.0]])
        self.assertEqual(
            float(gripper.effort_limit[0, 0]),
            6.0,
            "drive_joint's owning implicit actuator model effort_limit must be updated "
            "alongside the PhysX writer so an actuator re-sync does not clobber it "
            "(Isaac Lab issue #128)",
        )

        backend._set_gripper_effort_limit(0.0)
        self.assertEqual(backend.gripper_effort_limit, 12.0)
        self.assertEqual(backend._robot.limit_calls[-1]["limits"].tolist(), [[12.0]])
        self.assertEqual(float(gripper.effort_limit[0, 0]), 12.0)

        with self.assertRaises(ValueError):
            backend._set_gripper_effort_limit(-1.0)
        self.assertEqual(backend.gripper_effort_limit, 12.0)
        self.assertEqual(float(gripper.effort_limit[0, 0]), 12.0)

    def test_repeated_identical_gripper_effort_limit_writes_once(self) -> None:
        """The bridge re-sends the gripper packet at 150 Hz; an unchanged ceiling
        must not reach the PhysX writer again (each write costs a physics step)."""
        backend = _backend()
        backend._set_gripper_effort_limit(6.0)
        writes = len(backend._robot.limit_calls)
        for _ in range(5):
            backend._set_gripper_effort_limit(6.0)
        self.assertEqual(len(backend._robot.limit_calls), writes)
        self.assertEqual(backend.gripper_effort_limit_writes, 1)
        backend._set_gripper_effort_limit(0.0)
        self.assertEqual(len(backend._robot.limit_calls), writes + 1)
        self.assertEqual(backend.gripper_effort_limit, 12.0)
        # A zero request that restores the default is also a no-op when repeated.
        backend._set_gripper_effort_limit(0.0)
        self.assertEqual(len(backend._robot.limit_calls), writes + 1)

    def test_position_only_command_clears_affected_velocity_target(self) -> None:
        backend = _backend()

        self.assertTrue(
            backend.command_joints(JointCommand(("joint1",), positions=(0.75,)))
        )

        self.assertEqual(backend._velocity_targets.tolist(), [[2.0, 0.0]])
        self.assertAlmostEqual(float(backend._position_targets[0, 1]), 0.75)

    def test_complete_snapshot_retires_omitted_velocity_without_losing_position_hold(self) -> None:
        backend = _backend()
        backend.begin_command_snapshot(0)
        backend.command_joints(JointCommand(("joint1",), velocities=(1.25,)))
        backend.begin_command_snapshot(1)
        backend.command_joints(JointCommand(("drive_joint",), positions=(0.6,)))

        self.assertEqual(backend._velocity_targets.tolist(), [[0.0, 0.0]])
        # drive_joint positions defer to the close ramp (_ramp_drive_target in
        # step()), so the snapshot captures the target rather than writing it
        # straight into _position_targets. The velocity retirement above is the
        # subject; the captured target confirms the command committed.
        self.assertAlmostEqual(float(backend._drive_command_target), 0.6)

    def test_snapshot_boundary_preserves_active_mixed_base_and_arm_packets(self) -> None:
        backend = _backend()
        backend.begin_command_snapshot(0)
        backend.command_joints(JointCommand(("drive_joint",), velocities=(1.25,)))
        backend.command_joints(JointCommand(("joint1",), positions=(0.75,)))

        self.assertEqual(backend._velocity_targets.tolist(), [[1.25, 0.0]])
        self.assertAlmostEqual(float(backend._position_targets[0, 1]), 0.75)

    def test_multi_packet_snapshot_commits_only_after_final_packet(self) -> None:
        backend = _backend()
        first = encode_snapshot_packet(10, 2, 1)
        second = encode_snapshot_packet(10, 2, 2)

        backend.begin_command_snapshot(first)
        backend.command_joints(JointCommand(("joint1",), velocities=(1.25,)))
        self.assertEqual(backend._velocity_targets.tolist(), [[2.0, 3.0]])
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), 1.0)

        backend.begin_command_snapshot(second)
        backend.command_joints(JointCommand(("drive_joint",), positions=(0.6,)))
        self.assertEqual(backend._velocity_targets.tolist(), [[0.0, 1.25]])
        # drive_joint positions defer to the close ramp; the final packet
        # commits the captured target (see _apply_joint_command).
        self.assertAlmostEqual(float(backend._drive_command_target), 0.6)

    def test_gripper_close_slews_and_bounded_lead_stops_on_stalled_pads(self) -> None:
        # The close target must not jump to the command; it slews. And once the
        # pads stall against an object (measured pad position stops advancing),
        # the applied target must clamp at pad + max_lead and STOP -- never
        # running on to the fully-closed command (the open-loop runaway that
        # climbed to 248 N and defeated the facade stall detector).
        backend = _backend()
        backend._drive_joint_index = 0
        backend._gripper_close_slew = 1.5
        backend._gripper_max_lead = 0.015
        backend._gripper_contact_halt_force = 0.0
        backend._position_targets = torch.tensor([[0.0, 0.0]], dtype=torch.float32)
        backend._drive_command_target = 0.85

        step = 1.5 / 120.0
        # Pads stalled at 0.0 (object blocks them) and NOT moving -> the stall
        # gate lets the lead clamp engage.
        backend._measured_finger_closure = lambda: 0.0  # type: ignore[assignment]
        backend._measured_finger_speed = lambda: 0.0  # type: ignore[assignment]
        backend._ramp_drive_target()
        # First step: slew (0.0125) is below the lead cap (0.0 + 0.015).
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), step, places=6)
        for _ in range(30):
            backend._ramp_drive_target()
        # Converges to pad + lead and stays there -- nowhere near 0.85.
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), 0.015, places=5)

    def test_gripper_close_advances_while_pads_follow(self) -> None:
        # While the pads are MOVING, the stall gate holds the lead clamp OFF and
        # the target advances at the slew rate toward the command.
        backend = _backend()
        backend._drive_joint_index = 0
        backend._gripper_close_slew = 1.5
        backend._gripper_max_lead = 0.015
        backend._gripper_stall_speed = 0.1
        backend._gripper_contact_halt_force = 0.0
        backend._position_targets = torch.tensor([[0.0, 0.0]], dtype=torch.float32)
        backend._drive_command_target = 0.85
        backend._measured_finger_closure = lambda: 0.0  # type: ignore[assignment]
        backend._measured_finger_speed = lambda: 1.5  # moving at slew rate
        for _ in range(20):
            backend._ramp_drive_target()
        self.assertGreater(float(backend._position_targets[0, 0]), 0.2)

    def test_gripper_close_does_not_ratchet_backward_under_dynamic_lag(self) -> None:
        # cycle-3 regression: while the pads move, target - pad is dominated by
        # dynamic tracking lag (0.03 > lead 0.015). If the clamp engaged there,
        # min(slew, pad + lead) would drive the target BACKWARD and deadlock the
        # jaw open. The stall gate must hold it off while the pads are moving.
        backend = _backend()
        backend._drive_joint_index = 0
        backend._gripper_close_slew = 1.5
        backend._gripper_max_lead = 0.015
        backend._gripper_stall_speed = 0.1
        backend._gripper_contact_halt_force = 0.0
        backend._position_targets = torch.tensor([[0.1, 0.0]], dtype=torch.float32)
        backend._drive_command_target = 0.85
        backend._measured_finger_closure = lambda: float(  # lags by 0.03
            backend._position_targets[0, 0]
        ) - 0.03
        backend._measured_finger_speed = lambda: 1.5  # moving
        before = float(backend._position_targets[0, 0])
        backend._ramp_drive_target()
        self.assertGreater(float(backend._position_targets[0, 0]), before)

    def test_gripper_close_ramp_disabled_applies_target_instantly(self) -> None:
        backend = _backend()
        backend._drive_joint_index = 0
        backend._gripper_close_slew = 0.0  # ramp disabled
        backend._gripper_max_lead = 0.015
        backend._gripper_contact_halt_force = 0.0
        backend._position_targets = torch.tensor([[0.0, 0.0]], dtype=torch.float32)
        backend._drive_command_target = 0.85
        backend._ramp_drive_target()
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), 0.85, places=6)

    def test_gripper_open_ignores_close_bounds(self) -> None:
        # Only the closing stroke is bounded; an opening command must slew
        # freely even with stalled pads / contact force, so release is prompt.
        backend = _backend()
        backend._drive_joint_index = 0
        backend._gripper_close_slew = 1.5
        backend._gripper_max_lead = 0.015
        backend._gripper_contact_halt_force = 15.0
        backend._position_targets = torch.tensor([[0.85, 0.0]], dtype=torch.float32)
        backend._drive_command_target = 0.0  # open
        backend._measured_finger_closure = lambda: 0.85  # type: ignore[assignment]
        backend._gripper_grip_force = lambda: 50.0  # type: ignore[assignment]
        backend._ramp_drive_target()
        self.assertLess(float(backend._position_targets[0, 0]), 0.85)

    def test_gripper_optional_force_cap_freezes_when_enabled(self) -> None:
        # The optional hard force cap is off by default; when enabled it freezes
        # the close once raw pad contact force exceeds it.
        backend = _backend()
        backend._drive_joint_index = 0
        backend._gripper_close_slew = 1.5
        backend._gripper_max_lead = 0.0  # isolate the force cap
        backend._gripper_contact_halt_force = 15.0
        backend._position_targets = torch.tensor([[0.2, 0.0]], dtype=torch.float32)
        backend._drive_command_target = 0.85
        backend._gripper_grip_force = lambda: 20.0  # type: ignore[assignment]
        backend._ramp_drive_target()
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), 0.2, places=6)

    def test_gateway_applies_callbacks_in_arrival_order_across_safety_and_commands(self) -> None:
        class _GatewayBackend:
            safety_stopped = False

            def __init__(self) -> None:
                self.events: list[tuple[str, object]] = []
                self.position_target = 0.0

            def begin_command_snapshot(self, snapshot: int) -> None:
                pass

            def set_safety_stop(self, active: bool) -> None:
                self.events.append(("stop", active))
                self.safety_stopped = active
                if not active:
                    self.position_target = 0.0

            def command_joints(self, command: JointCommand) -> bool:
                self.events.append(("command", command.positions))
                if self.safety_stopped:
                    return False
                self.position_target = command.positions[0]
                return True

        backend = _GatewayBackend()
        gateway = object.__new__(RosStandardGateway)
        gateway.backend = backend
        gateway._incoming_events = queue.SimpleQueue()
        gateway._last_command_error = None
        gateway._safety_active = False
        gateway._command_epoch = 0
        gateway._last_snapshot_id = -1

        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(0, 0)),
                name=("joint1",),
                position=(1.5,),
                velocity=(),
                effort=(),
            )
        )
        gateway._safety_stop(SimpleNamespace(data=True))
        gateway._safety_stop(SimpleNamespace(data=False))
        gateway.spin_once()

        self.assertEqual(
            backend.events,
            [
                ("command", (1.5,)),
                ("stop", True),
                ("stop", False),
            ],
        )
        self.assertEqual(backend.position_target, 0.0)
        self.assertEqual(gateway._last_command_error, None)

    def test_delayed_old_command_is_rejected_after_stop_clear(self) -> None:
        class _EpochBackend:
            safety_stopped = False

            def __init__(self) -> None:
                self.events: list[tuple[str, object]] = []

            def begin_command_snapshot(self, snapshot: int) -> None:
                self.events.append(("snapshot", snapshot))

            def set_safety_stop(self, active: bool) -> None:
                self.safety_stopped = active
                self.events.append(("stop", active))

            def command_joints(self, command: JointCommand) -> bool:
                self.events.append(("command", command.positions))
                return not self.safety_stopped

        backend = _EpochBackend()
        gateway = object.__new__(RosStandardGateway)
        gateway.backend = backend
        gateway._incoming_events = queue.SimpleQueue()
        gateway._last_command_error = None
        gateway._safety_active = False
        gateway._command_epoch = 0
        gateway._last_snapshot_id = -1

        gateway._safety_stop(SimpleNamespace(data=True))
        gateway._safety_stop(SimpleNamespace(data=False))
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(0, 0)),
                name=("joint1",),
                position=(1.5,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()

        self.assertEqual(gateway._command_epoch, 2)
        self.assertEqual(backend.events, [("stop", True), ("stop", False)])
        self.assertIn("current epoch is 2", gateway._last_command_error)

        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(2, 1)),
                name=("joint1",),
                position=(2.0,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()
        self.assertEqual(
            backend.events,
            [("stop", True), ("stop", False), ("snapshot", 1), ("command", (2.0,))],
        )

    def test_duplicate_safety_messages_do_not_advance_epoch_twice(self) -> None:
        class _Backend:
            safety_stopped = False

            def __init__(self) -> None:
                self.stops: list[bool] = []

            def set_safety_stop(self, active: bool) -> None:
                self.stops.append(active)

        backend = _Backend()
        gateway = object.__new__(RosStandardGateway)
        gateway.backend = backend
        gateway._incoming_events = queue.SimpleQueue()
        gateway._last_command_error = None
        gateway._safety_active = False
        gateway._command_epoch = 0
        gateway._last_snapshot_id = -1

        for value in (False, False, True, True, False, False):
            gateway._safety_stop(SimpleNamespace(data=value))
        gateway.spin_once()

        self.assertEqual(gateway._command_epoch, 2)
        self.assertEqual(backend.stops, [True, False])

    def test_startup_false_clears_the_initial_backend_stop(self) -> None:
        class _Backend:
            safety_stopped = False

            def set_safety_stop(self, active: bool) -> None:
                self.safety_stopped = active

        gateway = object.__new__(RosStandardGateway)
        gateway.backend = _Backend()
        gateway._incoming_events = queue.SimpleQueue()
        gateway._last_command_error = None
        gateway._safety_active = True
        gateway._command_epoch = 0
        gateway._last_snapshot_id = -1
        gateway._safety_timeout_s = 1.0
        gateway._safety_last_sample_at = None

        gateway._safety_stop(SimpleNamespace(data=False))
        gateway.spin_once()
        self.assertEqual(gateway._command_epoch, 1)
        self.assertFalse(gateway.backend.safety_stopped)
        gateway._safety_stop(SimpleNamespace(data=True))
        gateway.spin_once()
        self.assertEqual(gateway._command_epoch, 2)
        self.assertTrue(gateway.backend.safety_stopped)

    def test_isaac_safety_heartbeat_timeout_reasserts_stop_and_invalidates_snapshots(self) -> None:
        class _Backend:
            safety_stopped = False

            def __init__(self) -> None:
                self.stops: list[bool] = []

            def set_safety_stop(self, active: bool) -> None:
                self.stops.append(active)
                self.safety_stopped = active

        backend = _Backend()
        gateway = object.__new__(RosStandardGateway)
        gateway.backend = backend
        gateway._incoming_events = queue.SimpleQueue()
        gateway._last_command_error = None
        gateway._safety_active = False
        gateway._command_epoch = 3
        gateway._last_snapshot_id = 44
        gateway._safety_timeout_s = 1.0
        gateway._safety_last_sample_at = 10.0

        gateway._enforce_safety_deadline(now=11.0)

        self.assertTrue(gateway._safety_active)
        self.assertTrue(backend.safety_stopped)
        self.assertEqual(backend.stops, [True])
        self.assertEqual(gateway._command_epoch, 4)
        self.assertEqual(gateway._last_snapshot_id, -1)
        self.assertEqual(gateway._last_command_error, "safety heartbeat expired")

    def test_stale_queued_clear_cannot_reopen_after_heartbeat_timeout(self) -> None:
        class _Backend:
            safety_stopped = False

            def set_safety_stop(self, active: bool) -> None:
                self.safety_stopped = active

        backend = _Backend()
        gateway = object.__new__(RosStandardGateway)
        gateway.backend = backend
        gateway._incoming_events = queue.SimpleQueue()
        gateway._last_command_error = None
        gateway._safety_active = False
        gateway._command_epoch = 2
        gateway._last_snapshot_id = 8
        gateway._safety_timeout_s = 1.0
        gateway._safety_last_sample_at = 10.0
        gateway._incoming_events.put(("safety_stop", (False, 10.0)))

        gateway.spin_once()

        self.assertTrue(gateway._safety_active)
        self.assertTrue(backend.safety_stopped)
        self.assertEqual(gateway._command_epoch, 3)

    def test_command_stream_loss_retires_nonzero_velocity_targets(self) -> None:
        gateway = _protocol_gateway()
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(11, 0)),
                name=("joint1",),
                position=(),
                velocity=(1.75,),
                effort=(),
            )
        )
        gateway.spin_once()

        self.assertFalse(gateway._command_stream_lost)
        self.assertEqual(gateway.backend._velocity_targets.tolist(), [[0.0, 1.75]])

        gateway._last_command_received_at = 10.0
        # Expiry needs the receipt stale in simulation time too (the sim has
        # stepped a full timeout past it), not only in wall time.
        gateway._last_command_received_sim_at = float(gateway.backend.simulation_time) - 1.0
        gateway._enforce_command_deadline(now=10.5)

        self.assertTrue(gateway._command_stream_lost)
        self.assertTrue(gateway.backend.safety_stopped)
        self.assertEqual(gateway.backend._velocity_targets.tolist(), [[0.0, 0.0]])
        self.assertIsNone(gateway._command_epoch)

    def test_same_session_stream_recovery_accepts_forward_snapshot_gap(self) -> None:
        gateway = _protocol_gateway()
        epoch = encode_command_epoch(13, 1)
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(epoch, 0)),
                name=("joint1",),
                position=(0.2,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()
        gateway._last_logical_snapshot_id = 10
        gateway._last_snapshot_id = 10
        gateway._last_snapshot_packet_count = 1
        gateway._last_snapshot_packet_index = 1
        gateway._last_command_received_at = 10.0
        # Expiry needs the receipt stale in simulation time too (the sim has
        # stepped a full timeout past it), not only in wall time.
        gateway._last_command_received_sim_at = float(gateway.backend.simulation_time) - 1.0
        gateway._enforce_command_deadline(now=10.5)

        gateway._safety_stop(SimpleNamespace(data=False))
        gateway.spin_once()

        gateway._incoming_events.put(
            (
                "command",
                (
                    JointCommand(("joint1",), positions=(0.4,)),
                    epoch,
                    encode_snapshot_packet(10, 1, 1),
                    time.monotonic(),
                ),
            )
        )
        gateway.spin_once()
        self.assertIn("expected a value greater than 10", gateway._last_command_error)

        restored = encode_snapshot_packet(85, 1, 1)
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(epoch, restored)),
                name=("joint1",),
                position=(0.8,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()
        self.assertEqual(gateway._last_logical_snapshot_id, 85)

        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(
                    frame_id=encode_command_frame(
                        epoch, encode_snapshot_packet(86, 1, 1)
                    )
                ),
                name=("joint1",),
                position=(0.9,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()
        self.assertEqual(gateway._last_logical_snapshot_id, 86)

    def test_command_stream_recovery_requires_fresh_clear_and_new_session(self) -> None:
        gateway = _protocol_gateway(epoch=101)
        gateway._retired_command_epochs = {101}
        gateway._command_loss_at = 10.0
        gateway._command_loss_safety_sequence = 4
        gateway._safety_sample_sequence = 4

        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(202, 0)),
                name=("joint1",),
                position=(0.6,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()
        self.assertIn("fresh safety clear", gateway._last_command_error)

        gateway._safety_stop(SimpleNamespace(data=False))
        gateway.spin_once()
        new_epoch = encode_command_epoch(7, 0)
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(new_epoch, 0)),
                name=("joint1",),
                position=(0.6,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()

        self.assertFalse(gateway._command_stream_lost)
        self.assertFalse(gateway.backend.safety_stopped)
        self.assertAlmostEqual(float(gateway.backend._position_targets[0, 1]), 0.6)

    def test_delayed_pre_stop_packet_is_rejected_after_asymmetric_timeout(self) -> None:
        gateway = _protocol_gateway(epoch=303)
        gateway._command_epoch = 303
        gateway._command_stream_lost = False
        gateway._last_command_received_at = 9.0
        gateway._last_logical_snapshot_id = 4
        gateway._last_snapshot_id = 4
        gateway._apply_safety_stop(True)
        loss_at = gateway._command_loss_at
        gateway._last_safety_clear_sequence = gateway._command_loss_safety_sequence + 1
        gateway._apply_safety_stop(False)
        gateway._incoming_events.put(
            (
                "command",
                (
                    JointCommand(("joint1",), positions=(9.0,)),
                    303,
                    encode_snapshot_packet(5, 1, 1),
                    loss_at - 0.001,
                ),
            )
        )
        gateway.spin_once()

        self.assertIn("before stream boundary", gateway._last_command_error)
        self.assertAlmostEqual(float(gateway.backend._position_targets[0, 1]), -0.4)

    def test_out_of_order_and_skipped_snapshots_are_rejected(self) -> None:
        gateway = _protocol_gateway(epoch=404)
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(404, 0)),
                name=("joint1",),
                position=(0.2,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(
                    frame_id=encode_command_frame(
                        404, encode_snapshot_packet(2, 1, 1)
                    )
                ),
                name=("joint1",),
                position=(0.8,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()

        self.assertIn("non-contiguous", gateway._last_command_error)
        self.assertAlmostEqual(float(gateway.backend._position_targets[0, 1]), 0.2)

    def test_gateway_only_timeout_adopts_newer_generation_after_fresh_false(self) -> None:
        gateway = _protocol_gateway()
        first_epoch = encode_command_epoch(17, 1)
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(first_epoch, 0)),
                name=("joint1",),
                position=(0.2,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()
        self.assertEqual(gateway._command_epoch, first_epoch)

        # The Isaac endpoint never sees an active safety sample.  The gateway
        # nevertheless advances its generation after its own heartbeat timeout
        # and emits the recovered generation after the next explicit false.
        gateway._safety_stop(SimpleNamespace(data=False))
        gateway.spin_once()
        recovered_epoch = encode_command_epoch(17, 2)
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(
                    frame_id=encode_command_frame(
                        recovered_epoch, encode_snapshot_packet(5, 1, 1)
                    )
                ),
                name=("joint1",),
                position=(0.7,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()

        self.assertEqual(gateway._command_epoch, recovered_epoch)
        self.assertIn(first_epoch, gateway._retired_command_epochs)
        self.assertAlmostEqual(float(gateway.backend._position_targets[0, 1]), 0.7)

        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(
                    frame_id=encode_command_frame(
                        encode_command_epoch(17, 1),
                        encode_snapshot_packet(6, 1, 1),
                    )
                ),
                name=("joint1",),
                position=(0.9,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()
        self.assertIn("retired command epoch", gateway._last_command_error)

    def test_restart_stopped_packets_before_clear_adopts_nonzero_baseline(self) -> None:
        gateway = _protocol_gateway()
        old_epoch = encode_command_epoch(31, 1)
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(old_epoch, 0)),
                name=("joint1",),
                position=(0.1,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()
        gateway._apply_safety_stop(True)

        restarted_stopped_epoch = encode_command_epoch(47, 0)
        for snapshot_id in (2, 3, 4):
            gateway._joint_command(
                SimpleNamespace(
                    header=SimpleNamespace(
                        frame_id=encode_command_frame(
                            restarted_stopped_epoch,
                            encode_snapshot_packet(snapshot_id, 1, 1),
                        )
                    ),
                    name=("joint1",),
                    position=(0.3,),
                    velocity=(),
                    effort=(),
                )
            )
        gateway.spin_once()
        self.assertTrue(gateway._safety_active)

        gateway._safety_stop(SimpleNamespace(data=False))
        gateway.spin_once()
        post_clear_epoch = encode_command_epoch(47, 1)
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(
                    frame_id=encode_command_frame(
                        post_clear_epoch, encode_snapshot_packet(5, 1, 1)
                    )
                ),
                name=("joint1",),
                position=(0.8,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()

        self.assertEqual(gateway._command_epoch, post_clear_epoch)
        self.assertAlmostEqual(float(gateway.backend._position_targets[0, 1]), 0.8)
        self.assertIn(31, gateway._retired_command_sessions)

    def test_isaac_gateway_rejects_malformed_and_future_epoch_commands(self) -> None:
        class _Backend:
            safety_stopped = False

            def __init__(self) -> None:
                self.commands: list[JointCommand] = []

            def begin_command_snapshot(self, snapshot: int) -> None:
                pass

            def command_joints(self, command: JointCommand) -> bool:
                self.commands.append(command)
                return True

        class _Logger:
            def error(self, message: str) -> None:
                pass

        backend = _Backend()
        gateway = object.__new__(RosStandardGateway)
        gateway.backend = backend
        gateway.node = SimpleNamespace(get_logger=lambda: _Logger())
        gateway._incoming_events = queue.SimpleQueue()
        gateway._last_command_error = None
        gateway._safety_active = False
        gateway._command_epoch = 0
        gateway._last_snapshot_id = -1

        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id="not-a-command"),
                name=("joint1",),
                position=(1.0,),
                velocity=(),
                effort=(),
            )
        )
        self.assertTrue(gateway._incoming_events.empty())
        gateway._joint_command(
            SimpleNamespace(
                header=SimpleNamespace(frame_id=encode_command_frame(1, 0)),
                name=("joint1",),
                position=(1.0,),
                velocity=(),
                effort=(),
            )
        )
        gateway.spin_once()

        self.assertEqual(backend.commands, [])
        self.assertIn("current epoch is 0", gateway._last_command_error)

    def test_robot_truth_keeps_pinned_xyzw_and_uses_tcp_body(self) -> None:
        backend = _backend()

        robot = backend._robot_truth_state()
        for actual, expected in zip(robot["base_pose"]["xyz"], [0.0, 0.0, 0.4]):
            self.assertAlmostEqual(float(actual), expected)
        for actual, expected in zip(robot["base_pose"]["quaternion_xyzw"], [0.1, 0.2, 0.3, 0.4]):
            self.assertAlmostEqual(float(actual), expected)
        for actual, expected in zip(robot["tcp_pose"]["xyz"], [1.0, 2.0, 3.0]):
            self.assertAlmostEqual(float(actual), expected)
        for actual, expected in zip(robot["tcp_pose"]["quaternion_xyzw"], [0.4, 0.3, 0.2, 0.1]):
            self.assertAlmostEqual(float(actual), expected)

    def test_command_target_truth_exposes_active_physx_targets(self) -> None:
        backend = _backend()

        targets = backend.command_target_state()

        self.assertEqual(targets["joint_names"], ["drive_joint", "joint1"])
        self.assertEqual(targets["joint_positions"], [1.0, -1.0])
        self.assertEqual(targets["joint_velocities"], [2.0, 3.0])
        self.assertEqual(targets["joint_efforts"], [4.0, 5.0])
        self.assertEqual(targets["gripper_effort_limit"], 12.0)
        self.assertEqual(backend.physics_frame_index, 0)

    def test_root_state_reorders_pinned_xyzw_to_public_wxyz(self) -> None:
        quaternion = _backend().root_state()["quaternion_wxyz"]

        for actual, expected in zip(quaternion, [0.4, 0.1, 0.2, 0.3]):
            self.assertAlmostEqual(float(actual), expected)

    def test_arm_scenario_collision_predicate_excludes_grasp_and_ground(self) -> None:
        self.assertEqual(
            IsaacWholeRobotBackend.ARM_CONTACT_BODIES,
            ("link1", "link2", "link3", "link4", "link5", "link6", "link7"),
        )
        predicate = IsaacWholeRobotBackend.is_arm_scenario_collision
        self.assertFalse(
            predicate(
                [
                    {
                        "body_a": "/World/Tinker/left_finger",
                        "body_b": "/World/Scenario/delivery_object",
                    },
                    {
                        "body_a": "/World/Tinker/link3",
                        "body_b": "/World/defaultGroundPlane",
                    },
                ]
            )
        )
        self.assertTrue(
            predicate(
                [
                    {
                        "body_a": "/World/Scenario/delivery_object",
                        "body_b": "/World/Tinker/link7",
                    }
                ]
            )
        )

    def test_reset_reacquires_object_views(self) -> None:
        backend = _backend()
        backend._robot_view_identity = -1
        backend._clock_step_origin = 0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 42)
        backend._object_views = {"delivery_object": object()}

        self.assertTrue(backend._refresh_robot_handles())
        self.assertEqual(backend._object_views, {})
        self.assertEqual(backend._contact_pairs_by_key, {})
        self.assertEqual(backend._clock_step_origin, 42)

    def test_contact_report_uses_identified_bodies_and_reported_normal(self) -> None:
        backend = _backend()
        backend.dt = 0.1
        backend._contact_event_found = "found"
        backend._contact_event_persist = "persist"
        backend._contact_event_lost = "lost"
        paths = {
            1: "/World/Tinker/left_finger",
            2: "/World/Scenario/delivery_object",
        }
        backend._contact_path_decoder = paths.__getitem__
        header = SimpleNamespace(
            actor0=1,
            actor1=2,
            collider0=11,
            collider1=22,
            type="found",
            contact_data_offset=0,
            num_contact_data=1,
        )
        sample = SimpleNamespace(
            impulse=(0.3, 0.4, 0.5),
            position=(0.1, 0.2, 0.3),
            normal=(0.0, 0.0, 1.0),
        )
        backend._on_contact_report_event([header], [sample])

        pairs = backend.contact_pairs()
        self.assertEqual(pairs[0]["body_a"], "/World/Tinker/left_finger")
        self.assertEqual(pairs[0]["body_b"], "/World/Scenario/delivery_object")
        self.assertAlmostEqual(float(pairs[0]["normal_force"]), 5.0)
        for actual, expected in zip(pairs[0]["normal"], [0.0, 0.0, 1.0]):
            self.assertAlmostEqual(float(actual), expected)
        for actual, expected in zip(pairs[0]["point"], [0.1, 0.2, 0.3]):
            self.assertAlmostEqual(float(actual), expected)
        self.assertTrue(backend.contact_state()["left_finger"]["in_contact"])

        header.type = "lost"
        header.num_contact_data = 0
        backend._on_contact_report_event([header], [])
        self.assertEqual(backend.contact_pairs(), [])
        self.assertFalse(backend.contact_state()["left_finger"]["in_contact"])

    def test_contact_report_sums_normal_impulses_without_tangential_cancellation(self) -> None:
        backend = _backend()
        backend.dt = 0.1
        backend._contact_event_found = "found"
        backend._contact_event_lost = "lost"
        backend._contact_event_persist = "persist"
        backend._contact_path_decoder = {
            1: "/World/Tinker/left_finger",
            2: "/World/Scenario/delivery_object",
        }.__getitem__
        header = SimpleNamespace(
            actor0=1,
            actor1=2,
            collider0=11,
            collider1=22,
            type="found",
            contact_data_offset=0,
            num_contact_data=3,
        )
        backend._on_contact_report_event(
            [header],
            [
                SimpleNamespace(
                    impulse=(3.0, 4.0, 0.0),
                    position=(0.0, 0.0, 0.0),
                    normal=(1.0, 0.0, 0.0),
                ),
                SimpleNamespace(
                    impulse=(-3.0, 4.0, 0.0),
                    position=(0.0, 0.0, 0.0),
                    normal=(1.0, 0.0, 0.0),
                ),
                SimpleNamespace(
                    impulse=(0.0, 2.0, 1.0),
                    position=(0.0, 0.0, 0.0),
                    normal=(0.0, 1.0, 0.0),
                ),
            ],
        )

        pair = backend.contact_pairs()[0]
        self.assertAlmostEqual(float(pair["normal_force"]), 80.0)
        for actual, expected in zip(pair["normal"], [0.948683298, 0.316227766, 0.0]):
            self.assertAlmostEqual(float(actual), expected, places=6)

    def test_contact_report_uses_deterministic_normal_for_degenerate_average(self) -> None:
        backend = _backend()
        backend.dt = 0.1
        backend._contact_event_found = "found"
        backend._contact_event_lost = "lost"
        backend._contact_event_persist = "persist"
        backend._contact_path_decoder = {
            1: "/World/Tinker/left_finger",
            2: "/World/Scenario/delivery_object",
        }.__getitem__
        header = SimpleNamespace(
            actor0=1,
            actor1=2,
            collider0=11,
            collider1=22,
            type="found",
            contact_data_offset=0,
            num_contact_data=2,
        )
        backend._on_contact_report_event(
            [header],
            [
                SimpleNamespace(
                    impulse=(0.0, 0.0, 2.0),
                    position=(0.0, 0.0, 0.0),
                    normal=(0.0, 0.0, 1.0),
                ),
                SimpleNamespace(
                    impulse=(0.0, 0.0, -2.0),
                    position=(0.0, 0.0, 0.0),
                    normal=(0.0, 0.0, -1.0),
                ),
            ],
        )

        pair = backend.contact_pairs()[0]
        self.assertAlmostEqual(float(pair["normal_force"]), 40.0)
        self.assertEqual(pair["normal"], [0.0, 0.0, 1.0])

    def test_truth_objects_are_measured_and_expected_stays_separate(self) -> None:
        backend = _backend()
        backend.dt = 0.1
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 0)
        backend._clock_step_origin = 0
        backend.scenario = "qualification-retention"
        backend.task = "qualification-retention"
        backend.physics_device = "cpu"
        backend.seed = 7
        expected = {"delivery_object": {"class_name": "dynamic_cube"}}
        actual = [{"id": "delivery_object", "pose": {"xyz": [1.0, 2.0, 3.0]}}]
        backend._expected_objects = expected
        backend._actual_object_states = lambda: actual

        truth = backend.truth_state(backend.TRUTH_TOKEN)
        self.assertEqual(truth["schema_version"], 2)
        self.assertEqual(
            truth["command_targets"]["joint_names"], ["drive_joint", "joint1"]
        )
        self.assertEqual(truth["expected_objects"], expected)
        self.assertEqual(truth["objects"], actual)
        self.assertEqual(truth["object"], actual[0])

    def test_measured_truth_normalizes_non_torch_rigid_view_arrays(self) -> None:
        backend = _backend()
        backend._expected_objects = {
            "delivery_object": {
                "class_name": "dynamic_cube",
                "actual_prim_path": "/World/Scenario/delivery_object",
            }
        }
        backend._object_views = {"delivery_object": _FakeRigidView()}

        objects = backend._actual_object_states()

        self.assertEqual(objects[0]["id"], "delivery_object")
        self.assertEqual(objects[0]["class_name"], "dynamic_cube")
        self.assertEqual(objects[0]["prim_path"], "/World/Scenario/delivery_object")
        for actual, expected in zip(objects[0]["pose"]["xyz"], [0.65, -0.1, 0.8]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(
            objects[0]["pose"]["quaternion_xyzw"], [0.1, 0.2, 0.3, 0.4]
        ):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(objects[0]["twist"]["linear"], [0.01, 0.02, 0.03]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(objects[0]["twist"]["angular"], [0.04, 0.05, 0.06]):
            self.assertAlmostEqual(actual, expected)

        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 0)
        backend._clock_step_origin = 0
        backend.dt = 1.0 / 120.0
        backend.scenario = "pick-deliver-place"
        backend.task = "pick-deliver-place"
        backend.physics_device = "cpu"
        backend.seed = 7
        truth = backend.truth_state(backend.TRUTH_TOKEN)
        self.assertEqual(truth["objects"], objects)
        self.assertEqual(truth["object"], objects[0])

    def test_manipulation_profile_and_artifact_are_strict(self) -> None:
        profile = json.loads(
            (ROOT / "simulation/profiles/manipulation-core.json").read_text(encoding="utf-8")
        )
        self.assertEqual(profile["physics_device"], "cpu")
        self.assertFalse(profile["render"])
        self.assertTrue(profile["contacts"])
        launch_source = (ROOT / "validation/run_sim.py").read_text(encoding="utf-8")
        self.assertIn("add_ground_plane=True", launch_source)
        artifact = _content_addressed_tinker_usd(ROOT, None)
        self.assertEqual(artifact.name, "robot.usd")
        with self.assertRaises(RuntimeError):
            _content_addressed_tinker_usd(ROOT, ROOT / "simulation/assets/primitives/task-object.usda")

    def test_expected_object_pose_and_twist_comes_from_scenario(self) -> None:
        expected = _expected_scenario_objects(ROOT, "pick-deliver-place")
        # Pose pinned to the arena-safe layout (e5d4312): the old (0.65, 0)
        # point sits inside shelf_02's rasterized footprint in rcw2026.
        self.assertEqual(
            expected["delivery_object"]["pose"]["position"], [-1.35, -2.0, 0.8]
        )
        self.assertEqual(expected["delivery_object"]["twist"]["linear"], [0.0, 0.0, 0.0])
        self.assertEqual(
            expected["delivery_object"]["prim_path"], "/World/Scenario/delivery_object"
        )
        classified = _expected_scenario_objects(ROOT, "qualification-retention")
        self.assertEqual(classified["qualification_cube"]["class_name"], "dynamic_cube")

    def test_gateway_uses_bool_stop_and_internal_truth_only(self) -> None:
        source = (ROOT / "simulation/tinker_sim_isaac/ros_gateway.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("Bool, \"/sim/hardware/safety_stop\"", source)
        self.assertIn('JointState, "/isaac_joint_states", reliable', source)
        self.assertIn('WrenchStamped, "/sim/parity/finger_contact", reliable', source)
        self.assertIn('Bool, "/sim/safety/collision", collision_qos', source)
        self.assertIn("durability=DurabilityPolicy.TRANSIENT_LOCAL", source)
        self.assertIn("initial_collision.data = False", source)
        self.assertIn("self.collision_pub.publish(initial_collision)", source)
        self.assertIn("backend.arm_scenario_collision()", source)
        self.assertIn("/sim/internal/physics_truth", source)
        self.assertNotIn("self.truth_pub", source)

    def test_finger_contact_wrench_publishes_every_tick_not_at_status_cadence(self) -> None:
        """Bug #22: /sim/parity/finger_contact was nested inside the 2 Hz
        _status_stride heartbeat block, so it read all-zero across most of a
        grasp trial even while /sim/internal/physics_truth (unconditional,
        every tick) carried real contact force. The wrench must publish on
        every control tick, independent of the status cadence."""
        from builtin_interfaces.msg import Time
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String
        from geometry_msgs.msg import WrenchStamped

        class _ContactBackend:
            dt = 0.02
            physics_device = "cpu"
            safety_stopped = False
            simulation_time = 0.0
            TRUTH_TOKEN = object()

            def joint_state(self):
                return ((), [], [], [])

            def root_state(self):
                return {"angular_velocity_world": (0.0, 0.0, 0.0)}

            def contact_state(self):
                return {
                    "left_finger": {"force": 3.5},
                    "right_finger": {"force": 2.5},
                }

            def physics_truth_frame(self, token):
                return {}

        class _RecordingPublisher:
            def __init__(self) -> None:
                self.messages: list[object] = []

            def publish(self, message) -> None:
                self.messages.append(message)

        gateway = object.__new__(RosStandardGateway)
        gateway.backend = _ContactBackend()
        gateway._Clock = Clock
        gateway._JointState = JointState
        gateway._Imu = Imu
        gateway._String = String
        gateway._WrenchStamped = WrenchStamped
        gateway.clock_pub = _RecordingPublisher()
        gateway.joint_pub = _RecordingPublisher()
        gateway.imu_pub = _RecordingPublisher()
        gateway.status_pub = _RecordingPublisher()
        gateway.contact_pub = _RecordingPublisher()
        gateway.physics_truth_pub = _RecordingPublisher()
        gateway.cloud_pub = _RecordingPublisher()
        gateway._camera_rig = None
        gateway._cloud_publish_enabled = lambda: False
        gateway._last_command_error = None
        gateway._command_stream_lost = False
        gateway._command_epoch = 0
        gateway._last_logical_snapshot_id = -1
        gateway.development_lidar = False
        gateway._publish_profile_enabled = False
        # Large strides so state/imu/status only fire on tick 0 (0 % N == 0
        # for any N); every subsequent tick must skip the status heartbeat.
        gateway._state_stride = 1_000_000
        gateway._imu_stride = 1_000_000
        gateway._status_stride = 1_000_000
        gateway._tick = 0

        for _ in range(3):
            gateway.publish()

        self.assertEqual(
            len(gateway.status_pub.messages), 1,
            "status heartbeat should only fire on tick 0 with a huge stride",
        )
        self.assertEqual(
            len(gateway.contact_pub.messages), 3,
            "finger_contact wrench must publish every tick, not gated on "
            "_status_stride like the status heartbeat",
        )
        forces = [msg.wrench.force.z for msg in gateway.contact_pub.messages]
        self.assertTrue(all(abs(force - 6.0) < 1e-6 for force in forces))

    def test_gateway_publishes_raw_truth_without_persisting_physics_truth(self) -> None:
        source = (ROOT / "simulation/tinker_sim_isaac/ros_gateway.py").read_text(
            encoding="utf-8"
        )
        # The gateway still publishes the raw serialized payload on the internal
        # physics-truth topic for the evaluator to consume.
        self.assertIn('"/sim/internal/physics_truth"', source)
        self.assertIn("physics_truth.data", source)
        self.assertIn("self.physics_truth_pub.publish(physics_truth)", source)
        # Raw truth persistence is owned by the evaluator callback, so the
        # gateway must not hold a physics-truth jsonl writer (the TINKER_SIM_TRUTH_JSONL
        # leak that double-wrote physics_truth.jsonl in the Isaac process).
        self.assertNotIn("self._truth_writer", source)

    def test_physics_truth_writer_appends_sorted_finite_json_once_per_frame(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "physics_truth.jsonl"
            writer = PhysicsTruthJsonlWriter(path)
            first = writer.append({"z": 1, "a": [0.25, 2.0]})
            second = writer.append({"frame": 2, "objects": []})
            writer.close()

            self.assertEqual(first, '{"a": [0.25, 2.0], "z": 1}')
            self.assertEqual(second, '{"frame": 2, "objects": []}')
            self.assertEqual(
                path.read_text(encoding="utf-8").splitlines(), [first, second]
            )
            self.assertEqual(
                [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()],
                [{"a": [0.25, 2.0], "z": 1}, {"frame": 2, "objects": []}],
            )

            append_writer = PhysicsTruthJsonlWriter(path)
            append_writer.append({"frame": 3, "object": None})
            append_writer.close()
            self.assertEqual(len(path.read_text(encoding="utf-8").splitlines()), 3)

    def test_physics_truth_writer_rejects_non_finite_values(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            writer = PhysicsTruthJsonlWriter(Path(directory) / "physics_truth.jsonl")
            with self.assertRaises(ValueError):
                writer.append({"bad": float("nan")})
            writer.close()

    def test_physics_truth_writer_is_disabled_without_path(self) -> None:
        writer = PhysicsTruthJsonlWriter(None)
        self.assertEqual(writer.append({"objects": [], "object": None}), '{"object": null, "objects": []}')
        self.assertIsNone(writer.path)
        writer.close()

    def test_physics_truth_writer_uses_qualification_environment_path(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "physics_truth.jsonl"
            with patch.dict(os.environ, {"TINKER_SIM_TRUTH_JSONL": str(path)}):
                writer = PhysicsTruthJsonlWriter.from_environment()
            writer.append({"objects": [], "object": None})
            writer.close()
            self.assertEqual(
                path.read_text(encoding="utf-8"),
                '{"object": null, "objects": []}\n',
            )

    def test_global_contact_callback_and_failure_status_contract(self) -> None:
        backend_source = (
            ROOT / "simulation/tinker_sim_isaac/backend.py"
        ).read_text(encoding="utf-8")
        run_source = (ROOT / "validation/run_sim.py").read_text(encoding="utf-8")
        cli_source = (ROOT / "tools/tinker_sim_deploy/cli.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("subscribe_contact_report_events", backend_source)
        self.assertNotIn("ContactSensor(", backend_source)
        self.assertNotIn("filter_prim_paths_expr", backend_source)
        self.assertIn("exit_code=1 if failed else 0", run_source)
        self.assertIn("return process.wait()", cli_source)

    def test_spawned_joint_drives_are_force_mode_before_articulation_init(self) -> None:
        backend_source = (
            ROOT / "simulation/tinker_sim_isaac/backend.py"
        ).read_text(encoding="utf-8")
        spawn_source = _spawn_usd_file_cfg_source(backend_source)
        self.assertTrue(
            spawn_source,
            "backend must spawn Tinker through a UsdFileCfg; no spawn call found",
        )
        # Spawned dynamic joint drives must be force-mode before the articulation
        # initializes, so the spawn's UsdFileCfg carries the joint-drive props.
        self.assertIn("joint_drive_props", spawn_source)
        self.assertIn("drive_type", spawn_source)
        self.assertIn("force", spawn_source)
        # The existing per-group ImplicitActuatorCfg gains remain authoritative.
        # The arm group carries per-joint stiffness/damping tiers as dicts;
        # head/gripper/wheels carry stiffness and damping as scalar keyword
        # arguments.
        self.assertIn('"joint[1-2]": 20000.0,', backend_source)
        self.assertIn('"joint[3]": 6000.0,', backend_source)
        self.assertIn('"joint[4]": 7000.0,', backend_source)
        self.assertIn('"joint[5]": 6000.0,', backend_source)
        self.assertIn('"joint[6]": 12000.0,', backend_source)
        self.assertIn('"joint[7]": 4000.0,', backend_source)
        self.assertIn('"joint[1-2]": 1500.0,', backend_source)
        self.assertIn('"joint[3]": 450.0,', backend_source)
        self.assertIn('"joint[4]": 600.0,', backend_source)
        self.assertIn('"joint[5]": 450.0,', backend_source)
        self.assertIn('"joint[6]": 800.0,', backend_source)
        self.assertIn('"joint[7]": 300.0,', backend_source)
        self.assertIn("stiffness=500.0", backend_source)
        self.assertIn("damping=50.0", backend_source)
        self.assertIn("stiffness=200.0", backend_source)
        self.assertIn("damping=20.0", backend_source)
        self.assertIn("stiffness=0.0", backend_source)
        self.assertIn("damping=200.0", backend_source)

    def test_arm_effort_limit_sim_resolves_to_seven_joint_tiers(self) -> None:
        """RED source contract: joint4 must be raised to 50 Nm, all other tiers preserved.

        A fresh-attempt live smoke (task43-force-truth-free-space-fjt) showed joint4
        alone had RMS ~0.180 rad / peak 0.342 rad because the USD/vendor cap pinned it
        at 30 Nm for ~72% of the action. The `arm` ImplicitActuatorCfg must therefore
        carry an ``effort_limit_sim`` dict that resolves across joint1..joint7 to
        [50, 50, 30, 50, 30, 20, 20] with a strict one-to-one match (exactly Isaac Lab's
        ``resolve_matching_names_values`` semantics via ``re.fullmatch``). No specific
        regex spelling is hard-coded; the keys only need full, non-overlapping coverage.
        """
        backend_source = (
            ROOT / "simulation/tinker_sim_isaac/backend.py"
        ).read_text(encoding="utf-8")
        tree = ast.parse(backend_source)

        arm_call: ast.Call | None = None
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            if not isinstance(func, ast.Name) or func.id != "ArticulationCfg":
                continue
            for keyword in node.keywords:
                if keyword.arg != "actuators":
                    continue
                actuators = keyword.value
                if not isinstance(actuators, ast.Dict):
                    continue
                for key, value in zip(actuators.keys, actuators.values):
                    if isinstance(key, ast.Constant) and key.value == "arm":
                        arm_call = value

        self.assertIsNotNone(
            arm_call,
            "backend ArticulationCfg.actuators has no 'arm' ImplicitActuatorCfg",
        )
        self.assertIsInstance(
            arm_call,
            ast.Call,
            "backend 'arm' actuator must be an ImplicitActuatorCfg(...) call",
        )

        effort_limit_sim = None
        for keyword in arm_call.keywords:
            if keyword.arg == "effort_limit_sim":
                effort_limit_sim = keyword.value
        self.assertIsNotNone(
            effort_limit_sim,
            "arm ImplicitActuatorCfg has no effort_limit_sim; joint4 stays pinned at "
            "the inherited 30 Nm vendor cap for most of the action",
        )

        limits = ast.literal_eval(effort_limit_sim)
        self.assertIsInstance(
            limits,
            dict,
            "arm effort_limit_sim must be a per-joint dict[str, float] so joint4 can "
            "be raised independently of its siblings",
        )

        joint_names = [f"joint{index}" for index in range(1, 8)]
        match_counts = {name: 0 for name in joint_names}
        resolved: dict[str, float] = {}
        for pattern, value in limits.items():
            self.assertIsInstance(
                pattern,
                str,
                "arm effort_limit_sim keys must be joint-name regex strings",
            )
            compiled = re.compile(pattern)
            for name in joint_names:
                if compiled.fullmatch(name) is not None:
                    match_counts[name] += 1
                    resolved[name] = float(value)

        self.assertEqual(
            match_counts,
            {name: 1 for name in joint_names},
            "arm effort_limit_sim regex keys must cover each of joint1..joint7 exactly "
            f"once; missing or multiply matched joints: {match_counts}",
        )
        self.assertEqual(
            [resolved[name] for name in joint_names],
            [100.0, 100.0, 30.0, 50.0, 30.0, 20.0, 20.0],
            "arm effort_limit_sim must provide 100 Nm shoulder authority for the "
            "measured 60-90 Nm grasp coupling load while preserving the elbow and "
            "distal tiers (j3:30, j4:50, j5:30, j6-7:20)",
        )

    def test_gripper_mimic_joints_driven_and_mirrored_to_drive_joint(self) -> None:
        """Source contract: the gripper mimic joints are actively driven and
        mirror drive_joint 1:1.

        The tinker2 gripper is a mimic linkage -- the URDF mimics all five
        finger/knuckle joints (left_finger_joint, left_inner_knuckle_joint,
        right_inner_knuckle_joint, right_outer_knuckle_joint, right_finger_joint)
        to drive_joint 1:1. But the URDF->USD import DROPPED every <mimic>: in
        robot.usd those joints carry no drive and no coupling (proven), so the
        earlier "leave them passive" contract let drive_joint close while the
        fingers flopped and the jaw closed straight through the object. The
        repair restores the coupling in software: the five mimics live in a
        distinct 'gripper_mimic' group with NON-ZERO gains, and step() mirrors
        drive_joint's target into them each step
        (_mirror_gripper_mimic_targets). The 'gripper' actuator still scopes to
        drive_joint only.
        """
        backend_source = (
            ROOT / "simulation/tinker_sim_isaac/backend.py"
        ).read_text(encoding="utf-8")
        tree = ast.parse(backend_source)

        actuators: ast.Dict | None = None
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            if not isinstance(func, ast.Name) or func.id != "ArticulationCfg":
                continue
            for keyword in node.keywords:
                if keyword.arg == "actuators" and isinstance(keyword.value, ast.Dict):
                    actuators = keyword.value
        self.assertIsNotNone(
            actuators,
            "backend ArticulationCfg.actuators must be a dict literal of actuator groups",
        )

        gripper_universe = {
            "drive_joint",
            "left_finger_joint",
            "left_inner_knuckle_joint",
            "right_inner_knuckle_joint",
            "right_outer_knuckle_joint",
            "right_finger_joint",
        }
        mimic_joints = frozenset(gripper_universe - {"drive_joint"})

        def cfg_kwargs(cfg: ast.expr) -> dict[str, object]:
            self.assertIsInstance(
                cfg,
                ast.Call,
                "each actuator group must be an ImplicitActuatorCfg(...) call",
            )
            assert isinstance(cfg, ast.Call)
            call_func = cfg.func
            if isinstance(call_func, ast.Name):
                self.assertEqual(
                    call_func.id,
                    "ImplicitActuatorCfg",
                    "actuator groups must be implicit actuator configs",
                )
            kwargs: dict[str, object] = {}
            for kw in cfg.keywords:
                if kw.arg is None:
                    continue
                kwargs[kw.arg] = ast.literal_eval(kw.value)
            return kwargs

        def matched_joints(exprs: object) -> set[str]:
            self.assertIsInstance(
                exprs, list, "joint_names_expr must be a literal list of regex strings"
            )
            matched: set[str] = set()
            for pattern in exprs:
                self.assertIsInstance(pattern, str)
                compiled = re.compile(pattern)
                for name in gripper_universe:
                    if compiled.fullmatch(name) is not None:
                        matched.add(name)
            return matched

        groups: dict[str, dict[str, object]] = {}
        for key, value in zip(actuators.keys, actuators.values):
            if not isinstance(key, ast.Constant):
                continue
            groups[str(key.value)] = cfg_kwargs(value)

        self.assertIn("gripper", groups, "backend must define a 'gripper' actuator group")
        gripper = groups["gripper"]
        gripper_matched = matched_joints(gripper.get("joint_names_expr"))
        self.assertEqual(
            gripper_matched,
            {"drive_joint"},
            "the 'gripper' actuator must actively drive only drive_joint; it currently "
            f"also claims {sorted(gripper_matched - {'drive_joint'})}",
        )
        gripper_stiffness = gripper.get("stiffness")
        self.assertIsNotNone(
            gripper_stiffness,
            "the 'gripper' actuator must carry an explicit non-zero stiffness",
        )
        self.assertNotEqual(
            float(gripper_stiffness),  # type: ignore[arg-type]
            0.0,
            "the 'gripper' actuator must keep a non-zero stiffness to actively drive drive_joint",
        )

        driven = [
            (name, cfg)
            for name, cfg in groups.items()
            if matched_joints(cfg.get("joint_names_expr")) == set(mimic_joints)
        ]
        self.assertTrue(
            driven,
            "no actuator group claims exactly the five mimic joints "
            f"{sorted(mimic_joints)}; they must live in a distinct group",
        )
        for name, cfg in driven:
            self.assertNotEqual(
                float(cfg.get("stiffness", 0.0)),  # type: ignore[arg-type]
                0.0,
                f"mimic group {name!r} must be actively driven (non-zero "
                "stiffness): robot.usd dropped the URDF mimic coupling, so a "
                "passive group leaves the fingers flopping and the jaw closes "
                "through the object",
            )

        # The coupling the USD dropped is restored in software: step() must
        # mirror drive_joint's MEASURED angle into the five mimic joints so
        # they track the master 1:1 (URDF mimic semantics; see the behavioral
        # test below).
        self.assertIn(
            "_mirror_gripper_mimic_targets",
            backend_source,
            "backend must mirror drive_joint's measured angle into the mimic joints",
        )
        for joint in mimic_joints:
            self.assertIn(
                joint,
                backend_source,
                f"the gripper mimic mirror must reference {joint}",
            )

    def test_gripper_mimic_followers_track_measured_drive_angle(self) -> None:
        """URDF ``<mimic joint="drive_joint" multiplier="1">`` means
        ``q_follower = q_drive`` -- the driving joint's ACTUAL angle. The real
        xArm gripper is one motor plus gear-coupled, parallelogram fingers, so
        a blocked drive knuckle stops the whole linkage.

        Mirroring the COMMANDED target instead breaks that the moment the
        object blocks the drive: the five followers keep chasing the far
        target as independent k=1500 motors -- the right knuckle over-runs
        the stalled left one and shoves the object into the weak pad, and the
        finger joints curl the pads about the finger axis (measured in-process:
        drive stalled at 0.43 rad while the pads ran to 0.845, 25 N vs 5 N
        pads, the object tilted 12 deg; the grasp bench's "pads close through
        to 0.728, 18 mm past the knife"). With the measured-angle mirror the
        same closes pinch symmetrically and retain through a lift.
        """
        names = (
            "drive_joint",
            "left_finger_joint",
            "left_inner_knuckle_joint",
            "right_outer_knuckle_joint",
            "right_finger_joint",
            "right_inner_knuckle_joint",
            "joint1",
        )
        backend = object.__new__(IsaacWholeRobotBackend)
        backend._torch = torch
        backend.dt = 1.0 / 120.0
        backend._drive_joint_index = 0
        backend._gripper_mimic_indices = (1, 2, 3, 4, 5)
        # Drive blocked by the object at 0.43 rad, still pushing at 1.2 rad/s
        # commanded slew; the command target sits far ahead at 0.85.
        backend._robot = SimpleNamespace(
            data=SimpleNamespace(
                joint_names=names,
                joint_pos=torch.tensor([[0.43, 0.6, 0.6, 0.6, 0.6, 0.6, 0.0]], dtype=torch.float32),
                joint_vel=torch.tensor([[1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], dtype=torch.float32),
            )
        )
        backend._position_targets = torch.tensor(
            [[0.85, 0.85, 0.85, 0.85, 0.85, 0.85, 0.0]], dtype=torch.float32
        )
        backend._mirror_gripper_mimic_targets()
        expected = 0.43 + 1.2 / 120.0  # measured angle + one control step of feed-forward
        for index in backend._gripper_mimic_indices:
            self.assertAlmostEqual(
                float(backend._position_targets[0, index]),
                expected,
                places=6,
                msg=f"follower {names[index]} must track the drive's measured angle, not its 0.85 target",
            )
        # The drive's own target is untouched (the master keeps its command).
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), 0.85, places=6)
        # At stall (zero drive velocity) the followers hold exactly the drive angle.
        backend._robot.data.joint_vel[0, 0] = 0.0
        backend._mirror_gripper_mimic_targets()
        for index in backend._gripper_mimic_indices:
            self.assertAlmostEqual(float(backend._position_targets[0, index]), 0.43, places=6)

    def test_gripper_lead_clamp_defaults_off_with_measured_mirror(self) -> None:
        """With mimic-correct followers the press is bounded by drive_joint's
        effort limit, and the stall-gated lead clamp self-locks the close into
        a 0.1 rad/s crawl (pads trail the drive by one step, so pad speed sits
        at the gate). It must default OFF; the env knob keeps it available."""
        backend_source = (ROOT / "simulation/tinker_sim_isaac/backend.py").read_text(encoding="utf-8")
        self.assertRegex(backend_source, r"self\._gripper_max_lead = 0\.0\s*\n")
        self.assertNotRegex(backend_source, r"self\._gripper_max_lead = 0\.015")
        self.assertIn("TINKER_SIM_GRIPPER_MAX_LEAD_RAD", backend_source)

    def test_set_entity_pose_physics_writes_transform_and_velocity(self) -> None:
        """A physics-effective park write pushes the world pose + zeroed twist
        into a cached rigid-body view as WARP arrays (the PhysxManager view's
        frontend), in xyzw order (no reorder), reusing the cached view."""
        import warp as wp

        class _RecordingView:
            count = 1

            def __init__(self) -> None:
                self.transforms = None
                self.velocities = None

            def get_transforms(self):
                # The method reads .device off this to place the warp arrays.
                return SimpleNamespace(device="cpu")

            def set_transforms(self, data, indices) -> None:
                self.transforms = data
                self.transform_indices = indices

            def set_velocities(self, data, indices) -> None:
                self.velocities = data

        backend = object.__new__(IsaacWholeRobotBackend)
        backend._torch = torch
        backend._robot = SimpleNamespace(device="cpu")
        view = _RecordingView()
        path = "/World/Scenario/bench_bottle_100"
        backend._park_views = {path: view}

        ok = backend.set_entity_pose_physics(
            path,
            position=[1.0, 2.0, 0.05],
            quaternion_xyzw=[0.0, 0.0, 0.3826834, 0.9238795],  # yaw 45 deg, xyzw
            linear_velocity=[0.0, 0.0, 0.0],
            angular_velocity=[0.0, 0.0, 0.0],
        )
        self.assertTrue(ok)
        # Warp arrays, required by the warp frontend (torch/numpy dtypes rejected).
        self.assertIsInstance(view.transforms, wp.array)
        self.assertIsInstance(view.transform_indices, wp.array)
        row = view.transforms.numpy()[0].tolist()
        self.assertEqual([round(v, 4) for v in row[:3]], [1.0, 2.0, 0.05])
        self.assertEqual(
            [round(v, 5) for v in row[3:7]], [0.0, 0.0, 0.38268, 0.92388]
        )
        vel = view.velocities.numpy()[0].tolist()
        self.assertEqual([round(v, 4) for v in vel], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertIs(backend._park_views[path], view)

    def test_set_entity_pose_physics_false_without_resolvable_view(self) -> None:
        """No cached view and no live PhysX (unit env) -> graceful False."""
        backend = object.__new__(IsaacWholeRobotBackend)
        backend._torch = torch
        backend._robot = SimpleNamespace(device="cpu")
        backend._park_views = {}
        self.assertFalse(
            backend.set_entity_pose_physics(
                "/World/Scenario/missing", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
            )
        )


if __name__ == "__main__":
    unittest.main()
