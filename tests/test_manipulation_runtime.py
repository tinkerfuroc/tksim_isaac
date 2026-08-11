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
    backend._sim = SimpleNamespace(
        get_physics_step_count=lambda: 0,
        step=lambda render=False: None,
    )
    backend.render = False
    backend.dt = 1.0 / 120.0
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
        self.assertAlmostEqual(float(backend._effort_targets[0, 1]), 3.7, places=4)
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), 0.25)
        self.assertAlmostEqual(float(backend._position_targets[0, 1]), -0.4)
        self.assertEqual(backend._robot.write_count, 2)
        self.assertEqual(
            [name for name, _ in backend._robot.gain_calls],
            ["stiffness", "damping", "stiffness", "damping"],
        )
        self.assertEqual(
            [call["limits"].tolist() for call in backend._robot.limit_calls[-2:]],
            [[[100.0]], [[100.0]]],
        )
        self.assertEqual(
            [call["stiffness"] for name, call in backend._robot.gain_calls if name == "stiffness"],
            [IsaacWholeRobotBackend.SAFETY_HOLD_STIFFNESS] * 2,
        )
        self.assertEqual(
            [call["damping"] for name, call in backend._robot.gain_calls if name == "damping"],
            [IsaacWholeRobotBackend.SAFETY_HOLD_DAMPING] * 2,
        )
        self.assertEqual(
            [kind for kind, _ in backend._robot.target_calls[-3:]],
            ["position", "velocity", "effort"],
        )
        self.assertEqual(
            [kind for kind, _ in backend._robot.target_calls],
            ["position", "velocity", "effort"] * 2,
        )
        effort_targets = [
            target for kind, target in backend._robot.target_calls if kind == "effort"
        ]
        self.assertEqual(len(effort_targets), 2)
        for target in effort_targets:
            self.assertAlmostEqual(float(target[0, 0]), 0.0)
            self.assertAlmostEqual(float(target[0, 1]), 3.7, places=4)

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

    def test_safety_hold_uses_explicit_drive_and_restores_nominal_gain(self) -> None:
        backend = _backend()
        backend.set_safety_stop(True)
        backend.step()

        self.assertEqual(
            [call["stiffness"] for name, call in backend._robot.gain_calls if name == "stiffness"],
            [IsaacWholeRobotBackend.SAFETY_HOLD_STIFFNESS],
        )
        self.assertEqual(
            [call["damping"] for name, call in backend._robot.gain_calls if name == "damping"],
            [IsaacWholeRobotBackend.SAFETY_HOLD_DAMPING],
        )
        self.assertEqual(backend._robot.target_calls[-1][0], "effort")
        self.assertAlmostEqual(float(backend._robot.target_calls[-1][1][0, 1]), 17.5)
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
        # The measured error is -0.01 rad and velocity is +0.01 rad/s.
        # -0.5 + 600(-0.01) - 80(0.01) = -7.3.
        self.assertAlmostEqual(float(backend._effort_targets[0, 0]), 0.0)
        self.assertAlmostEqual(float(backend._effort_targets[0, 1]), -7.3, places=4)

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
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), 0.6)

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
        self.assertAlmostEqual(float(backend._position_targets[0, 0]), 0.6)

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
        self.assertEqual(
            expected["delivery_object"]["pose"]["position"], [0.65, 0.0, 0.8]
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

    def test_gripper_actuator_drives_only_drive_joint_and_mimics_are_passive(self) -> None:
        """RED source contract: the gripper actuator drives only drive_joint.

        The five authored USD mimic joints (left_finger_joint, left_inner_knuckle_joint,
        right_inner_knuckle_joint, right_outer_knuckle_joint, right_finger_joint) follow
        drive_joint through the bundle URDF mimic constraints. They must not be actively
        driven by the 'gripper' ImplicitActuatorCfg, or the PD loop competes with the
        mimic constraint and the fingers fight the drive. The repair must scope the
        'gripper' actuator to drive_joint only and assign the five mimics to a distinct
        passive implicit actuator group with zero stiffness and zero damping. No explicit
        mimic target propagation is required.
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

        passive = [
            (name, cfg)
            for name, cfg in groups.items()
            if matched_joints(cfg.get("joint_names_expr")) == set(mimic_joints)
        ]
        self.assertTrue(
            passive,
            "no actuator group claims exactly the five mimic joints "
            f"{sorted(mimic_joints)}; they must live in a distinct passive group",
        )
        for name, cfg in passive:
            self.assertEqual(
                float(cfg.get("stiffness", 1.0)),  # type: ignore[arg-type]
                0.0,
                f"mimic passive group {name!r} must have zero stiffness",
            )
            self.assertEqual(
                float(cfg.get("damping", 1.0)),  # type: ignore[arg-type]
                0.0,
                f"mimic passive group {name!r} must have zero damping",
            )


if __name__ == "__main__":
    unittest.main()
