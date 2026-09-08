from __future__ import annotations

import ast
import contextlib
import io
import json
import math
import os
import queue
import re
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "validation"))
CAMERA_CONTRACT = ROOT / "simulation/sensors/hardware-parity.json"

from tinker_sim_core.command_mux import (
    JointCommand,
    encode_command_epoch,
    encode_command_frame,
    encode_snapshot_packet,
)
from tinker_sim_isaac.backend import (
    GRIPPER_COMMAND_TARGET_LOG_MAX_PER_WINDOW,
    GRIPPER_EFFORT_CEILING_NM,
    GRIPPER_EFFORT_FULL_SCALE_N,
    IsaacWholeRobotBackend,
    format_spawn_pose_trace,
    gripper_effort_limit_nm,
    resolve_backend_clock_epoch,
    resolve_clock_epoch,
    resolve_gripper_effort_ceiling_nm,
    resolve_gripper_effort_full_scale_n,
    resolve_spawn_yaw,
    resolve_spawn_yaw_via_view,
    resolve_use_fabric,
    spawn_root_rot_xyzw,
)
from tinker_sim_isaac.camera_rig import (
    OPTICAL_TO_USD_CAMERA_WXYZ,
    CameraRig,
    _rotate_by_quaternion,
    load_camera_specs,
    usd_camera_pose_to_ros_optical,
)
from tinker_sim_isaac.target_write_gate import TargetWriteGate
from tinker_sim_isaac.ros_gateway import PhysicsTruthJsonlWriter, RosStandardGateway
from manipulation_qualification import QualificationManifest, QualificationRunner
from run_sim import _content_addressed_tinker_usd, _expected_scenario_objects


class _FakeGripperRootView:
    """Minimal double for the PhysX ArticulationView's max-force tensor API
    (root_view.set_dof_max_forces / get_dof_max_forces), #20's direct-write
    path. Holds one (1, num_joints) row, mirroring the real tensor-API shape.
    """

    def __init__(self, num_joints: int, initial: list[float]) -> None:
        self._row = list(initial)
        assert len(self._row) == num_joints
        self.set_dof_max_forces_calls: list[dict[str, object]] = []

    def set_dof_max_forces(self, forces: object, indices: object = None) -> None:
        arr = forces.numpy() if hasattr(forces, "numpy") else forces
        self._row = [float(value) for value in arr[0]]
        self.set_dof_max_forces_calls.append({"forces": arr, "indices": indices})

    def get_dof_max_forces(self) -> list[list[float]]:
        return [list(self._row)]


class _FakeUsdPrim:
    """Minimal USD prim double: enough surface for the stage walk that the
    (removed) runtime DriveAPI authoring performed -- GetName / IsValid.
    """

    def __init__(self, name: str, children: tuple["_FakeUsdPrim", ...] = ()) -> None:
        self._name = name
        self.children = children

    def GetName(self) -> str:  # noqa: N802 - mirrors the pxr API spelling
        return self._name

    def IsValid(self) -> bool:  # noqa: N802 - mirrors the pxr API spelling
        return True


class _FakeUsdStage:
    def __init__(self, root: _FakeUsdPrim) -> None:
        self._root = root
        self.get_prim_at_path_calls: list[str] = []

    def GetPrimAtPath(self, path: str) -> _FakeUsdPrim:  # noqa: N802
        self.get_prim_at_path_calls.append(str(path))
        return self._root


class _RecordingUsdModules:
    """sys.modules doubles for ``omni.usd`` / ``pxr`` that RECORD every
    ``UsdPhysics.DriveAPI.Apply`` and ``CreateMaxForceAttr`` call.

    #33: omni.physx keeps a USD change listener on the stage; applying a
    DriveAPI (or authoring physics:* on an existing one) makes it re-create
    that joint's drive from the stage, discarding the runtime tensor-view
    gains Isaac Lab wrote from ImplicitActuatorCfg. Measured on bench round
    ahi: drive_joint's PhysX stiffness/damping flipped 200/20 -> 35809.86/0
    (the asset's 625 deg-unit stiffness, 0 damping) at the first
    _set_gripper_effort_limit write and stayed there. So the backend must
    never touch the USD drive at runtime; these doubles let a unit test
    assert that with no Isaac Sim in the process.
    """

    def __init__(self) -> None:
        self.apply_calls: list[tuple[str, str]] = []
        self.max_force_calls: list[tuple[str, float]] = []
        self.drive_prim = _FakeUsdPrim("drive_joint")
        self.root_prim = _FakeUsdPrim("tinker_full", (self.drive_prim,))
        self.stage = _FakeUsdStage(self.root_prim)

        recorder = self

        class _FakeDrive:
            def __init__(self, prim: _FakeUsdPrim, instance: str) -> None:
                self._prim = prim
                self._instance = instance

            def CreateMaxForceAttr(self, value: float) -> object:  # noqa: N802
                recorder.max_force_calls.append((self._instance, float(value)))
                return SimpleNamespace(Set=lambda _value: None)

        class _FakeDriveAPI:
            @staticmethod
            def Apply(prim: _FakeUsdPrim, instance: str) -> _FakeDrive:  # noqa: N802
                recorder.apply_calls.append((prim.GetName(), str(instance)))
                return _FakeDrive(prim, instance)

        def _prim_range(prim: _FakeUsdPrim) -> tuple[_FakeUsdPrim, ...]:
            return (prim,) + tuple(prim.children)

        self.modules = {
            "omni": SimpleNamespace(
                usd=SimpleNamespace(
                    get_context=lambda: SimpleNamespace(get_stage=lambda: self.stage)
                )
            ),
            "omni.usd": SimpleNamespace(
                get_context=lambda: SimpleNamespace(get_stage=lambda: self.stage)
            ),
            "pxr": SimpleNamespace(
                Usd=SimpleNamespace(PrimRange=_prim_range),
                UsdPhysics=SimpleNamespace(DriveAPI=_FakeDriveAPI),
            ),
        }


class _FakeRobot:
    device = "cpu"
    num_base_dofs = 2

    def __init__(self) -> None:
        self.is_initialized = True
        self.root_view = _FakeGripperRootView(2, [12.0, 30.0])
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
        self.root_pose_calls: list[torch.Tensor] = []
        self.root_velocity_calls: list[torch.Tensor] = []

    def write_joint_effort_limit_to_sim_index(self, **kwargs: object) -> None:
        self.limit_calls.append(kwargs)

    def write_root_pose_to_sim_index(self, **kwargs: object) -> None:
        self.root_pose_calls.append(kwargs["root_pose"].clone())

    def write_root_velocity_to_sim_index(self, **kwargs: object) -> None:
        self.root_velocity_calls.append(kwargs["root_velocity"].clone())

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
    # Equal to the ceiling above so gripper_effort_limit_nm degenerates to the
    # pre-#20 min(requested, ceiling) behaviour these existing fixtures were
    # written against; #20-specific tests override this to exercise the real
    # proportional map (ceiling != full scale).
    backend._gripper_effort_full_scale_n = 12.0
    backend._gripper_effort_limit = 12.0
    backend._expected_objects = {}
    backend._contact_pairs_by_key = {}
    # Mirrors __init__'s TINKER_SIM_CONTACT_TRACE_BODIES parse so a test can
    # exercise the env-gated path via patch.dict(os.environ, ...) before
    # calling _backend(), while every other test (env unset) gets the same
    # empty-frozenset default __init__ would produce.
    backend._contact_trace_bodies = frozenset(
        item.strip()
        for item in os.environ.get("TINKER_SIM_CONTACT_TRACE_BODIES", "").split(",")
        if item.strip()
    )
    backend._contact_trace_pairs_by_key = {}
    # Mirrors __init__'s TINKER_SIM_CONTACT_EXTRA_BODIES parse so a test can
    # exercise the env-gated path via patch.dict(os.environ, ...) before
    # calling _backend(), while every other test (env unset) gets the same
    # empty-tuple default __init__ would produce.
    backend._contact_extra_bodies = tuple(
        item.strip()
        for item in os.environ.get("TINKER_SIM_CONTACT_EXTRA_BODIES", "").split(",")
        if item.strip()
    )
    backend._contact_report_first_event_logged = False
    backend._parity_tcp_bodies_missing_logged = False
    backend._parity_gripper_torque_unresolved_logged = False
    backend._parity_gripper_torque_error_logged = False
    backend._parity_gripper_targets_unresolved_logged = False
    backend._parity_gripper_targets_error_logged = False
    backend._articulation_sleep_ids = None
    backend._last_target_write = False
    backend._robot_view_identity = id(backend._robot.root_view)
    backend._clock_step_origin = 0
    backend._clock_elapsed_steps = 0
    backend._clock_epoch_s = 0.0
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
    backend._spawn_x = 0.0
    backend._spawn_y = 0.0
    backend._spawn_yaw = 0.0
    backend._spawn_pose_checked = False
    backend._spawn_pose_check_settle_from = 0.0
    backend._base_hold_after_sim_s = 2.0
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


def _init_state_rot_source(backend_source: str) -> str:
    """Return the source text of the ``rot=`` keyword the backend passes to
    ``ArticulationCfg.InitialStateCfg(...)``.

    Task #30: the vendored Isaac Lab ``InitialStateCfg.rot`` contract is
    scalar-last (x, y, z, w). This extracts exactly the expression the
    backend builds for that keyword (without importing Isaac) so a test can
    eval it and assert it matches ``spawn_root_rot_xyzw`` byte for byte,
    instead of duplicating the construction by hand and risking the same
    order mistake in the test itself.
    """
    tree = ast.parse(backend_source)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Attribute) and func.attr == "InitialStateCfg"):
            continue
        for keyword in node.keywords:
            if keyword.arg == "rot":
                return ast.get_source_segment(backend_source, keyword.value) or ""
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

    def test_gripper_effort_limit_nm_mapping_monotone_and_capped(self) -> None:
        """#20: GripperCommand.max_effort (N) -> drive_joint ceiling (N*m).

        Real commanded values from the manipulation stack: grasp close and
        pre-open both send native_gripper_max_effort = 10 N (full scale, maps
        to the whole ceiling); grasp_benchmark's pre-open sends 5 N (half
        scale); the bridge's own reopen sends 50 N (over-range, saturates at
        the ceiling rather than over-shooting it). 0/None/negative all mean
        "no explicit request", which resolves to the ceiling, matching the
        pre-#20 default behaviour (not zero authority).
        """
        ceiling, full_scale = 2.5, 10.0
        self.assertAlmostEqual(
            gripper_effort_limit_nm(10.0, ceiling, full_scale), 2.5
        )
        self.assertAlmostEqual(
            gripper_effort_limit_nm(5.0, ceiling, full_scale), 1.25
        )
        self.assertAlmostEqual(
            gripper_effort_limit_nm(50.0, ceiling, full_scale), 2.5
        )
        self.assertAlmostEqual(
            gripper_effort_limit_nm(0.0, ceiling, full_scale), 2.5
        )
        self.assertAlmostEqual(
            gripper_effort_limit_nm(None, ceiling, full_scale), 2.5
        )
        self.assertAlmostEqual(
            gripper_effort_limit_nm(-5.0, ceiling, full_scale), 2.5
        )
        self.assertAlmostEqual(
            gripper_effort_limit_nm(float("nan"), ceiling, full_scale), 2.5
        )
        # Monotone over the reachable [0, full_scale] range.
        samples = [0.0, 1.0, 2.5, 5.0, 7.5, 10.0]
        mapped = [gripper_effort_limit_nm(value, ceiling, full_scale) for value in samples[1:]]
        self.assertEqual(mapped, sorted(mapped))

    def test_gripper_effort_ceiling_and_full_scale_env_overrides(self) -> None:
        self.assertEqual(
            resolve_gripper_effort_ceiling_nm(None), GRIPPER_EFFORT_CEILING_NM
        )
        self.assertEqual(
            resolve_gripper_effort_ceiling_nm(""), GRIPPER_EFFORT_CEILING_NM
        )
        self.assertEqual(resolve_gripper_effort_ceiling_nm("3.0"), 3.0)
        self.assertEqual(
            resolve_gripper_effort_ceiling_nm("not-a-number"), GRIPPER_EFFORT_CEILING_NM
        )
        self.assertEqual(
            resolve_gripper_effort_ceiling_nm("-1.0"), GRIPPER_EFFORT_CEILING_NM
        )
        self.assertEqual(
            resolve_gripper_effort_full_scale_n(None), GRIPPER_EFFORT_FULL_SCALE_N
        )
        self.assertEqual(resolve_gripper_effort_full_scale_n("25"), 25.0)
        self.assertEqual(
            resolve_gripper_effort_full_scale_n("0"), GRIPPER_EFFORT_FULL_SCALE_N
        )

        with patch.dict(
            os.environ,
            {
                "TINKER_SIM_GRIPPER_EFFORT_CEILING_NM": "1.75",
                "TINKER_SIM_GRIPPER_EFFORT_FULL_SCALE_N": "20",
            },
        ):
            backend = IsaacWholeRobotBackend.__new__(IsaacWholeRobotBackend)
            backend._default_gripper_effort_limit = resolve_gripper_effort_ceiling_nm(
                os.environ.get("TINKER_SIM_GRIPPER_EFFORT_CEILING_NM")
            )
            backend._gripper_effort_full_scale_n = resolve_gripper_effort_full_scale_n(
                os.environ.get("TINKER_SIM_GRIPPER_EFFORT_FULL_SCALE_N")
            )
        self.assertEqual(backend._default_gripper_effort_limit, 1.75)
        self.assertEqual(backend._gripper_effort_full_scale_n, 20.0)

    def test_gripper_joint_effort_limits_are_hardware_scale_in_config(self) -> None:
        """RED config contract (#20): the 'gripper' (drive_joint) and
        'gripper_mimic' (five followers) ImplicitActuatorCfg groups must both
        set effort_limit_sim to the 2.5 N*m hardware-parity ceiling. Prior to
        this fix, 'gripper' set no effort_limit_sim at all (runtime-only via
        _set_gripper_effort_limit, default 80) and 'gripper_mimic' set 180 --
        both far above the bracket threshold ($TMP/hwcap-result.md,
        hwcap2-result.md) where the PD stalls at the clamp instead of tipping
        the grasped object along the finger arc.

        This must FAIL on main (2c1b51d): 'gripper' has no effort_limit_sim
        keyword at all (AssertionError: gripper ImplicitActuatorCfg has no
        effort_limit_sim -- runtime-only default 80, not hardware-scale) and
        'gripper_mimic' resolves to 180.0, not 2.5.
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
            actuators, "backend ArticulationCfg.actuators must be a dict literal"
        )
        assert actuators is not None

        groups: dict[str, ast.Call] = {}
        for key, value in zip(actuators.keys, actuators.values):
            if isinstance(key, ast.Constant):
                groups[str(key.value)] = value

        for group_name in ("gripper", "gripper_mimic"):
            self.assertIn(
                group_name, groups, f"backend must define a '{group_name}' actuator group"
            )
            call = groups[group_name]
            self.assertIsInstance(call, ast.Call)
            effort_limit_sim = None
            effort_limit = None
            for kw in call.keywords:
                if kw.arg == "effort_limit_sim":
                    effort_limit_sim = kw.value
                if kw.arg == "effort_limit":
                    effort_limit = kw.value
            self.assertIsNotNone(
                effort_limit_sim,
                f"'{group_name}' ImplicitActuatorCfg has no effort_limit_sim -- "
                "must be 2.5 Nm hardware-parity ceiling",
            )
            value = ast.literal_eval(effort_limit_sim)
            self.assertEqual(
                float(value),
                2.5,
                f"'{group_name}' effort_limit_sim must equal the 2.5 Nm hardware "
                f"torque ceiling (#20 bracket), got {value}",
            )
            if effort_limit is not None:
                # If effort_limit is also set (not currently the case), it must
                # agree with effort_limit_sim rather than silently diverge.
                self.assertEqual(float(ast.literal_eval(effort_limit)), 2.5)

    def test_set_gripper_effort_limit_maps_commanded_effort_proportionally(self) -> None:
        """The runtime path (production ceiling/full-scale, not the 1:1 test
        fixture default) must apply gripper_effort_limit_nm, not pass the
        commanded N through as N*m."""
        backend = _backend()
        backend._default_gripper_effort_limit = 2.5
        backend._gripper_effort_full_scale_n = 10.0

        backend._set_gripper_effort_limit(10.0)
        self.assertAlmostEqual(backend.gripper_effort_limit, 2.5)

        backend._set_gripper_effort_limit(5.0)
        self.assertAlmostEqual(backend.gripper_effort_limit, 1.25)

        backend._set_gripper_effort_limit(50.0)
        self.assertAlmostEqual(backend.gripper_effort_limit, 2.5)

        backend._set_gripper_effort_limit(0.0)
        self.assertAlmostEqual(backend.gripper_effort_limit, 2.5)

    def test_set_gripper_effort_limit_writes_direct_physx_max_force(self) -> None:
        """#20: write_joint_effort_limit_to_sim_index alone was proven (probe
        cap5-analysis) not to guarantee the cap reaches the PhysX solver for
        these mimic-coupled joints. _set_gripper_effort_limit must also push
        the mapped limit straight onto the PhysX tensor view
        (root_view.set_dof_max_forces) so the runtime ceiling actually binds,
        and the readback (root_view.get_dof_max_forces) must reflect it.
        """
        backend = _backend()
        backend._default_gripper_effort_limit = 2.5
        backend._gripper_effort_full_scale_n = 10.0
        root_view = backend._robot.root_view

        # #20 review: must pass in isolation, not only because an earlier
        # test in the same process happened to initialize Warp first. In a
        # real boot Warp is initialized well before any gripper command is
        # processed (the fused-actuator path touches it every physics
        # step); pre-warm it here so this test does not depend on suite
        # ordering, and so Warp's one-time init banner (printed straight to
        # stdout, not through this module's JSON-line protocol) cannot leak
        # into the captured output parsed below.
        import warp as _wp

        _wp.init()

        with contextlib.redirect_stdout(io.StringIO()) as captured:
            backend._set_gripper_effort_limit(5.0)

        self.assertEqual(len(root_view.set_dof_max_forces_calls), 1)
        drive_index = backend._joint_index["drive_joint"]
        written = root_view.set_dof_max_forces_calls[0]["forces"]
        self.assertAlmostEqual(float(written[0][drive_index]), 1.25)
        self.assertAlmostEqual(
            float(root_view.get_dof_max_forces()[0][drive_index]), 1.25
        )

        events = [
            json.loads(line)
            for line in captured.getvalue().splitlines()
            if line.strip()
        ]
        effort_events = [event for event in events if event.get("event") == "gripper_effort_limit"]
        self.assertEqual(len(effort_events), 1)
        self.assertAlmostEqual(effort_events[0]["commanded_n"], 5.0)
        self.assertAlmostEqual(effort_events[0]["limit_nm"], 1.25)
        self.assertAlmostEqual(effort_events[0]["physx_max_force"][0], 1.25)
        self.assertTrue(effort_events[0]["physx_write_ok"])
        self.assertTrue(backend._gripper_effort_limit_written)

    def test_gripper_effort_limit_never_authors_usd_drive(self) -> None:
        """#33: _set_gripper_effort_limit must NEVER author drive_joint's USD
        DriveAPI.

        Measured, bench round ahi (2026-09-08, per-tick PhysX readback):
        drive_joint's PhysX stiffness/damping were the configured 200/20 until
        the stack's first GripperCommand; at that exact sample -- the first
        _set_gripper_effort_limit write (max_force 2.5 -> 1.25) -- they became
        35809.86 / 0.0 and stayed there for the rest of the run. 35809.86 is
        the asset's own authored drive stiffness (robot.usd authors
        PhysicsDriveAPI:angular stiffness 625.0 in USD degree units on
        /tinker_full/joints/drive_joint; 625 * 180/pi = 35809.86 PhysX radian
        units) with damping 0.0. The follower joints, which carry no DriveAPI
        in the asset, kept 1500/55.

        Mechanism: applying UsdPhysics.DriveAPI on the live prim and authoring
        physics:maxForce makes omni.physx's USD change listener re-create the
        drive from the stage, discarding the runtime tensor-view gains Isaac
        Lab wrote from ImplicitActuatorCfg("gripper", stiffness=200,
        damping=20). A headless one-variable control confirmed it: with the
        USD authoring replaced by a no-op the gains held at 200/20 for 726
        rows while get_dof_max_forces still read the new cap, so the direct
        tensor-view write alone is sufficient. Isaac Lab's
        data.joint_stiffness/joint_damping read 200/20 in BOTH legs (the Lab
        buffers are not re-synced), so nothing reading the Lab API could see
        the defect -- hence this test asserts on the USD surface directly.
        """
        backend = _backend()
        backend._default_gripper_effort_limit = 2.5
        backend._gripper_effort_full_scale_n = 10.0
        root_view = backend._robot.root_view

        # Pre-warm Warp so its one-time init banner cannot land in the
        # captured window -- see the matching note on
        # test_set_gripper_effort_limit_writes_direct_physx_max_force.
        import warp as _wp

        _wp.init()

        usd = _RecordingUsdModules()
        # omni.usd is not importable in this venv, so the pre-#33 authoring
        # helper failed closed and never ran under unit test. Inject module
        # doubles so a USD write WOULD be observable, then prove none happens.
        with patch.dict(sys.modules, usd.modules):
            with contextlib.redirect_stdout(io.StringIO()):
                backend._set_gripper_effort_limit(5.0)

        self.assertEqual(
            usd.apply_calls,
            [],
            "_set_gripper_effort_limit must not apply UsdPhysics.DriveAPI at "
            "runtime: omni.physx re-syncs the drive from the stage and reverts "
            "drive_joint's PhysX gains to the asset's 35809.86/0 (bench ahi)",
        )
        self.assertEqual(
            usd.max_force_calls,
            [],
            "_set_gripper_effort_limit must not author physics:maxForce on the "
            "USD drive; the direct tensor-view write is what binds the cap",
        )

        # ...and the cap still lands on PhysX through the tensor view alone.
        drive_index = backend._joint_index["drive_joint"]
        self.assertEqual(len(root_view.set_dof_max_forces_calls), 1)
        written = root_view.set_dof_max_forces_calls[0]["forces"]
        self.assertAlmostEqual(float(written[0][drive_index]), 1.25)
        self.assertAlmostEqual(
            float(root_view.get_dof_max_forces()[0][drive_index]), 1.25
        )
        self.assertTrue(backend._gripper_effort_limit_written)

    def test_probe_gripper_close_probe_never_authors_usd_drive(self) -> None:
        """#33: validation/gripper_close_probe.py must not author a
        UsdPhysics.DriveAPI on the follower joints either.

        The follower joints (left_finger_joint, right_outer_knuckle_joint,
        etc.) have NO DriveAPI in robot.usd at all -- unlike drive_joint,
        which the fix above stopped re-authoring. Applying one at runtime
        would create a drive whose USD stiffness/damping default to 0/0, and
        omni.physx's USD change listener would re-sync the runtime gains from
        that stage edit -- the same mechanism (measured on drive_joint, bench
        round ahi) that reverted its gains to 35809.86/0, except here it
        would zero the followers outright.

        gripper_close_probe.py imports isaacsim at module load (see
        tests/test_gripper_close_probe_arm_joints.py, which cannot import it
        either), so this is a source-level check rather than an exercised
        call path: the probe's source must not contain the call sites that
        perform the authoring. The parenthesis is deliberate -- it matches
        only an actual call (``UsdPhysics.DriveAPI.Apply(...)`` /
        ``drive.CreateMaxForceAttr(...)``), not the bare API names that
        legitimately appear in comments/docstrings describing why the probe
        does NOT do this anymore.
        """
        probe_source = (ROOT / "validation" / "gripper_close_probe.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn(
            "DriveAPI.Apply(",
            probe_source,
            "gripper_close_probe.py must not apply a UsdPhysics.DriveAPI on "
            "any joint -- the follower joints have no DriveAPI in the asset, "
            "so authoring one and letting omni.physx re-sync from the stage "
            "zeros their gains (#33 mechanism)",
        )
        self.assertNotIn(
            "CreateMaxForceAttr(",
            probe_source,
            "gripper_close_probe.py must not author physics:maxForce on a "
            "USD drive; the direct PhysX tensor-view write "
            "(_write_physx_max_forces_direct) is what binds the cap",
        )
        self.assertNotIn(
            "_author_usd_max_force",
            probe_source,
            "the USD-authoring helper must be removed entirely, not just "
            "made unreachable",
        )

    def test_set_gripper_effort_limit_physx_write_failure_does_not_latch_written(
        self,
    ) -> None:
        """#20 review finding 1: if the direct PhysX write
        (root_view.set_dof_max_forces) raises -- e.g. Warp's runtime not yet
        initialized in-process -- the failure must be logged (not silently
        swallowed) and _gripper_effort_limit_written must stay False so a
        later identical-effort command is NOT dedup-skipped by the
        _gripper_effort_limit_written guard at the top of
        _set_gripper_effort_limit, but retries the write instead."""
        backend = _backend()
        backend._default_gripper_effort_limit = 2.5
        backend._gripper_effort_full_scale_n = 10.0
        root_view = backend._robot.root_view

        def _raise(forces: object, indices: object = None) -> None:
            raise RuntimeError("boom: physx view not ready")

        root_view.set_dof_max_forces = _raise

        # Pre-warm Warp so its one-time init banner (real stdout text, not a
        # JSON line) cannot land inside the captured window below -- see the
        # matching note on test_set_gripper_effort_limit_writes_direct_physx_max_force.
        import warp as _wp

        _wp.init()

        with contextlib.redirect_stdout(io.StringIO()) as captured:
            backend._set_gripper_effort_limit(5.0)

        self.assertFalse(getattr(backend, "_gripper_effort_limit_written", False))

        events = [
            json.loads(line) for line in captured.getvalue().splitlines() if line.strip()
        ]
        error_events = [
            event
            for event in events
            if event.get("event") == "gripper_physx_max_force_write_error"
        ]
        self.assertEqual(len(error_events), 1)
        self.assertEqual(error_events[0]["level"], "warning")
        self.assertIn("boom", error_events[0]["error"])

        effort_events = [
            event for event in events if event.get("event") == "gripper_effort_limit"
        ]
        self.assertEqual(len(effort_events), 1)
        self.assertFalse(effort_events[0]["physx_write_ok"])

        # Retry: restore a working setter and re-issue the SAME commanded
        # effort. The dedup guard must not skip it (the write was never
        # marked as landed), so the direct PhysX write actually fires now.
        root_view.set_dof_max_forces = _FakeGripperRootView.set_dof_max_forces.__get__(
            root_view
        )
        with contextlib.redirect_stdout(io.StringIO()):
            backend._set_gripper_effort_limit(5.0)
        self.assertEqual(len(root_view.set_dof_max_forces_calls), 1)
        self.assertTrue(backend._gripper_effort_limit_written)

    def test_set_gripper_effort_limit_strict_physx_writes_reraises(self) -> None:
        """TINKER_SIM_STRICT_PHYSX_WRITES=1 must surface the PhysX write
        failure to the caller instead of swallowing it."""
        backend = _backend()
        backend._default_gripper_effort_limit = 2.5
        backend._gripper_effort_full_scale_n = 10.0
        root_view = backend._robot.root_view

        def _raise(forces: object, indices: object = None) -> None:
            raise RuntimeError("boom: strict mode")

        root_view.set_dof_max_forces = _raise

        with patch.dict(os.environ, {"TINKER_SIM_STRICT_PHYSX_WRITES": "1"}):
            with contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(RuntimeError):
                    backend._set_gripper_effort_limit(5.0)
        self.assertFalse(getattr(backend, "_gripper_effort_limit_written", False))

    def test_gripper_low_effort_pre_open_maps_above_zero(self) -> None:
        """#20 coordinator correction: pre-open commands (5 N from
        grasp_benchmark, 10 N native default elsewhere) must map to a
        strictly positive joint ceiling -- enough authority to open the
        gripper in free air, unlike a hypothetical zero-effort mapping. Live
        free-air-open timing is validated by the bench round (staged
        acceptance: $TMP/effortcap_chain.sh); this is the unit-level floor
        the coordinator asked for as a fallback since this worktree cannot
        launch the sim.
        """
        ceiling, full_scale = GRIPPER_EFFORT_CEILING_NM, GRIPPER_EFFORT_FULL_SCALE_N
        for pre_open_n in (5.0, 10.0):
            with self.subTest(pre_open_n=pre_open_n):
                mapped = gripper_effort_limit_nm(pre_open_n, ceiling, full_scale)
                self.assertGreater(mapped, 0.0)

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

    def test_gripper_command_target_logs_old_new_and_applied_on_change(self) -> None:
        """#33 observability: _apply_joint_command must announce every real
        change to _drive_command_target, alongside the ramp target currently
        applied to _position_targets (the pre-ramp value the facade would
        see this instant, not the commanded target).
        """
        backend = _backend()
        backend._drive_joint_index = 0
        backend._position_targets = torch.tensor([[0.05, -1.0]], dtype=torch.float32)

        with contextlib.redirect_stdout(io.StringIO()) as captured:
            backend._apply_joint_command(
                JointCommand(("drive_joint",), positions=(0.83,), efforts=(10.0,))
            )
            backend._apply_joint_command(
                JointCommand(("drive_joint",), positions=(0.15,), efforts=(5.0,))
            )

        lines = [
            line
            for line in captured.getvalue().splitlines()
            if line.startswith("gripper_command_target ")
        ]
        self.assertEqual(len(lines), 2)
        self.assertIn("old=None", lines[0])
        self.assertIn("new=0.83", lines[0])
        self.assertIn("effort=10.0", lines[0])
        self.assertIn("applied=0.050000", lines[0])
        self.assertIn("old=0.83", lines[1])
        self.assertIn("new=0.15", lines[1])
        self.assertIn("effort=5.0", lines[1])
        self.assertAlmostEqual(float(backend._drive_command_target), 0.15)

    def test_gripper_command_target_rate_limits_identical_alternations(self) -> None:
        backend = _backend()
        backend._drive_joint_index = 0

        with contextlib.redirect_stdout(io.StringIO()) as captured:
            for _ in range(4):
                backend._apply_joint_command(
                    JointCommand(("drive_joint",), positions=(0.83,))
                )
                backend._apply_joint_command(
                    JointCommand(("drive_joint",), positions=(0.15,))
                )

        lines = [
            line
            for line in captured.getvalue().splitlines()
            if line.startswith("gripper_command_target ")
        ]
        # 8 real alternations requested in the same instant; capped at
        # GRIPPER_COMMAND_TARGET_LOG_MAX_PER_WINDOW lines/s, not dropped
        # entirely.
        self.assertEqual(len(lines), GRIPPER_COMMAND_TARGET_LOG_MAX_PER_WINDOW)
        # An unchanged repeat of the same target is not a "change" at all
        # and must not consume any of the rate-limit budget.
        with contextlib.redirect_stdout(io.StringIO()) as captured:
            backend._apply_joint_command(
                JointCommand(("drive_joint",), positions=(0.15,))
            )
        self.assertEqual(captured.getvalue().strip(), "")

    def test_safety_stop_transition_logs_engage_then_release(self) -> None:
        """#33 observability: every real ``_safety_stopped`` flip announces
        state/reason/drive-snapshot on engage and applied/measured on
        release, through the caller-provided ``reason``.
        """
        backend = _backend()
        backend._drive_joint_index = 0

        with contextlib.redirect_stdout(io.StringIO()) as captured:
            backend.set_safety_stop(True, reason="sample_true")
            backend.set_safety_stop(False, reason="sample_false")

        lines = [
            line
            for line in captured.getvalue().splitlines()
            if line.startswith("sim_safety_stop ")
        ]
        self.assertEqual(len(lines), 2)
        self.assertIn("state=engaged", lines[0])
        self.assertIn("reason=sample_true", lines[0])
        self.assertIn("wall_age_s=n/a", lines[0])
        # joint_pos[0, 0] (the drive DOF) is 0.25 in _FakeRobot's fixture --
        # the snapshot latched by the engage.
        self.assertIn("drive_snapshot=0.250", lines[0])
        self.assertIn("state=released", lines[1])
        self.assertIn("reason=sample_false", lines[1])
        self.assertIn("drive_applied=0.250", lines[1])
        self.assertIn("drive_measured=0.250", lines[1])

    def test_safety_stop_repeated_sample_logs_nothing(self) -> None:
        backend = _backend()
        backend._drive_joint_index = 0
        backend.set_safety_stop(True, reason="sample_true")

        with contextlib.redirect_stdout(io.StringIO()) as captured:
            backend.set_safety_stop(True, reason="sample_true")

        self.assertEqual(captured.getvalue(), "")

    def test_safety_stop_transition_safe_when_drive_index_unresolved(self) -> None:
        """Boot-time safety: a fresh backend has no ``_drive_joint_index``
        yet and starts with ``_safety_snapshot`` unset (``_backend()``'s
        default). Both transitions must still log, degrading the drive
        fields to "n/a" instead of raising.
        """
        backend = _backend()
        self.assertIsNone(backend._safety_snapshot)
        self.assertFalse(hasattr(backend, "_drive_joint_index"))

        with contextlib.redirect_stdout(io.StringIO()) as captured:
            backend.set_safety_stop(True)
            backend.set_safety_stop(False)

        lines = [
            line
            for line in captured.getvalue().splitlines()
            if line.startswith("sim_safety_stop ")
        ]
        self.assertEqual(len(lines), 2)
        self.assertIn("reason=unspecified", lines[0])
        self.assertIn("drive_snapshot=n/a", lines[0])
        self.assertIn("drive_applied=n/a", lines[1])
        self.assertIn("drive_measured=n/a", lines[1])

    def test_safety_release_logs_applied_targets_reset_with_ramp_value_and_measured(
        self,
    ) -> None:
        """A release's ``applied_targets_reset`` line reports the value that
        was sitting in ``_position_targets`` before the replace (the
        gripper's frozen-hold/ramp value) as ``drive_before``, and the
        fresh joint_pos-derived hold target -- identical to the measured
        state -- as ``drive_after``.
        """
        backend = _backend()
        backend._drive_joint_index = 0
        backend._safety_stopped = True
        backend._safety_snapshot = backend._robot.data.joint_pos.clone()
        # A value distinct from joint_pos[0, 0] (0.25), standing in for
        # whatever the drive's ramp/hold target was before this release.
        backend._position_targets = torch.tensor([[0.62, -0.4]], dtype=torch.float32)

        with contextlib.redirect_stdout(io.StringIO()) as captured:
            backend.set_safety_stop(False, reason="sample_false")

        lines = [
            line
            for line in captured.getvalue().splitlines()
            if line.startswith("applied_targets_reset ")
        ]
        self.assertEqual(len(lines), 1)
        self.assertIn("source=sample_false", lines[0])
        self.assertIn("drive_before=0.620", lines[0])
        self.assertIn("drive_after=0.250", lines[0])
        self.assertIn("measured=0.250", lines[0])

    def test_step_safety_reassert_logs_once_per_engage_not_per_tick(self) -> None:
        """step()'s per-tick ``_position_targets.copy_(_safety_snapshot)``
        reassertion runs every physics step while stopped; its
        ``applied_targets_reset`` line must fire once (on the first tick
        after the engage) and stay silent on every following tick.
        """
        backend = _backend()
        backend._drive_joint_index = 0
        backend.set_safety_stop(True)

        with contextlib.redirect_stdout(io.StringIO()) as captured:
            backend.step()
            backend.step()
            backend.step()

        lines = [
            line
            for line in captured.getvalue().splitlines()
            if line.startswith("applied_targets_reset ")
            and "source=step_safety_reassert" in line
        ]
        self.assertEqual(len(lines), 1)

    def test_refresh_robot_handles_rebind_logs_applied_targets_reset(self) -> None:
        """``_refresh_robot_handles`` re-seeds ``_position_targets`` on every
        root-view identity change with NO safety stop involved -- the same
        re-origination shape the safety-stop lines cover. A genuine rebind
        (``reapply_spawn_yaw=True``, the default) must log
        ``source=refresh_robot_handles:reset_rebind`` with the pre-rebind
        value as ``drive_before``.
        """
        backend = _backend()
        backend._drive_joint_index = 0
        backend._position_targets = torch.tensor([[0.42, -1.0]], dtype=torch.float32)
        backend._robot_view_identity = -1  # force a "new view" rebind
        backend._clock_step_origin = 0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 42)
        backend._object_views = {}

        with contextlib.redirect_stdout(io.StringIO()) as captured:
            self.assertTrue(backend._refresh_robot_handles())

        lines = [
            line
            for line in captured.getvalue().splitlines()
            if line.startswith("applied_targets_reset ")
            and "source=refresh_robot_handles:reset_rebind" in line
        ]
        self.assertEqual(len(lines), 1)
        self.assertIn("drive_before=0.420", lines[0])
        # The reseed is joint_pos.clone(); joint_pos[0, 0] is 0.25 in
        # _FakeRobot's fixture.
        self.assertIn("drive_after=0.250", lines[0])
        self.assertIn("measured=0.250", lines[0])

    def test_step_profile_changed_targets_includes_drive_joint_past_old_top_8_cap(
        self,
    ) -> None:
        """#33 follow-up: ``changed_targets`` used to keep only the top 8
        most-changed keys, which silently dropped ``pos:drive_joint`` (a
        low-traffic key) whenever 8+ other joints changed more often in the
        same window -- exactly the blind spot the mid-close stall
        investigation ran into. With 9 other joints changing every round
        and ``drive_joint`` changing once, the old cap would keep the 8
        highest-count "other" keys and drop both ``drive_joint`` (count 1)
        and one tied "other" key; the fix must publish all 10.
        """
        backend = _backend()
        num_joints = 10
        names = [f"joint{i}" for i in range(1, num_joints)]
        names.insert(0, "drive_joint")
        backend._joint_index = {name: index for index, name in enumerate(names)}
        backend._position_targets = torch.zeros((1, num_joints), dtype=torch.float32)
        backend._velocity_targets = torch.zeros((1, num_joints), dtype=torch.float32)
        backend._effort_targets = torch.zeros((1, num_joints), dtype=torch.float32)
        backend.step_profile = {
            "enabled": True,
            "target_writes": 0,
            "n": 1,
            "targets": 0.0,
            "write_data": 0.0,
            "physx": 0.0,
            "robot_update": 0.0,
            "object_views": 0.0,
        }
        backend._profile_changed_targets()  # seed _profile_last_pushed

        for round_index in range(9):
            backend._position_targets = backend._position_targets.clone()
            for other in range(1, num_joints):
                backend._position_targets[0, other] = float(round_index + 1)
            backend._profile_changed_targets()
        backend._position_targets = backend._position_targets.clone()
        backend._position_targets[0, 0] = 1.0  # drive_joint, once
        backend._profile_changed_targets()

        snapshot = backend.step_profile_snapshot()
        changed = snapshot["changed_targets"]
        self.assertEqual(len(changed), num_joints)
        self.assertIn("pos:drive_joint", changed)
        self.assertEqual(changed["pos:drive_joint"], 1)

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

            def set_safety_stop(self, active: bool, reason: str | None = None) -> None:
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

            def set_safety_stop(self, active: bool, reason: str | None = None) -> None:
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

            def set_safety_stop(self, active: bool, reason: str | None = None) -> None:
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

            def set_safety_stop(self, active: bool, reason: str | None = None) -> None:
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

            def set_safety_stop(self, active: bool, reason: str | None = None) -> None:
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

            def set_safety_stop(self, active: bool, reason: str | None = None) -> None:
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

    def test_parity_tcp_frame_publishes_world_and_base_link_geometry(self) -> None:
        """Task #35 end to end through the backend method: link_tcp world
        pose, its base_link-relative pose, and the pad inner-face/midpoint
        points expressed in base_link -- all read from the same body_pos_w/
        body_quat_w tensors as _robot_truth_state (no separate PhysX query).
        """
        backend = _backend()
        backend._robot.data.body_names = (
            "base", "link_tcp", "left_finger", "right_finger",
        )
        backend._robot.data.body_pos_w = torch.cat(
            [
                backend._robot.data.body_pos_w,
                torch.tensor(
                    [[[5.0, 4.9295, 0.5], [5.0, 5.0705, 0.5]]], dtype=torch.float32
                ),
            ],
            dim=1,
        )
        backend._robot.data.body_quat_w = torch.cat(
            [
                backend._robot.data.body_quat_w,
                torch.tensor(
                    [[[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]], dtype=torch.float32
                ),
            ],
            dim=1,
        )
        # link_tcp itself: overwrite the shared fixture's [1,2,3] entry with a
        # pose offset from a non-trivial root, so tcp_pose_base is a real
        # translation, not a no-op.
        backend._robot.data.body_pos_w[0, 1] = torch.tensor([5.5, 5.0, 1.0])
        backend._robot.data.body_quat_w[0, 1] = torch.tensor([0.0, 0.0, 0.0, 1.0])
        backend._robot.data.root_pos_w = torch.tensor([[5.0, 5.0, 0.5]], dtype=torch.float32)
        backend._robot.data.root_quat_w = torch.tensor([[0.0, 0.0, 0.0, 1.0]], dtype=torch.float32)

        frame = backend.parity_tcp_frame()

        self.assertIsNotNone(frame)
        for actual, expected in zip(frame["tcp_pose_world"]["xyz"], (5.5, 5.0, 1.0)):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(frame["tcp_pose_base"]["xyz"], (0.5, 0.0, 0.5)):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(
            frame["tcp_pose_base"]["quaternion_xyzw"], (0.0, 0.0, 0.0, 1.0)
        ):
            self.assertAlmostEqual(actual, expected, places=6)
        left_point, right_point, midpoint = frame["pad_points_base"]
        for actual, expected in zip(left_point, (0.0, -0.0445, -0.02755)):
            self.assertAlmostEqual(actual, expected, places=5)
        for actual, expected in zip(right_point, (0.0, 0.0445, -0.02755)):
            self.assertAlmostEqual(actual, expected, places=5)
        for actual, expected in zip(midpoint, (0.0, 0.0, -0.02755)):
            self.assertAlmostEqual(actual, expected, places=5)

    def test_parity_tcp_frame_fails_soft_and_logs_once_when_unresolved(self) -> None:
        """link_tcp/left_finger/right_finger absent from body_names (e.g. a
        gripper-less articulation) must not raise -- just log once and let
        the gateway skip publishing that tick."""
        backend = _backend()  # body_names is only ("base", "link_tcp")

        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            first = backend.parity_tcp_frame()
            second = backend.parity_tcp_frame()

        self.assertIsNone(first)
        self.assertIsNone(second)
        lines = [line for line in captured.getvalue().splitlines() if line.strip()]
        self.assertEqual(len(lines), 1, "the unresolved-bodies diagnostic must log once")
        payload = json.loads(lines[0])
        self.assertEqual(payload["event"], "parity_tcp_bodies_unresolved")
        self.assertEqual(
            sorted(payload["missing"]), ["left_finger", "right_finger"]
        )

    def test_parity_gripper_torque_returns_six_joints_from_physx_projected_forces(
        self,
    ) -> None:
        """#33: parity_gripper_torque() must return the six
        PARITY_GRIPPER_JOINTS in order, with position/velocity from the same
        joint_pos/joint_vel tensors joint_state() reads, but torque from
        root_view.get_dof_projected_joint_forces() at the resolved DOF
        indices -- deliberately DIFFERENT here from data.applied_torque (the
        Isaac Lab actuator model's post-clip COMMAND echo) so a test that
        accidentally reads the wrong tensor fails loudly. The joint order is
        scrambled (and includes a non-gripper joint) to prove the indices
        are resolved by name, not assumed contiguous."""
        backend = _backend()
        joint_names = (
            "joint1",
            "right_finger_joint",
            "drive_joint",
            "right_inner_knuckle_joint",
            "left_finger_joint",
            "right_outer_knuckle_joint",
            "left_inner_knuckle_joint",
        )
        backend.joint_names = joint_names
        backend._joint_index = {name: index for index, name in enumerate(joint_names)}
        backend._parity_gripper_joint_indices = tuple(
            backend._joint_index[name] for name in backend.PARITY_GRIPPER_JOINTS
        )
        backend._robot.data.joint_pos = torch.tensor(
            [[10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0]], dtype=torch.float32
        )
        backend._robot.data.joint_vel = torch.tensor(
            [[20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0]], dtype=torch.float32
        )
        physx_row = [100.0, 101.0, 102.0, 103.0, 104.0, 105.0, 106.0]
        backend._robot.root_view = SimpleNamespace(
            get_dof_projected_joint_forces=lambda: [physx_row]
        )

        result = backend.parity_gripper_torque()

        self.assertIsNotNone(result)
        names, positions, velocities, physx_tau = result
        self.assertEqual(names, backend.PARITY_GRIPPER_JOINTS)
        for name, pos, vel, tau in zip(names, positions, velocities, physx_tau):
            index = joint_names.index(name)
            self.assertAlmostEqual(pos, 10.0 + index, places=6)
            self.assertAlmostEqual(vel, 20.0 + index, places=6)
            self.assertAlmostEqual(tau, 100.0 + index, places=6)

    def test_parity_gripper_torque_fails_soft_and_logs_once_when_view_raises(
        self,
    ) -> None:
        """#33: root_view.get_dof_projected_joint_forces() raising (a real
        PhysX API surface -- e.g. queried before the first physics step)
        must not propagate; log once and return None so the gateway skips
        that tick's publish, the same fail-soft contract as
        parity_tcp_frame()."""
        backend = _backend()
        joint_names = (
            "drive_joint",
            "left_finger_joint",
            "left_inner_knuckle_joint",
            "right_outer_knuckle_joint",
            "right_inner_knuckle_joint",
            "right_finger_joint",
        )
        backend.joint_names = joint_names
        backend._joint_index = {name: index for index, name in enumerate(joint_names)}
        backend._parity_gripper_joint_indices = tuple(
            backend._joint_index[name] for name in backend.PARITY_GRIPPER_JOINTS
        )

        def _raise():
            raise RuntimeError("physx view not ready")

        backend._robot.root_view = SimpleNamespace(
            get_dof_projected_joint_forces=_raise
        )

        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            first = backend.parity_gripper_torque()
            second = backend.parity_gripper_torque()

        self.assertIsNone(first)
        self.assertIsNone(second)
        lines = [line for line in captured.getvalue().splitlines() if line.strip()]
        self.assertEqual(len(lines), 1, "the read-error diagnostic must log once")
        payload = json.loads(lines[0])
        self.assertEqual(payload["event"], "parity_gripper_torque_read_error")

    def test_parity_gripper_targets_returns_per_layer_readback_for_three_joints(
        self,
    ) -> None:
        """#33 follow-up (bench round ahh): parity_gripper_targets() must
        return "<joint>/<field>" names for PARITY_GRIPPER_TARGET_JOINTS in
        order, with py_target from _position_targets, lab_target from
        data.joint_pos_target, physx_target/physx_k/physx_d/physx_max_force/
        physx_max_vel each from ONE PhysX view call batched across all
        three joints (not one call per joint), measured from data.joint_pos,
        and lab_applied_effort from data.applied_torque. The joint order is
        scrambled (and includes a non-target joint) to prove the indices
        are resolved by name."""
        backend = _backend()
        joint_names = (
            "joint1",
            "right_outer_knuckle_joint",
            "drive_joint",
            "left_finger_joint",
        )
        backend.joint_names = joint_names
        backend._joint_index = {name: index for index, name in enumerate(joint_names)}
        backend._parity_gripper_target_indices = tuple(
            backend._joint_index[name] for name in backend.PARITY_GRIPPER_TARGET_JOINTS
        )
        backend._position_targets = torch.tensor(
            [[1.0, 2.0, 3.0, 4.0]], dtype=torch.float32
        )
        backend._robot.data.joint_pos = torch.tensor(
            [[10.0, 11.0, 12.0, 13.0]], dtype=torch.float32
        )
        backend._robot.data.joint_pos_target = torch.tensor(
            [[20.0, 21.0, 22.0, 23.0]], dtype=torch.float32
        )
        backend._robot.data.applied_torque = torch.tensor(
            [[30.0, 31.0, 32.0, 33.0]], dtype=torch.float32
        )
        calls: list[str] = []

        def _tracked(name: str, row: list[float]):
            def _getter():
                calls.append(name)
                return [row]

            return _getter

        backend._robot.root_view = SimpleNamespace(
            get_dof_position_targets=_tracked(
                "get_dof_position_targets", [40.0, 41.0, 42.0, 43.0]
            ),
            get_dof_stiffnesses=_tracked(
                "get_dof_stiffnesses", [50.0, 51.0, 52.0, 53.0]
            ),
            get_dof_dampings=_tracked("get_dof_dampings", [60.0, 61.0, 62.0, 63.0]),
            get_dof_max_forces=_tracked(
                "get_dof_max_forces", [70.0, 71.0, 72.0, 73.0]
            ),
            get_dof_max_velocities=_tracked(
                "get_dof_max_velocities", [80.0, 81.0, 82.0, 83.0]
            ),
        )
        backend._last_target_write = True
        backend._articulation_sleep_ids = None  # unresolved -- field omitted

        result = backend.parity_gripper_targets()

        self.assertIsNotNone(result)
        names, values = result
        row = dict(zip(names, values))
        expected_by_field = {
            "py_target": 1.0,
            "lab_target": 20.0,
            "physx_target": 40.0,
            "physx_k": 50.0,
            "physx_d": 60.0,
            "physx_max_force": 70.0,
            "physx_max_vel": 80.0,
            "measured": 10.0,
            "lab_applied_effort": 30.0,
        }
        for joint_name in backend.PARITY_GRIPPER_TARGET_JOINTS:
            index = joint_names.index(joint_name)
            for field, base in expected_by_field.items():
                self.assertAlmostEqual(
                    row[f"{joint_name}/{field}"], base + index, places=6
                )
        self.assertEqual(row["articulation/target_write"], 1.0)
        self.assertNotIn("articulation/is_sleeping", names)
        # Each PhysX getter is called exactly once, batched across all
        # three joints -- not once per joint (which would be 3x these).
        for getter_name in (
            "get_dof_position_targets",
            "get_dof_stiffnesses",
            "get_dof_dampings",
            "get_dof_max_forces",
            "get_dof_max_velocities",
        ):
            self.assertEqual(calls.count(getter_name), 1)

    def test_parity_gripper_targets_returns_none_on_a_double_without_a_view(
        self,
    ) -> None:
        """A test double lacking a PhysX view (root_view/root_physx_view
        both absent/None -- the shape every non-Isaac test double in this
        file has) must return None rather than raising, same fail-soft
        contract as parity_gripper_torque()."""
        backend = _backend()
        joint_names = ("drive_joint", "left_finger_joint", "right_outer_knuckle_joint")
        backend.joint_names = joint_names
        backend._joint_index = {name: index for index, name in enumerate(joint_names)}
        backend._parity_gripper_target_indices = tuple(
            backend._joint_index[name] for name in backend.PARITY_GRIPPER_TARGET_JOINTS
        )
        backend._robot.root_view = None

        self.assertIsNone(backend.parity_gripper_targets())

    def test_resolve_articulation_sleep_ids_requires_the_rigid_body_api(
        self,
    ) -> None:
        """Review round 2: the articulation ROOT BODY is
        ``<prim_path>/base_link``, not the robot's top Xform
        (``cfg.prim_path`` itself, e.g. ``/World/Tinker``, which carries no
        ``PhysicsRigidBodyAPI``) -- ``is_sleeping()`` on a non-rigid-body
        prim id produced a native PhysX C++ error on EVERY tick on a live
        bench while the field silently published 0. Only a prim that
        actually carries ``UsdPhysics.RigidBodyAPI`` at resolve time may be
        resolved; a real in-memory USD stage (this venv's bare ``pxr`` has
        no ``PhysicsSchemaTools``, so that one call is faked) proves both
        directions: resolved with the API applied, omitted without it.

        Round 3 fix: the "omitted" half must ALSO patch
        ``pxr.PhysicsSchemaTools`` (the method imports it unconditionally,
        before the ``HasAPI`` check) -- without that patch this venv's real
        ``pxr`` lacking ``PhysicsSchemaTools`` raises ``ImportError``
        first, and the method's outer ``except Exception: return None``
        masks that as a "None" that looks like the HasAPI gate fired but
        actually never ran (the reviewer proved the old test still passed
        with the gate deleted). Tracking whether the fake ``sdfPathToInt``
        was actually CALLED distinguishes the two: the resolved branch must
        call it once, the omitted branch must never reach it.
        """
        from pxr import Usd, UsdPhysics

        backend = _backend()
        backend._robot.cfg = SimpleNamespace(prim_path="/World/Tinker")
        stage = Usd.Stage.CreateInMemory()
        stage.DefinePrim("/World/Tinker", "Xform")
        body_prim = stage.DefinePrim("/World/Tinker/base_link", "Xform")
        UsdPhysics.RigidBodyAPI.Apply(body_prim)

        fake_context = SimpleNamespace(
            get_stage=lambda: stage, get_stage_id=lambda: 12345
        )
        fake_usd = ModuleType("omni.usd")
        fake_usd.get_context = lambda: fake_context
        resolved_sdf_calls: list[object] = []
        fake_schema_tools = SimpleNamespace(
            sdfPathToInt=lambda path: resolved_sdf_calls.append(path) or 987
        )

        with patch.dict(sys.modules, {"omni.usd": fake_usd}):
            with patch("omni.usd", fake_usd, create=True):
                with patch("pxr.PhysicsSchemaTools", fake_schema_tools, create=True):
                    resolved = backend._resolve_articulation_sleep_ids()

        self.assertEqual(resolved, (12345, 987))
        self.assertEqual(len(resolved_sdf_calls), 1)

        # Without RigidBodyAPI on base_link, the same stage/context (and the
        # SAME PhysicsSchemaTools patch, so an ImportError can't masquerade
        # as the HasAPI gate) must omit (return None) WITHOUT ever calling
        # sdfPathToInt -- proving the gate itself, not an import failure,
        # is what produced the None.
        bare_stage = Usd.Stage.CreateInMemory()
        bare_stage.DefinePrim("/World/Tinker", "Xform")
        bare_stage.DefinePrim("/World/Tinker/base_link", "Xform")  # no API
        fake_context2 = SimpleNamespace(
            get_stage=lambda: bare_stage, get_stage_id=lambda: 1
        )
        fake_usd2 = ModuleType("omni.usd")
        fake_usd2.get_context = lambda: fake_context2
        omitted_sdf_calls: list[object] = []
        fake_schema_tools2 = SimpleNamespace(
            sdfPathToInt=lambda path: omitted_sdf_calls.append(path) or 987
        )

        with patch.dict(sys.modules, {"omni.usd": fake_usd2}):
            with patch("omni.usd", fake_usd2, create=True):
                with patch("pxr.PhysicsSchemaTools", fake_schema_tools2, create=True):
                    omitted = backend._resolve_articulation_sleep_ids()

        self.assertIsNone(omitted)
        self.assertEqual(len(omitted_sdf_calls), 0)

    def test_articulation_is_sleeping_self_check_disables_on_non_bool_return(
        self,
    ) -> None:
        """Review round 2: if ``is_sleeping()``'s first successful return
        value isn't a real ``bool``, the field must be disabled (logged
        once) for the rest of this backend's life rather than silently
        coerced through ``bool(...)`` every tick."""
        backend = _backend()
        backend._articulation_sleep_ids = (1, 2)
        fake_iface = SimpleNamespace(is_sleeping=lambda stage_id, prim_id: 1)  # int
        fake_physx = ModuleType("omni.physx")
        fake_physx.get_physx_simulation_interface = lambda: fake_iface

        with contextlib.redirect_stdout(io.StringIO()) as captured:
            with patch.dict(sys.modules, {"omni.physx": fake_physx}):
                first = backend._articulation_is_sleeping()
                second = backend._articulation_is_sleeping()

        self.assertIsNone(first)
        self.assertIsNone(second)
        self.assertTrue(backend._articulation_is_sleeping_disabled)
        lines = [line for line in captured.getvalue().splitlines() if line.strip()]
        self.assertEqual(len(lines), 1, "the type-mismatch diagnostic must log once")
        payload = json.loads(lines[0])
        self.assertEqual(payload["event"], "articulation_is_sleeping_unexpected_type")
        self.assertEqual(payload["type"], "int")

    def test_step_leaves_target_write_false_when_write_data_to_sim_raises(
        self,
    ) -> None:
        """Review round 2: ``articulation/target_write`` must mean
        "PhysX actually received this tick's targets", not "the write gate
        said yes" -- a ``write_data_to_sim()`` failure that
        ``_maybe_recover_simulation_view`` swallows must leave
        ``_last_target_write`` ``False`` for that tick, and a PRIOR tick's
        ``True`` must not leak into a failing tick either (reset happens at
        the very top of every ``step()`` call). Uses the safety-stop path
        (same minimal-setup shape as
        ``test_safety_stop_reapplies_explicit_hold_without_reviving_old_epoch``
        above) so the write is actually attempted without needing the
        wheel-slew machinery the free-run path requires."""
        backend = _backend()
        backend.set_safety_stop(True)
        backend._last_target_write = True  # stale True from a prior tick

        def _raise(target: object) -> None:
            raise RuntimeError("injected write_data_to_sim failure")

        backend._robot.write_data_to_sim = _raise
        backend._maybe_recover_simulation_view = lambda error: True  # swallow

        backend.step()

        self.assertFalse(backend._last_target_write)

    def test_usd_camera_pose_to_ros_optical_identity_looks_down_world_minus_z(
        self,
    ) -> None:
        """Task #36 worked example: a USD camera at the origin with no local
        rotation looks down -Z with +Y up (native convention). Converting to
        ROS optical must publish a frame whose +Z (forward) points along
        world -Z and whose +Y (down) points along world -Y."""
        position, quaternion_wxyz = usd_camera_pose_to_ros_optical(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
        )
        self.assertEqual(position, (0.0, 0.0, 0.0))
        forward = _rotate_by_quaternion(quaternion_wxyz, (0.0, 0.0, 1.0))
        down = _rotate_by_quaternion(quaternion_wxyz, (0.0, 1.0, 0.0))
        for actual, expected in zip(forward, (0.0, 0.0, -1.0)):
            self.assertAlmostEqual(actual, expected, places=9)
        for actual, expected in zip(down, (0.0, -1.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_usd_camera_pose_to_ros_optical_composes_about_the_cameras_own_axis(
        self,
    ) -> None:
        """A camera yawed 90 deg about world Z: the flip must compose about
        the camera's OWN current local X (Hamilton product on the RIGHT),
        landing optical 'down' at world +X here -- a left-multiply ordering
        bug would instead land it at world -X. (The forward axis alone does
        not discriminate the two orders: it sits on the yaw's own rotation
        axis either way, unchanged by the bug.)"""
        half = math.pi / 4.0
        yaw90_wxyz = (math.cos(half), 0.0, 0.0, math.sin(half))
        _position, quaternion_wxyz = usd_camera_pose_to_ros_optical(
            (0.0, 0.0, 0.0), yaw90_wxyz
        )
        down = _rotate_by_quaternion(quaternion_wxyz, (0.0, 1.0, 0.0))
        forward = _rotate_by_quaternion(quaternion_wxyz, (0.0, 0.0, 1.0))
        for actual, expected in zip(down, (1.0, 0.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=9)
        for actual, expected in zip(forward, (0.0, 0.0, -1.0)):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_camera_optical_pose_world_composes_link_and_mount_pose(self) -> None:
        """Task #36 fix: camera_optical_pose_world's per-tick composition
        (a known articulation-tensor link pose (position + SCALAR-LAST
        quaternion, matching IsaacWholeRobotBackend.body_pose_world) with a
        known static local mount transform (position + SCALAR-FIRST
        quaternion, matching what initialize() would cache)), checked
        against a hand-derived expectation -- position and the optical
        +z/+y axes -- with the link at identity."""
        rig = CameraRig(load_camera_specs(CAMERA_CONTRACT))
        # Mimic exactly what initialize() would have cached for a
        # robot-mounted camera: mount 0.5 m along the link's own local +Z,
        # oriented by the standard REP-103 mount flip (the real value every
        # spec's mount_rotation_wxyz happens to equal today).
        rig._mount_body_names["wrist_camera"] = "xarm_camera_link"
        rig._mount_local_pose["wrist_camera"] = (
            (0.0, 0.0, 0.5),
            OPTICAL_TO_USD_CAMERA_WXYZ,
        )
        link_position = (10.0, 20.0, 0.0)
        link_quaternion_xyzw = (0.0, 0.0, 0.0, 1.0)  # identity, scalar-last

        position, quaternion_wxyz = rig.camera_optical_pose_world(
            "wrist_camera", (link_position, link_quaternion_xyzw)
        )

        for actual, expected in zip(position, (10.0, 20.0, 0.5)):
            self.assertAlmostEqual(actual, expected, places=9)
        forward = _rotate_by_quaternion(quaternion_wxyz, (0.0, 0.0, 1.0))
        down = _rotate_by_quaternion(quaternion_wxyz, (0.0, 1.0, 0.0))
        for actual, expected in zip(forward, (0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected, places=9)
        for actual, expected in zip(down, (0.0, 1.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_camera_optical_pose_world_composes_link_and_mount_pose_with_yaw(
        self,
    ) -> None:
        """Same composition as above, but with the link yawed 90 deg about
        world Z and a mount offset with an X component -- a left- vs
        right-multiply bug in the link/mount composition (unlike the
        optical-flip composition already covered above) would rotate the
        offset the wrong way and land 'down' on the wrong world axis, while
        leaving the boresight (the mount's own +Z, unaffected by a Z yaw)
        unable to discriminate it -- hence checking both.

        Independently hand-derived (rotation matrices, not this module's
        quaternion helpers): a 90 deg yaw about +Z maps local (x, y) ->
        (-y, x), so the mount's local (0.1, 0.0) offset lands at world
        (0.0, 0.1); the mount's own +Z boresight is untouched by a Z yaw
        (still world +Z); its local +Y (down, once flipped into the
        optical frame) is yawed onto world -X.
        """
        rig = CameraRig(load_camera_specs(CAMERA_CONTRACT))
        rig._mount_body_names["wrist_camera"] = "xarm_camera_link"
        rig._mount_local_pose["wrist_camera"] = (
            (0.1, 0.0, 0.5),
            OPTICAL_TO_USD_CAMERA_WXYZ,
        )
        half = math.pi / 4.0
        link_quaternion_xyzw = (0.0, 0.0, math.sin(half), math.cos(half))
        link_position = (10.0, 20.0, 0.0)

        position, quaternion_wxyz = rig.camera_optical_pose_world(
            "wrist_camera", (link_position, link_quaternion_xyzw)
        )

        for actual, expected in zip(position, (10.0, 20.1, 0.5)):
            self.assertAlmostEqual(actual, expected, places=9)
        forward = _rotate_by_quaternion(quaternion_wxyz, (0.0, 0.0, 1.0))
        down = _rotate_by_quaternion(quaternion_wxyz, (0.0, 1.0, 0.0))
        for actual, expected in zip(forward, (0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected, places=9)
        for actual, expected in zip(down, (-1.0, 0.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_camera_optical_pose_world_per_tick_never_touches_pxr(self) -> None:
        """Regression test for the review finding on commit f8277df: the
        first cut of this publisher read a live ``ComputeLocalToWorldTransform``
        on the physics-driven render prim every tick, which goes stale under
        the default fabric-on config (``/physics/updateToUsd=False`` --
        PhysX stops writing rigid-body transforms back into USD). The fix
        composes a tensor-sourced link pose with a mount offset cached once
        at init; this test proves the per-tick call path is pure Python by
        monkeypatching ``UsdGeom.Xformable.ComputeLocalToWorldTransform`` to
        raise and confirming a normal (already-"initialized") call still
        succeeds."""
        from pxr import UsdGeom

        rig = CameraRig(load_camera_specs(CAMERA_CONTRACT))
        rig._mount_body_names["wrist_camera"] = "xarm_camera_link"
        rig._mount_local_pose["wrist_camera"] = (
            (0.0, 0.0, 0.5),
            OPTICAL_TO_USD_CAMERA_WXYZ,
        )

        def _must_not_be_called(self, *args, **kwargs):
            raise AssertionError(
                "camera_optical_pose_world's per-tick path must not call "
                "ComputeLocalToWorldTransform -- that read is fabric-unsafe "
                "on a physics-driven prim; see commit f8277df's review"
            )

        with patch.object(
            UsdGeom.Xformable,
            "ComputeLocalToWorldTransform",
            _must_not_be_called,
        ):
            result = rig.camera_optical_pose_world(
                "wrist_camera",
                ((10.0, 20.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            )

        self.assertIsNotNone(result)

    def test_camera_optical_pose_world_fails_soft_and_logs_once_before_initialize(
        self,
    ) -> None:
        """Task #36: camera_optical_pose_world() must not raise if
        initialize() has not run for the named camera (no Kit/pxr yet) --
        log once and let the gateway skip publishing that tick, the same
        fail-soft contract as parity_tcp_frame()."""
        rig = CameraRig(load_camera_specs(CAMERA_CONTRACT))

        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            first = rig.camera_optical_pose_world("wrist_camera")
            second = rig.camera_optical_pose_world("wrist_camera")

        self.assertIsNone(first)
        self.assertIsNone(second)
        lines = [line for line in captured.getvalue().splitlines() if line.strip()]
        self.assertEqual(len(lines), 1, "the unresolved-camera diagnostic must log once")
        payload = json.loads(lines[0])
        self.assertEqual(payload["event"], "camera_optical_pose_unresolved")
        self.assertEqual(payload["camera"], "wrist_camera")

    def test_gateway_registers_wrist_camera_pose_publishers_gated_by_env(self) -> None:
        source = (ROOT / "simulation/tinker_sim_isaac/ros_gateway.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'PoseStamped, "/sim/parity/wrist_camera_pose", reliable', source
        )
        self.assertIn(
            'PoseStamped, "/sim/parity/wrist_camera_pose_base", reliable', source
        )
        self.assertIn('camera_pose.header.frame_id = "world"', source)
        self.assertIn('camera_pose_base.header.frame_id = "base_link"', source)
        # Same env gate as the rest of #35/#36's parity block -- no separate
        # flag for the camera topics.
        self.assertIn(
            'os.environ.get("TINKER_SIM_PARITY_TCP", "1") != "0"', source
        )

    def test_wrist_camera_pose_publishers_emit_world_and_base_every_tick(self) -> None:
        """Task #36 end to end through the gateway: the wrist camera's world
        pose (as CameraRig.camera_optical_pose_world reports it) and its
        base_link-relative pose (via the same pose_in_frame() the #35 TCP
        parity block reuses), published every publish() tick alongside the
        TCP topics."""
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String
        from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, WrenchStamped

        class _ParityCameraBackend:
            dt = 0.02
            physics_device = "cpu"
            safety_stopped = False
            simulation_time = 0.0
            TRUTH_TOKEN = object()

            def joint_state(self):
                return ((), [], [], [])

            def root_state(self):
                # Root yawed 180 deg about Z, at (1, 2, 0.5) -- a non-trivial
                # frame so the base_link transform actually exercises
                # pose_in_frame's rotation, not just a translation.
                return {
                    "position": (1.0, 2.0, 0.5),
                    "quaternion_wxyz": (0.0, 0.0, 0.0, 1.0),
                    "angular_velocity_world": (0.0, 0.0, 0.0),
                }

            def contact_state(self):
                return {}

            def physics_truth_frame(self, token):
                return {}

            def parity_tcp_frame(self):
                return None  # keep this test focused on the camera block

            def parity_gripper_torque(self):
                return None  # keep this test focused on the camera block

            def body_pose_world(self, name):
                assert name == "xarm_camera_link"
                # Arbitrary tensor-style body pose; the fake camera rig
                # below ignores it and returns a fixed pose -- this test
                # exercises the gateway's plumbing (mount lookup -> backend
                # tensor read -> camera_rig composition -> publish), not
                # CameraRig's own composition math (see
                # test_camera_optical_pose_world_composes_link_and_mount_pose).
                return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)

        class _FakeWristCameraRig:
            def __init__(self, pose) -> None:
                self._pose = pose
                self.calls = 0

            def mount_body_name(self, name):
                assert name == "wrist_camera"
                return "xarm_camera_link"

            def camera_optical_pose_world(self, name, body_pose_world=None):
                self.calls += 1
                assert name == "wrist_camera"
                assert body_pose_world == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
                return self._pose

        class _RecordingPublisher:
            def __init__(self) -> None:
                self.messages: list[object] = []

            def publish(self, message) -> None:
                self.messages.append(message)

        gateway = object.__new__(RosStandardGateway)
        gateway.backend = _ParityCameraBackend()
        gateway._Clock = Clock
        gateway._JointState = JointState
        gateway._Imu = Imu
        gateway._String = String
        gateway._WrenchStamped = WrenchStamped
        gateway._PoseStamped = PoseStamped
        gateway._PolygonStamped = PolygonStamped
        gateway._Point32 = Point32
        gateway.clock_pub = _RecordingPublisher()
        gateway.joint_pub = _RecordingPublisher()
        gateway.imu_pub = _RecordingPublisher()
        gateway.status_pub = _RecordingPublisher()
        gateway.contact_pub = _RecordingPublisher()
        gateway.physics_truth_pub = _RecordingPublisher()
        gateway.cloud_pub = _RecordingPublisher()
        gateway.tcp_pose_pub = _RecordingPublisher()
        gateway.tcp_pose_base_pub = _RecordingPublisher()
        gateway.pad_points_pub = _RecordingPublisher()
        gateway.wrist_camera_pose_pub = _RecordingPublisher()
        gateway.wrist_camera_pose_base_pub = _RecordingPublisher()
        gateway._parity_tcp_enabled = True
        # Camera at (2, 2, 0.5) with an identity optical orientation in
        # world -- one metre along the root's own +X, at the same height.
        camera_rig = _FakeWristCameraRig(((2.0, 2.0, 0.5), (1.0, 0.0, 0.0, 0.0)))
        gateway._camera_rig = camera_rig
        gateway.camera_skipped_frames = 0
        gateway._cloud_publish_enabled = lambda: False
        gateway._last_command_error = None
        gateway._command_stream_lost = False
        gateway._command_epoch = 0
        gateway._last_logical_snapshot_id = -1
        gateway.development_lidar = False
        gateway._publish_profile_enabled = False
        gateway._state_stride = 1_000_000
        gateway._imu_stride = 1_000_000
        gateway._status_stride = 1_000_000
        gateway._tick = 0

        for _ in range(3):
            gateway.publish()

        self.assertEqual(camera_rig.calls, 3)
        self.assertEqual(len(gateway.wrist_camera_pose_pub.messages), 3)
        self.assertEqual(len(gateway.wrist_camera_pose_base_pub.messages), 3)

        world_pose = gateway.wrist_camera_pose_pub.messages[0]
        self.assertEqual(world_pose.header.frame_id, "world")
        for actual, expected in zip(
            (world_pose.pose.position.x, world_pose.pose.position.y, world_pose.pose.position.z),
            (2.0, 2.0, 0.5),
        ):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(
            (
                world_pose.pose.orientation.x,
                world_pose.pose.orientation.y,
                world_pose.pose.orientation.z,
                world_pose.pose.orientation.w,
            ),
            (0.0, 0.0, 0.0, 1.0),
        ):
            self.assertAlmostEqual(actual, expected, places=6)

        # Hand-derived via backend.pose_in_frame: relative = (1, 0, 0) in
        # world, rotated by the 180-deg-about-Z root's inverse -> (-1, 0, 0);
        # the orientation likewise composes to another 180-about-Z, (0, 0,
        # -1, 0) scalar-last.
        base_pose = gateway.wrist_camera_pose_base_pub.messages[0]
        self.assertEqual(base_pose.header.frame_id, "base_link")
        for actual, expected in zip(
            (base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z),
            (-1.0, 0.0, 0.0),
        ):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(
            (
                base_pose.pose.orientation.x,
                base_pose.pose.orientation.y,
                base_pose.pose.orientation.z,
                base_pose.pose.orientation.w,
            ),
            (0.0, 0.0, -1.0, 0.0),
        ):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_wrist_camera_pose_publishers_skip_when_env_disabled_or_camera_unresolved(
        self,
    ) -> None:
        """TINKER_SIM_PARITY_TCP=0 must fully disable the camera topics too
        (no camera_optical_pose_world() calls at all); a resolvable-but-
        currently-unresolved camera rig (fail-soft, returns None) must skip
        publishing without raising."""
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String
        from geometry_msgs.msg import PolygonStamped, PoseStamped, WrenchStamped

        class _NoOpBackend:
            dt = 0.02
            physics_device = "cpu"
            safety_stopped = False
            simulation_time = 0.0
            TRUTH_TOKEN = object()

            def joint_state(self):
                return ((), [], [], [])

            def root_state(self):
                return {
                    "position": (0.0, 0.0, 0.0),
                    "quaternion_wxyz": (1.0, 0.0, 0.0, 0.0),
                    "angular_velocity_world": (0.0, 0.0, 0.0),
                }

            def contact_state(self):
                return {}

            def physics_truth_frame(self, token):
                return {}

            def parity_tcp_frame(self):
                return None

            def parity_gripper_torque(self):
                return None

            def body_pose_world(self, name):
                raise AssertionError(
                    "must not be called: _UnresolvedCameraRig.mount_body_name "
                    "returns None, so there is no body to look up"
                )

        class _UnresolvedCameraRig:
            def __init__(self) -> None:
                self.calls = 0

            def mount_body_name(self, name):
                return None  # not yet resolved by initialize()

            def camera_optical_pose_world(self, name, body_pose_world=None):
                self.calls += 1
                assert body_pose_world is None
                return None  # unresolved this tick

        class _RecordingPublisher:
            def __init__(self) -> None:
                self.messages: list[object] = []

            def publish(self, message) -> None:
                self.messages.append(message)

        def _new_gateway(*, parity_tcp_enabled: bool, camera_rig) -> RosStandardGateway:
            gateway = object.__new__(RosStandardGateway)
            gateway.backend = _NoOpBackend()
            gateway._Clock = Clock
            gateway._JointState = JointState
            gateway._Imu = Imu
            gateway._String = String
            gateway._WrenchStamped = WrenchStamped
            gateway._PoseStamped = PoseStamped
            gateway._PolygonStamped = PolygonStamped
            gateway.clock_pub = _RecordingPublisher()
            gateway.joint_pub = _RecordingPublisher()
            gateway.imu_pub = _RecordingPublisher()
            gateway.status_pub = _RecordingPublisher()
            gateway.contact_pub = _RecordingPublisher()
            gateway.physics_truth_pub = _RecordingPublisher()
            gateway.cloud_pub = _RecordingPublisher()
            gateway.tcp_pose_pub = _RecordingPublisher()
            gateway.tcp_pose_base_pub = _RecordingPublisher()
            gateway.pad_points_pub = _RecordingPublisher()
            gateway.wrist_camera_pose_pub = _RecordingPublisher()
            gateway.wrist_camera_pose_base_pub = _RecordingPublisher()
            gateway._parity_tcp_enabled = parity_tcp_enabled
            gateway._camera_rig = camera_rig
            gateway.camera_skipped_frames = 0
            gateway._cloud_publish_enabled = lambda: False
            gateway._last_command_error = None
            gateway._command_stream_lost = False
            gateway._command_epoch = 0
            gateway._last_logical_snapshot_id = -1
            gateway.development_lidar = False
            gateway._publish_profile_enabled = False
            gateway._state_stride = 1_000_000
            gateway._imu_stride = 1_000_000
            gateway._status_stride = 1_000_000
            gateway._tick = 0
            return gateway

        disabled_rig = _UnresolvedCameraRig()
        disabled_gateway = _new_gateway(parity_tcp_enabled=False, camera_rig=disabled_rig)
        disabled_gateway.publish()
        self.assertEqual(len(disabled_gateway.wrist_camera_pose_pub.messages), 0)
        self.assertEqual(len(disabled_gateway.wrist_camera_pose_base_pub.messages), 0)
        self.assertEqual(disabled_rig.calls, 0)

        unresolved_rig = _UnresolvedCameraRig()
        unresolved_gateway = _new_gateway(parity_tcp_enabled=True, camera_rig=unresolved_rig)
        unresolved_gateway.publish()
        self.assertEqual(len(unresolved_gateway.wrist_camera_pose_pub.messages), 0)
        self.assertEqual(len(unresolved_gateway.wrist_camera_pose_base_pub.messages), 0)
        self.assertEqual(unresolved_rig.calls, 1)

        no_camera_gateway = _new_gateway(parity_tcp_enabled=True, camera_rig=None)
        no_camera_gateway.publish()  # must not raise with no camera rig at all
        self.assertEqual(len(no_camera_gateway.wrist_camera_pose_pub.messages), 0)

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

    # -- Task #21: /clock boot-epoch anchoring (TINKER_SIM_CLOCK_EPOCH) -----

    def test_resolve_clock_epoch_defaults_to_wall_clock(self) -> None:
        with patch("time.time", return_value=1_798_000_000.0):
            self.assertEqual(resolve_clock_epoch(None), 1_798_000_000.0)
            self.assertEqual(resolve_clock_epoch(""), 1_798_000_000.0)
            self.assertEqual(resolve_clock_epoch("wall"), 1_798_000_000.0)
            self.assertEqual(resolve_clock_epoch("WALL"), 1_798_000_000.0)

    def test_resolve_clock_epoch_zero_is_legacy_zero_based(self) -> None:
        self.assertEqual(resolve_clock_epoch("0"), 0.0)

    def test_resolve_clock_epoch_numeric_pins_value(self) -> None:
        self.assertEqual(resolve_clock_epoch("12345.5"), 12345.5)

    def test_resolve_clock_epoch_rejects_non_numeric(self) -> None:
        with self.assertRaises(ValueError):
            resolve_clock_epoch("not-a-number")

    def test_resolve_clock_epoch_rejects_non_finite(self) -> None:
        with self.assertRaises(ValueError):
            resolve_clock_epoch("nan")

    def test_resolve_clock_epoch_rejects_negative(self) -> None:
        # Fix round 1 (Finding 3): a negative epoch would let ros_clock_time
        # (simulation_time + epoch) legitimately read exactly 0, or go
        # negative, while physics is genuinely advancing -- indistinguishable
        # from evaluate_clock_domain's "no sample yet" zero-check, and
        # producing an invalid (negative-nanosecond) builtin_interfaces/Time
        # in ros_gateway.py's _stamp(). Reject at the source instead.
        with self.assertRaises(ValueError):
            resolve_clock_epoch("-30")

    def test_backend_clock_epoch_env_wiring_unset_uses_wall_clock(self) -> None:
        # Exercises __init__'s actual entry point (resolve_backend_clock_epoch,
        # which reads the real "TINKER_SIM_CLOCK_EPOCH" env-var name) rather
        # than only the pure resolve_clock_epoch(value) helper -- full backend
        # construction needs Isaac Sim/PhysX/torch and can't run in this
        # suite, so this is the closest reachable proof the __init__ wiring
        # (not just the parser) uses the right env-var name and default.
        env = dict(os.environ)
        env.pop("TINKER_SIM_CLOCK_EPOCH", None)
        with patch.dict(os.environ, env, clear=True), patch(
            "time.time", return_value=1_798_000_000.0
        ):
            self.assertEqual(resolve_backend_clock_epoch(), 1_798_000_000.0)

    def test_backend_clock_epoch_env_wiring_numeric_pins_value(self) -> None:
        with patch.dict(os.environ, {"TINKER_SIM_CLOCK_EPOCH": "12345.5"}):
            self.assertEqual(resolve_backend_clock_epoch(), 12345.5)

    def test_backend_clock_epoch_env_wiring_zero_is_legacy(self) -> None:
        with patch.dict(os.environ, {"TINKER_SIM_CLOCK_EPOCH": "0"}):
            self.assertEqual(resolve_backend_clock_epoch(), 0.0)

    def test_ros_clock_time_adds_epoch_without_changing_simulation_time(self) -> None:
        backend = _backend()
        backend._clock_epoch_s = 1_000.0
        backend.physics_dt = 1.0 / 120.0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 60)

        self.assertAlmostEqual(backend.simulation_time, 0.5)
        self.assertAlmostEqual(backend.ros_clock_time, 1_000.5)
        # simulation_time itself is untouched by the epoch: internal timers
        # (run-duration gating, base-hold, truth "t" fields) that already
        # depend on it starting near zero keep their existing meaning.
        self.assertAlmostEqual(backend.simulation_time, 0.5)

    def test_clock_epoch_zero_reproduces_legacy_zero_based_sequence(self) -> None:
        backend = _backend()
        backend._clock_epoch_s = resolve_clock_epoch("0")
        backend.physics_dt = 1.0 / 120.0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 12)

        self.assertAlmostEqual(backend.ros_clock_time, backend.simulation_time)
        self.assertAlmostEqual(backend.ros_clock_time, 0.1)

    def test_reset_monotonic_clock_holds_with_epoch_in_place(self) -> None:
        # The 767fb89 in-process STOP -> PLAY monotonic-clock fix must keep
        # working once an epoch is anchored on top of it.
        backend = _backend()
        backend._clock_epoch_s = 500.0
        backend.physics_dt = 1.0 / 120.0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 100)
        last_clock_before_reset = backend.ros_clock_time
        self.assertAlmostEqual(last_clock_before_reset, 500.0 + 100 / 120.0)

        # Standard ResetSimulation: the articulation view identity changes and
        # the physics step counter can reset to a small number.
        backend._robot_view_identity = -1
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 3)
        backend._object_views = {"delivery_object": object()}

        self.assertTrue(backend._refresh_robot_handles())
        first_clock_after_reset = backend.ros_clock_time
        self.assertGreaterEqual(first_clock_after_reset, last_clock_before_reset)

    def test_two_backends_back_to_back_publish_nondecreasing_clock(self) -> None:
        # Simulate a full sim-process restart (task #21): two independently
        # constructed backends, each anchoring its published clock to the
        # wall-clock time observed when its own clock origin is established.
        with patch("time.time", return_value=1_000.0):
            backend1 = _backend()
            backend1._clock_epoch_s = resolve_clock_epoch(None)
        backend1.physics_dt = 1.0 / 120.0
        backend1._sim = SimpleNamespace(get_physics_step_count=lambda: 6_000)  # 50s
        last_clock_backend1 = backend1.ros_clock_time
        self.assertAlmostEqual(last_clock_backend1, 1_050.0)

        # Wall-clock time elapses across the restart -- a real Isaac Sim boot
        # takes far longer than any sim time accumulated above.
        with patch("time.time", return_value=1_100.0):
            backend2 = _backend()
            backend2._clock_epoch_s = resolve_clock_epoch(None)
        backend2.physics_dt = 1.0 / 120.0
        backend2._sim = SimpleNamespace(get_physics_step_count=lambda: 0)
        first_clock_backend2 = backend2.ros_clock_time

        self.assertGreaterEqual(first_clock_backend2, last_clock_backend1)

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

    def test_contact_report_drops_unmonitored_pair_without_trace_env(self) -> None:
        # TINKER_SIM_CONTACT_TRACE_BODIES unset: a Bottle<->knuckle pair (neither
        # actor in ARM_CONTACT_BODIES/GRASP_CONTACT_BODIES) stays invisible on
        # both the existing and the new accessor -- unchanged default behavior.
        backend = _backend()
        backend.dt = 0.1
        backend._contact_event_found = "found"
        backend._contact_event_lost = "lost"
        backend._contact_event_persist = "persist"
        backend._contact_path_decoder = {
            1: "/World/Scenario/Bottle",
            2: "/World/Tinker/left_inner_knuckle",
        }.__getitem__
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
            impulse=(0.0, 0.0, 5.0),
            position=(0.1, 0.2, 0.3),
            normal=(0.0, 0.0, 1.0),
        )
        backend._on_contact_report_event([header], [sample])

        self.assertEqual(backend.contact_pairs(), [])
        self.assertEqual(backend.contact_trace_pairs(), [])

    def test_contact_report_trace_env_records_unmonitored_pair_with_point_and_normal(
        self,
    ) -> None:
        # TINKER_SIM_CONTACT_TRACE_BODIES=Bottle: the same Bottle<->knuckle pair
        # is now recorded (still absent from contact_pairs()/contact_state(),
        # which stay scoped to ARM_CONTACT_BODIES/GRASP_CONTACT_BODIES), with
        # its force, point, and normal kept on the new trace accessor.
        with patch.dict(os.environ, {"TINKER_SIM_CONTACT_TRACE_BODIES": "Bottle"}):
            backend = _backend()
        backend.dt = 0.1
        backend._contact_event_found = "found"
        backend._contact_event_lost = "lost"
        backend._contact_event_persist = "persist"
        backend._contact_path_decoder = {
            1: "/World/Scenario/Bottle",
            2: "/World/Tinker/left_inner_knuckle",
        }.__getitem__
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
            impulse=(0.0, 0.0, 5.0),
            position=(0.1, 0.2, 0.3),
            normal=(0.0, 0.0, 1.0),
        )
        backend._on_contact_report_event([header], [sample])

        self.assertEqual(backend.contact_pairs(), [])
        self.assertFalse(backend.contact_state()["left_finger"]["in_contact"])
        traced = backend.contact_trace_pairs()
        self.assertEqual(len(traced), 1)
        self.assertEqual(traced[0]["body_a"], "/World/Scenario/Bottle")
        self.assertEqual(traced[0]["body_b"], "/World/Tinker/left_inner_knuckle")
        self.assertAlmostEqual(float(traced[0]["normal_force"]), 50.0)
        self.assertEqual(traced[0]["point_count"], 1)
        for actual, expected in zip(traced[0]["point"], [0.1, 0.2, 0.3]):
            self.assertAlmostEqual(float(actual), expected)
        for actual, expected in zip(traced[0]["normal"], [0.0, 0.0, 1.0]):
            self.assertAlmostEqual(float(actual), expected)
        self.assertEqual(len(traced[0]["points"]), 1)
        for actual, expected in zip(traced[0]["points"][0], [0.1, 0.2, 0.3]):
            self.assertAlmostEqual(float(actual), expected)
        self.assertEqual(len(traced[0]["normals"]), 1)
        for actual, expected in zip(traced[0]["normals"][0], [0.0, 0.0, 1.0]):
            self.assertAlmostEqual(float(actual), expected)

        # a lost event clears the traced pair the same way it clears the
        # monitored one.
        header.type = "lost"
        header.num_contact_data = 0
        backend._on_contact_report_event([header], [])
        self.assertEqual(backend.contact_trace_pairs(), [])

    def test_contact_extra_bodies_unset_matches_default_monitored_set(self) -> None:
        # TINKER_SIM_CONTACT_EXTRA_BODIES unset: a pair on a body outside
        # ARM_CONTACT_BODIES/GRASP_CONTACT_BODIES (e.g. the gripper base)
        # stays invisible -- byte-identical to the pre-existing monitored set.
        backend = _backend()
        self.assertEqual(backend._contact_extra_bodies, ())
        backend.dt = 0.1
        backend._contact_event_found = "found"
        backend._contact_event_lost = "lost"
        backend._contact_event_persist = "persist"
        backend._contact_path_decoder = {
            1: "/World/Tinker/xarm_gripper_base_link",
            2: "/World/Scenario/delivery_object",
        }.__getitem__
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
            impulse=(0.0, 0.0, 5.0),
            position=(0.1, 0.2, 0.3),
            normal=(0.0, 0.0, 1.0),
        )
        backend._on_contact_report_event([header], [sample])

        self.assertEqual(backend.contact_pairs(), [])

    def test_contact_extra_bodies_env_records_pair_like_grasp_bodies(self) -> None:
        # TINKER_SIM_CONTACT_EXTRA_BODIES=xarm_gripper_base_link,xarm_camera_link
        # extends the monitored set: a push on the gripper base is now
        # recorded and aggregated by name the same way GRASP_CONTACT_BODIES
        # are, without touching left_finger/right_finger semantics.
        with patch.dict(
            os.environ,
            {
                "TINKER_SIM_CONTACT_EXTRA_BODIES": (
                    "xarm_gripper_base_link,xarm_camera_link"
                )
            },
        ):
            backend = _backend()
        self.assertEqual(
            backend._contact_extra_bodies,
            ("xarm_gripper_base_link", "xarm_camera_link"),
        )
        backend.dt = 0.1
        backend._contact_event_found = "found"
        backend._contact_event_lost = "lost"
        backend._contact_event_persist = "persist"
        backend._contact_path_decoder = {
            1: "/World/Tinker/xarm_gripper_base_link",
            2: "/World/Scenario/delivery_object",
        }.__getitem__
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
            impulse=(0.0, 0.0, 5.0),
            position=(0.1, 0.2, 0.3),
            normal=(0.0, 0.0, 1.0),
        )
        backend._on_contact_report_event([header], [sample])

        pairs = backend.contact_pairs()
        self.assertEqual(len(pairs), 1)
        self.assertEqual(pairs[0]["body_a"], "/World/Tinker/xarm_gripper_base_link")
        self.assertEqual(pairs[0]["body_b"], "/World/Scenario/delivery_object")
        state = backend.contact_state()
        self.assertTrue(state["xarm_gripper_base_link"]["in_contact"])
        self.assertAlmostEqual(float(state["xarm_gripper_base_link"]["force"]), 50.0)
        self.assertFalse(state["left_finger"]["in_contact"])
        self.assertFalse(state["right_finger"]["in_contact"])

        header.type = "lost"
        header.num_contact_data = 0
        backend._on_contact_report_event([header], [])
        self.assertEqual(backend.contact_pairs(), [])

    def test_contact_trace_and_extra_bodies_match_full_scenario_prim_path(self) -> None:
        # The bench's truth pairs carry full prim paths -- a spawned entity's
        # rigid body IS its own prim, e.g.
        # "/World/Scenario/bench_sugar_box_100_anygrasp_agt_a1" -- and the
        # bench sets TINKER_SIM_CONTACT_TRACE_BODIES to that entity's bare
        # leaf name (not a full path), same as TINKER_SIM_CONTACT_EXTRA_BODIES
        # is set to bare xarm link names. Both matchers must compare leaves
        # (see _contact_name_matches), exactly as the pre-existing
        # ARM_CONTACT_BODIES/GRASP_CONTACT_BODIES matching already resolves
        # e.g. "left_finger" from "/World/Tinker/left_finger".
        with patch.dict(
            os.environ,
            {
                "TINKER_SIM_CONTACT_TRACE_BODIES": "bench_sugar_box_100_anygrasp_agw_a1",
                "TINKER_SIM_CONTACT_EXTRA_BODIES": "xarm_gripper_base_link",
            },
        ):
            backend = _backend()
        backend.dt = 0.1
        backend._contact_event_found = "found"
        backend._contact_event_lost = "lost"
        backend._contact_event_persist = "persist"
        backend._contact_path_decoder = {
            1: "/World/Tinker/xarm_gripper_base_link",
            2: "/World/Scenario/bench_sugar_box_100_anygrasp_agw_a1",
        }.__getitem__
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
            impulse=(0.0, 0.0, 5.0),
            position=(0.1, 0.2, 0.3),
            normal=(0.0, 0.0, 1.0),
        )
        backend._on_contact_report_event([header], [sample])

        pairs = backend.contact_pairs()
        self.assertEqual(len(pairs), 1)
        self.assertEqual(pairs[0]["body_a"], "/World/Tinker/xarm_gripper_base_link")
        self.assertEqual(
            pairs[0]["body_b"], "/World/Scenario/bench_sugar_box_100_anygrasp_agw_a1"
        )
        traced = backend.contact_trace_pairs()
        self.assertEqual(len(traced), 1)
        self.assertEqual(traced[0]["body_a"], "/World/Tinker/xarm_gripper_base_link")
        self.assertEqual(
            traced[0]["body_b"], "/World/Scenario/bench_sugar_box_100_anygrasp_agw_a1"
        )

    def test_contact_report_first_event_logged_exactly_once(self) -> None:
        # #28 observability: a stale contacts-off boot produced a
        # structurally-silent /sim/truth/contacts for a whole bench round
        # with nothing in the sim log to say so. The first recorded pair
        # must print a one-line marker exactly once per backend life, not
        # once per contact (see _on_contact_report_event).
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
            num_contact_data=1,
        )
        sample = SimpleNamespace(
            impulse=(0.0, 0.0, 0.5),
            position=(0.0, 0.0, 0.0),
            normal=(0.0, 0.0, 1.0),
        )

        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            backend._on_contact_report_event([header], [sample])
            # A second recorded contact (persist event, still above the
            # force threshold) must not re-log the first-event marker.
            header.type = "persist"
            backend._on_contact_report_event([header], [sample])

        lines = [
            json.loads(line)
            for line in captured.getvalue().splitlines()
            if line.strip()
        ]
        first_event_lines = [
            entry for entry in lines if entry.get("event") == "contact_report_first_event"
        ]
        self.assertEqual(len(first_event_lines), 1)
        self.assertEqual(
            first_event_lines[0]["pair"],
            ["/World/Tinker/left_finger", "/World/Scenario/delivery_object"],
        )
        self.assertIn("simulation_time", first_event_lines[0])

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

    def test_truth_state_timestamp_is_anchored_not_elapsed(self) -> None:
        # Fix round 1 (Finding 1): /sim/internal/physics_truth -> truth_evaluator.py
        # -> /sim/truth/* must share the same anchored clock domain as
        # /clock and every other ros_gateway.py-stamped topic, not the
        # unanchored elapsed simulation_time. A non-zero epoch makes the
        # two values clearly distinguishable.
        backend = _backend()
        backend._clock_epoch_s = 1_000.0
        backend.dt = 0.1
        backend.physics_dt = 1.0 / 120.0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 60)  # 0.5s elapsed
        backend._clock_step_origin = 0
        backend.scenario = "qualification-retention"
        backend.task = "qualification-retention"
        backend.physics_device = "cpu"
        backend.seed = 7
        backend._expected_objects = {}
        backend._actual_object_states = lambda: []

        truth = backend.truth_state(backend.TRUTH_TOKEN)

        self.assertAlmostEqual(truth["timestamp"], 1_000.5)
        self.assertNotAlmostEqual(truth["timestamp"], backend.simulation_time)
        self.assertAlmostEqual(truth["timestamp"], backend.ros_clock_time)

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

    def test_spawned_bodies_appear_in_object_truth_view_free(self) -> None:
        # Task #11: harness-spawned /World/Scenario bodies (not scenario-
        # declared, so they never get a tensor view) must still reach object
        # truth, read view-free through _iter_spawned_bodies.
        backend = _backend()
        backend._object_views = {}
        backend._expected_objects = {}
        backend._iter_spawned_bodies = lambda: [
            (
                "/World/Scenario/bench_sugar_box_100_agq_a1",
                (0.5, 0.1, 0.7),
                (0.0, 0.0, 0.0, 1.0),
            )
        ]
        objects = backend._actual_object_states()
        self.assertEqual(len(objects), 1)
        spawned = objects[0]
        self.assertEqual(spawned["id"], "bench_sugar_box_100_agq_a1")
        self.assertEqual(
            spawned["prim_path"], "/World/Scenario/bench_sugar_box_100_agq_a1"
        )
        self.assertEqual(spawned["pose"]["xyz"], [0.5, 0.1, 0.7])
        self.assertEqual(spawned["pose"]["quaternion_xyzw"], [0.0, 0.0, 0.0, 1.0])
        self.assertEqual(spawned["twist"]["linear"], [0.0, 0.0, 0.0])

    def test_spawned_states_skip_scenario_declared_and_append_after(self) -> None:
        # A spawned path that duplicates a scenario-declared object's prim path
        # is not double-listed; spawned bodies append AFTER declared ones so
        # objects[0] stays the declared object (and its viewed pose).
        backend = _backend()
        backend._expected_objects = {
            "delivery_object": {
                "class_name": "dynamic_cube",
                "actual_prim_path": "/World/Scenario/delivery_object",
            }
        }
        backend._object_views = {"delivery_object": _FakeRigidView()}
        backend._iter_spawned_bodies = lambda: [
            ("/World/Scenario/delivery_object", (9.0, 9.0, 9.0), (0.0, 0.0, 0.0, 1.0)),
            ("/World/Scenario/bench_spam_100_agq_a1", (0.4, 0.0, 0.72), (0.0, 0.0, 0.0, 1.0)),
        ]
        objects = backend._actual_object_states()
        self.assertEqual(
            [o["id"] for o in objects],
            ["delivery_object", "bench_spam_100_agq_a1"],
        )
        self.assertEqual(objects[0]["prim_path"], "/World/Scenario/delivery_object")
        self.assertNotEqual(objects[0]["pose"]["xyz"], [9.0, 9.0, 9.0])

    def test_quaternion_xyzw_from_physx_maps_scalar_last(self) -> None:
        from tinker_sim_isaac.backend import IsaacWholeRobotBackend as _BE

        gf_like = SimpleNamespace(
            GetReal=lambda: 0.7071, GetImaginary=lambda: (0.0, 0.7071, 0.0)
        )
        self.assertEqual(
            _BE._quaternion_xyzw_from_physx(gf_like), (0.0, 0.7071, 0.0, 0.7071)
        )
        self.assertEqual(
            _BE._quaternion_xyzw_from_physx((1.0, 2.0, 3.0, 4.0)),
            (1.0, 2.0, 3.0, 4.0),
        )
        self.assertIsNone(_BE._quaternion_xyzw_from_physx(None))
        self.assertIsNone(_BE._quaternion_xyzw_from_physx((1.0, 2.0)))

    def test_pad_points_world_mirrors_toward_centreline_at_rest_orientation(
        self,
    ) -> None:
        """Task #35 pad-inner-face math. Both finger links share one static
        rest orientation -- a 180 deg rotation about local X, xyzw (1,0,0,0)
        -- confirmed live via pxr on the shipped robot USD (see
        docs/developer-log.md 2026-09-06, Task #35): rotating each finger's
        fixed LEFT/RIGHT_FINGER_PAD_LOCAL_OFFSET by that orientation is what
        turns the local mesh offset into "toward the jaw centreline" in
        world space. Fingers at +/-0.0705 m (matching the live pxr readback
        of the finger link origins) with the 26 mm inset must land the inner
        faces at +/-0.0445 m and their midpoint at 0 along the closing axis.
        """
        from tinker_sim_isaac import backend as backend_module

        rest_quaternion = (1.0, 0.0, 0.0, 0.0)
        left_pose = ((0.0, -0.0705, 0.0), rest_quaternion)
        right_pose = ((0.0, 0.0705, 0.0), rest_quaternion)

        left_point, right_point, midpoint = backend_module.pad_points_world(
            left_pose, right_pose
        )

        self.assertAlmostEqual(left_point[1], -0.0445, places=6)
        self.assertAlmostEqual(right_point[1], 0.0445, places=6)
        self.assertAlmostEqual(midpoint[1], 0.0, places=6)
        self.assertAlmostEqual(
            left_point[2], -backend_module.PAD_MID_REACH_M, places=6
        )
        self.assertAlmostEqual(
            right_point[2], -backend_module.PAD_MID_REACH_M, places=6
        )

    def test_pad_points_world_rotates_consistently_under_an_additional_yaw(
        self,
    ) -> None:
        """Composing an extra 90 deg yaw onto the rest orientation must
        rotate both inner-face points (and their midpoint) the same way a
        single rigid transform would -- i.e. the closing-axis separation
        the two inner faces started with (before the yaw) reappears, after
        the yaw, along the axis the yaw rotated the closing axis onto."""
        from tinker_sim_isaac import backend as backend_module

        rest_quaternion = (1.0, 0.0, 0.0, 0.0)
        yaw_90_z = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
        combined = backend_module._quaternion_multiply_xyzw(yaw_90_z, rest_quaternion)

        left_pose = (
            backend_module.rotate_vector_xyzw(yaw_90_z, (0.0, -0.0705, 0.0)),
            combined,
        )
        right_pose = (
            backend_module.rotate_vector_xyzw(yaw_90_z, (0.0, 0.0705, 0.0)),
            combined,
        )

        left_point, right_point, midpoint = backend_module.pad_points_world(
            left_pose, right_pose
        )

        # The un-yawed case put the +/-0.0445 m separation on Y (see the
        # test above); a 90 deg yaw about Z carries that separation onto X,
        # not Y, and leaves Z untouched.
        self.assertAlmostEqual(left_point[0], 0.0445, places=6)
        self.assertAlmostEqual(right_point[0], -0.0445, places=6)
        self.assertAlmostEqual(left_point[1], 0.0, places=6)
        self.assertAlmostEqual(right_point[1], 0.0, places=6)
        self.assertAlmostEqual(midpoint[0], 0.0, places=6)
        self.assertAlmostEqual(
            left_point[2], -backend_module.PAD_MID_REACH_M, places=6
        )

    def test_pose_in_frame_expresses_a_world_pose_relative_to_a_yawed_root(
        self,
    ) -> None:
        """Task #35 base_link transform math, checked against a known root
        pose: a root translated by (1, 2, 0) and yawed 90 deg about Z, and a
        world point one metre along world +X from the root -- expressed in
        the root's own frame that point must be one metre along the root's
        LOCAL -Y (since +X world is -Y in a frame yawed +90 deg about Z)."""
        from tinker_sim_isaac import backend as backend_module

        root_position = (1.0, 2.0, 0.0)
        root_quaternion = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
        world_position = (2.0, 2.0, 0.0)  # root_position + 1 m along world +X
        world_quaternion = root_quaternion  # co-oriented with the root itself

        local_position, local_quaternion = backend_module.pose_in_frame(
            root_position, root_quaternion, world_position, world_quaternion
        )

        self.assertAlmostEqual(local_position[0], 0.0, places=6)
        self.assertAlmostEqual(local_position[1], -1.0, places=6)
        self.assertAlmostEqual(local_position[2], 0.0, places=6)
        # Co-oriented with the frame itself -> the relative orientation is
        # the identity quaternion.
        for actual, expected in zip(local_quaternion, (0.0, 0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected, places=6)

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
        # Unrelated to this test (#22); off so publish() skips the #35 block.
        gateway._parity_tcp_enabled = False
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

    def test_gateway_registers_parity_tcp_publishers_gated_by_env(self) -> None:
        source = (ROOT / "simulation/tinker_sim_isaac/ros_gateway.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('PoseStamped, "/sim/parity/tcp_pose", reliable', source)
        self.assertIn(
            'PoseStamped, "/sim/parity/tcp_pose_base", reliable', source
        )
        self.assertIn(
            'PolygonStamped, "/sim/parity/pad_points", reliable', source
        )
        self.assertIn('tcp_pose.header.frame_id = "world"', source)
        self.assertIn('tcp_pose_base.header.frame_id = "base_link"', source)
        self.assertIn('pad_points.header.frame_id = "base_link"', source)
        self.assertIn(
            'os.environ.get("TINKER_SIM_PARITY_TCP", "1") != "0"', source
        )

    def test_parity_tcp_publishers_emit_world_base_and_pad_points_every_tick(
        self,
    ) -> None:
        """Task #35: the three parity topics publish at the same cadence as
        /sim/internal/physics_truth (unconditional, every tick), independent
        of the status heartbeat stride -- same contract as #22's finger
        contact wrench above."""
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String
        from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, WrenchStamped

        class _ParityTcpBackend:
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
                return {}

            def physics_truth_frame(self, token):
                return {}

            def parity_tcp_frame(self):
                return {
                    "tcp_pose_world": {
                        "xyz": (0.5, 0.0, 0.6),
                        "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0),
                    },
                    "tcp_pose_base": {
                        "xyz": (0.1, 0.0, 0.2),
                        "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0),
                    },
                    "pad_points_base": (
                        (0.1, -0.0445, 0.15),
                        (0.1, 0.0445, 0.15),
                        (0.1, 0.0, 0.15),
                    ),
                }

            def parity_gripper_torque(self):
                return None  # keep this test focused on the TCP/pad block

        class _RecordingPublisher:
            def __init__(self) -> None:
                self.messages: list[object] = []

            def publish(self, message) -> None:
                self.messages.append(message)

        gateway = object.__new__(RosStandardGateway)
        gateway.backend = _ParityTcpBackend()
        gateway._Clock = Clock
        gateway._JointState = JointState
        gateway._Imu = Imu
        gateway._String = String
        gateway._WrenchStamped = WrenchStamped
        gateway._PoseStamped = PoseStamped
        gateway._PolygonStamped = PolygonStamped
        gateway._Point32 = Point32
        gateway.clock_pub = _RecordingPublisher()
        gateway.joint_pub = _RecordingPublisher()
        gateway.imu_pub = _RecordingPublisher()
        gateway.status_pub = _RecordingPublisher()
        gateway.contact_pub = _RecordingPublisher()
        gateway.physics_truth_pub = _RecordingPublisher()
        gateway.cloud_pub = _RecordingPublisher()
        gateway.tcp_pose_pub = _RecordingPublisher()
        gateway.tcp_pose_base_pub = _RecordingPublisher()
        gateway.pad_points_pub = _RecordingPublisher()
        gateway._parity_tcp_enabled = True
        gateway._camera_rig = None
        gateway._cloud_publish_enabled = lambda: False
        gateway._last_command_error = None
        gateway._command_stream_lost = False
        gateway._command_epoch = 0
        gateway._last_logical_snapshot_id = -1
        gateway.development_lidar = False
        gateway._publish_profile_enabled = False
        gateway._state_stride = 1_000_000
        gateway._imu_stride = 1_000_000
        gateway._status_stride = 1_000_000
        gateway._tick = 0

        for _ in range(3):
            gateway.publish()

        self.assertEqual(len(gateway.tcp_pose_pub.messages), 3)
        self.assertEqual(len(gateway.tcp_pose_base_pub.messages), 3)
        self.assertEqual(len(gateway.pad_points_pub.messages), 3)

        tcp_pose = gateway.tcp_pose_pub.messages[0]
        self.assertEqual(tcp_pose.header.frame_id, "world")
        self.assertAlmostEqual(tcp_pose.pose.position.x, 0.5, places=6)
        self.assertAlmostEqual(tcp_pose.pose.orientation.w, 1.0, places=6)

        tcp_pose_base = gateway.tcp_pose_base_pub.messages[0]
        self.assertEqual(tcp_pose_base.header.frame_id, "base_link")
        self.assertAlmostEqual(tcp_pose_base.pose.position.z, 0.2, places=6)

        pad_points = gateway.pad_points_pub.messages[0]
        self.assertEqual(pad_points.header.frame_id, "base_link")
        self.assertEqual(len(pad_points.polygon.points), 3)
        self.assertAlmostEqual(pad_points.polygon.points[0].y, -0.0445, places=5)
        self.assertAlmostEqual(pad_points.polygon.points[1].y, 0.0445, places=5)
        self.assertAlmostEqual(pad_points.polygon.points[2].y, 0.0, places=5)

    def test_parity_tcp_publishers_skip_publish_when_env_disabled_or_unresolved(
        self,
    ) -> None:
        """TINKER_SIM_PARITY_TCP=0 must fully disable the block (no publish
        calls at all); a resolvable-but-currently-unresolved backend (##35's
        fail-soft contract) must skip publishing without raising."""
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String
        from geometry_msgs.msg import PolygonStamped, PoseStamped, WrenchStamped

        class _NoOpBackend:
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
                return {}

            def physics_truth_frame(self, token):
                return {}

            def parity_tcp_frame(self):
                self.calls = getattr(self, "calls", 0) + 1
                return None  # unresolved this tick

            def parity_gripper_torque(self):
                return None  # keep this test focused on the TCP/pad block

        class _RecordingPublisher:
            def __init__(self) -> None:
                self.messages: list[object] = []

            def publish(self, message) -> None:
                self.messages.append(message)

        def _new_gateway(backend, *, parity_tcp_enabled: bool) -> RosStandardGateway:
            gateway = object.__new__(RosStandardGateway)
            gateway.backend = backend
            gateway._Clock = Clock
            gateway._JointState = JointState
            gateway._Imu = Imu
            gateway._String = String
            gateway._WrenchStamped = WrenchStamped
            gateway._PoseStamped = PoseStamped
            gateway._PolygonStamped = PolygonStamped
            gateway.clock_pub = _RecordingPublisher()
            gateway.joint_pub = _RecordingPublisher()
            gateway.imu_pub = _RecordingPublisher()
            gateway.status_pub = _RecordingPublisher()
            gateway.contact_pub = _RecordingPublisher()
            gateway.physics_truth_pub = _RecordingPublisher()
            gateway.cloud_pub = _RecordingPublisher()
            gateway.tcp_pose_pub = _RecordingPublisher()
            gateway.tcp_pose_base_pub = _RecordingPublisher()
            gateway.pad_points_pub = _RecordingPublisher()
            gateway._parity_tcp_enabled = parity_tcp_enabled
            gateway._camera_rig = None
            gateway._cloud_publish_enabled = lambda: False
            gateway._last_command_error = None
            gateway._command_stream_lost = False
            gateway._command_epoch = 0
            gateway._last_logical_snapshot_id = -1
            gateway.development_lidar = False
            gateway._publish_profile_enabled = False
            gateway._state_stride = 1_000_000
            gateway._imu_stride = 1_000_000
            gateway._status_stride = 1_000_000
            gateway._tick = 0
            return gateway

        disabled_backend = _NoOpBackend()
        disabled_gateway = _new_gateway(disabled_backend, parity_tcp_enabled=False)
        disabled_gateway.publish()
        self.assertEqual(len(disabled_gateway.tcp_pose_pub.messages), 0)
        self.assertEqual(len(disabled_gateway.pad_points_pub.messages), 0)
        self.assertEqual(getattr(disabled_backend, "calls", 0), 0)

        unresolved_backend = _NoOpBackend()
        unresolved_gateway = _new_gateway(unresolved_backend, parity_tcp_enabled=True)
        unresolved_gateway.publish()
        self.assertEqual(len(unresolved_gateway.tcp_pose_pub.messages), 0)
        self.assertEqual(len(unresolved_gateway.tcp_pose_base_pub.messages), 0)
        self.assertEqual(len(unresolved_gateway.pad_points_pub.messages), 0)
        self.assertEqual(unresolved_backend.calls, 1)

    def test_gateway_registers_gripper_physx_tau_publisher_gated_by_env(self) -> None:
        source = (ROOT / "simulation/tinker_sim_isaac/ros_gateway.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'JointState, "/sim/parity/gripper_physx_tau", reliable', source
        )
        self.assertIn("self.backend.parity_gripper_torque()", source)
        self.assertIn(
            'os.environ.get("TINKER_SIM_PARITY_TCP", "1") != "0"', source
        )

    def test_gripper_physx_tau_publisher_emits_every_tick_with_sim_stamp(
        self,
    ) -> None:
        """#33: /sim/parity/gripper_physx_tau publishes at the same cadence
        as the other parity topics (unconditional, every publish() tick, not
        gated on the status-heartbeat stride), stamped with the same sim
        clock the /clock and /isaac_joint_states publishes use this tick,
        with name/position/velocity/effort taken straight from
        backend.parity_gripper_torque()."""
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String
        from geometry_msgs.msg import PolygonStamped, PoseStamped, WrenchStamped

        gripper_names = (
            "drive_joint",
            "left_finger_joint",
            "left_inner_knuckle_joint",
            "right_outer_knuckle_joint",
            "right_inner_knuckle_joint",
            "right_finger_joint",
        )
        gripper_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        gripper_velocities = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
        gripper_physx_tau = [2.0, 2.1, 2.2, 2.3, 2.4, 2.5]

        class _GripperTorqueBackend:
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
                return {}

            def physics_truth_frame(self, token):
                return {}

            def parity_tcp_frame(self):
                return None

            def parity_gripper_torque(self):
                return (
                    gripper_names,
                    list(gripper_positions),
                    list(gripper_velocities),
                    list(gripper_physx_tau),
                )

        class _RecordingPublisher:
            def __init__(self) -> None:
                self.messages: list[object] = []

            def publish(self, message) -> None:
                self.messages.append(message)

        gateway = object.__new__(RosStandardGateway)
        gateway.backend = _GripperTorqueBackend()
        gateway._Clock = Clock
        gateway._JointState = JointState
        gateway._Imu = Imu
        gateway._String = String
        gateway._WrenchStamped = WrenchStamped
        gateway._PoseStamped = PoseStamped
        gateway._PolygonStamped = PolygonStamped
        gateway.clock_pub = _RecordingPublisher()
        gateway.joint_pub = _RecordingPublisher()
        gateway.imu_pub = _RecordingPublisher()
        gateway.status_pub = _RecordingPublisher()
        gateway.contact_pub = _RecordingPublisher()
        gateway.physics_truth_pub = _RecordingPublisher()
        gateway.cloud_pub = _RecordingPublisher()
        gateway.tcp_pose_pub = _RecordingPublisher()
        gateway.tcp_pose_base_pub = _RecordingPublisher()
        gateway.pad_points_pub = _RecordingPublisher()
        gateway.gripper_physx_tau_pub = _RecordingPublisher()
        gateway._parity_tcp_enabled = True
        gateway._camera_rig = None
        gateway._cloud_publish_enabled = lambda: False
        gateway._last_command_error = None
        gateway._command_stream_lost = False
        gateway._command_epoch = 0
        gateway._last_logical_snapshot_id = -1
        gateway.development_lidar = False
        gateway._publish_profile_enabled = False
        gateway._state_stride = 1_000_000
        gateway._imu_stride = 1_000_000
        gateway._status_stride = 1_000_000
        gateway._tick = 0

        for _ in range(3):
            gateway.publish()

        self.assertEqual(len(gateway.gripper_physx_tau_pub.messages), 3)
        message = gateway.gripper_physx_tau_pub.messages[0]
        self.assertEqual(list(message.name), list(gripper_names))
        for actual, expected in zip(message.position, gripper_positions):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(message.velocity, gripper_velocities):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(message.effort, gripper_physx_tau):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertEqual(message.header.stamp, gateway.clock_pub.messages[0].clock)

    def test_gripper_physx_tau_publisher_skips_when_env_disabled_or_unresolved(
        self,
    ) -> None:
        """TINKER_SIM_PARITY_TCP=0 must fully disable this topic too (no
        parity_gripper_torque() calls at all); a resolvable-but-currently-
        unresolved backend (#33's fail-soft contract, returns None) must
        skip publishing without raising."""
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String
        from geometry_msgs.msg import PolygonStamped, PoseStamped, WrenchStamped

        class _NoOpBackend:
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
                return {}

            def physics_truth_frame(self, token):
                return {}

            def parity_tcp_frame(self):
                return None

            def parity_gripper_torque(self):
                self.calls = getattr(self, "calls", 0) + 1
                return None  # unresolved this tick

        class _RecordingPublisher:
            def __init__(self) -> None:
                self.messages: list[object] = []

            def publish(self, message) -> None:
                self.messages.append(message)

        def _new_gateway(backend, *, parity_tcp_enabled: bool) -> RosStandardGateway:
            gateway = object.__new__(RosStandardGateway)
            gateway.backend = backend
            gateway._Clock = Clock
            gateway._JointState = JointState
            gateway._Imu = Imu
            gateway._String = String
            gateway._WrenchStamped = WrenchStamped
            gateway._PoseStamped = PoseStamped
            gateway._PolygonStamped = PolygonStamped
            gateway.clock_pub = _RecordingPublisher()
            gateway.joint_pub = _RecordingPublisher()
            gateway.imu_pub = _RecordingPublisher()
            gateway.status_pub = _RecordingPublisher()
            gateway.contact_pub = _RecordingPublisher()
            gateway.physics_truth_pub = _RecordingPublisher()
            gateway.cloud_pub = _RecordingPublisher()
            gateway.tcp_pose_pub = _RecordingPublisher()
            gateway.tcp_pose_base_pub = _RecordingPublisher()
            gateway.pad_points_pub = _RecordingPublisher()
            gateway.gripper_physx_tau_pub = _RecordingPublisher()
            gateway._parity_tcp_enabled = parity_tcp_enabled
            gateway._camera_rig = None
            gateway._cloud_publish_enabled = lambda: False
            gateway._last_command_error = None
            gateway._command_stream_lost = False
            gateway._command_epoch = 0
            gateway._last_logical_snapshot_id = -1
            gateway.development_lidar = False
            gateway._publish_profile_enabled = False
            gateway._state_stride = 1_000_000
            gateway._imu_stride = 1_000_000
            gateway._status_stride = 1_000_000
            gateway._tick = 0
            return gateway

        disabled_backend = _NoOpBackend()
        disabled_gateway = _new_gateway(disabled_backend, parity_tcp_enabled=False)
        disabled_gateway.publish()
        self.assertEqual(len(disabled_gateway.gripper_physx_tau_pub.messages), 0)
        self.assertEqual(getattr(disabled_backend, "calls", 0), 0)

        unresolved_backend = _NoOpBackend()
        unresolved_gateway = _new_gateway(unresolved_backend, parity_tcp_enabled=True)
        unresolved_gateway.publish()
        self.assertEqual(len(unresolved_gateway.gripper_physx_tau_pub.messages), 0)
        self.assertEqual(unresolved_backend.calls, 1)

    def test_gateway_registers_gripper_targets_publisher_gated_by_env(self) -> None:
        source = (ROOT / "simulation/tinker_sim_isaac/ros_gateway.py").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            'JointState, "/sim/parity/gripper_targets", reliable', source
        )
        self.assertIn("self.backend.parity_gripper_targets()", source)

    def test_gripper_targets_publisher_emits_names_and_values_from_backend(
        self,
    ) -> None:
        """#33 follow-up (bench round ahh): /sim/parity/gripper_targets
        publishes the backend's (names, values) straight through, stamped
        with the tick's sim clock, same cadence/gate as the sibling parity
        topics."""
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String
        from geometry_msgs.msg import PolygonStamped, PoseStamped, WrenchStamped

        names = ("drive_joint/py_target", "drive_joint/measured", "articulation/target_write")
        values = [0.34, 0.30, 1.0]

        class _GripperTargetsBackend:
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
                return {}

            def physics_truth_frame(self, token):
                return {}

            def parity_tcp_frame(self):
                return None

            def parity_gripper_torque(self):
                return None

            def parity_gripper_targets(self):
                return list(names), list(values)

        class _RecordingPublisher:
            def __init__(self) -> None:
                self.messages: list[object] = []

            def publish(self, message) -> None:
                self.messages.append(message)

        gateway = object.__new__(RosStandardGateway)
        gateway.backend = _GripperTargetsBackend()
        gateway._Clock = Clock
        gateway._JointState = JointState
        gateway._Imu = Imu
        gateway._String = String
        gateway._WrenchStamped = WrenchStamped
        gateway._PoseStamped = PoseStamped
        gateway._PolygonStamped = PolygonStamped
        gateway.clock_pub = _RecordingPublisher()
        gateway.joint_pub = _RecordingPublisher()
        gateway.imu_pub = _RecordingPublisher()
        gateway.status_pub = _RecordingPublisher()
        gateway.contact_pub = _RecordingPublisher()
        gateway.physics_truth_pub = _RecordingPublisher()
        gateway.cloud_pub = _RecordingPublisher()
        gateway.tcp_pose_pub = _RecordingPublisher()
        gateway.tcp_pose_base_pub = _RecordingPublisher()
        gateway.pad_points_pub = _RecordingPublisher()
        gateway.gripper_physx_tau_pub = _RecordingPublisher()
        gateway.gripper_targets_pub = _RecordingPublisher()
        gateway._parity_tcp_enabled = True
        gateway._camera_rig = None
        gateway._cloud_publish_enabled = lambda: False
        gateway._last_command_error = None
        gateway._command_stream_lost = False
        gateway._command_epoch = 0
        gateway._last_logical_snapshot_id = -1
        gateway.development_lidar = False
        gateway._publish_profile_enabled = False
        gateway._state_stride = 1_000_000
        gateway._imu_stride = 1_000_000
        gateway._status_stride = 1_000_000
        gateway._tick = 0

        for _ in range(3):
            gateway.publish()

        self.assertEqual(len(gateway.gripper_targets_pub.messages), 3)
        message = gateway.gripper_targets_pub.messages[0]
        self.assertEqual(list(message.name), list(names))
        for actual, expected in zip(message.position, values):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertEqual(message.header.stamp, gateway.clock_pub.messages[0].clock)

    def test_gripper_targets_publisher_skips_cleanly_on_none_disabled_or_raise(
        self,
    ) -> None:
        """Fail-soft contract: TINKER_SIM_PARITY_TCP=0 disables the topic
        entirely (no parity_gripper_targets() calls); a backend returning
        None publishes nothing; a backend whose parity_gripper_targets()
        RAISES must not propagate out of publish() -- the gateway-side
        try/except (distinct from the backend's own internal fail-soft)
        must log once and continue publishing everything else."""
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import String
        from geometry_msgs.msg import PolygonStamped, PoseStamped, WrenchStamped

        class _Logger:
            def __init__(self) -> None:
                self.errors: list[str] = []

            def error(self, message: str) -> None:
                self.errors.append(message)

        class _Backend:
            dt = 0.02
            physics_device = "cpu"
            safety_stopped = False
            simulation_time = 0.0
            TRUTH_TOKEN = object()

            def __init__(self, mode: str) -> None:
                self.mode = mode
                self.calls = 0

            def joint_state(self):
                return ((), [], [], [])

            def root_state(self):
                return {"angular_velocity_world": (0.0, 0.0, 0.0)}

            def contact_state(self):
                return {}

            def physics_truth_frame(self, token):
                return {}

            def parity_tcp_frame(self):
                return None

            def parity_gripper_torque(self):
                return None

            def parity_gripper_targets(self):
                self.calls += 1
                if self.mode == "none":
                    return None
                raise RuntimeError("physx view not ready")

        class _RecordingPublisher:
            def __init__(self) -> None:
                self.messages: list[object] = []

            def publish(self, message) -> None:
                self.messages.append(message)

        def _new_gateway(backend, *, parity_tcp_enabled: bool) -> RosStandardGateway:
            gateway = object.__new__(RosStandardGateway)
            gateway.backend = backend
            gateway._Clock = Clock
            gateway._JointState = JointState
            gateway._Imu = Imu
            gateway._String = String
            gateway._WrenchStamped = WrenchStamped
            gateway._PoseStamped = PoseStamped
            gateway._PolygonStamped = PolygonStamped
            gateway.clock_pub = _RecordingPublisher()
            gateway.joint_pub = _RecordingPublisher()
            gateway.imu_pub = _RecordingPublisher()
            gateway.status_pub = _RecordingPublisher()
            gateway.contact_pub = _RecordingPublisher()
            gateway.physics_truth_pub = _RecordingPublisher()
            gateway.cloud_pub = _RecordingPublisher()
            gateway.tcp_pose_pub = _RecordingPublisher()
            gateway.tcp_pose_base_pub = _RecordingPublisher()
            gateway.pad_points_pub = _RecordingPublisher()
            gateway.gripper_physx_tau_pub = _RecordingPublisher()
            gateway.gripper_targets_pub = _RecordingPublisher()
            gateway._parity_tcp_enabled = parity_tcp_enabled
            gateway._camera_rig = None
            gateway._cloud_publish_enabled = lambda: False
            gateway._last_command_error = None
            gateway._command_stream_lost = False
            gateway._command_epoch = 0
            gateway._last_logical_snapshot_id = -1
            gateway.development_lidar = False
            gateway._publish_profile_enabled = False
            gateway._state_stride = 1_000_000
            gateway._imu_stride = 1_000_000
            gateway._status_stride = 1_000_000
            gateway._tick = 0
            # A SHARED logger instance -- get_logger() must return the same
            # object every call so error counts accumulate across ticks,
            # unlike the "must not raise" fixtures elsewhere in this file
            # that construct a fresh _Logger() per call and never inspect it.
            logger = _Logger()
            gateway.node = SimpleNamespace(get_logger=lambda: logger)
            return gateway

        disabled_backend = _Backend("none")
        disabled_gateway = _new_gateway(disabled_backend, parity_tcp_enabled=False)
        disabled_gateway.publish()
        self.assertEqual(len(disabled_gateway.gripper_targets_pub.messages), 0)
        self.assertEqual(disabled_backend.calls, 0)

        none_backend = _Backend("none")
        none_gateway = _new_gateway(none_backend, parity_tcp_enabled=True)
        none_gateway.publish()
        self.assertEqual(len(none_gateway.gripper_targets_pub.messages), 0)
        self.assertEqual(none_backend.calls, 1)

        raising_backend = _Backend("raise")
        raising_gateway = _new_gateway(raising_backend, parity_tcp_enabled=True)
        for _ in range(2):
            raising_gateway.publish()
        self.assertEqual(len(raising_gateway.gripper_targets_pub.messages), 0)
        self.assertEqual(raising_backend.calls, 2)
        # publish() itself must not have raised (physics_truth still fires
        # every tick), and the failure is logged exactly once, not per tick.
        self.assertEqual(len(raising_gateway.physics_truth_pub.messages), 2)
        self.assertEqual(len(raising_gateway.node.get_logger().errors), 1)

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


class UseFabricDerivationTest(unittest.TestCase):
    """TINKER_SIM_SPAWN_YAW_VIA_VIEW=1 lets use_fabric stay True even under a
    non-zero TINKER_SIM_SPAWN_YAW (#24); off (default) reproduces 13e4fdf's
    force-fabric-off behaviour byte for byte."""

    def test_via_view_flag_parsing(self) -> None:
        self.assertFalse(resolve_spawn_yaw_via_view(None))
        self.assertFalse(resolve_spawn_yaw_via_view(""))
        self.assertFalse(resolve_spawn_yaw_via_view("0"))
        self.assertFalse(resolve_spawn_yaw_via_view("false"))
        self.assertFalse(resolve_spawn_yaw_via_view("no"))
        self.assertTrue(resolve_spawn_yaw_via_view("1"))
        self.assertTrue(resolve_spawn_yaw_via_view("true"))
        self.assertTrue(resolve_spawn_yaw_via_view("True"))
        self.assertTrue(resolve_spawn_yaw_via_view(" 1 "))

    def test_yaw_set_flag_off_fabric_false(self) -> None:
        # Current/default behaviour (13e4fdf): unchanged.
        self.assertFalse(resolve_use_fabric("1.5708", None, None))
        self.assertFalse(resolve_use_fabric("1.5708", "0", None))

    def test_yaw_set_flag_on_fabric_true(self) -> None:
        self.assertTrue(resolve_use_fabric("1.5708", "1", None))
        self.assertTrue(resolve_use_fabric("1.5708", "true", None))

    def test_no_yaw_fabric_true_regardless_of_flag(self) -> None:
        self.assertTrue(resolve_use_fabric(None, None, None))
        self.assertTrue(resolve_use_fabric("0.0", None, None))
        self.assertTrue(resolve_use_fabric("0.0", "1", None))

    def test_use_fabric_env_override_still_wins(self) -> None:
        self.assertTrue(resolve_use_fabric("1.5708", None, "1"))
        self.assertFalse(resolve_use_fabric(None, None, "0"))
        # Override wins even over the new flag.
        self.assertFalse(resolve_use_fabric("1.5708", "1", "0"))

    def test_usd_authoring_guard_matches_derivation(self) -> None:
        """Mirrors backend.__init__'s USD-authoring guard,
        ``abs(spawn_yaw) > 1e-9 and not self._spawn_yaw_via_view``: verifies
        the pre-reset USD xformOp:orient path still runs, unmodified, when
        the flag is off, and is skipped only when it is on."""
        spawn_yaw = resolve_spawn_yaw("1.5708")
        spawn_yaw_set = abs(spawn_yaw) > 1.0e-9
        self.assertTrue(spawn_yaw_set)

        via_view_off = spawn_yaw_set and resolve_spawn_yaw_via_view(None)
        self.assertTrue(spawn_yaw_set and not via_view_off)  # USD path runs

        via_view_on = spawn_yaw_set and resolve_spawn_yaw_via_view("1")
        self.assertFalse(spawn_yaw_set and not via_view_on)  # USD path skipped


class SpawnRootRotXyzwTest(unittest.TestCase):
    """#30 root cause: ``ArticulationCfg.InitialStateCfg.rot`` is scalar-last
    (x, y, z, w) in the vendored Isaac Lab 3.0.0 (``asset_base_cfg.py``
    defaults it to ``(0.0, 0.0, 0.0, 1.0)``), not the scalar-first
    (w, x, y, z) order the backend used to build by hand. A zero/unset spawn
    yaw therefore used to hand Isaac Lab ``(1, 0, 0, 0)`` -- a 180 degree
    roll about X -- and the robot spawned upside down."""

    def test_zero_yaw_matches_vendored_default_identity(self) -> None:
        self.assertEqual(spawn_root_rot_xyzw(0.0), (0.0, 0.0, 0.0, 1.0))

    def test_quarter_turn_yaw_is_scalar_last(self) -> None:
        qx, qy, qz, qw = spawn_root_rot_xyzw(math.pi / 2.0)
        self.assertAlmostEqual(qx, 0.0, places=6)
        self.assertAlmostEqual(qy, 0.0, places=6)
        self.assertAlmostEqual(qz, 0.7071067811865476, places=6)
        self.assertAlmostEqual(qw, 0.7071067811865476, places=6)

    def test_never_matches_the_old_scalar_first_bug_at_zero_yaw(self) -> None:
        """Regression guard: the pre-#30 construction ``(cos(0), 0, 0, sin(0))``
        produced ``(1, 0, 0, 0)`` for a zero yaw -- a 180 degree rotation
        about X under the scalar-last contract. The fixed helper must not
        reproduce that."""
        self.assertNotEqual(spawn_root_rot_xyzw(0.0), (1.0, 0.0, 0.0, 0.0))


class BackendInitStateRotConstructionTest(unittest.TestCase):
    """Construction test (#30): the exact ``rot=`` expression the backend
    passes to ``ArticulationCfg.InitialStateCfg`` -- extracted from source so
    this cannot pass by the test independently re-deriving the same order
    bug -- must equal ``spawn_root_rot_xyzw(spawn_yaw)`` for both an unset
    and a non-zero ``TINKER_SIM_SPAWN_YAW``.

    On b6af3ce (pre-fix) the backend built this keyword as
    ``(math.cos(yaw/2), 0.0, 0.0, math.sin(yaw/2))`` -- scalar-first -- so
    for yaw unset (0.0) this evaluates to ``(1.0, 0.0, 0.0, 0.0)``, which
    fails the ``(0.0, 0.0, 0.0, 1.0)`` expectation below.
    """

    def _eval_rot(self, spawn_yaw_env: str | None) -> tuple:
        backend_source = (
            ROOT / "simulation/tinker_sim_isaac/backend.py"
        ).read_text(encoding="utf-8")
        rot_source = _init_state_rot_source(backend_source)
        self.assertTrue(
            rot_source,
            "no rot= keyword found on an *.InitialStateCfg(...) call in backend.py",
        )
        fake_os = SimpleNamespace(environ=SimpleNamespace(get=lambda *a, **k: spawn_yaw_env))
        namespace = {
            "math": math,
            "spawn_root_rot_xyzw": spawn_root_rot_xyzw,
            "resolve_spawn_yaw": resolve_spawn_yaw,
            "_os": fake_os,
        }
        return eval(compile(rot_source, "<rot-source>", "eval"), namespace)

    def test_unset_spawn_yaw_matches_helper(self) -> None:
        rot = self._eval_rot(None)
        expected = spawn_root_rot_xyzw(resolve_spawn_yaw(None))
        self.assertEqual(tuple(rot), expected)
        self.assertEqual(tuple(rot), (0.0, 0.0, 0.0, 1.0))

    def test_nonzero_spawn_yaw_matches_helper(self) -> None:
        rot = self._eval_rot("1.5708")
        expected = spawn_root_rot_xyzw(resolve_spawn_yaw("1.5708"))
        self.assertEqual(tuple(rot), expected)
        qx, qy, qz, qw = rot
        self.assertAlmostEqual(qx, 0.0, places=4)
        self.assertAlmostEqual(qy, 0.0, places=4)
        self.assertAlmostEqual(qz, 0.7071, places=4)
        self.assertAlmostEqual(qw, 0.7071, places=4)


class SpawnYawViaViewApplyTest(unittest.TestCase):
    """``IsaacWholeRobotBackend._apply_spawn_yaw_via_view`` (#24): writes the
    commanded yaw through the root-view writer post-reset, position
    untouched, and seeds the base-hold target when FIX_BASE is active."""

    def test_writes_yaw_quaternion_once_position_unchanged(self) -> None:
        backend = _backend()
        backend.base_fixed = False
        spawn_yaw = math.pi / 2.0

        backend._apply_spawn_yaw_via_view(spawn_yaw)

        self.assertEqual(len(backend._robot.root_pose_calls), 1)
        written = backend._robot.root_pose_calls[0]
        self.assertEqual(tuple(written.shape), (1, 7))
        written_row = written[0].tolist()
        original_pos = backend._robot.data.root_pos_w[0].tolist()
        for actual, expected in zip(written_row[:3], original_pos):
            self.assertAlmostEqual(actual, expected, places=6)
        # Root-view quaternions are (x, y, z, w) -- NOT the (w, x, y, z) order
        # InitialStateCfg.rot / the USD xformOp:orient path use.
        half = spawn_yaw / 2.0
        expected_quat = [0.0, 0.0, math.sin(half), math.cos(half)]
        for actual, expected in zip(written_row[3:], expected_quat):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_seeds_base_hold_target_when_fix_base_active(self) -> None:
        """#26: the ORIENTATION is seeded immediately (so it survives a
        rebind with no durable USD backing), but the POSITION is deferred
        to _apply_base_hold's own settle-latch -- _base_hold_pose itself
        must stay None here, not lock in the pre-settle root_pos_w read."""
        backend = _backend()
        backend.base_fixed = True
        backend._base_hold_pose = "stale-from-a-prior-rebind"
        backend._base_hold_vel = "stale-vel-from-a-prior-rebind"
        backend._scenario_child_signature = lambda: 7

        backend._apply_spawn_yaw_via_view(0.4)

        written = backend._robot.root_pose_calls[0]
        self.assertIsNone(backend._base_hold_pose)
        self.assertIsNone(backend._base_hold_vel)
        self.assertIsNotNone(backend._base_hold_seed_quat)
        self.assertEqual(
            backend._base_hold_seed_quat.tolist(), written[:, 3:].tolist()
        )

    def test_does_not_seed_base_hold_when_fix_base_inactive(self) -> None:
        backend = _backend()
        backend.base_fixed = False
        backend._base_hold_pose = None
        backend._base_hold_vel = None
        backend._base_hold_seed_quat = None

        backend._apply_spawn_yaw_via_view(0.4)

        self.assertIsNone(backend._base_hold_pose)
        self.assertIsNone(backend._base_hold_vel)
        self.assertIsNone(backend._base_hold_seed_quat)
        # The write itself still happens -- only the hold-target seeding is
        # gated on FIX_BASE.
        self.assertEqual(len(backend._robot.root_pose_calls), 1)

    def test_base_hold_latches_settled_height_with_seeded_yaw(self) -> None:
        """#26 regression: the seeded yaw must survive to the LATCHED hold,
        but the latched POSITION must come from the settled read
        _apply_base_hold takes once its settle timer elapses, not the
        pre-settle root_pos_w _apply_spawn_yaw_via_view saw at spawn time."""
        backend = _backend()
        backend.base_fixed = True
        backend._base_hold_pose = None
        backend._base_hold_vel = None
        backend._base_hold_seed_quat = None
        backend._base_hold_scene_sig = None
        backend._base_hold_skip_until_sim_s = 0.0
        backend._base_hold_resettle_s = 0.15
        backend._base_hold_dryrun = False
        backend._base_hold_after_sim_s = 2.0
        backend._robot.data.root_pos_w = torch.tensor(
            [[1.0, 2.0, 0.20]], dtype=torch.float32
        )

        backend._apply_spawn_yaw_via_view(0.4)
        self.assertIsNone(backend._base_hold_pose)
        half = 0.4 / 2.0
        expected_quat = [0.0, 0.0, math.sin(half), math.cos(half)]

        # Gravity settles the chassis onto its wheels: root_pos_w now reads
        # the lower, settled rest height.
        backend._robot.data.root_pos_w = torch.tensor(
            [[1.0, 2.0, 0.0775]], dtype=torch.float32
        )
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 241)
        backend._clock_step_origin = 0
        backend.physics_dt = 1.0 / 120.0

        backend._apply_base_hold()

        self.assertIsNotNone(backend._base_hold_pose)
        latched = backend._base_hold_pose[0].tolist()
        self.assertAlmostEqual(latched[2], 0.0775, places=6)
        for actual, expected in zip(latched[3:], expected_quat):
            self.assertAlmostEqual(actual, expected, places=6)


class SpawnYawViaViewRebindTest(unittest.TestCase):
    """#24 review Finding 1: a standard scenario reset (STOP -> spawn ->
    PLAY) recreates the articulation root view, which ``_refresh_robot_handles``
    detects as a new view identity. The yaw must be re-applied on that
    rebind too, not just at the initial boot bind, or the flag's own target
    use case (spawn objects, then reset) silently reverts to identity yaw.

    Round 2: whether a rebind reapplies is the caller's EXPLICIT
    ``reapply_spawn_yaw`` classification (default True == boot / genuine
    scenario reset), never inferred from pose. ``_maybe_recover_simulation_view``'s
    mid-run, state-preserving rebind passes ``reapply_spawn_yaw=False`` and
    must leave the root pose and any base-hold target completely untouched
    -- see ``SpawnYawViaViewRecoveryRebindTest`` below."""

    def test_reapplies_yaw_on_rebind_when_via_view_active(self) -> None:
        """#26: a rebind must re-seed the pending yaw (so it survives the
        rebind with no durable USD backing) but must NOT re-latch
        _base_hold_pose's position from the fresh, pre-settle rebind read
        -- it clears any stale hold from before the rebind instead, so
        _apply_base_hold's own settle-latch runs again on a SETTLED read."""
        backend = _backend()
        backend._robot_view_identity = -1  # force _refresh_robot_handles to see a "new" view
        backend._clock_step_origin = 0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 42)
        backend._object_views = {"delivery_object": object()}
        backend._spawn_yaw_via_view = True
        backend._spawn_yaw = math.pi / 2.0
        backend.base_fixed = True
        # Sentinel stale hold from before the rebind -- must be cleared.
        backend._base_hold_pose = torch.tensor([[9.0, 9.0, 9.0, 0.0, 0.0, 0.0, 1.0]])
        backend._base_hold_vel = torch.zeros((1, 6))

        self.assertTrue(backend._refresh_robot_handles())

        self.assertEqual(len(backend._robot.root_pose_calls), 1)
        written_row = backend._robot.root_pose_calls[0][0].tolist()
        half = backend._spawn_yaw / 2.0
        expected_quat = [0.0, 0.0, math.sin(half), math.cos(half)]
        for actual, expected in zip(written_row[3:], expected_quat):
            self.assertAlmostEqual(actual, expected, places=6)
        # The stale hold is cleared, and the yaw is remembered as pending,
        # rather than being re-latched immediately from the pre-settle
        # rebind read.
        self.assertIsNone(backend._base_hold_pose)
        self.assertIsNone(backend._base_hold_vel)
        seed_quat_row = backend._base_hold_seed_quat[0].tolist()
        for actual, expected in zip(seed_quat_row, expected_quat):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_reapplies_yaw_on_every_rebind_not_just_the_first(self) -> None:
        backend = _backend()
        backend._robot_view_identity = -1
        backend._clock_step_origin = 0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 1)
        backend._object_views = {}
        backend._spawn_yaw_via_view = True
        backend._spawn_yaw = 0.4
        backend.base_fixed = False

        self.assertTrue(backend._refresh_robot_handles())
        self.assertEqual(len(backend._robot.root_pose_calls), 1)

        # A second, independent rebind (e.g. a later reset cycle) must
        # re-apply the yaw again, not rely on the first application alone.
        backend._robot_view_identity = -2
        self.assertTrue(backend._refresh_robot_handles())
        self.assertEqual(len(backend._robot.root_pose_calls), 2)

    def test_no_write_on_rebind_when_via_view_inactive(self) -> None:
        backend = _backend()
        backend._robot_view_identity = -1
        backend._clock_step_origin = 0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 42)
        backend._object_views = {}
        backend._spawn_yaw_via_view = False

        self.assertTrue(backend._refresh_robot_handles())

        self.assertEqual(backend._robot.root_pose_calls, [])

    def test_no_write_on_rebind_when_flag_unset(self) -> None:
        # _backend() does not set _spawn_yaw_via_view at all -- the
        # getattr(..., False) default in _reapply_spawn_yaw_after_rebind
        # must hold, mirroring a real backend that never enabled the flag.
        backend = _backend()
        backend._robot_view_identity = -1
        backend._clock_step_origin = 0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 42)
        backend._object_views = {}

        self.assertTrue(backend._refresh_robot_handles())

        self.assertEqual(backend._robot.root_pose_calls, [])

    def test_unchanged_view_identity_does_not_reapply(self) -> None:
        backend = _backend()
        backend._spawn_yaw_via_view = True
        backend._spawn_yaw = math.pi / 2.0
        # _robot_view_identity already matches _FakeRobot().root_view's id
        # (set by _backend()), so this call must take the early "no rebind
        # happened" path and not touch the root-pose writer at all.
        self.assertTrue(backend._refresh_robot_handles())
        self.assertEqual(backend._robot.root_pose_calls, [])

    def test_boot_bind_reapplies_yaw(self) -> None:
        """The very first bind (``_robot_view_identity`` starts ``None``,
        as it does in ``__init__``) is also a ``reapply_spawn_yaw=True``
        (default) rebind, and must apply the yaw exactly like the boot path
        in ``IsaacWholeRobotBackend.__init__`` relies on."""
        backend = _backend()
        backend._robot_view_identity = None
        backend._clock_step_origin = 0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 0)
        backend._object_views = {}
        backend._spawn_yaw_via_view = True
        backend._spawn_yaw = 1.5708
        backend.base_fixed = False

        self.assertTrue(backend._refresh_robot_handles())

        self.assertEqual(len(backend._robot.root_pose_calls), 1)
        written_row = backend._robot.root_pose_calls[0][0].tolist()
        half = backend._spawn_yaw / 2.0
        expected_quat = [0.0, 0.0, math.sin(half), math.cos(half)]
        for actual, expected in zip(written_row[3:], expected_quat):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_rebind_after_boot_waits_for_settle_before_relatching(self) -> None:
        """#26 round 2: a genuine mid-run reset (STOP -> spawn -> PLAY),
        happening minutes into a live run, must NOT reuse the BOOT-relative
        settle deadline. simulation_time is kept monotonic across a rebind
        (#21), so a boot-relative 2.0s deadline is already satisfied by the
        time a live-run reset happens, and _apply_base_hold would otherwise
        re-latch on its very next call -- at the pre-settle respawn height.
        The deadline must instead be measured from THIS rebind's clear."""
        backend = _backend()
        backend._robot_view_identity = -1  # force a "new view" rebind
        backend._object_views = {}
        backend._spawn_yaw_via_view = True
        backend._spawn_yaw = math.pi / 2.0
        backend.base_fixed = True
        backend._base_hold_pose = None
        backend._base_hold_vel = None
        backend._base_hold_seed_quat = None
        backend._base_hold_scene_sig = None
        backend._base_hold_skip_until_sim_s = 0.0
        backend._base_hold_resettle_s = 0.15
        backend._base_hold_dryrun = False
        backend._base_hold_after_sim_s = 2.0
        backend.physics_dt = 1.0 / 120.0
        # 40.0s of elapsed sim time BEFORE this rebind -- well past the 2.0s
        # boot-relative deadline, representing a reset minutes into a live
        # run rather than the initial boot bind.
        backend._clock_elapsed_steps = 4800
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 4800)
        backend._clock_step_origin = 0
        # Pre-settle respawn height, exactly like a fresh spawn/boot.
        backend._robot.data.root_pos_w = torch.tensor(
            [[1.0, 2.0, 0.20]], dtype=torch.float32
        )

        self.assertTrue(backend._refresh_robot_handles())
        self.assertIsNone(backend._base_hold_pose)
        self.assertAlmostEqual(backend._base_hold_settle_from, 40.0, places=6)

        # Immediately after the rebind (same step count, simulation_time
        # still 40.0): must NOT re-latch yet, even though 40.0 >= 2.0.
        backend._apply_base_hold()
        self.assertIsNone(
            backend._base_hold_pose,
            "re-latched immediately after a mid-run rebind instead of "
            "waiting out a fresh settle window (#26 round 2 regression)",
        )

        # Advance exactly 2.0s past the rebind's clear -- the chassis has
        # now settled onto its wheels.
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 4800 + 240)
        backend._robot.data.root_pos_w = torch.tensor(
            [[1.0, 2.0, 0.0775]], dtype=torch.float32
        )

        backend._apply_base_hold()

        self.assertIsNotNone(backend._base_hold_pose)
        latched = backend._base_hold_pose[0].tolist()
        self.assertAlmostEqual(latched[2], 0.0775, places=6)
        half = backend._spawn_yaw / 2.0
        expected_quat = [0.0, 0.0, math.sin(half), math.cos(half)]
        for actual, expected in zip(latched[3:], expected_quat):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_flag_off_rebind_keeps_previously_latched_hold(self) -> None:
        """Flag-off comparison / byte-identical-behaviour guard:
        ``_reapply_spawn_yaw_after_rebind`` is a no-op when
        TINKER_SIM_SPAWN_YAW_VIA_VIEW is inactive, so a rebind must never
        clear (or otherwise disturb) an already-latched ``_base_hold_pose``
        -- unlike the via-view path, flag-off has no equivalent settle
        timer to fix, and this round's change must not give it one."""
        backend = _backend()
        backend._robot_view_identity = -1  # force a "new view" rebind
        backend._object_views = {}
        backend._spawn_yaw_via_view = False
        backend.base_fixed = True
        latched_pose = torch.tensor(
            [[1.0, 2.0, 0.0775, 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32
        )
        latched_vel = torch.zeros((1, 6))
        backend._base_hold_pose = latched_pose
        backend._base_hold_vel = latched_vel
        backend._base_hold_seed_quat = None
        backend._clock_elapsed_steps = 4800
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 4800)
        backend._clock_step_origin = 0

        self.assertTrue(backend._refresh_robot_handles())

        self.assertIs(backend._base_hold_pose, latched_pose)
        self.assertIs(backend._base_hold_vel, latched_vel)
        self.assertEqual(backend._robot.root_pose_calls, [])


class SpawnYawViaViewRecoveryRebindTest(unittest.TestCase):
    """#24 review round 2 (CONFIRMED regression in fix round 1):
    ``_maybe_recover_simulation_view``'s mid-run, state-PRESERVING view
    recovery calls ``_refresh_robot_handles(reapply_spawn_yaw=False)``. That
    rebind must NOT write a root pose, must NOT touch ``_spawn_yaw`` (frozen
    at boot), and must NOT touch any existing base-hold target -- a driven
    robot's live heading (and its base-hold lock, if FIX_BASE=1) must
    survive a mid-run recovery unchanged, exactly as the state-preserving
    USD-authoring path already does today (no code anywhere re-authors
    xformOp:orient outside boot)."""

    def test_recovery_rebind_issues_no_write_when_via_view_active(self) -> None:
        backend = _backend()
        backend._robot_view_identity = -1  # force a "new view" rebind
        backend._clock_step_origin = 0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 100)
        backend._object_views = {}
        backend._spawn_yaw_via_view = True
        backend._spawn_yaw = 1.5708  # frozen boot value
        backend.base_fixed = True
        # Sentinel base-hold state as if the robot had driven/turned and the
        # hold had long since latched a live (non-boot) pose.
        driven_pose = torch.tensor([[3.0, 4.0, 0.4, 0.0, 0.0, 0.9995, 0.0316]])
        driven_vel = torch.zeros((1, 6))
        backend._base_hold_pose = driven_pose.clone()
        backend._base_hold_vel = driven_vel.clone()
        backend._base_hold_scene_sig = 123

        self.assertTrue(
            backend._refresh_robot_handles(reapply_spawn_yaw=False)
        )

        self.assertEqual(backend._robot.root_pose_calls, [])
        self.assertEqual(backend._spawn_yaw, 1.5708)
        self.assertEqual(backend._base_hold_pose.tolist(), driven_pose.tolist())
        self.assertEqual(backend._base_hold_vel.tolist(), driven_vel.tolist())
        self.assertEqual(backend._base_hold_scene_sig, 123)

    def test_recovery_rebind_still_rebuilds_other_handles(self) -> None:
        """reapply_spawn_yaw=False must only skip the yaw reapply -- the
        rest of the rebind (joint index caches, clock re-anchoring, view
        identity bookkeeping) is unrelated and must still run, exactly as
        the pre-existing test_reset_reacquires_object_views expects for the
        default case."""
        backend = _backend()
        backend._robot_view_identity = -1
        backend._clock_step_origin = 0
        backend._sim = SimpleNamespace(get_physics_step_count=lambda: 42)
        backend._object_views = {"delivery_object": object()}
        backend._spawn_yaw_via_view = True
        backend._spawn_yaw = 1.5708

        self.assertTrue(
            backend._refresh_robot_handles(reapply_spawn_yaw=False)
        )

        self.assertEqual(backend._object_views, {})
        self.assertEqual(backend._contact_pairs_by_key, {})
        self.assertEqual(backend._clock_step_origin, 42)
        self.assertEqual(backend._robot.root_pose_calls, [])


def _set_root_pose(backend: IsaacWholeRobotBackend, xyz, yaw: float) -> None:
    half = yaw / 2.0
    backend._robot.data.root_pos_w = torch.tensor([list(xyz)], dtype=torch.float32)
    backend._robot.data.root_quat_w = torch.tensor(
        [[0.0, 0.0, math.sin(half), math.cos(half)]], dtype=torch.float32
    )


def _free_base_settled(backend: IsaacWholeRobotBackend) -> None:
    """Arm the free-base (TINKER_SIM_FIX_BASE off) settle gate so the very
    next ``_maybe_check_spawn_pose`` call fires: 2.0s of sim time elapsed
    since boot/the last rebind, matching ``_base_hold_after_sim_s``."""
    backend.base_fixed = False
    backend._base_hold_after_sim_s = 2.0
    backend._spawn_pose_check_settle_from = 0.0
    backend._clock_step_origin = 0
    backend._clock_elapsed_steps = 0
    backend._sim = SimpleNamespace(get_physics_step_count=lambda: 240)
    backend._spawn_pose_checked = False


class SpawnPoseCheckTest(unittest.TestCase):
    """Task #30: a GPSR run observed the robot base at (-0.69, -2.19) yaw
    +171 deg in the sim's own physics_truth while the launch commanded
    --spawn-xy=-2,-2 (yaw unset = 0) -- nothing in the sim log flagged it.
    ``_check_spawn_pose`` compares the truth base pose against the
    commanded spawn once the base settles and makes a mismatch loud."""

    def test_matching_pose_is_ok_with_no_mismatch_line(self) -> None:
        backend = _backend()
        backend._spawn_x = 1.0
        backend._spawn_y = 2.0
        backend._spawn_yaw = 0.5
        _set_root_pose(backend, (1.0, 2.0, 0.0775), 0.5)
        _free_base_settled(backend)

        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            backend._maybe_check_spawn_pose()

        self.assertTrue(backend._spawn_pose_checked)
        lines = [
            json.loads(line) for line in captured.getvalue().splitlines() if line.strip()
        ]
        # Task #30 also emits a "spawn_pose_trace" (stage "after_settle")
        # line alongside the check -- filter to the check/mismatch events
        # this test cares about; the trace line's own shape is covered by
        # SpawnPoseTraceFormatTest.
        check_lines = [
            entry
            for entry in lines
            if entry["event"] in ("spawn_pose_check", "spawn_pose_mismatch")
        ]
        events = [entry["event"] for entry in check_lines]
        self.assertEqual(events, ["spawn_pose_check"])
        self.assertIn("spawn_pose_trace", [entry["event"] for entry in lines])
        check = check_lines[0]
        self.assertTrue(check["ok"])
        self.assertAlmostEqual(check["dxy_m"], 0.0, places=5)
        # float32 root_quat_w round-trip: exact 0 isn't guaranteed.
        self.assertAlmostEqual(check["dyaw_deg"], 0.0, places=3)

    def test_pose_off_by_1_3m_171deg_emits_mismatch(self) -> None:
        backend = _backend()
        backend._spawn_x = -2.0
        backend._spawn_y = -2.0
        backend._spawn_yaw = 0.0
        _set_root_pose(backend, (-0.69, -2.19, 0.0775), math.radians(171))
        _free_base_settled(backend)

        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            backend._maybe_check_spawn_pose()

        lines = [
            json.loads(line) for line in captured.getvalue().splitlines() if line.strip()
        ]
        check_lines = [
            entry
            for entry in lines
            if entry["event"] in ("spawn_pose_check", "spawn_pose_mismatch")
        ]
        events = [entry["event"] for entry in check_lines]
        self.assertEqual(events, ["spawn_pose_check", "spawn_pose_mismatch"])
        check, mismatch = check_lines
        self.assertFalse(check["ok"])
        self.assertAlmostEqual(check["dxy_m"], 1.3237, places=3)
        self.assertAlmostEqual(check["dyaw_deg"], 171.0, places=3)
        self.assertEqual(mismatch["commanded"], [-2.0, -2.0, 0.0])
        self.assertAlmostEqual(mismatch["dxy_m"], 1.3237, places=3)
        self.assertAlmostEqual(mismatch["dyaw_deg"], 171.0, places=3)
        self.assertFalse(mismatch["ok"])
        self.assertEqual(mismatch.get("level"), "warning")

    def test_yaw_wrap_reports_small_delta(self) -> None:
        """Commanded 3.1 rad, actual -3.1 rad -- these are ~4.8 deg apart
        going the short way around the circle, not the ~355 deg a naive
        (unwrapped) subtraction would report."""
        backend = _backend()
        backend._spawn_x = 0.0
        backend._spawn_y = 0.0
        backend._spawn_yaw = 3.1
        _set_root_pose(backend, (0.0, 0.0, 0.0775), -3.1)
        _free_base_settled(backend)

        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            backend._maybe_check_spawn_pose()

        lines = [
            json.loads(line) for line in captured.getvalue().splitlines() if line.strip()
        ]
        check_lines = [
            entry
            for entry in lines
            if entry["event"] in ("spawn_pose_check", "spawn_pose_mismatch")
        ]
        self.assertEqual([entry["event"] for entry in check_lines], ["spawn_pose_check"])
        check = check_lines[0]
        self.assertTrue(check["ok"])
        self.assertAlmostEqual(abs(check["dyaw_deg"]), 4.7662, places=3)

    def test_strict_mode_raises_on_mismatch(self) -> None:
        backend = _backend()
        backend._spawn_x = -2.0
        backend._spawn_y = -2.0
        backend._spawn_yaw = 0.0
        _set_root_pose(backend, (-0.69, -2.19, 0.0775), math.radians(171))
        _free_base_settled(backend)

        with patch.dict(os.environ, {"TINKER_SIM_SPAWN_POSE_ASSERT": "strict"}):
            captured = io.StringIO()
            with contextlib.redirect_stdout(captured):
                with self.assertRaises(RuntimeError):
                    backend._maybe_check_spawn_pose()


class SpawnPoseTraceFormatTest(unittest.TestCase):
    """Task #30: ``spawn_pose_trace`` boot lines instrumenting the
    initial-state -> reset -> yaw-write -> rebind -> settle sequence the
    #30 investigation found under-observed. ``format_spawn_pose_trace`` is
    the pure formatter (no sim needed); ``_log_spawn_pose_trace`` wires it
    to a backend double's live root pose and an optional distinct
    ``base_link`` body -- also no sim needed."""

    def test_format_spawn_pose_trace_shape(self) -> None:
        payload = format_spawn_pose_trace(
            "after_reset",
            1.5,
            [-2.0, -2.0, 0.0775],
            [0.0, 0.0, 0.0, 1.0],
        )
        self.assertEqual(payload["event"], "spawn_pose_trace")
        self.assertEqual(payload["stage"], "after_reset")
        self.assertEqual(payload["t"], 1.5)
        self.assertEqual(payload["root_pos"], [-2.0, -2.0, 0.0775])
        self.assertAlmostEqual(payload["root_yaw_deg"], 0.0, places=6)
        self.assertNotIn("base_link_pos", payload)
        self.assertNotIn("base_link_yaw_deg", payload)
        self.assertNotIn("via", payload)
        self.assertNotIn("reason", payload)

    def test_format_spawn_pose_trace_with_base_link_via_and_reason(self) -> None:
        payload = format_spawn_pose_trace(
            "after_rebind:reset_rebind",
            2.0,
            [0.0, 0.0, 0.0775],
            [0.0, 0.0, 1.0, 0.0],  # 180 deg about world Z
            base_link_pos=[0.1, 0.2, 0.3],
            base_link_quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            via="view",
            reason="reset_rebind",
        )
        self.assertEqual(payload["stage"], "after_rebind:reset_rebind")
        self.assertAlmostEqual(payload["root_yaw_deg"], 180.0, places=3)
        self.assertEqual(payload["base_link_pos"], [0.1, 0.2, 0.3])
        self.assertAlmostEqual(payload["base_link_yaw_deg"], 0.0, places=6)
        self.assertEqual(payload["via"], "view")
        self.assertEqual(payload["reason"], "reset_rebind")

    def test_log_spawn_pose_trace_from_backend_double_no_sim(self) -> None:
        """A backend double (``_backend()``, no live Isaac Sim) with a
        ``base_link`` body distinct from the articulation root, at a
        DIFFERENT pose -- exercises "if base_link and the articulation root
        are different prims, log both" end to end through
        ``_log_spawn_pose_trace``."""
        backend = _backend()
        backend._robot.data.body_names = ("base", "link_tcp", "base_link")
        backend._robot.data.body_pos_w = torch.cat(
            [
                backend._robot.data.body_pos_w,
                torch.tensor([[[1.0, 2.0, 0.5]]], dtype=torch.float32),
            ],
            dim=1,
        )
        backend._robot.data.body_quat_w = torch.cat(
            [
                backend._robot.data.body_quat_w,
                torch.tensor([[[0.0, 0.0, 0.0, 1.0]]], dtype=torch.float32),
            ],
            dim=1,
        )

        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            backend._log_spawn_pose_trace(
                "after_reset",
                root_pos=[-2.0, -2.0, 0.0775],
                root_quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            )

        lines = [
            json.loads(line) for line in captured.getvalue().splitlines() if line.strip()
        ]
        self.assertEqual(len(lines), 1)
        payload = lines[0]
        self.assertEqual(payload["event"], "spawn_pose_trace")
        self.assertEqual(payload["stage"], "after_reset")
        self.assertEqual(payload["root_pos"], [-2.0, -2.0, 0.0775])
        self.assertEqual(payload["base_link_pos"], [1.0, 2.0, 0.5])
        self.assertNotEqual(payload["base_link_pos"], payload["root_pos"])

    def test_log_spawn_pose_trace_omits_base_link_when_unresolvable(self) -> None:
        """The stock backend double has no ``base_link`` body name -- the
        trace must omit the field rather than fail."""
        backend = _backend()
        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            backend._log_spawn_pose_trace(
                "after_first_step",
                root_pos=[0.0, 0.0, 0.0775],
                root_quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            )
        payload = json.loads(captured.getvalue().strip())
        self.assertNotIn("base_link_pos", payload)


if __name__ == "__main__":
    unittest.main()
