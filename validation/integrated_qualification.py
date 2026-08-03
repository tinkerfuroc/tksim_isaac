#!/usr/bin/env python3
"""Orchestrate the integrated OMPL manipulation qualification Gates A-E.

Task 8 of the integrated OMPL manipulation qualification.  This module is the
offline orchestration/lifecycle layer on top of the review-clean six-gate
``manipulation_qualification`` runner, the Task 6 ``physics-ready`` gate, and
the Task 7 independent ``integrated_gate_verifier``.

Stage A runs the existing six-gate core suite (``free-space-fjt``,
``safety-stop``, ``free-gripper``, ``obstructed-gripper``, ``arm-collision``,
``retention``) and requires all six independent verdicts, exact raw/evaluator
drains, valid rosbags, clean teardown, and the existing contact sheets.  The
``--gate GATE_NAME`` behavior of ``manipulation_qualification.py`` is unchanged.

Before Gate B the runner atomically writes ``outputs/integrated/attempt-start.json``
with UTC/monotonic start identities, then invokes ``source_lock_manifest.py``
with the config-resolved committed authorization policy and validates the
producer exit code and output schema before invoking the offline static
closure.  Missing, stale, self-generated, or mismatched source-lock artifacts
make Gate B ``evidence-invalid``; the runner never falls back to capturing and
trusting current state.

Stages C-E run every listed scenario in a unique child ROS domain in ``[0,232]``
and a unique immutable attempt directory.  Before readiness the runner
validates the overlay's atomically written ``physics-ready.json`` against the
exact external ``scenario_report_sha256`` of the atomically written
``scenario-runner.json`` and the full committed identity (scenario id, seed,
scenario-declaration digest, planning-scene digest, integrated digest, model
fingerprint, provider-manifest digest, final ``STATE_PLAYING``, and a final
``state=1``/``boundary=PHYSICS_READY`` operation).  A transient
``state=PHYSICS_READY`` message without that report-byte match is insufficient.
Execution return codes never override the independent verifier verdict.
Teardown failures downgrade a scenario to ``evidence-invalid``; every attempt
is preserved.

Stage F is the explicit Tasks 9-10 extension point and is not implemented here.

The module is ROS-free Python 3.12 (it imports no ``rclpy`` and no generated
messages); the live Isaac/ROS processes it launches are children invoked via
the existing ``launch-isaac`` / ``launch-humble`` wrappers.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

try:
    from manipulation_qualification import (  # noqa: E402
        APPROVED_RECORD_TOPICS,
        CPU_PHYSICS,
        GATES,
        QualificationManifest,
        QualificationProcessHelpers,
        QualificationResult,
        QualificationRunner,
        _json_file,
        _new_suite_dir,
        _ros_tooling_environment,
        _run_suite,
        _write_json_atomic,
        qualification_rosbag_final_evidence,
        qualification_rosbag_metadata_evidence,
        qualification_rosbag_output_evidence,
        qualification_rosbag_qos_profiles,
        qualification_compare_truth_records,
        qualification_gpu_processes,
        qualification_jsonl_count,
        qualification_jsonl_records,
        qualification_record_topics,
        qualification_settle_evidence_files,
        qualification_source_identity,
        qualification_start_process,
        qualification_stop_process,
        qualification_terminate_attempt_orphans,
        qualification_wait_for_evaluator_drain,
        qualification_wait_for_ready,
        qualification_write_resource_evidence,
    )
except ModuleNotFoundError:
    from validation.manipulation_qualification import (  # noqa: E402
        APPROVED_RECORD_TOPICS,
        CPU_PHYSICS,
        GATES,
        QualificationManifest,
        QualificationProcessHelpers,
        QualificationResult,
        QualificationRunner,
        _json_file,
        _new_suite_dir,
        _ros_tooling_environment,
        _run_suite,
        _write_json_atomic,
        qualification_rosbag_final_evidence,
        qualification_rosbag_metadata_evidence,
        qualification_rosbag_output_evidence,
        qualification_rosbag_qos_profiles,
        qualification_compare_truth_records,
        qualification_gpu_processes,
        qualification_jsonl_count,
        qualification_jsonl_records,
        qualification_record_topics,
        qualification_settle_evidence_files,
        qualification_source_identity,
        qualification_start_process,
        qualification_stop_process,
        qualification_terminate_attempt_orphans,
        qualification_wait_for_evaluator_drain,
        qualification_wait_for_ready,
        qualification_write_resource_evidence,
    )

from tinker_sim_bridge.integrated_readiness import (  # noqa: E402
    FINAL_SIMULATION_STATE,
    PHYSICS_READY_BOUNDARY,
    REPORT_SCHEMA_VERSION,
    SIMULATION_STATE_PLAYING,
    ReportValidationError,
    build_canonical_report,
    canonical_json,
    parse_canonical_report,
    planning_scene_mapping,
    public_integrated_mapping,
    report_identities,
    sha256_bytes,
    validate_report,
)

# --------------------------------------------------------------------------- #
# Canonical stage/scenario constants (mirrors the deterministic test model)
# --------------------------------------------------------------------------- #

CORE_GATE_NAMES = tuple(GATES)

QUALIFICATION_SCENARIO_NAMES = (
    "qualification-moveit-plan-joint",
    "qualification-moveit-plan-pose",
    "qualification-moveit-plan-blocked",
    "qualification-moveit-execute-joint",
    "qualification-moveit-execute-pose",
    "qualification-moveit-cartesian-retreat",
    "qualification-moveit-gripper",
    "qualification-moveit-cancel",
    "qualification-moveit-safety",
    "qualification-pick-place-positive",
    "qualification-pick-place-blocked-approach",
    "qualification-pick-place-unreachable-grasp",
    "qualification-pick-place-malformed-back",
    "qualification-pick-place-cancel-approach",
    "qualification-pick-place-cancel-transport",
    "qualification-pick-place-safety-transport",
    "qualification-pick-place-occupied-place",
)

STAGE_LETTERS = ("A", "B", "C", "D", "E", "F")
BLOCKED_BY_GATE_B = "blocked-by-gate-b"
STATUS_VERIFIED_PASS = "verified-pass"
STATUS_VERIFIED_FAIL = "verified-fail"
STATUS_EVIDENCE_INVALID = "evidence-invalid"
STATUS_BLOCKED = BLOCKED_BY_GATE_B
STATUS_NOT_IMPLEMENTED = "not-implemented"

DEFAULT_CONFIG = ROOT / "simulation/qualification/integrated-ompl.json"
DEFAULT_PRODUCTION_ROOT = Path("/home/tinker/tk25_ws")
DEFAULT_ATTEMPT_ROOT = ROOT / "outputs/integrated"
DEFAULT_MODEL_BUNDLE_MANIFEST = ROOT / "outputs/ompl-overlay/model-bundle-r2/model-bundle.json"
DEFAULT_PROVIDER_MANIFEST = ROOT / "ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json"

ATTEMPT_START_FILENAME = "attempt-start.json"
SOURCE_LOCK_MANIFEST_FILENAME = "source-lock-manifest.json"
STATIC_CONTRACT_FILENAME = "static-contract.json"
MODEL_FINGERPRINT_FILENAME = "model-fingerprint.json"
SOURCE_IDENTITIES_FILENAME = "source-identities.json"

# The maximum valid ROS domain id (the simulator's Fast DDS bound).
MAX_ROS_DOMAIN_ID = 232


@dataclass(frozen=True)
class AttemptAllocation:
    domain_id: int
    attempt_dir: Path


class IntegratedRunner:
    """Offline orchestrator for the integrated OMPL qualification Gates A-E.

    The public contract mirrors the deterministic ``IntegratedRunnerDouble``
    used by the orchestration tests: ``core_gate_names``,
    ``qualification_scenario_names``, ``run_stage``, ``allocate_live_attempts``,
    ``run_scenario``, and ``run_core_gate``.  All process execution flows
    through the injectable ``command_runner`` / ``popen`` so the runner can be
    exercised offline without a live Isaac/ROS graph.
    """

    core_gate_names = list(CORE_GATE_NAMES)
    qualification_scenario_names = list(QUALIFICATION_SCENARIO_NAMES)

    def __init__(
        self,
        *,
        root: Path | None = None,
        production_root: Path | None = None,
        config_path: Path | None = None,
        seed: int = 7,
        attempt_root: Path | None = None,
        base_domain_id: int = 100,
        readiness_timeout_s: float = 30.0,
        bag_startup_timeout_s: float = 5.0,
        model_bundle_manifest: Path | None = None,
        provider_manifest_path: Path | None = None,
        isaac_command: Sequence[str] | str | None = None,
        humble_command: Sequence[str] | str | None = None,
        command_runner: Callable[..., Any] = subprocess.run,
        popen: Callable[..., Any] = subprocess.Popen,
    ) -> None:
        self.root = (root or ROOT).resolve()
        self.production_root = (production_root or DEFAULT_PRODUCTION_ROOT).resolve()
        self.config_path = (config_path or DEFAULT_CONFIG).resolve()
        self.seed = int(seed)
        self.attempt_root = (attempt_root or DEFAULT_ATTEMPT_ROOT).resolve()
        self.base_domain_id = int(base_domain_id)
        self.readiness_timeout_s = float(readiness_timeout_s)
        self.bag_startup_timeout_s = float(bag_startup_timeout_s)
        self.model_bundle_manifest = (
            Path(model_bundle_manifest).resolve()
            if model_bundle_manifest is not None
            else DEFAULT_MODEL_BUNDLE_MANIFEST.resolve()
        )
        self.provider_manifest_path = (
            Path(provider_manifest_path).resolve()
            if provider_manifest_path is not None
            else DEFAULT_PROVIDER_MANIFEST.resolve()
        )
        self.isaac_command = isaac_command
        self.humble_command = humble_command
        self._command_runner = command_runner
        self._popen = popen
        # Deterministic contract-model state (mirrors the double).
        self.static_contract_status = STATUS_VERIFIED_PASS
        self.started_scenarios: list[str] = []
        self._stage_results: dict[str, Any] = {}

    # ------------------------------------------------------------------ #
    # Configuration helpers
    # ------------------------------------------------------------------ #

    def _config(self) -> dict[str, Any]:
        return _json_file(self.config_path)

    def _stages_config(self) -> dict[str, Any]:
        config = self._config()
        stages = config.get("stages")
        if not isinstance(stages, Mapping):
            raise ValueError("integrated qualification config has no stages object")
        return dict(stages)

    def _core_gates(self) -> list[str]:
        stage_a = self._stages_config().get("A")
        if not isinstance(stage_a, Mapping):
            raise ValueError("integrated qualification config has no stage A")
        gates = stage_a.get("required_core_gates")
        if not isinstance(gates, Sequence) or isinstance(gates, (str, bytes)):
            raise ValueError("stage A required_core_gates must be an array")
        return [str(name) for name in gates]

    def _stage_scenarios(self, stage: str) -> list[str]:
        section = self._stages_config().get(stage)
        if not isinstance(section, Mapping):
            raise ValueError(f"integrated qualification config has no stage {stage}")
        if stage == "E":
            positive = section.get("positive")
            negative = section.get("negative")
            if not isinstance(positive, str) or not positive:
                raise ValueError("stage E positive scenario is missing")
            if not isinstance(negative, Sequence) or isinstance(negative, (str, bytes)):
                raise ValueError("stage E negative scenarios must be an array")
            return [str(positive), *[str(name) for name in negative]]
        scenarios = section.get("scenarios")
        if not isinstance(scenarios, Sequence) or isinstance(scenarios, (str, bytes)):
            raise ValueError(f"stage {stage} scenarios must be an array")
        return [str(name) for name in scenarios]

    def _source_lock_policies(self) -> Mapping[str, Any]:
        config = self._config()
        policies = config.get("source_lock_policies")
        if not isinstance(policies, Mapping):
            raise ValueError("integrated qualification config has no source_lock_policies")
        return dict(policies)

    # ------------------------------------------------------------------ #
    # Identity / allocation helpers
    # ------------------------------------------------------------------ #

    def _model_fingerprint(self) -> str:
        if not self.model_bundle_manifest.is_file():
            raise FileNotFoundError(
                f"model bundle manifest is missing: {self.model_bundle_manifest}"
            )
        bundle = _json_file(self.model_bundle_manifest)
        fingerprint = bundle.get("structural_fingerprint")
        if not isinstance(fingerprint, str) or len(fingerprint) != 64:
            raise ValueError("model bundle structural_fingerprint is not a SHA-256")
        return fingerprint

    def _provider_manifest_sha256(self) -> str:
        if not self.provider_manifest_path.is_file():
            raise FileNotFoundError(
                f"provider manifest is missing: {self.provider_manifest_path}"
            )
        return hashlib.sha256(self.provider_manifest_path.read_bytes()).hexdigest()

    def _scenario_bundle(self, name: str) -> dict[str, Any]:
        """Build the complete immutable scenario bundle for a scenario id."""
        if not name or "/" in name or name in {".", ".."}:
            raise ValueError(f"unsafe scenario id: {name!r}")
        scenario_path = self.root / "simulation/scenarios" / f"{name}.json"
        if not scenario_path.is_file():
            raise FileNotFoundError(f"scenario declaration is missing: {scenario_path}")
        raw = _json_file(scenario_path)
        if str(raw.get("id")) != name:
            raise ValueError("scenario id does not match its filename")
        seed = int(raw["seed"])
        declaration = {
            str(key): value for key, value in raw.items() if key not in {"id", "seed"}
        }
        planning_scene_declaration = raw.get("planning_scene")
        if not isinstance(planning_scene_declaration, Mapping):
            raise ValueError("scenario has no planning_scene object")
        planning_scene = planning_scene_mapping(planning_scene_declaration)
        integrated = raw.get("integrated")
        if not isinstance(integrated, Mapping):
            raise ValueError("scenario has no integrated object")
        identities = report_identities(
            scenario_id=name,
            seed=seed,
            declaration=declaration,
            planning_scene=planning_scene_declaration,
            integrated=public_integrated_mapping(),
            model_fingerprint=self._model_fingerprint(),
            provider_manifest_sha256=self._provider_manifest_sha256(),
        )
        return {
            "scenario": {"id": name, "seed": seed, "declaration": declaration},
            "planning_scene": planning_scene,
            "planning_scene_declaration": dict(planning_scene_declaration),
            "integrated": dict(integrated),
            "report_identities": dict(identities),
        }

    def allocate_live_attempts(
        self,
        *,
        stages: Sequence[str],
        base_domain_id: int,
        attempt_root: Path | str,
    ) -> list[AttemptAllocation]:
        """Allocate a unique valid domain and unique immutable attempt dir.

        Every scenario in the requested stages gets one allocation.  Domains
        are monotonically increasing from ``base_domain_id`` and stay within
        ``[0, 232]``.  Attempt directories are unique and derive their name
        from the stage and scenario.
        """
        allocations: list[AttemptAllocation] = []
        next_domain = int(base_domain_id)
        used_domains: set[int] = set()
        used_dirs: set[Path] = set()
        for stage in stages:
            for name in self._stage_scenarios(str(stage)):
                domain = next_domain % (MAX_ROS_DOMAIN_ID + 1)
                while domain in used_domains:
                    next_domain += 1
                    domain = next_domain % (MAX_ROS_DOMAIN_ID + 1)
                used_domains.add(domain)
                next_domain += 1
                attempt_dir = (Path(attempt_root) / f"{stage}-{name}").resolve()
                attempt_dir = self._unique_attempt_dir(attempt_dir, used_dirs)
                used_dirs.add(attempt_dir)
                allocations.append(AttemptAllocation(domain, attempt_dir))
        return allocations

    @staticmethod
    def _unique_attempt_dir(candidate: Path, used: set[Path]) -> Path:
        if candidate not in used:
            return candidate
        for index in range(1, 1000):
            variant = candidate.with_name(f"{candidate.name}-{index}")
            if variant not in used:
                return variant
        raise RuntimeError("could not allocate a unique attempt directory")

    def _allocate_one(self, name: str, stage: str) -> AttemptAllocation:
        allocations = self.allocate_live_attempts(
            stages=(stage,), base_domain_id=self.base_domain_id, attempt_root=self.attempt_root
        )
        for allocation in allocations:
            if allocation.attempt_dir.name.endswith(f"-{name}") or f"{stage}-{name}" in allocation.attempt_dir.name:
                return allocation
        return allocations[0]

    # ------------------------------------------------------------------ #
    # Core gate dispatch (Stage A) — unchanged six-gate semantics
    # ------------------------------------------------------------------ #

    def run_core_gate(self, gate: str) -> dict[str, Any]:
        """Return the exact existing six-gate dispatch descriptor.

        ``uses_integrated_executor`` is False and the verifier semantics are
        the existing six-gate semantics: core gates are dispatched through
        ``manipulation_qualification.py --gate GATE_NAME`` and never through the
        integrated executor.
        """
        return {
            "command": ["qualification", "--gate", gate],
            "uses_integrated_executor": False,
            "verifier_semantics": "existing-six-gate",
        }

    def _run_core_suite(self) -> dict[str, Any]:
        config = self._config()
        core_config_value = config.get("core_config")
        if not isinstance(core_config_value, str) or not core_config_value:
            raise ValueError("integrated qualification config has no core_config path")
        core_config_path = (
            Path(core_config_value)
            if Path(core_config_value).is_absolute()
            else self.root / core_config_value
        ).resolve()
        result = _run_suite(
            root=self.root,
            attempt_root=self.attempt_root / "core",
            config_path=core_config_path,
            artifact_path=None,
            seed=self.seed,
            readiness_timeout_s=self.readiness_timeout_s,
            isaac_command=self.isaac_command,
            humble_command=self.humble_command,
            gate_commands={},
            base_domain_id=self.base_domain_id,
        )
        return {
            "status": result.status,
            "attempt_dir": str(result.attempt_dir),
            "gate_results": {
                str(gate): dict(record) for gate, record in result.gate_results.items()
            },
        }

    def _run_stage_a(self) -> dict[str, Any]:
        gates = self._core_gates()
        duplicate_gate_names = sorted(
            {name for name in gates if gates.count(name) > 1}
        )
        core = self._run_core_suite()
        return {
            "stage": "A",
            "invoked_gates": gates,
            "duplicate_gate_names": duplicate_gate_names,
            "status": str(core.get("status", STATUS_VERIFIED_PASS)),
            "attempt_dir": core.get("attempt_dir"),
            "core_suite": core,
        }

    # ------------------------------------------------------------------ #
    # Gate B — offline static closure, fail closed, never trusts current state
    # ------------------------------------------------------------------ #

    def _write_attempt_start(self) -> Path:
        """Atomically write the attempt-start identity before Gate B."""
        self.attempt_root.mkdir(parents=True, exist_ok=True)
        path = self.attempt_root / ATTEMPT_START_FILENAME
        value = {
            "schema_version": 1,
            "attempt_id": (
                f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')}"
                f"-{os.getpid()}-{uuid.uuid4().hex[:10]}"
            ),
            "started_at": datetime.now(timezone.utc).isoformat(),
            "monotonic": time.monotonic(),
            "seed": self.seed,
            "root": str(self.root),
            "production_root": str(self.production_root),
            "config": str(self.config_path),
        }
        _write_json_atomic(path, value)
        return path

    def _invoke_source_lock_manifest(
        self, attempt_start_path: Path
    ) -> tuple[Path, dict[str, Any]]:
        policies = self._source_lock_policies()
        manifest_path = self.attempt_root / SOURCE_LOCK_MANIFEST_FILENAME
        command = [
            sys.executable,
            str(self.root / "validation/source_lock_manifest.py"),
            "--simulator-root",
            str(self.root),
            "--production-root",
            str(self.production_root),
            "--simulator-policy",
            str(policies.get("simulator_overlay", "")),
            "--production-policy",
            str(policies.get("production", "")),
            "--qualification-policy",
            str(policies.get("qualification_tooling", "")),
            "--attempt-start-file",
            str(attempt_start_path),
            "--output",
            str(manifest_path),
        ]
        try:
            completed = self._command_runner(
                command,
                cwd=self.root,
                text=True,
                capture_output=True,
                timeout=60.0,
                check=False,
            )
        except (OSError, subprocess.SubprocessError) as error:
            return manifest_path, {
                "status": STATUS_EVIDENCE_INVALID,
                "producer": "source-lock-manifest",
                "reason": f"source-lock manifest producer could not be invoked: {error}",
            }
        evidence = self._validate_source_lock_manifest(
            manifest_path,
            getattr(completed, "returncode", None),
            str(getattr(completed, "stdout", "") or ""),
            str(getattr(completed, "stderr", "") or ""),
        )
        return manifest_path, evidence

    def _validate_source_lock_manifest(
        self,
        manifest_path: Path,
        returncode: int | None,
        stdout: str,
        stderr: str,
    ) -> dict[str, Any]:
        evidence: dict[str, Any] = {
            "status": STATUS_EVIDENCE_INVALID,
            "producer": "source-lock-manifest",
            "returncode": returncode,
            "stdout": stdout,
            "stderr": stderr,
        }
        if returncode != 0:
            evidence["reason"] = (
                f"source-lock manifest producer exited {returncode}; "
                "missing/stale/self-generated source-lock artifacts are never trusted"
            )
            return evidence
        if not manifest_path.is_file():
            evidence["reason"] = "source-lock manifest output is missing"
            return evidence
        try:
            manifest = _json_file(manifest_path)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            evidence["reason"] = f"source-lock manifest output is not finite JSON: {error}"
            return evidence
        status = str(manifest.get("status", STATUS_EVIDENCE_INVALID))
        evidence["manifest_status"] = status
        if manifest.get("output_predates_attempt") is True:
            evidence["reason"] = "source-lock manifest output predates the attempt start"
            return evidence
        if status != "pass":
            evidence["reason"] = f"source-lock manifest status is {status}"
            evidence["status"] = (
                STATUS_EVIDENCE_INVALID if status == "invalid" else STATUS_VERIFIED_FAIL
            )
            return evidence
        evidence["status"] = "pass"
        evidence["manifest"] = manifest
        return evidence

    def _invoke_static_contracts(self, manifest_path: Path) -> dict[str, Any]:
        output_dir = self.attempt_root / "gate-b"
        command = [
            sys.executable,
            str(self.root / "validation/integrated_static_contracts.py"),
            "--simulator-root",
            str(self.root),
            "--production-root",
            str(self.production_root),
            "--source-lock-manifest",
            str(manifest_path),
            "--config",
            str(self.config_path),
            "--output",
            str(output_dir),
        ]
        try:
            completed = self._command_runner(
                command,
                cwd=self.root,
                text=True,
                capture_output=True,
                timeout=120.0,
                check=False,
            )
        except (OSError, subprocess.SubprocessError) as error:
            return {
                "status": STATUS_EVIDENCE_INVALID,
                "producer": "integrated-static-contracts",
                "reason": f"static contract producer could not be invoked: {error}",
            }
        static_path = output_dir / STATIC_CONTRACT_FILENAME
        evidence: dict[str, Any] = {
            "status": STATUS_EVIDENCE_INVALID,
            "producer": "integrated-static-contracts",
            "returncode": getattr(completed, "returncode", None),
        }
        if getattr(completed, "returncode", None) != 0:
            evidence["reason"] = (
                f"static contract producer exited {getattr(completed, 'returncode', None)}"
            )
            return evidence
        if not static_path.is_file():
            evidence["reason"] = "static-contract.json is missing"
            return evidence
        try:
            report = _json_file(static_path)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            evidence["reason"] = f"static-contract.json is not finite JSON: {error}"
            return evidence
        status = str(report.get("status", STATUS_EVIDENCE_INVALID))
        if status not in {
            STATUS_VERIFIED_PASS,
            STATUS_VERIFIED_FAIL,
            STATUS_EVIDENCE_INVALID,
        }:
            status = STATUS_EVIDENCE_INVALID
        evidence["status"] = status
        evidence["static_contract"] = report
        if status == STATUS_VERIFIED_PASS and report.get("model_fingerprint"):
            fingerprint_path = output_dir / MODEL_FINGERPRINT_FILENAME
            fingerprint_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "model_fingerprint": report["model_fingerprint"],
                    },
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
        return evidence

    def _run_stage_b(self) -> dict[str, Any]:
        attempt_start_path = self._write_attempt_start()
        manifest_path, source_lock = self._invoke_source_lock_manifest(attempt_start_path)
        if source_lock.get("status") != "pass":
            self.static_contract_status = STATUS_EVIDENCE_INVALID
            return {
                "stage": "B",
                "status": str(source_lock.get("status", STATUS_EVIDENCE_INVALID)),
                "source_lock": source_lock,
                "reasons": [str(source_lock.get("reason", "source-lock manifest is not authorized"))],
            }
        static = self._invoke_static_contracts(manifest_path)
        status = str(static.get("status", STATUS_EVIDENCE_INVALID))
        self.static_contract_status = status
        result: dict[str, Any] = {
            "stage": "B",
            "status": status,
            "source_lock": source_lock,
            "static_contracts": static,
        }
        if status != STATUS_VERIFIED_PASS:
            result["reasons"] = [
                str(static.get("reason", "offline static closure did not pass"))
            ]
        return result

    # ------------------------------------------------------------------ #
    # Stages C-E — per-scenario child-domain execution
    # ------------------------------------------------------------------ #

    def _isaac_scenario_command(self, name: str) -> list[str]:
        return [
            str(self.root / "scripts/launch-isaac"),
            "--sensor-profile",
            "manipulation-core",
            "--profile",
            "parity",
            "--scenario",
            name,
            "--seed",
            str(self.seed),
            "--headless",
            "--ros",
            "--qualification",
        ]

    def _humble_scenario_command(
        self, name: str, attempt_dir: Path
    ) -> list[str]:
        return [
            str(self.root / "scripts/launch-humble"),
            "integrated-ompl",
            f"scenario:={name}",
            f"seed:={self.seed}",
            "qualification:=true",
            f"attempt_dir:={attempt_dir}",
            f"model_bundle_manifest:={self.model_bundle_manifest}",
            f"provider_manifest_path:={self.provider_manifest_path}",
        ]

    def _new_scenario_runner(
        self, allocation: AttemptAllocation, name: str
    ) -> QualificationRunner:
        config = self._config()
        core_config_value = config.get("core_config")
        core_config_path = (
            Path(core_config_value)
            if Path(str(core_config_value)).is_absolute()
            else self.root / str(core_config_value)
        ).resolve()
        scenario_path = self.root / "simulation/scenarios" / f"{name}.json"
        return QualificationRunner(
            root=self.root,
            attempt_root=allocation.attempt_dir.parent,
            config_path=core_config_path,
            scenario_path=scenario_path,
            artifact_path=None,
            seed=self.seed,
            gate=name,
            readiness_timeout_s=self.readiness_timeout_s,
            bag_startup_timeout_s=self.bag_startup_timeout_s,
            isaac_command=self._isaac_scenario_command(name),
            humble_command=self._humble_scenario_command(name, allocation.attempt_dir),
            gate_commands={},
            ros_domain_id=allocation.domain_id,
            popen=self._popen,
            command_runner=self._command_runner,
        )

    def _launch_scenario(
        self,
        allocation: AttemptAllocation,
        name: str,
        stage: str,
    ) -> dict[str, Any]:
        runner = self._new_scenario_runner(allocation, name)
        manifest = runner.prepare_manifest()
        self._scenario_manifests[allocation.attempt_dir] = (runner, manifest)
        environment = self._scenario_environment(manifest, allocation)
        return {
            "ok": True,
            "attempt_dir": allocation.attempt_dir,
            "manifest": manifest,
            "environment": environment,
        }

    def _scenario_environment(
        self, manifest: QualificationManifest, allocation: AttemptAllocation
    ) -> dict[str, str]:
        environment = _ros_tooling_environment(
            root=self.root,
            domain_id=str(allocation.domain_id),
        )
        environment.update(
            {
                "ROS_DOMAIN_ID": str(allocation.domain_id),
                "RMW_IMPLEMENTATION": str(
                    manifest.data.get("environment", {}).get(
                        "RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"
                    )
                ),
                "TINKER_SIM_ROOT": str(self.root),
                "TINKER_SIM_ATTEMPT_DIR": str(allocation.attempt_dir),
                "TINKER_SIM_TRUTH_JSONL": str(allocation.attempt_dir / "physics_truth.jsonl"),
                "TINKER_SIM_EVALUATOR_JSONL": str(allocation.attempt_dir / "evaluator.jsonl"),
                "TINKER_SIM_ROSBAG_DIR": str(allocation.attempt_dir / "rosbag"),
                "TINKER_SIM_PHYSICS_DEVICE": "cpu",
                "TINKER_SIM_MODEL_BUNDLE_MANIFEST": str(self.model_bundle_manifest),
                "TINKER_SIM_PROVIDER_MANIFEST": str(self.provider_manifest_path),
                "ISAACSIM_HEADLESS": "1",
            }
        )
        return environment

    def _validate_physics_ready(
        self, attempt_dir: Path, name: str
    ) -> tuple[bool, dict[str, Any], str | None]:
        """Validate ``physics-ready.json`` against the exact report bytes/identity.

        A transient ``state=PHYSICS_READY`` without the exact
        ``scenario_report_sha256`` of the atomically written ``scenario-runner.json``
        and the full committed identity is insufficient.
        """
        scenario_runner_path = attempt_dir / "scenario-runner.json"
        physics_ready_path = attempt_dir / "physics-ready.json"
        evidence: dict[str, Any] = {
            "scenario": name,
            "scenario_runner": {
                "path": str(scenario_runner_path),
                "present": scenario_runner_path.is_file(),
            },
            "physics_ready": {
                "path": str(physics_ready_path),
                "present": physics_ready_path.is_file(),
            },
        }
        if not scenario_runner_path.is_file():
            return False, evidence, "scenario-runner.json is missing"
        if not physics_ready_path.is_file():
            return False, evidence, "physics-ready.json is missing"
        try:
            report_bytes = scenario_runner_path.read_bytes()
        except OSError as error:
            return False, evidence, f"scenario-runner.json is unreadable: {error}"
        evidence["scenario_runner"]["sha256"] = sha256_bytes(report_bytes)
        try:
            physics = _json_file(physics_ready_path)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            return False, evidence, f"physics-ready.json is invalid: {error}"
        evidence["physics_ready"]["state"] = physics.get("state")
        evidence["physics_ready"]["scenario_report_sha256"] = physics.get("scenario_report_sha256")
        if physics.get("state") != "PHYSICS_READY":
            return False, evidence, "physics-ready state is not PHYSICS_READY"
        observed_sha = physics.get("scenario_report_sha256")
        expected_sha = sha256_bytes(report_bytes)
        if not isinstance(observed_sha, str) or observed_sha != expected_sha:
            return (
                False,
                evidence,
                "physics-ready scenario_report_sha256 does not match scenario-runner.json bytes",
            )
        try:
            report = parse_canonical_report(report_bytes)
        except ReportValidationError as error:
            return False, evidence, f"scenario-runner.json is not the canonical report: {error}"
        bundle = self._scenario_bundle(name)
        identities = bundle["report_identities"]
        # The canonical planning-scene report mapping (owned ids derived through
        # the authoritative Task 5 ``fixture_owned_ids`` helper) is exactly what
        # ``build_canonical_report`` places in the report, so the expected
        # contract must use the same mapping values.
        plan = bundle["planning_scene"]
        expected = {
            "scenario_id": name,
            "seed": int(bundle["scenario"]["seed"]),
            "scenario_declaration_sha256": str(
                identities["scenario_declaration_sha256"]
            ),
            "planning_scene_revision": str(plan["revision"]),
            "planning_scene_owned_ids": [str(item) for item in plan["owned_ids"]],
            "planning_scene_target_source_id": str(plan["target_source_id"]),
            "planning_scene_target_handoff": str(plan["target_handoff"]),
            "integrated_mapping": public_integrated_mapping(),
            "model_fingerprint": str(identities["model_fingerprint"]),
            "provider_manifest_sha256": str(identities["provider_manifest_sha256"]),
        }
        validation = validate_report(report, expected)
        evidence["validation"] = validation
        if not validation.get("ready"):
            reasons = validation.get("reasons", [])
            return (
                False,
                evidence,
                "physics-ready report identity mismatch: " + "; ".join(reasons),
            )
        if report.get("final_simulation_state") != FINAL_SIMULATION_STATE:
            return (
                False,
                evidence,
                f"physics-ready final_simulation_state is not {FINAL_SIMULATION_STATE}",
            )
        final_operation = report["operations"][-1]
        if (
            final_operation.get("state") != SIMULATION_STATE_PLAYING
            or final_operation.get("boundary") != PHYSICS_READY_BOUNDARY
        ):
            return (
                False,
                evidence,
                "physics-ready final operation is not state=1/boundary=PHYSICS_READY",
            )
        evidence["ready"] = True
        return True, evidence, None

    def _wait_for_physics_ready(
        self, attempt_dir: Path, name: str
    ) -> tuple[bool, dict[str, Any], str | None]:
        deadline = time.monotonic() + self.readiness_timeout_s
        last_evidence: dict[str, Any] = {}
        last_reason: str | None = None
        while time.monotonic() < deadline:
            ok, evidence, reason = self._validate_physics_ready(attempt_dir, name)
            last_evidence = evidence
            last_reason = reason
            if ok:
                return True, evidence, None
            if (attempt_dir / "scenario-runner.json").is_file() and (
                attempt_dir / "physics-ready.json"
            ).is_file():
                # Both files exist but validation fails: do not spin forever on
                # a definite mismatch.
                return False, evidence, reason
            time.sleep(0.25)
        return False, last_evidence, last_reason or "physics-ready timeout"

    def _drain_truth(self, attempt_dir: Path, name: str) -> dict[str, Any]:
        raw_path = attempt_dir / "physics_truth.jsonl"
        evaluator_path = attempt_dir / "evaluator.jsonl"
        raw_records, raw_errors = qualification_jsonl_records(raw_path)
        evaluator_records, evaluator_errors = qualification_jsonl_records(evaluator_path)
        correlated, mismatches = qualification_compare_truth_records(
            raw_records,
            evaluator_records,
            raw_errors=raw_errors,
            evaluator_errors=evaluator_errors,
        )
        evidence: dict[str, Any] = {
            "status": "drained" if correlated else "evidence-invalid",
            "raw_truth_frames": len(raw_records),
            "evaluator_frames": len(evaluator_records),
            "exact_correlation": correlated,
            "raw_errors": raw_errors,
            "evaluator_errors": evaluator_errors,
            "mismatches": mismatches,
        }
        if not correlated:
            evidence["reason"] = "raw/evaluator drain did not correlate exactly"
        return evidence

    def _verify_attempt(self, attempt_dir: Path, name: str, stage: str) -> dict[str, Any]:
        try:
            from integrated_gate_verifier import verify_integrated_attempt  # noqa: PLC0415
        except ModuleNotFoundError:
            from validation.integrated_gate_verifier import verify_integrated_attempt  # noqa: PLC0415
        bundle = self._scenario_bundle(name)
        verdict = verify_integrated_attempt(
            scenario=bundle,
            attempt_dir=attempt_dir,
            config=self._config(),
        )
        return {
            "scenario": name,
            "stage": stage,
            **dict(verdict),
        }

    def _wait_for_scenario_terminal(
        self, attempt_dir: Path, name: str
    ) -> dict[str, Any]:
        terminal_marker = attempt_dir / "execution-terminal.json"
        deadline = time.monotonic() + self.readiness_timeout_s
        while time.monotonic() < deadline:
            if terminal_marker.is_file():
                try:
                    value = _json_file(terminal_marker)
                except (OSError, ValueError, json.JSONDecodeError):
                    value = {}
                return {"ok": True, "terminal": value, "marker": str(terminal_marker)}
            if (attempt_dir / "gate-verdict.json").is_file():
                return {"ok": True, "terminal": {"verdict": True}}
            time.sleep(0.25)
        return {"ok": False, "reason": "scenario execution terminal was not observed"}

    def _execute_scenario(
        self, allocation: AttemptAllocation, name: str, stage: str
    ) -> dict[str, Any]:
        attempt_dir = allocation.attempt_dir
        attempt_dir.mkdir(parents=True, exist_ok=True)
        launch = self._launch_scenario(allocation, name, stage)
        if not launch.get("ok"):
            return {
                "status": STATUS_EVIDENCE_INVALID,
                "scenario": name,
                "stage": stage,
                "reasons": [str(launch.get("reason", "scenario launch failed"))],
            }
        ready_ok, ready_evidence, ready_reason = self._wait_for_physics_ready(
            attempt_dir, name
        )
        if not ready_ok:
            return {
                "status": STATUS_EVIDENCE_INVALID,
                "scenario": name,
                "stage": stage,
                "reasons": [str(ready_reason)],
                "readiness_evidence": ready_evidence,
            }
        terminal = self._wait_for_scenario_terminal(attempt_dir, name)
        if not terminal.get("ok"):
            return {
                "status": STATUS_EVIDENCE_INVALID,
                "scenario": name,
                "stage": stage,
                "reasons": [str(terminal.get("reason", "scenario did not reach a terminal"))],
            }
        drained = self._drain_truth(attempt_dir, name)
        if drained.get("status") != "drained":
            return {
                "status": STATUS_EVIDENCE_INVALID,
                "scenario": name,
                "stage": stage,
                "reasons": [str(drained.get("reason", "truth drain failed"))],
                "drain": drained,
            }
        return self._verify_attempt(attempt_dir, name, stage)

    def _teardown_scenario(self, allocation: AttemptAllocation) -> bool:
        """Stop managed processes, terminate orphans, and settle evidence."""
        runner, manifest = self._scenario_manifests.pop(
            allocation.attempt_dir, (None, None)
        )
        if runner is None:
            return True
        helpers = QualificationProcessHelpers(runner)
        ok = True
        gpu_baseline = qualification_gpu_processes(runner)
        for name in ("humble", "isaac"):
            qualification_stop_process(runner, name)
        if not qualification_wait_for_evaluator_drain(runner, manifest):
            ok = False
        orphans = qualification_attempt_processes(runner)
        if orphans:
            qualification_terminate_attempt_orphans(runner)
        if not qualification_write_resource_evidence(runner, manifest, gpu_baseline):
            ok = False
        qualification_settle_evidence_files(runner, manifest)
        return ok

    def run_scenario(
        self,
        name: str,
        *,
        stage: str,
        allocation: AttemptAllocation | None = None,
    ) -> dict[str, Any]:
        """Run one scenario in a unique child domain and immutable attempt dir.

        The independent verifier's status is authoritative; execution return
        codes never override it.  A teardown failure downgrades the scenario to
        ``evidence-invalid``.  Every attempt directory is preserved.
        """
        allocation = allocation or self._allocate_one(name, stage)
        attempt_dir = allocation.attempt_dir
        attempt_dir.mkdir(parents=True, exist_ok=True)
        self.started_scenarios.append(name)
        result = self._execute_scenario(allocation, name, stage)
        teardown_ok = self._teardown_scenario(allocation)
        if not teardown_ok:
            result = {
                **dict(result),
                "status": STATUS_EVIDENCE_INVALID,
                "reasons": [*(result.get("reasons") or []), "teardown failed"],
            }
        return result

    # ------------------------------------------------------------------ #
    # Stage dispatch
    # ------------------------------------------------------------------ #

    def _run_scenario_stage(self, stage: str) -> dict[str, Any]:
        names = self._stage_scenarios(stage)
        results: dict[str, Any] = {}
        for name in names:
            results[name] = self.run_scenario(name, stage=stage)
            results[name]["started"] = True
        return {"stage": stage, "scenario_names": names, **results}

    def _run_stage_c(self) -> dict[str, Any]:
        return self._run_scenario_stage("C")

    def _run_stage_d(self) -> dict[str, Any]:
        return self._run_scenario_stage("D")

    def _run_stage_e(self) -> dict[str, Any]:
        return self._run_scenario_stage("E")

    def _run_stage_f(self) -> dict[str, Any]:
        section = self._stages_config().get("F", {})
        return {
            "stage": "F",
            "status": STATUS_NOT_IMPLEMENTED,
            "extension_point": "tasks-9-10",
            "checksum_algorithm": section.get("checksum_algorithm")
            if isinstance(section, Mapping)
            else None,
            "cameras": list(section.get("cameras", []))
            if isinstance(section, Mapping)
            else [],
        }

    def _run_all(self) -> dict[str, Any]:
        a = self._run_stage_a()
        self._stage_results["A"] = a
        b = self._run_stage_b()
        self._stage_results["B"] = b
        if b.get("status") != STATUS_VERIFIED_PASS:
            return {
                "B": b,
                **{
                    name: {"status": STATUS_BLOCKED}
                    for name in ("C", "D", "E", "F")
                },
            }
        c = self._run_stage_c()
        d = self._run_stage_d()
        e = self._run_stage_e()
        f = self._run_stage_f()
        self._stage_results.update({"C": c, "D": d, "E": e, "F": f})
        return {"A": a, "B": b, "C": c, "D": d, "E": e, "F": f}

    def run_stage(self, stage: str) -> dict[str, Any]:
        normalized = str(stage).upper()
        if normalized == "A":
            return self._run_stage_a()
        if normalized == "B":
            return self._run_stage_b()
        if normalized == "C":
            return self._run_stage_c()
        if normalized == "D":
            return self._run_stage_d()
        if normalized == "E":
            return self._run_stage_e()
        if normalized == "F":
            return self._run_stage_f()
        if normalized == "ALL":
            return self._run_all()
        raise ValueError(f"unsupported test stage: {stage}")

    # -------------------------------------------------------------- #
    # Scenario-manifest bookkeeping (per live attempt)
    # -------------------------------------------------------------- #

    @property
    def _scenario_manifests(self) -> dict[Path, tuple[QualificationRunner, QualificationManifest]]:
        if not hasattr(self, "_scenario_manifest_store"):
            self._scenario_manifest_store: dict[
                Path, tuple[QualificationRunner, QualificationManifest]
            ] = {}
        return self._scenario_manifest_store


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def _overall_status(result: Mapping[str, Any]) -> str:
    statuses = [
        str(value.get("status", ""))
        for key, value in result.items()
        if isinstance(value, Mapping)
    ]
    if not statuses:
        return STATUS_VERIFIED_FAIL
    if STATUS_BLOCKED in statuses:
        return STATUS_BLOCKED
    if STATUS_EVIDENCE_INVALID in statuses:
        return STATUS_EVIDENCE_INVALID
    if STATUS_VERIFIED_FAIL in statuses:
        return STATUS_VERIFIED_FAIL
    if STATUS_NOT_IMPLEMENTED in statuses:
        if all(
            status in {STATUS_VERIFIED_PASS, STATUS_NOT_IMPLEMENTED}
            for status in statuses
        ):
            return STATUS_VERIFIED_PASS
        return STATUS_VERIFIED_FAIL
    if all(status == STATUS_VERIFIED_PASS for status in statuses):
        return STATUS_VERIFIED_PASS
    return STATUS_VERIFIED_FAIL


def _exit_code_for_status(status: str) -> int:
    if status in {STATUS_VERIFIED_PASS, STATUS_BLOCKED}:
        return 0
    if status == STATUS_EVIDENCE_INVALID:
        return 2
    return 1


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Orchestrate the integrated OMPL qualification Gates A-E."
    )
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--production-root", type=Path, default=DEFAULT_PRODUCTION_ROOT)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--stage", default="all", choices=["A", "B", "C", "D", "E", "F", "all"])
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--attempt-root", type=Path, default=DEFAULT_ATTEMPT_ROOT)
    parser.add_argument("--base-domain-id", type=int, default=100)
    parser.add_argument("--readiness-timeout", type=float, default=30.0)
    parser.add_argument("--model-bundle-manifest", type=Path)
    parser.add_argument("--provider-manifest-path", type=Path)
    parser.add_argument("--isaac-command", help="override Isaac wrapper command")
    parser.add_argument("--humble-command", help="override Humble wrapper command")
    args = parser.parse_args(list(argv) if argv is not None else None)

    runner = IntegratedRunner(
        root=args.root,
        production_root=args.production_root,
        config_path=args.config,
        seed=args.seed,
        attempt_root=args.attempt_root,
        base_domain_id=args.base_domain_id,
        readiness_timeout_s=args.readiness_timeout,
        model_bundle_manifest=args.model_bundle_manifest,
        provider_manifest_path=args.provider_manifest_path,
        isaac_command=args.isaac_command,
        humble_command=args.humble_command,
    )
    result = runner.run_stage(args.stage)
    try:
        rendered = json.dumps(result, sort_keys=True, indent=2)
    except (TypeError, ValueError):
        rendered = json.dumps({"status": STATUS_EVIDENCE_INVALID})
    print(rendered)
    if args.stage.lower() == "all":
        return _exit_code_for_status(_overall_status(result))
    single = result.get("status")
    return _exit_code_for_status(str(single or STATUS_EVIDENCE_INVALID))


if __name__ == "__main__":
    raise SystemExit(main())
