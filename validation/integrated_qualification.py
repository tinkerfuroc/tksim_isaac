#!/usr/bin/env python3
"""Orchestrate the integrated OMPL manipulation qualification Gates A-F.

Task 8 of the integrated OMPL manipulation qualification (fix round 1).  This
module is the offline orchestration/lifecycle layer on top of the review-clean
six-gate ``manipulation_qualification`` runner, the Task 6 ``physics-ready``
gate, and the Task 7 independent ``integrated_gate_verifier``.

Stage A runs the existing six-gate core suite (``free-space-fjt``,
``safety-stop``, ``free-gripper``, ``obstructed-gripper``, ``arm-collision``,
``retention``) and requires all six independent verdicts, exact raw/evaluator
drains, valid rosbags, clean teardown, and the existing contact sheets.  Before
reporting Stage A the orchestrator requires the integrated config's
``required_core_gates`` to equal the core config gate list exactly (order and
uniqueness); drift is ``evidence-invalid``.

Before Gate B the runner atomically writes a fresh per-invocation
``attempt-start.json`` with UTC/monotonic start identities, then invokes
``source_lock_manifest.py`` with the config-resolved committed authorization
policy and validates the producer exit code, output schema, and output
freshness before invoking the offline static closure.  Missing, stale,
self-generated, mismatched, or ``fail`` source-lock artifacts all make Gate B
``evidence-invalid``; the runner never falls back to capturing and trusting
current state.  Gate B evidence is written to a fresh per-invocation directory
bound to the attempt start, and its ``model-fingerprint.json`` is cross-bound to
the same runtime model fingerprint consumed by C-E readiness/verification.

Stages C-E run every listed scenario in a unique child ROS domain in ``[0,232]``
and a unique immutable attempt directory that is freshly created for the
invocation (never reused, never a stale pre-existing directory).  The configured
Isaac and Humble child commands are launched through the reusable
``QualificationRunner`` lifecycle with the exact scenario id, seed, attempt
directory, private domain, and RMW/DDS environment applied to the actual
subprocesses.  Before readiness the runner validates the overlay's atomically
written ``physics-ready.json`` against the exact external
``scenario_report_sha256`` of the atomically written ``scenario-runner.json``,
the full committed identity (scenario id, seed, scenario-declaration digest,
planning-scene digest, integrated digest, model fingerprint, provider-manifest
digest, final ``STATE_PLAYING``, and a final ``state=1``/``boundary=PHYSICS_READY``
operation), and the current-attempt manifest provenance.  A transient
``state=PHYSICS_READY`` message without that report-byte match is insufficient.

For each scenario the producers are stopped and the evaluator/raw drain is
required to correlate exactly before ``verify_integrated_attempt`` runs; rosbag,
cleanup, and resource evidence are finalized before verification.  Execution
return codes never override the independent verifier verdict, and a successful
verifier never overrides failed teardown/drain/bag/resource evidence.  Teardown
failures downgrade a scenario to ``evidence-invalid``; every attempt is
preserved; a malformed scenario fails closed without skipping later controls.

Stage F closes the reproducibility/visual-evidence gate: it validates the
persisted A-E stage records, then regenerates the evidence index, both
integrated contact sheets, and the qualification summary through the Task 9
producers, returning the validator verdict.  Standalone ``--stage F`` is pure
offline verification and never launches live processes.

The module is ROS-free Python 3.12 (it imports no ``rclpy`` and no generated
messages); the live Isaac/ROS processes it launches are children invoked via
the existing ``launch-isaac`` / ``launch-humble`` wrappers.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

ROOT = Path(__file__).resolve().parents[1]
# ROOT itself is required so the Task-9 producers (``integrated_evidence_index``
# imports ``simulation.tinker_sim_isaac.qualification_visual_capture``) resolve
# under every invocation form, including ``python validation/...py`` where the
# script directory, not ROOT, is the first sys.path entry.
sys.path.insert(0, str(ROOT))
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
        qualification_gpu_processes,
        qualification_jsonl_records,
        qualification_rosbag_final_evidence,
        qualification_rosbag_metadata_evidence,
        qualification_rosbag_output_evidence,
        qualification_rosbag_qos_profiles,
        qualification_attempt_processes,
        qualification_compare_truth_records,
        qualification_orphan_failure,
        qualification_record_topics,
        qualification_settle_evidence_files,
        qualification_start_process,
        qualification_stop_process,
        qualification_terminate_attempt_orphans,
        qualification_wait_for_evaluator_drain,
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
        qualification_gpu_processes,
        qualification_jsonl_records,
        qualification_rosbag_final_evidence,
        qualification_rosbag_metadata_evidence,
        qualification_rosbag_output_evidence,
        qualification_rosbag_qos_profiles,
        qualification_attempt_processes,
        qualification_compare_truth_records,
        qualification_orphan_failure,
        qualification_record_topics,
        qualification_settle_evidence_files,
        qualification_start_process,
        qualification_stop_process,
        qualification_terminate_attempt_orphans,
        qualification_wait_for_evaluator_drain,
        qualification_write_resource_evidence,
    )

from tinker_sim_bridge.integrated_readiness import (  # noqa: E402
    FINAL_SIMULATION_STATE,
    PHYSICS_READY_BOUNDARY,
    SIMULATION_STATE_PLAYING,
    ReportValidationError,
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

# Write-once, atomic per-stage records persisted under the integrated suite.
# Stage F is the only repeatable stage and regenerates only derived outputs.
STAGE_RECORD_FILENAMES = {
    "A": "stage-a-result.json",
    "B": "stage-b-result.json",
    "C": "stage-c-result.json",
    "D": "stage-d-result.json",
    "E": "stage-e-result.json",
}

# The six-gate Stage-A core suite lives outside the integrated Gate-F index in
# the sibling ``<suite>-core`` root (C1).
CORE_SUITE_DIRNAME_SUFFIX = "-core"

# Derived Stage-F outputs (the only artifacts F may regenerate).
INDEX_NAME = "evidence-index.json"
SUMMARY_NAME = "qualification-summary.json"
AGENT_SHEET_NAME = "contact-sheet-integrated-agent.png"
USER_SHEET_NAME = "contact-sheet-integrated-user.png"

DEFAULT_CONFIG = ROOT / "simulation/qualification/integrated-ompl.json"
DEFAULT_PRODUCTION_ROOT = Path("/home/tinker/tk25_ws/src/tk25_manipulation")
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

# The scenario terminal markers the executor/driver produce as durable scenario
# completion evidence.  ``execution-terminal.json`` is the authoritative
# cross-bound marker written by the source-run executor driver (F2.2) and is what
# ``_wait_for_scenario_terminal`` requires; ``integrated-execution.json`` is the
# executor's own terminal summary, written just before the driver marker.  The
# orchestrator never waits for its own verifier's ``gate-verdict.json``.
TERMINAL_EVIDENCE_FILENAMES = ("execution-terminal.json", "integrated-execution.json")


@dataclass(frozen=True)
class AttemptAllocation:
    domain_id: int
    attempt_dir: Path


def _reduce_scenario_statuses(statuses: Sequence[str]) -> str:
    """Fail-dominant stage status reduction over per-scenario verdicts.

    ``evidence-invalid`` > ``verified-fail`` > ``verified-pass``.  Any missing,
    empty, or malformed status is ``evidence-invalid``.
    """
    normalized = [str(status) for status in statuses]
    if not normalized:
        return STATUS_EVIDENCE_INVALID
    if STATUS_EVIDENCE_INVALID in normalized:
        return STATUS_EVIDENCE_INVALID
    if STATUS_VERIFIED_FAIL in normalized:
        return STATUS_VERIFIED_FAIL
    if all(status == STATUS_VERIFIED_PASS for status in normalized):
        return STATUS_VERIFIED_PASS
    return STATUS_EVIDENCE_INVALID


class IntegratedRunner:
    """Offline orchestrator for the integrated OMPL qualification Gates A-F.

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
        terminal_timeout_s: float | None = None,
        bag_startup_timeout_s: float = 5.0,
        model_bundle_manifest: Path | None = None,
        provider_manifest_path: Path | None = None,
        isaac_command: Sequence[str] | str | None = None,
        humble_command: Sequence[str] | str | None = None,
        command_runner: Callable[..., Any] = subprocess.run,
        popen: Callable[..., Any] = subprocess.Popen,
    ) -> None:
        self.root = Path(root or ROOT).resolve()
        self.production_root = Path(production_root or DEFAULT_PRODUCTION_ROOT).resolve()
        self.config_path = Path(config_path or DEFAULT_CONFIG).resolve()
        self.seed = int(seed)
        self.attempt_root = Path(attempt_root or DEFAULT_ATTEMPT_ROOT).resolve()
        self.base_domain_id = int(base_domain_id)
        self.readiness_timeout_s = float(readiness_timeout_s)
        # F2.5: the scenario terminal budget is derived from committed config
        # thresholds (305.0 s for the current integrated-ompl config) and is
        # deliberately separate from the physics-readiness budget.  A
        # constructor/CLI override is accepted for deterministic tests, but the
        # normal CLI/config path uses the derivation and never undercuts the
        # executor's sequential run-method deadlines.
        self.terminal_timeout_s = (
            float(terminal_timeout_s)
            if terminal_timeout_s is not None
            else self._derived_terminal_timeout()
        )
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
        # Collision-proof fresh attempt allocation state (F1.2): each live
        # scenario invocation gets a directory that did not previously exist.
        self._run_invocation_id = uuid.uuid4().hex[:8]
        self._attempt_counter = 0
        # Per-attempt lifecycle bookkeeping.
        self._scenario_manifest_store: dict[
            Path, tuple[QualificationRunner, QualificationManifest]
        ] = {}
        self._gpu_baselines: dict[Path, dict[str, Any]] = {}

    # ------------------------------------------------------------------ #
    # Configuration helpers
    # ------------------------------------------------------------------ #

    def _config(self) -> dict[str, Any]:
        return _json_file(self.config_path)

    def _derived_terminal_timeout(self) -> float:
        """Derive the scenario terminal budget from committed config thresholds.

        ``plan_timeout_s + 2*execute_timeout_s + cancel_timeout_s +
        scene_timeout_s + max(cancel_timeout_s, 30.0)`` = exactly ``305.0`` for
        the current integrated-ompl thresholds (15/120/10/10), covering the
        source-inspected worst E transport path (~275 s) and D gripper path
        (240 s).  Malformed config fails closed.
        """
        config = self._config()
        thresholds = config.get("thresholds")
        if not isinstance(thresholds, Mapping):
            raise ValueError("integrated config has no thresholds object")
        try:
            terms = {
                key: float(thresholds[key])
                for key in (
                    "plan_timeout_s",
                    "execute_timeout_s",
                    "cancel_timeout_s",
                    "scene_timeout_s",
                )
            }
        except (TypeError, ValueError, KeyError) as error:
            raise ValueError(f"integrated config terminal thresholds are malformed: {error}") from error
        for key, value in terms.items():
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"integrated config {key} must be finite and positive")
        cancel = terms["cancel_timeout_s"]
        settle = max(cancel, 30.0)
        return (
            terms["plan_timeout_s"]
            + 2.0 * terms["execute_timeout_s"]
            + terms["cancel_timeout_s"]
            + terms["scene_timeout_s"]
            + settle
        )

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

    def _core_config_path(self) -> Path:
        config = self._config()
        core_config_value = config.get("core_config")
        if not isinstance(core_config_value, str) or not core_config_value:
            raise ValueError("integrated qualification config has no core_config path")
        return (
            Path(core_config_value)
            if Path(core_config_value).is_absolute()
            else self.root / core_config_value
        ).resolve()

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

    def _allocate_attempt_dir(
        self, attempt_root: Path, stage: str, name: str, used: set[Path]
    ) -> Path:
        """Create a fresh, never-previously-existing attempt directory.

        The directory name carries the invocation id and a monotonic per-run
        counter so repeated allocation for the same stage/scenario yields a
        distinct preserved path.  ``mkdir(exist_ok=False)`` guarantees the
        directory did not previously exist; all prior evidence is preserved.
        """
        attempt_root = Path(attempt_root).resolve()
        attempt_root.mkdir(parents=True, exist_ok=True)
        base = attempt_root / f"{stage}-{name}-{self._run_invocation_id}"
        for counter in range(self._attempt_counter, 100000):
            attempt_dir = base.with_name(f"{base.name}-{counter}").resolve()
            if attempt_dir in used:
                continue
            try:
                attempt_dir.mkdir(parents=True, exist_ok=False)
            except FileExistsError:
                continue
            self._attempt_counter = counter + 1
            return attempt_dir
        raise RuntimeError(
            "could not allocate a fresh integrated attempt directory for "
            f"{stage}-{name}"
        )

    def allocate_live_attempts(
        self,
        *,
        stages: Sequence[str],
        base_domain_id: int,
        attempt_root: Path | str,
    ) -> list[AttemptAllocation]:
        """Allocate a unique valid domain and a fresh immutable attempt dir.

        Every scenario in the requested stages gets one allocation.  Domains
        are monotonically increasing from ``base_domain_id``, stay within
        ``[0, 232]``, and are unique within the batch.  Attempt directories are
        freshly created and unique; a deterministic exhaustion error is raised
        before any child starts when no free domain remains.
        """
        allocations: list[AttemptAllocation] = []
        next_domain = int(base_domain_id)
        used_domains: set[int] = set()
        used_dirs: set[Path] = set()
        for stage in stages:
            for name in self._stage_scenarios(str(stage)):
                domain = next_domain % (MAX_ROS_DOMAIN_ID + 1)
                attempts = 0
                while domain in used_domains:
                    next_domain += 1
                    domain = next_domain % (MAX_ROS_DOMAIN_ID + 1)
                    attempts += 1
                    if attempts > MAX_ROS_DOMAIN_ID:
                        raise RuntimeError(
                            "ROS domain exhaustion: no unique domain in "
                            f"[0, {MAX_ROS_DOMAIN_ID}] for {stage}-{name}"
                        )
                used_domains.add(domain)
                next_domain += 1
                attempt_dir = self._allocate_attempt_dir(
                    Path(attempt_root), stage, name, used_dirs
                )
                used_dirs.add(attempt_dir)
                allocations.append(AttemptAllocation(domain, attempt_dir))
        return allocations

    def _allocate_one(self, name: str, stage: str) -> AttemptAllocation:
        allocations = self.allocate_live_attempts(
            stages=(stage,), base_domain_id=self.base_domain_id, attempt_root=self.attempt_root
        )
        for allocation in allocations:
            if allocation.attempt_dir.name.startswith(f"{stage}-{name}-"):
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

    def _core_config_gates(self) -> list[str]:
        core_config_path = self._core_config_path()
        core_config = _json_file(core_config_path)
        gates = core_config.get("gates")
        if not isinstance(gates, Sequence) or isinstance(gates, (str, bytes)):
            raise ValueError("core config gates must be an array")
        return [str(name) for name in gates]

    def _run_core_suite(self) -> dict[str, Any]:
        core_config_path = self._core_config_path()
        # C1: the six-gate Stage-A core suite lives in the sibling
        # ``<suite>-core`` root, outside the integrated Gate-F index.
        core_attempt_root = (
            self.attempt_root.parent
            / f"{self.attempt_root.name}{CORE_SUITE_DIRNAME_SUFFIX}"
        )
        result = _run_suite(
            root=self.root,
            attempt_root=core_attempt_root,
            config_path=core_config_path,
            artifact_path=None,
            seed=self.seed,
            readiness_timeout_s=self.readiness_timeout_s,
            isaac_command=self.isaac_command,
            humble_command=self.humble_command,
            gate_commands={},
            base_domain_id=self.base_domain_id,
        )
        suite_dir = Path(result.attempt_dir).resolve()
        suite_result_path = suite_dir / "suite-result.json"
        try:
            suite_result_sha256 = (
                hashlib.sha256(suite_result_path.read_bytes()).hexdigest()
                if suite_result_path.is_file()
                else None
            )
        except OSError:
            suite_result_sha256 = None
        core_status = str(result.status)
        if core_status not in {
            STATUS_VERIFIED_PASS,
            STATUS_VERIFIED_FAIL,
            STATUS_EVIDENCE_INVALID,
        }:
            core_status = STATUS_EVIDENCE_INVALID
        return {
            "status": core_status,
            "attempt_dir": str(result.attempt_dir),
            "suite_dir": str(suite_dir),
            "suite_result_sha256": suite_result_sha256,
            "gate_results": {
                str(gate): dict(record) for gate, record in result.gate_results.items()
            },
        }

    def _run_stage_a(self) -> dict[str, Any]:
        record_path = self._stage_record_path("A")
        if record_path.is_file():
            try:
                invoked_gates = self._core_gates()
            except (OSError, ValueError, KeyError, TypeError):
                invoked_gates = []
            return {
                "stage": "A",
                "status": STATUS_EVIDENCE_INVALID,
                "invoked_gates": invoked_gates,
                "reasons": [
                    f"{record_path.name} already exists; refusing to overwrite or relaunch"
                ],
                "stage_record": record_path.name,
            }
        gates = self._core_gates()
        duplicate_gate_names = sorted(
            {name for name in gates if gates.count(name) > 1}
        )
        try:
            executed_gates = self._core_config_gates()
        except (OSError, ValueError, json.JSONDecodeError) as error:
            return self._persist_stage_record("A", {
                "stage": "A",
                "status": STATUS_EVIDENCE_INVALID,
                "invoked_gates": gates,
                "duplicate_gate_names": duplicate_gate_names,
                "reasons": [f"core config could not be read: {error}"],
            })
        if duplicate_gate_names:
            return self._persist_stage_record("A", {
                "stage": "A",
                "status": STATUS_EVIDENCE_INVALID,
                "invoked_gates": gates,
                "duplicate_gate_names": duplicate_gate_names,
                "reasons": [
                    "integrated config required_core_gates are not unique: "
                    + ", ".join(duplicate_gate_names)
                ],
            })
        if gates != executed_gates:
            return self._persist_stage_record("A", {
                "stage": "A",
                "status": STATUS_EVIDENCE_INVALID,
                "invoked_gates": gates,
                "executed_gates": executed_gates,
                "duplicate_gate_names": duplicate_gate_names,
                "reasons": [
                    "integrated config required_core_gates does not equal the "
                    "core config gate list exactly (order and uniqueness)"
                ],
            })
        core = self._run_core_suite()
        core_status = str(core.get("status", STATUS_VERIFIED_PASS))
        core_reasons: list[str] = []
        if core_status == STATUS_VERIFIED_PASS:
            # A purported verified-pass core is load-bearing: it must carry a
            # lowercase 64-hex suite_result_sha256, exact configured
            # gate_results keys, and every configured gate verified-pass;
            # otherwise the record is evidence-invalid.  A genuine core
            # verified-fail is preserved verbatim.
            suite_sha = core.get("suite_result_sha256")
            if (
                not isinstance(suite_sha, str)
                or len(suite_sha) != 64
                or suite_sha != suite_sha.lower()
                or any(char not in "0123456789abcdef" for char in suite_sha)
            ):
                core_reasons.append(
                    "verified-pass core suite_result_sha256 is not a lowercase 64-hex SHA-256"
                )
            gate_results = core.get("gate_results")
            if not isinstance(gate_results, Mapping):
                core_reasons.append("verified-pass core has no gate_results object")
            else:
                result_keys = sorted(str(key) for key in gate_results)
                if result_keys != sorted(gates):
                    core_reasons.append(
                        "verified-pass core gate_results keys do not equal the "
                        "configured gates exactly"
                    )
                for gate in gates:
                    entry = gate_results.get(gate)
                    if (
                        not isinstance(entry, Mapping)
                        or str(entry.get("status")) != STATUS_VERIFIED_PASS
                    ):
                        core_reasons.append(f"core gate {gate} is not verified-pass")
            if core_reasons:
                core_status = STATUS_EVIDENCE_INVALID
        record: dict[str, Any] = {
            "stage": "A",
            "invoked_gates": gates,
            "executed_gates": executed_gates,
            "duplicate_gate_names": duplicate_gate_names,
            "status": core_status,
            "attempt_dir": core.get("attempt_dir"),
            "core_suite": core,
        }
        if core_reasons:
            record["reasons"] = core_reasons
        return self._persist_stage_record("A", record)

    # ------------------------------------------------------------------ #
    # Gate B — offline static closure, fail closed, never trusts current state
    # ------------------------------------------------------------------ #

    def _write_attempt_start(self) -> Path:
        """Atomically write a fresh per-invocation attempt-start identity."""
        self.attempt_root.mkdir(parents=True, exist_ok=True)
        attempt_id = (
            f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')}"
            f"-{os.getpid()}-{uuid.uuid4().hex[:10]}"
        )
        path = self.attempt_root / f"attempt-start-{attempt_id}.json"
        value = {
            "schema_version": 1,
            "attempt_id": attempt_id,
            "started_at": datetime.now(timezone.utc).isoformat(),
            "monotonic": time.monotonic(),
            "seed": self.seed,
            "root": str(self.root),
            "production_root": str(self.production_root),
            "config": str(self.config_path),
        }
        _write_json_atomic(path, value)
        return path

    @staticmethod
    def _wall_epoch(value: Mapping[str, Any]) -> float:
        """Return the UTC epoch seconds of an attempt-start ``started_at``."""
        try:
            parsed = datetime.fromisoformat(str(value["started_at"]))
        except (KeyError, TypeError, ValueError):
            return 0.0
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        try:
            return parsed.timestamp()
        except (OverflowError, OSError, ValueError):
            return 0.0

    @staticmethod
    def _newly_written(path: Path, reference_epoch: float) -> bool:
        """True only when *path* was written after the attempt start."""
        if not path.is_file():
            return False
        try:
            return path.stat().st_mtime >= reference_epoch
        except OSError:
            return False

    def _invoke_source_lock_manifest(
        self, attempt_start_path: Path, manifest_path: Path
    ) -> tuple[Path, dict[str, Any]]:
        policies = self._source_lock_policies()
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
            # Every non-pass status (verified-fail / evidence-invalid) is an
            # authorization-evidence failure: the brief requires mismatched,
            # stale, self-generated, or missing artifacts to be evidence-invalid.
            evidence["reason"] = f"source-lock manifest status is {status}"
            return evidence
        evidence["status"] = "pass"
        evidence["manifest"] = manifest
        return evidence

    def _invoke_static_contracts(
        self, manifest_path: Path, output_dir: Path, reference_epoch: float
    ) -> dict[str, Any]:
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
        if not self._newly_written(static_path, reference_epoch):
            evidence["reason"] = (
                "static-contract.json was not newly produced for this invocation"
            )
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
        if status == STATUS_VERIFIED_PASS:
            # Cross-bind Gate B's model fingerprint to the runtime model bundle
            # consumed by C-E readiness and the verifier.
            fingerprint = report.get("model_fingerprint")
            if not isinstance(fingerprint, str) or fingerprint != self._model_fingerprint():
                evidence["status"] = STATUS_EVIDENCE_INVALID
                evidence["reason"] = (
                    "static contract model_fingerprint does not match the runtime "
                    "model bundle manifest"
                )
                return evidence
            fingerprint_path = output_dir / MODEL_FINGERPRINT_FILENAME
            fingerprint_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "model_fingerprint": fingerprint,
                    },
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
        return evidence

    def _run_stage_b(self) -> dict[str, Any]:
        record_path = self._stage_record_path("B")
        if record_path.is_file():
            return {
                "stage": "B",
                "status": STATUS_EVIDENCE_INVALID,
                "reasons": [
                    f"{record_path.name} already exists; refusing to overwrite or relaunch"
                ],
                "stage_record": str(record_path),
            }
        attempt_start_path = self._write_attempt_start()
        try:
            attempt_start = _json_file(attempt_start_path)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            return self._persist_stage_record("B", {
                "stage": "B",
                "status": STATUS_EVIDENCE_INVALID,
                "reasons": [f"attempt-start identity is invalid: {error}"],
            })
        attempt_id = str(attempt_start.get("attempt_id", attempt_start_path.stem))
        reference_epoch = self._wall_epoch(attempt_start)
        gate_b_dir = self.attempt_root / f"gate-b-{attempt_id}"
        gate_b_dir.mkdir(parents=True, exist_ok=True)
        manifest_path = gate_b_dir / SOURCE_LOCK_MANIFEST_FILENAME
        _, source_lock = self._invoke_source_lock_manifest(attempt_start_path, manifest_path)
        if source_lock.get("status") != "pass":
            self.static_contract_status = STATUS_EVIDENCE_INVALID
            return self._persist_stage_record("B", {
                "stage": "B",
                "status": STATUS_EVIDENCE_INVALID,
                "source_lock": source_lock,
                "reasons": [
                    str(source_lock.get("reason", "source-lock manifest is not authorized"))
                ],
            })
        static = self._invoke_static_contracts(manifest_path, gate_b_dir, reference_epoch)
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
        return self._persist_stage_record("B", result)

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

    def _executor_scenario_command(
        self, name: str, attempt_dir: Path, domain_id: int
    ) -> list[str]:
        """F2.3: the source-run Humble executor driver third-child command.

        ``/usr/bin/python3`` (Humble 3.10) runs the driver source under the
        existing ``ros-tooling`` environment, binding the exact bundle/config/
        attempt/domain/seed arguments to the current immutable attempt.  The
        driver is launched only after canonical PHYSICS_READY, never during the
        initial two-child launch.
        """
        return [
            "/usr/bin/python3",
            str(self.root / "validation/integrated_gate_executor_driver.py"),
            "--scenario-bundle",
            str(attempt_dir / "scenario-bundle.json"),
            "--attempt-dir",
            str(attempt_dir),
            "--config",
            str(self.config_path),
            "--domain",
            str(domain_id),
            "--seed",
            str(self.seed),
        ]

    def _write_scenario_bundle(
        self, attempt_dir: Path, name: str, manifest: QualificationManifest
    ) -> Path:
        """F2.3: atomically write the already-validated scenario bundle.

        The orchestrator computes the bundle (scenario id/seed/declaration,
        planning-scene declaration + mapping, integrated mapping, report
        identities) and serializes it once; the Humble driver loads it unchanged
        and never recomputes identities across Python versions.  The bundle also
        carries the current-attempt identity so the driver and orchestrator
        cross-bind the terminal marker.
        """
        bundle = self._scenario_bundle(name)
        payload = {
            "schema_version": 1,
            "scenario_id": name,
            "attempt_id": manifest.attempt_id,
            "attempt_dir": str(Path(attempt_dir).resolve()),
            **dict(bundle),
        }
        _write_json_atomic(attempt_dir / "scenario-bundle.json", payload)
        return attempt_dir / "scenario-bundle.json"

    def _new_scenario_runner(
        self, allocation: AttemptAllocation, name: str
    ) -> QualificationRunner:
        scenario_path = self.root / "simulation/scenarios" / f"{name}.json"
        return QualificationRunner(
            root=self.root,
            attempt_root=self.attempt_root,
            config_path=self._core_config_path(),
            scenario_path=scenario_path,
            artifact_path=None,
            seed=self.seed,
            gate="integrated",
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
        self, allocation: AttemptAllocation, name: str, stage: str
    ) -> tuple[QualificationRunner, QualificationManifest]:
        attempt_dir = allocation.attempt_dir
        if attempt_dir.is_dir() and any(attempt_dir.iterdir()):
            raise ValueError(
                f"attempt directory is not empty; refusing to reuse: {attempt_dir}"
            )
        runner = self._new_scenario_runner(allocation, name)
        # attempt_id is tied to the freshly allocated directory so manifest,
        # launch environment, child logs, rosbag, truth, verifier, teardown,
        # cleanup, and the returned result all reference one directory.
        manifest = runner.prepare_manifest_at(
            attempt_dir.name, attempt_dir, scenario_id=name
        )
        self._scenario_manifest_store[attempt_dir] = (runner, manifest)
        baseline = qualification_gpu_processes(runner)
        self._gpu_baselines[attempt_dir] = baseline
        qualification_start_process(runner, "isaac", runner.isaac_command, manifest)
        try:
            qualification_start_process(runner, "humble", runner.humble_command, manifest)
        except Exception:
            # Partial launch: the Isaac child is already owned by this runner;
            # stop it before surfacing so the lifecycle owner is never left with
            # an orphaned producer.
            qualification_stop_process(runner, "isaac")
            raise
        return runner, manifest

    def _validate_physics_ready(
        self,
        attempt_dir: Path,
        name: str,
        *,
        manifest: QualificationManifest | None = None,
    ) -> tuple[bool, dict[str, Any], str | None]:
        """Validate ``physics-ready.json`` against the exact report bytes/identity.

        A transient ``state=PHYSICS_READY`` without the exact
        ``scenario_report_sha256`` of the atomically written ``scenario-runner.json``
        and the full committed identity is insufficient.  When *manifest* is
        supplied (the real lifecycle), the current-attempt manifest must be
        present and match the allocation, so a prior attempt's evidence cannot
        satisfy readiness with zero child launches.
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
        if manifest is not None:
            manifest_path = attempt_dir / "manifest.json"
            if not manifest_path.is_file():
                return (
                    False,
                    evidence,
                    "attempt manifest.json is missing; refusing to trust pre-existing evidence",
                )
            try:
                recorded = _json_file(manifest_path)
            except (OSError, ValueError, json.JSONDecodeError) as error:
                return False, evidence, f"attempt manifest.json is invalid: {error}"
            if str(recorded.get("attempt_id")) != manifest.attempt_id:
                return (
                    False,
                    evidence,
                    "attempt manifest attempt_id does not match the current allocation",
                )
            if str(recorded.get("scenario", {}).get("id")) != name:
                return False, evidence, "attempt manifest scenario id does not match"
            evidence["manifest"] = {
                "path": str(manifest_path),
                "attempt_id": recorded.get("attempt_id"),
                "scenario_id": recorded.get("scenario", {}).get("id"),
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
        try:
            bundle = self._scenario_bundle(name)
        except (OSError, ValueError, KeyError, FileNotFoundError) as error:
            return False, evidence, f"scenario bundle could not be resolved: {error}"
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
        self,
        attempt_dir: Path,
        name: str,
        *,
        manifest: QualificationManifest | None = None,
    ) -> tuple[bool, dict[str, Any], str | None]:
        deadline = time.monotonic() + self.readiness_timeout_s
        last: tuple[bool, dict[str, Any], str | None] = (
            False, {}, "physics-ready timeout",
        )
        while time.monotonic() < deadline:
            ok, evidence, reason = self._validate_physics_ready(
                attempt_dir, name, manifest=manifest
            )
            last = (ok, evidence, reason)
            if ok:
                return True, evidence, None
            if manifest is not None and not (attempt_dir / "manifest.json").is_file():
                # A missing manifest means this runner never launched the
                # attempt: hard fail, never spin on stale evidence.
                return False, evidence, reason or "attempt manifest is missing"
            time.sleep(0.25)
        return last

    def _terminal_cross_binds(
        self,
        value: Mapping[str, Any],
        attempt_dir: Path,
        name: str,
        attempt_id: str | None,
    ) -> bool:
        """A terminal marker is eligible only when it binds the current attempt.

        F2.3: the orchestrator never accepts an arbitrary preexisting marker — it
        must carry the exact scenario id, the exact current-attempt id, and the
        current attempt path.
        """
        if str(value.get("scenario_id")) != name:
            return False
        if attempt_id is not None and str(value.get("attempt_id")) != attempt_id:
            return False
        marker_path = str(value.get("attempt_dir", ""))
        if marker_path:
            try:
                if Path(marker_path).resolve() != Path(attempt_dir).resolve():
                    return False
            except (OSError, ValueError):
                return False
        return True

    def _executor_exited(self, runner: QualificationRunner) -> bool:
        process = runner._processes.get("executor")
        if process is None:
            return False
        try:
            return process.poll() is not None
        except Exception:  # noqa: BLE001 - defensive process liveness boundary
            return False

    def _wait_for_scenario_terminal(
        self,
        attempt_dir: Path,
        name: str,
        *,
        runner: QualificationRunner | None = None,
        attempt_id: str | None = None,
    ) -> dict[str, Any]:
        """Wait for the executor driver terminal within the derived budget.

        F2.3: waits for ``execution-terminal.json`` (cross-bound to the current
        scenario/attempt/path).  If the executor driver process exits before
        producing a valid current-attempt marker, fail immediately rather than
        sleeping the full timeout.  F2.5: uses ``terminal_timeout_s`` (derived
        from config, 305.0 s for the current thresholds) — never the readiness
        budget, so a marker arriving after 30 s but before the derived deadline
        is still eligible.
        """
        deadline = time.monotonic() + self.terminal_timeout_s
        while time.monotonic() < deadline:
            marker = attempt_dir / "execution-terminal.json"
            if marker.is_file():
                try:
                    value = _json_file(marker)
                except (OSError, ValueError, json.JSONDecodeError):
                    value = {}
                if self._terminal_cross_binds(value, attempt_dir, name, attempt_id):
                    return {
                        "ok": True,
                        "terminal": value,
                        "marker": str(marker),
                        "source": "executor-driver",
                    }
                return {
                    "ok": False,
                    "reason": (
                        "terminal marker identity does not bind the current "
                        "scenario/attempt/path"
                    ),
                    "terminal": value,
                }
            if runner is not None and self._executor_exited(runner):
                return {
                    "ok": False,
                    "reason": (
                        "executor process exited without producing a current-attempt "
                        "terminal marker"
                    ),
                }
            time.sleep(0.25)
        return {
            "ok": False,
            "reason": "scenario execution terminal was not observed within the terminal budget",
        }

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

    @staticmethod
    def _minimal_rosbag_index(attempt_dir: Path) -> dict[str, Any]:
        """Build a minimal Task-9 evidence index for one attempt's rosbag.

        ``_validate_rosbag`` consumes a real ``files`` projection categorized by
        ``_category``; this minimal index carries only the rosbag metadata and
        storage entries so the load-bearing QoS/type/count/storage semantics
        apply to the attempt's bag without a full suite index.
        """
        attempt_dir = Path(attempt_dir).resolve()
        bag_dir = attempt_dir / "rosbag"
        files: list[dict[str, Any]] = []
        if bag_dir.is_dir():
            for path in sorted(bag_dir.rglob("*")):
                if not path.is_file():
                    continue
                try:
                    rel = path.relative_to(attempt_dir).as_posix()
                except ValueError:
                    continue
                if rel == "rosbag/metadata.yaml":
                    category = "rosbag-metadata"
                elif rel.startswith("rosbag/"):
                    category = "rosbag-storage"
                else:
                    continue
                files.append({"path": rel, "category": category})
        return {
            "schema_version": 1,
            "kind": "integrated-evidence-index",
            "files": files,
        }

    def _integrated_rosbag_evidence(
        self, attempt_dir: Path
    ) -> tuple[bool, dict[str, Any], list[str]]:
        """Validate any rosbag recorded in the attempt directory (C5).

        Approved-topic availability remains a live qualification check, so a
        missing ``rosbag`` directory is non-load-bearing evidence.  A present
        bag is load-bearing: corrupt/incomplete metadata, storage, a missing
        approved topic, bad QoS, or an invalid count downgrades the scenario to
        ``evidence-invalid`` before verifier success can be accepted.  This is
        the integrated finalizer and never delegates to
        ``qualification_rosbag_final_evidence`` (its six-gate minimum-count
        lookup rejects the ``integrated`` gate).
        """
        bag_dir = attempt_dir / "rosbag"
        evidence: dict[str, Any] = {
            "output_directory": bag_dir.is_dir(),
        }
        if not bag_dir.is_dir():
            evidence["status"] = "not-recorded"
            evidence["load_bearing"] = False
            return True, evidence, []
        metadata_path = bag_dir / "metadata.yaml"
        if not metadata_path.is_file():
            evidence["status"] = "invalid"
            evidence["load_bearing"] = True
            return (
                False,
                evidence,
                ["integrated rosbag directory is present but has no metadata.yaml"],
            )
        try:
            metadata = qualification_rosbag_metadata_evidence(
                metadata_path.read_text(encoding="utf-8"),
                minimum_message_counts=None,
            )
        except (OSError, ValueError) as error:
            evidence["status"] = "invalid"
            evidence["load_bearing"] = True
            return False, evidence, [f"integrated rosbag metadata is unreadable: {error}"]
        structured = bool(metadata.get("parsed"))
        evidence["metadata"] = metadata
        evidence["load_bearing"] = True
        if not structured:
            evidence["status"] = "invalid"
            return (
                False,
                evidence,
                ["integrated rosbag metadata is not structured rosbag2 metadata"],
            )
        # Storage: the output database must be present and openable.
        output = qualification_rosbag_output_evidence(bag_dir)
        evidence["output"] = output
        # The present bag is also validated by the Task-9 semantic rosbag
        # validator against a minimal metadata/storage index; every QoS, type,
        # count, or storage semantic reason is load-bearing.
        try:
            from integrated_evidence_index import _validate_rosbag  # noqa: PLC0415
        except ModuleNotFoundError:
            from validation.integrated_evidence_index import _validate_rosbag  # noqa: PLC0415
        semantic_reasons: list[str] = []
        try:
            _validate_rosbag(self._minimal_rosbag_index(attempt_dir), attempt_dir.resolve(), semantic_reasons)
        except Exception as error:  # noqa: BLE001 - semantic validator boundary
            semantic_reasons.append(f"integrated rosbag semantic validation failed: {error}")
        evidence["semantic"] = {
            "load_bearing": True,
            "reasons": semantic_reasons,
        }
        failures: list[str] = []
        for topic in metadata.get("missing_topics", []):
            failures.append(f"integrated rosbag is missing approved topic {topic}")
        for topic in metadata.get("below_minimum_topics", []):
            failures.append(
                f"integrated rosbag has an invalid message count for approved topic {topic}"
            )
        for topic in metadata.get("missing_qos_metadata", []):
            failures.append(f"integrated rosbag is missing QoS metadata for {topic}")
        if not output.get("open"):
            failures.append("integrated rosbag output database is missing or not openable")
        failures.extend(semantic_reasons)
        if failures:
            evidence["status"] = "invalid"
            return False, evidence, failures
        evidence["status"] = "valid"
        return True, evidence, []

    def _finalize_attempt(
        self,
        runner: QualificationRunner | None,
        manifest: QualificationManifest | None,
        gpu_baseline: dict[str, Any] | None,
    ) -> dict[str, Any]:
        """Stop producers, drain, finalize rosbag, and clean up; fail-dominant.

        F2.6: every cleanup phase is attempted independently — an exception in
        executor stop, Isaac stop, drain, Humble stop, rosbag handling, orphan
        termination, resource evidence, or settle is caught and recorded as a
        distinct failure reason while all later phases still run.  The executor
        is stopped first so it cannot continue issuing commands while the graph
        shuts down; then Isaac is stopped to freeze raw production; the exact
        evaluator/raw drain runs while Humble is still alive; then Humble and
        the recorder are stopped, rosbag evidence finalized, escaped orphans
        terminated, cleanup/resource evidence written, and evidence settled.
        Returns the failures list; the independent verifier runs only after this
        is final and clean.
        """
        failures: list[str] = []
        if runner is None or manifest is None:
            return {
                "failures": failures,
                "drained": False,
                "rosbag_ok": False,
                "exit_codes": {},
            }
        exit_codes: dict[str, int | None] = {}

        # Phase 1 — executor stop (before graph shutdown so it cannot continue
        # issuing commands; on normal terminal completion it has already exited).
        try:
            if "executor" in runner._processes:
                exit_codes["executor"] = qualification_stop_process(runner, "executor")
        except Exception as error:  # noqa: BLE001 - per-phase isolation
            failures.append(f"executor stop failed: {error}")

        # Phase 2 — Isaac stop (freeze raw production for the exact drain).
        try:
            if "isaac" in runner._processes:
                exit_codes["isaac"] = qualification_stop_process(runner, "isaac")
        except Exception as error:  # noqa: BLE001 - per-phase isolation
            failures.append(f"isaac stop failed: {error}")

        # Phase 3 — exact evaluator/raw drain while Humble is alive.
        drained = False
        try:
            drained = qualification_wait_for_evaluator_drain(runner, manifest)
            if not drained:
                failures.append("raw/evaluator drain did not correlate exactly")
        except Exception as error:  # noqa: BLE001 - per-phase isolation
            drained = False
            failures.append(f"evaluator drain failed: {error}")

        # Phase 4 — Humble stop.
        try:
            if "humble" in runner._processes:
                exit_codes["humble"] = qualification_stop_process(runner, "humble")
        except Exception as error:  # noqa: BLE001 - per-phase isolation
            failures.append(f"humble stop failed: {error}")

        # Phase 5 — recorder stop (both the six-gate and integrated paths own a
        # bag when the recorder was started; SIGINT-then-SIGTERM before the
        # load-bearing integrated bag evidence in Phase 6).
        rosbag_registered = "rosbag" in runner._processes
        try:
            if rosbag_registered:
                exit_codes["rosbag"] = qualification_stop_process(runner, "rosbag")
        except Exception as error:  # noqa: BLE001 - per-phase isolation
            failures.append(f"rosbag stop failed: {error}")

        # Phase 5b — the recorder must terminate as a planned stop: a
        # classification of ``planned-termination``, returncode 0, and forced
        # false.  A missing, malformed, unexpected, or forced termination fails
        # closed before the load-bearing bag evidence is accepted.
        rosbag_termination: dict[str, Any] | None = None
        rosbag_termination_ok = not rosbag_registered
        if rosbag_registered:
            rosbag_termination = runner._termination.get("rosbag")
            if not isinstance(rosbag_termination, Mapping):
                failures.append("rosbag recorder has no termination record after stop")
                rosbag_termination = None
            else:
                classification = str(rosbag_termination.get("classification", ""))
                returncode = rosbag_termination.get("returncode")
                forced = rosbag_termination.get("forced")
                if classification != "planned-termination":
                    failures.append(
                        f"rosbag recorder termination classification is {classification}, "
                        "expected planned-termination"
                    )
                if (
                    not isinstance(returncode, int)
                    or isinstance(returncode, bool)
                    or returncode != 0
                ):
                    failures.append(
                        f"rosbag recorder termination returncode is {returncode!r}, expected 0"
                    )
                if forced is not False:
                    failures.append(
                        f"rosbag recorder termination forced is {forced!r}, expected false"
                    )
            rosbag_termination_ok = (
                rosbag_termination is not None
                and str(rosbag_termination.get("classification", "")) == "planned-termination"
                and isinstance(rosbag_termination.get("returncode"), int)
                and not isinstance(rosbag_termination.get("returncode"), bool)
                and rosbag_termination.get("returncode") == 0
                and rosbag_termination.get("forced") is False
            )

        # Phase 6 — tolerant integrated rosbag evidence (F2.7 semantics).
        rosbag_ok = False
        rosbag_evidence: dict[str, Any] = {}
        try:
            rosbag_ok, rosbag_evidence, rosbag_failures = self._integrated_rosbag_evidence(
                manifest.attempt_dir
            )
            # An invalid registered-rosbag termination contract always fails the
            # finalize evidence: rosbag_ok must never be true while termination
            # is missing/malformed/unexpected/nonzero/forced.
            if not rosbag_termination_ok:
                rosbag_ok = False
            if rosbag_termination is not None:
                rosbag_evidence["termination"] = rosbag_termination
            if rosbag_failures:
                failures.extend(rosbag_failures)
        except Exception as error:  # noqa: BLE001 - per-phase isolation
            rosbag_ok = False
            failures.append(f"integrated rosbag evidence failed: {error}")

        # Phase 7 — orphan termination.
        orphan_initial: list[dict[str, Any]] = []
        try:
            orphan_initial = qualification_attempt_processes(runner)
            orphan_survivors = (
                qualification_terminate_attempt_orphans(runner) if orphan_initial else []
            )
            if qualification_orphan_failure(orphan_initial, orphan_survivors):
                failures.append("orphan attempt processes remained after teardown")
        except Exception as error:  # noqa: BLE001 - per-phase isolation
            failures.append(f"orphan termination failed: {error}")

        # Phase 8 — cleanup/resource evidence.
        resources_clean = False
        try:
            resources_clean = qualification_write_resource_evidence(
                runner, manifest, gpu_baseline or {}
            )
            if not resources_clean:
                failures.append(
                    "cleanup/resource evidence reported owned survivors or GPU memory growth"
                )
        except Exception as error:  # noqa: BLE001 - per-phase isolation
            resources_clean = False
            failures.append(f"resource evidence failed: {error}")

        # Phase 9 — settle evidence files.
        try:
            qualification_settle_evidence_files(runner, manifest)
        except Exception as error:  # noqa: BLE001 - per-phase isolation
            failures.append(f"settle failed: {error}")

        return {
            "failures": failures,
            "exit_codes": exit_codes,
            "drained": drained,
            "rosbag_ok": rosbag_ok,
            "rosbag_evidence": rosbag_evidence,
            "rosbag_termination": rosbag_termination,
            "orphan_cleanup_required": bool(orphan_initial),
            "resources_clean": resources_clean,
        }

    def _teardown_scenario(self, allocation: AttemptAllocation) -> bool:
        """Finalize a launched scenario; idempotent for never-launched/cleaned.

        Returns False when any teardown/drain/bag/resource obligation failed so
        ``run_scenario`` can downgrade the scenario to ``evidence-invalid``.
        F2.6: an unexpected ``_finalize_attempt`` escape is guarded and converted
        into durable per-scenario failure evidence (the phase-level isolation
        inside ``_finalize_attempt`` keeps every later cleanup running, so this
        guard only covers a whole-helper escape).
        """
        runner, manifest = self._scenario_manifest_store.pop(
            allocation.attempt_dir, (None, None)
        )
        if runner is None:
            return True
        baseline = self._gpu_baselines.pop(allocation.attempt_dir, None)
        try:
            finalize = self._finalize_attempt(runner, manifest, baseline)
        except Exception as error:  # noqa: BLE001 - F2.6 whole-helper escape guard
            return False
        return not finalize["failures"]

    def _execute_scenario(
        self, allocation: AttemptAllocation, name: str, stage: str
    ) -> dict[str, Any]:
        attempt_dir = allocation.attempt_dir
        runner = None
        manifest = None
        failure_reasons: list[str] = []
        finalize_evidence: dict[str, Any] = {}
        error_text: str | None = None
        try:
            try:
                runner, manifest = self._launch_scenario(allocation, name, stage)
            except Exception as error:  # noqa: BLE001 - per-scenario boundary
                error_text = str(error)
                failure_reasons.append(f"scenario launch failed: {error}")
            if error_text is None:
                try:
                    ready_ok, _ready_evidence, ready_reason = (
                        self._wait_for_physics_ready(attempt_dir, name, manifest=manifest)
                    )
                    if not ready_ok:
                        failure_reasons.append(
                            ready_reason or "physics readiness was not achieved"
                        )
                    else:
                        # F2.3: only after canonical PHYSICS_READY — atomically
                        # write the already-validated bundle and launch the
                        # source-run Humble executor driver as a third owned child
                        # under the exact same ROS domain and attempt directory.
                        try:
                            self._write_scenario_bundle(attempt_dir, name, manifest)
                        except Exception as error:  # noqa: BLE001 - fail-closed bundle write
                            failure_reasons.append(f"scenario bundle write failed: {error}")
                        if not failure_reasons:
                            try:
                                # C5: the load-bearing rosbag recorder starts
                                # after canonical PHYSICS_READY/bundle and before
                                # the executor, through the existing
                                # ``_start_rosbag`` QoS/output/readiness path.
                                if not runner._start_rosbag(manifest):
                                    failure_reasons.append(
                                        "rosbag recorder failed to start; executor not launched"
                                    )
                            except Exception as error:  # noqa: BLE001 - fail-closed rosbag startup
                                failure_reasons.append(
                                    f"rosbag recorder startup raised: {error}"
                                )
                        if not failure_reasons:
                            try:
                                qualification_start_process(
                                    runner,
                                    "executor",
                                    self._executor_scenario_command(
                                        name, attempt_dir, allocation.domain_id
                                    ),
                                    manifest,
                                )
                            except Exception as error:  # noqa: BLE001 - fail-closed executor launch
                                failure_reasons.append(f"executor launch failed: {error}")
                        if not failure_reasons:
                            terminal = self._wait_for_scenario_terminal(
                                attempt_dir,
                                name,
                                runner=runner,
                                attempt_id=manifest.attempt_id,
                            )
                            if not terminal.get("ok"):
                                failure_reasons.append(
                                    str(
                                        terminal.get(
                                            "reason",
                                            "scenario execution terminal was not observed",
                                        )
                                    )
                                )
                except Exception as error:  # noqa: BLE001 - fail-dominant lifecycle
                    error_text = str(error)
                    failure_reasons.append(f"scenario lifecycle raised: {error}")
        finally:
            # Every post-launch path attempts bounded cleanup: when the local
            # runner is None (partial launch or launch failure), fall back to the
            # registered lifecycle owner so an already-started Isaac/Humble child
            # is still stopped, drained, and accounted.
            stored_runner, stored_manifest = self._scenario_manifest_store.pop(
                attempt_dir, (None, None)
            )
            effective_runner = runner or stored_runner
            effective_manifest = manifest or stored_manifest
            baseline = self._gpu_baselines.pop(attempt_dir, None)
            try:
                finalize_evidence = self._finalize_attempt(
                    effective_runner, effective_manifest, baseline
                )
                failure_reasons.extend(finalize_evidence.get("failures", []))
            except Exception as error:  # noqa: BLE001 - F2.6 whole-helper escape guard
                error_text = error_text or str(error)
                finalize_evidence = {
                    "failures": [f"finalize_attempt escaped: {error}"],
                    "error": str(error),
                }
                failure_reasons.append(f"finalize_attempt escaped: {error}")
        if error_text is not None or failure_reasons:
            return self._scenario_result(
                name,
                stage,
                STATUS_EVIDENCE_INVALID,
                failure_reasons,
                finalize=finalize_evidence,
                error=error_text,
                attempt_dir=str(attempt_dir),
            )
        try:
            verdict = self._verify_attempt(attempt_dir, name, stage)
        except Exception as error:  # noqa: BLE001 - durable per-scenario boundary
            return self._scenario_result(
                name,
                stage,
                STATUS_EVIDENCE_INVALID,
                [f"verification failed: {error}"],
                finalize=finalize_evidence,
                attempt_dir=str(attempt_dir),
            )
        return self._scenario_result(
            name,
            stage,
            str(verdict.get("status", STATUS_EVIDENCE_INVALID)),
            [],
            verdict=verdict,
            finalize=finalize_evidence,
            attempt_dir=str(attempt_dir),
        )

    @staticmethod
    def _scenario_result(
        name: str,
        stage: str,
        status: str,
        reasons: Sequence[str],
        *,
        verdict: Mapping[str, Any] | None = None,
        finalize: Mapping[str, Any] | None = None,
        error: str | None = None,
        attempt_dir: str | None = None,
    ) -> dict[str, Any]:
        result: dict[str, Any] = {
            "scenario": name,
            "stage": stage,
            "status": status,
            "started": True,
        }
        if attempt_dir is not None:
            result["attempt_dir"] = str(attempt_dir)
        if reasons:
            result["reasons"] = [str(reason) for reason in reasons]
        if verdict is not None:
            result["verdict"] = dict(verdict)
        if finalize is not None:
            result["finalize"] = dict(finalize)
        if error is not None:
            result["error"] = error
        return result

    def run_scenario(
        self,
        name: str,
        *,
        stage: str,
        allocation: AttemptAllocation | None = None,
    ) -> dict[str, Any]:
        """Run one scenario in a fresh child domain and immutable attempt dir.

        A malformed scenario declaration/bundle/identity fails that scenario
        closed (``evidence-invalid``) without aborting the rest of the stage.
        The independent verifier's status is authoritative; execution return
        codes never override it.  A teardown failure downgrades the scenario to
        ``evidence-invalid``.  Every attempt directory is preserved.
        """
        try:
            self._scenario_bundle(name)
        except (OSError, ValueError, KeyError, FileNotFoundError, TypeError) as error:
            return {
                "scenario": name,
                "stage": stage,
                "status": STATUS_EVIDENCE_INVALID,
                "started": False,
                "reasons": [f"malformed scenario data: {error}"],
            }
        allocation = allocation or self._allocate_one(name, stage)
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
        # Pre-allocate the whole stage once: unique domains within the batch and
        # one fresh immutable attempt directory per scenario.  A malformed
        # scenario never launches, so its (empty) allocation is simply preserved.
        allocations = self.allocate_live_attempts(
            stages=(stage,),
            base_domain_id=self.base_domain_id,
            attempt_root=self.attempt_root,
        )
        results: dict[str, Any] = {}
        for name, allocation in zip(names, allocations):
            results[name] = self.run_scenario(
                name, stage=stage, allocation=allocation
            )
            # A patched/double run_scenario may omit the started flag; the real
            # runner records started=False for a malformed scenario that never
            # launched, and that durable flag must not be overwritten.
            if "started" not in results[name]:
                results[name]["started"] = True
        status = _reduce_scenario_statuses(
            [
                str(value.get("status", STATUS_EVIDENCE_INVALID))
                for value in results.values()
                if isinstance(value, Mapping)
            ]
        )
        return {"stage": stage, "status": status, "scenario_names": names, **results}

    def _run_stage_c(self) -> dict[str, Any]:
        return self._run_scenario_stage_write_once("C")

    def _run_stage_d(self) -> dict[str, Any]:
        return self._run_scenario_stage_write_once("D")

    def _run_stage_e(self) -> dict[str, Any]:
        return self._run_scenario_stage_write_once("E")

    def _run_scenario_stage_write_once(self, stage: str) -> dict[str, Any]:
        """Run a C/D/E scenario stage and persist its record write-once.

        When the stage record already exists the stage fails closed before any
        attempt allocation or launch, so a repeated run never merges new
        attempts into an old suite (C1).
        """
        record_path = self._stage_record_path(stage)
        if record_path.is_file():
            try:
                scenario_names = self._stage_scenarios(stage)
            except (OSError, ValueError, KeyError, TypeError):
                scenario_names = []
            return {
                "stage": stage,
                "status": STATUS_EVIDENCE_INVALID,
                "scenario_names": scenario_names,
                "reasons": [
                    f"{record_path.name} already exists; refusing to overwrite or relaunch"
                ],
                "stage_record": str(record_path),
            }
        return self._persist_stage_record(stage, self._run_scenario_stage(stage))

    # ------------------------------------------------------------------ #
    # Write-once stage records + Stage-F predecessor validation
    # ------------------------------------------------------------------ #

    def _stage_record_path(self, stage: str) -> Path:
        return self.attempt_root / STAGE_RECORD_FILENAMES[str(stage).upper()]

    def _persist_stage_record(self, stage: str, result: Mapping[str, Any]) -> dict[str, Any]:
        """Atomically persist a write-once stage record under the suite.

        The record path is created under the integrated suite; an existing
        record fails closed instead of being overwritten, so a repeated run can
        never merge into or replace an old suite's evidence (C1).
        """
        path = self._stage_record_path(stage)
        if path.is_file():
            return {
                **dict(result),
                "status": STATUS_EVIDENCE_INVALID,
                "reasons": [
                    f"{path.name} already exists; refusing to overwrite or relaunch"
                ],
                "stage_record": str(path),
            }
        payload = {**dict(result), "stage_record": path.name}
        _write_json_atomic(path, payload)
        return dict(payload)

    def _read_stage_record(self, stage: str) -> dict[str, Any] | None:
        """Return the persisted stage record, or None when absent.

        A present-but-malformed record raises ValueError so the predecessor
        validation fails closed instead of trusting partial evidence.
        """
        path = self._stage_record_path(stage)
        if not path.is_file():
            return None
        try:
            value = _json_file(path)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            raise ValueError(
                f"stage record {path.name} is not finite JSON: {error}"
            ) from error
        if not isinstance(value, Mapping):
            raise ValueError(f"stage record {path.name} is not a JSON object")
        return dict(value)

    def _validate_f_predecessors(self, suite_dir: Path) -> dict[str, Any]:
        """C3: validate the persisted A-E records before any Stage-F write.

        Returns ``verified-pass`` only when every predecessor is complete,
        consistent with the configured gate/scenario sets, and the referenced
        core evidence is still byte-current.  Any missing/malformed/mismatched
        predecessor yields ``evidence-invalid`` with explicit reasons and
        performs no index/sheet/summary generation.
        """
        suite_dir = Path(suite_dir).resolve()
        reasons: list[str] = []
        in_memory = {
            key: value
            for key, value in self._stage_results.items()
            if key in STAGE_RECORD_FILENAMES
        }
        configured_gates = self._core_gates()
        duplicate_configured_gates = sorted(
            {name for name in configured_gates if configured_gates.count(name) > 1}
        )
        if duplicate_configured_gates:
            reasons.append(
                "configured core gates are not unique: "
                + ", ".join(duplicate_configured_gates)
            )

        # ---- Stage A ----------------------------------------------------
        try:
            record_a = self._read_stage_record("A")
        except ValueError as error:
            reasons.append(str(error))
            record_a = None
        if record_a is None:
            reasons.append(f"{STAGE_RECORD_FILENAMES['A']} is missing")
        else:
            a_status = str(record_a.get("status", ""))
            if a_status != STATUS_VERIFIED_PASS:
                reasons.append(
                    f"stage A record status is {a_status}, expected {STATUS_VERIFIED_PASS}"
                )
            invoked = [str(name) for name in record_a.get("invoked_gates", [])]
            if invoked != configured_gates:
                reasons.append(
                    "stage A record invoked_gates do not match the configured core gates"
                )
            core = record_a.get("core_suite")
            if not isinstance(core, Mapping):
                reasons.append("stage A record has no core_suite reference")
            else:
                recorded_core_status = str(core.get("status", ""))
                if recorded_core_status != a_status:
                    reasons.append(
                        "stage A record top-level status conflicts with its core_suite status"
                    )
                core_gate_results = core.get("gate_results")
                if not isinstance(core_gate_results, Mapping):
                    reasons.append("stage A record core_suite has no gate_results object")
                else:
                    result_keys = sorted(str(key) for key in core_gate_results)
                    if result_keys != sorted(configured_gates):
                        reasons.append(
                            "stage A record core_suite gate_results keys do not equal the "
                            "configured gates exactly"
                        )
                    for gate in configured_gates:
                        entry = core_gate_results.get(gate)
                        if (
                            not isinstance(entry, Mapping)
                            or str(entry.get("status")) != STATUS_VERIFIED_PASS
                        ):
                            reasons.append(
                                f"stage A gate {gate} is not verified-pass in the record"
                            )
                core_dir_value = core.get("suite_dir") or core.get("attempt_dir")
                core_path: Path | None = None
                if core_dir_value is None:
                    reasons.append("stage A record core_suite has no suite_dir/attempt_dir")
                else:
                    try:
                        core_path = Path(str(core_dir_value)).resolve()
                    except (TypeError, ValueError):
                        reasons.append("stage A record core_suite path is not a path")
                if core_path is not None:
                    expected_core_root = (
                        suite_dir.parent / f"{suite_dir.name}{CORE_SUITE_DIRNAME_SUFFIX}"
                    ).resolve()
                    if not core_path.is_relative_to(expected_core_root):
                        reasons.append(
                            "stage A core suite does not live under the expected sibling core root"
                        )
                    suite_result_path = core_path / "suite-result.json"
                    if not suite_result_path.is_file():
                        reasons.append("stage A core suite-result.json is missing")
                    else:
                        try:
                            current_sha = hashlib.sha256(
                                suite_result_path.read_bytes()
                            ).hexdigest()
                        except OSError as error:
                            reasons.append(
                                f"stage A core suite-result.json is unreadable: {error}"
                            )
                        else:
                            recorded_sha = core.get("suite_result_sha256")
                            if not isinstance(recorded_sha, str) or recorded_sha != current_sha:
                                reasons.append(
                                    "stage A core suite-result.json SHA-256 no longer matches the record"
                                )
                        try:
                            current_suite = _json_file(suite_result_path)
                        except (OSError, ValueError, json.JSONDecodeError) as error:
                            reasons.append(
                                f"stage A core suite-result.json is not finite JSON: {error}"
                            )
                        else:
                            current_status = str(current_suite.get("status", ""))
                            if current_status != recorded_core_status:
                                reasons.append(
                                    "stage A core suite-result.json status no longer matches the record"
                                )
                            gates_map = current_suite.get("gates")
                            if not isinstance(gates_map, Mapping):
                                reasons.append(
                                    "stage A core suite-result.json gates are not an object"
                                )
                            else:
                                current_gate_keys = sorted(str(key) for key in gates_map)
                                if current_gate_keys != sorted(configured_gates):
                                    reasons.append(
                                        "stage A core suite-result.json gates keys do not equal "
                                        "the configured gates exactly"
                                    )
                                for gate in configured_gates:
                                    entry = gates_map.get(gate)
                                    if (
                                        not isinstance(entry, Mapping)
                                        or str(entry.get("status")) != STATUS_VERIFIED_PASS
                                    ):
                                        reasons.append(
                                            f"stage A core suite gate {gate} is not verified-pass"
                                        )
            if "A" in in_memory:
                mem_a = in_memory["A"]
                if str(mem_a.get("status")) != a_status:
                    reasons.append(
                        "stage A in-memory status conflicts with the persisted record"
                    )
                mem_invoked = [str(name) for name in mem_a.get("invoked_gates", [])]
                if mem_invoked != invoked:
                    reasons.append(
                        "stage A in-memory gate set conflicts with the persisted record"
                    )

        # ---- Stage B ----------------------------------------------------
        try:
            record_b = self._read_stage_record("B")
        except ValueError as error:
            reasons.append(str(error))
            record_b = None
        if record_b is None:
            reasons.append(f"{STAGE_RECORD_FILENAMES['B']} is missing")
        elif str(record_b.get("status")) != STATUS_VERIFIED_PASS:
            reasons.append("stage B record status is not verified-pass")
        if "B" in in_memory and record_b is not None:
            if str(in_memory["B"].get("status")) != str(record_b.get("status")):
                reasons.append(
                    "stage B in-memory status conflicts with the persisted record"
                )

        # ---- Stages C/D/E ------------------------------------------------
        seen_attempt_dirs: set[Path] = set()
        for stage in ("C", "D", "E"):
            try:
                record = self._read_stage_record(stage)
            except ValueError as error:
                reasons.append(str(error))
                record = None
            if record is None:
                reasons.append(f"{STAGE_RECORD_FILENAMES[stage]} is missing")
                continue
            if str(record.get("status")) != STATUS_VERIFIED_PASS:
                reasons.append(f"stage {stage} record status is not verified-pass")
            expected = self._stage_scenarios(stage)
            recorded_names = [str(name) for name in record.get("scenario_names", [])]
            if recorded_names != expected:
                reasons.append(
                    f"stage {stage} record scenario set does not match the configured scenario set"
                )
            if len(recorded_names) != len(set(recorded_names)):
                reasons.append(f"stage {stage} record scenario_names are not unique")
            expected_keys = {str(name) for name in expected} | {
                "stage",
                "status",
                "scenario_names",
                "stage_record",
            }
            recorded_keys = {str(key) for key in record}
            if recorded_keys != expected_keys:
                reasons.append(
                    f"stage {stage} record keys do not equal the expected scenario/summary key set"
                )
            for name in expected:
                entry = record.get(name)
                if not isinstance(entry, Mapping):
                    reasons.append(f"stage {stage} record is missing scenario {name}")
                    continue
                if str(entry.get("status")) != STATUS_VERIFIED_PASS:
                    reasons.append(f"stage {stage} scenario {name} is not verified-pass")
                attempt_value = entry.get("attempt_dir")
                if not attempt_value:
                    reasons.append(f"stage {stage} scenario {name} has no attempt_dir")
                else:
                    try:
                        attempt_path = Path(str(attempt_value)).resolve()
                    except (TypeError, ValueError):
                        reasons.append(
                            f"stage {stage} scenario {name} attempt_dir is not a path"
                        )
                        attempt_path = None
                    if attempt_path is not None:
                        if not attempt_path.is_relative_to(suite_dir):
                            reasons.append(
                                f"stage {stage} scenario {name} attempt directory escapes the integrated suite"
                            )
                        if not attempt_path.is_dir():
                            reasons.append(
                                f"stage {stage} scenario {name} attempt directory does not exist"
                            )
                        if attempt_path in seen_attempt_dirs:
                            reasons.append(
                                f"stage {stage} scenario {name} attempt directory is shared across scenarios"
                            )
                        seen_attempt_dirs.add(attempt_path)
            if stage in in_memory and record is not None:
                mem = in_memory[stage]
                if str(mem.get("status")) != str(record.get("status")):
                    reasons.append(
                        f"stage {stage} in-memory status conflicts with the persisted record"
                    )
                mem_names = [str(name) for name in mem.get("scenario_names", [])]
                if mem_names != recorded_names:
                    reasons.append(
                        f"stage {stage} in-memory scenario set conflicts with the persisted record"
                    )

        if reasons:
            return {"status": STATUS_EVIDENCE_INVALID, "reasons": reasons}
        return {"status": STATUS_VERIFIED_PASS, "reasons": []}

    def _regenerate_contact_sheets(self, suite_dir: Path) -> None:
        """Regenerate both canonical integrated contact sheets from the index."""
        try:
            from integrated_contact_sheets import (  # noqa: PLC0415
                _all_bound_capture_entries,
                build_contact_sheet,
            )
        except ModuleNotFoundError:
            from validation.integrated_contact_sheets import (  # noqa: PLC0415
                _all_bound_capture_entries,
                build_contact_sheet,
            )
        suite_resolved = Path(suite_dir).resolve()
        entries = _all_bound_capture_entries(suite_resolved)
        paths = [suite_resolved / entry["path"] for entry in entries]
        build_contact_sheet(
            suite_resolved, paths, output=suite_resolved / AGENT_SHEET_NAME
        )
        build_contact_sheet(
            suite_resolved,
            paths,
            output=suite_resolved / USER_SHEET_NAME,
            user=True,
        )

    def _run_stage_f(self) -> dict[str, Any]:
        suite_dir = self.attempt_root.resolve()
        index_path = suite_dir / INDEX_NAME
        summary_path = suite_dir / SUMMARY_NAME
        agent_sheet_path = suite_dir / AGENT_SHEET_NAME
        user_sheet_path = suite_dir / USER_SHEET_NAME
        try:
            section = self._stages_config().get("F", {})
        except (OSError, ValueError, KeyError, TypeError):
            section = {}
        checksum_algorithm = (
            section.get("checksum_algorithm") if isinstance(section, Mapping) else None
        )
        cameras = (
            list(section.get("cameras", [])) if isinstance(section, Mapping) else []
        )
        try:
            validation = self._validate_f_predecessors(suite_dir)
        except (OSError, ValueError, KeyError, TypeError) as error:
            validation = {
                "status": STATUS_EVIDENCE_INVALID,
                "reasons": [f"stage predecessor validation failed: {error}"],
            }
        if validation["status"] != STATUS_VERIFIED_PASS:
            return {
                "stage": "F",
                "status": STATUS_EVIDENCE_INVALID,
                "reasons": validation["reasons"],
                "suite_dir": str(suite_dir),
                "index": str(index_path),
                "summary": str(summary_path),
                "agent_sheet": str(agent_sheet_path),
                "user_sheet": str(user_sheet_path),
                "extension_point": "tasks-9-10",
                "checksum_algorithm": checksum_algorithm,
                "cameras": cameras,
                "evidence": validation,
            }
        try:
            # C4 step 2: remove only a prior derived summary so a repeated F can
            # rebuild a current checksum cycle.  Raw evidence, stage records,
            # attempts, bags, captures, and verdicts are never deleted.
            if summary_path.is_file():
                summary_path.unlink()
            # C4 steps 3-5: direct import-callable Task-9 producers.
            try:
                from integrated_evidence_index import (  # noqa: PLC0415
                    build_evidence_index,
                    build_qualification_summary,
                )
            except ModuleNotFoundError:
                from validation.integrated_evidence_index import (  # noqa: PLC0415
                    build_evidence_index,
                    build_qualification_summary,
                )
            build_evidence_index(suite_dir=suite_dir, output=index_path)
            self._regenerate_contact_sheets(suite_dir)
            verdict = build_qualification_summary(suite_dir)
        except Exception as error:  # noqa: BLE001 - producer boundary
            reason = f"stage F generation failed: {error}"
            return {
                "stage": "F",
                "status": STATUS_EVIDENCE_INVALID,
                "reasons": [reason],
                "suite_dir": str(suite_dir),
                "index": str(index_path),
                "summary": str(summary_path),
                "agent_sheet": str(agent_sheet_path),
                "user_sheet": str(user_sheet_path),
                "checksum_algorithm": checksum_algorithm,
                "cameras": cameras,
                "evidence": {
                    "status": STATUS_EVIDENCE_INVALID,
                    "producer_exception": True,
                    "reasons": [reason],
                },
            }
        status = str(verdict.get("status", STATUS_VERIFIED_FAIL))
        if status not in {STATUS_VERIFIED_PASS, STATUS_VERIFIED_FAIL}:
            status = STATUS_VERIFIED_FAIL
        return {
            "stage": "F",
            "status": status,
            "reasons": [str(reason) for reason in verdict.get("reasons", [])],
            "suite_dir": str(suite_dir),
            "index": str(index_path),
            "summary": str(summary_path),
            "agent_sheet": str(agent_sheet_path),
            "user_sheet": str(user_sheet_path),
            "checksum_algorithm": checksum_algorithm,
            "cameras": cameras,
            "evidence": dict(verdict),
        }

    def _run_all(self) -> dict[str, Any]:
        a = self._run_stage_a()
        self._stage_results["A"] = a
        b = self._run_stage_b()
        self._stage_results["B"] = b
        if (
            a.get("status") != STATUS_VERIFIED_PASS
            or b.get("status") != STATUS_VERIFIED_PASS
        ):
            return {
                "A": a,
                "B": b,
                **{
                    name: {"status": STATUS_BLOCKED}
                    for name in ("C", "D", "E", "F")
                },
            }
        c = self._run_stage_c()
        self._stage_results["C"] = c
        d = self._run_stage_d()
        self._stage_results["D"] = d
        e = self._run_stage_e()
        self._stage_results["E"] = e
        # Stage F cross-checks the persisted C/D/E records against the
        # in-memory results, so C/D/E must be registered before F runs.
        f = self._run_stage_f()
        self._stage_results["F"] = f
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


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def _overall_status(result: Mapping[str, Any]) -> str:
    """Fail-dominant overall status for ``--stage all``.

    Stage A is always retained.  ``blocked-by-gate-b`` child placeholders are
    diagnostic consequences: the underlying Gate B status is the overall
    failure cause.  ``not-implemented`` is treated only as a backward-
    compatibility status for older stage records; current Stage F always
    reports ``verified-pass``, ``verified-fail``, or ``evidence-invalid``.
    """
    statuses: list[str] = []
    for key in ("A", "B", "C", "D", "E", "F"):
        value = result.get(key)
        if isinstance(value, Mapping):
            statuses.append(str(value.get("status", "")))
    if not statuses:
        return STATUS_VERIFIED_FAIL
    if STATUS_EVIDENCE_INVALID in statuses:
        return STATUS_EVIDENCE_INVALID
    if STATUS_VERIFIED_FAIL in statuses:
        return STATUS_VERIFIED_FAIL
    if STATUS_BLOCKED in statuses:
        b = result.get("B")
        if isinstance(b, Mapping):
            b_status = str(b.get("status", ""))
            if b_status in {STATUS_EVIDENCE_INVALID, STATUS_VERIFIED_FAIL}:
                return b_status
        return STATUS_BLOCKED
    if STATUS_NOT_IMPLEMENTED in statuses:
        if all(
            status in {STATUS_VERIFIED_PASS, STATUS_NOT_IMPLEMENTED}
            for status in statuses
        ):
            return STATUS_NOT_IMPLEMENTED
        return STATUS_VERIFIED_FAIL
    if all(status == STATUS_VERIFIED_PASS for status in statuses):
        return STATUS_VERIFIED_PASS
    return STATUS_VERIFIED_FAIL


def _exit_code_for_status(status: str) -> int:
    if status == STATUS_VERIFIED_PASS:
        return 0
    if status in {STATUS_EVIDENCE_INVALID, STATUS_NOT_IMPLEMENTED}:
        return 2
    return 1


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Orchestrate the integrated OMPL qualification Gates A-F."
    )
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--production-root", type=Path, default=DEFAULT_PRODUCTION_ROOT)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--stage", default="all", choices=["A", "B", "C", "D", "E", "F", "all"])
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--attempt-root", type=Path, default=DEFAULT_ATTEMPT_ROOT)
    parser.add_argument("--base-domain-id", type=int, default=100)
    parser.add_argument("--readiness-timeout", type=float, default=30.0)
    # F2.5: optional override for deterministic tests; the normal CLI/config path
    # leaves this None and derives the terminal budget from config thresholds.
    parser.add_argument("--terminal-timeout", type=float, default=None)
    parser.add_argument("--model-bundle-manifest", type=Path)
    parser.add_argument("--provider-manifest-path", type=Path)
    parser.add_argument("--isaac-command", help="override Isaac wrapper command")
    parser.add_argument("--humble-command", help="override Humble wrapper command")
    parser.add_argument(
        "--offline",
        action="store_true",
        help=(
            "Stage-B compatibility flag (Stage B is already offline and never "
            "launches live processes)"
        ),
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.offline and str(args.stage).upper() != "B":
        print(
            json.dumps(
                {
                    "status": STATUS_EVIDENCE_INVALID,
                    "reasons": [
                        "--offline is accepted only as a Stage-B compatibility "
                        "flag; it must not make a live stage offline or bypass checks"
                    ],
                },
                sort_keys=True,
                indent=2,
            )
        )
        return _exit_code_for_status(STATUS_EVIDENCE_INVALID)

    runner = IntegratedRunner(
        root=args.root,
        production_root=args.production_root,
        config_path=args.config,
        seed=args.seed,
        attempt_root=args.attempt_root,
        base_domain_id=args.base_domain_id,
        readiness_timeout_s=args.readiness_timeout,
        terminal_timeout_s=args.terminal_timeout,
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
