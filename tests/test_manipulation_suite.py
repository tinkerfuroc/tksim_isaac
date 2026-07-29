from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from validation.manipulation_qualification import (
    GATES,
    QualificationResult,
    QualificationRunner,
    _run_suite,
)


class ManipulationSuiteTest(unittest.TestCase):
    def _root(self, temporary: str) -> Path:
        root = Path(temporary)
        config = root / "simulation/qualification/manipulation-core.json"
        config.parent.mkdir(parents=True)
        scenarios: dict[str, str] = {}
        for gate in GATES:
            name = f"qualification-{gate}"
            scenarios[gate] = name
            path = root / "simulation/scenarios" / f"{name}.json"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                json.dumps(
                    {
                        "schema_version": 2,
                        "id": name,
                        "seed": 7,
                        "actors": [],
                        "objects": [],
                    }
                ),
                encoding="utf-8",
            )
        config.write_text(
            json.dumps({"gates": list(GATES), "scenarios": scenarios}),
            encoding="utf-8",
        )
        generator = root / "validation/manipulation_contact_sheets.py"
        generator.parent.mkdir(parents=True)
        generator.write_text("# placeholder\n", encoding="utf-8")
        return root

    def test_suite_isolates_domains_and_aggregates_all_six_gates(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = self._root(temporary)
            invoked: list[tuple[str, str]] = []

            def fake_run(runner: QualificationRunner):
                attempt = runner.attempt_root / "attempt"
                attempt.mkdir(parents=True)
                invoked.append((runner.gate, str(runner.ros_domain_id)))
                return QualificationResult(
                    attempt,
                    "verified-pass",
                    {
                        runner.gate: {
                            "gate": runner.gate,
                            "status": "verified-pass",
                            "pass": True,
                        }
                    },
                    {},
                )

            def fake_subprocess(command, **_kwargs):
                suite_dir = Path(command[-1])
                (suite_dir / "visual-evidence-result.json").write_text(
                    json.dumps({"status": "valid"}),
                    encoding="utf-8",
                )
                (suite_dir / "contact-sheet-agent.png").write_bytes(b"agent")
                (suite_dir / "contact-sheet-user.png").write_bytes(b"user")
                return SimpleNamespace(returncode=0, stdout="", stderr="")

            with patch.object(
                QualificationRunner, "run", autospec=True, side_effect=fake_run
            ), patch(
                "validation.manipulation_qualification.subprocess.run",
                side_effect=fake_subprocess,
            ):
                result = _run_suite(
                    root=root,
                    attempt_root=root / "attempts",
                    config_path=None,
                    artifact_path=None,
                    seed=7,
                    readiness_timeout_s=1.0,
                    isaac_command=None,
                    humble_command=None,
                    gate_commands={},
                    base_domain_id=100,
                )

            self.assertEqual(result.status, "verified-pass")
            self.assertEqual([gate for gate, _domain in invoked], list(GATES))
            self.assertEqual(
                [domain for _gate, domain in invoked],
                [str(value) for value in range(100, 106)],
            )
            suite = json.loads(
                (result.attempt_dir / "suite-result.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(suite["status"], "verified-pass")
            self.assertEqual(set(suite["gates"]), set(GATES))

    def test_single_gate_defaults_to_its_configured_scenario(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = self._root(temporary)

            runner = QualificationRunner(root=root, gate="safety-stop")

            self.assertEqual(
                runner.scenario_path.name, "qualification-safety-stop.json"
            )
            self.assertIn(
                "qualification-safety-stop", runner._default_isaac_command()
            )

    def test_ordinary_gate_failure_does_not_skip_remaining_gates(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = self._root(temporary)
            invoked: list[str] = []

            def fake_run(runner: QualificationRunner):
                attempt = runner.attempt_root / "attempt"
                attempt.mkdir(parents=True)
                invoked.append(runner.gate)
                status = (
                    "verified-fail"
                    if runner.gate == "obstructed-gripper"
                    else "verified-pass"
                )
                return QualificationResult(
                    attempt,
                    status,
                    {runner.gate: {"gate": runner.gate, "status": status}},
                    {},
                )

            def fake_subprocess(command, **_kwargs):
                suite_dir = Path(command[-1])
                (suite_dir / "visual-evidence-result.json").write_text(
                    json.dumps({"status": "valid"}),
                    encoding="utf-8",
                )
                return SimpleNamespace(returncode=0, stdout="", stderr="")

            with patch.object(
                QualificationRunner, "run", autospec=True, side_effect=fake_run
            ), patch(
                "validation.manipulation_qualification.subprocess.run",
                side_effect=fake_subprocess,
            ):
                result = _run_suite(
                    root=root,
                    attempt_root=root / "attempts",
                    config_path=None,
                    artifact_path=None,
                    seed=7,
                    readiness_timeout_s=1.0,
                    isaac_command=None,
                    humble_command=None,
                    gate_commands={},
                    base_domain_id=20,
                )

            self.assertEqual(invoked, list(GATES))
            self.assertEqual(result.status, "verified-fail")


if __name__ == "__main__":
    unittest.main()
