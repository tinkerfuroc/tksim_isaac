from __future__ import annotations

import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_design, two_arm_urdf
import design_import
from design_import import EXIT_CONTRACT, EXIT_IMPORT, EXIT_RENDER, ImportResult, main, run_import, write_init


class StubHooks:
    def __init__(self, fail: bool = False) -> None:
        self.calls: list[tuple[Path, Path]] = []
        self.fail = fail

    def import_urdf(self, urdf_path: Path, usd_path: Path) -> None:
        self.calls.append((urdf_path, usd_path))
        if self.fail:
            raise RuntimeError("kit exploded")
        usd_path.write_bytes(b"#usda 1.0\n" + urdf_path.read_bytes()[:32])


def _make_design(repo: Path, name: str = "two_arm_fixture", urdf: bytes | None = None, design: dict | None = None) -> Path:
    design_dir = repo / "designs" / name
    design_dir.mkdir(parents=True)
    (design_dir / "robot.urdf").write_bytes(urdf or two_arm_urdf())
    raw = design or two_arm_design()
    raw["name"] = name
    (design_dir / "design.yaml").write_text(yaml.safe_dump(raw), encoding="utf-8")
    return design_dir


class RunImportTest(unittest.TestCase):
    def test_full_pipeline_with_stub_hooks_publishes_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = _make_design(repo)
            hooks = StubHooks()
            result = run_import(design_dir, repo, hooks, packages={})
            self.assertIsInstance(result, ImportResult)
            self.assertEqual(len(hooks.calls), 1)
            self.assertEqual(hooks.calls[0][0].name, "robot.isaac.urdf")
            artifact_dir = result.artifact_dir
            self.assertEqual(artifact_dir.parent, repo / "artifacts" / "robot" / "two_arm_fixture")
            for name in ("robot.urdf", "robot.usd", "robot-profile.yaml", "manifest.json", "source-lock.json"):
                self.assertTrue((artifact_dir / name).is_file(), name)
            profile = yaml.safe_load((artifact_dir / "robot-profile.yaml").read_text())
            self.assertEqual(profile["robot"], "two_arm_fixture")
            self.assertAlmostEqual(profile["wheels"]["track_m"], 0.4)
            self.assertEqual(len(profile["arms"]), 2)
            manifest = json.loads((artifact_dir / "manifest.json").read_text())
            self.assertEqual(manifest["qualification"], "design_candidate")
            self.assertEqual(manifest["kinematics"]["front_left_joint"], "front_left_wheel_joint")
            self.assertEqual(manifest["kinematics"]["wheel_track_m"], profile["wheels"]["track_m"])
            self.assertEqual(manifest["profile"]["mass_kg"], profile["mass_kg"])
            self.assertEqual(manifest["provenance"]["design_dir"], "designs/two_arm_fixture")
            current = json.loads((repo / "artifacts" / "robot" / "two_arm_fixture" / "current.json").read_text())
            self.assertEqual(current["artifact_id"], artifact_dir.name)
            self.assertEqual((artifact_dir / "robot.urdf").read_bytes(), result.canonical_urdf)

    def test_no_import_stops_before_hooks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            hooks = StubHooks()
            result = run_import(_make_design(repo), repo, hooks, no_import=True, packages={})
            self.assertIsNone(result.artifact_dir)
            self.assertEqual(hooks.calls, [])
            self.assertAlmostEqual(result.profile["mass_kg"], 32.05)

    def test_contract_failure_exits_3_listing_all_violations(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            raw = two_arm_design()
            raw["pan_tilt"] = None
            design_dir = _make_design(repo, design=raw)
            err = io.StringIO()
            with redirect_stdout(err):
                code = main(["--design", str(design_dir), "--stub-converter", "--artifacts", str(repo / "artifacts")])
            self.assertEqual(code, EXIT_CONTRACT)
            self.assertIn("pan_joint", err.getvalue())
            self.assertIn("tilt_joint", err.getvalue())
            self.assertFalse((repo / "artifacts").exists())

    def test_unresolved_mesh_exits_2(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            urdf = two_arm_urdf().replace(b'<box size="0.06 0.1 0.06" />', b'<mesh filename="package://nope/x.stl" />')
            design_dir = _make_design(repo, urdf=urdf)
            out = io.StringIO()
            with redirect_stdout(out):
                code = main(["--design", str(design_dir), "--stub-converter", "--artifacts", str(repo / "artifacts")])
            self.assertEqual(code, EXIT_RENDER)
            self.assertIn("nope", out.getvalue())

    def test_import_failure_exits_4_and_publishes_nothing(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = _make_design(repo)
            with self.assertRaises(design_import.ImportStageError):
                run_import(design_dir, repo, StubHooks(fail=True), packages={})
            self.assertFalse((repo / "artifacts" / "robot").exists())

    def test_main_stub_converter_prints_artifact_dir(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = _make_design(repo)
            out = io.StringIO()
            with redirect_stdout(out):
                code = main(["--design", str(design_dir), "--stub-converter", "--artifacts", str(repo / "artifacts")])
            self.assertEqual(code, 0)
            printed = Path(out.getvalue().strip().splitlines()[-1])
            self.assertTrue((printed / "manifest.json").is_file())

    def test_init_writes_design_yaml_and_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            design_dir = Path(temporary) / "two_arm_fixture"
            design_dir.mkdir()
            (design_dir / "robot.urdf").write_bytes(two_arm_urdf())
            path = write_init(design_dir, "two_arm_fixture")
            raw = yaml.safe_load(path.read_text())
            self.assertEqual(raw["wheels"]["driven"], ["front_left_wheel_joint", "front_right_wheel_joint"])
            with self.assertRaises(FileExistsError):
                write_init(design_dir, "two_arm_fixture")
            code = main(["--design", str(design_dir), "--init"])
            self.assertNotEqual(code, 0)

    def test_malformed_package_root_is_a_usage_error(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            design_dir = _make_design(repo)
            with redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as caught:
                    main(["--design", str(design_dir), "--no-import", "--package-root", "nope"])
            self.assertEqual(caught.exception.code, 2)


class DesignConvertImportTest(unittest.TestCase):
    def test_module_imports_without_isaac(self) -> None:
        before = {name for name in sys.modules if name.startswith(("isaacsim", "omni", "pxr"))}
        import design_convert
        after = {name for name in sys.modules if name.startswith(("isaacsim", "omni", "pxr"))}
        self.assertTrue(hasattr(design_convert, "IsaacHooks"))
        self.assertEqual(after, before, "design_convert must import Isaac only inside IsaacHooks.import_urdf")


if __name__ == "__main__":
    unittest.main()
