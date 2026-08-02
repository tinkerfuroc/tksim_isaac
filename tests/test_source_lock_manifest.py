"""Task 2: three-policy source-lock manifest observer tests.

These tests construct three independent authorization policies in two real
temporary Git repositories (the simulator repository carries both the
historical overlay transition lock and the future qualification-tooling lock;
the production repository carries the unique production lock).  No test uses a
mocked path claim or a self-observed hash.

Realistic simulator history (matching the review-clean ``ab8cf7e`` /
``490f907`` overlay and the post-Task-10 qualification lock):

* baseline commit introduces ``integration/source-locks.json`` (non-OMPL
  schema) -- the pre-existing/deployment path history;
* overlay implementation commit (overlay ``implementation_head``);
* overlay transition lock commit rewrites ``integration/source-locks.json``
  with the OMPL authorization fields -- raw path-history count two;
* qualification implementation descendant;
* qualification lock-only commit at ``integration/integrated-qualification-source-lock.json``
  -- raw path-history count one; checked-out HEAD at the qualification lock.

Production history (matching the review-clean ``1e24826`` / ``39d96a1``):

* production implementation commit (production ``implementation_head``);
* production policy lock-only commit at ``integration/source-locks.json`` --
  raw path-history count one; checked-out HEAD at the production lock.

Every commit uses a fixed past author/committer date so the resolved policy
commits always predate the attempt-start timestamp.
"""
from __future__ import annotations

import base64
import hashlib
import json
import os
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))

from source_lock_manifest import (  # noqa: E402
    ROLES,
    capture_manifest,
)

FIXED_COMMIT_DATE = "2026-07-01T00:00:00Z"
AUTHORIZATION_FIELDS = ("repository", "implementation_head", "policy_commit_resolution")
HEX40 = lambda value: isinstance(value, str) and len(value) == 40 and all(c in "0123456789abcdef" for c in value)  # noqa: E731


# ---------------------------------------------------------------------------
# git helpers
# ---------------------------------------------------------------------------
def _git(root: Path, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
    proc = subprocess.run(
        ["git", "-C", str(root), *args],
        capture_output=True,
        text=True,
        env={**os.environ, "LC_ALL": "C", "GIT_AUTHOR_DATE": FIXED_COMMIT_DATE, "GIT_COMMITTER_DATE": FIXED_COMMIT_DATE},
        check=False,
    )
    if check and proc.returncode != 0:
        raise AssertionError("git {} failed in {}: {}".format(" ".join(args), root, proc.stderr))
    return proc


def _git_bytes(root: Path, *args: str) -> bytes:
    proc = subprocess.run(
        ["git", "-C", str(root), *args],
        capture_output=True,
        env={**os.environ, "LC_ALL": "C"},
        check=False,
    )
    if proc.returncode != 0:
        raise AssertionError("git {} failed in {}: {}".format(" ".join(args), root, proc.stderr.decode(errors="replace")))
    return proc.stdout


def _git_head(root: Path) -> str:
    return _git(root, "rev-parse", "HEAD").stdout.strip()


def _git_init(root: Path) -> None:
    root.mkdir(parents=True, exist_ok=True)
    _git(root, "init", "-q")
    _git(root, "config", "user.email", "fixture@example.invalid")
    _git(root, "config", "user.name", "Fixture")


def _write(root: Path, relative: str, content: bytes | str) -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content if isinstance(content, bytes) else content.encode("utf-8"))
    return path


def _write_json_canonical(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, sort_keys=True, separators=(",", ":")), encoding="utf-8"
    )


def _read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def _commit(root: Path, message: str) -> None:
    _git(root, "add", "-A")
    _git(root, "commit", "-q", "-m", message)


def _commit_paths(root: Path, paths: list[str], message: str) -> None:
    """Stage and commit only the named paths (never sweeps the working tree)."""
    _git(root, "add", "--", *paths)
    _git(root, "commit", "-q", "-m", message)


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _untracked_manifest(status_bytes: bytes, root: Path) -> list[dict[str, object]]:
    entries: list[dict[str, object]] = []
    for record in status_bytes.split(b"\x00"):
        if not record.startswith(b"?? "):
            continue
        rel = record[3:].decode("utf-8", errors="surrogateescape")
        full = root / rel
        st = os.lstat(full)
        import stat as stat_module
        if stat_module.S_ISREG(st.st_mode):
            data = full.read_bytes()
            entries.append(
                {
                    "kind": "regular",
                    "mode": "100755" if (st.st_mode & 0o111) else "100644",
                    "path": rel,
                    "sha256": hashlib.sha256(data).hexdigest(),
                    "size": len(data),
                }
            )
        elif stat_module.S_ISLNK(st.st_mode):
            target = os.readlink(full).encode("utf-8", errors="surrogateescape")
            entries.append(
                {
                    "kind": "symlink",
                    "mode": "120000",
                    "path": rel,
                    "sha256": hashlib.sha256(target).hexdigest(),
                    "size": len(target),
                }
            )
        else:
            raise AssertionError("untracked entry is not regular/symlink: {}".format(rel))
    entries.sort(key=lambda entry: entry["path"])
    return entries


def _capture_evidence(root: Path) -> dict[str, object]:
    status = _git_bytes(root, "status", "--porcelain=v1", "-z", "--untracked-files=all")
    diff = _git_bytes(root, "diff", "--binary", "--no-ext-diff")
    manifest = _untracked_manifest(status, root)
    manifest_sha = _sha256(
        json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode("utf-8")
    )
    return {
        "status_bytes": {"encoding": "base64", "data": base64.b64encode(status).decode("ascii")},
        "diff_bytes": {"encoding": "base64", "data": base64.b64encode(diff).decode("ascii")},
        "status_sha256": _sha256(status),
        "diff_sha256": _sha256(diff),
        "untracked_manifest": manifest,
        "untracked_manifest_sha256": manifest_sha,
    }


def _policy_document(
    *,
    repository: str,
    root: Path,
    policy_rel: str,
    mode: str,
    implementation_head: str,
    authorization_commit: str | None = None,
    report_path: str | None = None,
    report_symlink_target: str | None = None,
) -> dict[str, object]:
    # F3.3: the authorization report must exist as a regular file that predates
    # the attempt.  The real repos gitignore their .superpowers sidecars; the
    # fixture mirrors that so the report is not an untracked evidence surface.
    # F2.4.2: ``report_path`` records an arbitrary authorization.report_path
    # (default the fixture sidecar).  A symlink target may be supplied to
    # exercise the canonical-escape containment rule; the report file is only
    # created when it actually resolves inside the repository root.
    report_rel = report_path if report_path is not None else ".superpowers/fixture-lock-report.md"
    report_target = root / report_rel
    try:
        report_target.resolve().relative_to(root.resolve())
        contained = True
    except ValueError:
        contained = False
    if report_symlink_target is not None:
        # Create the symlink regardless of where it points; it is the escape
        # surface the F2.4.2 containment rule must reject.
        report_target.parent.mkdir(parents=True, exist_ok=True)
        ignore = report_target.parent / ".gitignore"
        if not ignore.exists():
            ignore.write_text("*\n", encoding="utf-8")
        if report_target.exists() or report_target.is_symlink():
            report_target.unlink()
        os.symlink(report_symlink_target, report_target)
    elif contained:
        report_target.parent.mkdir(parents=True, exist_ok=True)
        ignore = report_target.parent / ".gitignore"
        if not ignore.exists():
            ignore.write_text("*\n", encoding="utf-8")
        report_target.write_text("fixture authorization report\n", encoding="utf-8")
        past = datetime(2026, 7, 1, tzinfo=timezone.utc).timestamp()
        os.utime(report_target, (past, past))

    evidence = _capture_evidence(root)
    return {
        "schema_version": 1,
        "repository": repository,
        "root": str(root),
        "implementation_head": implementation_head,
        "policy_path": policy_rel,
        "policy_commit_resolution": "commit_containing_policy_path",
        "mode": mode,
        "status_sha256": evidence["status_sha256"],
        "diff_sha256": evidence["diff_sha256"],
        "untracked_manifest_sha256": evidence["untracked_manifest_sha256"],
        "status_bytes": evidence["status_bytes"],
        "diff_bytes": evidence["diff_bytes"],
        "untracked_manifest": evidence["untracked_manifest"],
        "authorization": {
            "commit": authorization_commit,
            "phase": "fixture-lock-only",
            "report_path": report_rel,
        },
        "capture_commands": [
            ["env", "LC_ALL=C", "git", "status", "--porcelain=v1", "-z", "--untracked-files=all"],
            ["env", "LC_ALL=C", "git", "diff", "--binary", "--no-ext-diff"],
        ],
    }


def _commit_policy(
    root: Path,
    policy_path: Path,
    policy: dict[str, object],
    message: str,
    *,
    extra_paths: tuple[str, ...] = (),
) -> None:
    _write_json_canonical(policy_path, policy)
    _git(root, "add", "--", str(policy_path))
    for extra in extra_paths:
        _git(root, "add", "--", extra)
    _git(root, "commit", "-q", "-m", message)


def _apply_dirty_state(root: Path) -> None:
    """Create the authorized dirty state: modify a tracked file and add
    untracked regular/executable/symlink entries."""
    tracked = root / "README.md"
    tracked.write_text("fixture with an authorized local modification\n", encoding="utf-8")
    _write(root, "untracked/regular.txt", b"regular untracked")
    executable = _write(root, "untracked/run.sh", b"#!/bin/sh\necho run\n")
    executable.chmod(0o755)
    symlink = root / "untracked/link"
    if symlink.exists() or symlink.is_symlink():
        symlink.unlink()
    os.symlink("regular.txt", symlink)


# ---------------------------------------------------------------------------
# fixture construction
# ---------------------------------------------------------------------------
def create_git_fixture_repositories(
    simulator_root: Path,
    production_root: Path,
    *,
    mode: str,
    with_qualification_policy: bool = True,
    overlay_lock_extra_paths: tuple[str, ...] = (),
    production_lock_extra_paths: tuple[str, ...] = (),
    production_authorization_commit: str | None = None,
    production_mode: str | None = None,
    qualification_mode: str | None = None,
    production_report_path: str | None = None,
    production_report_symlink_target: str | None = None,
) -> None:
    """Build both repositories with realistic lock-only history.

    ``*_lock_extra_paths`` lets a test bundle extra paths into a lock-only
    commit (e.g. ``("docs/acceptance.md",)`` allowed, or ``("src/app.py",)``
    rejected) to exercise the resolved-commit scope check (F3.1).

    ``production_authorization_commit`` overrides the production policy's
    ``authorization.commit`` (F3.3): ``"bogus"`` creates a diverged side commit
    that is NOT an ancestor of the production lock commit; any other non-None
    value is used verbatim (``"impl"`` = the production implementation head, a
    valid ancestor).  ``None`` keeps ``authorization.commit: null``.

    ``production_mode`` / ``qualification_mode`` override the mode recorded for
    the production and qualification-tooling policies independently (F2.4.1:
    the simulator repository is shared, so the qualification policy may be
    clean only when the simulator repo itself is clean).

    ``production_report_path`` / ``production_report_symlink_target`` override
    the production policy's ``authorization.report_path`` (F2.4.2 containment):
    the recorded path must resolve to a regular file inside the repository, so
    an absolute external path or a symlink escaping the repo fails closed.
    """
    _git_init(simulator_root)
    _git_init(production_root)

    # ---- simulator ---------------------------------------------------------
    _write(simulator_root, "README.md", "simulator fixture\n")
    _write(
        simulator_root,
        "integration/source-locks.json",
        json.dumps({"schema_version": 0, "note": "pre-existing baseline"}),
    )
    _write(simulator_root, "docs/acceptance.md", "simulator acceptance baseline\n")
    _commit(simulator_root, "chore: establish simulator baseline")
    overlay_impl_head = _git_head(simulator_root)

    _write(simulator_root, "simulation/app.py", "def overlay_implementation():\n    pass\n")
    _commit(simulator_root, "feat: overlay implementation")
    overlay_impl_head = _git_head(simulator_root)

    if mode == "authorized_dirty":
        _apply_dirty_state(simulator_root)
    overlay_policy = _policy_document(
        repository="simulator",
        root=simulator_root,
        policy_rel="integration/source-locks.json",
        mode=mode,
        implementation_head=overlay_impl_head,
    )
    if "docs/acceptance.md" in overlay_lock_extra_paths:
        (simulator_root / "docs/acceptance.md").write_text(
            "overlay acceptance doc lock companion\n", encoding="utf-8"
        )
    if "src/app.py" in overlay_lock_extra_paths:
        (simulator_root / "src/app.py").write_text("bundled source payload\n", encoding="utf-8")
    _commit_policy(
        simulator_root,
        simulator_root / "integration/source-locks.json",
        overlay_policy,
        "chore: record simulator OMPL source lock",
        extra_paths=overlay_lock_extra_paths,
    )

    qualification_impl_head = None
    if with_qualification_policy:
        _write(simulator_root, "simulation/qual.py", "def qualification_implementation():\n    pass\n")
        # Surgical commit: never sweep up the overlay's authorized dirty working
        # tree into the qualification implementation commit.
        _commit_paths(simulator_root, ["simulation/qual.py"], "feat: integrated qualification implementation")
        qualification_impl_head = _git_head(simulator_root)
        qualification_policy = _policy_document(
            repository="simulator",
            root=simulator_root,
            policy_rel="integration/integrated-qualification-source-lock.json",
            mode=qualification_mode or mode,
            implementation_head=qualification_impl_head,
        )
        _commit_policy(
            simulator_root,
            simulator_root / "integration/integrated-qualification-source-lock.json",
            qualification_policy,
            "chore: record integrated qualification source lock",
        )

    # ---- production ---------------------------------------------------------
    _write(production_root, "README.md", "production fixture\n")
    _write(production_root, "src/mobile_bringup/launch/planning.launch.py", "def production_launch():\n    pass\n")
    _write(production_root, "docs/acceptance.md", "production acceptance baseline\n")
    _commit(production_root, "feat: production planning task-only launch")
    production_impl_head = _git_head(production_root)

    production_auth_commit: str | None
    if production_authorization_commit == "bogus":
        # A diverged side commit that is NOT an ancestor of the eventual
        # production lock commit.  The working tree is clean here, so the
        # checkout back to the implementation head restores a clean state.
        _git(production_root, "checkout", "-q", "-b", "side")
        _write(production_root, "src/side.py", "diverged side change\n")
        _commit(production_root, "chore: diverged side commit")
        side_commit = _git_head(production_root)
        _git(production_root, "checkout", "-q", production_impl_head)
        production_auth_commit = side_commit
    elif production_authorization_commit is None:
        production_auth_commit = None
    elif production_authorization_commit == "impl":
        production_auth_commit = production_impl_head
    else:
        production_auth_commit = production_authorization_commit

    prod_mode = production_mode or mode
    if prod_mode == "authorized_dirty":
        _apply_dirty_state(production_root)
    production_policy = _policy_document(
        repository="production",
        root=production_root,
        policy_rel="integration/source-locks.json",
        mode=prod_mode,
        implementation_head=production_impl_head,
        report_path=production_report_path,
        report_symlink_target=production_report_symlink_target,
        authorization_commit=production_auth_commit,
    )
    if "docs/acceptance.md" in production_lock_extra_paths:
        (production_root / "docs/acceptance.md").write_text(
            "production acceptance doc lock companion\n", encoding="utf-8"
        )
    if "src/app.py" in production_lock_extra_paths:
        (production_root / "src/app.py").write_text("bundled source payload\n", encoding="utf-8")
    _commit_policy(
        production_root,
        production_root / "integration/source-locks.json",
        production_policy,
        "chore: record production OMPL source lock",
        extra_paths=production_lock_extra_paths,
    )


def write_authorization_policy(
    policy_path: Path,
    *,
    repository: str,
    root: Path,
    mode: str,
    policy_after_attempt: bool = False,
    attempt_started_at: datetime | None = None,
) -> None:
    """Re-write an already-committed policy with identical canonical bytes.

    Optionally sets the file mtime to just after the attempt start so the
    freshness check fails (evidence-invalid).
    """
    committed = _read_json(policy_path)
    assert committed.get("repository") == repository
    assert committed.get("mode") == mode
    _write_json_canonical(policy_path, committed)
    if policy_after_attempt:
        assert attempt_started_at is not None
        future = attempt_started_at.timestamp() + 3600.0
        os.utime(policy_path, (future, future))
    else:
        # Policy mtime must predate the attempt start (freshness contract).
        past = datetime(2026, 7, 1, tzinfo=timezone.utc).timestamp()
        os.utime(policy_path, (past, past))


def capture_source_lock_manifest(
    *,
    simulator_root: Path,
    production_root: Path,
    simulator_policy_path: Path,
    production_policy_path: Path,
    qualification_policy_path: Path,
    attempt_started_at: datetime,
    output: Path,
) -> dict[str, object]:
    return capture_manifest(
        simulator_root=simulator_root,
        production_root=production_root,
        simulator_policy=simulator_policy_path,
        production_policy=production_policy_path,
        qualification_policy=qualification_policy_path,
        attempt_started_at=attempt_started_at,
        output=output,
    )


@dataclass(frozen=True)
class SourceLockFixture:
    simulator_root: Path
    production_root: Path
    simulator_policy_path: Path
    production_policy_path: Path
    qualification_policy_path: Path
    output: Path
    attempt_started_at: datetime

    def add_untracked(self, name: str, content: bytes) -> None:
        path = self.production_root / name
        path.write_bytes(content)


def make_source_lock_fixture(
    tmp_path: Path,
    *,
    mode: str = "clean",
    policy_after_attempt: bool = False,
    with_qualification_policy: bool = True,
    overlay_lock_extra_paths: tuple[str, ...] = (),
    production_lock_extra_paths: tuple[str, ...] = (),
    production_authorization_commit: str | None = None,
    production_mode: str | None = None,
    qualification_mode: str | None = None,
    production_report_path: str | None = None,
    production_report_symlink_target: str | None = None,
) -> SourceLockFixture:
    simulator_root = (tmp_path / "simulator").resolve()
    production_root = (tmp_path / "production").resolve()
    simulator_policy_path = simulator_root / "integration/source-locks.json"
    production_policy_path = production_root / "integration/source-locks.json"
    qualification_policy_path = (
        simulator_root / "integration/integrated-qualification-source-lock.json"
    )
    output = tmp_path / "source-lock-manifest.json"
    attempt_started_at = datetime.now(timezone.utc)
    create_git_fixture_repositories(
        simulator_root,
        production_root,
        mode=mode,
        with_qualification_policy=with_qualification_policy,
        overlay_lock_extra_paths=overlay_lock_extra_paths,
        production_lock_extra_paths=production_lock_extra_paths,
        production_authorization_commit=production_authorization_commit,
        production_mode=production_mode,
        qualification_mode=qualification_mode,
        production_report_path=production_report_path,
        production_report_symlink_target=production_report_symlink_target,
    )
    write_authorization_policy(
        simulator_policy_path,
        repository="simulator",
        root=simulator_root,
        mode=mode,
        policy_after_attempt=policy_after_attempt,
        attempt_started_at=attempt_started_at,
    )
    write_authorization_policy(
        production_policy_path,
        repository="production",
        root=production_root,
        mode=production_mode or mode,
        policy_after_attempt=policy_after_attempt,
        attempt_started_at=attempt_started_at,
    )
    if with_qualification_policy:
        write_authorization_policy(
            qualification_policy_path,
            repository="simulator",
            root=simulator_root,
            mode=qualification_mode or mode,
            policy_after_attempt=policy_after_attempt,
            attempt_started_at=attempt_started_at,
        )
    return SourceLockFixture(
        simulator_root,
        production_root,
        simulator_policy_path,
        production_policy_path,
        qualification_policy_path,
        output,
        attempt_started_at,
    )


# ---------------------------------------------------------------------------
# positive / negative tests
# ---------------------------------------------------------------------------
def _run_capture(fixture: SourceLockFixture) -> dict[str, object]:
    return capture_source_lock_manifest(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        simulator_policy_path=fixture.simulator_policy_path,
        production_policy_path=fixture.production_policy_path,
        qualification_policy_path=fixture.qualification_policy_path,
        attempt_started_at=fixture.attempt_started_at,
        output=fixture.output,
    )


def test_three_policy_verified_pass_clean(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-pass"
    for role in ROLES:
        assert observed[role]["status"] == "verified-pass"
        assert observed[role]["status_match"] is True
        assert observed[role]["diff_match"] is True
        assert observed[role]["untracked_match"] is True
        assert observed[role]["index_match"] is True
        assert observed[role]["observed_clean"] is True
        assert observed[role]["resolved_policy_commit"] != observed[role]["implementation_head"]
    assert fixture.output.is_file()
    # The output manifest postdates the attempt start.
    assert observed["output_predates_attempt"] is False


def test_three_policy_verified_pass_authorized_dirty(tmp_path):
    """Authorized-dirty is honoured where the role allows it: the production
    repository may be authorized-dirty, while the shared simulator repository
    stays clean so both simulator roles (overlay + qualification) pass.  F2.4.1:
    the qualification role itself must never be authorized-dirty."""
    fixture = make_source_lock_fixture(tmp_path, mode="clean", production_mode="authorized_dirty")
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-pass"
    for role in ROLES:
        assert observed[role]["status"] == "verified-pass"
        assert observed[role]["status_match"] is True
        assert observed[role]["diff_match"] is True
        assert observed[role]["untracked_match"] is True
        assert observed[role]["index_match"] is True
    assert observed["production"]["observed_clean"] is False
    assert observed["simulator_overlay"]["observed_clean"] is True
    assert observed["qualification_tooling"]["observed_clean"] is True
    # Exact dirty entries: regular, executable, symlink (production repo only).
    paths = {entry["path"] for entry in observed["production"]["untracked_manifest"]}
    assert "untracked/regular.txt" in paths
    assert "untracked/run.sh" in paths
    assert "untracked/link" in paths
    kinds = {entry["kind"] for entry in observed["production"]["untracked_manifest"]}
    assert kinds == {"regular", "symlink"}


def test_qualification_authorized_dirty_fails(tmp_path):
    """F2.4.1: an authorized-dirty qualification-tooling policy fails closed even
    when it is internally self-consistent."""
    fixture = make_source_lock_fixture(tmp_path, mode="authorized_dirty", qualification_mode="authorized_dirty")
    observed = _run_capture(fixture)
    assert observed["qualification_tooling"]["status"] == "verified-fail"
    assert any("mode must be clean" in reason for reason in observed["qualification_tooling"]["reasons"])


def test_absent_qualification_policy_is_evidence_invalid(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean", with_qualification_policy=False)
    observed = _run_capture(fixture)
    assert observed["status"] == "evidence-invalid"
    assert observed["qualification_tooling"]["policy_file_missing"] is True
    assert observed["simulator_overlay"]["status"] == "verified-pass"
    assert observed["production"]["status"] == "verified-pass"


def test_in_file_fake_policy_commit_is_rejected(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    policy_path = fixture.simulator_policy_path
    policy = _read_json(policy_path)
    assert "policy_commit" not in policy
    policy["policy_commit"] = "0" * 40
    _write_json_canonical(policy_path, policy)
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["simulator_overlay"]["status"] == "verified-fail"
    assert any("policy_commit" in reason for reason in observed["simulator_overlay"]["reasons"])


def test_policy_lock_only_commits_have_non_self_referential_parents(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    for policy_path in (fixture.simulator_policy_path, fixture.production_policy_path):
        policy = _read_json(policy_path)
        assert "policy_commit" not in policy
        assert HEX40(policy["implementation_head"])
        assert policy["policy_path"] == "integration/source-locks.json"
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-pass"
    for role in ROLES:
        assert observed[role]["implementation_head"] != observed[role]["resolved_policy_commit"]
        assert observed[role]["checks"]["first_parent_matches"] is True
        assert observed[role]["checks"]["resolved_ancestor_of_head"] is True
        assert observed[role]["checks"]["blob_resolved_equals_head"] is True
        assert observed[role]["checks"]["blob_resolved_equals_working"] is True


def test_current_head_cannot_substitute_for_historical_overlay_lock(tmp_path):
    """A docs descendant moves HEAD past the overlay and qualification locks;
    the observer must still resolve the historical overlay transition commit
    (ancestor-based), while qualification_tooling requires HEAD == its resolved
    lock commit and therefore fails closed on the descendant drift (F3.2)."""
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    _write(fixture.simulator_root, "docs/after.md", "docs after the qualification lock\n")
    _commit(fixture.simulator_root, "docs: harden source lock resolution")
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    # The historical overlay lock remains ancestor-based and passes.
    assert observed["simulator_overlay"]["status"] == "verified-pass"
    assert observed["simulator_overlay"]["head"] != observed["simulator_overlay"]["resolved_policy_commit"]
    assert observed["simulator_overlay"]["checks"]["blob_resolved_equals_head"] is True
    # qualification_tooling requires HEAD == resolved lock commit.
    assert observed["qualification_tooling"]["status"] == "verified-fail"
    assert observed["qualification_tooling"]["checks"]["qualification_head_matches_resolved"] is False


def test_dirty_state_cannot_self_authorize_or_change_after_authorization(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="authorized_dirty")
    fixture.add_untracked("late.txt", b"not authorized")
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["untracked_match"] is False


def test_policy_created_during_attempt_is_evidence_invalid(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean", policy_after_attempt=True)
    observed = _run_capture(fixture)
    assert observed["status"] == "evidence-invalid"
    assert observed["simulator_overlay"]["checks"]["policy_file_predates_attempt"] is False


def test_staged_index_change_fails(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    _write(fixture.production_root, "staged.txt", b"staged change\n")
    _git(fixture.production_root, "add", "--", "staged.txt")
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["index_match"] is False


def test_later_policy_rewrite_fails(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    policy_path = fixture.production_policy_path
    policy = _read_json(policy_path)
    policy["status_sha256"] = "f" * 64
    _write_json_canonical(policy_path, policy)
    _git(fixture.production_root, "add", "--", str(policy_path))
    _git(fixture.production_root, "commit", "-q", "-m", "chore: rewrite production source lock")
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["status"] == "verified-fail"
    assert observed["production"]["checks"]["history"] is False


def test_rewrite_then_revert_fails(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    policy_path = fixture.production_policy_path
    policy = _read_json(policy_path)
    original = json.dumps(policy, sort_keys=True, separators=(",", ":"))
    policy["status_sha256"] = "f" * 64
    _write_json_canonical(policy_path, policy)
    _git(fixture.production_root, "add", "--", str(policy_path))
    _git(fixture.production_root, "commit", "-q", "-m", "chore: rewrite then revert")
    _write_json_canonical(policy_path, json.loads(original))
    _git(fixture.production_root, "add", "--", str(policy_path))
    _git(fixture.production_root, "commit", "-q", "-m", "chore: revert rewrite")
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["checks"]["history"] is False


def test_deleted_and_recreated_path_fails(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    policy_path = fixture.production_policy_path
    policy = _read_json(policy_path)
    _git(fixture.production_root, "rm", "-q", str(policy_path))
    _git(fixture.production_root, "commit", "-q", "-m", "chore: delete policy")
    _write_json_canonical(policy_path, policy)
    _git(fixture.production_root, "add", "--", str(policy_path))
    _git(fixture.production_root, "commit", "-q", "-m", "chore: recreate policy")
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["checks"]["history"] is False


def test_wrong_parent_fails(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    # Rewrite the production lock with a wrong implementation_head parent claim.
    policy_path = fixture.production_policy_path
    policy = _read_json(policy_path)
    policy["implementation_head"] = "0" * 40
    _write_json_canonical(policy_path, policy)
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["checks"]["first_parent_matches"] is False


def test_cross_root_rejected(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    policy_path = fixture.production_policy_path
    policy = _read_json(policy_path)
    policy["root"] = str(fixture.simulator_root)
    _write_json_canonical(policy_path, policy)
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["status"] == "verified-fail"
    assert any("root" in reason for reason in observed["production"]["reasons"])


def test_all_zero_hash_rejected(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    policy_path = fixture.simulator_policy_path
    policy = _read_json(policy_path)
    policy["status_sha256"] = "0" * 64
    _write_json_canonical(policy_path, policy)
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert any("all-zero" in reason for reason in observed["simulator_overlay"]["reasons"])


def test_blob_disagreement_fails(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    policy_path = fixture.simulator_policy_path
    policy = _read_json(policy_path)
    policy["status_sha256"] = "a" * 64
    _write_json_canonical(policy_path, policy)
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["simulator_overlay"]["checks"]["blob_resolved_equals_working"] is False


def test_ambiguous_overlay_transition_fails(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    # Add a second commit that also qualifies as an overlay schema transition:
    # rewrite the policy with the auth fields (parent is the first transition).
    policy_path = fixture.simulator_policy_path
    policy = _read_json(policy_path)
    policy["status_sha256"] = "b" * 64
    _write_json_canonical(policy_path, policy)
    _git(fixture.simulator_root, "add", "--", str(policy_path))
    _git(fixture.simulator_root, "commit", "-q", "-m", "chore: second transition")
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["simulator_overlay"]["status"] == "verified-fail"


# ---------------------------------------------------------------------------
# fix round 1 / F3 mutation tests
# ---------------------------------------------------------------------------
def test_lock_commit_carrying_source_payload_fails(tmp_path):
    """A lock-only commit that bundles a source path fails the scope check."""
    fixture = make_source_lock_fixture(
        tmp_path, mode="clean", production_lock_extra_paths=("src/app.py",)
    )
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["status"] == "verified-fail"
    assert observed["production"]["checks"]["lock_commit_scope"] is False
    assert any(
        "non-lock paths" in reason for reason in observed["production"]["reasons"]
    )


def test_lock_commit_carrying_acceptance_doc_passes(tmp_path):
    """The exact acceptance-doc allowance (docs/acceptance.md) is permitted."""
    fixture = make_source_lock_fixture(
        tmp_path, mode="clean",
        production_lock_extra_paths=("docs/acceptance.md",),
        overlay_lock_extra_paths=("docs/acceptance.md",),
    )
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-pass"
    assert observed["production"]["checks"]["lock_commit_scope"] is True
    assert observed["simulator_overlay"]["checks"]["lock_commit_scope"] is True


def test_qualification_head_descendant_drift_fails(tmp_path):
    """qualification_tooling requires HEAD == its resolved lock commit."""
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    _write(fixture.simulator_root, "docs/after.md", "post-lock docs\n")
    _commit(fixture.simulator_root, "docs: after the qualification lock")
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["qualification_tooling"]["status"] == "verified-fail"
    assert observed["qualification_tooling"]["checks"]["qualification_head_matches_resolved"] is False
    assert observed["simulator_overlay"]["status"] == "verified-pass"


def test_missing_authorization_report_fails(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    report = fixture.production_root / ".superpowers/fixture-lock-report.md"
    report.unlink()
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["checks"]["authorization_report_present"] is False


def test_late_authorization_report_fails(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    report = fixture.production_root / ".superpowers/fixture-lock-report.md"
    future = fixture.attempt_started_at.timestamp() + 3600.0
    os.utime(report, (future, future))
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["checks"]["authorization_report_predates_attempt"] is False


def test_authorization_report_absolute_escape_fails(tmp_path):
    """F2.4.2: an absolute authorization.report_path that resolves outside the
    repository root fails closed (containment), even when the file exists."""
    outside = tmp_path / "outside-report.md"
    outside.write_text("outside authorization report\n", encoding="utf-8")
    past = datetime(2026, 7, 1, tzinfo=timezone.utc).timestamp()
    os.utime(outside, (past, past))
    fixture = make_source_lock_fixture(tmp_path, mode="clean", production_report_path=str(outside))
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["status"] == "verified-fail"
    assert observed["production"]["checks"]["authorization_report_contained"] is False
    assert any(
        "outside the repository root" in reason for reason in observed["production"]["reasons"]
    )


def test_authorization_report_symlink_escape_fails(tmp_path):
    """F2.4.2: a report_path that is a symlink inside the repo but resolves
    (canonically) outside the repository root fails containment."""
    outside = tmp_path / "outside-target.md"
    outside.write_text("outside target\n", encoding="utf-8")
    fixture = make_source_lock_fixture(
        tmp_path,
        mode="clean",
        production_report_path=".superpowers/escape-link.md",
        production_report_symlink_target=str(outside),
    )
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["status"] == "verified-fail"
    assert observed["production"]["checks"]["authorization_report_contained"] is False
    assert any(
        "outside the repository root" in reason for reason in observed["production"]["reasons"]
    )


def test_bogus_authorization_commit_fails(tmp_path):
    """A non-null authorization.commit that is not an ancestor of the resolved
    lock commit is rejected (F3.3).  The diverged side commit is created at
    fixture-build time so the policy blob at the lock commit carries the bogus
    value and working/committed bytes stay identical."""
    fixture = make_source_lock_fixture(
        tmp_path, mode="clean", production_authorization_commit="bogus"
    )
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert observed["production"]["status"] == "verified-fail"
    assert observed["production"]["checks"]["authorization_commit_ancestor_of_resolved"] is False
    assert any(
        "not an ancestor" in reason for reason in observed["production"]["reasons"]
    )


def test_valid_authorization_commit_passes(tmp_path):
    """A non-null authorization.commit that IS an ancestor of the resolved lock
    commit is accepted (F3.3).  Here it is the production implementation head,
    so the lock commit's first parent authorizes the locked state."""
    fixture = make_source_lock_fixture(
        tmp_path, mode="clean", production_authorization_commit="impl"
    )
    policy = _read_json(fixture.production_policy_path)
    assert policy["authorization"]["commit"] is not None
    assert len(policy["authorization"]["commit"]) == 40
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-pass"
    assert observed["production"]["status"] == "verified-pass"
    assert observed["production"]["checks"]["authorization_commit_ancestor_of_resolved"] is True
    assert observed["production"]["checks"]["authorization_commit_predates_attempt"] is True


def test_unknown_policy_key_rejected(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    policy_path = fixture.simulator_policy_path
    policy = _read_json(policy_path)
    policy["mystery_key"] = True
    _write_json_canonical(policy_path, policy)
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert any("unknown top-level policy keys" in reason for reason in observed["simulator_overlay"]["reasons"])


def test_unknown_authorization_key_rejected(tmp_path):
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    policy_path = fixture.production_policy_path
    policy = _read_json(policy_path)
    policy["authorization"]["mystery"] = True
    _write_json_canonical(policy_path, policy)
    observed = _run_capture(fixture)
    assert observed["status"] == "verified-fail"
    assert any("unknown authorization keys" in reason for reason in observed["production"]["reasons"])


def test_relative_policy_path_is_canonicalized(tmp_path):
    """Absolute/relative CLI arguments produce a canonical repository-relative
    policy_path so the static source-identity comparison stays stable."""
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    # Run the observer through the library with a relative production policy
    # path (repository-relative) and confirm the recorded path is canonical.
    from source_lock_manifest import capture_manifest

    result = capture_manifest(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        simulator_policy=fixture.simulator_policy_path,
        production_policy="integration/source-locks.json",
        qualification_policy=fixture.qualification_policy_path,
        attempt_started_at=fixture.attempt_started_at,
        output=fixture.output,
    )
    assert result["status"] == "verified-pass"
    assert result["production"]["policy_path"] == "integration/source-locks.json"


def test_output_predating_attempt_is_evidence_invalid(tmp_path):
    """An attempt started in the future makes the produced output manifest
    predate it -> evidence-invalid even when all repositories pass."""
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    future_attempt = datetime(2030, 1, 1, tzinfo=timezone.utc)
    result = capture_source_lock_manifest(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        simulator_policy_path=fixture.simulator_policy_path,
        production_policy_path=fixture.production_policy_path,
        qualification_policy_path=fixture.qualification_policy_path,
        attempt_started_at=future_attempt,
        output=fixture.output,
    )
    assert result["status"] == "evidence-invalid"
    assert result["output_predates_attempt"] is True
    # All three repositories individually pass; only output freshness fails.
    for role in ROLES:
        assert result[role]["status"] == "verified-pass"
    # Independent filesystem check: the written manifest's mtime predates the
    # future attempt start.
    import os as _os
    assert _os.stat(fixture.output).st_mtime < future_attempt.timestamp()


def test_output_single_atomic_write_mtime_agrees(tmp_path):
    """F2.4.5: the manifest is written/fsynced/replaced exactly once.  The
    persisted ``output_predates_attempt`` boolean (from a single pre-write
    timestamp) agrees with the final file's real mtime relation, and no temp
    files survive."""
    fixture = make_source_lock_fixture(tmp_path, mode="clean")
    result = capture_source_lock_manifest(
        simulator_root=fixture.simulator_root,
        production_root=fixture.production_root,
        simulator_policy_path=fixture.simulator_policy_path,
        production_policy_path=fixture.production_policy_path,
        qualification_policy_path=fixture.qualification_policy_path,
        attempt_started_at=fixture.attempt_started_at,
        output=fixture.output,
    )
    assert result["status"] == "verified-pass"
    assert result["output_predates_attempt"] is False
    # Independent filesystem relation: the final artifact postdates the attempt.
    assert os.stat(fixture.output).st_mtime > fixture.attempt_started_at.timestamp()
    leftovers = [p.name for p in fixture.output.parent.iterdir() if ".tmp" in p.name or p.name.endswith("~")]
    assert leftovers == []


def test_attempt_start_requires_iso_started_at(tmp_path):
    """The attempt-start reader rejects a missing/non-ISO started_at."""
    from source_lock_manifest import _read_attempt_start_file, SourceLockError
    start = tmp_path / "attempt-start.json"
    start.write_text(json.dumps({"note": "no started_at"}), encoding="utf-8")
    with pytest.raises(SourceLockError):
        _read_attempt_start_file(start)
    start.write_text(json.dumps({"started_at": "not-a-date"}), encoding="utf-8")
    with pytest.raises(SourceLockError):
        _read_attempt_start_file(start)
