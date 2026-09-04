#!/usr/bin/env python3
"""RoboCup 2026 YCB object importer CLI.

Pulls a pinned allowlist of ``ycb_*`` object models from the SOBITS
``tmc_wrs_gz`` checkout (``tmc_wrs_gz_worlds/models/<object_id>/meshes/{
textured.dae, nontextured.stl}``), converts each object's textured DAE
(visual) and non-textured STL (collision) into one composed ``object.usd``
via a running Kit (``ConverterHooks``, kept injectable so this whole
pipeline is unit-testable without a GPU -- mirrors ``tools/arena_import.py``
from Task 9), and publishes the result as a content-addressed asset artifact
(see ``tinker_sim_deploy.arena_artifact``).

Unlike the arena importer, this module does no world parsing, no navigation
map, and no placement surfaces: every ``ycb_*`` model's visual/collision
meshes are declared at an identity pose relative to their link in the
upstream model SDF (confirmed against the real pinned checkout -- every
allowlisted object's ``<visual><pose>``/``<collision><pose>`` is
``0 0 0 0 0 0`` and no ``<mesh><scale>`` is ever declared), so
``convert_object_to_usd`` needs no per-model scale/pose correction and this
orchestration never has to read the model SDF at all -- only the DAE, the
STL, and the shared repo-root ``LICENSE.txt``.

Every Kit/pxr call is isolated behind ``ConverterHooks`` -- this module
itself only imports the stdlib, so ``run_import``/``main`` (with a stub
converter) can run under system Python with no Isaac Sim installed.
"""
from __future__ import annotations

import argparse
import json
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Protocol, Sequence

from tinker_sim_deploy.arena_artifact import (
    AssetArtifactError,
    AssetPublication,
    attribution_markdown,
    publish_asset_artifact,
)
from tinker_sim_deploy.import_common import (
    _read_upstream,
    _source_record,
    _verify_pin,
    clone_pin,
)

ROOT = Path(__file__).resolve().parents[1]

# The real pinned tmc_wrs_gz repo's license file is named "LICENSE.txt" at
# the checkout root (confirmed live: "The Clear BSD License", (c) 2020
# TOYOTA MOTOR CORPORATION) -- unlike Task 9's sobits_gazebo_worlds repo,
# which uses the bare "LICENSE".
_LICENSE_PATH = "LICENSE.txt"


class ConverterHooks(Protocol):
    """The one Kit/pxr operation ``run_import`` needs, injectable so the
    orchestration in this module can be exercised without a running
    SimulationApp. The real implementation lives in ``arena_convert.py``;
    ``main()`` passes that module directly (it structurally satisfies this
    protocol).
    """

    def convert_object_to_usd(self, dae_path: Path, stl_path: Path, usd_path: Path) -> None: ...


@dataclass(frozen=True)
class _ObjectConversion:
    object_id: str
    payload: dict[str, bytes]
    source_records: tuple[dict[str, object], ...]


# --------------------------------------------------------------------------- #
# git / pin helpers and source-record helpers are shared with
# arena_import.py via tinker_sim_deploy.import_common (Task 10 review fix
# round, Finding 3 -- was byte-identical duplicated boilerplate); imported
# at module top.
# --------------------------------------------------------------------------- #


# --------------------------------------------------------------------------- #
# per-object conversion
# --------------------------------------------------------------------------- #
def _object_mesh_paths(checkout: Path, models_root: str, object_id: str) -> tuple[Path, Path]:
    model_dir = checkout / models_root / object_id
    return model_dir / "meshes" / "textured.dae", model_dir / "meshes" / "nontextured.stl"


def _relative_mesh_record_path(checkout: Path, mesh_path: Path) -> str:
    """Source-lock path derived from the file actually resolved and read,
    not a fabricated string -- same rationale as ``arena_import``'s
    ``_relative_glb_record_path`` (see the Task 9 report).
    """
    return mesh_path.relative_to(checkout).as_posix()


def _object_payload(usd_path: Path, object_id: str) -> dict[str, bytes]:
    """The published payload entries for one object: its ``object.usd`` plus
    any texture/material files ``convert_object_to_usd`` relocated to
    ``usd_path.parent / "textures" / object_id`` (see
    ``arena_convert._relocate_local_assets`` -- reused unmodified; without
    republishing those files alongside the USD that now references them by
    a relative path, every texture would resolve to nothing in the
    published artifact).

    Only that ``object_id``-namespaced subfolder is published, deliberately
    -- not a broader walk of everything under ``textures/``. Confirmed live
    (see the Task 10 report): Kit's DAE importer independently drops its
    own flat, unreferenced copy of the same file directly under
    ``textures/`` (a side effect of extracting the COLLADA ``<init_from>``
    image during conversion, before ``_relocate_local_assets`` ever runs);
    after ``_compose_object``'s ``Stage.Flatten()`` absolutizes the
    material's asset-path attribute and ``_relocate_local_assets`` copies
    that absolute source into the namespaced destination and rewrites the
    reference to point there, the original flat copy is simply orphaned
    scratch, never read by anything -- a first version of this function
    walked ``textures/`` recursively and accidentally published that
    orphan as dead weight alongside the real, referenced copy.
    """
    payload = {f"{object_id}/object.usd": usd_path.read_bytes()}
    textures_dir = usd_path.parent / "textures" / object_id
    if textures_dir.is_dir():
        for texture_file in sorted(textures_dir.iterdir()):
            if texture_file.is_file():
                payload[f"{object_id}/textures/{object_id}/{texture_file.name}"] = texture_file.read_bytes()
    return payload


def convert_ycb_object(
    checkout: Path,
    scratch: Path,
    models_root: str,
    object_id: str,
    converter: ConverterHooks,
) -> _ObjectConversion:
    dae_path, stl_path = _object_mesh_paths(checkout, models_root, object_id)
    if not dae_path.is_file():
        raise AssetArtifactError(f"{object_id}: missing visual mesh {dae_path}")
    if not stl_path.is_file():
        raise AssetArtifactError(f"{object_id}: missing collision mesh {stl_path}")

    usd_path = scratch / object_id / "object.usd"
    converter.convert_object_to_usd(dae_path, stl_path, usd_path)

    dae_bytes = dae_path.read_bytes()
    stl_bytes = stl_path.read_bytes()
    records = (
        _source_record(_relative_mesh_record_path(checkout, dae_path), dae_bytes),
        _source_record(_relative_mesh_record_path(checkout, stl_path), stl_bytes),
    )
    return _ObjectConversion(
        object_id=object_id,
        payload=_object_payload(usd_path, object_id),
        source_records=records,
    )


# --------------------------------------------------------------------------- #
# attribution
# --------------------------------------------------------------------------- #
def _attribution(config: Mapping[str, object], license_bytes: bytes, records: Sequence[Mapping[str, object]]) -> bytes:
    license_text = license_bytes.decode("utf-8", errors="replace")
    header = (
        f"Repository: {config['repository']}\n"
        f"Branch: {config['branch']}\n"
        f"Commit: {config['commit']}\n\n"
    )
    per_file = "\n".join(
        f"- `{record['path']}` (sha256 `{record['sha256']}`, {record['size']} bytes)"
        for record in sorted(records, key=lambda item: str(item["path"]))
        if str(record["path"]) != _LICENSE_PATH
    )
    ycb_body = (
        header
        + "These objects were converted from the Yale-CMU-Berkeley (YCB) Object "
        "and Model Set, distributed under the Creative Commons Attribution 4.0 "
        "International (CC BY 4.0) license.\n\n"
        "Per-object upstream source files:\n\n"
        + per_file
    )
    return attribution_markdown(
        [
            ("YCB Object and Model Set — CC BY 4.0", ycb_body),
            ("tmc_wrs_gz — Clear BSD", license_text),
        ]
    )


# --------------------------------------------------------------------------- #
# orchestration
# --------------------------------------------------------------------------- #
def run_import(
    config: Mapping[str, object], repo_root: Path, checkout: Path, converter: ConverterHooks
) -> AssetPublication:
    repo_root = Path(repo_root)
    checkout = Path(checkout)
    commit = str(config["commit"])
    _verify_pin(checkout, commit)

    models_root = str(config["models_root"])
    allowlist = sorted({str(item) for item in config["object_allowlist"]})
    if not allowlist:
        raise AssetArtifactError("object_allowlist must not be empty")

    with tempfile.TemporaryDirectory(prefix="ycb-import-scratch-") as scratch_dir:
        scratch = Path(scratch_dir)
        payload: dict[str, bytes] = {}
        records: list[dict[str, object]] = []

        for object_id in allowlist:
            conversion = convert_ycb_object(checkout, scratch, models_root, object_id, converter)
            payload.update(conversion.payload)
            records.extend(conversion.source_records)

        license_bytes = _read_upstream(checkout, _LICENSE_PATH)
        records.append(_source_record(_LICENSE_PATH, license_bytes))
        payload["ATTRIBUTION.md"] = _attribution(config, license_bytes, records)

        records.sort(key=lambda item: str(item["path"]))
        source_lock = {
            "repository": str(config["repository"]),
            "branch": str(config["branch"]),
            "commit": commit,
            "records": records,
        }
        publication = publish_asset_artifact(
            repo_root,
            kind="objects",
            asset_id="ycb",
            payload=payload,
            source_lock=source_lock,
        )

    print(f"published ycb objects artifact: identity={publication.identity} dir={publication.artifact_dir}")
    print(
        "operator reminder: register each <object_id>/object.usd path + sha256 under "
        "generated_object_usds in artifacts/asset-manifest.json for offline bundling"
    )
    return publication


# Published YCB catalog masses (kg), keyed by the artifact object-directory
# name. Authored explicitly so a grasped object loads the grip at its real mass
# instead of PhysX's density-derived estimate (~2x off). Overridable; the four
# grasp-bench objects (mustard/soup/sugar/bleach) were cross-checked against the
# catalog.
YCB_OBJECT_MASSES_KG: dict[str, float] = {
    "ycb_001_cheez-it": 0.411,          # cracker box
    "ycb_002_sugar_box": 0.514,
    "ycb_005_spam": 0.370,              # potted meat can
    "ycb_006_mustard_bottle": 0.603,
    "ycb_008_pudding_box": 0.187,
    "ycb_010_tomato_soup_can": 0.349,
    "ycb_011_banana": 0.066,
    "ycb_021_bleach_cleanser": 1.131,
    "ycb_024_bowl": 0.147,
    "ycb_025_mug": 0.118,
}

# Realistic plastic/metal default (grasp-bench-specified), bound onto every
# object collider so a closed grip can hold it under fabric-off.
YCB_STATIC_FRICTION = 0.8
YCB_DYNAMIC_FRICTION = 0.7


def repair_physics(repo_root: Path) -> AssetPublication:
    """Republish the current YCB artifact with rigid-body physics authored.

    The original import published each ``object.usd`` with colliders but no
    ``RigidBodyAPI`` on the default prim, so every spawned YCB object
    composed in as static scenery: the simulation's rigid-body view never
    resolved, ground-truth poses were silently absent, and grasps could not
    move anything (observed live 2026-08-31 as the ``/World/Scenario/soup``
    physx-tensors pattern-miss loop, ~17k error lines per session).

    This is a pure pxr edit -- ``author_object_rigid_body`` applied to each
    existing ``object.usd``, every other payload byte carried over verbatim,
    the source lock reused unchanged (upstream sources are identical) -- so
    it needs no Kit, no GPU, and no upstream checkout.  Publishing yields a
    new content-addressed identity; the old artifact directory remains for
    provenance.
    """
    from pxr import Usd

    from tinker_sim_deploy.arena_convert import (
        author_object_friction_material,
        author_object_mass,
        author_object_rigid_body,
        author_preview_surface_material,
    )

    current = json.loads(
        (repo_root / "artifacts/objects/ycb/current.json").read_text(encoding="utf-8")
    )
    manifest_path = repo_root / current["manifest"]
    artifact_dir = manifest_path.parent
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    source_lock = json.loads((artifact_dir / "source-lock.json").read_text(encoding="utf-8"))

    payload: dict[str, bytes] = {}
    repaired = []
    with tempfile.TemporaryDirectory(prefix="ycb-physics-repair-") as scratch_dir:
        scratch = Path(scratch_dir)
        for name in manifest["payload"]:
            data = (artifact_dir / name).read_bytes()
            if name.endswith("/object.usd"):
                object_id = name.split("/", 1)[0]
                work = scratch / object_id
                work.mkdir(parents=True, exist_ok=True)
                source = work / "object.usd"
                source.write_bytes(data)
                mass_kg = YCB_OBJECT_MASSES_KG.get(object_id)
                if mass_kg is None:
                    raise AssetArtifactError(
                        f"no catalog mass for {object_id!r}; add it to "
                        "YCB_OBJECT_MASSES_KG before repairing"
                    )
                stage = Usd.Stage.Open(str(source))
                author_object_rigid_body(stage)
                author_object_mass(stage, mass_kg)
                author_object_friction_material(
                    stage, YCB_STATIC_FRICTION, YCB_DYNAMIC_FRICTION
                )
                author_preview_surface_material(stage)
                repaired_path = work / "object.repaired.usd"
                if not stage.GetRootLayer().Export(str(repaired_path)):
                    raise AssetArtifactError(f"failed to export repaired USD for {name}")
                data = repaired_path.read_bytes()
                repaired.append(object_id)
            payload[name] = data
    if not repaired:
        raise AssetArtifactError("current YCB artifact contains no object.usd payloads")
    publication = publish_asset_artifact(
        repo_root,
        kind="objects",
        asset_id="ycb",
        payload=payload,
        source_lock=source_lock,
    )
    _repoint_asset_manifest(repo_root, artifact_dir.name, publication)
    return publication


def _repoint_asset_manifest(
    repo_root: Path, old_identity: str, publication: AssetPublication
) -> None:
    """Repoint ``generated_object_usds`` entries at the repaired artifact.

    ``tools/gpsr_scene.py`` resolves command-spawned object assets through
    ``artifacts/asset-manifest.json``, so a physics repair is incomplete for
    the bench until these entries follow the new identity.  Entries for
    other artifacts are left untouched; a missing manifest is left missing
    (offline bundles may not carry one).
    """
    manifest_path = repo_root / "artifacts" / "asset-manifest.json"
    if not manifest_path.is_file():
        return
    new_manifest = json.loads(
        (publication.artifact_dir / "manifest.json").read_text(encoding="utf-8")
    )
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    updated = 0
    for entry in data.get("generated_object_usds", []):
        path = str(entry.get("path", ""))
        if old_identity not in path:
            continue
        new_path = path.replace(old_identity, publication.identity)
        relative = new_path.split(f"{publication.identity}/", 1)[1]
        digest = new_manifest["payload"].get(relative)
        if digest is None:
            raise AssetArtifactError(
                f"asset-manifest entry {path!r} has no counterpart in the repaired artifact"
            )
        entry["path"] = new_path
        entry["sha256"] = digest
        updated += 1
    if updated:
        manifest_path.write_text(
            json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(f"asset-manifest.json: repointed {updated} generated_object_usds entries")


def _build_real_converter():
    from tinker_sim_deploy import arena_convert

    return arena_convert


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument(
        "--checkout",
        type=Path,
        default=None,
        help="reuse an existing pinned checkout instead of cloning fresh into the scratchpad",
    )
    parser.add_argument(
        "--repair-physics",
        action="store_true",
        help=(
            "republish the current artifact with rigid-body physics authored "
            "on each object.usd (pure pxr, no Kit, no checkout)"
        ),
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=None,
        help=(
            "repository root holding the real artifacts/ store (defaults to "
            "this checkout; pass the primary checkout from a worktree whose "
            "artifacts/ is a symlink -- the atomic publisher refuses symlink "
            "components)"
        ),
    )
    args = parser.parse_args(argv)

    if args.repair_physics:
        publication = repair_physics((args.root or ROOT).resolve())
        print(
            f"published physics-repaired ycb objects artifact: "
            f"identity={publication.identity} dir={publication.artifact_dir}"
        )
        print(
            "operator reminder: update scenario asset_uris and the "
            "generated_object_usds entries in artifacts/asset-manifest.json "
            "to the new identity"
        )
        return 0
    if args.config is None:
        parser.error("--config is required unless --repair-physics is given")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    repo_root = ROOT

    checkout = args.checkout
    if checkout is None:
        checkout = Path(tempfile.mkdtemp(prefix="ycb-import-checkout-"))
    if (checkout / ".git").is_dir():
        _verify_pin(checkout, str(config["commit"]))
    else:
        clone_pin(str(config["repository"]), str(config["commit"]), checkout)

    # Kit conversion (arena_convert.convert_object_to_usd) needs a running
    # SimulationApp with omni.kit.asset_converter enabled; the pure-
    # orchestration path exercised by the unit tests never reaches this
    # (StubHooks never imports Kit/pxr). SimulationApp also inspects
    # sys.argv (independent of the argparse call above) and re-launches the
    # underlying Kit process, passing through any arg it does not recognize
    # -- our own --config/--checkout confuse that re-launch and the app
    # exits immediately with no traceback, so sys.argv is hidden from it
    # here and restored after (see the Task 9 report for how this was
    # diagnosed).
    from isaacsim import SimulationApp

    real_argv, sys.argv = sys.argv, sys.argv[:1]
    try:
        app = SimulationApp({"headless": True})
    finally:
        sys.argv = real_argv
    try:
        from isaacsim.core.utils.extensions import enable_extension

        print("ycb_import: enabling omni.kit.asset_converter", flush=True)
        enable_extension("omni.kit.asset_converter")

        converter = _build_real_converter()

        print("ycb_import: running import pipeline", flush=True)
        run_import(config, repo_root, checkout, converter)
        print("ycb_import: pipeline finished", flush=True)
    except BaseException:
        import traceback

        traceback.print_exc()
        app.close()
        return 1
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
