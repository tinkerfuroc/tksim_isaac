#!/usr/bin/env python3
"""Person-model importer CLI.

GPSR's rcw2026 scenario commands *"go to the kitchen table and find a
person"*, but the arena has never contained one: ``gpsr-rcw2026.json`` ships
``"actors": []``, and the only person asset in the tree,
``simulation/assets/primitives/person.usda``, is a bare capsule (r=0.25,
h=1.2). Run12 drove to the table and then scanned 3930 times for somebody who
was not there -- ``no matches for "person" via vlm_sam``, which was the
correct answer.

``models/person_standing`` is already inside the pinned
``sobits_gazebo_worlds`` checkout the arena furniture itself comes from -- a
textured MakeHuman figure by Marina Kollmitz (Uni Freiburg), same repository,
same commit, same BSD-3 licence -- so this importer adds no new upstream
dependency, only a second model directory out of one we already vendor.

Shape mirrors ``tools/ycb_import.py``: clone/verify the pin, convert each
allowlisted model through injectable ``ConverterHooks``, publish a
content-addressed artifact via ``tinker_sim_deploy.arena_artifact``. Two
differences, both forced by upstream:

* **No separate collision mesh.** Every ``ycb_*`` model ships
  ``nontextured.stl`` beside ``textured.dae``; ``person_standing`` collides
  with the very mesh it draws (plus a small SDF box under the feet). Handing
  the same DAE to ``convert_object_to_usd`` for both roles does *not* work:
  ``arena_convert`` reads the two inputs under different unit assumptions --
  the visual path trusts the DAE stage's ``metersPerUnit``, the collision
  path assumes an already-metre-expressed STL -- so the collider came out
  exactly 100x the figure, which ``_check_bounds_overlap`` caught live::

      collision/visual size ratio 100.0000 on axis 0 is outside [0.5, 2.0]

  This importer therefore generates a metre-scale axis-aligned box STL from
  ``collision_box_m`` in the config and passes *that* as the collision
  source. A box is a deliberate simplification of upstream's mesh collider:
  the figure is scenery to walk around, never something to grasp, and a box
  is both cheaper in PhysX and immune to the unit mismatch above. The
  configured box is validated against the real converted mesh by the same
  ``_check_bounds_overlap`` guard, so an upstream re-pin that changes the
  figure's size fails the import instead of shipping a wrong collider.
* **The visual mesh filename is read from the model SDF**, not assumed.
  ``ycb_import`` can hardcode ``meshes/textured.dae`` because every YCB model
  uses it; the person models do not share a convention (``standing.dae``,
  and ``person_sitting`` ships a ``.glb``), so the SDF is the only honest
  source for it -- and reading it also lets this module check the pose below.

**The dropped visual pose.** ``person_standing``'s SDF declares its visual
and collision at ``0 0 0.02 0.04 0 0`` -- 2 cm up, rolled 2.3 deg, a
MakeHuman export artifact rather than anything meaningful. Folding it into
the converted USD would mean new ``arena_convert`` code that no test can
reach without a GPU, for an offset far below what a person-detection fixture
can notice, so this importer drops it and stands the figure upright. That
trade is only defensible while the offset stays small, so it is measured
against ``_MAX_UNFOLDED_*`` on every import and anything larger is refused
rather than silently misplaced.

Like ``ycb_import``, every Kit/pxr call is isolated behind ``ConverterHooks``
-- this module imports only the stdlib, so ``run_import``/``main`` run under
system Python with no Isaac Sim installed.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import xml.etree.ElementTree as ElementTree
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

# sobits_gazebo_worlds carries a bare "LICENSE" at the checkout root (unlike
# tmc_wrs_gz's "LICENSE.txt" -- see ycb_import's note).
_LICENSE_PATH = "LICENSE"

# How much SDF-declared visual pose this importer is willing to drop. Sized
# to accept upstream's 2 cm / 0.04 rad and little else; see the module
# docstring.
_MAX_UNFOLDED_TRANSLATION_M = 0.05
_MAX_UNFOLDED_ROTATION_RAD = 0.10


class ConverterHooks(Protocol):
    """The one Kit operation ``run_import`` needs, injectable so this
    orchestration runs without a GPU. ``arena_convert`` satisfies it
    structurally, exactly as it does for ``ycb_import``.
    """

    def convert_object_to_usd(self, dae_path: Path, stl_path: Path, usd_path: Path) -> None: ...


@dataclass(frozen=True)
class _PersonConversion:
    person_id: str
    payload: dict[str, bytes]
    source_records: tuple[dict[str, object], ...]


# --------------------------------------------------------------------------- #
# model SDF
# --------------------------------------------------------------------------- #
def _model_sdf_path(checkout: Path, models_root: str, person_id: str) -> Path:
    return checkout / models_root / person_id / "model.sdf"


def _parse_visual(sdf_bytes: bytes, person_id: str) -> tuple[str, tuple[float, ...]]:
    """Return the visual's mesh filename and its declared 6-DoF pose."""
    try:
        root = ElementTree.fromstring(sdf_bytes.decode("utf-8", errors="replace"))
    except ElementTree.ParseError as error:
        raise AssetArtifactError(f"{person_id}: model.sdf is not parseable: {error}") from error

    visual = root.find(".//link/visual")
    if visual is None:
        raise AssetArtifactError(f"{person_id}: model.sdf declares no <visual>")

    uri = visual.find("./geometry/mesh/uri")
    if uri is None or not (uri.text or "").strip():
        raise AssetArtifactError(f"{person_id}: model.sdf declares no <visual> mesh <uri>")
    # "model://person_standing/meshes/standing.dae" -> "meshes/standing.dae"
    relative = (uri.text or "").strip()
    prefix = f"model://{person_id}/"
    if not relative.startswith(prefix):
        raise AssetArtifactError(
            f"{person_id}: visual mesh uri is not a model:// reference to itself: {relative!r}"
        )
    mesh_relative = relative[len(prefix):]

    pose_element = visual.find("./pose")
    pose_text = (pose_element.text or "") if pose_element is not None else ""
    values = tuple(float(item) for item in pose_text.split()) if pose_text.strip() else (0.0,) * 6
    if len(values) != 6:
        raise AssetArtifactError(f"{person_id}: <visual><pose> is not 6 numbers: {pose_text!r}")
    return mesh_relative, values


def _check_unfolded_pose(person_id: str, pose: Sequence[float]) -> None:
    translation = math.dist((0.0, 0.0, 0.0), tuple(pose[:3]))
    rotation = max(abs(angle) for angle in pose[3:])
    if translation > _MAX_UNFOLDED_TRANSLATION_M or rotation > _MAX_UNFOLDED_ROTATION_RAD:
        raise AssetArtifactError(
            f"{person_id}: upstream declares a visual pose this importer does not fold in "
            f"({translation:.3f} m, {rotation:.3f} rad), beyond the "
            f"{_MAX_UNFOLDED_TRANSLATION_M} m / {_MAX_UNFOLDED_ROTATION_RAD} rad it may drop. "
            "Fold the pose into the conversion instead of shipping a misplaced person."
        )


# --------------------------------------------------------------------------- #
# collision proxy
# --------------------------------------------------------------------------- #
# The 12 triangles of an axis-aligned box, as vertex-index triples into the
# 8 corners enumerated by (x, y, z) bit order below.
_BOX_FACES = (
    (0, 2, 3), (0, 3, 1),   # -x
    (4, 5, 7), (4, 7, 6),   # +x
    (0, 1, 5), (0, 5, 4),   # -y
    (2, 6, 7), (2, 7, 3),   # +y
    (0, 4, 6), (0, 6, 2),   # -z
    (1, 3, 7), (1, 7, 5),   # +z
)


def _collision_box(config: Mapping[str, object], person_id: str) -> tuple[tuple[float, ...], tuple[float, ...]]:
    boxes = config.get("collision_box_m")
    entry = boxes.get(person_id) if isinstance(boxes, Mapping) else None
    if entry is None:
        raise AssetArtifactError(
            f"{person_id}: no collision_box_m entry in the importer config. Upstream "
            "ships no collision mesh, so the box proxy has to be declared."
        )
    minimum = tuple(float(value) for value in entry["min"])
    maximum = tuple(float(value) for value in entry["max"])
    if len(minimum) != 3 or len(maximum) != 3:
        raise AssetArtifactError(f"{person_id}: collision box needs 3-vector min and max")
    if any(hi <= lo for lo, hi in zip(minimum, maximum)):
        raise AssetArtifactError(
            f"{person_id}: collision box is degenerate or inverted: min={minimum} max={maximum}"
        )
    return minimum, maximum


def _collision_box_stl(minimum: Sequence[float], maximum: Sequence[float]) -> str:
    """An ASCII STL of the axis-aligned box, expressed in metres.

    Written by hand rather than pulled from a mesh library: the shape is
    twelve triangles, and the importer's whole point is to stay stdlib-only
    so it runs under system Python without Isaac Sim.
    """
    corners = [
        (
            maximum[0] if index & 4 else minimum[0],
            maximum[1] if index & 2 else minimum[1],
            maximum[2] if index & 1 else minimum[2],
        )
        for index in range(8)
    ]
    lines = ["solid person_collision"]
    for a, b, c in _BOX_FACES:
        triangle = (corners[a], corners[b], corners[c])
        lines.append(f"  facet normal {_face_normal(triangle)}")
        lines.append("    outer loop")
        for vertex in triangle:
            lines.append("      vertex {:.6f} {:.6f} {:.6f}".format(*vertex))
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid person_collision")
    return "\n".join(lines) + "\n"


def _face_normal(triangle: Sequence[Sequence[float]]) -> str:
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = triangle
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return f"{nx / length:.6f} {ny / length:.6f} {nz / length:.6f}"


# --------------------------------------------------------------------------- #
# per-model conversion
# --------------------------------------------------------------------------- #
def _relative_record_path(checkout: Path, path: Path) -> str:
    """Derived from the file actually resolved and read, never fabricated --
    same rationale as ycb_import's ``_relative_mesh_record_path``.
    """
    return path.relative_to(checkout).as_posix()


def _texture_records(checkout: Path, model_dir: Path) -> list[dict[str, object]]:
    """Every file under the model's ``materials/`` tree.

    The DAE references these by relative path and Kit resolves them at
    conversion time, so they are genuinely upstream inputs to the published
    USD even though nothing here opens them; leaving them out of the
    source-lock would let a texture change upstream go unrecorded.
    """
    materials = model_dir / "materials"
    if not materials.is_dir():
        return []
    return [
        _source_record(_relative_record_path(checkout, path), path.read_bytes())
        for path in sorted(materials.rglob("*"))
        if path.is_file()
    ]


def _person_payload(usd_path: Path, person_id: str) -> dict[str, bytes]:
    """The published entries for one model: ``person.usd`` plus whatever
    textures the conversion relocated beside it (see
    ``arena_convert._relocate_local_assets``; without republishing those the
    USD's relative material references resolve to nothing).
    """
    payload = {f"{person_id}/person.usd": usd_path.read_bytes()}
    textures_dir = usd_path.parent / "textures" / person_id
    if textures_dir.is_dir():
        for texture_file in sorted(textures_dir.iterdir()):
            if texture_file.is_file():
                payload[f"{person_id}/textures/{person_id}/{texture_file.name}"] = texture_file.read_bytes()
    return payload


def convert_person(
    checkout: Path,
    scratch: Path,
    config: Mapping[str, object],
    person_id: str,
    converter: ConverterHooks,
) -> _PersonConversion:
    models_root = str(config["models_root"])
    sdf_path = _model_sdf_path(checkout, models_root, person_id)
    if not sdf_path.is_file():
        raise AssetArtifactError(f"{person_id}: missing model SDF {sdf_path}")
    sdf_bytes = sdf_path.read_bytes()

    mesh_relative, visual_pose = _parse_visual(sdf_bytes, person_id)
    _check_unfolded_pose(person_id, visual_pose)

    model_dir = checkout / models_root / person_id
    mesh_path = model_dir / mesh_relative
    if not mesh_path.is_file():
        raise AssetArtifactError(f"{person_id}: missing visual mesh {mesh_path}")

    usd_path = scratch / person_id / "person.usd"
    minimum, maximum = _collision_box(config, person_id)
    collision_path = scratch / person_id / "collision.stl"
    collision_path.parent.mkdir(parents=True, exist_ok=True)
    collision_path.write_text(_collision_box_stl(minimum, maximum), encoding="utf-8")
    converter.convert_object_to_usd(mesh_path, collision_path, usd_path)

    records = [
        _source_record(_relative_record_path(checkout, sdf_path), sdf_bytes),
        _source_record(_relative_record_path(checkout, mesh_path), mesh_path.read_bytes()),
        *_texture_records(checkout, model_dir),
    ]
    return _PersonConversion(
        person_id=person_id,
        payload=_person_payload(usd_path, person_id),
        source_records=tuple(records),
    )


# --------------------------------------------------------------------------- #
# attribution
# --------------------------------------------------------------------------- #
def _attribution(
    config: Mapping[str, object], license_bytes: bytes, records: Sequence[Mapping[str, object]]
) -> bytes:
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
    body = (
        header
        + "The `person_standing` figure was created with MakeHuman "
        "(http://www.makehuman.org/) by Marina Kollmitz "
        "<kollmitz@cs.uni-freiburg.de>, University of Freiburg, and is "
        "redistributed here under the upstream repository's BSD 3-Clause "
        "licence reproduced below -- the same repository and pinned commit "
        "the rcw2026 arena furniture comes from.\n\n"
        "Upstream source files:\n\n"
        + per_file
    )
    return attribution_markdown(
        [
            ("person_standing — MakeHuman figure by Marina Kollmitz", body),
            ("sobits_gazebo_worlds — BSD 3-Clause", license_text),
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

    allowlist = sorted({str(item) for item in config["person_allowlist"]})
    if not allowlist:
        raise AssetArtifactError("person_allowlist must not be empty")

    with tempfile.TemporaryDirectory(prefix="person-import-scratch-") as scratch_dir:
        scratch = Path(scratch_dir)
        payload: dict[str, bytes] = {}
        records: list[dict[str, object]] = []

        for person_id in allowlist:
            conversion = convert_person(checkout, scratch, config, person_id, converter)
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
            kind="people",
            asset_id="sobits",
            payload=payload,
            source_lock=source_lock,
        )

    print(f"published people artifact: identity={publication.identity} dir={publication.artifact_dir}")
    print(
        "operator reminder: register each <person_id>/person.usd path + sha256 under "
        "generated_object_usds in artifacts/asset-manifest.json for offline bundling"
    )
    return publication


def _build_real_converter():
    from tinker_sim_deploy import arena_convert

    return arena_convert


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument(
        "--checkout",
        type=Path,
        default=None,
        help="reuse an existing pinned checkout instead of cloning fresh into the scratchpad",
    )
    args = parser.parse_args(argv)

    config = json.loads(args.config.read_text(encoding="utf-8"))
    repo_root = ROOT

    checkout = args.checkout
    if checkout is None:
        checkout = Path(tempfile.mkdtemp(prefix="person-import-checkout-"))
    if (checkout / ".git").is_dir():
        _verify_pin(checkout, str(config["commit"]))
    else:
        clone_pin(str(config["repository"]), str(config["commit"]), checkout)

    # SimulationApp inspects sys.argv independently of argparse and relaunches
    # Kit with anything it does not recognise, so --config/--checkout must be
    # hidden from it (see ycb_import's note for how that was diagnosed).
    from isaacsim import SimulationApp

    real_argv, sys.argv = sys.argv, sys.argv[:1]
    try:
        app = SimulationApp({"headless": True})
    finally:
        sys.argv = real_argv
    try:
        from isaacsim.core.utils.extensions import enable_extension

        print("person_import: enabling omni.kit.asset_converter", flush=True)
        enable_extension("omni.kit.asset_converter")

        converter = _build_real_converter()

        print("person_import: running import pipeline", flush=True)
        run_import(config, repo_root, checkout, converter)
        print("person_import: pipeline finished", flush=True)
    except BaseException:
        import traceback

        traceback.print_exc()
        app.close()
        return 1
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
