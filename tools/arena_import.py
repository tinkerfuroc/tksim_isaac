#!/usr/bin/env python3
"""RoboCup 2026 arena importer CLI.

Pulls the pinned SOBITS ``sobits_gazebo_worlds`` world/model definitions,
converts each allowlisted furniture GLB to USD via a running Kit
(``ConverterHooks``, kept injectable so this whole pipeline is unit-testable
without a GPU), composes the arena stage, derives the navigation map and
placement surfaces, and publishes the result as a content-addressed asset
artifact (see ``tinker_sim_deploy.arena_artifact``).

Every Kit/pxr call is isolated behind ``ConverterHooks`` -- this module
itself only imports the stdlib and the plain-Python Task 1-5 modules, so
``run_import``/``main`` (with a stub converter) can run under system Python
with no Isaac Sim installed.
"""
from __future__ import annotations

import argparse
import json
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Protocol, Sequence

from tinker_sim_deploy.arena_artifact import (
    AssetArtifactError,
    AssetPublication,
    attribution_markdown,
    publish_asset_artifact,
)
from tinker_sim_deploy.arena_map import livox_scan_height, rasterize
from tinker_sim_deploy.arena_surfaces import PlacementSurface, placement_json, world_surface
from tinker_sim_deploy.arena_world import (
    ArenaLayout,
    BoxCollider,
    MeshCollider,
    parse_model_colliders,
    parse_world,
)
from tinker_sim_deploy.import_common import (
    _read_upstream,
    _source_record,
    _verify_pin,
    clone_pin,
)

ROOT = Path(__file__).resolve().parents[1]

_IDENTITY_POSE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
_IDENTITY_SCALE = (1.0, 1.0, 1.0)


class ConverterHooks(Protocol):
    """The four Kit/pxr operations ``run_import`` needs, injectable so the
    orchestration in this module can be exercised without a running
    SimulationApp. The real implementation lives in ``arena_convert.py``;
    ``main()`` passes that module directly (its functions structurally
    satisfy this protocol).
    """

    def convert_glb_to_usd(
        self,
        glb_path: Path,
        usd_path: Path,
        *,
        mesh_scale: tuple[float, float, float] = _IDENTITY_SCALE,
        mesh_pose: tuple[float, float, float, float, float, float] = _IDENTITY_POSE,
    ) -> None: ...

    def author_model_colliders(
        self, usd_path: Path, colliders: tuple[BoxCollider | MeshCollider, ...]
    ) -> None: ...

    def compose_arena(
        self, arena_usd: Path, layout: ArenaLayout, furniture_dir_name: str = "furniture"
    ) -> None: ...

    def measure_bounds(
        self, usd_path: Path
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]: ...


@dataclass(frozen=True)
class _FurnitureConversion:
    model_id: str
    payload: dict[str, bytes]
    map_colliders: tuple[BoxCollider, ...]
    source_records: tuple[dict[str, object], ...]


# --------------------------------------------------------------------------- #
# git / pin helpers and source-record helpers are shared with ycb_import.py
# via tinker_sim_deploy.import_common (Task 10 review fix round, Finding 3
# -- was byte-identical duplicated boilerplate); imported at module top.
# --------------------------------------------------------------------------- #


# --------------------------------------------------------------------------- #
# SDF reading helpers (visual-mesh info is not part of Task 1's
# parse_model_colliders contract -- it is collision-only)
# --------------------------------------------------------------------------- #
def _visual_mesh_info(sdf_bytes: bytes) -> tuple[str, tuple[float, float, float], tuple[float, ...]]:
    """Return (mesh uri, mesh scale, visual pose) from the model SDF's one
    visual mesh. The upstream RCW26 GLBs are unit-normalized and Y-up; the
    SDF's ``<mesh><scale>`` and ``<visual><pose>`` (with a 90-degree roll)
    are what bring them to real-world size/orientation in Gazebo, and the
    importer must reproduce that when converting to USD (see the Task 9
    report for why ``convert_glb_to_usd`` grew ``mesh_scale``/``mesh_pose``).

    Fails closed if the model SDF declares more than one visual mesh: the
    importer only converts a single GLB per model (``convert_furniture_model``
    produces exactly one ``furniture/<id>.usd``), so silently picking the
    first match would drop the second mesh's geometry and its GLB would
    never enter the source lock.
    """
    root = ET.fromstring(sdf_bytes)
    found: tuple[str, tuple[float, float, float], tuple[float, ...]] | None = None
    for visual in root.iter("visual"):
        uri = visual.findtext("geometry/mesh/uri")
        if not uri:
            continue
        if found is not None:
            raise AssetArtifactError(
                "model SDF declares more than one visual mesh; the importer "
                "only supports a single visual mesh per model"
            )
        scale_text = visual.findtext("geometry/mesh/scale")
        scale = tuple(float(part) for part in scale_text.split()) if scale_text else _IDENTITY_SCALE
        pose_text = visual.findtext("pose")
        pose = tuple(float(part) for part in pose_text.split()) if pose_text else _IDENTITY_POSE
        if len(pose) != 6:
            raise AssetArtifactError(f"visual pose must have 6 values, got {pose_text!r}")
        if len(scale) != 3:
            raise AssetArtifactError(f"visual mesh scale must have 3 values, got {scale_text!r}")
        found = (uri.strip(), scale, pose)
    if found is None:
        raise AssetArtifactError("model SDF has no visual mesh")
    return found


def _resolve_glb_path(checkout: Path, model_id: str, uri: str) -> Path:
    """Resolve a model SDF's ``<mesh><uri>`` to a real file under ``checkout``,
    failing closed (before any read) on either escape this join is otherwise
    exposed to: an absolute URI remainder collapses ``checkout / "models" /
    model_id / "/abs"`` to plain ``/abs`` (``Path.__truediv__`` discards
    everything to the left of an absolute right operand), and a relative
    remainder containing ``..`` segments (e.g. ``model://<id>/../../..``)
    can walk lexically outside ``checkout`` while still passing a lexical
    ``relative_to`` check performed before resolution. Both classes are
    caught the same way here: resolve the joined path (collapses ``..`` and
    symlinks) and require it be contained within the resolved checkout root.
    """
    prefix = f"model://{model_id}/"
    if uri.startswith(prefix):
        relative = uri[len(prefix):]
    elif uri.startswith("model://"):
        raise AssetArtifactError(f"{model_id}: visual mesh references a different model: {uri!r}")
    else:
        relative = uri
    checkout_root = checkout.resolve()
    resolved = (checkout / "models" / model_id / relative).resolve()
    if not resolved.is_relative_to(checkout_root):
        raise AssetArtifactError(
            f"{model_id}: visual mesh URI resolves outside the checkout: {uri!r}"
        )
    return resolved


def _has_non_box_collision(sdf_bytes: bytes) -> bool:
    """True if any collision in this model is not a literal ``<box>`` (mesh
    or cylinder-derived). Those collision shapes are known upstream
    approximations of the visual mesh (not exact matches -- see the Task 9
    report), so they are exempt from the strict box-bounds check and instead
    contribute the measured USD AABB to the map slice.
    """
    root = ET.fromstring(sdf_bytes)
    for collision in root.iter("collision"):
        geometry = collision.find("geometry")
        if geometry is None or geometry.find("box") is None:
            return True
    return False


# --------------------------------------------------------------------------- #
# bounds checking
# --------------------------------------------------------------------------- #
def _box_world_extent(box: BoxCollider) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    import math

    cx, cy, cz = box.center
    sx, sy, sz = box.size
    cos_y, sin_y = math.cos(box.yaw), math.sin(box.yaw)
    hx = abs(cos_y) * sx / 2.0 + abs(sin_y) * sy / 2.0
    hy = abs(sin_y) * sx / 2.0 + abs(cos_y) * sy / 2.0
    hz = sz / 2.0
    return (cx - hx, cy - hy, cz - hz), (cx + hx, cy + hy, cz + hz)


def _union_extent(
    extents: Sequence[tuple[tuple[float, float, float], tuple[float, float, float]]]
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    mins = list(extents[0][0])
    maxs = list(extents[0][1])
    for lo, hi in extents[1:]:
        mins = [min(a, b) for a, b in zip(mins, lo)]
        maxs = [max(a, b) for a, b in zip(maxs, hi)]
    return tuple(mins), tuple(maxs)


def _check_bounds(
    model_id: str,
    measured: tuple[tuple[float, float, float], tuple[float, float, float]],
    declared: tuple[tuple[float, float, float], tuple[float, float, float]],
    tolerance: float,
) -> None:
    (mmin, mmax), (dmin, dmax) = measured, declared
    for axis in range(3):
        if abs(mmin[axis] - dmin[axis]) > tolerance or abs(mmax[axis] - dmax[axis]) > tolerance:
            raise AssetArtifactError(
                f"{model_id}: measured USD bounds {measured} exceed bounds_tolerance_m="
                f"{tolerance} vs SDF box-collider extents {declared}"
            )


# --------------------------------------------------------------------------- #
# per-model conversion
# --------------------------------------------------------------------------- #
def _convert_model(
    checkout: Path, scratch: Path, model_id: str, converter: ConverterHooks
) -> tuple[Path, bytes, tuple[BoxCollider | MeshCollider, ...], tuple[float, ...], tuple[float, ...]]:
    sdf_bytes = _read_upstream(checkout, f"models/{model_id}/model.sdf")
    colliders = parse_model_colliders(sdf_bytes)
    uri, scale, pose = _visual_mesh_info(sdf_bytes)
    glb_path = _resolve_glb_path(checkout, model_id, uri)

    usd_path = scratch / "furniture" / f"{model_id}.usd"
    converter.convert_glb_to_usd(glb_path, usd_path, mesh_scale=scale, mesh_pose=pose)
    converter.author_model_colliders(usd_path, colliders)
    return usd_path, sdf_bytes, colliders, uri, glb_path


def _relative_glb_record_path(checkout: Path, glb_path: Path) -> str:
    """Source-lock path for a resolved GLB: derived from the file actually
    resolved and read, not guessed from a naming convention -- a fabricated
    ``models/<id>/meshes/<name>`` string would silently diverge from the
    real path for any model whose mesh URI nests deeper than one directory.
    """
    return glb_path.relative_to(checkout).as_posix()


def convert_furniture_model(
    checkout: Path,
    scratch: Path,
    model_id: str,
    converter: ConverterHooks,
    tolerance: float,
    bounds_check_exceptions: frozenset[str] = frozenset(),
) -> _FurnitureConversion:
    usd_path, sdf_bytes, colliders, uri, glb_path = _convert_model(checkout, scratch, model_id, converter)
    measured = converter.measure_bounds(usd_path)

    box_colliders = tuple(item for item in colliders if isinstance(item, BoxCollider))
    is_strict_box = box_colliders and not _has_non_box_collision(sdf_bytes)
    if is_strict_box and model_id not in bounds_check_exceptions:
        declared = _union_extent([_box_world_extent(box) for box in box_colliders])
        _check_bounds(model_id, measured, declared, tolerance)
        map_colliders = box_colliders
    else:
        mmin, mmax = measured
        size = tuple(hi - lo for lo, hi in zip(mmin, mmax))
        center = tuple((lo + hi) / 2.0 for lo, hi in zip(mmin, mmax))
        map_colliders = (BoxCollider(size=size, center=center, yaw=0.0),)

    glb_bytes = glb_path.read_bytes()
    records = (
        _source_record(f"models/{model_id}/model.sdf", sdf_bytes),
        _source_record(_relative_glb_record_path(checkout, glb_path), glb_bytes),
    )
    return _FurnitureConversion(
        model_id=model_id,
        payload=_furniture_payload(usd_path, model_id),
        map_colliders=map_colliders,
        source_records=records,
    )


def _furniture_payload(usd_path: Path, model_id: str) -> dict[str, bytes]:
    """The published payload entries for one furniture model: its ``.usd``
    plus any texture/material files ``convert_glb_to_usd`` relocated to
    ``usd_path.parent / "textures" / model_id`` (see
    ``arena_convert._relocate_local_assets`` -- without republishing those
    files alongside the USD that now references them by a relative path,
    every texture would resolve to nothing in the published artifact).
    """
    payload = {f"furniture/{model_id}.usd": usd_path.read_bytes()}
    textures_dir = usd_path.parent / "textures" / model_id
    if textures_dir.is_dir():
        for texture_file in sorted(textures_dir.iterdir()):
            if texture_file.is_file():
                payload[f"furniture/textures/{model_id}/{texture_file.name}"] = texture_file.read_bytes()
    return payload


# --------------------------------------------------------------------------- #
# placement surfaces / attribution
# --------------------------------------------------------------------------- #
def _furniture_instance_suffixes(furniture: Sequence[object]) -> dict[int, str]:
    """Map each placement's ``id()`` to the disambiguating suffix
    ``arena_convert.compose_arena`` would give its furniture prim: computed
    the same way (iterate placements in order, first-seen bare, later
    instances of the same model_id get an incrementing ``_NN`` suffix) so
    ``placement.json`` surface_ids always name the same instance as the
    corresponding ``arena.usd`` prim. ``id()`` is safe here because every
    placement is a distinct object built once during ``parse_world``.
    """
    counts: dict[str, int] = {}
    suffixes: dict[int, str] = {}
    for placement in furniture:
        counts[placement.model_id] = counts.get(placement.model_id, 0) + 1
        count = counts[placement.model_id]
        suffixes[id(placement)] = "" if count == 1 else f"_{count:02d}"
    return suffixes


def _build_surfaces(layout: ArenaLayout, surface_specs: Sequence[Mapping[str, object]]) -> list[PlacementSurface]:
    instance_suffixes = _furniture_instance_suffixes(layout.furniture)
    surfaces: list[PlacementSurface] = []
    for spec in surface_specs:
        model_id = str(spec["model_id"])
        matches = [item for item in layout.furniture if item.model_id == model_id]
        if not matches:
            raise AssetArtifactError(f"surfaces config references unplaced model: {model_id}")
        for placement in matches:
            surfaces.append(
                world_surface(
                    placement,
                    surface_name=str(spec["surface_name"]),
                    local_center=tuple(spec["local_center"]),
                    size_xy=tuple(spec["size_xy"]),
                    edge_margin=float(spec["edge_margin"]),
                    instance_suffix=instance_suffixes[id(placement)],
                )
            )
    return surfaces


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
    )
    return attribution_markdown(
        [
            ("Upstream Source", header + license_text),
            ("Consumed Files", per_file),
        ]
    )


def _read_robot_urdf(repo_root: Path) -> bytes:
    pointer_path = repo_root / "artifacts/robot/tinker2/current.json"
    current = json.loads(pointer_path.read_text(encoding="utf-8"))
    manifest_path = Path(str(current["manifest"]))
    if not manifest_path.is_absolute():
        manifest_path = repo_root / manifest_path
    return (manifest_path.parent / "robot.urdf").read_bytes()


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

    world_bytes = _read_upstream(checkout, str(config["world"]))
    allowlist = frozenset(str(item) for item in config["model_allowlist"])
    skiplist = frozenset(str(item) for item in config.get("model_skiplist", ()))
    layout = parse_world(world_bytes, allowlist, model_skiplist=skiplist)
    tolerance = float(config.get("bounds_tolerance_m", 0.01))
    bounds_check_exceptions = frozenset(str(item) for item in config.get("bounds_check_exceptions", ()))
    model_ids = sorted({item.model_id for item in layout.furniture})

    with tempfile.TemporaryDirectory(prefix="arena-import-scratch-") as scratch_dir:
        scratch = Path(scratch_dir)
        payload: dict[str, bytes] = {}
        map_colliders: dict[str, tuple[BoxCollider, ...]] = {}
        records: list[dict[str, object]] = [_source_record(str(config["world"]), world_bytes)]

        for model_id in model_ids:
            conversion = convert_furniture_model(
                checkout, scratch, model_id, converter, tolerance, bounds_check_exceptions
            )
            payload.update(conversion.payload)
            map_colliders[model_id] = conversion.map_colliders
            records.extend(conversion.source_records)

        arena_usd_path = scratch / "arena.usd"
        converter.compose_arena(arena_usd_path, layout, furniture_dir_name="furniture")
        payload["arena.usd"] = arena_usd_path.read_bytes()

        scan_height = livox_scan_height(_read_robot_urdf(repo_root))
        pgm_bytes, map_yaml_bytes = rasterize(layout, map_colliders, scan_height=scan_height)
        payload["map.pgm"] = pgm_bytes
        payload["map.yaml"] = map_yaml_bytes

        surfaces = _build_surfaces(layout, config.get("surfaces", ()))
        payload["placement.json"] = placement_json(str(config["arena_id"]), surfaces)

        license_bytes = _read_upstream(checkout, "LICENSE")
        records.append(_source_record("LICENSE", license_bytes))
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
            kind="arena",
            asset_id=str(config["arena_id"]),
            payload=payload,
            source_lock=source_lock,
        )

    print(f"published arena artifact rcw2026: identity={publication.identity} dir={publication.artifact_dir}")
    print(
        "operator reminder: register the arena.usd path + sha256 under "
        "generated_arena_usds in artifacts/asset-manifest.json for offline bundling"
    )
    return publication


def report_bounds(config: Mapping[str, object], checkout: Path, converter: ConverterHooks) -> None:
    """``--report-bounds``: convert + author colliders + measure every
    referenced model, print the results, and exit without publishing. Used
    to fill the config's ``surfaces`` entries from real measured geometry.
    """
    world_bytes = _read_upstream(checkout, str(config["world"]))
    allowlist = frozenset(str(item) for item in config["model_allowlist"])
    skiplist = frozenset(str(item) for item in config.get("model_skiplist", ()))
    layout = parse_world(world_bytes, allowlist, model_skiplist=skiplist)
    model_ids = sorted({item.model_id for item in layout.furniture})

    with tempfile.TemporaryDirectory(prefix="arena-import-report-") as scratch_dir:
        scratch = Path(scratch_dir)
        for model_id in model_ids:
            usd_path, sdf_bytes, _colliders, _uri, _glb_path = _convert_model(checkout, scratch, model_id, converter)
            bounds = converter.measure_bounds(usd_path)
            kind = "non-box" if _has_non_box_collision(sdf_bytes) else "box"
            print(f"{model_id}: collision={kind} bounds={bounds}")


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
    parser.add_argument(
        "--report-bounds",
        action="store_true",
        help="print per-model measured bounds and exit without publishing",
    )
    args = parser.parse_args(argv)

    config = json.loads(args.config.read_text(encoding="utf-8"))
    repo_root = ROOT

    checkout = args.checkout
    if checkout is None:
        checkout = Path(tempfile.mkdtemp(prefix="arena-import-checkout-"))
    if (checkout / ".git").is_dir():
        _verify_pin(checkout, str(config["commit"]))
    else:
        clone_pin(str(config["repository"]), str(config["commit"]), checkout)

    # Kit conversion (arena_convert.convert_glb_to_usd/author_model_colliders
    # /compose_arena/measure_bounds) needs a running SimulationApp with
    # omni.kit.asset_converter enabled; the pure-orchestration path exercised
    # by the unit tests never reaches this (StubHooks never imports Kit/pxr).
    # SimulationApp also inspects sys.argv (independent of the argparse call
    # above) and re-launches the underlying Kit process, passing through any
    # arg it does not recognize -- our own --config/--checkout/--report-bounds
    # confuse that re-launch and the app exits immediately with no
    # traceback, so sys.argv is hidden from it here and restored after.
    from isaacsim import SimulationApp

    real_argv, sys.argv = sys.argv, sys.argv[:1]
    try:
        app = SimulationApp({"headless": True})
    finally:
        sys.argv = real_argv
    try:
        from isaacsim.core.utils.extensions import enable_extension

        print("arena_import: enabling omni.kit.asset_converter", flush=True)
        enable_extension("omni.kit.asset_converter")

        converter = _build_real_converter()

        print("arena_import: running import pipeline", flush=True)
        if args.report_bounds:
            report_bounds(config, checkout, converter)
        else:
            run_import(config, repo_root, checkout, converter)
        print("arena_import: pipeline finished", flush=True)
    except BaseException:
        import traceback

        traceback.print_exc()
        app.close()
        return 1
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
