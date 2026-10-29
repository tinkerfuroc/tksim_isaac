# CoACD Furniture Hulls Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every rcw2026 furniture model collides as a CoACD convex decomposition of its visual mesh, proven to match the nav map's outline within 5 mm and to still hold placed objects, before the arena switches to it.

**Architecture:** CoACD runs offline inside `tools/arena_import.py`. Each furniture USD gets invisible `Colliders/hull_NN` convex meshes in place of its SDF boxes or whole-mesh hull; explicit `collision_overrides` (the shelf) keep their boxes. The map keeps rasterizing from the declared SDF boxes, so `map.pgm` is unchanged. A new outline module compares each instance's hulls (sliced to the scan band, projected top-down) against those boxes at 1 mm resolution. Publishing no longer moves `current.json`; selection is an explicit step after the outline check, a user checkpoint, a live placement check and an RTF report.

**Tech Stack:** Python 3.12 (`.venv`), `coacd` 1.0.7, `trimesh` 4.11, `scipy` 1.17, `numpy`, `pxr` (from Isaac Sim's venv), Pillow 12.2; ROS 2 Humble system Python for the live check.

**Spec:** `docs/superpowers/specs/2026-10-22-top-lidar-placeholder-design.md`, Part 1. This plan covers Part 1 only; Part 2 has its own plan, `docs/superpowers/plans/2026-10-22-top-lidar-placeholder.md`, and starts only after this one ships.

## Global Constraints

- Run tests with `scripts/pytest-clean <test file> -q`. It uses `.venv/bin/python`, which has `pxr`, `coacd`, `trimesh`, `scipy`. There is no `shapely`; don't add it.
- Tests that need `pxr` or `coacd` start with `pytest.importorskip("pxr")` / `pytest.importorskip("coacd")`, as `tests/test_arena_collision_overrides.py` does. Kit-only modules (`omni.*`) can't be imported in tests.
- Outline rule (user's words): hull outline "roughly the same as the map outline, no bigger and no smaller. Very small corners (<5mm) are acceptable." Tolerance is **5.0 mm**, in both directions.
- Outline band = `arena_map.livox_scan_band(<tinker2 robot.urdf>)`, about 0.27–2.27 m above the floor, the same band the map uses.
- ~~The new artifact's `map.pgm` and `map.yaml` must be byte-identical to the current rcw2026 artifact's.~~ **Amended 2026-10-29 (user decision, see spec Checks 1/1b):** furniture footprints are redrawn from the hulls; `map.yaml` stays byte-identical to the box-based map's (`artifacts/arena/rcw2026/669e13f7…/`) and only furniture cells of `map.pgm` may differ. Execution order after Task 8's checkpoint: Task 10 → Task 11 → Task 9.
- `collision_overrides` in `config/arena-import.json` keep precedence over CoACD.
- CoACD parameters and seed are pinned in `config/arena-import.json` under `"coacd"` and recorded in the artifact.
- Nothing selects the new arena (writes `artifacts/arena/rcw2026/current.json`) until Task 9's user confirmation.
- Shared working tree, other sessions commit here too: stage only your own files with `git add <paths>`, check `git diff --cached --name-only` shows only them, then `git commit`. Never `git commit -a` or `git commit -- <paths>`. If `.git/index.lock` exists, wait for it to clear.
- Live sim steps (Tasks 8, 9): the GPU is shared. Before launching, `nvidia-smi --query-compute-apps=pid --format=csv,noheader` must print nothing, or ask the user. Use a private ROS domain that no process uses (check `grep -l ROS_DOMAIN_ID=<n> /proc/*/environ`), wrap every live command in `timeout -k 10 <seconds>`, and clean up with `scripts/kill-domain-nodes <domain> --force`.

## Review Focus

- Visual meshes with quads/n-gons and several mesh prims under `/World/geom`: CoACD must get one merged, triangulated mesh in model-local metres. Pinned by `test_read_visual_mesh_triangulates_and_merges` (Task 4).
- A hull piece wholly below or above the band (a plinth, a lamp top): contributes nothing and doesn't crash. Pinned by `test_piece_outside_band_contributes_nothing` (Task 5).
- Re-running the import on unchanged inputs gives the same artifact identity (CoACD determinism). Pinned by `test_decompose_is_deterministic` (Task 3) and `test_reimport_is_idempotent_with_coacd` (Task 6).
- Two instances of one model (`rcw26_shelf`, `rcw26_shelf_02`) are checked and reported separately, named like `compose_arena`'s prims. Pinned by `test_duplicate_instances_reported_separately` (Task 5).
- Older arena artifacts without CoACD hulls still get the support-surface box override. Pinned by `test_legacy_visual_mesh_colliders_still_boxed` (Task 7).

---

### Task 1: Publishing stops moving `current.json`; selection is explicit

**Files:**
- Modify: `tools/tinker_sim_deploy/arena_artifact.py` (`publish_asset_artifact`, new `select_asset_artifact`)
- Modify: `tools/arena_import.py` (`run_import` publishes unselected; `main` gains `--select`)
- Test: `tests/test_arena_artifact.py`, `tests/test_arena_import_cli.py`

**Interfaces:**
- Produces: `publish_asset_artifact(repo_root, *, kind, asset_id, payload, source_lock, select: bool = True) -> AssetPublication`. With `select=False` the pointer is not written or touched.
- Produces: `select_asset_artifact(repo_root: Path, *, kind: str, asset_id: str, identity: str) -> Path`. Writes `artifacts/<kind>/<asset_id>/current.json` (same bytes `publish_asset_artifact` would write) under `_publication_lock`, returns the pointer path. Raises `AssetArtifactError` if `artifacts/<kind>/<asset_id>/<identity>/` is missing or `verify_asset_artifact` reports any problem.
- Produces: `python tools/arena_import.py --config config/arena-import.json --select <identity>`: selects and exits 0 without cloning or importing.

- [ ] **Step 1: Write the failing tests**

In `tests/test_arena_artifact.py`:
- `test_publish_without_select_keeps_existing_pointer`: publish payload A (default select), then payload B with `select=False`; pointer bytes equal those after A; B's dir exists.
- `test_publish_without_select_first_time_writes_no_pointer`: `select=False` into a fresh root; `current.json` does not exist.
- `test_select_points_at_identity`: publish B unselected, `select_asset_artifact(..., identity=B.identity)`; pointer's `manifest` is `artifacts/arena/rcw2026/<B.identity>/manifest.json`.
- `test_select_rejects_unknown_identity` and `test_select_rejects_mutated_artifact` (overwrite one payload file, expect `AssetArtifactError`, pointer unchanged).

In `tests/test_arena_import_cli.py`:
- `test_import_does_not_select`: pre-write a pointer, run `run_import` with `StubHooks`; pointer bytes unchanged.
- `test_select_flag_selects_without_import`: `main(["--config", cfg, "--select", identity])` with a published unselected artifact in a temp repo root; pointer moves; no checkout is cloned (patch `clone_pin` to fail if called). `main` currently hard-codes `repo_root = ROOT`; give `main` a keyword-only `repo_root: Path | None = None` used only by tests.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_arena_artifact.py tests/test_arena_import_cli.py -q`
Expected: the new tests FAIL (unexpected keyword `select`, no `select_asset_artifact`, no `--select`).

- [ ] **Step 3: Implement**

`select=False` skips only the final `_atomic_write(... "current.json" ...)`. `run_import` passes `select=False`, and its closing print says the artifact is **not** selected and names the `--select <identity>` command. `--select` is mutually exclusive with `--report-bounds` and `--reuse-furniture-from`.

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_arena_artifact.py tests/test_arena_import_cli.py -q`
Expected: all PASS, including the existing ones.

- [ ] **Step 5: Commit**

```bash
git add tools/tinker_sim_deploy/arena_artifact.py tools/arena_import.py tests/test_arena_artifact.py tests/test_arena_import_cli.py
git commit -m "feat(arena-import): publish without selecting; explicit --select"
```

### Task 2: The sim can load an unselected arena artifact

**Files:**
- Modify: `validation/run_sim.py` (`resolve_arena_artifact`, ~l.397)
- Test: `tests/test_run_sim_arena_cli.py`

**Interfaces:**
- Produces: env var `TINKER_SIM_ARENA_ARTIFACT_DIR`. When set and non-empty, `resolve_arena_artifact(root, arena_id)` returns that directory instead of following `current.json`, after checking it is a direct child of `root/artifacts/arena/<arena_id>/`, contains `manifest.json`, `arena.usd` and `map.yaml`, and the manifest's `"identity"` equals the directory name. Any failed check raises `ValueError` naming the check. On success it prints one line to stderr: `arena artifact override: <dir>`. Unset or empty: today's behaviour, unchanged. (`run_sim` doesn't import `tools/`, so no `verify_asset_artifact` here.)

- [ ] **Step 1: Write the failing tests** in `tests/test_run_sim_arena_cli.py`, reusing its temp-root fixture:
`test_override_env_returns_that_dir`, `test_override_outside_arena_kind_rejected` (a dir under `artifacts/arena/other/`), `test_override_identity_mismatch_rejected`, `test_empty_override_follows_pointer`. Set/clear the env var with `unittest.mock.patch.dict(os.environ, ...)`.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_run_sim_arena_cli.py -q`
Expected: the override tests FAIL; the existing tests PASS.

- [ ] **Step 3: Implement** the override at the top of `resolve_arena_artifact`.

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_run_sim_arena_cli.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add validation/run_sim.py tests/test_run_sim_arena_cli.py
git commit -m "feat(run_sim): TINKER_SIM_ARENA_ARTIFACT_DIR loads an unselected arena"
```

### Task 3: CoACD decomposition module

**Files:**
- Create: `tools/tinker_sim_deploy/arena_coacd.py`
- Modify: `config/arena-import.json` (add `"coacd"` block)
- Test: `tests/test_arena_coacd.py`

**Interfaces:**
- Produces:
  ```python
  @dataclass(frozen=True)
  class CoacdParams:
      threshold: float; max_convex_hull: int; preprocess_mode: str
      preprocess_resolution: int; resolution: int; mcts_nodes: int
      mcts_iterations: int; mcts_max_depth: int; pca: bool; merge: bool
      max_ch_vertex: int; seed: int
      plate_surfaces: tuple[str, ...] = ()
      @classmethod
      def from_config(cls, config: Mapping[str, object]) -> "CoacdParams": ...
      def as_record(self) -> dict[str, object]: ...   # JSON-ready, sorted keys

  @dataclass(frozen=True)
  class ConvexPiece:
      vertices: np.ndarray   # (N, 3) float64, model-local metres
      faces: np.ndarray      # (M, 3) int64 triangle indices

  def decompose(vertices: np.ndarray, faces: np.ndarray, params: CoacdParams) -> list[ConvexPiece]: ...
  ```
- `from_config` raises `ValueError` on a missing `"coacd"` block or missing/extra keys (fail closed, like `load_lidar_spec`).
- `decompose` calls `coacd.run_coacd(coacd.Mesh(vertices, faces), ...)` with every param except `plate_surfaces`, and returns pieces sorted by (min z, min y, min x) of their vertices so output order is stable.

- [ ] **Step 1: Add the config block** to `config/arena-import.json`, with these starting values (Task 8 may tune `threshold`):
  ```json
  "coacd": {
    "_comment": "CoACD convex decomposition of every furniture visual mesh (collision_overrides keep precedence). max_ch_vertex 64: PhysX's convexHull cooking caps hulls at 64 vertices by default, so larger pieces would be re-simplified at load time and stop matching what the outline check measured. seed fixed for reproducible artifacts.",
    "threshold": 0.02, "max_convex_hull": -1, "preprocess_mode": "auto",
    "preprocess_resolution": 50, "resolution": 2000, "mcts_nodes": 20,
    "mcts_iterations": 150, "mcts_max_depth": 3, "pca": false, "merge": true,
    "max_ch_vertex": 64, "seed": 0, "plate_surfaces": []
  }
  ```

- [ ] **Step 2: Write the failing tests** in `tests/test_arena_coacd.py` (`pytest.importorskip("coacd")`). The L-shaped test mesh is `trimesh.util.concatenate` of two non-overlapping boxes that share a face: 1.0×0.2×0.2 at the origin, and 0.2×0.8×0.2 centred at (0.4, 0.5, 0). Its volume is the sum of the two boxes'.
- `test_from_config_reads_repo_config`: `CoacdParams.from_config(json.load(open("config/arena-import.json")))` gives `max_ch_vertex == 64`, `seed == 0`, `plate_surfaces == ()`.
- `test_from_config_rejects_missing_key` and `test_from_config_rejects_unknown_key`.
- `test_l_shape_splits_into_convex_pieces`: at least 2 pieces; every piece is convex (`trimesh.Trimesh(p.vertices, p.faces).is_convex`); every piece has ≤ 64 vertices; the sum of piece volumes is within 5% of the L's volume.
- `test_decompose_is_deterministic`: two calls give pieces with `np.array_equal` vertices and faces.

- [ ] **Step 3: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_arena_coacd.py -q`
Expected: FAIL (`No module named 'tinker_sim_deploy.arena_coacd'`).

- [ ] **Step 4: Implement** `arena_coacd.py`.

- [ ] **Step 5: Run the tests**

Run: `scripts/pytest-clean tests/test_arena_coacd.py -q`
Expected: all PASS. If `test_decompose_is_deterministic` fails, CoACD's threading is non-deterministic: set `OMP_NUM_THREADS=1` inside `decompose` (save and restore the env var around the call) and re-run. Report it if that doesn't fix it.

- [ ] **Step 6: Commit**

```bash
git add tools/tinker_sim_deploy/arena_coacd.py config/arena-import.json tests/test_arena_coacd.py
git commit -m "feat(arena-import): pinned CoACD decomposition module"
```

### Task 4: Author and read CoACD colliders in furniture USDs

**Files:**
- Modify: `tools/tinker_sim_deploy/arena_convert.py` (new functions next to `author_model_colliders`)
- Test: `tests/test_arena_coacd_usd.py`

**Interfaces:**
- Consumes: `CoacdParams`, `ConvexPiece`, `decompose` (Task 3).
- Produces:
  - `read_visual_mesh(usd_path: Path) -> tuple[np.ndarray, np.ndarray]`: every `UsdGeom.Mesh` under `<default prim>/geom`, points transformed to the default prim's frame (= model-local; `/World` has no ops) with `UsdGeom.XformCache`, faces fan-triangulated, all merged into one `(vertices, faces)`.
  - `author_coacd_colliders(usd_path: Path, params: CoacdParams) -> int`: runs `decompose(*read_visual_mesh(usd_path), params)` and writes each piece as `UsdGeom.Mesh` at `<default prim>/Colliders/hull_NN` (`NN` from 00, sorted order), with `points`, `faceVertexCounts` (all 3), `faceVertexIndices`, `UsdPhysics.CollisionAPI`, `UsdPhysics.MeshCollisionAPI` approximation `"convexHull"`, made invisible. Visual meshes get **no** collision API. Saved with `_resave_fresh`. Returns the piece count.
  - `read_collider_pieces(usd_path: Path) -> list[ConvexPiece]`: every collider under `<default prim>/Colliders` as a piece in model-local frame: `hull_*` meshes as stored, and `box_*` cubes (from `author_model_colliders`) as their 8 corners and 12 triangles after their translate/rotateZ/scale ops. Used by the outline check so overrides and plates are checked the same way as hulls.
- `strip_model_colliders` already removes the `Colliders` scope, so hulls are stripped by it unchanged. A test proves it.

- [ ] **Step 1: Write the failing tests** in `tests/test_arena_coacd_usd.py` (`importorskip` pxr and coacd). A helper writes a temp furniture USD shaped like a real one: default prim `/World`, `/World/geom` with a translate op, two `Mesh` children, one using quad faces, together forming an L.
- `test_read_visual_mesh_triangulates_and_merges`: all faces are triangles; vertex count = sum of both meshes'; the bounds include the `/World/geom` translate.
- `test_author_coacd_colliders_writes_invisible_convex_hulls`: return value ≥ 2; every `hull_NN` has `CollisionAPI`, approximation `convexHull`, visibility `invisible`; no prim under `/World/geom` has `MeshCollisionAPI` or `CollisionAPI`.
- `test_read_collider_pieces_round_trips_hulls_and_boxes`: after `author_coacd_colliders` plus `author_model_colliders(usd, (BoxCollider(size=(0.2,0.2,0.01), center=(0,0,0.7), yaw=0.5),))`, `read_collider_pieces` returns hulls + 1 box piece; the box piece's vertices match the rotated corners to 1e-9.
- `test_strip_removes_coacd_hulls`: after `strip_model_colliders`, no prim under `/World/Colliders`.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_arena_coacd_usd.py -q`
Expected: FAIL (`module 'tinker_sim_deploy.arena_convert' has no attribute 'read_visual_mesh'`).

- [ ] **Step 3: Implement** the three functions in `arena_convert.py`, importing `pxr` inside each function as the module does.

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_arena_coacd_usd.py tests/test_arena_convert.py tests/test_arena_collision_overrides.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/tinker_sim_deploy/arena_convert.py tests/test_arena_coacd_usd.py
git commit -m "feat(arena-convert): author and read CoACD hull colliders"
```

### Task 5: Outline check (hulls vs map boxes, 5 mm)

**Files:**
- Modify: `tools/tinker_sim_deploy/arena_map.py` (extract per-instance footprints; no behaviour change)
- Create: `tools/tinker_sim_deploy/arena_outline.py`
- Test: `tests/test_arena_outline.py`; existing `tests/test_arena_map.py` must stay green

**Interfaces:**
- Consumes: `ConvexPiece` (Task 3), `ArenaLayout`, `FurniturePlacement`, `BoxCollider` (`arena_world`).
- Produces in `arena_map.py`: `instance_footprints(item: FurniturePlacement, boxes: Sequence[BoxCollider], band: tuple[float, float]) -> list[tuple[float, float, float, float, float]]`, the same `(cx, cy, half_x, half_y, yaw)` rectangles `_footprints_at` builds for one furniture item. `_footprints_at` calls it.
- Produces in `arena_outline.py`:
  ```python
  TOLERANCE_MM = 5.0
  RESOLUTION_M = 0.001

  def band_polygons(pieces: Sequence[ConvexPiece], item: FurniturePlacement,
                    band: tuple[float, float]) -> list[np.ndarray]: ...
      # each piece -> world (Rz(item.yaw) @ v + item.position), clipped to
      # low <= z <= high (trimesh slice_plane twice, cap=False), projected to
      # XY, 2-D convex hull (scipy.spatial.ConvexHull); pieces with < 3
      # non-collinear projected points are dropped. Returns (K, 2) arrays.

  @dataclass(frozen=True)
  class OutlineDiff:
      bigger_mm: float     # max distance of a hull-only cell from the map shape
      smaller_mm: float    # max distance of a map-only cell from the hull shape
      origin_xy: tuple[float, float]
      hull_mask: np.ndarray   # bool, rows = +y up, 1 mm cells
      map_mask: np.ndarray

  def compare_outlines(hull_polygons: Sequence[np.ndarray],
                       map_rects: Sequence[tuple[float, float, float, float, float]],
                       resolution: float = RESOLUTION_M) -> OutlineDiff: ...

  @dataclass(frozen=True)
  class InstanceOutline:
      instance: str        # model_id + compose_arena suffix, e.g. "rcw26_shelf_02"
      model_id: str
      bigger_mm: float
      smaller_mm: float
      passed: bool         # both <= TOLERANCE_MM
      diff: OutlineDiff

  def check_arena(layout: ArenaLayout,
                  map_colliders: Mapping[str, tuple[BoxCollider, ...]],
                  pieces: Mapping[str, Sequence[ConvexPiece]],
                  band: tuple[float, float]) -> list[InstanceOutline]: ...

  def write_outline_report(results: Sequence[InstanceOutline], out_dir: Path) -> Path: ...
      # writes out_dir/summary.json ({"tolerance_mm", "passed", "instances": [
      # {instance, model_id, bigger_mm, smaller_mm, passed}]}, rounded to 0.1 mm)
      # and out_dir/<instance>.png per instance; returns summary path.
  ```
- `compare_outlines` algorithm, which the signature doesn't fix: rasterize both shapes on one 1 mm grid covering their union bbox plus 10 mm (a cell is in a polygon or rectangle if its centre passes every edge's half-plane test, vectorized in numpy). Then `bigger_mm = max(distance_transform_edt(~map_mask)[hull_mask & ~map_mask]) * 1000 * resolution` (0 if empty), and `smaller_mm` likewise with the masks swapped. An empty hull shape gives `smaller_mm` = the largest distance inside the map shape, which fails.
- Instance names come from `arena_import._furniture_instance_suffixes`. Move that helper into `arena_outline.py` (or a shared spot) and import it back in `arena_import.py`, rather than duplicating it.
- PNG colours: both = light grey, hull-only = red, map-only = blue, white elsewhere. A text line at the top (Pillow `ImageDraw.text`) gives instance, bigger/smaller mm and PASS/FAIL. Downsample to at most 1000 px on the long side.

- [ ] **Step 1: Write the failing tests** in `tests/test_arena_outline.py` (`importorskip` trimesh and scipy). Pieces are boxes built with `trimesh.creation.box`.
- `test_identical_box_passes`: hull = map box 0.8×0.6×0.72 at the origin → `bigger_mm < 1.5`, `smaller_mm < 1.5`, passed.
- `test_hull_10mm_bigger_fails`: hull box 10 mm wider on +x → `9 <= bigger_mm <= 11`, not passed.
- `test_hull_4mm_bigger_passes`: 4 mm wider → passed.
- `test_round_table_inscribed_in_box_fails_smaller`: hull = 32-sided prism radius 0.4 inside a 0.8×0.8 box → `smaller_mm > 150` (corner gap ≈ 166 mm), not passed.
- `test_piece_outside_band_contributes_nothing`: a 0.1 m plinth piece below band low → `band_polygons` returns `[]`.
- `test_yaw_and_position_applied`: item yaw π/2 at (2, -3, 0); map box and hull box both rotate the same way → passed.
- `test_duplicate_instances_reported_separately`: layout with two placements of one model gives instances `"m"` and `"m_02"`.
- `test_report_writes_summary_and_pngs`: `summary.json` has `"tolerance_mm": 5.0` and one PNG per instance.
- In `tests/test_arena_map.py`, add `test_rasterize_unchanged_by_refactor`: a synthetic layout with two walls and two furniture items carrying yawed boxes. **Before** touching `arena_map.py`, compute the sha256 of `rasterize(...)`'s pgm bytes and paste it into the test as a constant. The test asserts the same sha256 after the refactor.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_arena_outline.py tests/test_arena_map.py -q`
Expected: outline tests FAIL (module missing); map tests PASS.

- [ ] **Step 3: Implement** `instance_footprints`, then `arena_outline.py`.

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_arena_outline.py tests/test_arena_map.py tests/test_arena_import_cli.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/tinker_sim_deploy/arena_map.py tools/tinker_sim_deploy/arena_outline.py tools/arena_import.py tests/test_arena_outline.py tests/test_arena_map.py
git commit -m "feat(arena-outline): 5 mm hull-vs-map outline check with diff report"
```

### Task 6: Wire CoACD and the outline check into the import

**Files:**
- Modify: `tools/arena_import.py` (`ConverterHooks`, `_convert_model`, `convert_furniture_model`, `run_import`, `ReuseFurnitureHooks`, `main`)
- Test: `tests/test_arena_import_cli.py` (extend `StubHooks`)

**Interfaces:**
- Consumes: Tasks 1, 3, 4, 5.
- `ConverterHooks` gains:
  - `author_coacd_colliders(self, usd_path: Path, params: CoacdParams) -> int`
  - `read_collider_pieces(self, usd_path: Path) -> list[ConvexPiece]`

  `ReuseFurnitureHooks` forwards both to `arena_convert`; the real converter is `arena_convert` itself.
- `_convert_model(checkout, scratch, model_id, converter, collider_override=None, coacd: CoacdParams | None = None, plates: tuple[BoxCollider, ...] = ())`. With `coacd=None` (only `report_bounds` passes that) it behaves exactly as today. Otherwise: if the model has a `collision_overrides` entry → `author_model_colliders(usd, override_boxes)` as today; else → `author_coacd_colliders(usd, coacd)`, then `author_model_colliders(usd, plates)` if `plates` is non-empty. `run_import` builds `plates` for models in `coacd.plate_surfaces`. Plates are one `BoxCollider(size=(sx, sy, 0.01), center=(cx, cy, cz - 0.005), yaw=0.0)` per config `surfaces` entry for that model (`local_center` = (cx, cy, cz), `size_xy` = (sx, sy)). It still returns the **SDF** colliders, so `convert_furniture_model`'s `map_colliders` logic and bounds check are unchanged.
- `run_import(config, repo_root, checkout, converter) -> ImportOutcome`:
  ```python
  @dataclass(frozen=True)
  class ImportOutcome:
      publication: AssetPublication
      outlines: tuple[InstanceOutline, ...]
      report: Path          # summary.json
  ```
  After conversion it reads `converter.read_collider_pieces(...)` for each model, runs `check_arena(layout, map_colliders, pieces, scan_band)`, adds payload `collision.json` = `{"coacd": params.as_record(), "pieces": {model_id: count}, "overrides": [sorted ids], "plates": [sorted ids]}` (canonical JSON, sorted keys), publishes unselected (Task 1), then writes the report to `repo_root/reports/arena-outline/<identity[:12]>/`. It prints one line per failing instance and a final `outline check: PASS` or `outline check: FAIL (<n> instances)`.
- `main` exits 0 when publish succeeded, even if the outline check fails (the artifact is unselected anyway), and prints the report path.

- [ ] **Step 1: Write the failing tests** in `tests/test_arena_import_cli.py`. Extend `StubHooks` with `author_coacd_calls`, `author_box_calls` and a configurable `pieces` mapping returned by `read_collider_pieces` (default: one box piece equal to the model's map box, so outlines pass). Add a `"coacd"` block to `_base_config`.
- `test_coacd_authored_for_models_without_override`: the kitchen table gets an `author_coacd_colliders` call and no SDF-box `author_model_colliders` call.
- `test_override_models_keep_boxes`: with a `collision_overrides` entry for the model, no CoACD call; one box call with the override boxes.
- `test_plate_added_for_listed_surface`: `plate_surfaces: ["rcw26_kitchen_table"]` → a box call with exactly one plate whose top z equals the surface `local_center` z (0.74) and size_xy (1.2, 0.6).
- `test_map_unchanged_by_coacd`: patch `arena_import.rasterize` with a recorder that calls the real one; the map colliders it receives for the kitchen table equal the SDF box colliders, not the stub's hull pieces.
- `test_outline_failure_reported_not_selected`: stub pieces 20 mm bigger → `outcome.outlines[0].passed is False`, `summary.json` exists with `"passed": false`, pointer untouched.
- `test_collision_record_in_payload`: `collision.json` in the artifact has the params and the piece counts.
- `test_reimport_is_idempotent_with_coacd`: two `run_import`s with identical stubs give the same `publication.identity`.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_arena_import_cli.py -q`
Expected: the new tests FAIL.

- [ ] **Step 3: Implement** the wiring above.

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_arena_import_cli.py tests/test_arena_artifact.py tests/test_arena_outline.py tests/test_arena_coacd.py tests/test_arena_coacd_usd.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/arena_import.py tests/test_arena_import_cli.py
git commit -m "feat(arena-import): CoACD colliders, outline report, collision record"
```

### Task 7: The support-surface box override leaves CoACD hulls alone

**Files:**
- Create: `simulation/tinker_sim_isaac/arena_surface_colliders.py`
- Modify: `simulation/tinker_sim_isaac/backend.py:2225-2275` (`_apply_arena_support_surface_colliders` uses the new helper)
- Test: `tests/test_arena_surface_colliders.py`

**Interfaces:**
- Produces: `support_surface_box_targets(stage, furniture_paths: Iterable[str]) -> list` (pxr prims). These are the prims under each furniture path that have `MeshCollisionAPI` **and** are not inside a prim named `Colliders`, i.e. legacy whole-visual-mesh hulls only. The backend sets `boundingCube` on exactly these. Its JSON print and return value stay the same.

- [ ] **Step 1: Write the failing tests** (`importorskip` pxr). Build an in-memory stage with `/World/Arena/Furniture/rcw26_kitchen_table` holding (a) a visual mesh with `MeshCollisionAPI` and (b) `Colliders/hull_00` with `MeshCollisionAPI`:
- `test_legacy_visual_mesh_colliders_still_boxed`: (a) is returned.
- `test_coacd_hulls_not_boxed`: (b) is not returned.
- `test_missing_furniture_path_skipped`.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_arena_surface_colliders.py -q`
Expected: FAIL (module missing).

- [ ] **Step 3: Implement** the helper and call it from the backend method (keep the backend's `placement.json` target derivation as is).

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_arena_surface_colliders.py tests/test_backend_arena_inputs.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add simulation/tinker_sim_isaac/arena_surface_colliders.py simulation/tinker_sim_isaac/backend.py tests/test_arena_surface_colliders.py
git commit -m "fix(backend): support-surface box override skips CoACD hulls"
```

### Task 8: Build the candidate arena, then stop at the user checkpoint

No product code. This task runs the import on CPU (no Kit, no GPU) and brings the outline report to the user.

- [ ] **Step 1: Import Kit-free from the current artifact's converted furniture**

```bash
cd /home/tinker/tinker-sim/6.0.1
CUR=$(python3 -c "import json;print(json.load(open('artifacts/arena/rcw2026/current.json'))['manifest'].rsplit('/',1)[0])")
timeout -k 10 3600 .venv/bin/python tools/arena_import.py --config config/arena-import.json --reuse-furniture-from "$CUR" 2>&1 | tee reports/arena-import-coacd-$(date +%F).log
```
Expected: `published arena artifact rcw2026: identity=<id>`, a line saying it is not selected, `outline check: PASS` or `FAIL (<n> instances)`, and the report path. `artifacts/arena/rcw2026/current.json` is unchanged.

- [ ] **Step 2: Confirm the map is byte-identical**

Run: `cmp artifacts/arena/rcw2026/<new id>/map.pgm "$CUR/map.pgm" && cmp artifacts/arena/rcw2026/<new id>/map.yaml "$CUR/map.yaml" && echo SAME`
Expected: `SAME`. If not, stop: something changed the map source, and that's a bug in Task 6.

- [ ] **Step 3: Separate CoACD coarseness from map disagreement**

For each failing instance, open its PNG. A failure where the red/blue band follows the mesh's real shape (a round top, plant leaves, a chair's open back) is a map-vs-mesh disagreement. A failure where hull pieces bridge across a real gap or bulge past a flat face is CoACD coarseness. For coarseness only, lower `coacd.threshold` (0.02 → 0.01 → 0.005), re-run Step 1, and keep the lowest value that removes it. Commit any threshold change:
```bash
git add config/arena-import.json
git commit -m "tune(arena-import): CoACD threshold <value> for the outline check"
```

- [ ] **Step 4: USER CHECKPOINT. Stop here.**

Bring the user `summary.json`'s failing instances (instance, bigger mm, smaller mm, your one-line read of the PNG) and the PNG paths. The spec says a failing model blocks the switch until the user has decided. Don't change the check's tolerance. Don't select. The user's per-model decisions become follow-up work, planned then. Continue to Task 9 only when every instance passes or the user has explicitly accepted the remaining ones.

### Task 9: Live placement check, RTF report, then select on the user's word

**Files:**
- Create: `validation/placement_settle_check.py`
- Test: `tests/test_placement_settle_check.py`

**Interfaces:**
- Produces, as pure functions (no ROS import at module top; `rclpy` only inside `main`):
  - `spawn_poses(placement: Mapping[str, object], drop_m: float = 0.05) -> list[tuple[str, tuple[float, float, float]]]`: one `(surface_id, xyz)` per `placement.json` surface, at its `center_xyz` plus `drop_m` in z.
  - `judge_rest(surface: Mapping[str, object], first_xyz, last_xyz, settle_tol_m: float = 0.005) -> tuple[bool, str]`: passes when `last_xyz` lies inside the surface rectangle (`center_xyz`, `size_xy`, `yaw`) shrunk by `edge_margin`, `surface_z - 0.01 <= last_z <= surface_z + 0.15`, and `|last - first| <= settle_tol_m`. Reasons, checked in this order: `"fell off"` (XY outside the shrunk rectangle), `"sank"` (inside, `last_z < surface_z - 0.01`), `"floating"` (inside, `last_z > surface_z + 0.15`), `"still moving"`, else `"rests"`.
- `main(argv)`: `--placement PATH --asset ycb_010_tomato_soup_can --settle-s 8 --out PATH`. Waits for the sim's services (reuse `gpsr_spawn._wait_for_services_ready` and `_make_ros_service_client` from `tools/gpsr_spawn.py`; spawn as `/World/Scenario/settle_<n>` with a `SpawnItem` from `tools/gpsr_scene.py`). Spawns one object per surface, waits `--settle-s` of **sim** time on `/clock`, then reads two `/sim/truth/object_state` samples 1 s of sim time apart. Spawned bodies report zero twist, so use the displacement between samples. Writes a JSON verdict per surface and exits 1 if any fail. Resolve the asset path from `artifacts/objects/ycb/current.json`.

- [ ] **Step 1: Write the failing tests** for `spawn_poses` and `judge_rest`, using the real candidate `placement.json` shape (seven surfaces, e.g. kitchen table `center_xyz` [2.587, -3.058, 0.724], yaw 1.5708): `test_seven_spawns_5cm_above`, `test_rest_on_surface_passes`, `test_off_the_edge_fails_fell_off`, `test_below_surface_inside_rect_fails_sank`, `test_moving_fails`, `test_yawed_rectangle_respected` (a point inside the yawed rect but outside the unrotated one passes).

- [ ] **Step 2: Run them to see them fail, implement, run them to pass**

Run: `scripts/pytest-clean tests/test_placement_settle_check.py -q`
Expected: FAIL before, PASS after implementing.

- [ ] **Step 3: Commit**

```bash
git add validation/placement_settle_check.py tests/test_placement_settle_check.py
git commit -m "feat(validation): live placement-settle check for arena surfaces"
```

- [ ] **Step 4: Run it live on the candidate** (GPU, domain and timeout rules in Global Constraints). In shell 1:
```bash
cd /home/tinker/tinker-sim/6.0.1; set -a; source .deployment.env; set +a
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH
export ROS_DOMAIN_ID=<private> TINKER_SIM_ARENA_ARTIFACT_DIR=$PWD/artifacts/arena/rcw2026/<new id>
timeout -k 10 900 ./scripts/tinker-sim launch --sensor-profile sensor-rich --scenario find-and-approach-person-rcw2026 --seed 0 --arena rcw2026 --spawn-xy=-2.0,-2.0 --ros --headless --duration 300 2>&1 | tee reports/settle-sim-$(date +%F).log
```
(`find-and-approach-person-rcw2026` declares no objects, so nothing else sits on the surfaces. Its one person walks near (−2, −3.5), away from them.)
In shell 2 (ROS sourced as in `docs/gpsr-sim-runbook.md` Stage 2, same domain), start the truth evaluator with `timeout -k 10 900 ros2 launch tinker_sim_bridge manipulation.launch.py project_root:=$PWD tinker_workspace:=/home/tinker/tk25_ws scenario:=find-and-approach-person-rcw2026 seed:=0` (it brings up `truth_evaluator`). Then in shell 3:
```bash
timeout -k 10 600 python3 validation/placement_settle_check.py --placement artifacts/arena/rcw2026/<new id>/placement.json --out reports/placement-settle-$(date +%F).json
```
Expected: the sim log has `arena artifact override:` and `{"arena_surface_boxes": 0}` (CoACD hulls aren't boxed), and the check exits 0 with all seven surfaces passing.
If a surface fails, add its model to `coacd.plate_surfaces` in `config/arena-import.json`, re-run Task 8 Steps 1–2 (and re-check outlines), then repeat this step. Commit the config change as in Task 8 Step 3.
Tear down: `scripts/kill-domain-nodes <private> --force`.

- [ ] **Step 5: RTF before/after**

Run each variant once, same settings, alternating current then candidate. The only difference is `TINKER_SIM_ARENA_ARTIFACT_DIR` (unset for current, the new dir for candidate):
```bash
TINKER_SIM_PHYSICS_HZ=40 TINKER_SIM_CONTROL_HZ=40 TINKER_SIM_PROFILE=1 TINKER_SIM_PROFILE_EVERY=10 \
timeout -k 10 900 ./scripts/tinker-sim launch --sensor-profile navigation-parity --scenario gpsr-rcw2026 --seed 0 --arena rcw2026 --spawn-xy=-2.0,-2.0 --ros --headless --duration 120 2>&1 | tee reports/rtf-<variant>-$(date +%F).log
python3 tools/step_profile_summary.py <variant> reports/rtf-<variant>-$(date +%F).log --skip-first-s 30
```
Expected: both logs contain `live raycast lidar:` (else the run used the occupancy lidar and the numbers are void). Record the two RTF rows.

- [ ] **Step 6: Ask the user to confirm the switch.** Report the placement results and both RTF numbers. Selecting changes every rcw2026 run in this shared checkout, including other sessions'. Wait for an explicit yes.

- [ ] **Step 7: Select and register** (only after the yes)

```bash
.venv/bin/python tools/arena_import.py --config config/arena-import.json --select <new id>
```
Expected: `artifacts/arena/rcw2026/current.json` names the new manifest. Then register the new `arena.usd` path and sha256 under `generated_arena_usds` in `artifacts/asset-manifest.json`, as the importer's operator reminder says.

---

## Amendment 2026-10-29 — map drawn from the hulls

At Task 8's checkpoint the user chose: "keep the real shape and redraw the map from the hulls, but keep the center and orientation the same." Tasks 10 and 11 implement that; Task 9 runs after them. Spec: Part 1 "What changes" (map bullet) and Checks 1 (checkpoint outcome) and 1b.

### Task 10: Map furniture drawn from hull outlines; centre-shift gate

**Files:**
- Modify: `tools/tinker_sim_deploy/arena_map.py` (`rasterize` gains `furniture_outlines`)
- Modify: `tools/tinker_sim_deploy/arena_outline.py` (`instance_outlines`, centre-shift gate, `write_map_diff`)
- Modify: `tools/arena_import.py` (`run_import` draws the map from outlines and checks the frame)
- Test: `tests/test_arena_map.py`, `tests/test_arena_outline.py`, `tests/test_arena_import_cli.py`

**Interfaces:**
- Consumes: `band_polygons`, `check_arena`, `InstanceOutline`, `write_outline_report`, `_furniture_instance_suffixes` (Task 5); `instance_footprints` (Task 5, arena_map); `ImportOutcome` and the `run_import` flow (Task 6).
- Produces in `arena_outline.py`:
  - `instance_outlines(layout: ArenaLayout, pieces: Mapping[str, Sequence[ConvexPiece]], band: tuple[float, float]) -> dict[str, list[np.ndarray]]`: instance name → that instance's `band_polygons` (world XY), keyed with the same `_furniture_instance_suffixes` names `check_arena` uses.
  - `CENTER_TOLERANCE_MM = 25.0` (half a 5 cm map cell).
  - `InstanceOutline` gains `center_shift_mm: float`: distance between the bbox centre (world XY) of the instance's hull outline and the bbox centre of its map-box outline (`instance_footprints` rectangles' corners); `inf` when either is empty. `passed` becomes: `center_shift_mm <= CENTER_TOLERANCE_MM`, or both shapes empty. `bigger_mm`/`smaller_mm` stay as information. `TOLERANCE_MM` stays defined (it is still printed as the old outline rule) but no longer gates.
  - `summary.json` gains per instance `center_shift_mm` (null when infinite) and top-level `"center_tolerance_mm": 25.0`; `"passed"` follows the new rule. PNG titles add `centre shift <x> mm`.
  - `write_map_diff(old_pgm: bytes, new_pgm: bytes, path: Path) -> None`: same-size P5 images → PNG with cells occupied in both = grey, old only = blue, new only = red, free = white. Raises `ValueError` if sizes differ.
- Produces in `arena_map.py`: `rasterize(..., furniture_outlines: Mapping[str, Sequence[np.ndarray]] | None = None)`.
  - `None` (default): byte-identical to today (pinned by `test_rasterize_unchanged_by_refactor`).
  - Given: the grid origin, width and height are computed exactly as today from walls + `furniture_box_colliders` (so `map.yaml` is byte-identical); walls are rasterized exactly as today; furniture boxes are **not** drawn; instead each instance's polygons are rasterized on a 0.005 m sub-grid over that instance's bbox window (sub-cell centre inside a convex polygon by half-plane test), enclosed hollows filled per instance with `scipy.ndimage.binary_fill_holes`, and a map cell is occupied if **any** of its sub-cells is. numpy/scipy are imported only in this branch (the default path stays importable under system python).
- `run_import`: computes `outlines = instance_outlines(layout, pieces, scan_band)`, builds the box map exactly as before (`box_pgm, box_yaml = rasterize(layout, map_colliders, scan_band=scan_band)`), then the published map with `rasterize(..., furniture_outlines=outlines)`; raises `ArenaImportError`/`RuntimeError` if the new `map.yaml` bytes differ from `box_yaml`. It writes `map-diff.png` (old box map vs new map) into the outline report dir, and prints `map: furniture drawn from hulls; map.yaml unchanged`.

- [ ] **Step 1: Write the failing tests**
  - `tests/test_arena_map.py::test_outlines_change_only_furniture_cells`: a synthetic layout with walls and one 0.8×0.8 box-declared table whose outline is a 32-gon of radius 0.4 → `map.yaml` bytes equal the box map's; every wall cell equal; the four corner cells of the table box are free; the centre cell occupied.
  - `test_thin_ring_rasterizes_and_fills`: an outline made of 64 thin convex quads forming a 1 cm-thick ring of radius 0.2 → every cell whose centre is within 0.18 m of the ring centre is occupied (filled), and cells farther than 0.27 m are free.
  - `test_default_rasterize_unchanged`: the existing sha test still passes with no `furniture_outlines`.
  - `tests/test_arena_outline.py::test_center_shift_gate`: hull box shifted 30 mm → `center_shift_mm ≈ 30`, not passed; shifted 10 mm → passed even though `bigger_mm` ≈ 10; round hull inside a square map box (centred) → passed, `smaller_mm` > 150.
  - `test_missing_hull_fails_gate`: map footprint present, no hull in band → `center_shift_mm == inf`, not passed.
  - `test_write_map_diff_colours` and `test_write_map_diff_rejects_size_mismatch`.
  - `tests/test_arena_import_cli.py::test_map_drawn_from_outlines_frame_unchanged`: with stub pieces smaller than the map box, the published `map.yaml` equals the box map's yaml (compute it with `rasterize(layout, map_colliders, scan_band=...)` in the test), `map.pgm` differs, and `map-diff.png` exists in the report dir.

- [ ] **Step 2: Run them to see them fail**

Run: `scripts/pytest-clean tests/test_arena_map.py tests/test_arena_outline.py tests/test_arena_import_cli.py -q`
Expected: the new tests FAIL; existing ones PASS except those whose expectations the new gate changes (update those to the new rule in Step 3 and say which in the report).

- [ ] **Step 3: Implement**

- [ ] **Step 4: Run the tests**

Run: `scripts/pytest-clean tests/test_arena_map.py tests/test_arena_outline.py tests/test_arena_import_cli.py tests/test_arena_artifact.py -q`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add tools/tinker_sim_deploy/arena_map.py tools/tinker_sim_deploy/arena_outline.py tools/arena_import.py tests/test_arena_map.py tests/test_arena_outline.py tests/test_arena_import_cli.py
git commit -m "feat(arena-import): draw map furniture from hull outlines; centre-shift gate"
```

### Task 11: Rebuild the candidate arena with the hull-drawn map

No product code. CPU only, like Task 8, and no user stop unless something below fails.

- [ ] **Step 1: Import** with the same command as Task 8 Step 1 (current settings: `preprocess_resolution` 100; reuse the SOBITS checkout), in the background, `timeout -k 10 3600`.
Expected: `outline check: PASS` under the centre-shift gate, `map: furniture drawn from hulls; map.yaml unchanged`, a new unselected identity, `artifacts/arena/rcw2026/current.json` unchanged.

- [ ] **Step 2: Check the map**
  - `cmp <new>/map.yaml <669e13f7…>/map.yaml` → identical.
  - Using the two `map.pgm` files: every differing cell lies within 0.30 m of some furniture instance's placement position's footprint (walls unchanged). Report the count of cells freed and added, and look at `map-diff.png`.
- [ ] **Step 3: Report** per instance the centre shift (expected ≤ 17 mm, tall plant largest), the piece counts from `collision.json` (trash bin ~119, laundry basket ~641), and the `map-diff.png` path. If any instance fails the gate or a wall cell changed, stop and report to the user instead of continuing to Task 9.
