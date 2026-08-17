# RoboCup 2026 arena in Isaac Sim — design

- Date: 2026-08-17
- Status: approved design, pre-implementation
- Scope owner decision record: vision realism + navigation on the 2026 layout +
  manipulation surfaces; full task rehearsal deferred.

## 1. Context and motivation

Today the simulator's only arena is the committed "RoboCup Arena 3" occupancy
map: `map.yaml`/PGM rasterized by `tinker_sim_core.occupancy.OccupancyMap` and
spawned as flat-colored kinematic cuboids by
`simulation/tinker_sim_isaac/backend.py` under `/World/NavigationMap`.
Scenarios (`simulation/scenarios/*.json`, schema v2) spawn primitives from
`simulation/assets/primitives/` via `asset_uri` + pose under
`/World/Scenario/<id>`; `world` is always `{"mode": "current"}`.

This serves navigation smoke work but fails three goals the team now has:

1. **Vision realism** — RTX cameras should see real textured furniture, not
   colored cuboids, feeding the validated `tk26_vision` hardware-parity stack.
2. **Navigation on the real 2026 floor plan** — rehearse Nav2/AMCL on the
   actual RoboCup@Home 2026 arena layout.
3. **Manipulation surfaces** — pick/place scenarios need accurate surface
   heights (tables, shelf plates) and real task objects (YCB).

## 2. Research summary (2026-08-17 spike)

- No maintained SDF→USD converter exists (gz-omni is archived). Nobody
  publishes a pre-built RoboCup@Home arena for Isaac Sim. Wholesale world-file
  conversion is a dead end; per-asset harvest is the viable path.
- Source: `TeamSOBITS/sobits_gazebo_worlds`, branch **`feature/hri`** — a
  strict superset of the default `jazzy-devel` (22 commits ahead, 0 behind,
  most recently active). It carries:
  - `worlds/rcw2026_arena.world.xacro` — RoboCup@Home 2026 international
    arena replica: 9×9 m outer footprint, four quadrants
    (kitchen/bedroom/dining/living), inline SDF wall boxes, `<include>`
    entries with poses. Despite the extension it is **plain XML, not real
    xacro** (upstream parses it with ElementTree); no xacro toolchain needed.
  - The complete 22-model `rcw26_*` real-furniture family, each
    `model.config` + `model.sdf` + one GLB mesh with embedded textures.
  - License BSD-3-Clause on every branch.
- Task objects are **not** in that repo: YCB models (~50, textured DAE visual
  + STL collision) live in `TeamSOBITS/tmc_wrs_gz` (Toyota WRS fork) —
  wrapper under Clear BSD, YCB content flagged **CC BY 4.0** (attribution is
  a license obligation).
- Human/pose assets live in sibling `TeamSOBITS/gz_human_sim`, which has
  **no license file** — excluded from this design.
- Naming trap: `feature/pick_and_place`'s `rcw26_pnp.world.xacro` is a
  different competition's prop (WRC frame), not the RoboCup arena. Do not
  source from it.
- Upstream's placement YAML system covers only legacy arenas; no placement
  config exists for `rcw2026_arena` on any branch. Surface configs here are
  authored by our importer, not harvested.

## 3. Decision

**Approach A — arena as a content-addressed artifact.** A dev-time importer
converts pinned upstream sources into a published
`artifacts/arena/rcw2026/<hash>/` artifact (arena USD + furniture USDs +
derived `map.yaml` + `placement.json` + provenance), mirroring the robot
artifact machinery. Backend/launcher gain an opt-in arena path; scenarios
declare the arena they require. Strictly additive: the existing Arena 3 map,
robot artifact, and all current behavior are untouched.

Alternatives rejected:

- **B — furniture as committed assets, keep cuboid walls**: fails the vision
  goal (walls stay cuboids), bloats git with binaries and every scenario with
  ~20 furniture entries, leaves wall/map drift unsolved.
- **C — one flattened monolithic arena USD**: no per-asset provenance or
  collider control, nothing to derive surfaces or `map.yaml` from.

## 4. Importer pipeline (`tools/arena_import.py`)

Dev-time tool, run from the simulator venv (needs network and Kit; it is not
part of deployment/runtime). All inputs come from one pinned-source config
`config/arena-import.json`:

```json
{
  "repository": "https://github.com/TeamSOBITS/sobits_gazebo_worlds",
  "branch": "feature/hri",
  "commit": "<40-hex pin>",
  "world": "worlds/rcw2026_arena.world.xacro",
  "arena_id": "rcw2026",
  "model_allowlist": ["rcw26_bed", "rcw26_chair", "..."],
  "surface_furniture": ["rcw26_kitchen_table", "rcw26_shelf", "rcw26_side_table",
                        "rcw26_laundry_desk", "rcw26_sofa"],
  "bounds_tolerance_m": 0.01
}
```

The `commit` value is captured once at implementation start (the then-current
tip of `feature/hri`) and thereafter changes only by deliberate config edit.

Steps:

1. Clone the pinned commit into the session scratchpad (never into the repo);
   verify HEAD equals the pin, fail closed otherwise.
2. Parse the world file with ElementTree: extract inline wall box geometry
   (size + pose) and `<include>` records (model URI, pose, static flag).
   Unknown model URIs not on the allowlist fail the import.
3. Convert each allowlisted GLB to USD via `omni.kit.asset_converter`
   (headless Kit). Author colliders per §6.
4. Compose `arena.usd`: authored wall cuboids + per-furniture USD references
   at their world poses, all static/kinematic, under `/World/Arena`
   (`/World/Arena/Walls/wall_NNNN`, `/World/Arena/Furniture/<model_id>`).
5. Derive `map.pgm` + `map.yaml` (§5) and emit `placement.json` (§6).
6. Self-check: converted-USD bounds vs `model.sdf` declared collider bounds
   within `bounds_tolerance_m`; fail closed on violation.
7. Publish the artifact (§7).

Determinism/identity: the artifact's source lock records repository URL,
commit, ordered per-input-file `path/size/sha256` records, the asset-converter
and importer algorithm versions. Re-running against an unchanged pin must
produce an artifact with identical payload hashes or be a no-op.

## 5. Walls and map — single source of truth

The parsed wall layout is the sole source of truth for both the visible arena
and navigation:

- Walls become authored cuboids in `arena.usd` (real materials, kinematic,
  collidable — same physics convention as today's occupancy cuboids).
- The importer rasterizes wall footprints **plus static furniture collision
  footprints sliced at the tinker2 Livox scan-plane height** (read from the
  robot configuration, not hardcoded) into `map.pgm`/`map.yaml`, at the same
  resolution conventions the existing Arena 3 map uses. Rationale: AMCL's map
  must match what the sim lidar returns; a SLAM-built competition map would
  contain furniture at scan height too.
- `OccupancyMap` consumes the derived `map.yaml` unchanged for truth/raycast.
  Visual arena and nav map cannot drift because both derive from one parse.

## 6. Colliders and placement surfaces

- Default furniture collider: re-authored from that model's `model.sdf`
  collision geometry — boxes pass through as USD box colliders; mesh
  colliders become convex-hull approximations.
- Furniture listed in `surface_furniture` additionally gets hand-verified
  box colliders for each flat placement surface, measured from the mesh by
  the importer and checked against the SDF-declared bounds (§4 step 6),
  because manipulation correctness depends on exact surface heights.
- `placement.json` (in the artifact) records every placement surface in the
  world frame:

```json
{
  "schema_version": 1,
  "arena_id": "rcw2026",
  "surfaces": [
    {
      "surface_id": "kitchen_table#top",
      "furniture_id": "rcw26_kitchen_table",
      "center_xyz": [0.0, 0.0, 0.74],
      "size_xy": [1.2, 0.6],
      "yaw": 0.0,
      "edge_margin": 0.05
    }
  ]
}
```

Increment one consumes `placement.json` as the authoritative reference for
hand-authoring scenario object poses and evaluator `regions`. The deferred
seeded placement generator reads the same file later; nothing else about its
schema is designed now (YAGNI).

## 7. Artifact layout and publication

```
artifacts/arena/rcw2026/<hash>/
  arena.usd
  furniture/<model_id>.usd        (one per allowlisted model)
  map.yaml
  map.pgm
  placement.json
  source-lock.json
  ATTRIBUTION.md
artifacts/arena/rcw2026/current.json   (pointer, sole commit point)
```

Publication reuses the existing robot-artifact machinery and discipline:
content-addressed identity, fsync of files/directories, atomic claim of the
immutable artifact directory, `current.json` replacement as the sole commit
point, nonblocking inter-process publication lock. The artifact is registered
in `artifacts/asset-manifest.json` (generated-USD group) so offline bundles
carry and verify it.

## 8. Backend and launcher integration

- `IsaacWholeRobotBackend` gains `arena_artifact: Path | None`.
  - Set: reference the artifact's `arena.usd` into the stage; build
    `OccupancyMap` from the artifact's colocated `map.yaml`. Passing both
    `arena_artifact` and `map_yaml` is an error (fail closed).
  - Unset: current behavior, byte-for-byte unchanged.
- `validation/run_sim.py` / `scripts/launch-isaac` gain `--arena <id>`:
  resolve `artifacts/arena/<id>/current.json` → verified artifact directory;
  fail closed when absent or unverifiable. `scripts/launch-arena-streaming
  --arena rcw2026` is the visual-inspection path.
- `--arena-colors` remains cuboid-mode-only; combining it with `--arena` is
  rejected with a clear error.

## 9. Scenario schema

- New world mode: `world: {"mode": "arena", "arena": "rcw2026"}`.
  Orchestration validates the declaration against the launcher-selected arena
  and fails closed on mismatch or on an arena scenario run without `--arena`.
  `{"mode": "current"}` scenarios behave exactly as today.
- Arena loading stays a launch-time decision; the unused `load_world`
  operation path is not activated.
- Furniture is arena structure, never `objects[]`. Task objects and actors
  keep using existing `objects[]`/`actors[]` with hand-authored poses drawn
  from `placement.json` surfaces.

## 10. YCB task objects

Separate artifact family via `tools/ycb_import.py` (may share importer
internals), pinned to `TeamSOBITS/tmc_wrs_gz` by commit in
`config/ycb-import.json` with an object allowlist (start with roughly the ten
objects the pick/place scenarios need; exact list chosen at implementation
time and recorded in that config):

```
artifacts/objects/ycb/<hash>/
  <object_id>/object.usd   (DAE visual, convex-decomposed STL collision)
  source-lock.json
  ATTRIBUTION.md
artifacts/objects/ycb/current.json
```

Scenario `asset_uri` already accepts arbitrary repo-relative files, so these
need zero schema change.

## 11. Provenance and licensing

`ATTRIBUTION.md` is generated into each artifact:

- Arena: SOBITS BSD-3-Clause license text plus per-model upstream paths and
  hashes.
- YCB: **CC BY 4.0 attribution block naming the Yale-CMU-Berkeley Object and
  Model Set** (license obligation) plus Toyota's Clear BSD for the wrapper
  repository.
- `gz_human_sim` content is excluded entirely (no upstream license).

The source lock makes either artifact auditable back to exact upstream bytes.

## 12. Testing and acceptance

Unit (system Python, no GPU, `tests/`):

- World-XML layout extraction against a small vendored fixture snippet
  (walls + includes + an unknown-model rejection case).
- Map rasterizer: known synthetic layout → expected occupied cells, origin,
  resolution.
- `placement.json` schema and world-frame math.
- Artifact publication invariants, reusing robot-artifact test patterns
  (atomicity, pointer commit point, lock behavior, manifest registration).
- Scenario arena-declaration validation: mismatch and missing-arena
  fail-closed paths; `mode: current` regression.
- Backend argument validation (`arena_artifact` + `map_yaml` rejection,
  `--arena-colors` + `--arena` rejection) at the CLI-parsing level.

Importer self-checks (run during import, not in CI): converted bounds vs SDF
bounds within `bounds_tolerance_m`.

Live smoke (dev host, recorded as development-validated, explicitly **not
release-qualified**, matching the vision-stack precedent):

- `launch-arena-streaming --arena rcw2026` renders walls + furniture.
- A sensor-rich run shows furniture in head-camera frames.
- AMCL converges on the derived map against the sim lidar in the arena.

## 13. Out of scope (increment one)

Seeded placement generator; GPSR/LLM scenario generation; human pose library
(`gz_human_sim`); `rcw2026_hri` and RCJO arena variants (the importer takes
the world file as config, so these are cheap follow-ups); door articulation;
release qualification of any of the above.
