# Shape Outline Height Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Filled shapes' outline rings take the shape's height. Where a ring is part of a wall line it takes
max(wall, shape). The rule must be identical in the saved arena (Python), the live 3D preview and the 2D map
(TypeScript).

**Architecture:**
- One pure classifier per language: `map_check/rings.py` and `web/src/mapArena/rings.ts`. Each implements the
  spec's rule exactly.
- `build_boxes` and `boxesFor` group cells by (kind, owner, height) and merge rects per group.
- A hand-authored golden fixture of small grids, with their expected per-cell heights, is the single truth that
  pytest and vitest both check.

**Tech Stack:** Python 3 + numpy/scipy (existing tool env); TypeScript + vitest (existing web app).

**Spec:** docs/superpowers/specs/2026-10-20-shape-outline-height-design.md. Read "The rule, exactly", the case
table, and "Decisions on the details". The prototype `2026-10-20-shape-outline-height/outline-proto.py` beside
it is the reference implementation of the strip test.

## Global Constraints

- Every rule in `.superpowers/sdd/2026-10-19-map-check/context.md` still applies:
  - test commands with the env prefix;
  - commit only listed files;
  - never write to the real maps dir or artifacts;
  - no new dependencies;
  - Rulings 1–10.
- Wall height is `settings.wall_height` / `EditState.wallHeight`.
- Unfilled shapes and rooms keep their rings as walls.
- The web grid encoding and the HTTP API don't change.
- Box group order:
  1. walls at the wall height;
  2. raised strip walls, by ascending height;
  3. filled shapes by id (interior plus owned free edges).

  Within each group, order by `merge_rects` (row0, col0, nrows, ncols).
- Ownership of a free-edge cell goes to the tallest filled shape whose ring holds it; on a tie, the lowest id
  wins. A strip cell's height is max(wall height, the tallest owner's height).

## Review Focus

1. **Unticking a shape's fill.** Its ring must return to plain wall at wall height (case H). Test:
   `test_unfill_restores_wall` / vitest `unfill restores wall`.
2. **Erasing a wall cell next to a strip.** The neighbouring strip cells may become free edges, and erased cells
   produce no box (case J). The fixture case `J_erase_beside_strip` covers this.
3. **A low shape in a wall line (case C).** No cell of the wall line may drop below the wall height. Test
   `test_wall_line_never_notched`: on every fixture case, each cell of the original wall that is not a free edge
   keeps a height ≥ the wall height.
4. **Editing heights must not re-run the strip classification** in the browser. The classification depends only
   on the filled set and the erased cells. Vitest `classification cached across height edits` asserts that the
   same object is returned.
5. **Already-saved arenas without `outline_rule`** must still load and render. Test
   `test_load_current_without_outline_rule` reads an old-style manifest.

---

### Task 1: Python classifier, `build_boxes`, golden fixture, manifest marker

**Files:**
- Create: `tools/map_check/map_check/rings.py`
- Create: `tools/map_check/tests/test_rings.py`
- Create: `tools/map_check/tests/fixtures/outline_golden.json` (hand-authored, never generated)
- Modify:
  - `tools/map_check/map_check/arena_build.py` (`build_boxes`)
  - `tools/map_check/map_check/arena_store.py` (manifest `settings.outline_rule`)
  - `tools/map_check/tests/test_arena_build.py` (`test_build_boxes_partition` now expects rings on shapes)
- Create: `tools/map_check/scripts/outline_ab.py`, the A/B sanity script, kept in the repo.

**Interfaces:**
- Produces:
  - `classify_rings(occ: np.ndarray[bool], labels: np.ndarray[int], heights: dict[int, float]) -> Rings`.
    - `occ` is already erased.
    - `heights` maps **filled** shape ids to their heights.
  - `Rings`, a frozen dataclass:
    - `owner: np.ndarray[int32]`: 0 = not in any filled ring, else the owning shape id;
    - `strip: np.ndarray[bool]`.
  - `build_boxes(m, labels, settings)`: same signature as today, new grouping and order (see Global Constraints).
  - Manifest `settings["outline_rule"] = "shape-height-v1"`.
- **Fixture format** (`outline_golden.json`), consumed by Task 2. A list of cases:
  `{"name": str, "cells": [[int]], "wall_height": float, "shapes": {"<id>": {"fill": bool, "height": float}}, "erased": [int], "heights": [[float|null]], "kinds": [[str]]}`
  - `cells` uses web-grid values: 0 free, 255 occupied, k interior.
  - `heights` gives each cell's built height, or null for no box.
  - `kinds` holds `"wall"`, `"shape:<id>"` or `""`.

- [ ] **Step 1: Write the fixture by hand from the spec's rule.**
  - Cases: `A_free_standing`, `B_free_standing_tall`, `C_wall_1cell`, `D_wall_1cell_tall`, `E_wall_2cell`,
    `F_corner`, `G_two_tables`, `H_unfilled`, `I_room_unfilled`, `J_erase_beside_strip`.
  - Grids are at most 16×10. A–F use the prototype's layouts, whose printed classifications are in the spec
    folder.
  - Each case's heights and kinds are derived by hand. Do not run your implementation to produce them.
- [ ] **Step 2: Write failing tests** in `test_rings.py`:
  - `test_golden_cases[name]`, parametrized: build a `OccupancyMap` with resolution 1.0 and origin (0, 0) from
    `cells`, run `build_boxes`, rasterise the boxes back to per-cell heights and kinds, and assert they equal
    `heights` and `kinds`.
  - `test_wall_line_never_notched` (Review Focus 3).
  - `test_unfill_restores_wall` (Review Focus 1).
  - `test_measured_map_defaults_unchanged` (case K, using the `real_root` fixture; skip if absent). The per-cell
    height map equals the one from the current code's rule: every occupied or interior cell is at 0.60, with all
    16 shapes filled.
  - `test_load_current_without_outline_rule` (Review Focus 5).
  - Update `test_build_boxes_partition`:
    - `kind=="wall"` boxes cover exactly `occ - free_edges`;
    - shape k's boxes cover its interior plus the free edges it owns.
  - Update `test_erased_excluded` the same way: walls cover `occ - erased - free_edges`.
  - No other existing assertion may be loosened.
- [ ] **Step 3: Run** `timeout -k 10 300 env -u PYTHONPATH -u AMENT_PREFIX_PATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --project tools/map_check pytest tools/map_check/tests/test_rings.py -q`. Expected: FAIL (no module `map_check.rings`).
- [ ] **Step 4: Implement** `rings.py`, the new `build_boxes` and the manifest field.
  - The strip test must equal the spec's rule cell for cell. Vectorising is allowed. The prototype loops per cell
    and takes about 1 s on the largest map; keep a typical save well under 1 s.
- [ ] **Step 5: Run** the full Python suite. Expected: all pass, with the golden-rect test unchanged.
- [ ] **Step 6: A/B sanity run** with `scripts/outline_ab.py`. Over every `*.yaml` map in the Map Studio maps
  folder (read-only), with default settings, it:
  - asserts that cells outside filled rings have identical heights under the old rule (all occupied cells at the
    wall height) and the new one;
  - prints, per map, the free-edge and strip counts and the classify time.

  Expected: no assertion failures, and the measured map's counts equal the spec's (strip cells 16, 40, 20, 16 and
  9 on shapes 5, 6, 7, 8 and 14; zero on the rest). Paste the output into the report.
- [ ] **Step 7: Commit:** `feat(map-check): filled shapes' outlines take the shape height`.

### Task 2: TypeScript classifier, `boxesFor`, 3D preview, 2D colours

**Files:**
- Create: `tools/map_check/web/src/mapArena/rings.ts`
- Create: `tools/map_check/web/src/mapArena/rings.test.ts`
- Modify:
  - `editState.ts` (`Boxes`, `boxesFor`, cached classification)
  - `Preview3D.tsx` (per-group heights)
  - `MapCanvas2D.tsx` (colours, the selected-shape highlight including its free edges, and the hover readout
    saying "table edge" / "wall")

**Interfaces:**
- Consumes: `outline_golden.json` (Task 1's format) and `mergeRects` (existing).
- Produces:
  - `classifyRings(g: Grid, erased: ReadonlySet<number>, filled: ReadonlySet<number>): { inRing: Uint8Array; strip: Uint8Array }`. Heights are left out, so the result can be cached.
  - `owners(g: Grid, rings, heights: Record<number, number>): Int32Array`, cheap and recomputed on height edits.
  - `Boxes` becomes `{ walls: Rect[]; raised: { h: number; rects: Rect[] }[]; shapes: Record<number, Rect[]> }`.
    Raised groups are sorted by ascending `h`.
  - `ringsFor(s: EditState, g: Grid)`: memoised on the identity of `(g, s.erased)` and the set of filled ids.

- [ ] **Step 1: Write failing vitests** in `rings.test.ts`:
  - `golden cases match python`: for each case, `boxesFor` rasterised back to per-cell heights and kinds equals
    the fixture.
  - `unfill restores wall`.
  - `classification cached across height edits` (Review Focus 4).
  - The existing `editState.test.ts` golden tests treat each mask as one outline with no occupied cells, so they
    keep passing unchanged. If one fails, the new `boxesFor` is wrong; don't edit the test.
- [ ] **Step 2: Run** `timeout -k 10 600 npm --prefix tools/map_check/web test`. Expected: FAIL (no `./rings`).
- [ ] **Step 3: Implement** `rings.ts` and the new `boxesFor`. Then update:
  - **Preview3D:** instanced walls at `wallHeight`, each raised group at its `h`, and shapes as today.
  - **MapCanvas2D:**
    - free-edge cells take their owner's shape colour, or the selected colour when the owner is selected;
    - strip cells take the wall colour;
    - the hover kind reads "shape N edge" for free edges.
  - **Stats line:** count raised groups.
- [ ] **Step 4: Run** the vitest and the build. Expected: all pass, with no type errors.
- [ ] **Step 5: Commit:** `feat(map-check): preview and 2D map show shape outlines at the shape height`.

The controller's acceptance check comes after both tasks:
- a browser check of case L (shape 4 at 1.50 with no 0.60 rim) and a low table against a wall (e.g. shape 6 at
  0.45: the back strip stays at 0.60, the free edges drop);
- one live render if the user allows the GPU.
