# Shape outlines follow the shape's height — design

Status: approved by the user on 2026-10-20 ("taller of the two" for shared walls). This spec is a follow-up to
`2026-10-17-map-check-design.md`, and that spec's constraints still apply.

## Why

On Map → arena, filling a closed outline (a table, a shelf, a chair) raises only the cells *inside* the outline
to the shape's height. The outline itself is a ring of occupied cells, and today it is built as wall at the wall
height (0.60 m by default). So:

- a 0.45 m table renders as a 0.45 m block sitting inside a 0.60 m rim;
- a 1.50 m shelf renders as a tall block standing inside a 0.60 m fence.

Teammates check their poses against these renders and head-cam views. A rim that doesn't exist in the real room
makes a pose look blocked, or makes the head cam see an edge that isn't there. The overlap flags, which are
computed from the same boxes, then name the wrong thing ("wall" instead of "shape 7") at the wrong height.

## What we found in the real map

In `robocup_home_2026_measured_5cm` (370 × 290 cells, 5 cm):

- every one of the 16 closed outlines is a ring exactly **one cell thick** around its interior;
- **10 of the 16** shapes are pushed against a wall, so part of their ring is that wall line, connected to the
  main wall structure;
- the 6 free-standing shapes (3, 4, 9, 10, 11, 13) have rings that touch nothing else.

## Goal

For every **filled** shape:

1. **Free edges.** Ring cells that face the room take the **shape's height**, so the shape becomes one solid
   block of its height.
2. **Shared wall strip (the user's choice).** Where the shape is pushed against a wall, the ring cells that are
   part of that wall take **max(wall height, shape height)**:
   - a low table never notches the wall behind it;
   - a tall shelf raises the strip of wall behind it to the shelf's height.
3. **Junctions.** The cell where a free edge meets the wall line belongs to the strip, so the wall line stays
   unbroken at ≥ wall height. A low table therefore keeps a one-cell (5 cm) post at wall height at each end of
   its back edge. This follows from the rule below, and the cases encode it cell by cell.
4. **Two filled shapes sharing ring cells** take the taller of the two heights *(my call)*.

## The rule, exactly

This rule is prototyped and checked against the cases below and the real map; see "What the prototype showed".
Python and TypeScript implement exactly this.

Inputs:
- `occ`: the occupied mask **after erasing**. Erased cells count as free everywhere.
- `labels`: the interiors of the closed outlines.
- The set of **filled** shape ids.

1. **Ring of shape k:** the cells of `occ` that are 8-adjacent to an interior cell of k, corners included. A cell
   can be in several rings. Rings exist only for filled shapes, so the rings of unfilled shapes, including rooms,
   stay plain wall.
2. **R:** the union of all filled shapes' rings. **W** = `occ` minus R, which is the wall that remains.
3. **Strip test.** A cell c in R is a **strip** cell if, for at least one of the four directions (left, right,
   up, down):
   - walking from c in that direction through cells of R, the first cell outside R is a cell w of W (not free,
     not an interior, not off the map); and
   - at w, W's straight run along the walking direction is **strictly longer** than W's run across it. The run
     length is the number of consecutive W cells through w in that row or column.

   This means the shape's wall line continues into a real wall running the same way. A thick wall's cross-section
   or a lone speck doesn't qualify.
3b. **Corner extension (added 2026-10-20 after the final review; the user chose it).** Applied once, after
   step 3, with no chaining:
   - Look at each straight ring run that step 3 left partly or wholly non-strip.
   - Take an end cell e of that run that is a strip cell **by step 3**.
   - Look at the occupied straight line through e, perpendicular to the run. If that line does **not** continue
     on both sides of e, so the wall turns a corner at e (an L, not a T), every cell of the run becomes strip.
   - Why: a table in a room corner whose back wall ends at a doorway right after it would otherwise lower that
     stretch of room wall to table height.
   - Known cost, accepted by the user: a table edge next to a wall corner that turns can stay at wall height. For
     example, after erasing the wall beside a table (case J2), the table's side edge keeps wall height.
   - No change on the measured map. 556 cells across 36 noisier maps move from free edge to strip.
4. Every other cell of R is a **free edge**.
5. **Heights:**
   - a free-edge cell takes the max of the heights of the filled shapes whose rings contain it;
   - a strip cell takes max(wall height, that same shape max);
   - cells of W keep the wall height;
   - interiors keep their shape height, as today;
   - wall height means `settings.wall_height`.
6. **Box kinds and ids:**
   - free-edge cells join their shape's boxes, as `kind: "shape"` with the id of the tallest owning shape (the
     lowest id on a tie);
   - strip cells are `kind: "wall"`. A strip cell whose height equals the wall height merges with W.

## What the prototype showed

The prototype is `outline-proto.py` beside this spec in `2026-10-20-shape-outline-height/`. It printed
cases C, E, F and A as expected. On the measured map:
- the free-standing shapes 3, 4, 9, 10, 11 and 13 have all-free rings;
- shapes 5, 6, 7, 8 and 14 have their wall-line cells as strip (16, 40, 20, 16 and 9 cells);
- shapes 1, 2, 12, 15 and 16 **abut** the wall: their own ring is a separate row next to the wall row, so they
  share nothing. The table edge takes the table height, and the wall behind it stays at wall height;
- the slowest of the 42 real maps classifies in about 1 s with a per-cell Python loop, so a vectorized or TS
  version has room to spare.

## Non-goals and anti-goals

- **Unfilled shapes stay as today:** their rings remain walls at wall height *(my call)*. Rooms are closed
  outlines too, just above the fill limit (4 m² by default), and their walls must never move.
- **The 2D map is never changed.** The Map Studio map, Nav2 and the sim lidar's `map.yaml` are untouched; only
  the 3D boxes change.
- **Already-saved arena versions are not rebuilt.** The new rule applies from the next save.
- **Anti-goal:** any change that lowers a room wall anywhere, or leaves a gap in a wall line.

## Behaviour by case

These cases also serve as test fixtures. Each is a small synthetic grid, plus the real map where noted.

| Case | Expected |
|---|---|
| A. Free-standing table, 1-cell ring, height 0.45 | The whole ring and interior are at 0.45. There are no boxes at wall height anywhere in its footprint. |
| B. The same table at 1.50 | Ring and interior at 1.50. |
| C. Table against a 1-cell wall, height 0.45 | Interior and the three free edges at 0.45. The back strip (the wall line) stays at 0.60, continuous with the wall on both sides. |
| D. The same table at 1.50 | Interior and free edges at 1.50. The back strip rises to 1.50, and the wall beyond the table stays at 0.60. |
| E. Table against a 2-cell-thick wall, 0.45 | As C. The table's side edges are free edges and drop to 0.45 (they must not be mistaken for wall because the wall is thick). |
| F. Table in a room corner (two walls), 0.45 | Both back strips stay at 0.60. The two free edges drop to 0.45. |
| G. Two filled tables sharing an edge, 0.45 and 0.75 | The shared cells are at 0.75. |
| H. Same as C but the table is **unfilled** | No change from today: the ring is all wall at 0.60, and the interior is empty. |
| I. A room (closed outline above the fill limit, unfilled) | No change from today. |
| J. Erased ring cells | Erased cells produce no box at all, the same as erased wall cells today. |
| K. Measured map, defaults (all 16 filled at 0.60) | The same set of boxes in 3D as today, apart from box kinds and ids: the heights all equal 0.60 anyway. |
| N. Table in a room corner, back wall ends one cell past the table at a doorway, 0.30 | The whole back-wall row and the corner column stay at 0.60. Only the table's inner edges (the side facing the doorway and the edge facing the room) drop to 0.30. |
| N2. Like N, but the wall ends exactly at the table's far corner | The back-wall row and the corner column stay at 0.60. The far side edge stays a table edge (no chaining). |
| L. Measured map, shape 4 (free-standing) at 1.50 | No 0.60 rim remains around it in the preview or in the saved `arena.usd`. |

## Where it applies

The rule must give identical results in all four places:

1. **Python `build_boxes`** (`tools/map_check/map_check/arena_build.py`). This builds the saved `arena.usd` and
   `boxes.json`, and the overlap flags read those same boxes.
2. **TypeScript `boxesFor`** (`web/src/mapArena/editState.ts`). This drives the live 3D preview, which must equal
   what gets saved.
3. **The 2D canvas** (`MapCanvas2D.tsx`):
   - draws a filled shape's free-edge cells in the shape colour, and its shared strip in the wall colour;
   - the selected shape's ring highlights with its interior.
4. **Overlap flags** (`overlap.py`). These follow automatically from the boxes. A ring box of a filled shape is
   `kind: "shape"` with that shape's id, so flags read "… shape 7 …", not "wall". A shared strip box raised by a
   shape *(my call)* counts as `wall`.

Parity is enforced the same way `merge_rects` is today: a golden fixture generated by Python and checked by a
vitest. The fixture now covers the box sets for cases A–J (heights included), and the measured-map shape list for
case K when the real map is available (Ruling 1 `real_root`; skip when absent).

## Saved data

- The arena manifest's `settings` gains `"outline_rule": "shape-height-v1"` *(my call)*, so a reader can tell new
  versions from old ones. Older manifests simply lack the field.
- `boxes.json` keeps its format. Ring boxes appear as `kind: "shape"` with their `shape_id`; raised shared strips
  appear as `kind: "wall"` with their raised `z1`.

## Decisions on the details *(my calls)*

These settle the builder check's questions.

- **Rings are recomputed on both sides from the existing web grid.** The grid already carries interior ids and
  marks occupied cells as 255. The web grid encoding and the API don't change.
- **Erasing.** Classification runs on the mask after erasing, so it reflects what gets built:
  - an erased ring cell produces no box;
  - erasing the wall beside a shape can turn its strip into free edges;
  - a shape whose ring has erased cells keeps its filled interior.
- **Box grouping and order.** Rects are merged per group, in this order:
  1. walls at the wall height (W plus strip cells at the wall height);
  2. raised strip walls, by ascending height;
  3. then each filled shape by id, with its interior and free edges merged together.

  Each group is sorted by (row0, col0, nrows, ncols), as `merge_rects` does today. USD prim names may renumber;
  old arena versions are untouched, and nothing references prim names.
- **Case K compares per-cell height maps**, not rect lists. Ring and interior now merge into fewer, larger rects.
- **The 2D canvas uses the same classification function as `boxesFor`.** The result is cached in the edit state
  and recomputed when fill flags or erased cells change. Colours:
  - free edges in the shape colour;
  - strip cells in the wall colour;
  - unfilled rings in the wall colour, as today.
- **Overlap flags.** Raised strip cells report as "wall", and free edges report as their shape.
- **Real-map sanity run.** Run on every map in the Map Studio maps folder
  (`/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps`, read-only). It is an A/B against the
  current `build_boxes`, using default settings:
  - every cell outside the filled rings has the same height before and after;
  - report each map's free-edge and strip counts.

## Left to the builder

- Vectorising the strip test, as long as the results equal the rule cell for cell.
- The exact palette shades.

## Done when

- Cases A–J pass in both pytest and vitest from one golden fixture.
- Case K passes against the real map when present.
- The controller sees case L in the browser preview, and in one live render if the user allows the GPU.
- Both suites and the web build pass.
- No existing test was weakened.
