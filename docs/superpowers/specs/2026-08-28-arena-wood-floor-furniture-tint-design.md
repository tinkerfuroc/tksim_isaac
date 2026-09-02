# Arena wood floor + tinted furniture (rcw2026)

**Date:** 2026-08-28
**Status:** approved design, ready for implementation plan

## Problem

The rendered rcw2026 arena reads as visually flat and "stark-white":

- The **floor** is Isaac's default `GroundPlaneCfg()` gray grid
  (`simulation/tinker_sim_isaac/backend.py`). It is *not* part of
  `arena.usd`.
- The **furniture** (20 GLB-converted models referenced by `arena.usd`)
  renders white/bland. The deployed artifact *does* ship per-model
  `textures/<id>/*.png`, so the most likely cause is that the GLB's own
  material/texture bindings do not resolve at render time. Rather than
  chase texture resolution, we override each model with a deterministic
  solid PBR color.
- The **walls** are a mid-gray (`0.6`) `UsdPreviewSurface`. Left
  unchanged by explicit decision (see Non-goals).

## Decisions (from brainstorming)

- Floor is authored as a **prim inside `arena.usd`** (not a backend
  ground-material swap), so it travels with the arena artifact.
- **Solid tinted PBR only** — no image textures — keeping render cost
  ≈ current. Speed is prioritized over texture fidelity.
- Realism is the goal *within* that speed budget.
- Walls stay as-is.

## Non-goals

- No change to walls (currently gray `0.6`). If they should be
  brightened to true white later, that is a one-line follow-up.
- No image-based / MDL textures, no UV work.
- No change to physics, collision, navigation, or the occupancy map.
  The floor slab is **visual-only**; physics still rides the existing
  `GroundPlaneCfg` plane.
- No re-conversion of the GLB furniture geometry — only an added
  material *override binding* at compose time.

## Design

All changes are in `tools/tinker_sim_deploy/arena_convert.py`, which is
"live-only" (every `pxr`/Kit call is lazily imported; `pxr` and
`isaacsim` are not importable in the plain-Python test/dev env). The
pure geometry/color logic is factored into standalone functions so it
is unit-testable without `pxr`, matching the existing split that
`tests/test_arena_convert.py` already documents.

### 1. Floor slab

New pure helper:

```
def floor_slab(layout: ArenaLayout, *, margin: float = 0.10,
               thickness: float = 0.02, lift: float = 0.002
               ) -> tuple[tuple[float,float,float], tuple[float,float,float]]:
    """Return (center_xyz, size_xyz) for a thin visual floor slab that
    covers the arena footprint.

    Footprint = axis-aligned bounding box of all wall boxes' XY extents,
    expanded by `margin` on each side. The slab is `thickness` tall; its
    top face sits at z = `lift` (a hair above the physics ground plane at
    z=0) so it wins the depth test against Isaac's default ground grid,
    so center z = lift - thickness/2.
    """
```

- Footprint from `layout.walls`: for each `WallBox`, its XY half-extent
  is `size.xy / 2` about `center.xy` (wall yaw is 0 in rcw2026; if a wall
  has non-zero yaw the axis-aligned half-extent is a conservative
  over-approximation — acceptable for a floor that only needs to *cover*
  the interior).
- `compose_arena` authors `/World/Arena/Floor` as a `UsdGeom.Cube`
  (size attr 1.0, scaled to the slab size, translated to center),
  binds the oak material, and marks it **visual-only** (no
  `UsdPhysics.CollisionAPI`).

### 2. Furniture material override

New color table + pure lookup:

```
FURNITURE_COLORS: dict[str, tuple[tuple[float,float,float], float]]
    # model_id -> (linear rgb 0..1, roughness)
_FURNITURE_FALLBACK = ((0.55, 0.50, 0.45), 0.8)  # warm neutral, never white

def furniture_material(model_id: str) -> tuple[tuple[float,float,float], float]:
    return FURNITURE_COLORS.get(model_id, _FURNITURE_FALLBACK)
```

Category assignments (roughness 0.8 unless noted):

| Category | Models | RGB |
|---|---|---|
| Wood (medium) | kitchen_table, side_table, laundry_desk, stand | ~0.55, 0.38, 0.22 |
| Wood (light) | shelf, door | ~0.63, 0.46, 0.28 |
| Wood (dark) | tv_stand, bed | ~0.46, 0.31, 0.19 |
| Fabric | sofa (0.35,0.40,0.48), cushion (0.70,0.55,0.45), chair (0.30,0.32,0.35) | — |
| Appliance (off-white steel, rough 0.5) | refrigerator, washing_machine, dishwasher_close, sink | ~0.82–0.88 gray |
| Plastic/wicker | trashbin (0.25,0.28,0.32), laundry_basket (0.80,0.75,0.60) | — |
| Plant | plant_mid, plant_tall | ~0.19, 0.44, 0.19 |
| Electronics | tv | 0.05, 0.05, 0.06 |

(Exact values fixed in code; the table above is indicative.) Any
model_id not listed falls back to the warm neutral — so a newly
allowlisted model can never regress to stark-white.

In `compose_arena`, for each furniture wrapper prim, define a unique
`UsdShade.Material` under `/World/Arena/Materials/furn_<model_id>` and
bind it on the **wrapper** prim with binding strength
`UsdShade.Tokens.strongerThanDescendants`, so it overrides the
referenced GLB subtree's own material bindings.

### 3. Shared material authoring

Refactor `_bind_gray_material` into a small internal helper that
authors a `UsdPreviewSurface` material for an arbitrary
`(name, rgb, roughness)` and binds it at a given strength, then express
the existing wall gray, the new floor oak, and the furniture tints
through it. Keeps one code path for PBR authoring.

## Determinism

The artifact is content-addressed (sha256 of the payload). A fixed
color table + fixed slab geometry means `arena.usd` bytes are a pure
function of the world XML + furniture set, so regeneration yields a
stable new identity hash and `publish_asset_artifact` re-points
`artifacts/arena/rcw2026/current.json`.

## Testing

Pure-Python unit tests in `tests/test_arena_convert.py` (no `pxr`):

- `floor_slab`: correct AABB from a set of `WallBox`es; margin applied;
  top face at `lift`, thickness honored; degenerate (single wall) and
  multi-wall cases.
- `furniture_material`: every model in the rcw2026 allowlist resolves to
  a non-white color; unlisted id hits the fallback; roughness in range.

## Operator handoff (Isaac box — not runnable in dev/CI env)

Regeneration and visual/vision validation require Kit (`pxr` +
`omni.kit.asset_converter`) and the pinned SOBITS checkout, so they run
on the Isaac box, not in this session:

1. Regenerate + re-pin the artifact:
   `python tools/arena_import.py --config <rcw2026 config>` (optionally
   `--checkout <existing pinned checkout>` to skip the clone).
   This publishes a new content-addressed arena artifact and updates
   `current.json`; then register the new `arena.usd` path + sha256 under
   `generated_arena_usds` in `artifacts/asset-manifest.json` (the tool
   prints this reminder).
2. Eyeball the render: wood floor present, furniture tinted, walls
   unchanged.
3. Run the vision/detection smoke to confirm detection parity and that
   per-frame render time stayed close to the pre-change baseline
   (solid PBR should not move it).

Steps are documented for the operator in `docs/developer-log.md`.

## Risks / mitigations

- **z-fighting** floor vs. Isaac ground grid → floor top lifted `lift`
  (2 mm) above z=0; slab covers the interior so the gray grid is only
  visible outside the walls.
- **Override strength wrong** (tint doesn't win over GLB) → use
  `strongerThanDescendants`; the operator render check catches it.
- **Detection regression** from appearance change → vision smoke in the
  handoff; realism kept conservative (matte solids, appliances stay
  light).
