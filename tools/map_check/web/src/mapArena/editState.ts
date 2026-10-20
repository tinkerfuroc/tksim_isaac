// Edit state for the Map → arena screen: pure functions, no React, no DOM.
// The saved arena is built from the same settings on the server
// (arena_build.build_boxes), so `boxesFor` must give the rectangles it gives.
import type { MapBody, Outline, SaveRequest, ShapeSetting, WebGrid } from '../api';
import { mergeRects, type Rect } from '../rects';
import { classifyRings, owners, type Rings } from './rings';

/** Cell codes of the web grid: 0 free, 1..254 the closed outline with that id, 255 occupied. */
export const FREE = 0;
export const WALL = 255;

/** The decoded web grid. Row 0 is the top of the map (max y), as in arena_build.web_grid. */
export interface Grid {
  W: number;
  H: number;
  res: number;
  ox: number;
  oy: number;
  cells: Uint8Array; // row-major, H rows of W cells
}

/** The settings that get saved; edits are "unsaved" while they differ from the baseline. */
interface Baseline {
  wallHeight: number;
  shapes: Record<number, ShapeSetting>;
  erased: Set<number>;
}

export interface EditState {
  outlines: Outline[];
  wallHeight: number;
  defaultShapeHeight: number;
  shapes: Record<number, ShapeSetting>;
  erased: Set<number>; // flat indices r*W+c of occupied cells left out of the arena
  strokes: number[][]; // erased cells per brush drag, oldest first
  selected: number | null;
  baseline: Baseline; // as last loaded or saved
}

/**
 * The arena's boxes, in the saved arena's group order: walls at the wall height, raised wall
 * strips by ascending height, then each filled shape by id (its interior plus the free edges of
 * its outline that it owns).
 */
export interface Boxes {
  walls: Rect[]; // at wallHeight
  raised: { h: number; rects: Rect[] }[]; // wall strips behind shapes taller than the walls
  shapes: Record<number, Rect[]>; // filled shapes only, at their own height
}

const FALLBACK_SHAPE_HEIGHT = 0.6;

export function decodeGrid(web: WebGrid): Grid {
  const raw = atob(web.grid);
  if (raw.length !== web.W * web.H) {
    throw new Error(`map grid has ${raw.length} cells, expected ${web.W} × ${web.H}`);
  }
  const cells = new Uint8Array(raw.length);
  for (let i = 0; i < raw.length; i++) cells[i] = raw.charCodeAt(i);
  return { W: web.W, H: web.H, res: web.res, ox: web.ox, oy: web.oy, cells };
}

/** Map metres of a cell centre. */
export function cellCenter(g: Grid, row: number, col: number): [number, number] {
  return [g.ox + (col + 0.5) * g.res, g.oy + (g.H - row - 0.5) * g.res];
}

/**
 * The starting edit for a map body. The grid carries no edit state; it is taken so every
 * edit function sees the same (state, grid) pair.
 */
export function initEdit(_grid: Grid, outlines: Outline[], defaults: MapBody['defaults']): EditState {
  const shapes: Record<number, ShapeSetting> = {};
  for (const o of outlines) {
    const d = defaults.shapes[String(o.id)];
    shapes[o.id] = { fill: d ? d.fill : false, height: d ? d.height : FALLBACK_SHAPE_HEIGHT };
  }
  const first = outlines.length ? shapes[outlines[0].id].height : FALLBACK_SHAPE_HEIGHT;
  const erased = new Set<number>();
  return {
    outlines,
    wallHeight: defaults.wall_height,
    defaultShapeHeight: first,
    shapes,
    erased,
    strokes: [],
    selected: null,
    baseline: { wallHeight: defaults.wall_height, shapes, erased },
  };
}

/**
 * True while the settings that would be saved (wall height, each shape's fill and height,
 * erased cells) differ from what was last loaded or saved. Selection is not an edit.
 */
export function isDirty(s: EditState): boolean {
  const b = s.baseline;
  if (s.wallHeight !== b.wallHeight) return true;
  if (s.shapes !== b.shapes) {
    for (const o of s.outlines) {
      const x = s.shapes[o.id];
      const y = b.shapes[o.id];
      if (x.fill !== y.fill || x.height !== y.height) return true;
    }
  }
  if (s.erased !== b.erased) {
    if (s.erased.size !== b.erased.size) return true;
    for (const i of s.erased) if (!b.erased.has(i)) return true;
  }
  return false;
}

/** After a save: the settings of `sent` (the state that was saved; default `s`) become the baseline. */
export function markSaved(s: EditState, sent: EditState = s): EditState {
  return { ...s, baseline: { wallHeight: sent.wallHeight, shapes: sent.shapes, erased: sent.erased } };
}

/**
 * [row0, col0, row1, col1) around every non-free cell, grown by `margin` cells and clipped to
 * the map; the whole map when nothing is occupied. The 2D view shows this part of the map.
 */
export function contentBounds(g: Grid, margin: number): [number, number, number, number] {
  let r0 = g.H, c0 = g.W, r1 = 0, c1 = 0;
  for (let r = 0; r < g.H; r++) {
    for (let c = 0; c < g.W; c++) {
      if (g.cells[r * g.W + c] === FREE) continue;
      if (r < r0) r0 = r;
      if (r >= r1) r1 = r + 1;
      if (c < c0) c0 = c;
      if (c >= c1) c1 = c + 1;
    }
  }
  if (r1 === 0) return [0, 0, g.H, g.W];
  return [Math.max(0, r0 - margin), Math.max(0, c0 - margin), Math.min(g.H, r1 + margin), Math.min(g.W, c1 + margin)];
}

function mapShapes(s: EditState, f: (id: number, v: ShapeSetting) => ShapeSetting): EditState {
  const shapes: Record<number, ShapeSetting> = {};
  for (const o of s.outlines) shapes[o.id] = f(o.id, s.shapes[o.id]);
  return { ...s, shapes };
}

/** "Fill closed outlines up to N m²": fill exactly the outlines whose area is at most maxArea. */
export function applyFillRule(s: EditState, maxArea: number): EditState {
  const area = new Map(s.outlines.map((o) => [o.id, o.area_m2]));
  return mapShapes(s, (id, v) => ({ ...v, fill: (area.get(id) ?? Infinity) <= maxArea }));
}

/** Sets the default shape height and gives it to every outline. */
export function setDefaultShapeHeight(s: EditState, h: number): EditState {
  return { ...mapShapes(s, (_id, v) => ({ ...v, height: h })), defaultShapeHeight: h };
}

export function toggleShape(s: EditState, id: number): EditState {
  if (!s.shapes[id]) return s;
  return { ...s, shapes: { ...s.shapes, [id]: { ...s.shapes[id], fill: !s.shapes[id].fill } } };
}

export function setShapeHeight(s: EditState, id: number, h: number): EditState {
  if (!s.shapes[id]) return s;
  return { ...s, shapes: { ...s.shapes, [id]: { ...s.shapes[id], height: h } } };
}

export function setWallHeight(s: EditState, h: number): EditState {
  return { ...s, wallHeight: h };
}

export function selectShape(s: EditState, id: number | null): EditState {
  return s.selected === id ? s : { ...s, selected: id };
}

/** The cells a square brush of `size` cells covers when centred on (row, col), clipped to the map. */
function brushCells(g: Grid, row: number, col: number, size: number, out: number[]): void {
  const n = Math.max(1, Math.round(size));
  const lo = Math.floor((n - 1) / 2);
  for (let r = row - lo; r < row - lo + n; r++) {
    if (r < 0 || r >= g.H) continue;
    for (let c = col - lo; c < col - lo + n; c++) {
      if (c >= 0 && c < g.W) out.push(r * g.W + c);
    }
  }
}

/**
 * Brush along the straight line from (r0, c0) to (r1, c1), so a fast drag leaves no gaps.
 * Only occupied cells not yet erased are touched. With `continueStroke` the cells join the
 * last stroke (the same drag); otherwise they start a new one. Returns `s` itself when
 * nothing changed, so no empty stroke is ever recorded.
 */
export function eraseLine(
  s: EditState, g: Grid, r0: number, c0: number, r1: number, c1: number,
  size: number, continueStroke = false,
): EditState {
  const covered: number[] = [];
  const steps = Math.max(Math.abs(r1 - r0), Math.abs(c1 - c0));
  for (let k = 0; k <= steps; k++) {
    const t = steps ? k / steps : 0;
    brushCells(g, Math.round(r0 + (r1 - r0) * t), Math.round(c0 + (c1 - c0) * t), size, covered);
  }
  const added: number[] = [];
  const seen = new Set<number>();
  for (const i of covered) {
    if (g.cells[i] === WALL && !s.erased.has(i) && !seen.has(i)) {
      seen.add(i);
      added.push(i);
    }
  }
  if (!added.length) return s;
  const erased = new Set(s.erased);
  for (const i of added) erased.add(i);
  const strokes = continueStroke && s.strokes.length
    ? [...s.strokes.slice(0, -1), [...s.strokes[s.strokes.length - 1], ...added]]
    : [...s.strokes, added];
  return { ...s, erased, strokes };
}

/** One brush dab at (row, col); see eraseLine. */
export function eraseAt(
  s: EditState, g: Grid, row: number, col: number, size: number, continueStroke = false,
): EditState {
  return eraseLine(s, g, row, col, row, col, size, continueStroke);
}

/** Restores the cells of the last stroke. */
export function undoStroke(s: EditState): EditState {
  if (!s.strokes.length) return s;
  const erased = new Set(s.erased);
  for (const i of s.strokes[s.strokes.length - 1]) erased.delete(i);
  return { ...s, erased, strokes: s.strokes.slice(0, -1) };
}

// Bounding box [r0, c0, r1, c1) of every outline id in a grid, computed once per grid.
const boundsCache = new WeakMap<Grid, Map<number, [number, number, number, number]>>();

function shapeBounds(g: Grid) {
  let b = boundsCache.get(g);
  if (b) return b;
  b = new Map();
  for (let r = 0; r < g.H; r++) {
    for (let c = 0; c < g.W; c++) {
      const v = g.cells[r * g.W + c];
      if (v === FREE || v === WALL) continue;
      const e = b.get(v);
      if (!e) b.set(v, [r, c, r + 1, c + 1]);
      else {
        e[0] = Math.min(e[0], r); e[1] = Math.min(e[1], c);
        e[2] = Math.max(e[2], r + 1); e[3] = Math.max(e[3], c + 1);
      }
    }
  }
  boundsCache.set(g, b);
  return b;
}

/** Filled shape id -> height. */
function filledHeights(s: EditState): Record<number, number> {
  const out: Record<number, number> = {};
  for (const o of s.outlines) {
    const v = s.shapes[o.id];
    if (v?.fill) out[o.id] = v.height;
  }
  return out;
}

// The last ring classification per grid, with the erased set and the filled ids it was made for.
const ringsCache = new WeakMap<Grid, { erased: Set<number>; filled: string; rings: Rings }>();

/**
 * The ring classification of the edit's filled shapes. It depends only on the grid, the erased
 * cells (by identity: every erase or undo makes a new set) and which shapes are filled, so
 * height edits return the same object without reclassifying.
 */
export function ringsFor(s: EditState, g: Grid): Rings {
  const ids = Object.keys(filledHeights(s)).map(Number).sort((a, b) => a - b);
  const filled = ids.join(',');
  const hit = ringsCache.get(g);
  if (hit && hit.erased === s.erased && hit.filled === filled) return hit.rings;
  const rings = classifyRings(g, s.erased, new Set(ids));
  ringsCache.set(g, { erased: s.erased, filled, rings });
  return rings;
}

/**
 * Per cell, the filled shape whose free edge it is (built at that shape's height), 0 for every
 * other cell. Ring cells that continue a wall line (the strip) stay wall and read 0.
 */
export function freeEdgeOwners(s: EditState, g: Grid): Int32Array {
  const rings = ringsFor(s, g);
  const own = owners(g, rings, filledHeights(s));
  for (let i = 0; i < own.length; i++) if (rings.strip[i]) own[i] = 0;
  return own;
}

/** mergeRects over the cells of [r0, r1) x [c0, c1) that `keep`, in map coordinates. */
function mergeWhere(g: Grid, r0: number, c0: number, r1: number, c1: number, keep: (i: number) => boolean): Rect[] {
  // The merge only looks at runs, so merging the crop and shifting back equals a full-map merge.
  const w = c1 - c0;
  const h = r1 - r0;
  if (w <= 0 || h <= 0) return [];
  const crop = new Uint8Array(w * h);
  for (let r = 0; r < h; r++) {
    for (let c = 0; c < w; c++) crop[r * w + c] = keep((r0 + r) * g.W + c0 + c) ? 1 : 0;
  }
  return mergeRects(crop, w, h).map(([a, b2, nr, nc]) => [a + r0, b2 + c0, nr, nc] as Rect);
}

/**
 * The arena's boxes as grid rectangles [row0, col0, nrows, ncols], built exactly as the server's
 * arena_build.build_boxes (rule: rings.ts): occupied cells that are not erased are walls, except
 * that a filled shape's outline takes the shape's height. Its free edges join the shape; where
 * the outline is part of a wall line (the strip) it stays wall at max(wall height, shape height).
 * Same groups, merge and order as the server.
 */
export function boxesFor(s: EditState, g: Grid): Boxes {
  const n = g.W * g.H;
  const rings = ringsFor(s, g);
  const heights = filledHeights(s);
  const own = owners(g, rings, heights);

  const wallMask = new Uint8Array(n);
  for (let i = 0; i < n; i++) wallMask[i] = g.cells[i] === WALL ? 1 : 0;
  for (const i of s.erased) wallMask[i] = 0;
  const raisedH = new Float64Array(n); // a raised strip cell's height, 0 elsewhere
  const raisedBox = new Map<number, [number, number, number, number]>(); // height -> [r0, c0, r1, c1)
  for (let i = 0; i < n; i++) {
    if (!rings.inRing[i]) continue;
    if (!rings.strip[i]) { wallMask[i] = 0; continue; } // free edge: built with its shape
    const h = heights[own[i]];
    if (!(h > s.wallHeight)) continue; // strip at the wall height: part of the walls
    wallMask[i] = 0;
    raisedH[i] = h;
    const r = Math.floor(i / g.W), c = i - r * g.W;
    const bb = raisedBox.get(h);
    if (!bb) raisedBox.set(h, [r, c, r + 1, c + 1]);
    else {
      bb[0] = Math.min(bb[0], r); bb[1] = Math.min(bb[1], c);
      bb[2] = Math.max(bb[2], r + 1); bb[3] = Math.max(bb[3], c + 1);
    }
  }
  const walls = mergeRects(wallMask, g.W, g.H);
  const raised = [...raisedBox.keys()].sort((a, b) => a - b).map((h) => {
    const [r0, c0, r1, c1] = raisedBox.get(h)!;
    return { h, rects: mergeWhere(g, r0, c0, r1, c1, (i) => raisedH[i] === h) };
  });

  const shapes: Record<number, Rect[]> = {};
  const bounds = shapeBounds(g);
  for (const o of s.outlines) {
    if (!s.shapes[o.id]?.fill) continue;
    const bb = bounds.get(o.id);
    if (!bb) { shapes[o.id] = []; continue; }
    // The interior's box grown by one cell holds its whole outline ring.
    const id = o.id;
    shapes[id] = mergeWhere(g, Math.max(0, bb[0] - 1), Math.max(0, bb[1] - 1),
      Math.min(g.H, bb[2] + 1), Math.min(g.W, bb[3] + 1),
      (i) => g.cells[i] === id || (own[i] === id && !rings.strip[i]));
  }
  return { walls, raised, shapes };
}

/** The POST /api/map/save body. */
export function savePayload(s: EditState, token: string, name: string): SaveRequest {
  const shapes: Record<string, ShapeSetting> = {};
  for (const o of s.outlines) {
    const v = s.shapes[o.id];
    shapes[String(o.id)] = { fill: v.fill, height: v.height };
  }
  return {
    token,
    name,
    wall_height: s.wallHeight,
    shapes,
    erased: [...s.erased].sort((a, b) => a - b),
  };
}
