// Edit state for the Map → arena screen: pure functions, no React, no DOM.
// The saved arena is built from the same settings on the server
// (arena_build.build_boxes), so `boxesFor` must give the rectangles it gives.
import type { MapBody, Outline, SaveRequest, ShapeSetting, WebGrid } from '../api';
import { mergeRects, type Rect } from '../rects';

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

export interface EditState {
  outlines: Outline[];
  wallHeight: number;
  defaultShapeHeight: number;
  shapes: Record<number, ShapeSetting>;
  erased: Set<number>; // flat indices r*W+c of occupied cells left out of the arena
  strokes: number[][]; // erased cells per brush drag, oldest first
  selected: number | null;
}

export interface Boxes {
  walls: Rect[];
  shapes: Record<number, Rect[]>; // filled shapes only, as in the saved arena
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
  return {
    outlines,
    wallHeight: defaults.wall_height,
    defaultShapeHeight: first,
    shapes,
    erased: new Set(),
    strokes: [],
    selected: null,
  };
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

/**
 * The arena's boxes as grid rectangles [row0, col0, nrows, ncols]: occupied cells that are
 * not erased as walls, then each filled outline. Same merge and order as the server.
 */
export function boxesFor(s: EditState, g: Grid): Boxes {
  const wallMask = new Uint8Array(g.W * g.H);
  for (let i = 0; i < wallMask.length; i++) wallMask[i] = g.cells[i] === WALL ? 1 : 0;
  for (const i of s.erased) wallMask[i] = 0;
  const walls = mergeRects(wallMask, g.W, g.H);

  const shapes: Record<number, Rect[]> = {};
  const bounds = shapeBounds(g);
  for (const o of s.outlines) {
    if (!s.shapes[o.id]?.fill) continue;
    const bb = bounds.get(o.id);
    if (!bb) { shapes[o.id] = []; continue; }
    // Merge inside the outline's bounding box and shift back: the merge only looks at
    // runs, so the rectangles (and their sorted order) equal a full-map merge.
    const [r0, c0, r1, c1] = bb;
    const w = c1 - c0;
    const h = r1 - r0;
    const mask = new Uint8Array(w * h);
    for (let r = 0; r < h; r++) {
      for (let c = 0; c < w; c++) mask[r * w + c] = g.cells[(r0 + r) * g.W + c0 + c] === o.id ? 1 : 0;
    }
    shapes[o.id] = mergeRects(mask, w, h).map(([a, b, n, m]) => [a + r0, b + c0, n, m] as Rect);
  }
  return { walls, shapes };
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
