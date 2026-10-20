// Outline rings of filled shapes (spec 2026-10-20-shape-outline-height-design.md). The golden
// fixture is the one map_check's pytest checks build_boxes against, so the preview equals the save.
import { describe, expect, it } from 'vitest';
import golden from '../../../tests/fixtures/outline_golden.json';
import type { Outline } from '../api';
import type { Rect } from '../rects';
import {
  boxesFor, eraseAt, initEdit, ringsFor, setDefaultShapeHeight, setShapeHeight, setWallHeight, toggleShape,
  undoStroke, type Boxes, type EditState, type Grid,
} from './editState';
import { classifyRings, owners } from './rings';

interface Case {
  name: string;
  wall_height: number;
  shapes: Record<string, { fill: boolean; height: number }>;
  erased: number[];
  cells: number[][];
  heights: (number | null)[][];
  kinds: string[][];
}

const CASES = golden as unknown as Case[];
const byName = (n: string) => {
  const c = CASES.find((x) => x.name === n);
  if (!c) throw new Error(`no fixture case ${n}`);
  return c;
};

function gridOf(rows: number[][]): Grid {
  const H = rows.length;
  const W = H ? rows[0].length : 0;
  return { W, H, res: 1, ox: 0, oy: 0, cells: Uint8Array.from(rows.flat()) };
}

/** The (grid, edit state) of a fixture case; `fill` overrides every shape's fill flag. */
function caseState(c: Case, fill?: boolean): { g: Grid; s: EditState } {
  const g = gridOf(c.cells);
  const outlines: Outline[] = Object.keys(c.shapes).map((k) => ({ id: Number(k), area_m2: 1, cx: 0, cy: 0 }));
  const shapes: Record<string, { fill: boolean; height: number }> = {};
  for (const [k, v] of Object.entries(c.shapes)) shapes[k] = { fill: fill ?? v.fill, height: v.height };
  let s = initEdit(g, outlines, { wall_height: c.wall_height, shapes });
  if (c.erased.length) s = { ...s, erased: new Set(c.erased) };
  return { g, s };
}

/** Per-cell built height (null = no box) and kind ("wall", "shape:<id>", ""), as the fixture has them. */
function raster(b: Boxes, s: EditState, g: Grid) {
  const heights: (number | null)[][] = Array.from({ length: g.H }, () => Array(g.W).fill(null));
  const kinds: string[][] = Array.from({ length: g.H }, () => Array(g.W).fill(''));
  const paint = (rects: Rect[], h: number, kind: string) => {
    for (const [r0, c0, nr, nc] of rects) {
      for (let r = r0; r < r0 + nr; r++) {
        for (let c = c0; c < c0 + nc; c++) {
          expect(heights[r][c], `cell (${r}, ${c}) covered twice`).toBeNull();
          heights[r][c] = h;
          kinds[r][c] = kind;
        }
      }
    }
  };
  paint(b.walls, s.wallHeight, 'wall');
  for (const { h, rects } of b.raised) paint(rects, h, 'wall');
  for (const [id, rects] of Object.entries(b.shapes)) paint(rects, s.shapes[Number(id)].height, `shape:${id}`);
  return { heights, kinds };
}

function occupiedAfterErase(c: Case): boolean[][] {
  const W = c.cells[0].length;
  return c.cells.map((row, r) => row.map((v, col) => v === 255 && !c.erased.includes(r * W + col)));
}

describe('outline rings', () => {
  it('the fixture has every case', () => {
    expect(CASES.length).toBeGreaterThanOrEqual(14);
    expect(byName('E_wall_2cell').wall_height).toBe(0.8);
  });

  it.each(CASES.map((c) => [c.name, c] as const))('golden cases match python: %s', (_n, c) => {
    const { g, s } = caseState(c);
    const { heights, kinds } = raster(boxesFor(s, g), s, g);
    expect(kinds).toEqual(c.kinds);
    expect(heights).toEqual(c.heights);
  });

  it.each(CASES.map((c) => [c.name, c] as const))('unfill restores wall: %s', (_n, c) => {
    const { g, s } = caseState(c, false);
    const b = boxesFor(s, g);
    expect(b.raised).toEqual([]);
    expect(b.shapes).toEqual({});
    const occ = occupiedAfterErase(c);
    const { heights, kinds } = raster(b, s, g);
    expect(heights).toEqual(occ.map((row) => row.map((o) => (o ? c.wall_height : null))));
    expect(kinds).toEqual(occ.map((row) => row.map((o) => (o ? 'wall' : ''))));
  });

  it('unfill restores wall: C with its fill unticked builds exactly H', () => {
    const { g, s } = caseState(byName('C_wall_1cell'));
    const off = toggleShape(s, 1);
    const { heights, kinds } = raster(boxesFor(off, g), off, g);
    expect(heights).toEqual(byName('H_unfilled').heights);
    expect(kinds).toEqual(byName('H_unfilled').kinds);
    // and ticking it again gives C back
    const on = toggleShape(off, 1);
    expect(raster(boxesFor(on, g), on, g).heights).toEqual(byName('C_wall_1cell').heights);
  });

  it('wall line never notched: every wall cell that is not a free edge stays >= wall height', () => {
    for (const c of CASES) {
      const { g, s } = caseState(c);
      const rings = ringsFor(s, g);
      const { heights, kinds } = raster(boxesFor(s, g), s, g);
      const occ = occupiedAfterErase(c);
      for (let r = 0; r < g.H; r++) {
        for (let col = 0; col < g.W; col++) {
          const i = r * g.W + col;
          if (!occ[r][col] || (rings.inRing[i] && !rings.strip[i])) continue;
          expect(kinds[r][col], `${c.name} (${r}, ${col})`).toBe('wall');
          expect(heights[r][col]!, `${c.name} (${r}, ${col})`).toBeGreaterThanOrEqual(c.wall_height);
        }
      }
    }
  });

  it('box groups: walls, raised strips by ascending height, then shapes by id', () => {
    const { g, s: s0 } = caseState(byName('D2_pair_on_wall'));
    // a raised group at 0.9 too: shape 2 at 0.9, a phantom filled shape 3 with no interior
    let s = setShapeHeight(s0, 1, 1.2);
    s = { ...s, outlines: [...s.outlines, { id: 3, area_m2: 0, cx: 0, cy: 0 }], shapes: { ...s.shapes, 3: { fill: true, height: 0.9 } } };
    s = setShapeHeight(s, 2, 0.9);
    const b = boxesFor(s, g);
    expect(b.raised.map((x) => x.h)).toEqual([0.9, 1.2]);
    expect(Object.keys(b.shapes)).toEqual(['1', '2', '3']);
    expect(b.shapes[3]).toEqual([]);
    const sorted = (rects: Rect[]) => [...rects].sort((a, b2) => a[0] - b2[0] || a[1] - b2[1] || a[2] - b2[2] || a[3] - b2[3]);
    for (const rects of [b.walls, ...b.raised.map((x) => x.rects), ...Object.values(b.shapes)]) {
      expect(rects).toEqual(sorted(rects));
    }
    // raising the wall above every shape folds the strips back into the walls
    const hi = setWallHeight(s, 2.0);
    expect(boxesFor(hi, g).raised).toEqual([]);
  });

  it('classification cached across height edits', () => {
    const { g, s } = caseState(byName('C_wall_1cell'));
    const a = ringsFor(s, g);
    expect(ringsFor(setShapeHeight(s, 1, 1.5), g)).toBe(a);
    expect(ringsFor(setDefaultShapeHeight(s, 0.3), g)).toBe(a);
    expect(ringsFor(setWallHeight(s, 0.9), g)).toBe(a);
    // a copy of the same filled set (new shapes object) still hits the cache
    expect(ringsFor({ ...s, shapes: { ...s.shapes } }, g)).toBe(a);
    // the filled set and the erased cells do reclassify
    const off = toggleShape(s, 1);
    const b = ringsFor(off, g);
    expect(b).not.toBe(a);
    expect([...b.inRing].every((v) => v === 0)).toBe(true);
    const back = ringsFor(toggleShape(off, 1), g);
    expect([...back.strip]).toEqual([...a.strip]);
  });

  it('erasing beside a strip reclassifies it, and undo brings the strip back', () => {
    const c = byName('C_wall_1cell');
    const { g, s } = caseState(c);
    const strip0 = ringsFor(s, g).strip;
    expect(strip0[4]).toBe(1); // the back row is strip
    const e1 = eraseAt(s, g, 0, 3, 1);
    const e2 = eraseAt(e1, g, 0, 11, 1);
    const r2 = ringsFor(e2, g);
    expect(r2.strip.every((v) => v === 0)).toBe(true); // nothing left to continue
    const b = boxesFor(e2, g);
    expect(b.raised).toEqual([]);
    expect(b.shapes[1].some(([r0]) => r0 === 0)).toBe(true); // the back row joined the table
    const u = undoStroke(undoStroke(e2));
    expect([...ringsFor(u, g).strip]).toEqual([...strip0]);
    expect(raster(boxesFor(u, g), u, g).heights).toEqual(c.heights);
  });
});

describe('classifyRings and owners', () => {
  /** The prototype's per-cell walk (outline-proto.py) and the spec's ownership rule, cell by cell. */
  function reference(g: Grid, erased: Set<number>, heights: Record<number, number>) {
    const { W, H } = g;
    const occ = (r: number, c: number) => g.cells[r * W + c] === 255 && !erased.has(r * W + c);
    const owner = new Int32Array(W * H);
    const ring = new Uint8Array(W * H);
    const ids = Object.keys(heights).map(Number).sort((a, b) => heights[b] - heights[a] || a - b);
    for (const k of ids) {
      for (let r = 0; r < H; r++) {
        for (let c = 0; c < W; c++) {
          if (!occ(r, c) || ring[r * W + c]) continue;
          let hit = false;
          for (let dr = -1; dr <= 1; dr++) {
            for (let dc = -1; dc <= 1; dc++) {
              const rr = r + dr, cc = c + dc;
              if (rr >= 0 && rr < H && cc >= 0 && cc < W && g.cells[rr * W + cc] === k) hit = true;
            }
          }
          if (hit) { ring[r * W + c] = 1; owner[r * W + c] = k; }
        }
      }
    }
    const wall = (r: number, c: number) => r >= 0 && r < H && c >= 0 && c < W && occ(r, c) && !ring[r * W + c];
    const run = (r: number, c: number, dr: number, dc: number) => {
      let n = 0;
      for (let rr = r + dr, cc = c + dc; wall(rr, cc); rr += dr, cc += dc) n++;
      return n;
    };
    const strip = new Uint8Array(W * H);
    for (let r = 0; r < H; r++) {
      for (let c = 0; c < W; c++) {
        if (!ring[r * W + c]) continue;
        for (const [dr, dc, horiz] of [[0, -1, true], [0, 1, true], [-1, 0, false], [1, 0, false]] as const) {
          let rr = r + dr, cc = c + dc;
          while (rr >= 0 && rr < H && cc >= 0 && cc < W && ring[rr * W + cc]) { rr += dr; cc += dc; }
          if (!wall(rr, cc)) continue;
          const hr = 1 + run(rr, cc, 0, -1) + run(rr, cc, 0, 1);
          const vr = 1 + run(rr, cc, -1, 0) + run(rr, cc, 1, 0);
          if (horiz ? hr > vr : vr > hr) { strip[r * W + c] = 1; break; }
        }
      }
    }
    return { owner, strip, ring };
  }

  it('match the per-cell reference walk on random grids', () => {
    let seed = 20261020;
    const rand = () => { seed = (seed * 1103515245 + 12345) >>> 0; return seed / 2 ** 32; };
    let checked = 0;
    for (let t = 0; t < 400; t++) {
      const H = 4 + Math.floor(rand() * 20);
      const W = 4 + Math.floor(rand() * 20);
      const pOcc = 0.3 + rand() * 0.45;
      const cells = new Uint8Array(W * H);
      for (let i = 0; i < cells.length; i++) {
        const x = rand();
        cells[i] = x < pOcc ? 255 : x < pOcc + 0.15 ? 1 + Math.floor(rand() * 4) : 0;
      }
      const g: Grid = { W, H, res: 1, ox: 0, oy: 0, cells };
      const erased = new Set<number>();
      for (let i = 0; i < cells.length; i++) if (cells[i] === 255 && rand() < 0.1) erased.add(i);
      const heights: Record<number, number> = {};
      for (let k = 1; k <= 4; k++) if (rand() < 0.7) heights[k] = [0.3, 0.6, 0.9][Math.floor(rand() * 3)];
      const filled = new Set(Object.keys(heights).map(Number));
      const rings = classifyRings(g, erased, filled);
      const own = owners(g, rings, heights);
      const ref = reference(g, erased, heights);
      expect([...rings.inRing]).toEqual([...ref.ring]);
      expect([...rings.strip]).toEqual([...ref.strip]);
      expect([...own]).toEqual([...ref.owner]);
      if (filled.size) checked++;
    }
    expect(checked).toBeGreaterThan(200);
  });

  it('owners follow height edits without reclassifying (tallest wins, lowest id on a tie)', () => {
    const c = byName('G2_two_tables_tie');
    const { g } = caseState(c);
    const rings = classifyRings(g, new Set(), new Set([1, 2]));
    const shared = [...owners(g, rings, { 1: 0.5, 2: 0.5 })];
    const two = [...owners(g, rings, { 1: 0.5, 2: 0.9 })];
    let tie = 0;
    for (let i = 0; i < shared.length; i++) {
      if (shared[i] && two[i] !== shared[i]) { expect(shared[i]).toBe(1); expect(two[i]).toBe(2); tie++; }
    }
    expect(tie).toBeGreaterThan(0);
  });
});
