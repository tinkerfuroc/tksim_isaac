// Outline rings of filled shapes: free edges versus the shared wall strip. Port of
// map_check/rings.py ("The rule, exactly" in docs/superpowers/specs/2026-10-20-shape-outline-height-design.md);
// the golden fixture outline_golden.json checks both against the same cells.
import type { Grid } from './editState';

const WALL = 255; // occupied cell code of the web grid (editState.WALL)

/**
 * The classification, which depends only on the occupied cells after erasing and the set of
 * filled shapes (not on heights), so it can be cached across height edits.
 */
export interface Rings {
  inRing: Uint8Array; // 1: a remaining occupied cell 8-adjacent to a filled shape's interior
  strip: Uint8Array; // 1: a ring cell that continues a wall line (built as wall)
}

/**
 * Classify the rings of the filled shapes.
 *
 * Ring of k = occupied, non-erased cells 8-adjacent to an interior cell of k. W = the remaining
 * occupied cells. A ring cell is strip when walking from it left, right, up or down through ring
 * cells first leaves the ring onto a W cell whose W run along the walk is strictly longer than
 * its run across it. Every cell of a straight ring run leaves it at the same two cells (one past
 * each end), so a whole run qualifies when either end's outer neighbour does. Then, once, a ring run
 * with a strip end cell where the wall turns a corner becomes all strip (step 3b, `cornerExtension`).
 */
export function classifyRings(g: Grid, erased: ReadonlySet<number>, filled: ReadonlySet<number>): Rings {
  const { W, H, cells } = g;
  const n = W * H;
  const isFilled = new Uint8Array(256);
  for (const k of filled) if (k > 0 && k < WALL) isFilled[k] = 1;
  const occ = new Uint8Array(n);
  for (let i = 0; i < n; i++) occ[i] = cells[i] === WALL ? 1 : 0;
  for (const i of erased) if (i >= 0 && i < n) occ[i] = 0;

  const inRing = new Uint8Array(n);
  if (filled.size) {
    for (let r = 0; r < H; r++) {
      const ra = r > 0 ? r - 1 : r, rb = r < H - 1 ? r + 1 : r;
      for (let c = 0; c < W; c++) {
        const i = r * W + c;
        if (!occ[i]) continue;
        const ca = c > 0 ? c - 1 : c, cb = c < W - 1 ? c + 1 : c;
        search: for (let rr = ra; rr <= rb; rr++) {
          for (let cc = ca; cc <= cb; cc++) {
            if (isFilled[cells[rr * W + cc]]) { inRing[i] = 1; break search; }
          }
        }
      }
    }
  }
  const strip = new Uint8Array(n);
  if (!filled.size) return { inRing, strip };

  // W and its straight run lengths through every W cell.
  const wall = new Uint8Array(n);
  for (let i = 0; i < n; i++) wall[i] = occ[i] && !inRing[i] ? 1 : 0;
  const hrun = new Int32Array(n);
  const vrun = new Int32Array(n);
  for (let r = 0; r < H; r++) {
    let c = 0;
    while (c < W) {
      if (!wall[r * W + c]) { c++; continue; }
      const c0 = c;
      while (c < W && wall[r * W + c]) c++;
      for (let k = c0; k < c; k++) hrun[r * W + k] = c - c0;
    }
  }
  for (let c = 0; c < W; c++) {
    let r = 0;
    while (r < H) {
      if (!wall[r * W + c]) { r++; continue; }
      const r0 = r;
      while (r < H && wall[r * W + c]) r++;
      for (let k = r0; k < r; k++) vrun[k * W + c] = r - r0;
    }
  }

  // Horizontal ring runs that end on a W cell running horizontally.
  for (let r = 0; r < H; r++) {
    let c = 0;
    while (c < W) {
      if (!inRing[r * W + c]) { c++; continue; }
      const c0 = c;
      while (c < W && inRing[r * W + c]) c++;
      const a = c0 - 1, b = c;
      if ((a >= 0 && wall[r * W + a] && hrun[r * W + a] > vrun[r * W + a])
        || (b < W && wall[r * W + b] && hrun[r * W + b] > vrun[r * W + b])) {
        for (let k = c0; k < c; k++) strip[r * W + k] = 1;
      }
    }
  }
  // Vertical ring runs that end on a W cell running vertically.
  for (let c = 0; c < W; c++) {
    let r = 0;
    while (r < H) {
      if (!inRing[r * W + c]) { r++; continue; }
      const r0 = r;
      while (r < H && inRing[r * W + c]) r++;
      const a = r0 - 1, b = r;
      if ((a >= 0 && wall[a * W + c] && vrun[a * W + c] > hrun[a * W + c])
        || (b < H && wall[b * W + c] && vrun[b * W + c] > hrun[b * W + c])) {
        for (let k = r0; k < r; k++) strip[k * W + c] = 1;
      }
    }
  }
  return { inRing, strip: cornerExtension(W, H, occ, inRing, strip) };
}

/**
 * Step 3b, applied once to the step-3 `strip`: a straight ring run becomes all strip when one of its
 * end cells is strip and the occupied line across the run at that cell does not continue on both
 * sides (the wall turns a corner there: an L, not a T). Only step-3 strip cells trigger it, in both
 * directions, so the cells it adds trigger nothing (no chaining).
 */
function cornerExtension(W: number, H: number, occ: Uint8Array, inRing: Uint8Array, strip: Uint8Array): Uint8Array {
  const out = strip.slice();
  const o = (r: number, c: number) => r >= 0 && r < H && c >= 0 && c < W && occ[r * W + c] === 1;
  const turns = (r: number, c: number, horiz: boolean) =>
    strip[r * W + c] === 1 && !(horiz ? o(r - 1, c) && o(r + 1, c) : o(r, c - 1) && o(r, c + 1));
  for (let r = 0; r < H; r++) {
    let c = 0;
    while (c < W) {
      if (!inRing[r * W + c]) { c++; continue; }
      const c0 = c;
      while (c < W && inRing[r * W + c]) c++;
      if (turns(r, c0, true) || turns(r, c - 1, true)) {
        for (let k = c0; k < c; k++) out[r * W + k] = 1;
      }
    }
  }
  for (let c = 0; c < W; c++) {
    let r = 0;
    while (r < H) {
      if (!inRing[r * W + c]) { r++; continue; }
      const r0 = r;
      while (r < H && inRing[r * W + c]) r++;
      if (turns(r0, c, false) || turns(r - 1, c, false)) {
        for (let k = r0; k < r; k++) out[k * W + c] = 1;
      }
    }
  }
  return out;
}

/**
 * The owner of every ring cell: of the filled shapes (the keys of `heights`) whose interior is
 * 8-adjacent to it, the tallest, the lowest id on a tie; 0 outside the rings. Cheap, so it is
 * recomputed on height edits while the classification stays cached.
 */
export function owners(g: Grid, rings: Rings, heights: Record<number, number>): Int32Array {
  const { W, H, cells } = g;
  const hOf = new Float64Array(256).fill(-Infinity);
  for (const [k, h] of Object.entries(heights)) {
    const id = Number(k);
    if (id > 0 && id < WALL) hOf[id] = h;
  }
  const out = new Int32Array(W * H);
  const { inRing } = rings;
  for (let r = 0; r < H; r++) {
    const ra = r > 0 ? r - 1 : r, rb = r < H - 1 ? r + 1 : r;
    for (let c = 0; c < W; c++) {
      const i = r * W + c;
      if (!inRing[i]) continue;
      const ca = c > 0 ? c - 1 : c, cb = c < W - 1 ? c + 1 : c;
      let best = 0;
      let bestH = -Infinity;
      for (let rr = ra; rr <= rb; rr++) {
        for (let cc = ca; cc <= cb; cc++) {
          const v = cells[rr * W + cc];
          const h = hOf[v];
          if (h > bestH || (h === bestH && h > -Infinity && v < best)) { best = v; bestH = h; }
        }
      }
      out[i] = best;
    }
  }
  return out;
}
