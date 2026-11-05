export type Rect = [number, number, number, number]; // [row0, col0, nrows, ncols]

/**
 * Port of arena_build.merge_rects. `mask` is row-major, H rows by W columns.
 * Row runs are merged while the identical column span repeats on consecutive
 * rows. Output is sorted ascending by (row0, col0, nrows, ncols).
 */
export function mergeRects(mask: Uint8Array, W: number, H: number): Rect[] {
  const open = new Map<number, number>(); // c0 * (W + 1) + c1 -> start row
  const rects: Rect[] = [];
  for (let r = 0; r <= H; r++) {
    const runs = new Map<number, [number, number]>();
    if (r < H) {
      let c = 0;
      while (c < W) {
        if (!mask[r * W + c]) { c++; continue; }
        const c0 = c;
        while (c < W && mask[r * W + c]) c++;
        runs.set(c0 * (W + 1) + c, [c0, c]);
      }
    }
    for (const [k, r0] of [...open]) {
      if (!runs.has(k)) {
        open.delete(k);
        const c0 = Math.floor(k / (W + 1));
        const c1 = k - c0 * (W + 1);
        rects.push([r0, c0, r - r0, c1 - c0]);
      }
    }
    for (const k of runs.keys()) if (!open.has(k)) open.set(k, r);
  }
  return rects.sort((a, b) => a[0] - b[0] || a[1] - b[1] || a[2] - b[2] || a[3] - b[3]);
}
