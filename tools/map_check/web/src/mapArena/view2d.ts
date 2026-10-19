// Placement of the 2D map canvas in its pane, and pane pixels -> map cells.

/** [row0, col0, row1, col1): the part of the map shown (see editState.contentBounds). */
export type View = [number, number, number, number];

/** The canvas box inside the stage, in CSS px; `cell` is the size of one map cell. */
export interface Fit { left: number; top: number; w: number; h: number; cell: number }

/** The largest square-cell box for `view` inside a stage of sw x sh, `pad` px from every side, centred. */
export function fitView(sw: number, sh: number, view: View, pad: number): Fit | null {
  const rows = view[2] - view[0];
  const cols = view[3] - view[1];
  const aw = sw - 2 * pad;
  const ah = sh - 2 * pad;
  if (aw <= 0 || ah <= 0 || rows <= 0 || cols <= 0) return null;
  const cell = Math.min(aw / cols, ah / rows);
  const w = cols * cell;
  const h = rows * cell;
  return { left: (sw - w) / 2, top: (sh - h) / 2, w, h, cell };
}

/**
 * The map cell [row, col] under stage point (x, y). Off the canvas it is null, or with `clamp`
 * the nearest edge cell (so a brush drag that leaves the canvas keeps painting the border).
 */
export function pointToCell(f: Fit, view: View, x: number, y: number, clamp: boolean): [number, number] | null {
  const rows = view[2] - view[0];
  const cols = view[3] - view[1];
  let r = Math.floor(((y - f.top) / f.h) * rows);
  let c = Math.floor(((x - f.left) / f.w) * cols);
  if (clamp) {
    r = Math.min(rows - 1, Math.max(0, r));
    c = Math.min(cols - 1, Math.max(0, c));
  } else if (r < 0 || c < 0 || r >= rows || c >= cols) {
    return null;
  }
  return [view[0] + r, view[1] + c];
}
