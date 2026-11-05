import { useEffect, useLayoutEffect, useMemo, useRef, useState, type PointerEvent } from 'react';
import { FREE, WALL, cellCenter, contentBounds, freeEdgeOwners, type EditState, type Grid } from './editState';
import { PALETTE, rgb } from './palette';
import { fitView, pointToCell, type Fit } from './view2d';

const PAD_PX = 16; // space between the map and the pane edges
const MARGIN_CELLS = 4; // free cells kept around the occupied part of the map

interface Props {
  grid: Grid;
  edit: EditState;
  erasing: boolean;
  brush: number;
  onSelect: (id: number | null) => void;
  /** One brush move from (r0, c0) to (r1, c1). `first` is the pointer-down of a new drag. */
  onErase: (r0: number, c0: number, r1: number, c1: number, first: boolean) => void;
}

type Cell = [number, number];

const COLORS = {
  free: rgb(PALETTE.free),
  wall: rgb(PALETTE.wall),
  erase: rgb(PALETTE.erase),
  shape: rgb(PALETTE.shape),
  shapeOff: rgb(PALETTE.shapeOff),
  selected: rgb(PALETTE.selected),
};

function describeCell(g: Grid, edit: EditState, edges: Int32Array, [r, c]: Cell): string {
  const [x, y] = cellCenter(g, r, c);
  const i = r * g.W + c;
  const v = g.cells[i];
  let kind: string;
  if (v === FREE) kind = 'free';
  else if (v === WALL) kind = edit.erased.has(i) ? 'wall, erased' : edges[i] ? `shape ${edges[i]} edge` : 'wall';
  else kind = `shape ${v}${edit.shapes[v]?.fill ? '' : ' (not filled)'}`;
  return `x ${x.toFixed(2)}  y ${y.toFixed(2)}  ·  ${kind}`;
}

/**
 * The 2D map: one canvas pixel per cell, scaled up with crisp edges. It shows the occupied part
 * of the map (plus a small free margin) as large as the pane allows.
 */
export function MapCanvas2D({ grid, edit, erasing, brush, onSelect, onErase }: Props) {
  const stage = useRef<HTMLDivElement>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const view = useMemo(() => contentBounds(grid, MARGIN_CELLS), [grid]);
  const [fit, setFit] = useState<Fit | null>(null); // canvas box in stage px; null while hidden
  const [hover, setHover] = useState<Cell | null>(null);
  const drag = useRef<Cell | null>(null); // last cell of the brush drag in progress
  // The free edges of filled outlines (built at the shape's height) and the shape each belongs to.
  // The ring classification behind it is cached in editState, so height edits don't redo it.
  const edges = useMemo(() => freeEdgeOwners(edit, grid), [grid, edit.erased, edit.shapes]);

  // Fit the canvas to the pane on every resize (and when a hidden tab is shown again).
  useLayoutEffect(() => {
    const st = stage.current;
    if (!st) return;
    const resize = () => setFit(fitView(st.clientWidth, st.clientHeight, view, PAD_PX));
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(st);
    return () => ro.disconnect();
  }, [view]);

  // Paint the shown cells; cheap enough (one ImageData) to redo on each edit.
  useEffect(() => {
    const cv = canvas.current;
    const ctx = cv?.getContext('2d');
    if (!cv || !ctx) return;
    const [r0, c0, r1, c1] = view;
    const w = c1 - c0;
    cv.width = w;
    cv.height = r1 - r0;
    const img = ctx.createImageData(w, r1 - r0);
    const px = img.data;
    for (let r = r0; r < r1; r++) {
      for (let c = c0; c < c1; c++) {
        const i = r * grid.W + c;
        const v = grid.cells[i];
        let col: readonly number[];
        if (v === FREE) col = COLORS.free;
        else if (v === WALL) {
          const k = edges[i]; // a free edge takes its shape's colour; a shared strip stays wall
          col = edit.erased.has(i) ? COLORS.erase : !k ? COLORS.wall : k === edit.selected ? COLORS.selected : COLORS.shape;
        } else if (edit.selected === v) col = COLORS.selected;
        else col = edit.shapes[v]?.fill ? COLORS.shape : COLORS.shapeOff;
        const o = ((r - r0) * w + (c - c0)) * 4;
        px[o] = col[0]; px[o + 1] = col[1]; px[o + 2] = col[2]; px[o + 3] = 255;
      }
    }
    ctx.putImageData(img, 0, 0);
  }, [grid, view, edges, edit.erased, edit.shapes, edit.selected]);

  // Pointer position -> map cell, through the same fit the canvas is placed with.
  const toCell = (e: PointerEvent, clamp: boolean): Cell | null => {
    if (!fit || !stage.current) return null;
    const b = stage.current.getBoundingClientRect();
    return pointToCell(fit, view, e.clientX - b.left, e.clientY - b.top, clamp);
  };
  const cellAt = (e: PointerEvent) => toCell(e, false);
  const clampCell = (e: PointerEvent) => toCell(e, true);

  const onPointerDown = (e: PointerEvent<HTMLCanvasElement>) => {
    if (e.button !== 0) return;
    const cell = cellAt(e);
    if (!cell) return;
    if (erasing) {
      e.currentTarget.setPointerCapture(e.pointerId);
      drag.current = cell;
      onErase(cell[0], cell[1], cell[0], cell[1], true);
      return;
    }
    const i = cell[0] * grid.W + cell[1];
    const v = grid.cells[i];
    const id = v !== FREE && v !== WALL ? v : edges[i] || null; // a free edge selects its shape
    onSelect(id !== null && id === edit.selected ? null : id);
  };
  const onPointerMove = (e: PointerEvent<HTMLCanvasElement>) => {
    setHover(cellAt(e));
    if (!drag.current) return;
    const cell = clampCell(e);
    if (!cell) return;
    const [r0, c0] = drag.current;
    if (r0 === cell[0] && c0 === cell[1]) return;
    drag.current = cell;
    onErase(r0, c0, cell[0], cell[1], false);
  };
  const endDrag = () => { drag.current = null; };

  // A newly opened map has nothing under the pointer yet.
  useEffect(() => { setHover(null); drag.current = null; }, [grid]);

  // Leaving erase mode mid-drag ends the drag.
  useEffect(() => { if (!erasing) drag.current = null; }, [erasing]);

  const n = Math.max(1, Math.round(brush));
  const lo = Math.floor((n - 1) / 2);

  return (
    <section className="ma-pane">
      <h2>2D map <span>{hover ? describeCell(grid, edit, edges, hover) : 'hover a cell'}</span></h2>
      <div className="ma-stage" ref={stage}>
        <canvas
          ref={canvas}
          className={erasing ? 'map2d erasing' : 'map2d'}
          style={fit ? { left: fit.left, top: fit.top, width: fit.w, height: fit.h } : { visibility: 'hidden' }}
          aria-label="2D map. Click a shape to highlight it; with the erase brush on, drag over specks."
          onPointerDown={onPointerDown}
          onPointerMove={onPointerMove}
          onPointerUp={endDrag}
          onPointerCancel={endDrag}
          onLostPointerCapture={endDrag}
          onPointerLeave={() => setHover(null)}
        />
        {erasing && hover && fit && (
          <div className="ma-brush" style={{
            left: fit.left + (hover[1] - lo - view[1]) * fit.cell,
            top: fit.top + (hover[0] - lo - view[0]) * fit.cell,
            width: n * fit.cell,
            height: n * fit.cell,
          }} />
        )}
        <div className="ma-legend">
          <span><i style={{ background: PALETTE.wall }} />wall</span>
          <span><i style={{ background: PALETTE.shape }} />filled shape</span>
          <span><i style={{ background: PALETTE.shapeOff }} />closed, not filled</span>
          <span><i style={{ background: PALETTE.erase }} />erased</span>
          <span><i style={{ background: PALETTE.selected }} />selected</span>
        </div>
      </div>
    </section>
  );
}
