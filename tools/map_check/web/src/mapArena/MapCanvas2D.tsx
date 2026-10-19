import { useEffect, useLayoutEffect, useRef, useState, type PointerEvent } from 'react';
import { FREE, WALL, cellCenter, type EditState, type Grid } from './editState';
import { PALETTE, rgb } from './palette';

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

function describeCell(g: Grid, edit: EditState, [r, c]: Cell): string {
  const [x, y] = cellCenter(g, r, c);
  const v = g.cells[r * g.W + c];
  let kind: string;
  if (v === FREE) kind = 'free';
  else if (v === WALL) kind = edit.erased.has(r * g.W + c) ? 'wall, erased' : 'wall';
  else kind = `shape ${v}${edit.shapes[v]?.fill ? '' : ' (not filled)'}`;
  return `x ${x.toFixed(2)}  y ${y.toFixed(2)}  ·  ${kind}`;
}

/** The 2D map: one canvas pixel per cell, scaled up to fit with crisp edges. */
export function MapCanvas2D({ grid, edit, erasing, brush, onSelect, onErase }: Props) {
  const stage = useRef<HTMLDivElement>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const [fit, setFit] = useState({ w: 0, h: 0, left: 0, top: 0 }); // canvas box in stage px
  const [hover, setHover] = useState<Cell | null>(null);
  const drag = useRef<Cell | null>(null); // last cell of the brush drag in progress

  // Scale the canvas to fit the pane, keeping square cells.
  useLayoutEffect(() => {
    const st = stage.current;
    if (!st) return;
    const resize = () => {
      const k = Math.min(st.clientWidth / grid.W, st.clientHeight / grid.H);
      const w = Math.max(1, Math.floor(grid.W * k));
      const h = Math.max(1, Math.floor(grid.H * k));
      setFit({ w, h, left: (st.clientWidth - w) / 2, top: (st.clientHeight - h) / 2 });
    };
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(st);
    return () => ro.disconnect();
  }, [grid]);

  // Paint every cell; cheap enough (one ImageData) to redo on each edit.
  useEffect(() => {
    const cv = canvas.current;
    const ctx = cv?.getContext('2d');
    if (!cv || !ctx) return;
    cv.width = grid.W;
    cv.height = grid.H;
    const img = ctx.createImageData(grid.W, grid.H);
    const px = img.data;
    for (let i = 0; i < grid.cells.length; i++) {
      const v = grid.cells[i];
      let col: readonly number[];
      if (v === FREE) col = COLORS.free;
      else if (v === WALL) col = edit.erased.has(i) ? COLORS.erase : COLORS.wall;
      else if (edit.selected === v) col = COLORS.selected;
      else col = edit.shapes[v]?.fill ? COLORS.shape : COLORS.shapeOff;
      px[i * 4] = col[0]; px[i * 4 + 1] = col[1]; px[i * 4 + 2] = col[2]; px[i * 4 + 3] = 255;
    }
    ctx.putImageData(img, 0, 0);
  }, [grid, edit.erased, edit.shapes, edit.selected]);

  const cellAt = (e: PointerEvent): Cell | null => {
    const b = canvas.current!.getBoundingClientRect();
    const r = Math.floor(((e.clientY - b.top) / b.height) * grid.H);
    const c = Math.floor(((e.clientX - b.left) / b.width) * grid.W);
    return r >= 0 && c >= 0 && r < grid.H && c < grid.W ? [r, c] : null;
  };
  const clampCell = (e: PointerEvent): Cell => {
    const b = canvas.current!.getBoundingClientRect();
    const r = Math.floor(((e.clientY - b.top) / b.height) * grid.H);
    const c = Math.floor(((e.clientX - b.left) / b.width) * grid.W);
    return [Math.min(grid.H - 1, Math.max(0, r)), Math.min(grid.W - 1, Math.max(0, c))];
  };

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
    const v = grid.cells[cell[0] * grid.W + cell[1]];
    const id = v !== FREE && v !== WALL ? v : null;
    onSelect(id !== null && id === edit.selected ? null : id);
  };
  const onPointerMove = (e: PointerEvent<HTMLCanvasElement>) => {
    setHover(cellAt(e));
    if (!drag.current) return;
    const cell = clampCell(e);
    const [r0, c0] = drag.current;
    if (r0 === cell[0] && c0 === cell[1]) return;
    drag.current = cell;
    onErase(r0, c0, cell[0], cell[1], false);
  };
  const endDrag = () => { drag.current = null; };

  // Leaving erase mode mid-drag ends the drag.
  useEffect(() => { if (!erasing) drag.current = null; }, [erasing]);

  const cellPx = fit.w / grid.W;
  const n = Math.max(1, Math.round(brush));
  const lo = Math.floor((n - 1) / 2);

  return (
    <section className="ma-pane">
      <h2>2D map <span>{hover ? describeCell(grid, edit, hover) : 'hover a cell'}</span></h2>
      <div className="ma-stage" ref={stage}>
        <canvas
          ref={canvas}
          className={erasing ? 'map2d erasing' : 'map2d'}
          style={{ width: fit.w, height: fit.h }}
          aria-label="2D map. Click a shape to highlight it; with the erase brush on, drag over specks."
          onPointerDown={onPointerDown}
          onPointerMove={onPointerMove}
          onPointerUp={endDrag}
          onPointerCancel={endDrag}
          onLostPointerCapture={endDrag}
          onPointerLeave={() => setHover(null)}
        />
        {erasing && hover && (
          <div className="ma-brush" style={{
            left: fit.left + (hover[1] - lo) * cellPx,
            top: fit.top + (hover[0] - lo) * cellPx,
            width: n * cellPx,
            height: n * cellPx,
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
