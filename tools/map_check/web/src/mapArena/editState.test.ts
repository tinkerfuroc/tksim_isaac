import { describe, expect, it } from 'vitest';
import golden from '../../../tests/fixtures/rects_golden.json';
import type { Outline, WebGrid } from '../api';
import {
  applyFillRule, boxesFor, decodeGrid, eraseAt, eraseLine, initEdit, savePayload, selectShape,
  setDefaultShapeHeight, setShapeHeight, setWallHeight, toggleShape, undoStroke,
  type Grid,
} from './editState';

type Case = { mask: number[][]; rects: [number, number, number, number][] };

const WALL = 255;

/** A W x H grid from rows of cell codes (0 free, 1..254 outline id, 255 wall). */
function gridOf(rows: number[][], res = 0.5, ox = -1, oy = 2): Grid {
  const H = rows.length;
  const W = H ? rows[0].length : 0;
  return { W, H, res, ox, oy, cells: Uint8Array.from(rows.flat()) };
}

// 6 x 5: a closed square outline (id 1, 4 cells) and a single-cell outline (id 2).
const ROWS = [
  [255, 255, 255, 255, 255, 0],
  [255, 1, 1, 255, 2, 255],
  [255, 1, 1, 255, 255, 0],
  [255, 255, 255, 255, 0, 0],
  [0, 0, 0, 0, 0, 255],
];
const OUTLINES: Outline[] = [
  { id: 1, area_m2: 1.0, cx: 0.5, cy: 3.5 },
  { id: 2, area_m2: 0.25, cx: 1.25, cy: 4.25 },
];
const DEFAULTS = {
  wall_height: 0.6,
  shapes: { '1': { fill: true, height: 0.6 }, '2': { fill: true, height: 0.6 } },
};

describe('decodeGrid', () => {
  it('decodes the base64 web grid row by row', () => {
    const cells = Uint8Array.from([255, 0, 1, 255, 2, 0]);
    let bin = '';
    cells.forEach((v) => { bin += String.fromCharCode(v); });
    const web: WebGrid = { W: 3, H: 2, res: 0.05, ox: 1, oy: 2, grid: btoa(bin) };
    const g = decodeGrid(web);
    expect([g.W, g.H, g.res, g.ox, g.oy]).toEqual([3, 2, 0.05, 1, 2]);
    expect([...g.cells]).toEqual([255, 0, 1, 255, 2, 0]);
  });
  it('refuses a grid whose size does not match W x H', () => {
    const web: WebGrid = { W: 3, H: 2, res: 0.05, ox: 0, oy: 0, grid: btoa('abc') };
    expect(() => decodeGrid(web)).toThrow(/3 × 2/);
  });
});

describe('initEdit', () => {
  it('takes the server defaults', () => {
    const s = initEdit(gridOf(ROWS), OUTLINES, {
      wall_height: 0.7, shapes: { '1': { fill: true, height: 0.5 }, '2': { fill: false, height: 0.5 } },
    });
    expect(s.wallHeight).toBe(0.7);
    expect(s.defaultShapeHeight).toBe(0.5);
    expect(s.shapes).toEqual({ 1: { fill: true, height: 0.5 }, 2: { fill: false, height: 0.5 } });
    expect(s.erased.size).toBe(0);
    expect(s.strokes).toEqual([]);
    expect(s.selected).toBeNull();
  });
});

describe('shape settings', () => {
  it('fill rule sets checkboxes by area', () => {
    const s0 = initEdit(gridOf(ROWS), OUTLINES, DEFAULTS);
    const s1 = applyFillRule(s0, 0.5);
    expect(s1.shapes[1].fill).toBe(false);
    expect(s1.shapes[2].fill).toBe(true);
    const s2 = applyFillRule(s1, 1.0); // "up to" includes the limit
    expect(s2.shapes[1].fill).toBe(true);
    expect(applyFillRule(s2, 0).shapes[2].fill).toBe(false);
    expect(s0.shapes[1].fill).toBe(true); // pure: the input is untouched
  });
  it('toggles one shape and sets heights', () => {
    let s = initEdit(gridOf(ROWS), OUTLINES, DEFAULTS);
    s = toggleShape(s, 2);
    expect(s.shapes[2].fill).toBe(false);
    s = setShapeHeight(s, 1, 0.75);
    expect(s.shapes[1].height).toBe(0.75);
    s = setDefaultShapeHeight(s, 0.45);
    expect(s.defaultShapeHeight).toBe(0.45);
    expect(s.shapes[1].height).toBe(0.45);
    expect(s.shapes[2].height).toBe(0.45);
    expect(setWallHeight(s, 1.2).wallHeight).toBe(1.2);
  });
  it('selects and clears a shape', () => {
    let s = initEdit(gridOf(ROWS), OUTLINES, DEFAULTS);
    s = selectShape(s, 2);
    expect(s.selected).toBe(2);
    expect(selectShape(s, null).selected).toBeNull();
  });
});

describe('erase', () => {
  it('erase only touches occupied cells and undo restores', () => {
    const g = gridOf(ROWS);
    const s0 = initEdit(g, OUTLINES, DEFAULTS);
    // 3x3 brush centred on (1, 1): covers rows 0-2, cols 0-2 -> walls (0,0) (0,1) (0,2) (1,0) (2,0)
    const s1 = eraseAt(s0, g, 1, 1, 3);
    expect([...s1.erased].sort((a, b) => a - b)).toEqual([0, 1, 2, 6, 12]);
    expect(s1.strokes).toHaveLength(1);
    // a second stroke over free and outline cells only erases nothing and records nothing
    const s2 = eraseAt(s1, g, 4, 1, 1);
    expect(s2).toBe(s1);
    // erasing cells already erased records nothing either
    expect(eraseAt(s1, g, 0, 0, 1)).toBe(s1);
    const s3 = eraseAt(s1, g, 3, 3, 1);
    expect(s3.strokes).toHaveLength(2);
    expect(s3.erased.has(21)).toBe(true);
    const s4 = undoStroke(s3);
    expect([...s4.erased].sort((a, b) => a - b)).toEqual([0, 1, 2, 6, 12]);
    const s5 = undoStroke(s4);
    expect(s5.erased.size).toBe(0);
    expect(s5.strokes).toEqual([]);
    expect(undoStroke(s5)).toBe(s5);
    expect(s0.erased.size).toBe(0); // pure
  });
  it('one drag is one stroke', () => {
    const g = gridOf(ROWS);
    let s = initEdit(g, OUTLINES, DEFAULTS);
    s = eraseAt(s, g, 0, 0, 1);
    s = eraseAt(s, g, 0, 1, 1, true);
    s = eraseLine(s, g, 0, 2, 0, 4, 1, true);
    expect(s.strokes).toEqual([[0, 1, 2, 3, 4]]);
    expect(undoStroke(s).erased.size).toBe(0);
  });
  it('an even brush covers exactly that many cells and clips at the edge', () => {
    const g = gridOf([[255, 255, 255], [255, 255, 255], [255, 255, 255]]);
    const s = eraseAt(initEdit(g, [], { wall_height: 0.6, shapes: {} }), g, 1, 1, 2);
    expect([...s.erased].sort((a, b) => a - b)).toEqual([4, 5, 7, 8]);
    const edge = eraseAt(initEdit(g, [], { wall_height: 0.6, shapes: {} }), g, 0, 0, 3);
    expect([...edge.erased].sort((a, b) => a - b)).toEqual([0, 1, 3, 4]);
  });
});

describe('boxesFor', () => {
  it('boxesFor matches python boxes', () => {
    const c = (golden as Case[])[0];
    const g = gridOf(c.mask.map((row) => row.map((v) => (v ? WALL : 0))));
    const s = initEdit(g, [], { wall_height: 0.6, shapes: {} });
    const b = boxesFor(s, g);
    expect(b.walls).toEqual(c.rects);
    expect(b.shapes).toEqual({});
  });
  it('filled shapes match python boxes too (golden masks as one outline each)', () => {
    (golden as Case[]).forEach((c) => {
      // pad by a free border so the outline's bounding box is smaller than the map
      const rows = [
        Array(c.mask[0].length + 2).fill(0),
        ...c.mask.map((row) => [0, ...row.map((v) => (v ? 7 : 0)), 0]),
        Array(c.mask[0].length + 2).fill(0),
      ];
      const g = gridOf(rows);
      const s = initEdit(g, [{ id: 7, area_m2: 1, cx: 0, cy: 0 }],
        { wall_height: 0.6, shapes: { '7': { fill: true, height: 0.6 } } });
      const shifted = c.rects.map(([r, col, n, m]) => [r + 1, col + 1, n, m]);
      expect(boxesFor(s, g).shapes[7]).toEqual(shifted);
    });
  });
  it('leaves erased cells out of the walls and unfilled shapes out entirely', () => {
    const g = gridOf(ROWS);
    let s = initEdit(g, OUTLINES, DEFAULTS);
    expect(boxesFor(s, g).shapes).toEqual({ 1: [[1, 1, 2, 2]], 2: [[1, 4, 1, 1]] });
    s = toggleShape(s, 2);
    s = eraseAt(s, g, 4, 5, 1);
    const b = boxesFor(s, g);
    expect(b.shapes).toEqual({ 1: [[1, 1, 2, 2]] });
    expect(b.walls.some(([r0, c0]) => r0 === 4 && c0 === 5)).toBe(false);
  });
});

describe('savePayload', () => {
  it('savePayload shape', () => {
    const g = gridOf(ROWS);
    let s = initEdit(g, OUTLINES, DEFAULTS);
    s = toggleShape(s, 2);
    s = setShapeHeight(s, 1, 0.8);
    s = eraseAt(s, g, 3, 3, 1);
    s = eraseAt(s, g, 0, 0, 1);
    s = setWallHeight(s, 0.5);
    s = selectShape(s, 1);
    const p = savePayload(s, 'tok', 'my_map');
    expect(p).toEqual({
      token: 'tok',
      name: 'my_map',
      wall_height: 0.5,
      shapes: { '1': { fill: true, height: 0.8 }, '2': { fill: false, height: 0.6 } },
      erased: [0, 21],
    });
    expect(p.erased.every(Number.isInteger)).toBe(true);
    expect(JSON.parse(JSON.stringify(p))).toEqual(p);
  });
});
