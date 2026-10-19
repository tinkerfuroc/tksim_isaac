import { describe, expect, it } from 'vitest';
import { fitView, pointToCell } from './view2d';

// The map region shown: rows 10..40, cols 20..80 (30 x 60 cells).
const VIEW: [number, number, number, number] = [10, 20, 40, 80];

describe('fitView', () => {
  it('fills the stage width or height, less the padding, keeping square cells', () => {
    // 1000 x 400 stage, 16 px padding: height-limited, 368 / 30 cells = 12.27 px per cell
    const f = fitView(1000, 400, VIEW, 16)!;
    expect(f.cell).toBeCloseTo(368 / 30, 9);
    expect(f.h).toBeCloseTo(368, 9);
    expect(f.w).toBeCloseTo(60 * 368 / 30, 9);
    expect(f.left).toBeCloseTo((1000 - f.w) / 2, 9);
    expect(f.top).toBeCloseTo(16, 9);
    // a tall stage is width-limited
    const t = fitView(550, 800, VIEW, 16)!;
    expect(t.w).toBeCloseTo(518, 9);
    expect(t.left).toBeCloseTo(16, 9);
  });
  it('is null for a stage with no room (a hidden tab)', () => {
    expect(fitView(0, 0, VIEW, 16)).toBeNull();
    expect(fitView(20, 500, VIEW, 16)).toBeNull();
  });
});

describe('pointToCell', () => {
  const f = { left: 100, top: 50, w: 600, h: 300, cell: 10 };
  it('maps a point on the canvas to the map cell under it', () => {
    expect(pointToCell(f, VIEW, 100, 50, false)).toEqual([10, 20]); // top-left corner
    expect(pointToCell(f, VIEW, 105, 55, false)).toEqual([10, 20]);
    expect(pointToCell(f, VIEW, 115, 65, false)).toEqual([11, 21]);
    expect(pointToCell(f, VIEW, 699.9, 349.9, false)).toEqual([39, 79]); // bottom-right cell
  });
  it('is null off the canvas, or clamped to the edge cell when asked', () => {
    expect(pointToCell(f, VIEW, 99, 60, false)).toBeNull();
    expect(pointToCell(f, VIEW, 700, 60, false)).toBeNull();
    expect(pointToCell(f, VIEW, 0, 0, true)).toEqual([10, 20]);
    expect(pointToCell(f, VIEW, 5000, 5000, true)).toEqual([39, 79]);
  });
});
