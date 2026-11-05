import { describe, expect, it } from 'vitest';
import golden from '../../tests/fixtures/rects_golden.json';
import { mergeRects } from './rects';

type Case = { mask: number[][]; rects: [number, number, number, number][] };

describe('mergeRects', () => {
  const cases = golden as Case[];
  it('has golden cases', () => expect(cases.length).toBeGreaterThan(0));
  cases.forEach((c, i) => {
    it(`matches the Python rects for golden mask ${i}`, () => {
      const H = c.mask.length;
      const W = H ? c.mask[0].length : 0;
      const flat = Uint8Array.from(c.mask.flat());
      expect(mergeRects(flat, W, H)).toEqual(c.rects);
    });
  });
  it('returns nothing for an empty mask', () => expect(mergeRects(new Uint8Array(0), 0, 0)).toEqual([]));
});
