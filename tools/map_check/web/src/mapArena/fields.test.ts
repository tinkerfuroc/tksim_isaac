import { describe, expect, it } from 'vitest';
import { fixFieldsMessage, parseInRange } from './fields';

describe('parseInRange', () => {
  it('accepts numbers within the bounds, inclusive', () => {
    expect(parseInRange('0.6', 0.05, 10)).toBe(0.6);
    expect(parseInRange(' 10 ', 0.05, 10)).toBe(10);
    expect(parseInRange('0.05', 0.05, 10)).toBe(0.05);
  });
  it('refuses empty, partial, non-numeric and out-of-range text', () => {
    for (const t of ['', ' ', '-', '.', 'abc', '0', '10.01', 'NaN', 'Infinity']) {
      expect(parseInRange(t, 0.05, 10)).toBeNull();
    }
  });
});

describe('fixFieldsMessage', () => {
  it('is null when every field is valid', () => expect(fixFieldsMessage([])).toBeNull());
  it('names the fields to fix', () => {
    expect(fixFieldsMessage(['wall height'])).toBe('Fix the wall height before saving.');
    expect(fixFieldsMessage(['wall height', 'shape 3 height', 'brush size']))
      .toBe('Fix the wall height, shape 3 height and brush size before saving.');
  });
});
