import { describe, expect, it } from 'vitest';
import { nameProblem } from './names';

describe('nameProblem (Map Studio map name rules)', () => {
  it('accepts map names Map Studio accepts', () => {
    for (const n of ['1104', '0406.map', 'robocup_home_2026_measured_5cm', 'a-b.c', 'x'.repeat(100)]) {
      expect(nameProblem(n)).toBeNull();
    }
  });
  it('refuses the rest with a reason', () => {
    for (const n of ['', '.hidden', '-x', 'a..b', 'room.poses', 'has space', 'x'.repeat(101), 'a/b']) {
      expect(nameProblem(n)).toMatch(/./);
    }
  });
});
