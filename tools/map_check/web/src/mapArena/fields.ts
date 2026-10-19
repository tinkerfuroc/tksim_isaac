// Number-field rules shared by NumberField and the Save step.

/** The number in `text` when it lies within [min, max]; null for anything else (empty, partial, out of range). */
export function parseInRange(text: string, min: number, max: number): number | null {
  if (text.trim() === '') return null;
  const v = Number(text);
  return Number.isFinite(v) && v >= min && v <= max ? v : null;
}

/** "Fix the wall height and shape 3 height before saving." or null when nothing needs fixing. */
export function fixFieldsMessage(fields: string[]): string | null {
  if (!fields.length) return null;
  const list = fields.length === 1 ? fields[0]
    : `${fields.slice(0, -1).join(', ')} and ${fields[fields.length - 1]}`;
  return `Fix the ${list} before saving.`;
}
