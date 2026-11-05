import { useEffect, useState } from 'react';
import { parseInRange } from './fields';

interface Props {
  value: number;
  onChange: (v: number) => void;
  min: number;
  max: number;
  step: number;
  digits: number;
  /** What the teammate calls this field, e.g. "wall height"; used in "Fix the … before saving". */
  field: string;
  /** Told whenever the field turns valid or invalid, and that it is valid again on unmount. */
  onValidity?: (field: string, valid: boolean) => void;
  id?: string;
  label?: string; // aria-label when there is no <label for>
  disabled?: boolean;
  onEnter?: () => void;
}

/**
 * A number input that lets you type freely: the value is passed on only while the text is a
 * number within [min, max]. Anything else stays as typed, outlined in red, and is reported
 * through onValidity so Save can wait until it is fixed.
 */
export function NumberField({
  value, onChange, min, max, step, digits, field, onValidity, id, label, disabled, onEnter,
}: Props) {
  const [text, setText] = useState(value.toFixed(digits));
  const valid = parseInRange(text, min, max) !== null;

  // Follow outside changes (e.g. "default shape height" rewriting every row).
  useEffect(() => {
    setText((t) => (parseInRange(t, -Infinity, Infinity) === value ? t : value.toFixed(digits)));
  }, [value, digits]);

  useEffect(() => {
    onValidity?.(field, valid);
  }, [field, valid, onValidity]);
  useEffect(() => () => onValidity?.(field, true), [field, onValidity]);

  return (
    <input
      type="number" id={id} aria-label={label} disabled={disabled}
      min={min} max={max} step={step} value={text}
      className={valid ? undefined : 'bad'}
      aria-invalid={!valid}
      title={valid ? undefined : `Enter a number from ${min} to ${max}`}
      onChange={(e) => {
        setText(e.target.value);
        const v = parseInRange(e.target.value, min, max);
        if (v !== null) onChange(v);
      }}
      onBlur={() => { if (valid) setText(value.toFixed(digits)); }}
      onKeyDown={(e) => { if (e.key === 'Enter' && onEnter && valid) onEnter(); }}
      onClick={(e) => e.stopPropagation()}
    />
  );
}
