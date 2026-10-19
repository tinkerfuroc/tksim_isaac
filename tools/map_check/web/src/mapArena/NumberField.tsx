import { useEffect, useState } from 'react';

interface Props {
  value: number;
  onChange: (v: number) => void;
  min: number;
  max: number;
  step: number;
  digits: number;
  id?: string;
  label?: string; // aria-label when there is no <label for>
  disabled?: boolean;
  onEnter?: () => void;
}

/**
 * A number input that lets you type freely: the value is passed on only while the text is a
 * number within [min, max]. Anything else is outlined in red and snaps back on blur.
 */
export function NumberField({ value, onChange, min, max, step, digits, id, label, disabled, onEnter }: Props) {
  const [text, setText] = useState(value.toFixed(digits));
  const parsed = text.trim() === '' ? NaN : Number(text);
  const valid = Number.isFinite(parsed) && parsed >= min && parsed <= max;

  // Follow outside changes (e.g. "default shape height" rewriting every row).
  useEffect(() => {
    setText((t) => (Number(t) === value && t.trim() !== '' ? t : value.toFixed(digits)));
  }, [value, digits]);

  return (
    <input
      type="number" id={id} aria-label={label} disabled={disabled}
      min={min} max={max} step={step} value={text}
      className={valid ? undefined : 'bad'}
      aria-invalid={!valid}
      title={valid ? undefined : `Enter a number from ${min} to ${max}`}
      onChange={(e) => {
        setText(e.target.value);
        const v = Number(e.target.value);
        if (e.target.value.trim() !== '' && Number.isFinite(v) && v >= min && v <= max) onChange(v);
      }}
      onBlur={() => setText(value.toFixed(digits))}
      onKeyDown={(e) => { if (e.key === 'Enter' && onEnter) onEnter(); }}
      onClick={(e) => e.stopPropagation()}
    />
  );
}
