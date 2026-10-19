import { useEffect, useRef } from 'react';
import type { Outline, ShapeSetting } from '../api';
import { NumberField } from './NumberField';

export const MIN_HEIGHT_M = 0.05;
export const MAX_HEIGHT_M = 10; // server.MAX_HEIGHT_M

interface Props {
  outlines: Outline[];
  shapes: Record<number, ShapeSetting>;
  selected: number | null;
  onToggle: (id: number) => void;
  onHeight: (id: number, h: number) => void;
  onSelect: (id: number | null) => void;
  onValidity: (field: string, valid: boolean) => void;
}

/**
 * Closed outlines with a fill checkbox and a height each. Each row's name is a toggle button
 * (keyboard: Tab to it, Space/Enter) that highlights the shape; a click anywhere on the row
 * does the same for the mouse.
 */
export function ShapesList({ outlines, shapes, selected, onToggle, onHeight, onSelect, onValidity }: Props) {
  const box = useRef<HTMLDivElement>(null);

  // A shape picked on the 2D map scrolls its row into view.
  useEffect(() => {
    if (selected === null || !box.current) return;
    box.current.querySelector(`tr[data-id="${selected}"]`)?.scrollIntoView({ block: 'nearest' });
  }, [selected]);

  return (
    <div className="ma-shapes" ref={box}>
      <table>
        <thead>
          <tr><th>fill</th><th>shape · x, y</th><th>m²</th><th>height m</th></tr>
        </thead>
        <tbody>
          {outlines.length === 0 && (
            <tr><td colSpan={4} className="empty">This map has no closed outlines.</td></tr>
          )}
          {outlines.map((o) => {
            const s = shapes[o.id];
            const on = selected === o.id;
            const pick = () => onSelect(on ? null : o.id);
            return (
              <tr key={o.id} data-id={o.id} className={on ? 'sel' : undefined} onClick={pick}>
                <td>
                  <input type="checkbox" checked={s.fill} aria-label={`Fill shape ${o.id}`}
                         onClick={(e) => e.stopPropagation()} onChange={() => onToggle(o.id)} />
                </td>
                <td className="name">
                  <button type="button" className="pick" aria-pressed={on}
                          title={`centre x ${o.cx.toFixed(2)}, y ${o.cy.toFixed(2)}`}
                          aria-label={`Highlight shape ${o.id} at x ${o.cx.toFixed(1)}, y ${o.cy.toFixed(1)}`}
                          onClick={(e) => { e.stopPropagation(); pick(); }}>
                    shape {o.id}<br />
                    <span className="xy">{o.cx.toFixed(1)}, {o.cy.toFixed(1)}</span>
                  </button>
                </td>
                <td className="num">{o.area_m2.toFixed(2)}</td>
                <td>
                  <NumberField value={s.height} onChange={(h) => onHeight(o.id, h)}
                               min={MIN_HEIGHT_M} max={MAX_HEIGHT_M} step={0.05} digits={2}
                               disabled={!s.fill} label={`Height of shape ${o.id}`}
                               field={`shape ${o.id} height`} onValidity={onValidity} />
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}
