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
}

/** Closed outlines with a fill checkbox and a height each. Click a row to highlight the shape. */
export function ShapesList({ outlines, shapes, selected, onToggle, onHeight, onSelect }: Props) {
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
            return (
              <tr key={o.id} data-id={o.id} className={selected === o.id ? 'sel' : undefined}
                  aria-selected={selected === o.id}
                  onClick={() => onSelect(selected === o.id ? null : o.id)}>
                <td>
                  <input type="checkbox" checked={s.fill} aria-label={`Fill shape ${o.id}`}
                         onClick={(e) => e.stopPropagation()} onChange={() => onToggle(o.id)} />
                </td>
                <td className="name" title={`centre x ${o.cx.toFixed(2)}, y ${o.cy.toFixed(2)}`}>
                  shape {o.id}<br />
                  <span className="xy">{o.cx.toFixed(1)}, {o.cy.toFixed(1)}</span>
                </td>
                <td className="num">{o.area_m2.toFixed(2)}</td>
                <td>
                  <NumberField value={s.height} onChange={(h) => onHeight(o.id, h)}
                               min={MIN_HEIGHT_M} max={MAX_HEIGHT_M} step={0.05} digits={2}
                               disabled={!s.fill} label={`Height of shape ${o.id}`} />
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}
