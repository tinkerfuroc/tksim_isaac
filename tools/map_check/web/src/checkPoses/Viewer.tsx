import { useEffect, useRef } from 'react';
import type { ViewerItem } from './tableLogic';

interface Props {
  items: ViewerItem[];
  index: number;
  onIndex: (i: number) => void;
  onClose: () => void;
  /** False while the Check poses tab is hidden: the keys belong to the other screen then. */
  active: boolean;
}

/** Full-screen image viewer: ← → step through every view, Esc or a click outside closes. */
export function Viewer({ items, index, onIndex, onClose, active }: Props) {
  const closeBtn = useRef<HTMLButtonElement>(null);
  const n = items.length;
  const cur = items[index];
  const step = (d: number) => onIndex((index + d + n) % n);

  // Focus moves in on open and back to what opened it on close.
  useEffect(() => {
    const before = document.activeElement as HTMLElement | null;
    closeBtn.current?.focus();
    return () => before?.focus?.();
  }, []);

  useEffect(() => {
    if (!active) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') { e.preventDefault(); onClose(); }
      else if (e.key === 'ArrowLeft') { e.preventDefault(); step(-1); }
      else if (e.key === 'ArrowRight') { e.preventDefault(); step(1); }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  });

  // Fetch the neighbours so stepping shows them at once.
  useEffect(() => {
    for (const d of [1, -1]) {
      const it = items[(index + d + n) % n];
      if (it) new Image().src = it.url;
    }
  }, [items, index, n]);

  if (!cur) return null;
  return (
    <div className="cp-overlay cp-viewer" role="dialog" aria-modal="true" aria-label="Enlarged view"
         onClick={(e) => { if (e.target === e.currentTarget) onClose(); }}>
      <div className="cp-vbar">
        <span>{cur.title} · {cur.caption}</span>
        <button ref={closeBtn} type="button" className="cp-btn" onClick={onClose}>Close · Esc</button>
      </div>
      <div className="cp-vimg" onClick={(e) => { if (e.target === e.currentTarget) onClose(); }}>
        <img key={cur.url} src={cur.url} alt={`${cur.title}, ${cur.caption}`} />
      </div>
      <div className="cp-vfoot">
        {index + 1} of {n}
        {cur.note && <span className="cp-vnote"> · {cur.note}</span>}
      </div>
      {n > 1 && (
        <>
          <button type="button" className="cp-nav prev" aria-label="Previous view" onClick={() => step(-1)}>‹</button>
          <button type="button" className="cp-nav next" aria-label="Next view" onClick={() => step(1)}>›</button>
        </>
      )}
    </div>
  );
}
