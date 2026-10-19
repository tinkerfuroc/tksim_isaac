import { useEffect, useRef, useState } from 'react';
import { exportList, type BenchRow } from '../api';
import { copyText, downloadText } from './clipboard';
import { duplicateNames } from './tableLogic';
import { errorText } from './useBench';

type Format = 'code' | 'yaml';
type Which = 'all' | 'ticked';

interface Props {
  map: string;
  rows: BenchRow[];
  /** The bench version; the text is fetched again when it changes. */
  version: number;
  /** Sends edits still waiting to go out, so the export reads them. */
  flush: () => Promise<void>;
  onClose: () => void;
  active: boolean;
}

/** "Export list…": both Map Studio formats, everything listed or only the ticked poses. */
export function ExportDialog({ map, rows, version, flush, onClose, active }: Props) {
  const [format, setFormat] = useState<Format>('code');
  const [which, setWhich] = useState<Which>('all');
  const [text, setText] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [note, setNote] = useState<string | null>(null);
  const area = useRef<HTMLTextAreaElement>(null);
  const closeBtn = useRef<HTMLButtonElement>(null);

  useEffect(() => {
    let live = true;
    setNote(null);
    (async () => {
      try {
        await flush();
        const t = await exportList(map, format, which);
        if (live) { setText(t); setError(null); }
      } catch (e) {
        if (live) setError(errorText(e));
      }
    })();
    return () => { live = false; };
  }, [map, format, which, version, flush]);

  useEffect(() => {
    const before = document.activeElement as HTMLElement | null;
    closeBtn.current?.focus();
    return () => before?.focus?.();
  }, []);

  useEffect(() => {
    if (!active) return;
    const onKey = (e: KeyboardEvent) => { if (e.key === 'Escape') { e.preventDefault(); onClose(); } };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [active, onClose]);

  const count = which === 'all' ? rows.length : rows.filter((r) => r.state.included).length;
  const dupes = duplicateNames(rows, which);
  const filename = format === 'code' ? `${map}_poses.py` : `${map}.poses.yaml`;

  const copy = async () => {
    if (await copyText(text)) setNote('Copied');
    else { area.current?.select(); setNote('Press Ctrl+C to copy'); }
  };

  return (
    <div className="cp-overlay" role="dialog" aria-modal="true" aria-label="Export list"
         onClick={(e) => { if (e.target === e.currentTarget) onClose(); }}>
      <div className="cp-vbar">
        <span>Export list · Map Studio format</span>
        <button ref={closeBtn} type="button" className="cp-btn" onClick={onClose}>Close · Esc</button>
      </div>
      <div className="cp-xbody" onClick={(e) => { if (e.target === e.currentTarget) onClose(); }}>
        <div className="cp-card cp-xcard">
          <fieldset className="cp-radios">
            <legend className="cp-lbl">Format</legend>
            <label><input type="radio" name="cp-xf" checked={format === 'code'} onChange={() => setFormat('code')} /> Code (same as "Copy all as code")</label>
            <label><input type="radio" name="cp-xf" checked={format === 'yaml'} onChange={() => setFormat('yaml')} /> .poses.yaml file</label>
          </fieldset>
          <fieldset className="cp-radios">
            <legend className="cp-lbl">Which poses</legend>
            <label><input type="radio" name="cp-xw" checked={which === 'all'} onChange={() => setWhich('all')} /> Everything listed</label>
            <label><input type="radio" name="cp-xw" checked={which === 'ticked'} onChange={() => setWhich('ticked')} /> Only ticked for render</label>
          </fieldset>
          {error
            ? <div className="cp-msg err" role="alert">Couldn't export: {error}</div>
            : <textarea ref={area} className="cp-xtext" readOnly value={text} aria-label="Exported poses" spellCheck={false} />}
          <div className="cp-row">
            <button type="button" className="cp-btn primary" disabled={!!error || !text} onClick={() => void copy()}>Copy</button>
            <button type="button" className="cp-btn" disabled={!!error || !text}
                    onClick={() => downloadText(text, filename, format === 'code' ? 'text/x-python' : 'application/yaml')}
                    title={`Saves ${filename}`}>Download</button>
            <span className="cp-hint" aria-live="polite">
              {note ?? `${count} pose${count === 1 ? '' : 's'}`}
            </span>
          </div>
          {dupes.length > 0 && (
            <div className="cp-msg warn" role="status">
              {dupes.length === 1 ? '1 name clash' : `${dupes.length} name clashes`} ({dupes.join(', ')}):
              rename before importing, or the later one wins.
            </div>
          )}
        </div>
      </div>
      <div className="cp-vfoot">Pasted names are exported without the POS_ prefix, lowercased, so they paste back into Map Studio unchanged.</div>
    </div>
  );
}
