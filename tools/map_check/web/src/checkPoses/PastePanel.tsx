import { useState } from 'react';
import type { BenchBody } from '../api';

const BLOCK = /\bPOS_[A-Za-z0-9_]+\s*=\s*PoseStamped\s*\(/g;

interface Props {
  disabled: boolean;
  /** Sends add_pasted; resolves to the server's answer (with paste_errors), or null on failure. */
  onAdd: (text: string) => Promise<BenchBody | null>;
}

type Result = { tone: 'ok' | 'err'; text: string; lines: string[] };

const plural = (n: number, one: string, many = `${one}s`) => `${n} ${n === 1 ? one : many}`;

/** Map Studio's "Copy all as code" text from another machine → pasted rows. */
export function PastePanel({ disabled, onAdd }: Props) {
  const [text, setText] = useState('');
  const [busy, setBusy] = useState(false);
  const [result, setResult] = useState<Result | null>(null);

  const add = async () => {
    if (!text.trim() || busy || disabled) return;
    const blocks = (text.match(BLOCK) ?? []).length;
    if (!blocks) {
      setResult({ tone: 'err', text: 'No POS_… = PoseStamped(…) blocks found. Paste Map Studio\'s "Copy all as code" text.', lines: [] });
      return;
    }
    setBusy(true);
    try {
      const b = await onAdd(text);
      if (!b) return; // the screen shows the error
      const errors = b.paste_errors ?? [];
      const added = blocks - errors.length;
      if (!errors.length) {
        setResult({ tone: 'ok', text: `Added ${plural(added, 'pose')}.`, lines: [] });
        setText('');
      } else {
        setResult({
          tone: 'err',
          text: added ? `Added ${added} of ${blocks} blocks. Fix these and add them again:` : 'Nothing added:',
          lines: errors.map((e) => `block ${e.index}: ${e.reason.replace(/\.\.\./g, '…')}`),
        });
      }
    } finally {
      setBusy(false);
    }
  };

  return (
    <aside className="cp-side">
      <label className="cp-lbl" htmlFor="cp-paste">Paste from Map Studio on another machine</label>
      <textarea id="cp-paste" value={text} spellCheck={false} placeholder="POS_… = PoseStamped(…)"
                onChange={(e) => { setText(e.target.value); setResult(null); }}
                onKeyDown={(e) => { if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) { e.preventDefault(); void add(); } }} />
      <div className="cp-row">
        <button type="button" className="cp-btn" disabled={disabled || busy || !text.trim()} onClick={() => void add()}>
          {busy ? 'Adding…' : 'Add to list'}
        </button>
        {text.trim() && <span className="cp-hint">Ctrl+Enter</span>}
      </div>
      {result && (
        <div className={`cp-msg ${result.tone}`} role="status">
          <span>{result.text}</span>
          {result.lines.length > 0 && <ul>{result.lines.map((l) => <li key={l}>{l}</li>)}</ul>}
        </div>
      )}
      <span className="cp-hint">Reads x, y and yaw from each POS_… block. Pasted poses stay until you clear them.</span>
    </aside>
  );
}
