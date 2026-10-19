import type { GpuStatus, JobStatus } from '../api';
import { estimateText, jobText } from './tableLogic';

interface Props {
  poses: number;
  views: number;
  seconds: number;
  gpu: GpuStatus | null;
  gpuError: string | null;
  /** busy_text from the last refused render (409); the button then reads "Render anyway…". */
  busyText: string | null;
  confirming: boolean;
  starting: boolean;
  /** Why Render can't be used right now (beyond "nothing ticked"), or null. */
  blocked: string | null;
  job: JobStatus | null;
  error: string | null;
  onRender: () => void;
  onConfirm: () => void;
  onCancel: () => void;
}

/** Sticky bottom bar: what the next render costs, the GPU, the button and the running job. */
export function RenderBar(p: Props) {
  const busy = p.busyText ?? (p.gpu?.busy ? p.gpu.text : null);
  const job = jobText(p.job);
  const running = p.job?.status === 'running';
  const pct = running && p.job!.total_views ? Math.round((100 * p.job!.done_views) / p.job!.total_views) : 0;
  const label = p.starting ? 'Starting…'
    : !p.poses ? 'Render'
    : busy ? 'Render anyway…'
    : `Render ${p.poses} pose${p.poses === 1 ? '' : 's'}`;

  return (
    <footer className="cp-bar">
      <div className="cp-summary" aria-live="polite">
        {p.poses
          ? <>Next render: <b>{p.poses}</b> pose{p.poses === 1 ? '' : 's'} · <b>{p.views}</b> views incl. overview · {estimateText(p.seconds)}</>
          : 'Tick poses to render'}
      </div>
      {p.gpuError && !busy
        ? <span className="cp-gpu unknown" title={p.gpuError}><i />GPU status unknown</span>
        : busy
          ? <span className="cp-gpu busy" title={busy}><i />{busy}</span>
          : p.gpu ? <span className="cp-gpu free"><i />GPUs free</span> : null}
      {job && (
        <span className={`cp-job ${job.tone}`} role="status">
          {running && <span className="cp-prog" aria-hidden="true"><span style={{ width: `${pct}%` }} /></span>}
          {job.text}
        </span>
      )}
      {p.error && <span className="cp-bar-err" role="alert">{p.error}</span>}
      <span className="cp-spacer" />
      {p.blocked && <span className="cp-hint">{p.blocked}</span>}
      {p.confirming ? (
        <span className="cp-confirm" role="alertdialog" aria-label="Render anyway?">
          <span>The GPU is in use: <b>{p.busyText}</b>. Render anyway?</span>
          <button type="button" className="cp-btn primary warn" disabled={p.starting} onClick={p.onConfirm}>
            {p.starting ? 'Starting…' : 'Yes, render anyway'}
          </button>
          <button type="button" className="cp-btn" autoFocus disabled={p.starting} onClick={p.onCancel}>Cancel</button>
        </span>
      ) : (
        <button type="button" className={busy ? 'cp-btn primary warn' : 'cp-btn primary'}
                disabled={!p.poses || p.starting || !!p.blocked} onClick={p.onRender}>
          {label}
        </button>
      )}
    </footer>
  );
}
