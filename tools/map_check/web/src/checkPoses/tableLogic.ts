// Pure rules behind the Check poses table, render bar and viewer.
import type { BenchBody, BenchOp, BenchRow, Flag, JobStatus, LastRender, RowState } from '../api';

export type Col = 'included' | 'behind' | 'top' | 'head';
export type HeaderState = 'all' | 'none' | 'some' | 'disabled';
export const VIEW_COLS: Col[] = ['behind', 'top', 'head'];

/** The rows whose name contains `filter`, ignoring case and surrounding spaces. */
export function visibleRows(rows: BenchRow[], filter: string): BenchRow[] {
  const f = filter.trim().toLowerCase();
  return f ? rows.filter((r) => r.pose.name.toLowerCase().includes(f)) : rows;
}

/** The rows a column header acts on: every shown row for Render, shown included rows for a view. */
function pool(rows: BenchRow[], col: Col): BenchRow[] {
  return col === 'included' ? rows : rows.filter((r) => r.state.included);
}

/** `rows` are the rows shown (after the filter). */
export function headerState(rows: BenchRow[], col: Col): HeaderState {
  const p = pool(rows, col);
  if (!p.length) return 'disabled';
  const on = p.filter((r) => r.state[col]).length;
  return on === p.length ? 'all' : on === 0 ? 'none' : 'some';
}

/** A header click: all on → all off; otherwise all on. Acts only on `rows` (the shown rows). */
export function headerToggleOps(rows: BenchRow[], col: Col): BenchOp[] {
  const p = pool(rows, col);
  if (!p.length) return [];
  return [{ op: 'set_rows', rows: p.map((r) => r.key), fields: { [col]: headerState(rows, col) !== 'all' } }];
}

/** Shift-click on a Render box: every shown row from the anchor to the clicked row, either way. */
export function rangeOps(rows: BenchRow[], fromKey: string, toKey: string, value: boolean): BenchOp[] {
  const a = rows.findIndex((r) => r.key === fromKey);
  const b = rows.findIndex((r) => r.key === toKey);
  if (b < 0) return [];
  const span = a < 0 ? [rows[b]] : rows.slice(Math.min(a, b), Math.max(a, b) + 1);
  return [{ op: 'set_rows', rows: span.map((r) => r.key), fields: { included: value } }];
}

/** The head-cam header's "set": these angles on every shown, included row with Head cam on. */
export function panTiltOps(rows: BenchRow[], pan: number, tilt: number): BenchOp[] {
  const keys = rows.filter((r) => r.state.included && r.state.head).map((r) => r.key);
  return keys.length ? [{ op: 'set_rows', rows: keys, fields: { pan_deg: pan, tilt_deg: tilt } }] : [];
}

/** What the next render costs: views include the overview once; time is 40 + 3 s per view. */
export function summary(rows: BenchRow[]): { poses: number; views: number; seconds: number } {
  const inc = rows.filter((r) => r.state.included);
  const views = inc.reduce((n, r) => n + +r.state.behind + +r.state.top + +r.state.head, 0) + (inc.length ? 1 : 0);
  return { poses: inc.length, views, seconds: 40 + 3 * views };
}

export function estimateText(seconds: number): string {
  return seconds >= 120 ? `about ${seconds} s (${Math.round(seconds / 60)} min)` : `about ${seconds} s`;
}

/**
 * The bench as it will be once `ops` land (optimistic display). Ops the page can't predict
 * (add_pasted, restore_saved) are left to the server's answer. Never mutates `bench`.
 */
export function applyOps(bench: BenchBody, ops: BenchOp[]): BenchBody {
  if (!ops.length) return bench;
  let { rows, removed_count, arm_pose } = bench;
  const set = (keys: string[], fields: Partial<RowState>) => {
    const ks = new Set(keys);
    rows = rows.map((r) => (ks.has(r.key) ? { ...r, state: { ...r.state, ...fields } } : r));
  };
  for (const op of ops) {
    switch (op.op) {
      case 'set_row': set([op.row], op.fields); break;
      case 'set_rows': set(op.rows, op.fields); break;
      case 'remove': {
        const gone = rows.find((r) => r.key === op.row);
        rows = rows.filter((r) => r.key !== op.row);
        if (gone?.group === 'saved') removed_count += 1;
        break;
      }
      case 'clear_pasted': rows = rows.filter((r) => r.group !== 'pasted'); break;
      case 'set_arm': arm_pose = op.name; break;
      default: break;
    }
  }
  return { ...bench, rows, removed_count, arm_pose };
}

/** Map Studio's name for a pose: without POS_, lowercased (poses.key on the server). */
export const poseKey = (name: string) => (name.startsWith('POS_') ? name.slice(4) : name).toLowerCase();

/**
 * Same-name distances for both rows of each pair. The server gives it on the pasted row only;
 * the Map Studio row with the same name gets the same number.
 */
export function clashByKey(rows: BenchRow[]): Map<string, number> {
  const out = new Map<string, number>();
  const byName = new Map<string, number>();
  for (const r of rows) {
    if (r.group === 'pasted' && r.clash_m != null) {
      out.set(r.key, r.clash_m);
      byName.set(poseKey(r.pose.name), r.clash_m);
    }
  }
  for (const r of rows) {
    const d = r.group === 'saved' ? byName.get(poseKey(r.pose.name)) : undefined;
    if (d !== undefined) out.set(r.key, d);
  }
  return out;
}

export function clashText(m: number): string {
  const cm = Math.round(m * 100);
  return cm < 1 ? 'same name · under 1 cm apart' : `same name · ${cm} cm apart`;
}

/** Export names that appear more than once (Map Studio keeps the later one). */
export function duplicateNames(rows: BenchRow[], which: 'all' | 'ticked'): string[] {
  const seen = new Set<string>();
  const dup = new Set<string>();
  for (const r of rows) {
    if (which === 'ticked' && !r.state.included) continue;
    const k = poseKey(r.pose.name);
    if (seen.has(k)) dup.add(k); else seen.add(k);
  }
  return [...dup];
}

/** "near" (inside the margin, not touching) is amber; a real overlap or leaving the arena is red. */
export const flagTone = (f: Flag): 'bad' | 'near' => (f.kind === 'near' ? 'near' : 'bad');

/** The render bar's job line; null when there is nothing to say (no job, or it finished fine). */
export function jobText(job: JobStatus | null): { text: string; tone: 'run' | 'bad' } | null {
  if (!job) return null;
  switch (job.status) {
    case 'running': return { text: `Rendering · ${job.done_views} of ${job.total_views} views`, tone: 'run' };
    case 'queued': return { text: `Queued · position ${job.position ?? 1}`, tone: 'run' };
    case 'failed': case 'interrupted': case 'timeout':
      return { text: `Render ${job.status}${job.message ? `: ${job.message}` : ''}`, tone: 'bad' };
    default: return null;
  }
}

export interface ViewerItem { url: string; title: string; caption: string; note: string; key: string }

/** Every image of a render in strip order: the overview, then each pose's views. */
export function viewerItems(last: LastRender | null): ViewerItem[] {
  if (!last) return [];
  const out: ViewerItem[] = [];
  if (last.overview_url) {
    out.push({ url: last.overview_url, title: 'Overview', caption: 'top-down, rendered poses labeled', note: '', key: 'overview' });
  }
  for (const it of last.items) {
    const note = it.flags.map((f) => f.text).join(' · ');
    for (const v of it.views) out.push({ url: v.url, title: it.name, caption: v.caption, note, key: `${it.key}/${v.view}` });
  }
  return out;
}
