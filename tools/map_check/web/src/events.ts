import type { JobStatus } from './api';

const FIRST_DELAY_MS = 1000;
const MAX_DELAY_MS = 10000;

/** Live updates for one map. Reconnects with backoff (1 s, doubling, capped at 10 s). Returns a close function. */
export function subscribe(
  map: string,
  onBench: (version: number) => void,
  onJob: (status: JobStatus) => void,
): () => void {
  let es: EventSource | null = null;
  let timer: ReturnType<typeof setTimeout> | undefined;
  let delay = FIRST_DELAY_MS;
  let closed = false;

  const open = () => {
    es = new EventSource(`/api/events?map=${encodeURIComponent(map)}`);
    es.addEventListener('open', () => { delay = FIRST_DELAY_MS; });
    es.addEventListener('bench', (e) => {
      const v = Number((e as MessageEvent).data);
      if (Number.isFinite(v)) onBench(v);
    });
    es.addEventListener('job', (e) => {
      try { onJob(JSON.parse((e as MessageEvent).data) as JobStatus); } catch { /* ignore a bad frame */ }
    });
    es.addEventListener('error', () => {
      es?.close();
      es = null;
      if (closed) return;
      timer = setTimeout(open, delay);
      delay = Math.min(delay * 2, MAX_DELAY_MS);
    });
  };
  open();

  return () => {
    closed = true;
    if (timer !== undefined) clearTimeout(timer);
    es?.close();
    es = null;
  };
}
