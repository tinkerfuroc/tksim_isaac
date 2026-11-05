import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { ApiError, benchOps, getBench, type BenchBody, type BenchOp, type JobStatus } from '../api';
import { subscribe } from '../events';
import { applyOps } from './tableLogic';

const DEBOUNCE_MS = 400; // typed angles go to the server once typing pauses
const MAX_CATCH_UP = 3; // refetches in a row while chasing a newer version

export function errorText(e: unknown): string {
  if (e instanceof ApiError) return e.detail;
  if (e instanceof TypeError) return `Couldn't reach Map Check: ${e.message}`;
  return e instanceof Error ? e.message : String(e);
}

interface Entry {
  id: number;
  map: string;
  ops: BenchOp[];
  key?: string; // debounce key: a newer edit of the same field replaces an unsent one
  timer?: ReturnType<typeof setTimeout>;
  promise?: Promise<BenchBody | null>;
}

/**
 * The shared bench for one map. The server's answer is the truth; edits still waiting for it
 * are drawn on top (optimistic). SSE `bench` events refetch once per newer version, `job`
 * events update the job line directly.
 */
export function useBench(map: string | null) {
  const [server, setServer] = useState<BenchBody | null>(null);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [opError, setOpError] = useState<string | null>(null);
  const [pending, setPending] = useState<Entry[]>([]);
  const mapRef = useRef(map);
  const serverRef = useRef<BenchBody | null>(null);
  const pendingRef = useRef<Entry[]>([]);
  const nextId = useRef(1);
  const inflight = useRef(false);
  const wanted = useRef(-1); // newest version announced by SSE

  mapRef.current = map;

  const accept = useCallback((b: BenchBody, forMap: string) => {
    if (forMap !== mapRef.current) return;
    const cur = serverRef.current;
    if (cur && cur.map === b.map && b.version < cur.version) return; // an older answer, overtaken
    serverRef.current = b;
    setServer(b);
    setLoadError(null);
  }, []);

  const refresh = useCallback(async () => {
    const m = mapRef.current;
    if (!m || inflight.current) return;
    inflight.current = true;
    try {
      for (let i = 0; i < MAX_CATCH_UP; i++) {
        const b = await getBench(m);
        if (m !== mapRef.current) return;
        accept(b, m);
        if (wanted.current <= b.version) break;
      }
    } catch (e) {
      if (m === mapRef.current) setLoadError(errorText(e));
    } finally {
      inflight.current = false;
    }
  }, [accept]);

  const setEntries = (f: (es: Entry[]) => Entry[]) => {
    pendingRef.current = f(pendingRef.current);
    setPending(pendingRef.current);
  };

  const flush = useCallback((e: Entry): Promise<BenchBody | null> => {
    if (e.timer !== undefined) { clearTimeout(e.timer); e.timer = undefined; }
    if (e.promise) return e.promise;
    e.promise = (async () => {
      try {
        const b = await benchOps(e.map, e.ops);
        accept(b, e.map);
        return b;
      } catch (err) {
        if (e.map === mapRef.current) {
          setOpError(errorText(err));
          void refresh(); // show what the server really has
        }
        return null;
      } finally {
        setEntries((es) => es.filter((x) => x !== e));
      }
    })();
    return e.promise;
  }, [accept, refresh]);

  /**
   * Send ops now and resolve to the server's answer, or (with `debounceKey`) once edits to that
   * field pause; a debounced send resolves to null at once, its edit shows in the meantime.
   */
  const send = useCallback((ops: BenchOp[], debounceKey?: string): Promise<BenchBody | null> => {
    const m = mapRef.current;
    if (!m || !ops.length) return Promise.resolve(null);
    setOpError(null);
    if (!debounceKey) {
      const entry: Entry = { id: nextId.current++, map: m, ops };
      setEntries((es) => [...es, entry]);
      return flush(entry);
    }
    let entry = pendingRef.current.find((e) => e.key === debounceKey && e.map === m && !e.promise);
    if (entry) {
      clearTimeout(entry.timer);
      const old = entry;
      entry = { ...old, ops };
      setEntries((es) => es.map((x) => (x === old ? entry! : x)));
    } else {
      entry = { id: nextId.current++, map: m, ops, key: debounceKey };
      const added = entry;
      setEntries((es) => [...es, added]);
    }
    const e = entry;
    e.timer = setTimeout(() => { void flush(e); }, DEBOUNCE_MS);
    return Promise.resolve(null);
  }, [flush]);

  /** Send every waiting edit now (before a render or an export reads the bench). */
  const flushAll = useCallback(async () => {
    await Promise.all(pendingRef.current.map(flush));
  }, [flush]);

  const setJob = useCallback((job: JobStatus) => {
    const cur = serverRef.current;
    if (!cur || job.map !== cur.map) return;
    serverRef.current = { ...cur, job };
    setServer(serverRef.current);
  }, []);

  // Open a map: load it and follow its live updates.
  useEffect(() => {
    serverRef.current = null;
    setServer(null);
    setLoadError(null);
    setOpError(null);
    wanted.current = -1;
    inflight.current = false;
    void flushAll(); // edits typed into the previous map still go to that map
    if (!map) return;
    void refresh();
    return subscribe(map, (v) => {
      wanted.current = Math.max(wanted.current, v);
      const cur = serverRef.current;
      if (!cur || v > cur.version) void refresh();
    }, setJob);
  }, [map, refresh, setJob, flushAll]);

  const bench = useMemo(() => {
    if (!server) return null;
    return applyOps(server, pending.filter((e) => e.map === server.map).flatMap((e) => e.ops));
  }, [server, pending]);

  return { bench, loadError, opError, clearOpError: () => setOpError(null), send, flushAll, refresh, setJob };
}
