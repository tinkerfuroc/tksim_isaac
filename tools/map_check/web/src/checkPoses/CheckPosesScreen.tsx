import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { ApiError, getGpu, getMaps, startRender, type GpuStatus, type MapsList } from '../api';
import { ExportDialog } from './ExportDialog';
import { PastePanel } from './PastePanel';
import { PoseTable } from './PoseTable';
import { RenderBar } from './RenderBar';
import { ResultsStrip } from './ResultsStrip';
import { arenaMaps, clashByKey, mapToShow, summary, viewerItems, visibleRows } from './tableLogic';
import { errorText, useBench } from './useBench';
import { Viewer } from './Viewer';
import './checkPoses.css';

const MAP_KEY = 'map_check.map'; // the map is per browser; the bench behind it is shared
const GPU_POLL_MS = 10000;
const OVERLAP_MARGIN_CM = 2; // settings.overlap_margin (Global Constraints: 0.02 m)

function readStoredMap(): string | null {
  try { return window.localStorage.getItem(MAP_KEY); } catch { return null; }
}
function storeMap(name: string) {
  try { window.localStorage.setItem(MAP_KEY, name); } catch { /* private window: per tab only */ }
}

/** The arm pose's joint values, for the select's tooltip. */
const joints = (v: number[] | undefined) => (v ? `joints 1–${v.length}: ${v.join(', ')}°` : undefined);

/** `active` is false while another tab is shown; the screen stays mounted, its keys stay off. */
export function CheckPosesScreen({ active = true }: { active?: boolean }) {
  const [maps, setMaps] = useState<MapsList | null>(null);
  const [mapsError, setMapsError] = useState<string | null>(null);
  const [map, setMap] = useState<string | null>(readStoredMap);
  const [filter, setFilter] = useState('');
  const [viewer, setViewer] = useState<number | null>(null);
  const [exporting, setExporting] = useState(false);
  const [gpu, setGpu] = useState<GpuStatus | null>(null);
  const [gpuError, setGpuError] = useState<string | null>(null);
  const [busyText, setBusyText] = useState<string | null>(null); // from a 409
  const [confirming, setConfirming] = useState(false);
  const [starting, setStarting] = useState(false);
  const [renderError, setRenderError] = useState<string | null>(null);
  // Only maps with a Map Check arena are listed (spec, Ruling 9).
  const choices = useMemo(() => arenaMaps(maps), [maps]);
  // Load a bench only for a map the list confirms (a stored name may be gone).
  const known = maps && map && choices.includes(map) ? map : null;
  const { bench, loadError, opError, clearOpError, send, flushAll, refresh, setJob } = useBench(known);
  const gpuSeq = useRef(0);

  const refreshMaps = useCallback(() => {
    getMaps().then((m) => { setMaps(m); setMapsError(null); }).catch((e) => setMapsError(errorText(e)));
  }, []);

  const refreshGpu = useCallback(() => {
    const seq = ++gpuSeq.current;
    getGpu().then((g) => {
      if (seq !== gpuSeq.current) return;
      setGpu(g);
      setGpuError(null);
      if (!g.busy) { setBusyText(null); setConfirming(false); } // the override is per click; nothing to override now
    }).catch((e) => { if (seq === gpuSeq.current) setGpuError(errorText(e)); });
  }, []);

  // Coming back to this tab: arenas may have been built meanwhile, the GPU may have changed.
  useEffect(() => {
    if (!active) return;
    refreshMaps();
    void refresh();
    refreshGpu();
    // No polling from a hidden browser tab; catch up as soon as it is shown again.
    const t = setInterval(() => { if (!document.hidden) refreshGpu(); }, GPU_POLL_MS);
    const onVisible = () => { if (!document.hidden) refreshGpu(); };
    document.addEventListener('visibilitychange', onVisible);
    return () => {
      clearInterval(t);
      document.removeEventListener('visibilitychange', onVisible);
    };
  }, [active, refreshMaps, refresh, refreshGpu]);

  // A stored map that is no longer listed (or none yet): the first map listed.
  useEffect(() => {
    if (!maps) return;
    const next = mapToShow(choices, map);
    if (next !== map) setMap(next);
  }, [maps, choices, map]);

  const pickMap = (name: string) => {
    storeMap(name);
    setMap(name);
    setViewer(null);
    setExporting(false);
    setBusyText(null);
    setConfirming(false);
    setRenderError(null);
  };

  const rows = bench?.rows ?? [];
  const shown = useMemo(() => visibleRows(rows, filter), [rows, filter]);
  const clash = useMemo(() => clashByKey(rows), [rows]);
  const sum = summary(rows);
  const last = bench?.last_render ?? null;
  const items = useMemo(() => viewerItems(last), [last]);

  // A new render replaces the strip; a viewer open on the old one closes.
  const lastId = last?.job_id;
  useEffect(() => { setViewer(null); }, [lastId, map]);

  // When a job ends, the GPU it held is free again.
  const jobStatus = bench?.job?.status;
  useEffect(() => { if (jobStatus && jobStatus !== 'running' && jobStatus !== 'queued') refreshGpu(); }, [jobStatus, refreshGpu]);

  const render = async (override: boolean) => {
    if (!map) return;
    setStarting(true);
    setRenderError(null);
    try {
      await flushAll();
      const job = await startRender(map, override);
      setJob(job);
      setBusyText(null);
      setConfirming(false);
    } catch (e) {
      if (e instanceof ApiError && e.status === 409) {
        setBusyText(e.busy_text ?? e.detail);
        setConfirming(true);
      } else {
        setRenderError(errorText(e));
        setConfirming(false);
      }
    } finally {
      setStarting(false);
    }
  };

  const noArena = !!bench && !bench.arena;
  const armNames = bench ? Object.keys(bench.arm_poses) : [];
  if (bench && !armNames.includes(bench.arm_pose)) armNames.unshift(bench.arm_pose);
  const closeViewer = useCallback(() => setViewer(null), []);
  const closeExport = useCallback(() => setExporting(false), []);

  return (
    <div className="cp">
      <div className="cp-ctx">
        <div className="cp-field">
          <label className="cp-lbl" htmlFor="cp-map">Map</label>
          <select id="cp-map" value={known ?? ''} disabled={!choices.length}
                  onChange={(e) => { if (e.target.value) pickMap(e.target.value); }}>
            {!known && <option value="" disabled>{maps ? 'No arenas yet' : 'Loading maps…'}</option>}
            {choices.map((n) => <option key={n} value={n}>{n}</option>)}
          </select>
        </div>
        <div className="cp-field">
          <label className="cp-lbl" htmlFor="cp-arm">Arm pose (whole render)</label>
          <select id="cp-arm" value={bench?.arm_pose ?? ''} disabled={!bench || !armNames.length}
                  title={joints(bench?.arm_poses[bench.arm_pose])}
                  onChange={(e) => send([{ op: 'set_arm', name: e.target.value }])}>
            {!bench && <option value="">…</option>}
            {armNames.map((n) => (
              <option key={n} value={n} title={joints(bench?.arm_poses[n])}>
                {bench?.arm_poses[n] ? n : `${n} (not in constants.json)`}
              </option>
            ))}
          </select>
        </div>
        {maps && !choices.length && (
          <div className="cp-msg warn cp-noarena" role="status">
            <span>No arenas yet — build one on <a href="#/arena">Map → arena</a>.</span>
          </div>
        )}
        {noArena && (
          <div className="cp-msg warn cp-noarena" role="status">
            <span><code>{map}</code> has no Map Check arena yet, so it can't be rendered or checked for overlaps.
              {' '}Build one on <a href="#/arena">Map → arena</a>.</span>
          </div>
        )}
      </div>

      {mapsError && <div className="cp-wrap"><div className="cp-msg err" role="alert">Couldn't list the maps: {mapsError}</div></div>}

      <ResultsStrip last={last} loading={!bench && !loadError && !mapsError && (!maps || choices.length > 0)} onOpen={setViewer} />

      <div className="cp-wrap cp-grow">
        {loadError && <div className="cp-msg err" role="alert">Couldn't load the poses for {map}: {loadError}</div>}
        {opError && (
          <div className="cp-msg err cp-dismiss" role="alert">
            <span>That change didn't save: {opError}</span>
            <button type="button" className="cp-link" onClick={clearOpError}>dismiss</button>
          </div>
        )}
        {bench?.flags_error && (
          <div className="cp-msg warn" role="status">Overlap check: {bench.flags_error}</div>
        )}
        <section className="cp-card cp-split" aria-label="Poses">
          <div className="cp-main">
            <div className="cp-toolbar">
              <input type="search" value={filter} onChange={(e) => setFilter(e.target.value)}
                     placeholder="Filter poses, e.g. dining or table_1" aria-label="Filter poses" />
              <span className="cp-hint">Column checkboxes act on the rows shown · shift-click for a range</span>
              <button type="button" className="cp-btn" disabled={!bench} onClick={() => setExporting(true)}>Export list…</button>
            </div>
            {bench ? (
              <PoseTable rows={rows} shown={shown} filter={filter} removedCount={bench.removed_count}
                         clash={clash} tiltDownSign={bench.tilt_down_sign} send={send} />
            ) : (
              <div className="cp-tablewrap cp-center">{loadError ? '' : known ? 'Loading poses…' : maps && !choices.length ? 'No arenas yet.' : 'Pick a map.'}</div>
            )}
            <div className="cp-legend">
              <span><i className="bad" />robot overlaps a wall or shape</span>
              <span><i className="near" />closer than {OVERLAP_MARGIN_CM} cm</span>
              <span><i className="clash" />same name in both lists</span>
              <span className="cp-hint">checked live for the chosen arm pose and head angles</span>
            </div>
          </div>
          <PastePanel disabled={!bench} onAdd={(text) => send([{ op: 'add_pasted', text }])} />
        </section>
      </div>

      <RenderBar poses={sum.poses} views={sum.views} seconds={sum.seconds} gpu={gpu} gpuError={gpuError}
                 busyText={busyText} confirming={confirming} starting={starting}
                 blocked={noArena ? 'Needs a Map Check arena first' : null}
                 job={bench?.job ?? null} error={renderError}
                 onRender={() => void render(false)} onConfirm={() => void render(true)}
                 onCancel={() => setConfirming(false)} />

      {viewer !== null && items.length > 0 && (
        <Viewer items={items} index={Math.min(viewer, items.length - 1)} onIndex={setViewer}
                onClose={closeViewer} active={active} />
      )}
      {exporting && map && bench && (
        <ExportDialog map={map} rows={rows} version={bench.version} flush={flushAll}
                      onClose={closeExport} active={active} />
      )}
    </div>
  );
}
