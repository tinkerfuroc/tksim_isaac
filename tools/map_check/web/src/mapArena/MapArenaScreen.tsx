import { lazy, Suspense, useCallback, useEffect, useRef, useState } from 'react';
import { ApiError, getMaps, getStudioMap, saveMap, uploadMap, type MapBody, type MapsList } from '../api';
import {
  applyFillRule, decodeGrid, eraseLine, initEdit, isDirty, markSaved, savePayload, selectShape,
  setDefaultShapeHeight, setShapeHeight, setWallHeight, toggleShape, undoStroke, type EditState, type Grid,
} from './editState';
import { fixFieldsMessage } from './fields';
import { MapCanvas2D } from './MapCanvas2D';
import { nameProblem } from './names';
import { NumberField } from './NumberField';
import { PALETTE_VARS } from './palette';
import { MAX_HEIGHT_M, MIN_HEIGHT_M, ShapesList } from './ShapesList';
import './mapArena.css';

// three.js is large; load it after the rest of the page.
const Preview3D = lazy(() => import('./Preview3D'));

const DEFAULT_FILL_AREA_M2 = 4.0;
const DEFAULT_BRUSH_CELLS = 3;

// Field names as "Fix the … before saving" says them.
const F_WALL = 'wall height';
const F_FILL = 'fill area limit';
const F_DEFAULT = 'default shape height';
const F_BRUSH = 'brush size';
const shapeField = (id: number) => `shape ${id} height`;

type Source =
  | { kind: 'studio'; name: string; token: string }
  | { kind: 'upload'; token: string; yaml: File; image: File; identicalTo: string | null };

interface Loaded { id: number; source: Source; grid: Grid; gridB64: string }

interface Saved { name: string; version: string; uploaded: boolean }

function errorText(e: unknown): string {
  if (e instanceof ApiError) return e.detail; // 409 and the rest: the server's words, verbatim
  if (e instanceof TypeError) return `Couldn't reach Map Check: ${e.message}`; // fetch failed
  return e instanceof Error ? e.message : String(e);
}

const trim = (v: number, d: number) => String(+v.toFixed(d));
const basename = (p: string) => p.replace(/[\\/]+$/, '').split(/[\\/]/).pop() ?? p;
const isYaml = (f: File) => /\.ya?ml$/i.test(f.name);
const sourceLabel = (s: Source) => (s.kind === 'studio' ? s.name : s.yaml.name);

/** `active` is false while another tab is shown; the screen stays mounted so edits survive. */
export function MapArenaScreen({ active = true }: { active?: boolean }) {
  const [maps, setMaps] = useState<MapsList | null>(null);
  const [mapsError, setMapsError] = useState<string | null>(null);
  const [loaded, setLoaded] = useState<Loaded | null>(null);
  const [edit, setEditState] = useState<EditState | null>(null);
  const [loading, setLoading] = useState<string | null>(null);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [pending, setPending] = useState<{ go: () => void } | null>(null); // a load waiting on "Discard"
  const [fillArea, setFillArea] = useState(DEFAULT_FILL_AREA_M2);
  const [erasing, setErasing] = useState(false);
  const [brush, setBrush] = useState(DEFAULT_BRUSH_CELLS);
  const [badFields, setBadFields] = useState<string[]>([]);
  const [name, setName] = useState('');
  const [saving, setSaving] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);
  const [saved, setSaved] = useState<Saved | null>(null);
  const editRef = useRef<EditState | null>(null);
  const loadSeq = useRef(0); // bumps on every map open; stale responses compare against it
  const strokeOpen = useRef(false); // the current brush drag has erased something
  const fileInput = useRef<HTMLInputElement>(null);

  const setEdit = (s: EditState | null) => { editRef.current = s; setEditState(s); };
  // Every edit goes through here, so pointer events in one frame each see the previous result.
  const update = useCallback((f: (s: EditState) => EditState) => {
    const cur = editRef.current;
    if (!cur) return;
    const next = f(cur);
    if (next !== cur) { editRef.current = next; setEditState(next); }
  }, []);

  const onValidity = useCallback((field: string, valid: boolean) => {
    setBadFields((prev) => {
      if (valid) return prev.includes(field) ? prev.filter((f) => f !== field) : prev;
      return prev.includes(field) ? prev : [...prev, field];
    });
  }, []);

  const refreshMaps = useCallback(() => {
    getMaps().then((m) => { setMaps(m); setMapsError(null); })
      .catch((e) => setMapsError(errorText(e)));
  }, []);
  useEffect(refreshMaps, [refreshMaps]);

  const open = (seq: number, body: MapBody, source: Source) => {
    const grid = decodeGrid(body.grid);
    setLoaded({ id: seq, source, grid, gridB64: body.grid.grid });
    setEdit(initEdit(grid, body.outlines, body.defaults));
    setName(source.kind === 'studio' ? source.name : body.name_suggestion);
    setFillArea(DEFAULT_FILL_AREA_M2);
    setErasing(false);
    setSaved(null);
    setSaveError(null);
  };

  const load = async (label: string, fetchBody: () => Promise<MapBody>, toSource: (b: MapBody) => Source) => {
    const seq = ++loadSeq.current;
    setLoading(label);
    setLoadError(null);
    try {
      const body = await fetchBody();
      if (seq === loadSeq.current) open(seq, body, toSource(body));
    } catch (e) {
      if (seq === loadSeq.current) setLoadError(errorText(e));
    } finally {
      if (seq === loadSeq.current) setLoading(null);
    }
  };

  const loadStudio = (mapName: string) => load(`Opening ${mapName}…`, () => getStudioMap(mapName),
    (b) => ({ kind: 'studio', name: mapName, token: b.token }));
  const loadUpload = (yaml: File, image: File) => load(`Reading ${yaml.name}…`, () => uploadMap(yaml, image),
    (b) => ({ kind: 'upload', token: b.token, yaml, image, identicalTo: b.identical_to }));
  const reload = (s: Source) => (s.kind === 'studio' ? loadStudio(s.name) : loadUpload(s.yaml, s.image));

  /** Runs `go` (which replaces the open map) now, or after "Discard" when there are unsaved edits. */
  const guard = (go: () => void) => {
    const cur = editRef.current;
    if (cur && isDirty(cur)) { setPending({ go }); return; }
    setPending(null);
    go();
  };

  const pickFiles = (files: FileList | null) => {
    const list = [...(files ?? [])];
    if (!list.length) return;
    const yamls = list.filter(isYaml);
    const images = list.filter((f) => !isYaml(f));
    if (yamls.length !== 1 || images.length !== 1) {
      setLoadError('Choose two files together: the map .yaml and its image (.pgm or .png).');
      return;
    }
    guard(() => loadUpload(yamls[0], images[0]));
  };

  // Esc leaves the brush; Ctrl/Cmd+Z undoes the last stroke (not while typing in a field).
  useEffect(() => {
    if (!active) return;
    const onKey = (e: KeyboardEvent) => {
      const typing = e.target instanceof HTMLElement && e.target.closest('input, select, textarea');
      if (e.key === 'Escape') setErasing(false);
      else if ((e.ctrlKey || e.metaKey) && !e.shiftKey && e.key.toLowerCase() === 'z' && !typing) {
        e.preventDefault();
        update(undoStroke);
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [active, update]);

  const onErase = (r0: number, c0: number, r1: number, c1: number, first: boolean) => {
    if (!loaded) return;
    if (first) strokeOpen.current = false;
    update((s) => {
      const next = eraseLine(s, loaded.grid, r0, c0, r1, c1, brush, strokeOpen.current);
      if (next !== s) strokeOpen.current = true;
      return next;
    });
  };

  const src = loaded?.source;
  const uploading = src?.kind === 'upload';
  const dirty = !!edit && isDirty(edit);
  const nameErr = uploading ? nameProblem(name) : null;
  let takenErr: string | null = null;
  if (uploading && !nameErr && maps) {
    if (maps.studio.includes(name)) takenErr = `A map called ${name} already exists in Map Studio. Pick another name.`;
    else if (maps.arenas.includes(name)) takenErr = `An arena called ${name} already exists. Pick another name.`;
  }
  // Invalid number fields, in the order the rail shows them.
  const fieldOrder = [F_WALL, F_FILL, F_DEFAULT, ...(edit?.outlines.map((o) => shapeField(o.id)) ?? []), F_BRUSH];
  const fixMsg = fixFieldsMessage(fieldOrder.filter((f) => badFields.includes(f)));
  const canSave = !!(loaded && edit && !saving && !nameErr && !takenErr && !fixMsg && !loading);

  const save = async () => {
    if (!loaded || !editRef.current || !canSave) return;
    const seq = loadSeq.current;
    const source = loaded.source;
    const sent = editRef.current;
    setSaving(true);
    setSaveError(null);
    setSaved(null);
    try {
      const res = await saveMap(savePayload(sent, source.token, name));
      if (seq !== loadSeq.current) return;
      update((s) => markSaved(s, sent));
      if (source.kind === 'upload') {
        // The map is in Map Studio now: carry on as that map, so the next save adds a version.
        try {
          const body = await getStudioMap(res.name);
          if (seq === loadSeq.current && body.grid.grid === loaded.gridB64) {
            setLoaded((l) => l && { ...l, source: { kind: 'studio', name: res.name, token: body.token } });
          }
        } catch { /* keep the upload; saving it again is refused by name, which says why */ }
      }
      if (seq === loadSeq.current) {
        setSaved({ name: res.name, version: basename(res.version_dir), uploaded: source.kind === 'upload' });
      }
      refreshMaps();
    } catch (e) {
      if (seq === loadSeq.current) setSaveError(errorText(e));
    } finally {
      setSaving(false);
    }
  };

  const filled = edit ? edit.outlines.filter((o) => edit.shapes[o.id].fill).length : 0;
  const imageExt = src?.kind === 'upload' ? (src.image.name.match(/\.[^.]+$/)?.[0] ?? '') : '';
  const shown = name || '<name>';
  const fillBad = badFields.includes(F_FILL);
  const applyFill = () => { if (!fillBad) update((s) => applyFillRule(s, fillArea)); };

  return (
    <div className="ma" style={PALETTE_VARS}>
      <aside className="ma-rail">
        <section className="ma-step">
          <h3>1 · Source map</h3>
          <div className="ma-row">
            <select aria-label="Pick a Map Studio map" value={src?.kind === 'studio' ? src.name : ''}
                    disabled={!maps?.studio.length}
                    onChange={(e) => { const n = e.target.value; if (n) guard(() => loadStudio(n)); }}>
              <option value="" disabled>
                {maps ? (maps.studio.length ? 'Pick a Map Studio map…' : 'No maps in Map Studio yet') : 'Loading maps…'}
              </option>
              {maps?.studio.map((n) => (
                <option key={n} value={n}>{maps.arenas.includes(n) ? `${n} · has an arena` : n}</option>
              ))}
            </select>
          </div>
          <div className="ma-row">
            <span className="ma-or">or</span>
            <button className="btn" onClick={() => fileInput.current?.click()}>Upload new map…</button>
            <span className="ma-hint">map .yaml + its image</span>
            <input ref={fileInput} type="file" multiple hidden accept=".yaml,.yml,.pgm,.png"
                   onChange={(e) => { pickFiles(e.target.files); e.target.value = ''; }} />
          </div>
          {src?.kind === 'upload' && (
            <div className="ma-files"><span className="ma-file">{src.yaml.name}</span><span className="ma-file">{src.image.name}</span></div>
          )}
          {loaded && !loading && (
            <div className="ma-row ma-between">
              <span className="ma-meta">
                {loaded.grid.W} × {loaded.grid.H} cells · {trim(loaded.grid.res * 100, 2)} cm · origin {trim(loaded.grid.ox, 3)}, {trim(loaded.grid.oy, 3)}
              </span>
              <button className="btn small" disabled={saving}
                      title="Open this map again with the starting settings"
                      onClick={() => guard(() => reload(loaded.source))}>Reset edits</button>
            </div>
          )}
          <div role="status" aria-live="polite" className={loading ? 'ma-hint' : undefined}>{loading}</div>
          {mapsError && <div className="ma-msg err" role="alert">Couldn't list the Map Studio maps: {mapsError}</div>}
          {loadError && <div className="ma-msg err" role="alert">{loadError}</div>}
          {pending && src && (
            <div className="ma-msg warn" role="alertdialog" aria-labelledby="ma-discard-q">
              <span id="ma-discard-q">Discard unsaved changes to <code>{sourceLabel(src)}</code>?</span>
              <div className="ma-row">
                <button className="btn danger" onClick={() => { const p = pending; setPending(null); p.go(); }}>Discard</button>
                <button className="btn" autoFocus onClick={() => setPending(null)}>Keep editing</button>
              </div>
            </div>
          )}
          {src?.kind === 'upload' && src.identicalTo && (
            <div className="ma-msg note">
              <span>This is the same map as <code>{src.identicalTo}</code> in Map Studio — use that map instead?</span>
              <button className="btn" onClick={() => { const n = src.identicalTo!; guard(() => loadStudio(n)); }}>
                Use {src.identicalTo}
              </button>
            </div>
          )}
        </section>

        {!edit || !loaded ? (
          ['2 · Walls', '3 · Filled shapes', '4 · Erase noise', '5 · Save'].map((t) => (
            <section key={t} className="ma-step off" aria-disabled="true"><h3>{t}</h3></section>
          ))
        ) : (
          // Keyed by the load, so number fields start fresh (and valid) for each opened map.
          <div key={loaded.id} className="ma-steps">
            <section className="ma-step">
              <h3>2 · Walls</h3>
              <div className="ma-row">
                <label htmlFor="ma-wall-h">Height</label>
                <NumberField id="ma-wall-h" value={edit.wallHeight} min={MIN_HEIGHT_M} max={MAX_HEIGHT_M}
                             step={0.05} digits={2} field={F_WALL} onValidity={onValidity}
                             onChange={(h) => update((s) => setWallHeight(s, h))} /> m
              </div>
              <span className="ma-hint">Every occupied cell that isn't part of a filled shape.</span>
            </section>

            <section className="ma-step">
              <h3>3 · Filled shapes <span className="count">
                {edit.outlines.length ? `${filled} of ${edit.outlines.length} filled` : 'none'}
              </span></h3>
              <div className="ma-row">
                <label htmlFor="ma-fill-area">Fill closed outlines up to</label>
                <NumberField id="ma-fill-area" value={fillArea} min={0} max={1e6} step={0.5} digits={1}
                             field={F_FILL} onValidity={onValidity}
                             onChange={setFillArea} onEnter={applyFill} /> m²
                <button className="btn" disabled={fillBad} onClick={applyFill}>apply</button>
              </div>
              <div className="ma-row">
                <label htmlFor="ma-def-h">Default shape height</label>
                <NumberField id="ma-def-h" value={edit.defaultShapeHeight} min={MIN_HEIGHT_M} max={MAX_HEIGHT_M}
                             step={0.05} digits={2} field={F_DEFAULT} onValidity={onValidity}
                             onChange={(h) => update((s) => setDefaultShapeHeight(s, h))} /> m
              </div>
              <ShapesList outlines={edit.outlines} shapes={edit.shapes} selected={edit.selected}
                          onToggle={(id) => update((s) => toggleShape(s, id))}
                          onHeight={(id, h) => update((s) => setShapeHeight(s, id, h))}
                          onSelect={(id) => update((s) => selectShape(s, id))}
                          onValidity={onValidity} />
              <span className="ma-hint">Click a shape here or on the 2D map to highlight it.</span>
            </section>

            <section className="ma-step">
              <h3>4 · Erase noise <span className="count">
                {edit.erased.size ? `${edit.erased.size} cells erased` : ''}
              </span></h3>
              <div className="ma-row">
                <button className={erasing ? 'btn on' : 'btn'} aria-pressed={erasing}
                        onClick={() => setErasing((v) => !v)}>Erase brush</button>
                <label htmlFor="ma-brush">size</label>
                <NumberField id="ma-brush" value={brush} min={1} max={15} step={1} digits={0}
                             field={F_BRUSH} onValidity={onValidity} onChange={setBrush} /> cells
                <button className="btn" disabled={!edit.strokes.length} onClick={() => update(undoStroke)}>undo</button>
              </div>
              <span className="ma-hint">
                Paint over stray specks on the 2D map. Only the arena changes;{' '}
                {uploading ? 'the saved 2D map stays as uploaded.' : 'the Map Studio map stays as it is.'}
                {' '}The brush needs a mouse or pen. Esc puts the brush away, Ctrl+Z undoes a stroke.
              </span>
            </section>

            <section className="ma-step">
              <h3>5 · Save <span className="count">{dirty ? 'unsaved changes' : ''}</span></h3>
              <div className="ma-row">
                <label htmlFor="ma-name">Name</label>
                <input id="ma-name" type="text" value={name} readOnly={!uploading} spellCheck={false}
                       className={nameErr || takenErr ? 'bad' : undefined}
                       aria-invalid={!!(nameErr || takenErr)}
                       title={uploading ? undefined : 'A picked map keeps its own name'}
                       onChange={(e) => { setName(e.target.value.trim()); setSaveError(null); }}
                       onKeyDown={(e) => { if (e.key === 'Enter') save(); }} />
              </div>
              {nameErr && name && <div className="ma-msg err">{nameErr}</div>}
              {takenErr && <div className="ma-msg err">{takenErr}</div>}
              {fixMsg && <div className="ma-msg err" aria-live="polite">{fixMsg}</div>}
              {saveError && <div className="ma-msg err" role="alert">{saveError}</div>}
              {uploading ? (
                <div className="ma-dest">
                  <span>2D map → navigation_bringup/maps/{shown}.yaml + {imageExt || 'image'}</span>
                  <span>3D arena → artifacts/arena/{shown}/ (sim: --arena {shown})</span>
                </div>
              ) : (
                <div className="ma-dest">
                  <span>2D map → stays as it is in Map Studio</span>
                  <span>3D arena → artifacts/arena/{shown}/, a new version (sim: --arena {shown})</span>
                </div>
              )}
              <div>
                <button className="btn primary" disabled={!canSave} onClick={save}>
                  {saving ? 'Saving…' : uploading ? 'Save map + arena' : 'Save arena'}
                </button>
              </div>
              {saved && (
                <div className="ma-msg ok" role="status">
                  <span>Saved arena version <code>{saved.version}</code> — the sim uses it with <code>--arena {saved.name}</code></span>
                  {saved.uploaded && <span>Rebuild navigation_bringup before Nav2 can use this map.</span>}
                </div>
              )}
            </section>
          </div>
        )}
      </aside>

      <div className="ma-views">
        {loaded && edit ? (
          <>
            <MapCanvas2D grid={loaded.grid} edit={edit} erasing={erasing} brush={brush}
                         onSelect={(id) => update((s) => selectShape(s, id))} onErase={onErase} />
            <Suspense fallback={<EmptyPane title="3D preview" text="Loading 3D…" />}>
              <Preview3D grid={loaded.grid} edit={edit} />
            </Suspense>
          </>
        ) : (
          <>
            <EmptyPane title="2D map" text={loading ?? 'Pick a Map Studio map or upload one to start.'} />
            <EmptyPane title="3D preview" text="The arena appears here, live, as you change the settings." />
          </>
        )}
      </div>
    </div>
  );
}

function EmptyPane({ title, text }: { title: string; text: string }) {
  return (
    <section className="ma-pane">
      <h2>{title}</h2>
      <div className="ma-stage"><div className="ma-center">{text}</div></div>
    </section>
  );
}
