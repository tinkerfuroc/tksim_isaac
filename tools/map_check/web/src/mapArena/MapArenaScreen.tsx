import { lazy, Suspense, useCallback, useEffect, useRef, useState } from 'react';
import { ApiError, getMaps, getStudioMap, saveMap, uploadMap, type MapBody, type MapsList } from '../api';
import {
  applyFillRule, decodeGrid, eraseLine, initEdit, savePayload, selectShape, setDefaultShapeHeight,
  setShapeHeight, setWallHeight, toggleShape, undoStroke, type EditState, type Grid,
} from './editState';
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

type Source =
  | { kind: 'studio'; name: string; token: string }
  | { kind: 'upload'; token: string; yamlName: string; imageName: string; identicalTo: string | null };

interface Loaded { source: Source; grid: Grid; gridB64: string }

interface Saved { name: string; version: string; uploaded: boolean }

function errorText(e: unknown): string {
  if (e instanceof ApiError) return e.detail; // 409 and the rest: the server's words, verbatim
  if (e instanceof TypeError) return `Couldn't reach Map Check: ${e.message}`; // fetch failed
  return e instanceof Error ? e.message : String(e);
}

const trim = (v: number, d: number) => String(+v.toFixed(d));
const basename = (p: string) => p.replace(/[\\/]+$/, '').split(/[\\/]/).pop() ?? p;
const isYaml = (f: File) => /\.ya?ml$/i.test(f.name);

export function MapArenaScreen() {
  const [maps, setMaps] = useState<MapsList | null>(null);
  const [mapsError, setMapsError] = useState<string | null>(null);
  const [loaded, setLoaded] = useState<Loaded | null>(null);
  const [edit, setEditState] = useState<EditState | null>(null);
  const [loading, setLoading] = useState<string | null>(null);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [fillArea, setFillArea] = useState(DEFAULT_FILL_AREA_M2);
  const [erasing, setErasing] = useState(false);
  const [brush, setBrush] = useState(DEFAULT_BRUSH_CELLS);
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

  const refreshMaps = useCallback(() => {
    getMaps().then((m) => { setMaps(m); setMapsError(null); })
      .catch((e) => setMapsError(errorText(e)));
  }, []);
  useEffect(refreshMaps, [refreshMaps]);

  const open = (body: MapBody, source: Source) => {
    const grid = decodeGrid(body.grid);
    setLoaded({ source, grid, gridB64: body.grid.grid });
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
      if (seq === loadSeq.current) open(body, toSource(body));
    } catch (e) {
      if (seq === loadSeq.current) setLoadError(errorText(e));
    } finally {
      if (seq === loadSeq.current) setLoading(null);
    }
  };

  const pickStudio = (mapName: string) => load(`Opening ${mapName}…`, () => getStudioMap(mapName),
    (b) => ({ kind: 'studio', name: mapName, token: b.token }));

  const pickFiles = (files: FileList | null) => {
    const list = [...(files ?? [])];
    if (!list.length) return;
    const yamls = list.filter(isYaml);
    const images = list.filter((f) => !isYaml(f));
    if (yamls.length !== 1 || images.length !== 1) {
      setLoadError('Choose two files together: the map .yaml and its image (.pgm or .png).');
      return;
    }
    const [yaml, image] = [yamls[0], images[0]];
    load(`Reading ${yaml.name}…`, () => uploadMap(yaml, image), (b) => ({
      kind: 'upload', token: b.token, yamlName: yaml.name, imageName: image.name, identicalTo: b.identical_to,
    }));
  };

  // Esc leaves the brush; Ctrl/Cmd+Z undoes the last stroke (not while typing in a field).
  useEffect(() => {
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
  }, [update]);

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
  const nameErr = uploading ? nameProblem(name) : null;
  let takenErr: string | null = null;
  if (uploading && !nameErr && maps) {
    if (maps.studio.includes(name)) takenErr = `A map called ${name} already exists in Map Studio. Pick another name.`;
    else if (maps.arenas.includes(name)) takenErr = `An arena called ${name} already exists. Pick another name.`;
  }
  const canSave = !!(loaded && edit && !saving && !nameErr && !takenErr && !loading);

  const save = async () => {
    if (!loaded || !editRef.current || !canSave) return;
    const seq = loadSeq.current;
    const source = loaded.source;
    setSaving(true);
    setSaveError(null);
    setSaved(null);
    try {
      const res = await saveMap(savePayload(editRef.current, source.token, name));
      if (seq !== loadSeq.current) return;
      if (source.kind === 'upload') {
        // The map is in Map Studio now: carry on as that map, so the next save adds a version.
        try {
          const body = await getStudioMap(res.name);
          if (seq === loadSeq.current && body.grid.grid === loaded.gridB64) {
            setLoaded({ ...loaded, source: { kind: 'studio', name: res.name, token: body.token } });
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
  const imageExt = src?.kind === 'upload' ? (src.imageName.match(/\.[^.]+$/)?.[0] ?? '') : '';
  const shown = name || '<name>';

  return (
    <div className="ma" style={PALETTE_VARS}>
      <aside className="ma-rail">
        <section className="ma-step">
          <h3>1 · Source map</h3>
          <div className="ma-row">
            <select aria-label="Pick a Map Studio map" value={src?.kind === 'studio' ? src.name : ''}
                    disabled={!maps?.studio.length}
                    onChange={(e) => { if (e.target.value) pickStudio(e.target.value); }}>
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
            <div className="ma-files"><span className="ma-file">{src.yamlName}</span><span className="ma-file">{src.imageName}</span></div>
          )}
          {loaded && !loading && (
            <div className="ma-meta">
              {loaded.grid.W} × {loaded.grid.H} cells · {trim(loaded.grid.res * 100, 2)} cm · origin {trim(loaded.grid.ox, 3)}, {trim(loaded.grid.oy, 3)}
            </div>
          )}
          {loading && <div className="ma-hint">{loading}</div>}
          {mapsError && <div className="ma-msg err">Couldn't list the Map Studio maps: {mapsError}</div>}
          {loadError && <div className="ma-msg err">{loadError}</div>}
          {src?.kind === 'upload' && src.identicalTo && (
            <div className="ma-msg note">
              <span>This is the same map as <code>{src.identicalTo}</code> in Map Studio — use that map instead?</span>
              <button className="btn" onClick={() => pickStudio(src.identicalTo!)}>Use {src.identicalTo}</button>
            </div>
          )}
        </section>

        {!edit || !loaded ? (
          ['2 · Walls', '3 · Filled shapes', '4 · Erase noise', '5 · Save'].map((t) => (
            <section key={t} className="ma-step off" aria-disabled="true"><h3>{t}</h3></section>
          ))
        ) : (
          <>
            <section className="ma-step">
              <h3>2 · Walls</h3>
              <div className="ma-row">
                <label htmlFor="ma-wall-h">Height</label>
                <NumberField id="ma-wall-h" value={edit.wallHeight} min={MIN_HEIGHT_M} max={MAX_HEIGHT_M}
                             step={0.05} digits={2} onChange={(h) => update((s) => setWallHeight(s, h))} /> m
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
                             onChange={setFillArea} onEnter={() => update((s) => applyFillRule(s, fillArea))} /> m²
                <button className="btn" onClick={() => update((s) => applyFillRule(s, fillArea))}>apply</button>
              </div>
              <div className="ma-row">
                <label htmlFor="ma-def-h">Default shape height</label>
                <NumberField id="ma-def-h" value={edit.defaultShapeHeight} min={MIN_HEIGHT_M} max={MAX_HEIGHT_M}
                             step={0.05} digits={2} onChange={(h) => update((s) => setDefaultShapeHeight(s, h))} /> m
              </div>
              <ShapesList outlines={edit.outlines} shapes={edit.shapes} selected={edit.selected}
                          onToggle={(id) => update((s) => toggleShape(s, id))}
                          onHeight={(id, h) => update((s) => setShapeHeight(s, id, h))}
                          onSelect={(id) => update((s) => selectShape(s, id))} />
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
                <NumberField id="ma-brush" value={brush} min={1} max={15} step={1} digits={0} onChange={setBrush} /> cells
                <button className="btn" disabled={!edit.strokes.length} onClick={() => update(undoStroke)}>undo</button>
              </div>
              <span className="ma-hint">
                Paint over stray specks on the 2D map. Only the arena changes;{' '}
                {uploading ? 'the saved 2D map stays as uploaded.' : 'the Map Studio map stays as it is.'}
                {' '}Esc puts the brush away, Ctrl+Z undoes a stroke.
              </span>
            </section>

            <section className="ma-step">
              <h3>5 · Save</h3>
              <div className="ma-row">
                <label htmlFor="ma-name">Name</label>
                <input id="ma-name" type="text" value={name} readOnly={!uploading} spellCheck={false}
                       className={nameErr || takenErr ? 'bad' : undefined}
                       title={uploading ? undefined : 'A picked map keeps its own name'}
                       onChange={(e) => { setName(e.target.value.trim()); setSaveError(null); }}
                       onKeyDown={(e) => { if (e.key === 'Enter') save(); }} />
              </div>
              {nameErr && name && <div className="ma-msg err">{nameErr}</div>}
              {takenErr && <div className="ma-msg err">{takenErr}</div>}
              {saveError && <div className="ma-msg err">{saveError}</div>}
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
          </>
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
