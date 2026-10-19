import { useEffect, useRef, useState, type ChangeEvent } from 'react';
import type { BenchOp, BenchRow } from '../api';
import { NumberField } from '../mapArena/NumberField';
import {
  clashText, flagTone, headerState, headerToggleOps, panTiltOps, rangeOps, type Col, type HeaderState,
} from './tableLogic';

export const PAN_MIN = -180;
export const PAN_MAX = 180;
export const TILT_MIN = -30;
export const TILT_MAX = 90;
const COLS = 7;

interface Props {
  rows: BenchRow[]; // every row on the bench
  shown: BenchRow[]; // after the filter
  filter: string;
  removedCount: number;
  clash: Map<string, number>;
  tiltDownSign: number;
  send: (ops: BenchOp[], debounceKey?: string) => void;
}

const yawDeg = (rad: number) => (rad * 180) / Math.PI;

export function PoseTable({ rows, shown, filter, removedCount, clash, tiltDownSign, send }: Props) {
  const anchor = useRef<string | null>(null); // last Render box clicked, for shift-click ranges
  const [panAll, setPanAll] = useState(0);
  const [tiltAll, setTiltAll] = useState(20);
  const [clearAsk, setClearAsk] = useState(false);

  const tiltDown = tiltDownSign >= 0 ? '+' : '−';
  const setAngles = panTiltOps(shown, panAll, tiltAll);

  const onInclude = (r: BenchRow, e: ChangeEvent<HTMLInputElement>) => {
    const value = e.currentTarget.checked;
    const shift = (e.nativeEvent as MouseEvent).shiftKey === true;
    const from = anchor.current;
    anchor.current = r.key;
    send(shift && from && from !== r.key
      ? rangeOps(shown, from, r.key, value)
      : [{ op: 'set_row', row: r.key, fields: { included: value } }]);
  };

  const groups = (['saved', 'pasted'] as const).map((g) => {
    const all = rows.filter((r) => r.group === g);
    return { g, all, shown: shown.filter((r) => r.group === g), inRender: all.filter((r) => r.state.included).length };
  });

  const header = (col: Col, label: string, aria: string) => (
    <div className="cp-hcheck">
      <HeaderCheck state={headerState(shown, col)} label={aria} onToggle={() => send(headerToggleOps(shown, col))} />
      <span>{label}</span>
    </div>
  );

  return (
    <div className="cp-tablewrap">
      <table className="cp-table">
        <thead>
          <tr>
            <th className="col">{header('included', 'Render', 'Render all shown')}</th>
            <th>Pose</th>
            <th>x, y, yaw</th>
            <th className="col">{header('behind', 'Behind', 'Behind view for all shown')}</th>
            <th className="col">{header('top', 'Top', 'Top view for all shown')}</th>
            <th className="col">
              {header('head', 'Head cam', 'Head camera for all shown')}
              <div className="cp-setall">
                <NumberField value={panAll} min={PAN_MIN} max={PAN_MAX} step={5} digits={0} field="pan for all shown"
                             label="Pan for all shown" onChange={(v) => setPanAll(Math.round(v))}
                             onEnter={() => send(setAngles)} />°
                <NumberField value={tiltAll} min={TILT_MIN} max={TILT_MAX} step={5} digits={0} field="tilt for all shown"
                             label="Tilt for all shown" onChange={(v) => setTiltAll(Math.round(v))}
                             onEnter={() => send(setAngles)} />°
                <button type="button" className="cp-btn tiny" disabled={!setAngles.length}
                        title={setAngles.length ? 'Set pan and tilt on every shown pose with Head cam on'
                          : 'Tick Head cam on a shown pose first'}
                        onClick={() => send(setAngles)}>set</button>
              </div>
              <div className="cp-tiltnote" title={`Pan ${PAN_MIN}…${PAN_MAX}°, tilt ${TILT_MIN}…${TILT_MAX}° (pan_joint, tilt_joint)`}>
                tilt {tiltDown}° looks down
              </div>
            </th>
            <th className="col"><div className="cp-hcheck"><span>Remove</span></div></th>
          </tr>
        </thead>
        <tbody>
          {groups.map(({ g, all, shown: gs, inRender }) => {
            if (g === 'pasted' && !all.length) return null;
            return [
              <tr key={`${g}-head`} className="group">
                <td colSpan={COLS}>
                  {g === 'saved' ? 'From Map Studio' : 'Pasted'} <span className="gcount">({all.length})</span>
                  <span className="gcount"> · {inRender} of {all.length} in render</span>
                  {filter.trim() && gs.length !== all.length && <span className="gcount"> · {gs.length} shown</span>}
                  {g === 'saved' && removedCount > 0 && (
                    <> · <span className="gcount">{removedCount} removed</span> · <button type="button" className="cp-link"
                      title="Bring back every Map Studio pose removed from this list"
                      onClick={() => send([{ op: 'restore_saved' }])}>restore</button></>
                  )}
                  {g === 'pasted' && (clearAsk ? (
                    <span className="cp-ask"> · Delete all {all.length} pasted poses?{' '}
                      <button type="button" className="cp-link bad" autoFocus
                              onClick={() => { setClearAsk(false); send([{ op: 'clear_pasted' }]); }}>clear</button>{' '}
                      <button type="button" className="cp-link" onClick={() => setClearAsk(false)}>keep</button>
                    </span>
                  ) : (
                    <> · <button type="button" className="cp-link" onClick={() => setClearAsk(true)}>clear</button></>
                  ))}
                </td>
              </tr>,
              ...gs.map((r) => (
                <Row key={r.key} r={r} clash={clash.get(r.key)} onInclude={onInclude} send={send} />
              )),
              !gs.length && (
                <tr key={`${g}-empty`} className="empty">
                  <td colSpan={COLS}>
                    {all.length ? `No ${g === 'saved' ? 'Map Studio' : 'pasted'} poses match "${filter.trim()}".`
                      : removedCount ? 'Every Map Studio pose is removed from this list.'
                        : 'No poses saved in Map Studio for this map.'}
                  </td>
                </tr>
              ),
            ];
          })}
        </tbody>
      </table>
    </div>
  );
}

function Row({ r, clash, onInclude, send }: {
  r: BenchRow;
  clash: number | undefined;
  onInclude: (r: BenchRow, e: ChangeEvent<HTMLInputElement>) => void;
  send: (ops: BenchOp[], debounceKey?: string) => void;
}) {
  const s = r.state;
  const name = r.pose.name;
  const view = (col: 'behind' | 'top' | 'head', label: string) => (
    <input type="checkbox" checked={s[col]} disabled={!s.included} aria-label={`${label} for ${name}`}
           onChange={(e) => send([{ op: 'set_row', row: r.key, fields: { [col]: e.currentTarget.checked } }])} />
  );
  const angles = s.included && s.head;
  return (
    <tr className={s.included ? undefined : 'off'}>
      <td className="c inc">
        <input type="checkbox" checked={s.included} aria-label={`Render ${name}`} onChange={(e) => onInclude(r, e)} />
      </td>
      <td className="name">
        <span>{name}</span>
        {clash !== undefined && (
          <span className="cp-chip clash" title="A pasted pose and a Map Studio pose share this name">{clashText(clash)}</span>
        )}
        {r.flags.map((f, i) => <span key={i} className={`cp-chip ${flagTone(f)}`}>{f.text}</span>)}
      </td>
      <td className="coords">{r.pose.x.toFixed(3)}, {r.pose.y.toFixed(3)}, {yawDeg(r.pose.yaw).toFixed(1)}°</td>
      <td className="c">{view('behind', 'Behind view')}</td>
      <td className="c">{view('top', 'Top view')}</td>
      <td className="c">
        <div className="cp-headcell">
        {view('head', 'Head camera')}
        <span className="cp-pt" aria-hidden={!angles} style={{ visibility: angles ? 'visible' : 'hidden' }}>
          <NumberField value={s.pan_deg} min={PAN_MIN} max={PAN_MAX} step={5} digits={0} field={`pan ${name}`}
                       label={`Pan for ${name}`} disabled={!angles}
                       onChange={(v) => send([{ op: 'set_row', row: r.key, fields: { pan_deg: Math.round(v) } }], `${r.key}/pan`)} />
          <NumberField value={s.tilt_deg} min={TILT_MIN} max={TILT_MAX} step={5} digits={0} field={`tilt ${name}`}
                       label={`Tilt for ${name}`} disabled={!angles}
                       onChange={(v) => send([{ op: 'set_row', row: r.key, fields: { tilt_deg: Math.round(v) } }], `${r.key}/tilt`)} />
        </span>
        </div>
      </td>
      <td className="c">
        <button type="button" className="cp-btn rm" aria-label={`Remove ${name}`}
                title={r.group === 'saved' ? "Remove from this list (Map Studio's file is not changed)" : 'Delete this pasted pose'}
                onClick={() => send([{ op: 'remove', row: r.key }])}>✕</button>
      </td>
    </tr>
  );
}

function HeaderCheck({ state, label, onToggle }: { state: HeaderState; label: string; onToggle: () => void }) {
  const ref = useRef<HTMLInputElement>(null);
  useEffect(() => { if (ref.current) ref.current.indeterminate = state === 'some'; }, [state]);
  return (
    <input ref={ref} type="checkbox" aria-label={label} checked={state === 'all'} disabled={state === 'disabled'}
           aria-checked={state === 'some' ? 'mixed' : state === 'all'} onChange={onToggle} />
  );
}
