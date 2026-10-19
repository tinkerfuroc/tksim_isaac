import { useState } from 'react';
import type { LastRender } from '../api';

interface Props {
  last: LastRender | null;
  loading: boolean;
  /** Opens the viewer at this index of viewerItems(last). */
  onOpen: (index: number) => void;
}

export function timeText(epochS: number): string {
  const d = new Date(epochS * 1000);
  const hm = d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  return d.toDateString() === new Date().toDateString()
    ? hm : `${d.toLocaleDateString([], { month: 'short', day: 'numeric' })} ${hm}`;
}

/** "Last render": the overview, then one card per pose holding only its views, all one height. */
export function ResultsStrip({ last, loading, onOpen }: Props) {
  if (!last) {
    return (
      <>
        <div className="cp-strip-head">
          <h2>Last render</h2>
          <span className="cp-hint">
            {loading ? 'Loading…' : 'Nothing rendered for this map yet. Tick poses below, then Render.'}
          </span>
        </div>
        <div className="cp-strip cp-strip-empty" aria-hidden="true"><div className="cp-shot-ph" /></div>
      </>
    );
  }
  const n = last.items.length;
  let index = 0;
  return (
    <>
      <div className="cp-strip-head">
        <h2>Last render</h2>
        <span className="cp-hint">
          {timeText(last.finished_at)} · {last.arm_pose} · {n} pose{n === 1 ? '' : 's'} · scroll sideways,
          click an image to enlarge (← → to step, Esc to close)
        </span>
      </div>
      <div className="cp-strip" tabIndex={0} role="region" aria-label="Rendered views">
        {last.overview_url && (
          <article className="cp-pose">
            <h3><span>Overview</span></h3>
            <div className="cp-views">
              <Shot url={last.overview_url} caption="top-down, rendered poses labeled" label="Overview"
                    index={index++} onOpen={onOpen} />
            </div>
          </article>
        )}
        {last.items.map((it) => {
          const flagged = it.flags.length > 0;
          const pasted = it.key.startsWith('pasted:');
          return (
            <article key={it.key} className={flagged ? 'cp-pose flagged' : 'cp-pose'}>
              <h3>
                <span title={pasted ? 'pasted pose' : 'from Map Studio'}>
                  {it.name}{pasted && <span className="cp-pasted"> (pasted)</span>}
                </span>
                <span className="st">{flagged ? it.flags.map((f) => f.text).join(' · ') : 'no overlap'}</span>
              </h3>
              <div className="cp-views">
                {it.views.length ? it.views.map((v) => (
                  <Shot key={v.view} url={v.url} caption={v.caption} label={it.name} index={index++} onOpen={onOpen} />
                )) : <div className="cp-noview">no images came back for this pose</div>}
              </div>
            </article>
          );
        })}
      </div>
    </>
  );
}

function Shot({ url, caption, label, index, onOpen }: {
  url: string; caption: string; label: string; index: number; onOpen: (i: number) => void;
}) {
  const [state, setState] = useState<'loading' | 'ok' | 'missing'>('loading');
  return (
    <button type="button" className={`cp-shot ${state}`} onClick={() => onOpen(index)}
            aria-label={`Enlarge ${label}, ${caption}`}>
      {state === 'missing'
        ? <span className="cp-shot-missing">image missing</span>
        : <img src={url} alt={`${label}, ${caption}`} decoding="async"
               onLoad={() => setState('ok')} onError={() => setState('missing')} />}
      <span className="cp-cap">{caption}</span>
    </button>
  );
}
