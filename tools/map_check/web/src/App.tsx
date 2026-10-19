import { useEffect, useState } from 'react';
import { CheckPoses } from './CheckPoses';
import { MapArena } from './MapArena';

const TABS = [
  { hash: '#/arena', label: 'Map → arena' },
  { hash: '#/poses', label: 'Check poses' },
] as const;

function currentHash(): string {
  return TABS.some((t) => t.hash === window.location.hash) ? window.location.hash : '#/arena';
}

export function App() {
  const [hash, setHash] = useState(currentHash);
  useEffect(() => {
    const on = () => setHash(currentHash());
    window.addEventListener('hashchange', on);
    return () => window.removeEventListener('hashchange', on);
  }, []);
  return (
    <div className="app">
      <header className="top">
        <h1>Tinker Map Check</h1>
        <nav role="tablist">
          {TABS.map((t) => (
            <a key={t.hash} href={t.hash} role="tab" aria-selected={hash === t.hash}
               className={hash === t.hash ? 'tab on' : 'tab'}>
              {t.label}
            </a>
          ))}
        </nav>
      </header>
      {/* Both screens stay mounted so unsaved edits survive a tab switch; only one is shown. */}
      <main>
        <div role="tabpanel" aria-label="Map → arena" hidden={hash !== '#/arena'}>
          <MapArena active={hash === '#/arena'} />
        </div>
        <div role="tabpanel" aria-label="Check poses" hidden={hash !== '#/poses'}>
          <CheckPoses />
        </div>
      </main>
    </div>
  );
}
