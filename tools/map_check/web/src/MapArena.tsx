import { MapArenaScreen } from './mapArena/MapArenaScreen';

/** `active` is false while the tab is hidden (the screen stays mounted to keep unsaved edits). */
export function MapArena({ active = true }: { active?: boolean }) {
  return <MapArenaScreen active={active} />;
}
