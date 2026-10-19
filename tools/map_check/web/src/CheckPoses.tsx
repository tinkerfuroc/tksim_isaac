import { CheckPosesScreen } from './checkPoses/CheckPosesScreen';

/** `active` is false while the tab is hidden (the screen stays mounted, its keys stay off). */
export function CheckPoses({ active = true }: { active?: boolean }) {
  return <CheckPosesScreen active={active} />;
}
