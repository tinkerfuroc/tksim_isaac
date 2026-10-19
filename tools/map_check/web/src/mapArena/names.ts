// Map name rules, as map_check/names.py (copied from Map Studio). The server checks too;
// this only lets the page say what is wrong before Save.
const MAP_NAME = /^[A-Za-z0-9_][A-Za-z0-9_.-]{0,99}$/;

export function nameProblem(name: string): string | null {
  if (!name) return 'Enter a name for the map.';
  if (!MAP_NAME.test(name) || name.includes('..') || name.endsWith('.poses')) {
    return 'Use up to 100 letters, digits, _ . or - (not starting with . or -, no "..", not ending in ".poses").';
  }
  return null;
}
