// Map colours shared by the 2D canvas, the 3D preview and the legends (mockup map-arena-v1 tokens).
// The saved arena uses the same wall and shape colours (arena_build.WALL_COLOR / SHAPE_COLOR).
export const PALETTE = {
  wall: '#7f9cc0',
  shape: '#d2a865',
  shapeOff: '#4a3f2c',
  erase: '#f2776c',
  free: '#0c0f13',
  selected: '#6aa5ec', // theme --accent
  ground: '#11151b', // theme --ground
  floor: '#2a2f37',
} as const;

/** The palette as CSS custom properties for the screen's root element. */
export const PALETTE_VARS = {
  '--wall': PALETTE.wall,
  '--shape': PALETTE.shape,
  '--shape-off': PALETTE.shapeOff,
  '--erase': PALETTE.erase,
  '--free': PALETTE.free,
} as Record<string, string>;

/** '#rrggbb' -> [r, g, b] bytes. */
export function rgb(hex: string): [number, number, number] {
  const n = parseInt(hex.slice(1), 16);
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
}
