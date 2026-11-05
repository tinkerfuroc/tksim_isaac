import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { Rect } from '../rects';
import { boxesFor, type EditState, type Grid } from './editState';
import { PALETTE } from './palette';

interface Props { grid: Grid; edit: EditState }

interface Scene3D {
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  group: THREE.Group;
  box: THREE.BoxGeometry;
  wallMat: THREE.MeshStandardMaterial;
  shapeMat: THREE.MeshStandardMaterial;
  render: () => void;
}

const MIN_ELEVATION_RAD = THREE.MathUtils.degToRad(5); // the camera stays this far above horizontal
const shapeColor = new THREE.Color(PALETTE.shape);
const selColor = new THREE.Color(PALETTE.selected);

/** Frees the per-build instance buffers; the shared box geometry and materials live until unmount. */
function clearGroup(group: THREE.Group) {
  for (const child of [...group.children]) {
    if (child instanceof THREE.InstancedMesh) child.dispose();
    group.remove(child);
  }
}

/** Grid rectangles as unit boxes scaled into place (z-up, map metres, standing on z = 0). */
function instanced(g: Grid, rects: { rect: Rect; h: number; color?: THREE.Color }[],
                   geo: THREE.BoxGeometry, mat: THREE.Material): THREE.InstancedMesh | null {
  if (!rects.length) return null;
  const mesh = new THREE.InstancedMesh(geo, mat, rects.length);
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  const p = new THREE.Vector3();
  const s = new THREE.Vector3();
  rects.forEach(({ rect: [r0, c0, nr, nc], h, color }, i) => {
    p.set(g.ox + (c0 + nc / 2) * g.res, g.oy + (g.H - r0 - nr / 2) * g.res, h / 2);
    s.set(nc * g.res, nr * g.res, h);
    mesh.setMatrixAt(i, m.compose(p, q, s));
    if (color) mesh.setColorAt(i, color);
  });
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

/** Live 3D preview of the arena boxes, rebuilt at most once per frame as the edit changes. */
export default function Preview3D({ grid, edit }: Props) {
  const host = useRef<HTMLDivElement>(null);
  const three = useRef<Scene3D | null>(null);
  const latest = useRef({ grid, edit });
  const frame = useRef(0);
  const [stats, setStats] = useState('');
  const [error, setError] = useState<string | null>(null);
  latest.current = { grid, edit };

  // Scene, camera, lights and controls: once per map.
  useEffect(() => {
    const el = host.current;
    if (!el) return;
    let renderer: THREE.WebGLRenderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true });
    } catch (e) {
      setError(`Couldn't start the 3D preview: ${e instanceof Error ? e.message : String(e)}`);
      return;
    }
    setError(null);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    el.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(PALETTE.ground);
    const spanX = grid.W * grid.res;
    const spanY = grid.H * grid.res;
    const span = Math.max(spanX, spanY, 1);
    const cx = grid.ox + spanX / 2;
    const cy = grid.oy + spanY / 2;

    const camera = new THREE.PerspectiveCamera(40, 1, 0.05, span * 20);
    camera.up.set(0, 0, 1);
    camera.position.set(cx - 0.7 * span, cy - 1.2 * span, 1.2 * span);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(cx, cy, 0);
    // Keep the view on the arena: never at or below the horizon, never inside a box or lost
    // in the distance, and the orbit centre (moved by panning) on the arena, not under the floor.
    controls.maxPolarAngle = Math.PI / 2 - MIN_ELEVATION_RAD;
    controls.minDistance = Math.max(0.5, 0.05 * span);
    controls.maxDistance = 3 * span;
    controls.update();
    const lo = new THREE.Vector3(grid.ox - 1, grid.oy - 1, 0);
    const hi = new THREE.Vector3(grid.ox + spanX + 1, grid.oy + spanY + 1, Math.max(3, 0.25 * span));
    const shift = new THREE.Vector3();
    const keepTargetOnArena = () => {
      shift.copy(controls.target).clamp(lo, hi).sub(controls.target);
      if (shift.lengthSq() === 0) return;
      controls.target.add(shift);
      camera.position.add(shift);
    };

    // three r155+ lights are physical; ×π matches the mockup's legacy-light look.
    scene.add(new THREE.HemisphereLight(0xdfe8f5, 0x20242b, 0.75 * Math.PI));
    const sun = new THREE.DirectionalLight(0xffffff, 0.75 * Math.PI);
    sun.position.set(cx - 0.55 * span, cy - 0.35 * span, 1.3 * span);
    sun.target.position.set(cx, cy, 0);
    sun.castShadow = true;
    const half = span / 2 + 1;
    Object.assign(sun.shadow.camera, { left: -half, right: half, top: half, bottom: -half, far: span * 4 });
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.bias = -0.0005;
    scene.add(sun, sun.target);

    const floorGeo = new THREE.PlaneGeometry(spanX + 2, spanY + 2);
    const floorMat = new THREE.MeshStandardMaterial({ color: PALETTE.floor, roughness: 1 });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.position.set(cx, cy, 0);
    floor.receiveShadow = true;
    scene.add(floor);

    const group = new THREE.Group();
    scene.add(group);
    const box = new THREE.BoxGeometry(1, 1, 1);
    const wallMat = new THREE.MeshStandardMaterial({ color: PALETTE.wall, roughness: 0.7 });
    // Shapes take their colour per instance (shape or selected), so the material stays white.
    const shapeMat = new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.6 });

    const render = () => renderer.render(scene, camera);
    const onChange = () => { keepTargetOnArena(); render(); };
    controls.addEventListener('change', onChange);
    const resize = () => {
      const w = el.clientWidth;
      const h = el.clientHeight;
      if (!w || !h) return;
      renderer.setSize(w, h);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      render();
    };
    const ro = new ResizeObserver(resize);
    ro.observe(el);
    three.current = { renderer, scene, camera, group, box, wallMat, shapeMat, render };
    resize();

    return () => {
      ro.disconnect();
      controls.removeEventListener('change', onChange);
      controls.dispose();
      clearGroup(group);
      box.dispose();
      wallMat.dispose();
      shapeMat.dispose();
      floorGeo.dispose();
      floorMat.dispose();
      sun.shadow.map?.dispose();
      renderer.forceContextLoss();
      renderer.dispose();
      renderer.domElement.remove();
      three.current = null;
    };
  }, [grid]);

  // Boxes: rebuilt on the next frame after any edit, so a brush drag rebuilds once per frame.
  useEffect(() => {
    if (frame.current) return;
    frame.current = requestAnimationFrame(() => {
      frame.current = 0;
      const t = three.current;
      if (!t) return;
      const { grid: g, edit: s } = latest.current;
      const b = boxesFor(s, g);
      clearGroup(t.group);
      // Walls at the wall height, then the strips raised behind taller shapes, each at its own height.
      const wallRects = [
        ...b.walls.map((rect) => ({ rect, h: s.wallHeight })),
        ...b.raised.flatMap(({ h, rects }) => rects.map((rect) => ({ rect, h }))),
      ];
      const walls = instanced(g, wallRects, t.box, t.wallMat);
      const shapeRects = Object.entries(b.shapes).flatMap(([id, rects]) => {
        const h = s.shapes[Number(id)].height;
        const color = Number(id) === s.selected ? selColor : shapeColor;
        return rects.map((rect) => ({ rect, h, color }));
      });
      const shapes = instanced(g, shapeRects, t.box, t.shapeMat);
      if (walls) t.group.add(walls);
      if (shapes) t.group.add(shapes);
      const n = wallRects.length + shapeRects.length;
      setStats(`${n} boxes · walls ${s.wallHeight.toFixed(2)} m`);
      t.render();
    });
  }, [grid, edit]);

  useEffect(() => () => {
    if (frame.current) cancelAnimationFrame(frame.current);
    frame.current = 0;
  }, []);

  return (
    <section className="ma-pane">
      <h2>3D preview <span>{stats}</span></h2>
      <div className="ma-stage">
        <div className="ma-view3d" ref={host} />
        {error && <div className="ma-center">{error}</div>}
        <div className="ma-legend"><span>drag: orbit · right-drag: pan · wheel: zoom</span></div>
      </div>
    </section>
  );
}
