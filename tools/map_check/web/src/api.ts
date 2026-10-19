// Typed client for the Map Check HTTP API (tools/map_check/map_check/server.py).

export class ApiError extends Error {
  status: number;
  detail: string;
  busy_text?: string;
  constructor(status: number, detail: string, busy_text?: string) {
    super(detail);
    this.name = 'ApiError';
    this.status = status;
    this.detail = detail;
    this.busy_text = busy_text;
  }
}

export interface MapsList { studio: string[]; arenas: string[] }

export interface WebGrid { W: number; H: number; res: number; ox: number; oy: number; grid: string }
/** A closed outline (occupancy.Outline): 1-based id, area in m², centre x/y in map metres. */
export interface Outline { id: number; area_m2: number; cx: number; cy: number }
export interface ShapeSetting { fill: boolean; height: number }
export interface MapBody {
  token: string;
  name_suggestion: string;
  identical_to: string | null;
  grid: WebGrid;
  outlines: Outline[];
  defaults: { wall_height: number; shapes: Record<string, ShapeSetting> };
}

export interface SaveRequest {
  token: string;
  name: string;
  wall_height: number;
  shapes: Record<string, ShapeSetting>;
  erased: number[];
}
/** `version_dir` is the absolute path of the new arena version directory. */
export interface SaveResult { name: string; version_dir: string; boxes: number }

export interface Pose { name: string; x: number; y: number; yaw: number }
export interface RowState {
  included: boolean; behind: boolean; top: boolean; head: boolean; pan_deg: number; tilt_deg: number;
}
export interface Flag {
  kind: 'overlap' | 'near' | 'outside';
  part: string; target: string; distance_m: number; text: string;
}
export interface BenchRow {
  key: string;
  group: 'saved' | 'pasted';
  pose: Pose;
  state: RowState;
  clash: string | null;
  clash_m: string | null;
  flags: Flag[];
}
export interface JobStatus {
  id: string; map: string; status: string; queued_at: number;
  started_at: number | null; finished_at: number | null;
  done_views: number; total_views: number; position: number | null; message: string;
}
export interface RenderView { view: string; url: string; caption: string }
export interface RenderItem { key: string; name: string; flags: Flag[]; views: RenderView[] }
export interface LastRender {
  job_id: string; finished_at: number; arm_pose: string; items: RenderItem[]; overview_url: string | null;
}
export interface PasteError { index: number; text: string; reason: string }
export interface BenchBody {
  map: string;
  version: number;
  arena: boolean;
  arm_pose: string;
  arm_poses: Record<string, number[]>;
  tilt_down_sign: number;
  rows: BenchRow[];
  removed_count: number;
  last_render: LastRender | null;
  job: JobStatus | null;
  flags_error: string | null;
  paste_errors?: PasteError[]; // only on the ops response
}

export type RowFields = Partial<RowState>;
export type BenchOp =
  | { op: 'set_row'; row: string; fields: RowFields }
  | { op: 'set_rows'; rows: string[]; fields: RowFields }
  | { op: 'add_pasted'; text: string }
  | { op: 'remove'; row: string }
  | { op: 'restore_saved' }
  | { op: 'clear_pasted' }
  | { op: 'set_arm'; name: string };

export interface GpuStatus { busy: boolean; text: string }

async function fail(res: Response): Promise<never> {
  let detail = `${res.status} ${res.statusText}`;
  let busy: string | undefined;
  try {
    const j = await res.json();
    if (typeof j?.detail === 'string') detail = j.detail;
    else if (j?.detail !== undefined) detail = JSON.stringify(j.detail);
    if (typeof j?.busy_text === 'string') busy = j.busy_text;
  } catch { /* body was not JSON; keep the status line */ }
  throw new ApiError(res.status, detail, busy);
}

async function json<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(path, init);
  if (!res.ok) return fail(res);
  return (await res.json()) as T;
}

const post = (body: unknown): RequestInit => ({
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify(body),
});
const enc = encodeURIComponent;

export const getMaps = () => json<MapsList>('/api/maps');

export function uploadMap(yaml: File, image: File): Promise<MapBody> {
  const form = new FormData();
  form.append('yaml', yaml);
  form.append('image', image);
  return json<MapBody>('/api/map/upload', { method: 'POST', body: form });
}

export const getStudioMap = (name: string) => json<MapBody>(`/api/map/studio/${enc(name)}`);
export const saveMap = (req: SaveRequest) => json<SaveResult>('/api/map/save', post(req));
export const getBench = (map: string) => json<BenchBody>(`/api/bench/${enc(map)}`);
export const benchOps = (map: string, ops: BenchOp[]) =>
  json<BenchBody>(`/api/bench/${enc(map)}/ops`, post({ ops }));

export async function exportList(
  map: string, format: 'code' | 'yaml' = 'code', which: 'all' | 'ticked' = 'all',
): Promise<string> {
  const res = await fetch(`/api/bench/${enc(map)}/export?format=${format}&which=${which}`);
  if (!res.ok) return fail(res);
  return res.text();
}

export const getGpu = () => json<GpuStatus>('/api/gpu');
/** Throws ApiError with status 409 and busy_text when the GPU is busy and override is false. */
export const startRender = (map: string, override = false) =>
  json<JobStatus>(`/api/bench/${enc(map)}/render`, post({ override }));
