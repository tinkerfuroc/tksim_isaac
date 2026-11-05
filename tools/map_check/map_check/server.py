"""Map Check HTTP server: JSON API over the Python core, an SSE stream, and the web UI."""
from __future__ import annotations

import asyncio
import atexit
import contextlib
import dataclasses
import json
import io
import re
import secrets
import signal
import threading
import time
import warnings
from dataclasses import asdict, dataclass
from pathlib import Path
from urllib.parse import urlsplit

from fastapi import FastAPI, File, HTTPException, Request, UploadFile
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse, PlainTextResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image
from pydantic import BaseModel
import uvicorn

from . import robot as robot_mod
from .arena_build import BuildSettings, ShapeSetting, build_boxes, default_settings, web_grid
from .arena_store import ArenaStore, NameTaken, check_image_suffix
from .bench import BenchStore
from .gpu import describe, gpu_status
from .jobs import GpuBusy, JobRunner, default_isaac_command
from .names import check_map_name
from .occupancy import MapFileError, closed_outlines, load_map_dir, parse_map
from .overlap import RobotModel, arena_bounds
from .poses import export_code, export_yaml
from .render.cameras import TILT_DOWN_SIGN

LOCAL_HOSTNAMES = ('localhost', '127.0.0.1', '::1')
TOKEN_TTL_S = 3600
MAX_UPLOAD_BYTES = 64 * 1024 * 1024
MAX_HEIGHT_M = 10.0
SSE_PING_S = 15.0
FLAG_CACHE_MAX = 4096
MAX_TOKENS = 16
GRACEFUL_SHUTDOWN_S = 5


def origin_allowed(origin, host):
    """Copied from Map Studio. Browser requests carry Origin; they must come from this
    server's own page, addressed by a local name. That refuses other websites and DNS
    rebinding. Requests without Origin (curl, scripts) are already local."""
    if origin is None:
        return True
    if not host:
        return False
    o = urlsplit(origin)
    if o.scheme not in ('http', 'https') or o.netloc.lower() != host.lower():
        return False
    return urlsplit('//' + host).hostname in LOCAL_HOSTNAMES


class ShapeIn(BaseModel):
    fill: bool
    height: float


class SaveIn(BaseModel):
    token: str
    name: str
    wall_height: float
    shapes: dict[str, ShapeIn] = {}
    erased: list[int] = []


class OpsIn(BaseModel):
    ops: list


class RenderIn(BaseModel):
    override: bool = False


@dataclass
class _Held:
    expires: float
    kind: str  # "upload" | "studio"
    name: str  # suggested name (upload) or the Map Studio map's name (studio)
    m: object
    identical_to: str | None = None


def _suggest(filename: str | None) -> str:
    stem = Path(filename or '').name
    stem = stem[:-len('.yaml')] if stem.endswith('.yaml') else Path(stem).stem
    stem = re.sub(r'[^A-Za-z0-9_.-]', '_', stem).lstrip('.-')
    try:
        return check_map_name(stem)
    except ValueError:
        return 'map'


def _height(v: float, what: str) -> float:
    if not 0 < v <= MAX_HEIGHT_M:
        raise HTTPException(400, f'{what} must be above 0 and at most {MAX_HEIGHT_M:g} m')
    return float(v)


def create_app(settings, job_command_factory=default_isaac_command, robot_model=None, *,
               gpu_probe=gpu_status, poll_interval: float = 0.5, web_dir=None,
               clock=time.time) -> FastAPI:
    store = ArenaStore(settings)
    bench = BenchStore(settings.state_dir)
    renders_dir = Path(settings.state_dir) / 'renders'
    renders_dir.mkdir(parents=True, exist_ok=True)
    stopping = threading.Event()
    held: dict[str, _Held] = {}
    held_lock = threading.Lock()
    flag_cache: dict = {}
    model_lock = threading.Lock()
    models: list = []  # the lazily built real model, once it exists

    @contextlib.asynccontextmanager
    async def lifespan(app):
        stopping.clear()
        runner = JobRunner(settings, bench, job_command_factory, gpu_probe=gpu_probe,
                           robot_model=robot_model)
        app.state.runner = runner
        atexit.register(runner.shutdown)  # a forced exit skips the lifespan
        try:
            yield
        finally:
            stopping.set()
            runner.shutdown()

    app = FastAPI(title='Map Check', lifespan=lifespan)
    app.state.bench = bench
    app.state.stopping = stopping

    @app.middleware('http')
    async def same_origin_only(request, call_next):
        if not origin_allowed(request.headers.get('origin'), request.headers.get('host')):
            return PlainTextResponse('cross-origin request refused', status_code=403)
        return await call_next(request)

    # -- errors: always {"detail": str} --
    @app.exception_handler(RequestValidationError)
    async def _validation(request, exc):
        msg = '; '.join(f"{'.'.join(str(p) for p in e['loc'])}: {e['msg']}" for e in exc.errors())
        return JSONResponse({'detail': msg}, status_code=422)

    @app.exception_handler(NameTaken)
    async def _taken(request, exc):
        return JSONResponse({'detail': str(exc)}, status_code=409)

    @app.exception_handler(MapFileError)
    async def _map_file(request, exc):
        return JSONResponse({'detail': str(exc)}, status_code=422)

    @app.exception_handler(ValueError)
    async def _bad_value(request, exc):
        return JSONResponse({'detail': str(exc)}, status_code=400)

    # -- upload tokens --
    def hold(entry: _Held) -> str:
        token = secrets.token_urlsafe(16)
        with held_lock:
            now = clock()
            for t in [t for t, h in held.items() if h.expires <= now]:
                del held[t]
            while len(held) >= MAX_TOKENS:
                del held[next(iter(held))]  # oldest first
            entry.expires = now + TOKEN_TTL_S
            held[token] = entry
        return token

    def held_get(token: str) -> _Held:
        now = clock()
        with held_lock:
            for t in [t for t, h in held.items() if h.expires <= now]:
                del held[t]
            entry = held.get(token)
        if entry is None:
            raise HTTPException(404, 'unknown or expired map token; open the map again')
        return entry

    def map_body(token, name, m, identical_to):
        labels, outlines = closed_outlines(m)
        d = default_settings(m, outlines, settings.default_wall_height,
                             settings.default_shape_height, settings.default_fill_area)
        return {
            'token': token, 'name_suggestion': name, 'identical_to': identical_to,
            'grid': web_grid(m, labels), 'outlines': [asdict(o) for o in outlines],
            'defaults': {'wall_height': d.wall_height,
                         'shapes': {str(k): asdict(s) for k, s in d.shapes.items()}},
        }

    # -- maps --
    @app.get('/api/maps')
    def list_maps():
        return {'studio': store.map_names(), 'arenas': store.arenas()}

    @app.post('/api/map/upload')
    def upload_map(yaml: UploadFile = File(...), image: UploadFile = File(...)):
        yaml_bytes = yaml.file.read(MAX_UPLOAD_BYTES + 1)
        image_bytes = image.file.read(MAX_UPLOAD_BYTES + 1)
        if len(yaml_bytes) > MAX_UPLOAD_BYTES or len(image_bytes) > MAX_UPLOAD_BYTES:
            raise HTTPException(413, 'file too large')
        _check_image_size(image_bytes)
        files = {Path((image.filename or '').replace('\\', '/')).name: image_bytes}
        m = parse_map(yaml_bytes, files)
        check_image_suffix(m.image_name)  # refused now, not after the teammate's edits
        identical = store.identical_existing(yaml_bytes, files)
        name = _suggest(yaml.filename)
        token = hold(_Held(0, 'upload', name, m, identical))
        return map_body(token, name, m, identical)

    @app.get('/api/map/studio/{name}')
    def pick_studio_map(name: str):
        check_map_name(name)
        path = Path(settings.maps_dir) / f'{name}.yaml'
        if not path.is_file():
            raise HTTPException(404, f'no map {name!r} in the Map Studio maps')
        m = load_map_dir(path)
        token = hold(_Held(0, 'studio', name, m))
        return map_body(token, name, m, None)

    @app.post('/api/map/save')
    def save_map(body: SaveIn):
        entry = held_get(body.token)
        name = check_map_name(body.name)
        m = entry.m
        if entry.kind == 'studio' and name != entry.name:
            raise HTTPException(400, f"a picked map is saved under its own name '{entry.name}'")
        if entry.kind == 'upload':
            if (Path(settings.maps_dir) / f'{name}.yaml').exists():
                raise NameTaken(f"a map named '{name}' already exists in {settings.maps_dir}")
            if (Path(settings.arena_root) / name).exists():
                raise NameTaken(f"an arena named '{name}' already exists in {settings.arena_root}")
        labels, outlines = closed_outlines(m)
        build = default_settings(m, outlines, settings.default_wall_height,
                                 settings.default_shape_height, settings.default_fill_area)
        shapes = dict(build.shapes)
        for k, s in body.shapes.items():
            try:
                sid = int(k)
            except ValueError:
                raise HTTPException(400, f'shape id {k!r} is not a number')
            if sid not in shapes:
                raise HTTPException(400, f'there is no outline {sid} in this map')
            shapes[sid] = ShapeSetting(fill=s.fill, height=_height(s.height, f'shape {sid} height'))
        n_cells = m.occupied.size
        flat = m.occupied.reshape(-1)
        for i in body.erased:
            if not 0 <= i < n_cells or not flat[i]:
                raise HTTPException(400, f'erased cell {i} is not a wall cell of this map')
        used = BuildSettings(wall_height=_height(body.wall_height, 'wall height'), shapes=shapes,
                             erased=frozenset(body.erased))
        boxes = build_boxes(m, labels, used)
        if entry.kind == 'upload':
            store.save_uploaded_map(name, m.yaml_bytes, {m.image_name: m.image_bytes})
        version = store.save_arena(name, used, boxes, m)
        return {'name': name, 'version_dir': str(version.version_dir), 'boxes': len(boxes)}

    # -- bench --
    def real_model():
        with model_lock:
            if not models:
                files = robot_mod.resolve_robot(settings.robot_pointer)
                models.append(RobotModel(files.urdf, files.usd, Path(settings.state_dir) / 'cache',
                                         repo_root=settings.repo_root,
                                         margin=settings.overlap_margin))
            return models[0]

    def bench_body(map_name: str) -> dict:
        st = bench.get(map_name)
        view = bench.view(map_name, settings.maps_dir)
        errors = []
        try:
            arms = robot_mod.arm_poses(settings.arm_poses_json)
        except ValueError as e:
            arms, errors = {}, [str(e)]
        try:
            _, boxes, arena_dir = store.load_current(map_name)
            bounds = arena_bounds(boxes) if boxes else None
        except (OSError, ValueError, KeyError, TypeError):
            boxes = arena_dir = bounds = None
        model = None
        if boxes and bounds and st.arm_pose in arms:
            try:
                model = robot_model if robot_model is not None else real_model()
            except (OSError, ValueError) as e:
                errors.append(f'overlap check unavailable: {e}')
        elif boxes and st.arm_pose not in arms:
            errors.append(f'unknown arm pose {st.arm_pose!r}')
        rows = []
        for r in view.rows:
            flags = []
            if model is not None:
                s, p = r.state, r.pose
                ck = (str(arena_dir), p.x, p.y, p.yaw, tuple(arms[st.arm_pose]), s.pan_deg, s.tilt_deg)
                try:
                    with model_lock:
                        if ck not in flag_cache:
                            if len(flag_cache) >= FLAG_CACHE_MAX:
                                flag_cache.clear()
                            flag_cache[ck] = model.flags(p, arms[st.arm_pose], s.pan_deg,
                                                         s.tilt_deg, boxes, bounds)
                        flags = flag_cache[ck]
                except Exception as e:  # noqa: BLE001 - report it, keep the rest of the bench
                    errors.append(f'overlap check failed for {r.pose.name}: {type(e).__name__}: {e}')
                    model = None
            rows.append({'key': r.key, 'group': r.group, 'pose': asdict(r.pose),
                         'state': asdict(r.state), 'clash': r.clash_m, 'clash_m': r.clash_m,
                         'flags': [asdict(f) for f in flags]})
        return {
            'map': map_name, 'version': st.version, 'arena': boxes is not None,
            'arm_pose': st.arm_pose, 'arm_poses': arms, 'tilt_down_sign': TILT_DOWN_SIGN,
            'rows': rows, 'removed_count': view.removed_count,
            'last_render': st.last_render.to_dict() if st.last_render else None,
            'job': st.job.to_dict() if st.job else None,
            'flags_error': '; '.join(errors) or None,
        }

    @app.get('/api/bench/{map_name}')
    def get_bench(map_name: str):
        check_map_name(map_name)
        return bench_body(map_name)

    @app.post('/api/bench/{map_name}/ops')
    def bench_ops(map_name: str, body: OpsIn):
        check_map_name(map_name)
        _, errors = bench.apply(map_name, body.ops)
        out = bench_body(map_name)
        out['paste_errors'] = [asdict(e) for e in errors]
        return out

    @app.get('/api/bench/{map_name}/export')
    def export(map_name: str, format: str = 'code', which: str = 'all'):
        check_map_name(map_name)
        if format not in ('code', 'yaml'):
            raise HTTPException(400, "format must be 'code' or 'yaml'")
        if which not in ('all', 'ticked'):
            raise HTTPException(400, "which must be 'all' or 'ticked'")
        view = bench.view(map_name, settings.maps_dir)
        poses = [r.pose for r in view.rows if which == 'all' or r.state.included]
        text = export_code(poses) if format == 'code' else export_yaml(map_name, poses)
        return PlainTextResponse(text)

    # -- GPU and render --
    @app.get('/api/gpu')
    def gpu():
        busy = gpu_probe(own_pids=app.state.runner._own_pids()).busy
        return {'busy': bool(busy), 'text': describe(busy)}

    @app.post('/api/bench/{map_name}/render')
    def render(map_name: str, body: RenderIn):
        try:
            return app.state.runner.submit(map_name, body.override).to_dict()
        except GpuBusy as e:
            return JSONResponse({'detail': str(e), 'busy_text': describe(e.reasons)},
                                status_code=409)

    # -- events --
    @app.get('/api/events')
    async def events(request: Request, map: str, max_events: int | None = None):
        # max_events exists so tests can read a bounded number of events.
        check_map_name(map)

        async def gen():
            sent, last_version, last_job = 0, None, None
            last_ping = time.monotonic()
            while not stopping.is_set():
                st = await asyncio.to_thread(bench.get, map)
                job = st.job.to_dict() if st.job else None
                out = []
                if st.version != last_version:
                    last_version = st.version
                    out.append(f'event: bench\ndata: {st.version}\n\n')
                if job is not None and job != last_job:
                    out.append(f'event: job\ndata: {json.dumps(job)}\n\n')
                last_job = job
                for chunk in out:
                    yield chunk
                    sent += 1
                    if max_events is not None and sent >= max_events:
                        return
                if time.monotonic() - last_ping > SSE_PING_S:
                    last_ping = time.monotonic()
                    yield ': ping\n\n'
                if await request.is_disconnected():
                    return
                await asyncio.sleep(poll_interval)

        return StreamingResponse(gen(), media_type='text/event-stream',
                                 headers={'Cache-Control': 'no-cache',
                                          'X-Accel-Buffering': 'no'})

    app.mount('/renders', StaticFiles(directory=renders_dir), name='renders')
    _mount_ui(app, web_dir if web_dir is not None else _default_web_dir())
    return app


def _check_image_size(data: bytes) -> None:
    """Refuse decompression bombs with a 400 before the image is decoded."""
    try:
        with warnings.catch_warnings():
            warnings.simplefilter('error', Image.DecompressionBombWarning)
            Image.open(io.BytesIO(data)).size
    except (Image.DecompressionBombError, Image.DecompressionBombWarning) as e:
        raise HTTPException(400, f'map image is too large to open safely: {e}')
    except Exception:  # noqa: BLE001 - other unreadable images are reported by parse_map
        pass


LOOPBACK_HOSTS = ('127.0.0.1', '::1', 'localhost')


class _Server(uvicorn.Server):
    """Ends the event streams and the render job as soon as the server is told to exit."""

    def __init__(self, config, app):
        super().__init__(config)
        self._app = app

    @contextlib.contextmanager
    def capture_signals(self):
        """uvicorn's (0.54) handling, plus SIGHUP: a dropped ssh session must stop the render too.

        A SIGHUP ignored on entry (nohup) stays ignored.
        """
        if threading.current_thread() is not threading.main_thread():
            yield
            return
        sigs = [signal.SIGINT, signal.SIGTERM]
        if signal.getsignal(signal.SIGHUP) is not signal.SIG_IGN:
            sigs.append(signal.SIGHUP)
        original = {sig: signal.signal(sig, self.handle_exit) for sig in sigs}
        try:
            yield
        finally:
            for sig, handler in original.items():
                signal.signal(sig, handler)
        # As uvicorn does: after the graceful shutdown, re-raise for the default behaviour.
        for captured in reversed(self._captured_signals):
            signal.raise_signal(captured)

    def handle_exit(self, sig, frame):
        again = self.should_exit
        self._app.state.stopping.set()  # open SSE streams would block uvicorn's shutdown
        if again:
            self.stop_runner()  # a second signal forces exit and skips the lifespan
        super().handle_exit(sig, frame)

    def stop_runner(self):
        runner = getattr(self._app.state, 'runner', None)
        if runner is not None:
            runner.shutdown()


def serve(app, host: str, port: int) -> None:
    server = _Server(uvicorn.Config(app, host=host, port=port,
                                    timeout_graceful_shutdown=GRACEFUL_SHUTDOWN_S), app)
    try:
        server.run()
    finally:
        server.stop_runner()  # idempotent; the lifespan normally did this already


def _mount_ui(app, web_dir):
    # Mounted last so every /api route above wins.
    if web_dir is not None and (Path(web_dir) / 'index.html').is_file():
        app.mount('/', StaticFiles(directory=web_dir, html=True), name='web')
        return

    @app.get('/')
    def no_ui():
        return PlainTextResponse(
            'Map Check API is running, but there is no web UI build '
            '(tools/map_check/web/dist). Build it with `npm run build`.')


def _default_web_dir():
    return Path(__file__).resolve().parents[1] / 'web' / 'dist'
