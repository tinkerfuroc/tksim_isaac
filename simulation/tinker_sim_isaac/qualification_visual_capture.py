from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, Callable, Mapping


CAMERAS = {
    "overview": {
        # Wide, low-angle context for the arm, TCP/gripper, and task cube.
        # The extra distance keeps the pre-close TCP near x=0.095 and the
        # cube near x=0.65 in the same frame.
        "eye": [2.40, 2.40, 1.90],
        "target": [0.35, 0.0, 0.55],
    },
    "manipulation_closeup": {
        # Keep the task region centered between the observed pre-close TCP
        # ([0.095, 0, 0.273]) and cube ([0.65, 0, ~0.0]) instead of aiming
        # above the cube, which previously excluded both from the image.
        "eye": [1.75, 1.50, 1.25],
        "target": [0.35, 0.0, 0.38],
    },
}

# The request journal is written by the ROS-side executor while this process
# services it through the render/event-pump boundary.  Four physics frames is
# the observed upper bound for that handoff; keep the value explicit and
# bounded rather than turning capture freshness into a wall-clock timeout.
MAX_CAPTURE_LATENCY_FRAMES = 4
CAPTURE_LATENCY_CONTRACT = {
    "unit": "physics_frames",
    "max_frames": MAX_CAPTURE_LATENCY_FRAMES,
    "basis": "raw_frame_index-requested_physics_frame_index",
}


def _json_lines(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    records: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError("visual capture requests must be JSON objects")
        records.append(value)
    return records


class QualificationVisualCapture:
    """Capture sparse RGB evidence without advancing the explicit PhysX loop."""

    def __init__(
        self,
        *,
        app: Any,
        backend: Any,
        attempt_dir: Path,
        gate: str,
        event_pump: Callable[[], None] | None = None,
    ) -> None:
        self.app = app
        self.backend = backend
        self.attempt_dir = attempt_dir
        self.gate = gate
        self.event_pump = event_pump
        self.request_path = attempt_dir / "visual-capture-requests.jsonl"
        self.output_dir = attempt_dir / "visual/source"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.records_path = attempt_dir / "visual-keyframes.jsonl"
        self._handled_sequences: set[int] = set()
        self._records: list[dict[str, Any]] = []
        self._errors: list[str] = []
        self._reported_error_keys: set[tuple] = set()
        self._sensors: dict[str, Any] = {}
        self._initialize_cameras()

    @classmethod
    def from_environment(
        cls,
        *,
        app: Any,
        backend: Any,
        event_pump: Callable[[], None] | None = None,
    ) -> "QualificationVisualCapture | None":
        if os.environ.get("TINKER_SIM_VISUAL_EVIDENCE") != "1":
            return None
        attempt = os.environ.get("TINKER_SIM_ATTEMPT_DIR")
        gate = os.environ.get("TINKER_SIM_QUALIFICATION_GATE")
        if not attempt or not gate:
            raise RuntimeError(
                "visual evidence requires TINKER_SIM_ATTEMPT_DIR and "
                "TINKER_SIM_QUALIFICATION_GATE"
            )
        return cls(
            app=app,
            backend=backend,
            attempt_dir=Path(attempt),
            gate=gate,
            event_pump=event_pump,
        )

    def _render_update(self) -> None:
        if self.event_pump is not None:
            self.event_pump()
        self.backend.render_frame()

    def _initialize_cameras(self) -> None:
        from isaacsim.core.rendering_manager import ViewportManager
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.sensors.experimental.rtx")
        for _ in range(4):
            self._render_update()
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

        for name, fixture in CAMERAS.items():
            camera = RtxCamera(f"/World/QualificationCameras/{name}", tick_rate=0.0)
            ViewportManager.set_camera_view(
                camera.paths[0],
                eye=fixture["eye"],
                target=fixture["target"],
            )
            self._sensors[name] = CameraSensor(
                camera,
                resolution=(540, 960),
                annotators=["rgb"],
            )

    @staticmethod
    def _rgb_image(value: Any):
        import numpy as np
        from PIL import Image

        candidate = value
        if hasattr(candidate, "cpu"):
            candidate = candidate.cpu()
        if hasattr(candidate, "numpy"):
            candidate = candidate.numpy()
        array = np.asarray(candidate)
        if array.ndim == 4 and array.shape[0] == 1:
            array = array[0]
        if array.ndim != 3 or array.shape[:2] != (540, 960):
            raise ValueError(f"unexpected RGB frame shape: {array.shape}")
        if array.shape[2] < 3:
            raise ValueError(f"RGB frame has fewer than three channels: {array.shape}")
        array = array[:, :, :3]
        if array.dtype.kind == "f":
            if not np.isfinite(array).all():
                raise ValueError("RGB frame contains non-finite values")
            scale = 255.0 if float(array.max(initial=0.0)) <= 1.0 else 1.0
            array = np.clip(array * scale, 0.0, 255.0).astype(np.uint8)
        else:
            array = np.clip(array, 0, 255).astype(np.uint8)
        return Image.fromarray(array, mode="RGB")

    def _capture_request(self, request: Mapping[str, Any]) -> None:
        sequence = int(request["sequence"])
        event = str(request["event"])
        requested_time = float(request["simulated_timestamp"])
        if request.get("gate") != self.gate:
            raise ValueError(
                f"capture request gate {request.get('gate')!r} does not match {self.gate!r}"
            )
        if not math.isfinite(requested_time):
            raise ValueError("capture request timestamp must be finite")

        physics_dt = float(self.backend.dt)
        if not math.isfinite(physics_dt) or physics_dt <= 0:
            raise ValueError("backend physics dt must be finite and positive")
        requested_frame_index = int(math.floor(requested_time / physics_dt + 0.5))
        captured_frame_index = int(self.backend.physics_frame_index)
        capture_latency_frames = captured_frame_index - requested_frame_index
        if not 0 <= capture_latency_frames <= MAX_CAPTURE_LATENCY_FRAMES:
            raise ValueError(
                "capture latency is outside the bounded contract: "
                f"{capture_latency_frames} frames"
            )

        for _ in range(2):
            self._render_update()
        for camera_name, sensor in self._sensors.items():
            rgb = None
            sensor_info: Mapping[str, Any] = {}
            for _ in range(30):
                rgb, sensor_info = sensor.get_data("rgb")
                if rgb is not None:
                    break
                self._render_update()
            if rgb is None:
                raise RuntimeError(f"{camera_name} produced no RGB frame")
            image = self._rgb_image(rgb)
            relative = Path("visual/source") / (
                f"{sequence:04d}-{event}-{camera_name}.png"
            )
            output = self.attempt_dir / relative
            image.save(output, format="PNG", optimize=False)
            record = {
                    "schema_version": 1,
                    "gate": self.gate,
                    "event": event,
                    "request_sequence": sequence,
                    "execution_event_sequence": request.get(
                        "source_execution_event_sequence"
                    ),
                    "requested_simulated_timestamp": requested_time,
                    "requested_physics_frame_index": requested_frame_index,
                    "capture_latency_frames": capture_latency_frames,
                    "max_capture_latency_frames": MAX_CAPTURE_LATENCY_FRAMES,
                    "simulated_timestamp": float(self.backend.simulation_time),
                    "sensor_info_keys": sorted(str(key) for key in sensor_info),
                    "raw_frame_index": captured_frame_index,
                    "physics_dt": physics_dt,
                    "camera": camera_name,
                    "camera_fixture": CAMERAS[camera_name],
                    "path": relative.as_posix(),
                    "width": 960,
                    "height": 540,
                    "mode": "RGB",
                }
            self._records.append(record)
            with self.records_path.open("a", encoding="utf-8") as stream:
                stream.write(
                    json.dumps(record, sort_keys=True, separators=(",", ":"))
                    + "\n"
                )
                stream.flush()
                os.fsync(stream.fileno())

    @staticmethod
    def _is_executor_diagnostic(record: Mapping[str, Any]) -> bool:
        """Recognize the exact integrated executor diagnostic record shape.

        ``IntegratedGateExecutor._append_visual_request`` writes
        ``{schema_version, report_revision, scenario_id, phase,
        capture:{kind,target}, diagnostic_only: true}`` with no
        ``sequence``/``gate``/``event``/``simulated_timestamp``.  Such records
        are diagnostic-only evidence: never capture-driving, never a handled
        sequence, never an error.
        """
        return (
            record.get("diagnostic_only") is True
            and not isinstance(record.get("sequence"), int)
            and isinstance(record.get("scenario_id"), str)
            and bool(record["scenario_id"])
            and isinstance(record.get("phase"), str)
            and bool(record["phase"])
            and isinstance(record.get("capture"), Mapping)
        )

    def _record_capture_error_once(self, key: tuple, message: str) -> None:
        """Durably report a malformed/unknown record exactly once (no error loop)."""
        if key in self._reported_error_keys:
            return
        self._reported_error_keys.add(key)
        if message not in self._errors:
            self._errors.append(message)

    def poll(self) -> None:
        try:
            requests = _json_lines(self.request_path)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            message = f"invalid visual capture request stream: {error}"
            if message not in self._errors:
                self._errors.append(message)
            return
        for request in requests:
            if not isinstance(request, Mapping):
                self._record_capture_error_once(
                    ("non-mapping", repr(request)),
                    f"unrecognized visual capture request record (not an object): {request!r}",
                )
                continue
            if self._is_executor_diagnostic(request):
                # F2.3: executor diagnostic records are co-tenanted with the
                # canonical sequence records; skip silently, never error-spam,
                # never fabricate a handled sequence.
                continue
            try:
                sequence = int(request["sequence"])
                requested_time = float(request["simulated_timestamp"])
            except (KeyError, TypeError, ValueError):
                self._record_capture_error_once(
                    ("malformed", json.dumps(request, sort_keys=True, default=str)[:200]),
                    "unrecognized visual capture request record (missing canonical "
                    f"sequence/simulated_timestamp): {json.dumps(request, sort_keys=True, default=str)[:200]}",
                )
                continue
            if sequence in self._handled_sequences:
                continue
            if requested_time > float(self.backend.simulation_time) + 1e-12:
                continue
            try:
                self._capture_request(request)
            except (KeyError, TypeError, ValueError, RuntimeError) as error:
                sequence = request.get("sequence")
                self._errors.append(f"capture request {sequence!r} failed: {error}")
                if isinstance(sequence, int):
                    self._handled_sequences.add(sequence)
                continue
            self._handled_sequences.add(sequence)
            break

    def close(self) -> None:
        payload = {
            "schema_version": 1,
            "gate": self.gate,
            "physics_frame_s": float(self.backend.dt),
            "capture_latency_contract": CAPTURE_LATENCY_CONTRACT,
            "cameras": CAMERAS,
            "records": self._records,
            "errors": self._errors,
        }
        path = self.attempt_dir / "visual-keyframes.json"
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
        for sensor in self._sensors.values():
            close = getattr(sensor, "close", None)
            if callable(close):
                close()
