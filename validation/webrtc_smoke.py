#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


MARKERS = (
    "omni.kit.livestream.webrtc-",
    "isaacsim.exp.full.streaming-",
)


def _stop_process_group(process: subprocess.Popen[bytes]) -> str:
    if process.poll() is not None:
        return "exited"
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20)
        return "supervisor_sigint"
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=10)
        return "supervisor_sigterm"
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=10)
        return "supervisor_sigkill"


def main() -> int:
    reports = Path("reports")
    reports.mkdir(parents=True, exist_ok=True)
    log_path = reports / "webrtc-startup-latest.log"
    result_path = reports / "webrtc-startup-latest.json"
    command = [
        "isaacsim",
        "isaacsim.exp.full.streaming",
        "--no-window",
        "--/app/livestream/allowResize=false",
        "--/physics/useGpu=false",
        "--/physics/cudaDevice=-1",
    ]
    deadline = time.monotonic() + 120
    matched: set[str] = set()

    with log_path.open("wb") as log:
        process = subprocess.Popen(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        read_offset = 0
        while time.monotonic() < deadline:
            log.flush()
            with log_path.open("rb") as reader:
                reader.seek(read_offset)
                new_output = reader.read()
                read_offset = reader.tell()
            decoded = new_output.decode("utf-8", errors="replace")
            for marker in MARKERS:
                if marker in decoded:
                    matched.add(marker)
            if len(matched) == len(MARKERS):
                break
            if process.poll() is not None:
                break
            time.sleep(0.25)
        shutdown = _stop_process_group(process)

    result = {
        "command": command,
        "log": str(log_path),
        "markers": {marker: marker in matched for marker in MARKERS},
        "returncode_after_supervisor_stop": process.returncode,
        "shutdown": shutdown,
    }
    result_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, sort_keys=True), flush=True)
    if len(matched) != len(MARKERS):
        print(
            f"WebRTC startup markers were not observed; inspect {log_path}",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
