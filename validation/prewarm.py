#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
import traceback
from pathlib import Path


def main() -> int:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    enabled: list[str] = []
    try:
        from isaacsim.core.utils.extensions import enable_extension

        for extension in (
            "isaacsim.ros2.bridge",
            "isaacsim.sensors.rtx",
            "isaacsim.asset.importer.urdf",
        ):
            enable_extension(extension)
            enabled.append(extension)
        for _ in range(120):
            app.update()
        cache = Path(os.environ["ISAACSIM_CACHE_PATH"])
        cache.mkdir(parents=True, exist_ok=True)
        marker = cache / "prewarm.json"
        marker.write_text(
            json.dumps({"isaac_extensions": enabled}, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(marker, flush=True)
    except BaseException:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
