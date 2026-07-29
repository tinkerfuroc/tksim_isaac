#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import traceback
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=10_000)
    args = parser.parse_args()

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "disable_viewport_updates": True,
            "extra_args": ["--/physics/useGpu=false", "--/physics/cudaDevice=-1"],
        }
    )
    started = time.monotonic()
    try:
        from isaacsim.core.api import World

        world = World(stage_units_in_meters=1.0, backend="torch", device="cpu")
        world.scene.add_default_ground_plane()
        world.reset()
        for _ in range(args.steps):
            world.step(render=False)
        result = {
            "steps": args.steps,
            "elapsed_seconds": time.monotonic() - started,
            "simulation_time": float(world.current_time),
            "physics_device": str(world.device),
            "gpu_physics": False,
            "shutdown": "verified_then_process_exit",
        }
        result_path = Path("reports/headless-physx-latest.json")
        result_path.parent.mkdir(parents=True, exist_ok=True)
        result_path.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps(result, sort_keys=True), flush=True)
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
