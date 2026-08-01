#!/usr/bin/env bash
# Print the absolute path of the canonical simulator full URDF selected through
# the authoritative current.json resolver.  Used by the reproducible model-bundle
# acceptance command documented in ros2_ws/src/tinker_sim_bridge/README.md.
set -eo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

"${project_root}/.venv/bin/python" -c "
import sys
from pathlib import Path
sys.path.insert(0, str(Path('${project_root}/ros2_ws/src/tinker_sim_bridge')))
from tinker_sim_bridge.model_bundle import resolve_simulator_full_urdf
print(resolve_simulator_full_urdf(Path('${project_root}')))
"
