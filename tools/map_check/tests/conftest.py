import os
from pathlib import Path

import pytest


@pytest.fixture
def real_root() -> Path:
    """Checkout holding the git-ignored real data (artifacts/, .venv)."""
    return Path(os.environ.get("MAP_CHECK_REAL_ROOT", "/home/tinker/tinker-sim/6.0.1"))
