"""Host test bootstrap: put every package under src/ on sys.path so the pure-Python cores import without
colcon/ROS (`.venv/Scripts/python -m pytest -q`). Nothing here imports rclpy."""

from __future__ import annotations

import sys
import os
from pathlib import Path

WS = Path(__file__).resolve().parent
for pkg in sorted((WS / "src").iterdir()):
    if (pkg / "package.xml").exists() and str(pkg) not in sys.path:
        sys.path.insert(0, str(pkg))

import pytest  # noqa: E402

# the workspace folder that holds the repos side by side (robot/, backend/, docs/, ai/ ...); override with SW_WORKSPACE
REPO = Path(os.environ["SW_WORKSPACE"]) if os.environ.get("SW_WORKSPACE") else WS.parents[1]
WAREHOUSE_MAP = REPO / "backend" / "app" / "warehouse_map.yaml"
SCHEMAS = REPO / "docs" / "schemas"


@pytest.fixture(scope="session")
def ws_root() -> Path:
    return WS


@pytest.fixture(scope="session")
def warehouse_map_path() -> Path:
    """The single source of truth for the lane graph (read-only; owned by backend)."""
    assert WAREHOUSE_MAP.exists(), WAREHOUSE_MAP
    return WAREHOUSE_MAP


@pytest.fixture(scope="session")
def schemas_dir() -> Path:
    return SCHEMAS
