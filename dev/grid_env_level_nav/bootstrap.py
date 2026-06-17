#!/usr/bin/env python3
"""Insert grid_env_level_nav (+ deps) on sys.path for scripts and scenarios."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

PKG_DIR = Path(__file__).resolve().parent
REPO_ROOT = PKG_DIR.parent.parent


def setup_paths(*, scenario: Optional[str] = None) -> Path:
    """Return scenario directory (or PKG_DIR) after path bootstrap."""
    scenario_dir = PKG_DIR / "scenarios" / scenario if scenario else PKG_DIR
    ordered = [
        REPO_ROOT,
        PKG_DIR,
        scenario_dir,
        REPO_ROOT / "dev" / "grid_env_hri",
        REPO_ROOT / "dev" / "grid_env_10k",
        REPO_ROOT / "dev" / "grid_env_depth_perception",
    ]
    for entry in ordered:
        text = str(entry)
        if text not in sys.path:
            sys.path.insert(0, text)
    return scenario_dir
