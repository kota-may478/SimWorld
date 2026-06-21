#!/usr/bin/env python3
"""Release stale UnrealCV TCP sessions. Delegates to grid_env_level_semantic/release_ue_connection.py."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

_TARGET = Path(__file__).resolve().parent.parent / "grid_env_level_semantic" / "release_ue_connection.py"

if __name__ == "__main__":
    raise SystemExit(subprocess.call([sys.executable, str(_TARGET)]))
