#!/usr/bin/env python3
"""30 m × 30 m compact navigation work region (local origin = Level work origin)."""

from __future__ import annotations

import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from level_coords import REGION_ORIGIN_WORLD_XY
from paths import L0_MASK_STRICT

REGION_SIZE_CM = 3000.0
GOAL_REGION_MAX_CM = 2500.0
ROBOT_START_LOCAL_CM = (100.0, 100.0)
GOAL_LOCAL_CM = (2500.0, 2500.0)
DEFAULT_L0_PATH = L0_MASK_STRICT
DEFAULT_RESOLUTION_CM = 30.0
