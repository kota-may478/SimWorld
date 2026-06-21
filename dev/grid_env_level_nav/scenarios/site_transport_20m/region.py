#!/usr/bin/env python3
"""20 m × 20 m site transport work region."""

from __future__ import annotations

from level_coords import REGION_ORIGIN_WORLD_XY
from paths import L0_MASK_STRICT

REGION_SIZE_CM = 2000.0
ROBOT_START_LOCAL_CM = (100.0, 100.0)
HUMANOID_LOCAL_CM = (100.0, 30.0)
MATERIAL_YARD_CORNER_CM = (2000.0, 2000.0)
DEFAULT_L0_PATH = L0_MASK_STRICT
DEFAULT_RESOLUTION_CM = 30.0
LAYOUT_ID = "layout_01"
