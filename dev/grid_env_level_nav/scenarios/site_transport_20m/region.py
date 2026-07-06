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


def is_local_xy_in_work_region(
    lx: float,
    ly: float,
    *,
    margin_cm: float = 0.0,
) -> bool:
    """True when local XY lies inside the 20 m × 20 m site transport bounds."""
    lo = margin_cm
    hi = REGION_SIZE_CM - margin_cm
    return lo <= lx <= hi and lo <= ly <= hi
