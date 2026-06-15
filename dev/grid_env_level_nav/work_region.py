#!/usr/bin/env python3
"""Work region geometry for /Game/Maps/Level (local origin at world (-1000, -2200))."""

from __future__ import annotations

import math
from typing import Tuple

from level_coords import REGION_ORIGIN_WORLD_XY, WorldXY

REGION_SIZE_X_CM = 7000.0
REGION_SIZE_Y_CM = 7900.0
DEFAULT_RESOLUTION_CM = 10.0
DEFAULT_XY_TOLERANCE_CM = 4.5
DEFAULT_Z_TOLERANCE_CM = 50.0


def region_width_cells(resolution_cm: float = DEFAULT_RESOLUTION_CM) -> int:
    return int(math.ceil(REGION_SIZE_X_CM / resolution_cm))


def region_height_cells(resolution_cm: float = DEFAULT_RESOLUTION_CM) -> int:
    return int(math.ceil(REGION_SIZE_Y_CM / resolution_cm))


def cell_center_world_xy(gx: int, gy: int, resolution_cm: float) -> WorldXY:
    """Cell center in UE world XY (cm). costs[gy, gx] convention."""
    ox, oy = REGION_ORIGIN_WORLD_XY
    return (
        ox + (gx + 0.5) * resolution_cm,
        oy + (gy + 0.5) * resolution_cm,
    )


def world_xy_to_cell(
    wx: float,
    wy: float,
    resolution_cm: float,
    *,
    clamp: bool = True,
) -> Tuple[int, int] | None:
    ox, oy = REGION_ORIGIN_WORLD_XY
    gx = int(math.floor((wx - ox) / resolution_cm))
    gy = int(math.floor((wy - oy) / resolution_cm))
    w = region_width_cells(resolution_cm)
    h = region_height_cells(resolution_cm)
    if clamp:
        gx = min(max(gx, 0), w - 1)
        gy = min(max(gy, 0), h - 1)
        return gx, gy
    if gx < 0 or gy < 0 or gx >= w or gy >= h:
        return None
    return gx, gy


def local_rect_to_cells(
    lx0: float,
    ly0: float,
    lx1: float,
    ly1: float,
    resolution_cm: float,
) -> list[tuple[int, int]]:
    """Inclusive local-XY rectangle (cm) → list of (gx, gy)."""
    from level_coords import local_xy_to_world

    wx0, wy0 = local_xy_to_world(lx0, ly0)
    wx1, wy1 = local_xy_to_world(lx1, ly1)
    gx0, gy0 = world_xy_to_cell(min(wx0, wx1), min(wy0, wy1), resolution_cm, clamp=True)
    gx1, gy1 = world_xy_to_cell(max(wx0, wx1), max(wy0, wy1), resolution_cm, clamp=True)
    cells: list[tuple[int, int]] = []
    for gy in range(gy0, gy1 + 1):
        for gx in range(gx0, gx1 + 1):
            cells.append((gx, gy))
    return cells
