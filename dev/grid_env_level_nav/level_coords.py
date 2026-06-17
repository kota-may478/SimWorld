#!/usr/bin/env python3
"""Local ↔ world coordinates for /Game/Maps/Level work region.

Convention (matches L0 costmap / UE top-down plots):
  - **local X** aligns with **UE world Y** (north); UE Y increase → local X increase
  - **local Y** aligns with **UE world X** (east); UE X increase → local Y increase
  - Signs are preserved; axes are swapped vs naive (local_x = wx - ox).
"""

from __future__ import annotations

from typing import Tuple

WorldXY = Tuple[float, float]
WorldXYZ = Tuple[float, float, float]
LocalXY = Tuple[float, float]
LocalXYZ = Tuple[float, float, float]

REGION_ORIGIN_WORLD_XY = (-1000.0, -2200.0)
# Measured floor band on Level (cm). Adjust if site calibration changes.
FLOOR_REF_Z_CM = 6440.0
DEFAULT_FOOT_OFFSET_Z_CM = 50.0
# NavProjectPoint probe height when BP ProjectExtentCm ≈ 30 (Z search ±extent).
NAV_PROJECT_PROBE_Z_CM = FLOOR_REF_Z_CM + 10.0


def local_xy_to_world(lx: float, ly: float) -> WorldXY:
    """local (X∥UE Y, Y∥UE X) → UE world (X, Y)."""
    ox, oy = REGION_ORIGIN_WORLD_XY
    return ox + ly, oy + lx


def world_xy_to_local(wx: float, wy: float) -> LocalXY:
    """UE world (X, Y) → local (X∥UE Y, Y∥UE X)."""
    ox, oy = REGION_ORIGIN_WORLD_XY
    return wy - oy, wx - ox


def local_xyz_to_world(lx: float, ly: float, lz: float) -> WorldXYZ:
    wx, wy = local_xy_to_world(lx, ly)
    return wx, wy, FLOOR_REF_Z_CM + lz


def world_xyz_to_local(wx: float, wy: float, wz: float) -> LocalXYZ:
    lx, ly = world_xy_to_local(wx, wy)
    return lx, ly, wz - FLOOR_REF_Z_CM


def foot_world_xyz_from_local_xy(lx: float, ly: float, foot_offset_cm: float = DEFAULT_FOOT_OFFSET_Z_CM) -> WorldXYZ:
    wx, wy = local_xy_to_world(lx, ly)
    return wx, wy, FLOOR_REF_Z_CM + foot_offset_cm
