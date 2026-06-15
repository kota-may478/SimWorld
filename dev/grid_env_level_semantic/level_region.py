#!/usr/bin/env python3
"""Level map region: two diagonal corners + outward margin → gx/gy grid."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterator, Tuple

BlockIndex = Tuple[int, int]

# User-measured corners on /Game/Maps/Level (PIE camera XY, UE cm).
# World x/y map to gx/gy with both axes flipped vs naive origin indexing:
#   gx=1 → max world X, gy=1 → max world Y (see ``_world_gx_gy``).
CORNER_A_XY_CM = (-208.95, -1620.88)
CORNER_B_XY_CM = (6029.27, 5552.99)
# Default block bottom Z [cm] before per-site depth calibration (legacy 65 m).
BLOCK_BOTTOM_Z_CM = 6500.0
# Block placement bottom Z [cm] (locked after calibration).
LOCKED_BLOCK_BOTTOM_Z_CM = 6470.0
# Label probes: wall tier = placement + 2 m; floor/air tier = wall tier − 2.30 m.
LABEL_PROBE_ABOVE_PLACE_CM = 200.0
LABEL_PROBE_LOWER_STEP_CM = 230.0
# Height-scan step when all cells are air [cm] (legacy auto-height only).
HEIGHT_STEP_CM = 30.0
OUTWARD_MARGIN_M = 3.0
OUTWARD_MARGIN_CM = OUTWARD_MARGIN_M * 100.0

MAX_HEIGHT_TRIES = 10
MAX_HEIGHT_TRIES_ALL_AIR = 100

CELL_SIZE_CM = 30.0  # 0.3 m cube grid
CELL_HALF_CM = CELL_SIZE_CM / 2.0


@dataclass(frozen=True)
class LevelRegionConfig:
    corner_a_xy_cm: Tuple[float, float]
    corner_b_xy_cm: Tuple[float, float]
    block_bottom_z_cm: float
    outward_margin_cm: float = OUTWARD_MARGIN_CM
    cell_size_cm: float = CELL_SIZE_CM

    @property
    def core_x_min_cm(self) -> float:
        return min(self.corner_a_xy_cm[0], self.corner_b_xy_cm[0])

    @property
    def core_x_max_cm(self) -> float:
        return max(self.corner_a_xy_cm[0], self.corner_b_xy_cm[0])

    @property
    def core_y_min_cm(self) -> float:
        return min(self.corner_a_xy_cm[1], self.corner_b_xy_cm[1])

    @property
    def core_y_max_cm(self) -> float:
        return max(self.corner_a_xy_cm[1], self.corner_b_xy_cm[1])

    @property
    def expanded_x_min_cm(self) -> float:
        return self.core_x_min_cm - self.outward_margin_cm

    @property
    def expanded_x_max_cm(self) -> float:
        return self.core_x_max_cm + self.outward_margin_cm

    @property
    def expanded_y_min_cm(self) -> float:
        return self.core_y_min_cm - self.outward_margin_cm

    @property
    def expanded_y_max_cm(self) -> float:
        return self.core_y_max_cm + self.outward_margin_cm

    @property
    def grid_origin_xy_cm(self) -> Tuple[float, float]:
        """Snap grid origin to corner A (core min XY); cell (1,1) anchors at corner A."""
        ox = math.floor(self.core_x_min_cm / self.cell_size_cm) * self.cell_size_cm
        oy = math.floor(self.core_y_min_cm / self.cell_size_cm) * self.cell_size_cm
        return ox, oy

    @property
    def grid_extent_x_cm(self) -> float:
        ox, _ = self.grid_origin_xy_cm
        x_end = math.ceil(self.expanded_x_max_cm / self.cell_size_cm) * self.cell_size_cm
        return x_end - ox

    @property
    def grid_extent_y_cm(self) -> float:
        _, oy = self.grid_origin_xy_cm
        y_end = math.ceil(self.expanded_y_max_cm / self.cell_size_cm) * self.cell_size_cm
        return y_end - oy

    @property
    def grid_nx(self) -> int:
        return max(1, int(round(self.grid_extent_x_cm / self.cell_size_cm)))

    @property
    def grid_ny(self) -> int:
        return max(1, int(round(self.grid_extent_y_cm / self.cell_size_cm)))

    @property
    def cell_count(self) -> int:
        return self.grid_nx * self.grid_ny

    def iter_indices(self) -> Iterator[BlockIndex]:
        for gx in range(1, self.grid_nx + 1):
            for gy in range(1, self.grid_ny + 1):
                yield gx, gy

    def cell_center_xy_cm(self, gx: int, gy: int) -> Tuple[float, float]:
        ox, oy = self.grid_origin_xy_cm
        x = ox + (gx - 1) * self.cell_size_cm + CELL_HALF_CM
        y = oy + (gy - 1) * self.cell_size_cm + CELL_HALF_CM
        return x, y


def default_level_region(
    *,
    block_bottom_z_cm: float = BLOCK_BOTTOM_Z_CM,
) -> LevelRegionConfig:
    return LevelRegionConfig(
        corner_a_xy_cm=CORNER_A_XY_CM,
        corner_b_xy_cm=CORNER_B_XY_CM,
        block_bottom_z_cm=block_bottom_z_cm,
    )


def initial_block_bottom_z_cm(
    surface_z_cm: float,
    *,
    block_height_cm: float = CELL_SIZE_CM,
    height_step_cm: float = HEIGHT_STEP_CM,
) -> float:
    """Initial bottom Z: on-floor cells are air; one ``height_step`` lower → floor.

    At ``S + block_height + height_step`` the z_lower band is above local surface S.
    After lowering once by ``height_step``, bottom becomes ``S + block_height`` and
    the z_lower band ``[S, S + block_height]`` includes S → floor (per-column depth).
    """
    return surface_z_cm + block_height_cm + height_step_cm


def world_xy_to_cell_index(
    region: LevelRegionConfig,
    x_cm: float,
    y_cm: float,
) -> BlockIndex:
    """Nearest gx/gy (1-based) for a world XY [cm]."""
    ox, oy = region.grid_origin_xy_cm
    gx = int(round((x_cm - ox - CELL_HALF_CM) / region.cell_size_cm)) + 1
    gy = int(round((y_cm - oy - CELL_HALF_CM) / region.cell_size_cm)) + 1
    gx = max(1, min(region.grid_nx, gx))
    gy = max(1, min(region.grid_ny, gy))
    return gx, gy


def subgrid_around_cell(
    center_gx: int,
    center_gy: int,
    *,
    half: int = 2,
    region: LevelRegionConfig | None = None,
) -> Tuple[int, int, int, int]:
    """Inclusive gx/gy rectangle; ``half=2`` → 5×5."""
    gx0 = center_gx - half
    gy0 = center_gy - half
    gx1 = center_gx + half
    gy1 = center_gy + half
    if region is not None:
        gx0 = max(1, gx0)
        gy0 = max(1, gy0)
        gx1 = min(region.grid_nx, gx1)
        gy1 = min(region.grid_ny, gy1)
    return gx0, gy0, gx1, gy1


def initial_bottom_on_wall_detected(bottom_z_cm: float) -> float:
    """Block bottom Z after wall cells appear: detection height + one 0.15 m step."""
    return bottom_z_cm + HEIGHT_STEP_CM


def all_air_height_scan_action(
    counts: dict[str, int],
    total_cells: int,
) -> str | None:
    """Height-scan policy starting from ``BLOCK_BOTTOM_Z_CM`` (65 m).

    - all air → ``'lower'`` (−30 cm per step)
    - any wall → ``'lock_wall'`` (initial bottom = detection height + 30 cm)
    - floor-only / mixed without wall → keep lowering until wall or all air
    """
    wall = int(counts.get("wall", 0))
    air = int(counts.get("air", 0))
    floor = int(counts.get("floor", 0))
    if wall == 0 and air == total_cells:
        return "lower"
    if wall > 0:
        return "lock_wall"
    if wall == 0 and air == 0 and floor == 0:
        return "stop"
    return "lower"


def semantic_count_adjustment(
    counts: dict[str, int],
) -> str | None:
    """Legacy raise/lower helper (superseded by ``all_air_height_scan_action``)."""
    wall = int(counts.get("wall", 0))
    air = int(counts.get("air", 0))
    floor = int(counts.get("floor", 0))
    if floor != 0:
        return None
    if wall == 0 and air == 0:
        return None
    if wall > air:
        return "raise"
    if air > wall:
        return "lower"
    return None


def height_scan_attempt_limit(
    counts: dict[str, int],
    total_cells: int,
    *,
    nadir_surface_hits: int = 0,
) -> int:
    """10 tries by default; extend to 100 only when nadir sees geometry but labels are all air."""
    air = int(counts.get("air", 0))
    wall = int(counts.get("wall", 0))
    floor = int(counts.get("floor", 0))
    all_air = air == total_cells and wall == 0 and floor == 0
    if all_air and nadir_surface_hits == 0:
        return 1
    return MAX_HEIGHT_TRIES_ALL_AIR if all_air else MAX_HEIGHT_TRIES
