#!/usr/bin/env python3
"""Perception standoff: keep robot ~1m from obstacles before L2 depth / replan."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from costmap_layers import LayeredCostmap  # noqa: E402
from grid_env_10k_pie_patrol import dist2d  # noqa: E402
from perception_layer import L2_LETHAL_COST  # noqa: E402

WorldXY = Tuple[float, float]


@dataclass(frozen=True)
class StandoffCheck:
    """Nearest perceived obstacle relative to robot."""

    nearest_dist_cm: float
    obstacle_xy: Optional[WorldXY]
    source: str  # "l2", "registry", "none"

    @property
    def within_standoff(self) -> bool:
        return self.obstacle_xy is not None

    def needs_backoff(self, standoff_cm: float) -> bool:
        if not self.within_standoff or standoff_cm <= 0.0:
            return False
        return self.nearest_dist_cm < standoff_cm

    def backoff_cm(self, standoff_cm: float, *, margin_cm: float = 15.0) -> float:
        if not self.needs_backoff(standoff_cm):
            return 0.0
        return min(80.0, max(0.0, standoff_cm - self.nearest_dist_cm + margin_cm))


def _cell_center_xy(layers: LayeredCostmap, gx: int, gy: int) -> WorldXY:
    res = layers.resolution_cm
    return (
        layers.origin_xy[0] + (gx + 0.5) * res,
        layers.origin_xy[1] + (gy + 0.5) * res,
    )


def nearest_l2_obstacle_cm(
    robot_xy: WorldXY,
    layers: LayeredCostmap,
    *,
    search_radius_cm: float = 200.0,
) -> Tuple[Optional[WorldXY], float]:
    """Return nearest lethal L2 cell center and distance (inf if none)."""
    costmap = layers.to_costmap2d()
    grid = costmap.world_xy_to_grid(robot_xy, clamp=True)
    if grid is None:
        return None, float("inf")
    gx_c, gy_c = grid
    radius_cells = max(1, int(math.ceil(search_radius_cm / layers.resolution_cm)))
    gx0 = max(0, gx_c - radius_cells)
    gx1 = min(layers.width_cells, gx_c + radius_cells + 1)
    gy0 = max(0, gy_c - radius_cells)
    gy1 = min(layers.height_cells, gy_c + radius_cells + 1)
    lethal_thresh = L2_LETHAL_COST * 0.5
    nearest_xy: Optional[WorldXY] = None
    nearest_dist = float("inf")
    for gy in range(gy0, gy1):
        for gx in range(gx0, gx1):
            if layers.l2[gy, gx] < lethal_thresh:
                continue
            cell_xy = _cell_center_xy(layers, gx, gy)
            d = dist2d(robot_xy, cell_xy)
            if d < nearest_dist:
                nearest_dist = d
                nearest_xy = cell_xy
    return nearest_xy, nearest_dist


def nearest_registry_obstacle_cm(
    robot_xy: WorldXY,
    positions: Sequence[WorldXY],
) -> Tuple[Optional[WorldXY], float]:
    nearest_xy: Optional[WorldXY] = None
    nearest_dist = float("inf")
    for pos in positions:
        d = dist2d(robot_xy, pos)
        if d < nearest_dist:
            nearest_dist = d
            nearest_xy = pos
    return nearest_xy, nearest_dist


def check_perception_standoff(
    robot_xy: WorldXY,
    layers: LayeredCostmap,
    *,
    registry_positions: Sequence[WorldXY] = (),
    standoff_cm: float = 100.0,
) -> StandoffCheck:
    """Find nearest obstacle among L2 lethal cells and registry positions."""
    if standoff_cm <= 0.0:
        return StandoffCheck(nearest_dist_cm=float("inf"), obstacle_xy=None, source="none")

    l2_xy, l2_dist = nearest_l2_obstacle_cm(robot_xy, layers, search_radius_cm=standoff_cm + 120.0)
    reg_xy, reg_dist = nearest_registry_obstacle_cm(robot_xy, registry_positions)

    if l2_dist <= reg_dist and l2_xy is not None:
        return StandoffCheck(nearest_dist_cm=l2_dist, obstacle_xy=l2_xy, source="l2")
    if reg_xy is not None and reg_dist < float("inf"):
        return StandoffCheck(nearest_dist_cm=reg_dist, obstacle_xy=reg_xy, source="registry")
    return StandoffCheck(nearest_dist_cm=float("inf"), obstacle_xy=None, source="none")


def away_bearing_deg(robot_xy: WorldXY, obstacle_xy: WorldXY) -> float:
    """Bearing from robot away from obstacle (degrees, same convention as yaw_to_target)."""
    dx = robot_xy[0] - obstacle_xy[0]
    dy = robot_xy[1] - obstacle_xy[1]
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return 0.0
    return math.degrees(math.atan2(dy, dx))


@dataclass(frozen=True)
class StandoffGateResult:
    """Whether perception may proceed after optional backoff."""

    run_perceive: bool
    robot_xy: WorldXY
    triggered: bool = False
    deferred: bool = False
