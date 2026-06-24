#!/usr/bin/env python3
"""Perception standoff: keep robot ~1m from obstacles before L2 depth / replan."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Set, Tuple

GridCell = Tuple[int, int]

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

    def backoff_cm(self, standoff_cm: float, *, margin_cm: float = 15.0, max_cm: float = 80.0) -> float:
        if not self.needs_backoff(standoff_cm):
            return 0.0
        return min(max_cm, max(0.0, standoff_cm - self.nearest_dist_cm + margin_cm))


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


def depth_confirms_clearance(
    forward_depth_cm: Optional[float],
    standoff_cm: float,
) -> bool:
    """True when live forward depth shows at least standoff clearance."""
    return (
        standoff_cm > 0.0
        and forward_depth_cm is not None
        and math.isfinite(forward_depth_cm)
        and forward_depth_cm >= standoff_cm
    )


def nearest_environment_distance_cm(
    robot_xy: WorldXY,
    layers: LayeredCostmap,
    *,
    registry_positions: Sequence[WorldXY] = (),
    forward_depth_cm: Optional[float] = None,
    search_radius_cm: float = 250.0,
) -> Tuple[Optional[float], str]:
    """Conservative distance to environment objects using all available sources."""
    candidates: list[Tuple[float, str]] = []
    if forward_depth_cm is not None and math.isfinite(forward_depth_cm):
        candidates.append((forward_depth_cm, "depth"))
    l2_xy, l2_dist = nearest_l2_obstacle_cm(
        robot_xy, layers, search_radius_cm=search_radius_cm
    )
    if l2_xy is not None and math.isfinite(l2_dist):
        candidates.append((l2_dist, "l2"))
    reg_xy, reg_dist = nearest_registry_obstacle_cm(robot_xy, registry_positions)
    if reg_xy is not None and math.isfinite(reg_dist):
        candidates.append((reg_dist, "registry"))
    if not candidates:
        return None, "none"
    return min(candidates, key=lambda item: item[0])


def check_perception_standoff(
    robot_xy: WorldXY,
    layers: LayeredCostmap,
    *,
    registry_positions: Sequence[WorldXY] = (),
    standoff_cm: float = 100.0,
    forward_depth_cm: Optional[float] = None,
) -> StandoffCheck:
    """Find nearest obstacle among L2 lethal cells and registry positions.

    When live forward depth confirms clearance, stale L2 cells are ignored so
    map-based standoff does not trigger false backoff.
    """
    if standoff_cm <= 0.0:
        return StandoffCheck(nearest_dist_cm=float("inf"), obstacle_xy=None, source="none")

    l2_xy, l2_dist = nearest_l2_obstacle_cm(robot_xy, layers, search_radius_cm=standoff_cm + 120.0)
    reg_xy, reg_dist = nearest_registry_obstacle_cm(robot_xy, registry_positions)

    depth_clear = depth_confirms_clearance(forward_depth_cm, standoff_cm)
    if depth_clear and l2_xy is not None and l2_dist < standoff_cm:
        l2_xy, l2_dist = None, float("inf")

    if l2_dist <= reg_dist and l2_xy is not None:
        return StandoffCheck(nearest_dist_cm=l2_dist, obstacle_xy=l2_xy, source="l2")
    if reg_xy is not None and reg_dist < float("inf"):
        return StandoffCheck(nearest_dist_cm=reg_dist, obstacle_xy=reg_xy, source="registry")
    return StandoffCheck(nearest_dist_cm=float("inf"), obstacle_xy=None, source="none")


def _bearing_deg(from_xy: WorldXY, to_xy: WorldXY) -> float:
    return math.degrees(math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0]))


def _angle_delta_deg(a: float, b: float) -> float:
    delta = (a - b + 180.0) % 360.0 - 180.0
    return delta


def evict_stale_l2_in_forward_cone(
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    layers: LayeredCostmap,
    *,
    forward_depth_cm: float,
    standoff_cm: float,
    l2_seen_cells: Optional[Set[GridCell]] = None,
    cone_half_deg: float = 45.0,
    depth_margin_cm: float = 15.0,
) -> int:
    """Clear non-latched L2 cells in the forward cone contradicted by live depth."""
    if not depth_confirms_clearance(forward_depth_cm, standoff_cm):
        return 0
    lethal_thresh = L2_LETHAL_COST * 0.5
    max_dist_cm = min(forward_depth_cm - depth_margin_cm, standoff_cm + 40.0)
    if max_dist_cm <= layers.resolution_cm:
        return 0
    removed = 0
    radius_cells = max(1, int(math.ceil(max_dist_cm / layers.resolution_cm)))
    costmap = layers.to_costmap2d()
    grid = costmap.world_xy_to_grid(robot_xy, clamp=True)
    if grid is None:
        return 0
    gx_c, gy_c = grid
    gx0 = max(0, gx_c - radius_cells)
    gx1 = min(layers.width_cells, gx_c + radius_cells + 1)
    gy0 = max(0, gy_c - radius_cells)
    gy1 = min(layers.height_cells, gy_c + radius_cells + 1)
    for gy in range(gy0, gy1):
        for gx in range(gx0, gx1):
            if layers.l2[gy, gx] < lethal_thresh:
                continue
            if layers.l2_static_latch[gy, gx]:
                continue
            cell_xy = _cell_center_xy(layers, gx, gy)
            dist_cm = dist2d(robot_xy, cell_xy)
            if dist_cm > max_dist_cm:
                continue
            bearing = _bearing_deg(robot_xy, cell_xy)
            if abs(_angle_delta_deg(bearing, robot_yaw_deg)) > cone_half_deg:
                continue
            layers.clear_l2_cell(gx, gy)
            if l2_seen_cells is not None:
                l2_seen_cells.discard((gx, gy))
            removed += 1
    return removed


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
