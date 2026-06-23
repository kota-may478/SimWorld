#!/usr/bin/env python3
"""L0/L1/L2 layered costmap merge + A* planning."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Set, Tuple

import numpy as np

from l0_nav_mask import COSTMAP_DEFAULT_CELL_COST, COSTMAP_LETHAL_COST, load_l0_mask_npz
from level_coords import REGION_ORIGIN_WORLD_XY, local_xy_to_world
from work_region import DEFAULT_RESOLUTION_CM
from zone_registry import ZoneRegistry

import sys

_MT = Path(__file__).resolve().parent.parent / "llm_material_transport"
if str(_MT) not in sys.path:
    sys.path.insert(0, str(_MT))

from path_planning_costmap import (  # noqa: E402
    AStarPlanResult,
    Costmap2D,
    costmap_from_array,
    plan_waypoints_grid_astar,
)

WorldXY = Tuple[float, float]
GridCell = Tuple[int, int]

PATH_WP_SPACING_CM = 300.0


# Log-odds occupancy (L2_depth): hit/miss updates, threshold → binary l2 cost.
L2_LOG_ODDS_HIT = 0.85
L2_LOG_ODDS_MISS = -0.40
L2_LOG_ODDS_MIN = -4.0
L2_LOG_ODDS_MAX = 6.0
L2_LOG_ODDS_OCCUPIED = 0.5


@dataclass
class LayeredCostmap:
    l0: np.ndarray
    origin_xy: WorldXY
    resolution_cm: float
    lethal_cost: float = COSTMAP_LETHAL_COST
    l1: np.ndarray = field(init=False)
    l2: np.ndarray = field(init=False)
    l2_log_odds: np.ndarray = field(init=False)
    l2_static_latch: np.ndarray = field(init=False)
    _closed_zones: Set[str] = field(default_factory=set)

    def __post_init__(self) -> None:
        shape = self.l0.shape
        self.l1 = np.zeros(shape, dtype=np.float32)
        self.l2 = np.zeros(shape, dtype=np.float32)
        self.l2_log_odds = np.zeros(shape, dtype=np.float32)
        self.l2_static_latch = np.zeros(shape, dtype=bool)

    @classmethod
    def from_l0_cache(cls, path: Path | str) -> LayeredCostmap:
        costs, resolution_cm, origin, lethal = load_l0_mask_npz(Path(path))
        ox, oy = origin
        return cls(
            l0=costs,
            origin_xy=(ox, oy),
            resolution_cm=resolution_cm,
            lethal_cost=lethal,
        )

    @classmethod
    def from_l0_array(
        cls,
        costs: np.ndarray,
        *,
        resolution_cm: float = DEFAULT_RESOLUTION_CM,
        origin_xy: WorldXY = REGION_ORIGIN_WORLD_XY,
    ) -> LayeredCostmap:
        return cls(l0=costs.astype(np.float32), origin_xy=origin_xy, resolution_cm=resolution_cm)

    @property
    def height_cells(self) -> int:
        return int(self.l0.shape[0])

    @property
    def width_cells(self) -> int:
        return int(self.l0.shape[1])

    def reset_l1(self) -> None:
        self.l1.fill(0.0)
        self._closed_zones.clear()

    def reset_l2(self) -> None:
        self.l2.fill(0.0)
        self.l2_log_odds.fill(0.0)
        self.l2_static_latch.fill(False)

    def sync_l2_from_log_odds(self, *, occupied_threshold: float = L2_LOG_ODDS_OCCUPIED) -> int:
        """Project log-odds field to binary L2_depth cost (static latch always occupied)."""
        occupied = (self.l2_log_odds >= occupied_threshold) | self.l2_static_latch
        self.l2[:, :] = np.where(occupied, COSTMAP_LETHAL_COST, 0.0).astype(np.float32)
        return int(np.count_nonzero(occupied))

    def update_l2_log_odds_cell(
        self,
        gx: int,
        gy: int,
        delta: float,
        *,
        latch_static: bool = False,
    ) -> None:
        if not (0 <= gx < self.width_cells and 0 <= gy < self.height_cells):
            return
        if self.l2_static_latch[gy, gx] and delta < 0:
            return
        prev = float(self.l2_log_odds[gy, gx])
        new_val = max(L2_LOG_ODDS_MIN, min(L2_LOG_ODDS_MAX, prev + delta))
        self.l2_log_odds[gy, gx] = new_val
        if latch_static and delta > 0:
            self.l2_static_latch[gy, gx] = True
        if self.l2_static_latch[gy, gx] or new_val >= L2_LOG_ODDS_OCCUPIED:
            self.l2[gy, gx] = COSTMAP_LETHAL_COST
        elif prev >= L2_LOG_ODDS_OCCUPIED and new_val < L2_LOG_ODDS_OCCUPIED:
            self.l2[gy, gx] = 0.0

    def _write_zone_to_l1(
        self,
        registry: ZoneRegistry,
        zone_id: str,
        cost_value: float,
    ) -> int:
        zdef = registry.get(zone_id)
        count = 0
        h, w = self.l1.shape
        for gx, gy in zdef.cells:
            if 0 <= gx < w and 0 <= gy < h:
                self.l1[gy, gx] = cost_value
                count += 1
        return count

    def close_zone(self, zone_id: str, registry: ZoneRegistry) -> int:
        if abs(registry.resolution_cm - self.resolution_cm) > 0.01:
            raise ValueError(
                f"zone registry resolution {registry.resolution_cm}cm "
                f"!= costmap {self.resolution_cm}cm; rebuild zone_registry.json"
            )
        zdef = registry.get(zone_id)
        n = self._write_zone_to_l1(registry, zone_id, zdef.closed_cost)
        self._closed_zones.add(zone_id)
        return n

    def open_zone(self, zone_id: str, registry: ZoneRegistry) -> int:
        if abs(registry.resolution_cm - self.resolution_cm) > 0.01:
            raise ValueError(
                f"zone registry resolution {registry.resolution_cm}cm "
                f"!= costmap {self.resolution_cm}cm; rebuild zone_registry.json"
            )
        zdef = registry.get(zone_id)
        n = self._write_zone_to_l1(registry, zone_id, zdef.default_cost)
        self._closed_zones.discard(zone_id)
        return n

    def is_zone_closed(self, zone_id: str) -> bool:
        return zone_id in self._closed_zones

    def set_l2_cell(self, gx: int, gy: int, cost: float) -> None:
        if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
            self.l2[gy, gx] = cost

    def clear_l2_cell(self, gx: int, gy: int) -> None:
        if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
            if self.l2_static_latch[gy, gx]:
                return
            self.l2[gy, gx] = 0.0
            self.l2_log_odds[gy, gx] = 0.0

    def merged_costs(self) -> np.ndarray:
        """Per-cell max of L0, L1, L2 (0 = no extra cost from layer)."""
        base = self.l0.astype(np.float32)
        extra = np.maximum(self.l1, self.l2)
        out = np.maximum(base, extra)
        return out

    def to_costmap2d(self) -> Costmap2D:
        return costmap_from_array(
            self.merged_costs(),
            origin_xy=self.origin_xy,
            resolution_cm=self.resolution_cm,
            lethal_cost=self.lethal_cost,
        )

    def plan_astar(
        self,
        start_xy: WorldXY,
        goal_xy: WorldXY,
        *,
        max_segment_cm: float = PATH_WP_SPACING_CM,
    ) -> AStarPlanResult:
        return plan_waypoints_grid_astar(
            self.to_costmap2d(),
            start_xy,
            goal_xy,
            max_segment_cm=max_segment_cm,
        )

    def plan_astar_local(
        self,
        start_local_xy: Tuple[float, float],
        goal_local_xy: Tuple[float, float],
        **kwargs,
    ) -> AStarPlanResult:
        sx, sy = local_xy_to_world(*start_local_xy)
        gx, gy = local_xy_to_world(*goal_local_xy)
        return self.plan_astar((sx, sy), (gx, gy), **kwargs)

    def snapshot_layers(self) -> Dict[str, np.ndarray]:
        return {
            "l0": self.l0.copy(),
            "l1": self.l1.copy(),
            "l2": self.l2.copy(),
            "l2_log_odds": self.l2_log_odds.copy(),
            "l2_static_latch": self.l2_static_latch.copy(),
            "merged": self.merged_costs(),
        }
