#!/usr/bin/env python3
"""L2_depth: depth-only costmap updates (ungated FOV + ray clearing + carry-forward mask)."""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Set, Tuple

import numpy as np

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from costmap_layers import LayeredCostmap  # noqa: E402
from perception_layer import (  # noqa: E402
    EgocentricPerceptionConfig,
    apply_l2_obstacle_cells,
    close_range_keepout_cells_from_depth,
    depth_hits_from_image,
    apply_depth_ray_update,
    update_l2_from_depth_image,
)

WorldXY = Tuple[float, float]
GridCell = Tuple[int, int]


@dataclass
class DepthCellTracker:
    """Tracks depth-painted cells for soft reset (phantom eviction only)."""

    active_cells: Set[GridCell] = field(default_factory=set)
    carry_forward_mask: Set[GridCell] = field(default_factory=set)

    def snapshot_occupied(self, layers: LayeredCostmap) -> Set[GridCell]:
        """Capture latched/static L2 cells to preserve across leg transitions."""
        occupied: Set[GridCell] = set()
        for gy in range(layers.height_cells):
            for gx in range(layers.width_cells):
                if layers.l2[gy, gx] > 0:
                    occupied.add((gx, gy))
        self.carry_forward_mask = set(occupied)
        return set(occupied)

    def clear_carry_forward(self) -> None:
        self.carry_forward_mask.clear()


@dataclass(frozen=True)
class DepthUpdateResult:
    hit_cells: int
    cleared_cells: int
    keepout_cells: int
    total_cells_added: int

    @property
    def l2_changed(self) -> bool:
        return self.total_cells_added > 0 or self.cleared_cells > 0


def update_l2_depth(
    depth_m: np.ndarray,
    layers: LayeredCostmap,
    *,
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    config: Optional[EgocentricPerceptionConfig] = None,
    tracker: Optional[DepthCellTracker] = None,
    camera_pitch_deg: float = 0.0,
    close_range_clearance_cm: float = 125.0,
    close_range_keepout_cm: float = 100.0,
) -> DepthUpdateResult:
    """Full FOV depth → L2_depth (ungated) + close-range keepout reflex."""
    cfg = config or EgocentricPerceptionConfig()
    if camera_pitch_deg != 0.0:
        cfg = EgocentricPerceptionConfig(
            fov_deg=cfg.fov_deg,
            max_range_cm=cfg.max_range_cm,
            min_obstacle_height_cm=cfg.min_obstacle_height_cm,
            stride_px=cfg.stride_px,
            use_lethal=cfg.use_lethal,
            camera_offset_forward_cm=cfg.camera_offset_forward_cm,
            camera_height_cm=cfg.camera_height_cm,
            camera_pitch_deg=camera_pitch_deg,
            self_exclude_radius_cm=cfg.self_exclude_radius_cm,
            use_log_odds=cfg.use_log_odds,
            latch_static=cfg.latch_static,
        )

    preserve = tracker.carry_forward_mask if tracker is not None else set()
    hits = depth_hits_from_image(
        depth_m,
        layers,
        robot_xy=robot_xy,
        robot_yaw_deg=robot_yaw_deg,
        config=cfg,
    )
    hit_count, cleared_count = apply_depth_ray_update(
        layers,
        hits,
        robot_xy=robot_xy,
        config=cfg,
        preserve_cells=preserve,
    )

    keepout = close_range_keepout_cells_from_depth(
        depth_m,
        layers,
        robot_xy=robot_xy,
        robot_yaw_deg=robot_yaw_deg,
        config=cfg,
        min_clearance_cm=close_range_clearance_cm,
        keepout_radius_cm=close_range_keepout_cm,
        camera_pitch_deg=camera_pitch_deg,
    )
    keepout_added = apply_l2_obstacle_cells(layers, keepout, config=cfg) if keepout else 0

    new_active: Set[GridCell] = {hit.cell for hit in hits}
    new_active.update(keepout)
    if tracker is not None:
        tracker.active_cells = new_active | preserve

    total = hit_count + keepout_added
    return DepthUpdateResult(
        hit_cells=hit_count,
        cleared_cells=cleared_count,
        keepout_cells=keepout_added,
        total_cells_added=total,
    )


def soft_l2_depth_reset(
    layers: LayeredCostmap,
    tracker: DepthCellTracker,
    l2_seen_cells: Set[GridCell],
    *,
    stuck_world_xy: Optional[WorldXY] = None,
    evict_near_radius_cm: float = 600.0,
) -> int:
    """Clear depth phantoms near stuck position; preserve static latch + carry-forward mask."""
    from grid_env_10k_pie_patrol import dist2d  # noqa: WPS433

    removed = 0
    preserve = set(tracker.carry_forward_mask)
    for gy in range(layers.height_cells):
        for gx in range(layers.width_cells):
            if layers.l2_static_latch[gy, gx]:
                preserve.add((gx, gy))

    if stuck_world_xy is not None:
        to_clear: list[GridCell] = []
        for gx, gy in tracker.active_cells:
            if (gx, gy) in preserve:
                continue
            cell_xy = (
                layers.origin_xy[0] + (gx + 0.5) * layers.resolution_cm,
                layers.origin_xy[1] + (gy + 0.5) * layers.resolution_cm,
            )
            if dist2d(stuck_world_xy, cell_xy) < evict_near_radius_cm:
                to_clear.append((gx, gy))
        for gx, gy in to_clear:
            layers.clear_l2_cell(gx, gy)
            l2_seen_cells.discard((gx, gy))
            tracker.active_cells.discard((gx, gy))
            removed += 1
    else:
        for gx, gy in list(tracker.active_cells):
            if (gx, gy) in preserve:
                continue
            layers.clear_l2_cell(gx, gy)
            l2_seen_cells.discard((gx, gy))
            tracker.active_cells.discard((gx, gy))
            removed += 1

    return removed
