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

MAX_KEEPOUT_CELLS_PER_FRAME = 24
KEEPOUT_NEAR_FRACTION_SKIP = 0.35


def _depth_near_fraction(depth_m: np.ndarray, clearance_m: float) -> float:
    finite = depth_m[np.isfinite(depth_m) & (depth_m > 0.05)]
    if finite.size == 0:
        return 0.0
    return float(np.sum(finite <= clearance_m)) / float(finite.size)


@dataclass
class DepthCellTracker:
    """Tracks depth-painted cells for soft reset (phantom eviction only)."""

    active_cells: Set[GridCell] = field(default_factory=set)
    carry_forward_mask: Set[GridCell] = field(default_factory=set)

    def snapshot_occupied(self, layers: LayeredCostmap) -> Set[GridCell]:
        """Capture static-latched L2 cells to preserve across leg transitions.

        Only 2-hit latched obstacles are carried forward; ephemeral depth hits
        from the prior leg are dropped so they cannot block the return path.
        """
        occupied: Set[GridCell] = set()
        for gy in range(layers.height_cells):
            for gx in range(layers.width_cells):
                if layers.l2_static_latch[gy, gx]:
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
    near_frac = _depth_near_fraction(depth_m, close_range_clearance_cm / 100.0)
    if near_frac > KEEPOUT_NEAR_FRACTION_SKIP:
        keepout = []
    elif len(keepout) > MAX_KEEPOUT_CELLS_PER_FRAME:
        keepout = keepout[:MAX_KEEPOUT_CELLS_PER_FRAME]
    keepout_added = (
        apply_l2_obstacle_cells(layers, keepout, config=cfg, latch_static=False)
        if keepout
        else 0
    )

    new_active: Set[GridCell] = {hit.cell for hit in hits}
    new_active.update(keepout)
    if tracker is not None:
        tracker.active_cells.update(new_active)
        for gy in range(layers.height_cells):
            for gx in range(layers.width_cells):
                if layers.l2[gy, gx] > 0:
                    tracker.active_cells.add((gx, gy))

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
    aggressive: bool = False,
) -> int:
    """Clear depth phantoms near stuck position; preserve carry-forward mask.

    When aggressive=True (LAST RESORT), also evicts static-latched cells near the
    stuck pose so phantom obstacles on the robot path can be cleared.
    """
    from grid_env_10k_pie_patrol import dist2d  # noqa: WPS433

    removed = 0
    preserve = set(tracker.carry_forward_mask)
    if aggressive and stuck_world_xy is not None:
        # LAST RESORT: allow evicting carry-forward phantoms near the stuck pose.
        filtered: Set[GridCell] = set()
        for gx, gy in preserve:
            cell_xy = (
                layers.origin_xy[0] + (gx + 0.5) * layers.resolution_cm,
                layers.origin_xy[1] + (gy + 0.5) * layers.resolution_cm,
            )
            if dist2d(stuck_world_xy, cell_xy) >= evict_near_radius_cm:
                filtered.add((gx, gy))
        preserve = filtered
    elif not aggressive:
        for gy in range(layers.height_cells):
            for gx in range(layers.width_cells):
                if layers.l2_static_latch[gy, gx]:
                    preserve.add((gx, gy))

    to_clear: list[GridCell] = []
    for gy in range(layers.height_cells):
        for gx in range(layers.width_cells):
            if layers.l2[gy, gx] <= 0:
                continue
            if (gx, gy) in preserve:
                continue
            if stuck_world_xy is not None:
                cell_xy = (
                    layers.origin_xy[0] + (gx + 0.5) * layers.resolution_cm,
                    layers.origin_xy[1] + (gy + 0.5) * layers.resolution_cm,
                )
                if dist2d(stuck_world_xy, cell_xy) >= evict_near_radius_cm:
                    continue
            to_clear.append((gx, gy))

    clear_fn = layers.force_clear_l2_cell if aggressive else layers.clear_l2_cell
    for gx, gy in to_clear:
        clear_fn(gx, gy)
        l2_seen_cells.discard((gx, gy))
        tracker.active_cells.discard((gx, gy))
        removed += 1

    return removed
