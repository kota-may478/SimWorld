"""Obstacle-aware velocity scaling for RPP and open-loop segment execution."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class VelocityScaleConfig:
    """Parameters for shrinking forward motion near obstacles."""

    max_move_cm: float = 120.0
    near_obstacle_slow_cm: float = 220.0
    perception_standoff_cm: float = 50.0
    mid_band_move_cm: float = 140.0
    standoff_band_move_cm: float = 70.0
    close_band_move_cm: float = 35.0


def _finite_refs(
    nearest_dist_cm: float,
    forward_depth_cm: Optional[float],
) -> list[float]:
    return [
        d
        for d in (nearest_dist_cm, forward_depth_cm)
        if d is not None and math.isfinite(d) and d < float("inf")
    ]


def dynamic_max_move_cm(
    nearest_dist_cm: float,
    forward_depth_cm: Optional[float],
    *,
    config: VelocityScaleConfig,
) -> float:
    """Return absolute open-loop move cap (cm) from nearest obstacle references."""
    refs = _finite_refs(nearest_dist_cm, forward_depth_cm)
    if not refs:
        return config.max_move_cm
    nearest = min(refs)
    if nearest >= config.near_obstacle_slow_cm:
        return config.max_move_cm
    if nearest >= config.perception_standoff_cm + 40.0:
        return min(config.max_move_cm, config.mid_band_move_cm)
    if nearest >= config.perception_standoff_cm:
        return config.standoff_band_move_cm
    return config.close_band_move_cm


def velocity_scale_factor(
    nearest_dist_cm: float,
    forward_depth_cm: Optional[float],
    *,
    config: VelocityScaleConfig,
) -> float:
    """Scalar multiplier in [0, 1] for RPP ``max_move_cm`` (roadmap §2.5 table)."""
    refs = _finite_refs(nearest_dist_cm, forward_depth_cm)
    if not refs:
        return 1.0
    nearest = min(refs)
    standoff = config.perception_standoff_cm
    if nearest >= config.near_obstacle_slow_cm:
        return 1.0
    if nearest >= standoff + 40.0:
        return 0.7
    if nearest >= standoff:
        return 0.5
    return 0.25
