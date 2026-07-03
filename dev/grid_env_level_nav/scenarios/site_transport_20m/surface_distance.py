#!/usr/bin/env python3
"""Surface-distance helpers for proximity violation metrics (Phase 3)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Sequence, Tuple

from navmesh_types import ActorBounds

WorldXY = Tuple[float, float]


@dataclass(frozen=True)
class SurfaceObstacle:
    obstacle_id: str
    cx: float
    cy: float
    half_x: float
    half_y: float

    @classmethod
    def from_actor_bounds(cls, bounds: ActorBounds) -> "SurfaceObstacle":
        return cls(
            obstacle_id=bounds.actor_name,
            cx=bounds.cx,
            cy=bounds.cy,
            half_x=bounds.half_x,
            half_y=bounds.half_y,
        )

    @classmethod
    def from_center_radius(
        cls,
        obstacle_id: str,
        center_xy: WorldXY,
        radius_cm: float,
    ) -> "SurfaceObstacle":
        r = max(1.0, radius_cm)
        return cls(
            obstacle_id=obstacle_id,
            cx=center_xy[0],
            cy=center_xy[1],
            half_x=r,
            half_y=r,
        )


def center_to_aabb_surface_distance_cm(robot_xy: WorldXY, obstacle: SurfaceObstacle) -> float:
    """2D distance from robot center to axis-aligned box surface."""
    dx = abs(robot_xy[0] - obstacle.cx) - obstacle.half_x
    dy = abs(robot_xy[1] - obstacle.cy) - obstacle.half_y
    dx = max(0.0, dx)
    dy = max(0.0, dy)
    return math.hypot(dx, dy)


def nearest_surface_distance_cm(
    robot_xy: WorldXY,
    obstacles: Sequence[SurfaceObstacle],
) -> Tuple[Optional[float], Optional[str]]:
    best_dist: Optional[float] = None
    best_id: Optional[str] = None
    for obstacle in obstacles:
        dist = center_to_aabb_surface_distance_cm(robot_xy, obstacle)
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_id = obstacle.obstacle_id
    return best_dist, best_id


def build_surface_obstacles_from_bounds(
    bounds_cache: Dict[str, ActorBounds],
    *,
    extra: Optional[Iterable[SurfaceObstacle]] = None,
) -> Tuple[SurfaceObstacle, ...]:
    items = [SurfaceObstacle.from_actor_bounds(b) for b in bounds_cache.values()]
    if extra is not None:
        items.extend(list(extra))
    return tuple(items)
