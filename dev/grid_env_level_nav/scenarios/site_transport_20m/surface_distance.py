#!/usr/bin/env python3
"""Surface-distance helpers for proximity violation metrics (Phase 3)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

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


def min_clearance_on_segment_cm(
    start_xy: WorldXY,
    end_xy: WorldXY,
    obstacles: Sequence[SurfaceObstacle],
    *,
    sample_spacing_cm: float = 20.0,
) -> Optional[float]:
    """Minimum center-to-AABB-surface distance sampled along a straight segment."""
    seg_len = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    if seg_len < 1e-6:
        return nearest_surface_distance_cm(start_xy, obstacles)[0]
    step_cm = max(1.0, sample_spacing_cm)
    samples = max(2, int(math.ceil(seg_len / step_cm)))
    best: Optional[float] = None
    for step in range(samples + 1):
        t = step / samples
        sample_xy = (
            start_xy[0] + (end_xy[0] - start_xy[0]) * t,
            start_xy[1] + (end_xy[1] - start_xy[1]) * t,
        )
        dist, _ = nearest_surface_distance_cm(sample_xy, obstacles)
        if dist is None:
            continue
        best = dist if best is None else min(best, dist)
    return best


def densify_waypoints_for_chord_clearance(
    points: Sequence[WorldXY],
    obstacles: Sequence[SurfaceObstacle],
    *,
    min_clearance_cm: float,
    sample_spacing_cm: float = 20.0,
    max_insertions: int = 256,
) -> List[WorldXY]:
    """Insert midpoints on segments where open-loop motion would violate clearance."""
    if len(points) < 2 or not obstacles:
        return list(points)
    dense: List[WorldXY] = list(points)
    insertions = 0
    idx = 0
    while idx < len(dense) - 1 and insertions < max_insertions:
        start_xy = dense[idx]
        end_xy = dense[idx + 1]
        clearance = min_clearance_on_segment_cm(
            start_xy,
            end_xy,
            obstacles,
            sample_spacing_cm=sample_spacing_cm,
        )
        if clearance is not None and clearance < min_clearance_cm:
            dense.insert(
                idx + 1,
                (
                    (start_xy[0] + end_xy[0]) * 0.5,
                    (start_xy[1] + end_xy[1]) * 0.5,
                ),
            )
            insertions += 1
            continue
        idx += 1
    return dense


def build_surface_obstacles_from_bounds(
    bounds_cache: Dict[str, ActorBounds],
    *,
    extra: Optional[Iterable[SurfaceObstacle]] = None,
) -> Tuple[SurfaceObstacle, ...]:
    items = [SurfaceObstacle.from_actor_bounds(b) for b in bounds_cache.values()]
    if extra is not None:
        items.extend(list(extra))
    return tuple(items)
