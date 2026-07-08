#!/usr/bin/env python3
"""Surface-distance helpers for proximity violation metrics (Phase 3)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from navmesh_types import ActorBounds

WorldXY = Tuple[float, float]


@dataclass(frozen=True)
class PathClearanceReport:
    ok: bool
    min_center_clearance_cm: Optional[float] = None
    min_body_edge_clearance_cm: Optional[float] = None
    worst_obstacle_id: Optional[str] = None
    violating_wp_count: int = 0
    violating_segment_count: int = 0


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


def body_edge_to_aabb_surface_distance_cm(
    robot_xy: WorldXY,
    obstacles: Sequence[SurfaceObstacle],
    *,
    body_radius_cm: float,
) -> Tuple[Optional[float], Optional[str]]:
    """Body outer edge to nearest obstacle AABB surface."""
    center_dist, obstacle_id = nearest_surface_distance_cm(robot_xy, obstacles)
    if center_dist is None:
        return None, obstacle_id
    return center_dist - body_radius_cm, obstacle_id


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


def validate_path_center_clearance(
    points: Sequence[WorldXY],
    obstacles: Sequence[SurfaceObstacle],
    *,
    min_center_clearance_cm: float,
    body_radius_cm: float = 0.0,
    sample_spacing_cm: float = 16.0,
    exclude_first_n_waypoints: int = 0,
    exclude_last_n_waypoints: int = 0,
    validate_segments: bool = False,
) -> PathClearanceReport:
    """Validate waypoint points (and optionally straight segments) against obstacle AABBs."""
    if not points or not obstacles:
        return PathClearanceReport(ok=True)

    exclude_first_n_waypoints = max(0, min(exclude_first_n_waypoints, len(points) - 1))
    exclude_last_n_waypoints = max(0, min(exclude_last_n_waypoints, len(points) - 1))
    corridor_points = list(points)
    if exclude_first_n_waypoints > 0:
        corridor_points = corridor_points[exclude_first_n_waypoints:]
    if exclude_last_n_waypoints > 0:
        corridor_points = corridor_points[:-exclude_last_n_waypoints]
    if len(corridor_points) < 1:
        return PathClearanceReport(ok=True)

    min_center: Optional[float] = None
    min_body_edge: Optional[float] = None
    worst_id: Optional[str] = None
    wp_viol = 0
    seg_viol = 0

    for point in corridor_points:
        center, oid = nearest_surface_distance_cm(point, obstacles)
        if center is None:
            continue
        edge = center - body_radius_cm
        if center < min_center_clearance_cm:
            wp_viol += 1
        if min_center is None or center < min_center:
            min_center = center
            worst_id = oid
        if min_body_edge is None or edge < min_body_edge:
            min_body_edge = edge

    if validate_segments:
        for idx in range(len(corridor_points) - 1):
            center = min_clearance_on_segment_cm(
                corridor_points[idx],
                corridor_points[idx + 1],
                obstacles,
                sample_spacing_cm=sample_spacing_cm,
            )
            if center is None:
                continue
            edge = center - body_radius_cm
            if center < min_center_clearance_cm:
                seg_viol += 1
            if min_center is None or center < min_center:
                min_center = center
            if min_body_edge is None or edge < min_body_edge:
                min_body_edge = edge

    ok = wp_viol == 0 and (
        min_center is None or min_center >= min_center_clearance_cm
    )
    if validate_segments:
        ok = ok and seg_viol == 0
    return PathClearanceReport(
        ok=ok,
        min_center_clearance_cm=min_center,
        min_body_edge_clearance_cm=min_body_edge,
        worst_obstacle_id=worst_id,
        violating_wp_count=wp_viol,
        violating_segment_count=seg_viol,
    )


def validate_path_corridor_clearance(
    points: Sequence[WorldXY],
    obstacles: Sequence[SurfaceObstacle],
    *,
    min_center_clearance_cm: float,
    body_radius_cm: float = 0.0,
    sample_spacing_cm: float = 16.0,
) -> PathClearanceReport:
    """Validate transit corridor waypoints only (start/goal exempt; no chord segments)."""
    return validate_path_center_clearance(
        points,
        obstacles,
        min_center_clearance_cm=min_center_clearance_cm,
        body_radius_cm=body_radius_cm,
        sample_spacing_cm=sample_spacing_cm,
        exclude_first_n_waypoints=1,
        exclude_last_n_waypoints=1,
        validate_segments=False,
    )


def adjust_xy_for_planning_clearance(
    xy: WorldXY,
    obstacles: Sequence[SurfaceObstacle],
    *,
    min_center_clearance_cm: float,
    step_cm: float = 20.0,
    max_steps: int = 40,
) -> Tuple[WorldXY, bool]:
    """Push XY away from nearest prop AABB until planning center clearance is met."""
    if not obstacles:
        return xy, True
    cur = xy
    for _ in range(max_steps):
        center, oid = nearest_surface_distance_cm(cur, obstacles)
        if center is None or center >= min_center_clearance_cm:
            return cur, True
        obs = next((o for o in obstacles if o.obstacle_id == oid), None)
        if obs is None:
            return cur, False
        closest_x = min(max(cur[0], obs.cx - obs.half_x), obs.cx + obs.half_x)
        closest_y = min(max(cur[1], obs.cy - obs.half_y), obs.cy + obs.half_y)
        dx = cur[0] - closest_x
        dy = cur[1] - closest_y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            dx = cur[0] - obs.cx
            dy = cur[1] - obs.cy
            dist = math.hypot(dx, dy)
            if dist < 1e-3:
                dx, dy, dist = 1.0, 0.0, 1.0
        push = max(step_cm, min_center_clearance_cm - center + 1.0)
        cur = (cur[0] + dx / dist * push, cur[1] + dy / dist * push)
    for _ in range(8):
        center, _ = nearest_surface_distance_cm(cur, obstacles)
        if center is None or center >= min_center_clearance_cm:
            break
        obs_list = sorted(
            obstacles,
            key=lambda o: center_to_aabb_surface_distance_cm(cur, o),
        )
        obs = obs_list[0]
        closest_x = min(max(cur[0], obs.cx - obs.half_x), obs.cx + obs.half_x)
        closest_y = min(max(cur[1], obs.cy - obs.half_y), obs.cy + obs.half_y)
        dx = cur[0] - closest_x
        dy = cur[1] - closest_y
        dist = math.hypot(dx, dy) or 1.0
        push = min_center_clearance_cm - (center or 0.0) + 1.0
        cur = (cur[0] + dx / dist * push, cur[1] + dy / dist * push)
    center, _ = nearest_surface_distance_cm(cur, obstacles)
    return cur, center is not None and center >= min_center_clearance_cm


def densify_waypoints_for_chord_clearance(
    points: Sequence[WorldXY],
    obstacles: Sequence[SurfaceObstacle],
    *,
    min_clearance_cm: float,
    sample_spacing_cm: float = 20.0,
    max_insertions: int = 2048,
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


def build_path_clearance_obstacles(
    bounds_cache: Dict[str, ActorBounds],
    *,
    exempt_actor_names: Optional[Iterable[str]] = None,
) -> Tuple[SurfaceObstacle, ...]:
    """Obstacles subject to the 1 m planning clearance rule (unspecified props only)."""
    exempt = set(exempt_actor_names or ())
    return tuple(
        SurfaceObstacle.from_actor_bounds(bounds)
        for actor_name, bounds in bounds_cache.items()
        if actor_name not in exempt
    )
