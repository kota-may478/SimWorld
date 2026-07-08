#!/usr/bin/env python3
"""Track moving actors and trigger local NavMesh rebuild + replan."""

from __future__ import annotations

import math
import time
from typing import Any, Optional, Sequence, Tuple

import nav_query as nq
from metrics import NavTimingAccumulator
from navmesh_config import (
    DYNAMIC_OBSTACLE_REPLAN_DELTA_CM,
    HUMANOID_NAV_OBSTACLE_ID,
    NAV_DYNAMIC_OBSTACLE_MODIFIER_ENABLED,
    NAV_PROP_OBSTACLE_PADDING_CM,
    NAV_REBUILD_LOCAL_DIRTY_MARGIN_CM,
)
from navmesh_obstacles import (
    prime_dynamic_nav_modifier,
    sync_dynamic_obstacle_modifiers_local,
)

WorldXY = Tuple[float, float]
NavAABB = Tuple[float, float, float, float, float, float]


def _dist2d(a: WorldXY, b: WorldXY) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class DynamicNavObstacleTracker:
    """Tracks one moving actor for local NavModifier updates."""

    __slots__ = (
        "actor_name",
        "obstacle_id",
        "half_extent_pad_cm",
        "register_planning",
        "last_xy",
        "last_modifier_aabb",
    )

    def __init__(
        self,
        *,
        actor_name: str,
        obstacle_id: str,
        half_extent_pad_cm: float = NAV_PROP_OBSTACLE_PADDING_CM,
        register_planning: bool = True,
        last_xy: Optional[WorldXY] = None,
        last_modifier_aabb: Optional[NavAABB] = None,
    ) -> None:
        self.actor_name = actor_name
        self.obstacle_id = obstacle_id
        self.half_extent_pad_cm = half_extent_pad_cm
        self.register_planning = register_planning
        self.last_xy = last_xy
        self.last_modifier_aabb = last_modifier_aabb

    def with_pose(
        self,
        xy: WorldXY,
        modifier_aabb: Optional[NavAABB],
    ) -> "DynamicNavObstacleTracker":
        return DynamicNavObstacleTracker(
            actor_name=self.actor_name,
            obstacle_id=self.obstacle_id,
            half_extent_pad_cm=self.half_extent_pad_cm,
            register_planning=self.register_planning,
            last_xy=xy,
            last_modifier_aabb=modifier_aabb,
        )


def mission_dynamic_obstacle_trackers(registry) -> Tuple[DynamicNavObstacleTracker, ...]:
    """Default tracked movers for site_transport_20m (humanoid)."""
    return (
        DynamicNavObstacleTracker(
            actor_name=registry.humanoid_actor_name,
            obstacle_id=HUMANOID_NAV_OBSTACLE_ID,
            half_extent_pad_cm=NAV_PROP_OBSTACLE_PADDING_CM,
            register_planning=False,
        ),
    )


def resolve_dynamic_obstacle_trackers(
    *,
    trackers: Optional[Sequence[DynamicNavObstacleTracker]],
    registry: Any = None,
    humanoid_actor_name: Optional[str] = None,
    dynamic_humanoid: bool = False,
) -> Tuple[DynamicNavObstacleTracker, ...]:
    if trackers:
        return tuple(trackers)
    if registry is not None:
        return mission_dynamic_obstacle_trackers(registry)
    if dynamic_humanoid and humanoid_actor_name:
        return (
            DynamicNavObstacleTracker(
                actor_name=humanoid_actor_name,
                obstacle_id=HUMANOID_NAV_OBSTACLE_ID,
                half_extent_pad_cm=NAV_PROP_OBSTACLE_PADDING_CM,
                register_planning=False,
            ),
        )
    return ()


def prime_dynamic_obstacle_trackers(
    ucv,
    nav_actor: str,
    trackers: Sequence[DynamicNavObstacleTracker],
    *,
    modifier_enabled: bool = NAV_DYNAMIC_OBSTACLE_MODIFIER_ENABLED,
    local_dirty_margin_cm: float = NAV_REBUILD_LOCAL_DIRTY_MARGIN_CM,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> Tuple[DynamicNavObstacleTracker, ...]:
    """Register modifiers at mission start; local rebuild around each mover."""
    if not modifier_enabled or not trackers:
        return tuple(trackers)
    local_api = nq.nav_local_rebuild_api_available(ucv, nav_actor)
    if not local_api:
        print(
            "[NavMeshDyn] NavRebuildDirtyRegion API missing — "
            "local rebuild disabled (full NavRebuild fallback)"
        )
    updated: list[DynamicNavObstacleTracker] = []
    dirty_boxes: list[NavAABB] = []
    for tracker in trackers:
        try:
            loc = ucv.get_location(tracker.actor_name)
            xy = (float(loc[0]), float(loc[1]))
        except Exception as exc:
            print(f"[NavMeshDyn] prime skip {tracker.actor_name}: {exc}")
            updated.append(tracker)
            continue
        bounds, aabb = prime_dynamic_nav_modifier(
            ucv,
            nav_actor,
            tracker.actor_name,
            tracker.obstacle_id,
            half_extent_pad_cm=tracker.half_extent_pad_cm,
            register_planning=tracker.register_planning,
            nav_timing=nav_timing,
        )
        if bounds is None or aabb is None:
            updated.append(tracker)
            continue
        dirty_boxes.append(aabb)
        updated.append(tracker.with_pose(xy, aabb))
    if dirty_boxes:
        if local_api:
            sync_dynamic_obstacle_modifiers_local(
                ucv,
                nav_actor,
                dirty_boxes,
                local_dirty_margin_cm=local_dirty_margin_cm,
                nav_timing=nav_timing,
            )
        else:
            from navmesh_obstacles import _timed_rebuild

            _timed_rebuild(ucv, nav_actor, nav_timing)
        print(
            f"[NavMeshDyn] primed {len(dirty_boxes)} dynamic modifier(s) "
            f"({'local' if local_api else 'full'} rebuild, "
            f"margin={local_dirty_margin_cm:.0f}cm)"
        )
    return tuple(updated)


def poll_dynamic_obstacles_for_replan(
    ucv,
    nav_actor: str,
    trackers: Sequence[DynamicNavObstacleTracker],
    *,
    replan_delta_cm: float = DYNAMIC_OBSTACLE_REPLAN_DELTA_CM,
    modifier_enabled: bool = NAV_DYNAMIC_OBSTACLE_MODIFIER_ENABLED,
    local_dirty_margin_cm: float = NAV_REBUILD_LOCAL_DIRTY_MARGIN_CM,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> Tuple[Tuple[DynamicNavObstacleTracker, ...], bool]:
    """
    If any tracked actor moved >= replan_delta_cm, update its modifier and
    local-rebuild the union of old/new modifier AABBs. Returns (trackers, moved).
    """
    if not trackers:
        return tuple(trackers), False

    updated: list[DynamicNavObstacleTracker] = []
    dirty_boxes: list[NavAABB] = []
    any_moved = False

    for tracker in trackers:
        try:
            t_loc = time.perf_counter()
            loc = ucv.get_location(tracker.actor_name)
            if nav_timing is not None:
                nav_timing.record_elapsed("pose_query_ms", t_loc)
            new_xy = (float(loc[0]), float(loc[1]))
        except Exception as exc:
            print(f"[NavMeshDyn] pose skip {tracker.actor_name}: {exc}")
            updated.append(tracker)
            continue

        moved = (
            tracker.last_xy is not None
            and _dist2d(new_xy, tracker.last_xy) >= replan_delta_cm
        )
        if not moved:
            updated.append(tracker)
            continue

        any_moved = True
        new_aabb: Optional[NavAABB] = None
        if modifier_enabled:
            from navmesh_obstacles import update_dynamic_nav_modifier

            result = update_dynamic_nav_modifier(
                ucv,
                nav_actor,
                tracker.actor_name,
                tracker.obstacle_id,
                half_extent_pad_cm=tracker.half_extent_pad_cm,
                register_planning=tracker.register_planning,
                nav_timing=nav_timing,
            )
            if result is not None:
                _, new_aabb = result
                if tracker.last_modifier_aabb is not None:
                    dirty_boxes.append(
                        union_nav_aabbs(tracker.last_modifier_aabb, new_aabb)
                    )
                else:
                    dirty_boxes.append(new_aabb)

        updated.append(tracker.with_pose(new_xy, new_aabb or tracker.last_modifier_aabb))

    if any_moved and modifier_enabled and dirty_boxes:
        if nq.nav_local_rebuild_api_available(ucv, nav_actor):
            sync_dynamic_obstacle_modifiers_local(
                ucv,
                nav_actor,
                dirty_boxes,
                local_dirty_margin_cm=local_dirty_margin_cm,
                nav_timing=nav_timing,
            )
        else:
            from navmesh_obstacles import _timed_rebuild

            print("[NavMeshDyn] fallback full NavRebuild (local API missing)")
            _timed_rebuild(ucv, nav_actor, nav_timing)

    return tuple(updated), any_moved


def union_nav_aabbs(a: NavAABB, b: NavAABB) -> NavAABB:
    return (
        min(a[0], b[0]),
        min(a[1], b[1]),
        min(a[2], b[2]),
        max(a[3], b[3]),
        max(a[4], b[4]),
        max(a[5], b[5]),
    )
