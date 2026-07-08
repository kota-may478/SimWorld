#!/usr/bin/env python3
"""Register runtime NavMesh box obstacles from site_transport actors."""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

import nav_query as nq
from metrics import NavTimingAccumulator
from navmesh_config import (
    HUMANOID_NAV_OBSTACLE_ID,
    NAV_FINDPATH_AGENT_RADIUS_CM,
    NAV_OBSTACLE_HALF_HEIGHT_CM,
    NAV_PROP_OBSTACLE_PADDING_CM,
    NAV_REBUILD_LOCAL_DIRTY_MARGIN_CM,
    NAV_REBUILD_SETTLE_S,
    NAV_ROADBLOCK_OBSTACLE_EXTRA_PADDING_CM,
    PROXIMITY_EDGE_FROM_SURFACE_CM,
)

WorldXYZ = Tuple[float, float, float]
NavAABB = Tuple[float, float, float, float, float, float]


from navmesh_types import ActorBounds


def _timed_rebuild(
    ucv,
    nav_actor: str,
    nav_timing: Optional[NavTimingAccumulator],
) -> dict:
    t0 = time.perf_counter()
    rebuild = nq.nav_rebuild(ucv, nav_actor)
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_rebuild_ms", t0)
        nav_timing.nav_rebuild_count += 1
    return rebuild


def _timed_local_rebuild(
    ucv,
    nav_actor: str,
    dirty_aabb: NavAABB,
    *,
    local_dirty_margin_cm: float = NAV_REBUILD_LOCAL_DIRTY_MARGIN_CM,
    nav_timing: Optional[NavTimingAccumulator],
) -> dict:
    t0 = time.perf_counter()
    min_xyz = (dirty_aabb[0], dirty_aabb[1], dirty_aabb[2])
    max_xyz = (dirty_aabb[3], dirty_aabb[4], dirty_aabb[5])
    if nq.nav_local_rebuild_api_available(ucv, nav_actor):
        rebuild = nq.nav_rebuild_dirty_region(
            ucv,
            nav_actor,
            min_xyz,
            max_xyz,
            margin_cm=local_dirty_margin_cm,
        )
    else:
        rebuild = nq.nav_rebuild(ucv, nav_actor)
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_local_rebuild_ms", t0)
        nav_timing.nav_local_rebuild_count += 1
    return rebuild


def modifier_aabb_for_bounds(
    bounds: ActorBounds,
    *,
    half_extent_pad_cm: float,
) -> NavAABB:
    half_x = max(5.0, bounds.half_x + half_extent_pad_cm)
    half_y = max(5.0, bounds.half_y + half_extent_pad_cm)
    half_z = max(5.0, NAV_OBSTACLE_HALF_HEIGHT_CM)
    return (
        bounds.cx - half_x,
        bounds.cy - half_y,
        bounds.cz - half_z,
        bounds.cx + half_x,
        bounds.cy + half_y,
        bounds.cz + half_z,
    )


def _union_nav_aabbs(boxes: Sequence[NavAABB]) -> NavAABB:
    min_x = min(box[0] for box in boxes)
    min_y = min(box[1] for box in boxes)
    min_z = min(box[2] for box in boxes)
    max_x = max(box[3] for box in boxes)
    max_y = max(box[4] for box in boxes)
    max_z = max(box[5] for box in boxes)
    return (min_x, min_y, min_z, max_x, max_y, max_z)


def sync_dynamic_obstacle_modifiers_local(
    ucv,
    nav_actor: str,
    dirty_boxes: Sequence[NavAABB],
    *,
    local_dirty_margin_cm: float = NAV_REBUILD_LOCAL_DIRTY_MARGIN_CM,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> dict:
    if not dirty_boxes:
        return {"ok": True}
    combined = _union_nav_aabbs(dirty_boxes)
    return _timed_local_rebuild(
        ucv,
        nav_actor,
        combined,
        local_dirty_margin_cm=local_dirty_margin_cm,
        nav_timing=nav_timing,
    )


def update_dynamic_nav_modifier(
    ucv,
    nav_actor: str,
    actor_name: str,
    obstacle_id: str,
    *,
    half_extent_pad_cm: float,
    register_planning: bool = True,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> Optional[Tuple[ActorBounds, NavAABB]]:
    bounds = fetch_actor_bounds(
        ucv, nav_actor, actor_name, nav_timing=nav_timing
    )
    if bounds is None:
        return None
    nav_bounds = ActorBounds(
        actor_name=actor_name,
        cx=bounds.cx,
        cy=bounds.cy,
        cz=bounds.cz,
        half_x=bounds.half_x,
        half_y=bounds.half_y,
        half_z=bounds.half_z,
    )
    half = (
        max(5.0, nav_bounds.half_x + half_extent_pad_cm),
        max(5.0, nav_bounds.half_y + half_extent_pad_cm),
        max(5.0, NAV_OBSTACLE_HALF_HEIGHT_CM),
    )
    t0 = time.perf_counter()
    raw = nq.nav_register_box_obstacle(
        ucv,
        nav_actor,
        obstacle_id,
        nav_bounds.center_xyz(),
        half,
    )
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_register_ms", t0)
        nav_timing.nav_register_count += 1
    if not raw.get("ok"):
        print(
            f"[NavMeshObs] dynamic modifier failed {actor_name}: "
            f"{raw.get('error', raw)}"
        )
        return None
    if register_planning and nq.nav_validated_api_available(ucv, nav_actor):
        register_planning_obstacle(ucv, nav_actor, nav_bounds, nav_timing=nav_timing)
    aabb = modifier_aabb_for_bounds(nav_bounds, half_extent_pad_cm=half_extent_pad_cm)
    return nav_bounds, aabb


def prime_dynamic_nav_modifier(
    ucv,
    nav_actor: str,
    actor_name: str,
    obstacle_id: str,
    *,
    half_extent_pad_cm: float,
    register_planning: bool = True,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> Tuple[Optional[ActorBounds], Optional[NavAABB]]:
    result = update_dynamic_nav_modifier(
        ucv,
        nav_actor,
        actor_name,
        obstacle_id,
        half_extent_pad_cm=half_extent_pad_cm,
        register_planning=register_planning,
        nav_timing=nav_timing,
    )
    if result is None:
        return None, None
    bounds, aabb = result
    return bounds, aabb


def bounds_from_nav_json(actor_name: str, raw: dict) -> Optional[ActorBounds]:
    if not raw.get("ok"):
        return None
    return ActorBounds(
        actor_name=actor_name,
        cx=float(raw["cx"]),
        cy=float(raw["cy"]),
        cz=float(raw["cz"]),
        half_x=float(raw["half_x"]),
        half_y=float(raw["half_y"]),
        half_z=float(raw.get("half_z", NAV_OBSTACLE_HALF_HEIGHT_CM)),
    )


def fetch_actor_bounds(
    ucv,
    nav_actor: str,
    target_actor: str,
    *,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> Optional[ActorBounds]:
    t0 = time.perf_counter()
    raw = nq.get_actor_bounds(ucv, nav_actor, target_actor)
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_bounds_ms", t0)
        nav_timing.nav_bounds_count += 1
    return bounds_from_nav_json(target_actor, raw)


def register_box_obstacle(
    ucv,
    nav_actor: str,
    bounds: ActorBounds,
    *,
    nav_timing: Optional[NavTimingAccumulator] = None,
    half_extent_pad_cm: float = 0.0,
) -> bool:
    half = (
        max(5.0, bounds.half_x + half_extent_pad_cm),
        max(5.0, bounds.half_y + half_extent_pad_cm),
        max(5.0, NAV_OBSTACLE_HALF_HEIGHT_CM),
    )
    t0 = time.perf_counter()
    raw = nq.nav_register_box_obstacle(
        ucv,
        nav_actor,
        bounds.obstacle_id,
        bounds.center_xyz(),
        half,
    )
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_register_ms", t0)
        nav_timing.nav_register_count += 1
    if not raw.get("ok"):
        print(
            f"[NavMeshObs] register failed {bounds.actor_name}: {raw.get('error', raw)}"
        )
        return False
    actual_hx = raw.get("actual_half_x")
    if actual_hx is not None and float(actual_hx) < 1.0:
        print(
            f"[NavMeshObs] WARN {bounds.actor_name}: modifier bounds near zero "
            f"(actual_half_x={actual_hx}) — rebuild UE NavQueryService.cpp"
        )
    return True


def register_planning_obstacle(
    ucv,
    nav_actor: str,
    bounds: ActorBounds,
    *,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> bool:
    t0 = time.perf_counter()
    raw = nq.nav_register_planning_obstacle(
        ucv,
        nav_actor,
        bounds.obstacle_id,
        bounds.cx,
        bounds.cy,
        bounds.half_x,
        bounds.half_y,
    )
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_register_ms", t0)
        nav_timing.nav_register_count += 1
    if not raw.get("ok"):
        print(
            f"[NavMeshObs] planning obstacle failed {bounds.actor_name}: "
            f"{raw.get('error', raw)}"
        )
        return False
    return True


def _prop_modifier_pad_cm(prop) -> float:
    pad_cm = NAV_PROP_OBSTACLE_PADDING_CM
    if prop.cluster_id == "no_entry_roadblock":
        pad_cm += NAV_ROADBLOCK_OBSTACLE_EXTRA_PADDING_CM
    return pad_cm


def register_props_from_registry(
    ucv,
    nav_actor: str,
    registry,
    *,
    skip_transport_target: bool = True,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> Tuple[Dict[str, ActorBounds], int]:
    """Register static prop NavModifier boxes and planning AABBs."""
    cached: Dict[str, ActorBounds] = {}
    registered = 0
    for prop in registry.props:
        if skip_transport_target and prop.is_transport_target:
            continue
        actor_name = prop.slot_id
        bounds = fetch_actor_bounds(
            ucv, nav_actor, actor_name, nav_timing=nav_timing
        )
        if bounds is None:
            print(f"[NavMeshObs] skip {actor_name}: bounds unavailable")
            continue
        pad_cm = _prop_modifier_pad_cm(prop)
        if not register_box_obstacle(
            ucv, nav_actor, bounds, nav_timing=nav_timing, half_extent_pad_cm=pad_cm
        ):
            continue
        if nq.nav_validated_api_available(ucv, nav_actor):
            register_planning_obstacle(
                ucv, nav_actor, bounds, nav_timing=nav_timing
            )
        cached[actor_name] = bounds
        registered += 1
    return cached, registered


def update_humanoid_obstacle(
    ucv,
    nav_actor: str,
    humanoid_actor_name: str,
    *,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> Optional[ActorBounds]:
    result = update_dynamic_nav_modifier(
        ucv,
        nav_actor,
        humanoid_actor_name,
        HUMANOID_NAV_OBSTACLE_ID,
        half_extent_pad_cm=NAV_PROP_OBSTACLE_PADDING_CM,
        register_planning=True,
        nav_timing=nav_timing,
    )
    if result is None:
        print(f"[NavMeshObs] humanoid obstacle failed: {humanoid_actor_name}")
        return None
    bounds, _ = result
    return bounds


def setup_static_navmesh_obstacles(
    ucv,
    nav_actor: str,
    registry,
    *,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> Tuple[Dict[str, ActorBounds], bool]:
    """Clear, register static props, rebuild NavMesh. Returns (bounds_cache, ok)."""
    if not nq.nav_runtime_api_available(ucv, nav_actor):
        print(
            "[NavMeshObs] extended NavQueryService API missing — "
            "rebuild UE with ue_native/NavQueryService (see NAVMESH_UE_SETUP.md)"
        )
        return {}, False
    if not nq.nav_validated_api_available(ucv, nav_actor):
        print(
            "[NavMeshObs] WARN: NavFindPathValidated API missing — "
            "planning will use Python fallback until UE NavQueryService is rebuilt"
        )

    t_clear = time.perf_counter()
    nq.nav_clear_box_obstacles(ucv, nav_actor)
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_clear_ms", t_clear)

    cached, count = register_props_from_registry(
        ucv, nav_actor, registry, nav_timing=nav_timing
    )
    rebuild = _timed_rebuild(ucv, nav_actor, nav_timing)
    if not rebuild.get("ok"):
        print(f"[NavMeshObs] NavRebuild failed: {rebuild.get('error', rebuild)}")
        return cached, False
    t_settle = time.perf_counter()
    time.sleep(NAV_REBUILD_SETTLE_S)
    if nav_timing is not None:
        nav_timing.record_elapsed("settle_ms", t_settle)
    sample = next(iter(cached.values()), None)
    if sample is not None:
        print(
            f"[NavMeshObs] sample {sample.actor_name}: "
            f"center=({sample.cx:.0f},{sample.cy:.0f},{sample.cz:.0f}) "
            f"half=({sample.half_x:.0f},{sample.half_y:.0f})"
        )
    print(
        f"[NavMeshObs] registered {count} prop obstacles; "
        f"modifier_pad={NAV_PROP_OBSTACLE_PADDING_CM:.0f}cm "
        f"findpath_agent_radius={NAV_FINDPATH_AGENT_RADIUS_CM:.0f}cm "
        f"(edge={PROXIMITY_EDGE_FROM_SURFACE_CM:.0f}cm + body radius)"
    )
    return cached, True


def planning_clearance_exempt_actor_names(registry) -> Tuple[str, ...]:
    """Actors allowed within 1 m during planning (material + humanoid)."""
    return (registry.material_actor_name, registry.humanoid_actor_name)


def horizontal_extent_cm(bounds: ActorBounds) -> float:
    return math.hypot(bounds.half_x, bounds.half_y)
