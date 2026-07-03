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
    NAV_OBSTACLE_HALF_HEIGHT_CM,
    NAV_REBUILD_SETTLE_S,
    PROXIMITY_CENTER_FROM_SURFACE_CM,
)

WorldXYZ = Tuple[float, float, float]


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
) -> bool:
    half = (
        max(5.0, bounds.half_x),
        max(5.0, bounds.half_y),
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
    return True


def register_props_from_registry(
    ucv,
    nav_actor: str,
    registry,
    *,
    skip_transport_target: bool = True,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> Tuple[Dict[str, ActorBounds], int]:
    """Register static prop AABB obstacles (surface boundary, no extra inflation)."""
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
        if register_box_obstacle(
            ucv, nav_actor, bounds, nav_timing=nav_timing
        ):
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
    bounds = fetch_actor_bounds(
        ucv, nav_actor, humanoid_actor_name, nav_timing=nav_timing
    )
    if bounds is None:
        return None
    human_bounds = ActorBounds(
        actor_name=humanoid_actor_name,
        cx=bounds.cx,
        cy=bounds.cy,
        cz=bounds.cz,
        half_x=bounds.half_x,
        half_y=bounds.half_y,
        half_z=bounds.half_z,
    )
    half = (
        max(5.0, human_bounds.half_x),
        max(5.0, human_bounds.half_y),
        max(5.0, NAV_OBSTACLE_HALF_HEIGHT_CM),
    )
    t0 = time.perf_counter()
    raw = nq.nav_register_box_obstacle(
        ucv,
        nav_actor,
        HUMANOID_NAV_OBSTACLE_ID,
        human_bounds.center_xyz(),
        half,
    )
    if nav_timing is not None:
        nav_timing.record_elapsed("nav_register_ms", t0)
        nav_timing.nav_register_count += 1
    if not raw.get("ok"):
        print(f"[NavMeshObs] humanoid obstacle failed: {raw.get('error', raw)}")
        return None
    return human_bounds


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
        f"planning agent_radius={PROXIMITY_CENTER_FROM_SURFACE_CM:.0f}cm"
    )
    return cached, True


def horizontal_extent_cm(bounds: ActorBounds) -> float:
    return math.hypot(bounds.half_x, bounds.half_y)
