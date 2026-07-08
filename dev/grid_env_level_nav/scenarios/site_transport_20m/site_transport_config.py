#!/usr/bin/env python3
"""Navigation profiles for site_transport_20m (default vs fast quick-wins)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class NavProfile:
    """Tunable navigation parameters; applied to layered_nav module globals."""

    name: str
    perception_interval_s: float = 1.0
    l2_replan_cell_delta_threshold: int = 1
    enable_l1_by_default: bool = True
    site_max_open_loop_move_cm: float = 120.0
    moves_per_cycle: int = 2
    site_robot_speed: float = 180.0
    nav_warmup_settle_s: float = 4.0
    post_motion_settle_s: float = 0.15
    pre_leg1_settle_s: float = 6.0
    depth_stride_px: int = 6
    planning_clearance_cm: float = 100.0
    perception_standoff_cm: float = 50.0
    standoff_backoff_max_cm: float = 80.0
    standoff_backoff_speed: float = 120.0
    sight_registry_every_n: int = 1
    max_turn_deg_per_step: float = 18.0
    depth_cache_ttl_s: float = 0.3
    depth_pose_delta_max_cm: float = 5.0
    depth_move_invalidate_cm: float = 30.0
    depth_camera_settle_s: float = 0.08
    standoff_evict_cone_half_deg: float = 45.0
    standoff_evict_depth_margin_cm: float = 15.0
    use_rpp_controller: bool = False
    rpp_lookahead_cm: float = 80.0
    rpp_regulated_min_radius_cm: float = 120.0
    segment_chunk_max_move_cm: float = 50.0
    open_loop_distance_scale: float = 1.0
    local_costmap_size_cm: float = 600.0
    local_costmap_resolution_cm: float = 50.0
    controller_hz: float = 5.0


DEFAULT_PROFILE = NavProfile(
    name="default",
    use_rpp_controller=True,
    perception_interval_s=2.0,
    l2_replan_cell_delta_threshold=5,
    sight_registry_every_n=2,
    depth_cache_ttl_s=0.5,
    depth_pose_delta_max_cm=10.0,
    depth_move_invalidate_cm=80.0,
    depth_camera_settle_s=0.06,
)

FAST_PROFILE = NavProfile(
    name="fast",
    perception_interval_s=5.5,
    l2_replan_cell_delta_threshold=10,
    enable_l1_by_default=True,
    site_max_open_loop_move_cm=250.0,
    moves_per_cycle=3,
    site_robot_speed=285.0,
    nav_warmup_settle_s=1.0,
    post_motion_settle_s=0.05,
    pre_leg1_settle_s=1.0,
    depth_stride_px=12,
    planning_clearance_cm=150.0,
    perception_standoff_cm=100.0,
    standoff_evict_cone_half_deg=50.0,
    standoff_evict_depth_margin_cm=20.0,
    standoff_backoff_max_cm=100.0,
    standoff_backoff_speed=140.0,
    sight_registry_every_n=2,
    max_turn_deg_per_step=27.0,
    depth_cache_ttl_s=0.55,
    depth_pose_delta_max_cm=12.0,
    depth_move_invalidate_cm=120.0,
    depth_camera_settle_s=0.05,
    use_rpp_controller=False,
    rpp_lookahead_cm=100.0,
    rpp_regulated_min_radius_cm=150.0,
    segment_chunk_max_move_cm=70.0,
    open_loop_distance_scale=1.05,
)

from navmesh_config import (
    NAVMESH_MOVES_PER_CYCLE,
    NAVMESH_NAV_WARMUP_SETTLE_S,
    NAVMESH_PERCEPTION_INTERVAL_S,
    NAVMESH_POST_MOTION_SETTLE_S,
    NAVMESH_PRE_LEG1_SETTLE_S,
)

NAVMESH_PROFILE = NavProfile(
    name="navmesh",
    perception_interval_s=NAVMESH_PERCEPTION_INTERVAL_S,
    l2_replan_cell_delta_threshold=999,
    enable_l1_by_default=False,
    site_max_open_loop_move_cm=100.0,
    moves_per_cycle=NAVMESH_MOVES_PER_CYCLE,
    site_robot_speed=220.0,
    nav_warmup_settle_s=NAVMESH_NAV_WARMUP_SETTLE_S,
    post_motion_settle_s=NAVMESH_POST_MOTION_SETTLE_S,
    pre_leg1_settle_s=NAVMESH_PRE_LEG1_SETTLE_S,
    depth_stride_px=12,
    planning_clearance_cm=100.0,
    perception_standoff_cm=100.0,
    sight_registry_every_n=2,
    use_rpp_controller=False,
    segment_chunk_max_move_cm=60.0,
)

PROFILES: Dict[str, NavProfile] = {
    "default": DEFAULT_PROFILE,
    "careful": DEFAULT_PROFILE,
    "fast": FAST_PROFILE,
    "navmesh": NAVMESH_PROFILE,
}

PERCEPTION_STANDOFF_CM = FAST_PROFILE.perception_standoff_cm


def resolve_profile(name: str) -> NavProfile:
    key = (name or "default").strip().lower()
    if key not in PROFILES:
        known = ", ".join(sorted(PROFILES))
        raise ValueError(f"unknown profile {name!r}; choose from: {known}")
    return PROFILES[key]


def apply_profile_to_layered_nav(profile: NavProfile) -> None:
    """Push profile values into layered_nav module-level constants."""
    import layered_nav as ln  # noqa: WPS433 — intentional late import

    ln.SITE_DEFAULT_PERCEPTION_INTERVAL_S = profile.perception_interval_s
    ln.L2_REPLAN_CELL_DELTA_THRESHOLD = profile.l2_replan_cell_delta_threshold
    ln.SITE_MAX_OPEN_LOOP_MOVE_CM = profile.site_max_open_loop_move_cm
    ln.MOVES_PER_CYCLE = profile.moves_per_cycle
    ln.SITE_ROBOT_SPEED = profile.site_robot_speed
    ln.NAV_WARMUP_SETTLE_S = profile.nav_warmup_settle_s
    ln.POST_MOTION_SETTLE_S = profile.post_motion_settle_s
    ln.SITE_PLANNING_CLEARANCE_CM = profile.planning_clearance_cm
    ln.PERCEPTION_STANDOFF_CM = profile.perception_standoff_cm
    ln.STANDOFF_BACKOFF_MAX_CM = profile.standoff_backoff_max_cm
    ln.STANDOFF_BACKOFF_SPEED = profile.standoff_backoff_speed
    ln.STANDOFF_EVICT_CONE_HALF_DEG = profile.standoff_evict_cone_half_deg
    ln.STANDOFF_EVICT_DEPTH_MARGIN_CM = profile.standoff_evict_depth_margin_cm
    ln.MAX_TURN_DEG_PER_STEP = profile.max_turn_deg_per_step
    ln.USE_RPP_CONTROLLER = profile.use_rpp_controller
    ln.RPP_LOOKAHEAD_CM = profile.rpp_lookahead_cm
    ln.RPP_REGULATED_MIN_RADIUS_CM = profile.rpp_regulated_min_radius_cm
    ln.SEGMENT_CHUNK_MAX_MOVE_CM = profile.segment_chunk_max_move_cm
    ln.OPEN_LOOP_DISTANCE_SCALE = profile.open_loop_distance_scale
    ln.SIGHT_REGISTRY_EVERY_N = profile.sight_registry_every_n
