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


DEFAULT_PROFILE = NavProfile(name="default")

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
    planning_clearance_cm=100.0,
    perception_standoff_cm=100.0,
    standoff_backoff_max_cm=100.0,
    standoff_backoff_speed=140.0,
    sight_registry_every_n=2,
    max_turn_deg_per_step=27.0,
    depth_cache_ttl_s=0.55,
    depth_pose_delta_max_cm=12.0,
    depth_move_invalidate_cm=120.0,
    depth_camera_settle_s=0.05,
)

PROFILES: Dict[str, NavProfile] = {
    "default": DEFAULT_PROFILE,
    "careful": DEFAULT_PROFILE,
    "fast": FAST_PROFILE,
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
    ln.MAX_TURN_DEG_PER_STEP = profile.max_turn_deg_per_step
