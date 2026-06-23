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


DEFAULT_PROFILE = NavProfile(name="default")

FAST_PROFILE = NavProfile(
    name="fast",
    perception_interval_s=5.0,
    l2_replan_cell_delta_threshold=5,
    enable_l1_by_default=True,
    site_max_open_loop_move_cm=250.0,
    moves_per_cycle=4,
    site_robot_speed=275.0,
    nav_warmup_settle_s=1.0,
    post_motion_settle_s=0.05,
    pre_leg1_settle_s=1.0,
    depth_stride_px=12,
)

PROFILES: Dict[str, NavProfile] = {
    "default": DEFAULT_PROFILE,
    "careful": DEFAULT_PROFILE,
    "fast": FAST_PROFILE,
}


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
