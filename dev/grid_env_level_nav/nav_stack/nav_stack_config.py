"""Runtime navigation config derived from NavProfile (globals replacement)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from site_transport_config import NavProfile


@dataclass(frozen=True)
class NavStackConfig:
    perception_interval_s: float = 1.0
    l2_replan_cell_delta_threshold: int = 1
    site_max_open_loop_move_cm: float = 120.0
    moves_per_cycle: int = 2
    site_robot_speed: float = 180.0
    nav_warmup_settle_s: float = 4.0
    post_motion_settle_s: float = 0.15
    site_planning_clearance_cm: float = 100.0
    site_planning_clearance_cost: float = 300.0
    perception_standoff_cm: float = 50.0
    standoff_backoff_max_cm: float = 80.0
    standoff_backoff_speed: float = 120.0
    standoff_evict_cone_half_deg: float = 45.0
    standoff_evict_depth_margin_cm: float = 15.0
    max_turn_deg_per_step: float = 18.0
    use_rpp_controller: bool = False
    rpp_lookahead_cm: float = 80.0
    rpp_regulated_min_radius_cm: float = 120.0
    segment_chunk_max_move_cm: float = 50.0
    open_loop_distance_scale: float = 1.0
    local_costmap_size_cm: float = 600.0
    local_costmap_resolution_cm: float = 50.0
    sight_registry_every_n: int = 1

    @classmethod
    def from_profile(cls, profile: NavProfile) -> NavStackConfig:
        return cls(
            perception_interval_s=profile.perception_interval_s,
            l2_replan_cell_delta_threshold=profile.l2_replan_cell_delta_threshold,
            site_max_open_loop_move_cm=profile.site_max_open_loop_move_cm,
            moves_per_cycle=profile.moves_per_cycle,
            site_robot_speed=profile.site_robot_speed,
            nav_warmup_settle_s=profile.nav_warmup_settle_s,
            post_motion_settle_s=profile.post_motion_settle_s,
            site_planning_clearance_cm=profile.planning_clearance_cm,
            perception_standoff_cm=profile.perception_standoff_cm,
            standoff_backoff_max_cm=profile.standoff_backoff_max_cm,
            standoff_backoff_speed=profile.standoff_backoff_speed,
            standoff_evict_cone_half_deg=profile.standoff_evict_cone_half_deg,
            standoff_evict_depth_margin_cm=profile.standoff_evict_depth_margin_cm,
            max_turn_deg_per_step=profile.max_turn_deg_per_step,
            use_rpp_controller=profile.use_rpp_controller,
            rpp_lookahead_cm=profile.rpp_lookahead_cm,
            rpp_regulated_min_radius_cm=profile.rpp_regulated_min_radius_cm,
            segment_chunk_max_move_cm=profile.segment_chunk_max_move_cm,
            open_loop_distance_scale=profile.open_loop_distance_scale,
            local_costmap_size_cm=profile.local_costmap_size_cm,
            local_costmap_resolution_cm=profile.local_costmap_resolution_cm,
            sight_registry_every_n=profile.sight_registry_every_n,
        )

    @classmethod
    def from_layered_nav_globals(cls, ln: Any) -> NavStackConfig:
        return cls(
            perception_interval_s=ln.SITE_DEFAULT_PERCEPTION_INTERVAL_S,
            l2_replan_cell_delta_threshold=ln.L2_REPLAN_CELL_DELTA_THRESHOLD,
            site_max_open_loop_move_cm=ln.SITE_MAX_OPEN_LOOP_MOVE_CM,
            moves_per_cycle=ln.MOVES_PER_CYCLE,
            site_robot_speed=ln.SITE_ROBOT_SPEED,
            nav_warmup_settle_s=ln.NAV_WARMUP_SETTLE_S,
            post_motion_settle_s=ln.POST_MOTION_SETTLE_S,
            site_planning_clearance_cm=ln.SITE_PLANNING_CLEARANCE_CM,
            site_planning_clearance_cost=ln.SITE_PLANNING_CLEARANCE_COST,
            perception_standoff_cm=ln.PERCEPTION_STANDOFF_CM,
            standoff_backoff_max_cm=ln.STANDOFF_BACKOFF_MAX_CM,
            standoff_backoff_speed=ln.STANDOFF_BACKOFF_SPEED,
            standoff_evict_cone_half_deg=ln.STANDOFF_EVICT_CONE_HALF_DEG,
            standoff_evict_depth_margin_cm=ln.STANDOFF_EVICT_DEPTH_MARGIN_CM,
            max_turn_deg_per_step=ln.MAX_TURN_DEG_PER_STEP,
            use_rpp_controller=ln.USE_RPP_CONTROLLER,
            rpp_lookahead_cm=ln.RPP_LOOKAHEAD_CM,
            rpp_regulated_min_radius_cm=ln.RPP_REGULATED_MIN_RADIUS_CM,
            segment_chunk_max_move_cm=ln.SEGMENT_CHUNK_MAX_MOVE_CM,
            open_loop_distance_scale=ln.OPEN_LOOP_DISTANCE_SCALE,
            sight_registry_every_n=getattr(ln, "SIGHT_REGISTRY_EVERY_N", 1),
        )
