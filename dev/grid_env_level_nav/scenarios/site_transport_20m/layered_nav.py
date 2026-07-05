#!/usr/bin/env python3
"""L0+L1+L2 navigation with L2 perception updates, carry sync, and motion sampling."""

from __future__ import annotations

import math
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, MutableMapping, Optional, Sequence, Set, Tuple, Union

import numpy as np

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
from costmap_layers import LayeredCostmap  # noqa: E402
from path_planning_costmap import Costmap2D, world_segment_is_traversable  # noqa: E402
from grid_env_10k_pie_patrol import (  # noqa: E402
    PATH_MAX_STEPS_PER_WP,
    PATH_MAX_TOTAL_STEPS,
    PATH_WP_REACH_TOLERANCE_CM,
    ROBOT_TURN_DUR_S,
    SegmentCommand,
    _nearest_waypoint_index_ahead,
    dist2d,
    get_pos2d,
    get_yaw,
    plan_astar_waypoints,
    yaw_to_target,
)
from carry import is_carry_ue_attached, sync_carry_pose  # noqa: E402
from l2_fusion import apply_l2_from_fusion_detections, detections_summary  # noqa: E402
from level_coords import local_xy_to_world, world_xy_to_local  # noqa: E402
from level_nav_robot import is_robot_tipped, recover_robot_upright  # noqa: E402
from perception_layer import L2_LETHAL_COST  # noqa: E402
from pie_safety import PieSessionLost, require_live_ucv, tick_settle  # noqa: E402
from pie_spawn_safety import ensure_live_or_reconnect  # noqa: E402
from viz import NavTrace  # noqa: E402
from metrics import NavTimingAccumulator  # noqa: E402
from nav_pose_query import (  # noqa: E402
    fetch_nav_pose,
    invalidate_robot_pose,
    sync_robot_pose_cache,
)
from nav_stack.nav_types import PerceptionOutcome as PerceiveOutcome  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

WorldXY = Tuple[float, float]
GridCell = Tuple[int, int]
ROBOT_ACTOR = geh.ROBOT_ACTOR_NAME
PerceiveFn = Callable[..., Union[list, "PerceiveOutcome"]]
PoseSampleFn = Callable[[WorldXY, float], None]
CarrySyncFn = Callable[[], None]
PoseCache = MutableMapping[str, Any]


@contextmanager
def _loop_bucket(nav_timing: Optional[NavTimingAccumulator], attr: str):
    if nav_timing is None:
        yield
        return
    t0 = time.perf_counter()
    try:
        yield
    finally:
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        setattr(nav_timing, attr, getattr(nav_timing, attr) + elapsed_ms)


def _loop_bucket_total_ms(nav_timing: NavTimingAccumulator) -> float:
    return (
        nav_timing.waypoint_select_ms
        + nav_timing.stuck_check_ms
        + nav_timing.replan_decision_ms
        + nav_timing.costmap_scan_ms
        + nav_timing.nav_branch_ms
    )


def _account_outer_loop_slice(
    loop_t0: float,
    accounted_at_start: float,
    loop_bucket_at_start: float,
    nav_timing: Optional[NavTimingAccumulator],
) -> None:
    if nav_timing is None:
        return
    slice_ms = (time.perf_counter() - loop_t0) * 1000.0
    bucketed_ms = nav_timing.accounted_ms() - accounted_at_start
    loop_sub_ms = _loop_bucket_total_ms(nav_timing) - loop_bucket_at_start
    gap = slice_ms - bucketed_ms - loop_sub_ms
    if gap > 0.0:
        nav_timing.loop_residual_ms += gap
    nav_timing.loop_overhead_ms += loop_sub_ms + max(0.0, gap)


def _sync_pose_cache(
    pose_cache: Optional[PoseCache],
    pos_xy: WorldXY,
    yaw_deg: float,
) -> None:
    sync_robot_pose_cache(pose_cache, pos_xy, yaw_deg)


def _read_pose_cache(
    pose_cache: Optional[PoseCache],
) -> Optional[Tuple[WorldXY, float]]:
    if pose_cache is None:
        return None
    xy = pose_cache.get("xy")
    yaw = pose_cache.get("yaw")
    if xy is None or yaw is None:
        return None
    return xy, float(yaw)


def _fetch_nav_pose(
    ucv: UnrealCV,
    robot_name: str,
    nav_timing: Optional[NavTimingAccumulator],
    pose_cache: Optional[PoseCache],
    *,
    force: bool = False,
) -> Tuple[WorldXY, float, UnrealCV]:
    try:
        return fetch_nav_pose(
            ucv,
            robot_name,
            nav_timing,
            pose_cache,
            force=force,
        )
    except (ConnectionError, AttributeError, OSError, ValueError, TypeError) as exc:
        print(f"  [SiteNav] fetch_nav_pose failed ({exc}) — reconnect")
        try:
            ucv = ensure_live_or_reconnect(ucv, reason="fetch_nav_pose")
            return fetch_nav_pose(
                ucv,
                robot_name,
                nav_timing,
                pose_cache,
                force=True,
            )
        except (ConnectionError, OSError, RuntimeError, PieSessionLost) as exc2:
            raise PieSessionLost(f"UnrealCV reconnect failed during nav: {exc2}") from exc2


def _invoke_perceive(
    perceive_fn: PerceiveFn,
    *,
    layers: LayeredCostmap,
    l2_seen_cells: Set[GridCell],
) -> PerceiveOutcome:
    try:
        raw = perceive_fn(layers=layers, l2_seen_cells=l2_seen_cells)
    except TypeError:
        raw = perceive_fn()
    if isinstance(raw, PerceiveOutcome):
        return raw
    return PerceiveOutcome(detections=list(raw or []), l2_applied=False)

SITE_DEFAULT_PERCEPTION_INTERVAL_S = 1.0
L2_REPLAN_CELL_DELTA_THRESHOLD = 1
PERCEPTION_START_DELAY_S = 0.0
MOTION_SETTLE_BEFORE_PERCEIVE_S = 0.2
POST_MOTION_SETTLE_S = 0.15
MOVES_PER_CYCLE = 2
SITE_ROBOT_SPEED = 180.0
SITE_MOVE_SLICE_S = 0.18
NAV_WARMUP_SETTLE_S = 4.0
FIRST_MOVE_PRIME_CM = 20.0
SITE_ROTATE_THR_DEG = 12.0
SITE_SMOOTH_TURN_MOVE_DEG = 35.0
SITE_MAX_OPEN_LOOP_MOVE_CM = 120.0
MAX_TURN_DEG_PER_STEP = 18.0
TURN_SLEEP_FRAC = 0.35
STUCK_MOVE_THRESHOLD_CM = 14.0
STUCK_CHECK_MOVES = 4
UNSTUCK_BACKUP_CM = 100.0
UNSTUCK_BACK_SPEED = 100.0
ESCAPE_STEP_MIN_CM = 70.0
ESCAPE_STEP_MAX_CM = 150.0
ESCAPE_MAX_TURN_DEG = 135.0
MAX_UNSTUCK_ATTEMPTS = 16
STUCK_CORRIDOR_LENGTH_CM = 120.0
STUCK_CORRIDOR_HALF_WIDTH_CELLS = 2
STUCK_HOTSPOT_RADIUS_CELLS = 2
ESCAPE_MIN_DISPLACEMENT_CM = 35.0
ESCAPE_CANDIDATE_LIMIT = 8

PROGRESS_REGRESS_THRESHOLD_CM = 350.0
MAX_TURN_ONLY_STEPS = 12
MAX_L2_FLUSH_COUNT = 3
LAST_RESORT_PERCEIVE_PAUSE_STEPS = 40
L2_REPLAN_THRASH_POS_CM = 35.0
L2_REPLAN_THRASH_GOAL_CM = 50.0
L2_REPLAN_THRASH_MAX = 4
GOAL_DIST_STALL_STEPS = 45
GOAL_DIST_STALL_IMPROVE_CM = 25.0
SITE_PLANNING_CLEARANCE_CM = 100.0
SITE_PLANNING_CLEARANCE_COST = 300.0
ROBOT_L2_SELF_EXCLUDE_RADIUS_CM = 70.0
ROBOT_BODY_CLEARANCE_CM = 45.0
NEAR_OBSTACLE_SLOW_CM = 220.0

PERCEPTION_STANDOFF_CM = 0.0
STANDOFF_BACKOFF_SPEED = 120.0
STANDOFF_BACKOFF_MAX_CM = 80.0
STANDOFF_EVICT_CONE_HALF_DEG = 55.0
STANDOFF_EVICT_DEPTH_MARGIN_CM = 10.0
USE_RPP_CONTROLLER = False
RPP_LOOKAHEAD_CM = 80.0
RPP_REGULATED_MIN_RADIUS_CM = 120.0
SEGMENT_CHUNK_MAX_MOVE_CM = 50.0
OPEN_LOOP_DISTANCE_SCALE = 1.0
SIGHT_REGISTRY_EVERY_N = 1


def _stack_config_from_ctx(nav_ctx: Optional[Any]) -> Any:
    from nav_stack.nav_stack_config import NavStackConfig  # noqa: WPS433

    if nav_ctx is not None and getattr(nav_ctx, "stack_config", None) is not None:
        return nav_ctx.stack_config
    return NavStackConfig.from_layered_nav_globals(sys.modules[__name__])


def _kpi_from_ctx(nav_ctx: Optional[Any]) -> Optional[Any]:
    if nav_ctx is None:
        return None
    return getattr(nav_ctx, "kpi", None)


def _l2_cell_delta_warrants_replan(cells_added: int, cells_removed: int = 0) -> bool:
    """Replan on L2 updates only when cell delta exceeds threshold (path-block uses separate replan)."""
    from nav_stack.l2_replan_policy import l2_cell_delta_warrants_replan  # noqa: WPS433

    return l2_cell_delta_warrants_replan(
        cells_added,
        cells_removed,
        threshold=L2_REPLAN_CELL_DELTA_THRESHOLD,
    )


def _nav_settle(
    ucv: UnrealCV,
    *,
    settle_s: float,
    ticks: int,
    nav_timing: Optional[NavTimingAccumulator],
) -> None:
    t0 = time.perf_counter()
    tick_settle(ucv, settle_s=settle_s, ticks=ticks)
    if nav_timing is not None:
        nav_timing.settle_ms += (time.perf_counter() - t0) * 1000.0


def _timed_replan_on_merged_layers(
    layers: LayeredCostmap,
    pos_xy: WorldXY,
    goal_xy: WorldXY,
    *,
    reason: str,
    trace: Optional[NavTrace],
    l2_seen_cells: Set[Tuple[int, int]],
    nav_timing: Optional[NavTimingAccumulator],
    nav_kpi: Optional[Any] = None,
) -> Optional[list]:
    t0 = time.perf_counter()
    result = _replan_on_merged_layers(
        layers,
        pos_xy,
        goal_xy,
        reason=reason,
        trace=trace,
        l2_seen_cells=l2_seen_cells,
        nav_kpi=nav_kpi,
    )
    if nav_timing is not None:
        nav_timing.replan_ms += (time.perf_counter() - t0) * 1000.0
    return result


def _timed_planning_costmap(
    layers: LayeredCostmap,
    nav_timing: Optional[NavTimingAccumulator],
) -> Costmap2D:
    t0 = time.perf_counter()
    result = _planning_costmap(layers)
    if nav_timing is not None:
        nav_timing.costmap_scan_ms += (time.perf_counter() - t0) * 1000.0
    return result


def _planning_costmap(layers: LayeredCostmap) -> Costmap2D:
    """Merged map plus soft 1m clearance cost around lethal cells for planning."""
    from nav_stack.global_costmap import build_planning_costmap  # noqa: WPS433

    return build_planning_costmap(
        layers,
        planning_clearance_cm=SITE_PLANNING_CLEARANCE_CM,
        planning_clearance_cost=SITE_PLANNING_CLEARANCE_COST,
    )


def _site_dog_move(
    ucv: UnrealCV,
    robot_name: str,
    speed: float,
    duration_s: float,
    direction: int = 0,
) -> None:
    """Open-loop move with a single wait (UnrealCV.dog_move sleeps twice if used directly)."""
    cmd = f"vbp {robot_name} Move_Speed {speed} {duration_s} {direction}"
    geh._ue_request(ucv, cmd, timeout_s=max(20.0, duration_s + 10.0))  # noqa: SLF001
    time.sleep(duration_s)


def _site_dog_rotate(
    ucv: UnrealCV,
    robot_name: str,
    duration_s: float,
    angle_deg: float,
    clockwise: int,
) -> None:
    cmd = f"vbp {robot_name} Rotate_Angle {duration_s} {angle_deg} {clockwise}"
    geh._ue_request(ucv, cmd, timeout_s=max(20.0, duration_s + 10.0))  # noqa: SLF001
    time.sleep(duration_s)


def _dynamic_max_move_cm(
    nearest_dist_cm: float,
    forward_depth_cm: Optional[float],
) -> float:
    """Shrink open-loop moves when L2 or forward depth shows nearby obstacles."""
    from nav_stack.controllers.velocity_scaler import (  # noqa: WPS433
        VelocityScaleConfig,
        dynamic_max_move_cm,
    )

    return dynamic_max_move_cm(
        nearest_dist_cm,
        forward_depth_cm,
        config=VelocityScaleConfig(
            max_move_cm=SITE_MAX_OPEN_LOOP_MOVE_CM,
            near_obstacle_slow_cm=NEAR_OBSTACLE_SLOW_CM,
            perception_standoff_cm=PERCEPTION_STANDOFF_CM,
        ),
    )


def _rpp_config():
    from nav_stack.controllers.rpp import RppConfig  # noqa: WPS433

    return RppConfig(
        lookahead_cm=RPP_LOOKAHEAD_CM,
        regulated_linear_scaling_min_radius_cm=RPP_REGULATED_MIN_RADIUS_CM,
        rotate_to_heading_threshold_deg=SITE_SMOOTH_TURN_MOVE_DEG,
        rotate_thr_deg=SITE_ROTATE_THR_DEG,
    )


def _get_controller_server():
    from nav_stack.controller_server import ControllerServer, ControllerServerConfig  # noqa: WPS433
    from nav_stack.controllers.segment_executor import SegmentExecutorConfig  # noqa: WPS433
    from nav_stack.controllers.velocity_scaler import VelocityScaleConfig  # noqa: WPS433

    return ControllerServer(
        ControllerServerConfig(
            use_rpp=USE_RPP_CONTROLLER,
            rpp=_rpp_config(),
            executor=SegmentExecutorConfig(
                chunk_max_move_cm=SEGMENT_CHUNK_MAX_MOVE_CM,
                chunk_max_turn_deg=MAX_TURN_DEG_PER_STEP,
                open_loop_distance_scale=OPEN_LOOP_DISTANCE_SCALE,
            ),
            velocity=VelocityScaleConfig(
                max_move_cm=SITE_MAX_OPEN_LOOP_MOVE_CM,
                near_obstacle_slow_cm=NEAR_OBSTACLE_SLOW_CM,
                perception_standoff_cm=PERCEPTION_STANDOFF_CM,
            ),
            wp_reach_tolerance_cm=PATH_WP_REACH_TOLERANCE_CM,
        )
    )


def _segment_command_for_waypoint(
    pos_xy: WorldXY,
    yaw_deg: float,
    waypoint_xy: WorldXY,
    *,
    waypoints: Optional[Sequence[WorldXY]] = None,
    wp_index: int = 0,
) -> Optional[SegmentCommand]:
    server = _get_controller_server()
    return server.compute_segment_command(
        pos_xy,
        yaw_deg,
        waypoint_xy,
        waypoints or [],
        wp_index,
        legacy_command_fn=_smooth_segment_command,
    )


def _clamp_segment_move(command: SegmentCommand, max_move_cm: float) -> SegmentCommand:
    if command.move_cm <= max_move_cm:
        return command
    return SegmentCommand(
        turn_deg=command.turn_deg,
        turn_clockwise=command.turn_clockwise,
        move_cm=max(0.0, max_move_cm),
    )


def _nearest_standoff_dist_cm(
    pos_xy: WorldXY,
    layers: LayeredCostmap,
    registry_positions: Sequence[WorldXY],
    *,
    forward_depth_cm: Optional[float] = None,
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> float:
    from perception_standoff import check_perception_standoff  # noqa: WPS433

    if PERCEPTION_STANDOFF_CM <= 0.0:
        return float("inf")
    t0 = time.perf_counter()
    check = check_perception_standoff(
        pos_xy,
        layers,
        registry_positions=registry_positions,
        standoff_cm=PERCEPTION_STANDOFF_CM,
        forward_depth_cm=forward_depth_cm,
    )
    if nav_timing is not None:
        nav_timing.costmap_scan_ms += (time.perf_counter() - t0) * 1000.0
    return check.nearest_dist_cm


def _maybe_evict_stale_l2_for_depth(
    ucv: UnrealCV,
    robot_name: str,
    pos_xy: WorldXY,
    layers: LayeredCostmap,
    *,
    forward_depth_cm: Optional[float],
    l2_seen_cells: Optional[Set[Tuple[int, int]]],
    registry_positions: Sequence[WorldXY] = (),
    robot_yaw_deg: Optional[float] = None,
) -> None:
    from perception_standoff import (  # noqa: WPS433
        depth_confirms_clearance,
        evict_stale_l2_in_forward_cone,
    )

    if not depth_confirms_clearance(forward_depth_cm, PERCEPTION_STANDOFF_CM):
        return
    if robot_yaw_deg is None:
        try:
            robot_yaw_deg = get_yaw(ucv, robot_name)
        except (ConnectionError, OSError, ValueError, RuntimeError):
            return
    evict_stale_l2_in_forward_cone(
        pos_xy,
        robot_yaw_deg,
        layers,
        forward_depth_cm=float(forward_depth_cm),  # type: ignore[arg-type]
        standoff_cm=PERCEPTION_STANDOFF_CM,
        l2_seen_cells=l2_seen_cells,
        registry_positions=registry_positions,
        cone_half_deg=STANDOFF_EVICT_CONE_HALF_DEG,
        depth_margin_cm=STANDOFF_EVICT_DEPTH_MARGIN_CM,
    )


def _depth_backoff_move(
    ucv: UnrealCV,
    robot_name: str,
    *,
    forward_depth_cm: float,
    carry_motion_cb: Optional[CarrySyncFn],
    nav_timing: Optional[NavTimingAccumulator] = None,
) -> None:
    """Reverse when forward depth is inside standoff but no L2 anchor exists."""
    t0 = time.perf_counter()
    deficit_cm = max(0.0, PERCEPTION_STANDOFF_CM - forward_depth_cm + 20.0)
    backup_cm = min(STANDOFF_BACKOFF_MAX_CM, max(25.0, deficit_cm))
    duration_s = max(0.25, backup_cm / STANDOFF_BACKOFF_SPEED)
    print(
        f"  [SiteNav] depth standoff: forward {forward_depth_cm:.0f}cm "
        f"< {PERCEPTION_STANDOFF_CM:.0f}cm → reverse {backup_cm:.0f}cm"
    )
    _site_dog_move(ucv, robot_name, -STANDOFF_BACKOFF_SPEED, duration_s, direction=0)
    if carry_motion_cb is not None:
        carry_motion_cb()
    if nav_timing is not None:
        nav_timing.depth_reverse_ms += (time.perf_counter() - t0) * 1000.0


def _invalidate_pose_after_standoff_motion(pose_cache: Optional[PoseCache]) -> None:
    invalidate_robot_pose(pose_cache, reason="standoff_motion")


def _ensure_move_standoff(
    ucv: UnrealCV,
    robot_name: str,
    pos_xy: WorldXY,
    layers: LayeredCostmap,
    *,
    registry_positions: Sequence[WorldXY],
    forward_depth_cm: Optional[float],
    carry_motion_cb: Optional[CarrySyncFn],
    nav_timing: Optional[NavTimingAccumulator],
    depth_refresh_fn: Optional[Callable[[], Optional[float]]] = None,
    l2_seen_cells: Optional[Set[Tuple[int, int]]] = None,
    depth_invalidate_fn: Optional[Callable[[str], None]] = None,
    robot_yaw_deg: Optional[float] = None,
    pose_cache: Optional[PoseCache] = None,
) -> Tuple[bool, WorldXY, Optional[float], float]:
    """Backoff before forward motion when map or depth violates standoff."""
    from perception_standoff import check_perception_standoff  # noqa: WPS433

    if PERCEPTION_STANDOFF_CM <= 0.0:
        return True, pos_xy, forward_depth_cm, float("inf")

    if forward_depth_cm is None and depth_refresh_fn is not None:
        forward_depth_cm = depth_refresh_fn()

    _maybe_evict_stale_l2_for_depth(
        ucv,
        robot_name,
        pos_xy,
        layers,
        forward_depth_cm=forward_depth_cm,
        l2_seen_cells=l2_seen_cells,
        registry_positions=registry_positions,
        robot_yaw_deg=robot_yaw_deg,
    )

    standoff = check_perception_standoff(
        pos_xy,
        layers,
        registry_positions=registry_positions,
        standoff_cm=PERCEPTION_STANDOFF_CM,
        forward_depth_cm=forward_depth_cm,
    )
    nearest_dist = standoff.nearest_dist_cm
    depth_violation = (
        forward_depth_cm is not None
        and forward_depth_cm < PERCEPTION_STANDOFF_CM - 5.0
        and not standoff.needs_backoff(PERCEPTION_STANDOFF_CM)
    )
    if not standoff.needs_backoff(PERCEPTION_STANDOFF_CM) and not depth_violation:
        return True, pos_xy, forward_depth_cm, nearest_dist

    standoff_t0 = time.perf_counter()
    did_backoff = False
    if standoff.needs_backoff(PERCEPTION_STANDOFF_CM) and standoff.obstacle_xy is not None:
        backoff_cm = standoff.backoff_cm(
            PERCEPTION_STANDOFF_CM,
            max_cm=STANDOFF_BACKOFF_MAX_CM,
        )
        print(
            f"  [SiteNav] move standoff: {standoff.nearest_dist_cm:.0f}cm "
            f"< {PERCEPTION_STANDOFF_CM:.0f}cm ({standoff.source}) "
            f"→ backoff {backoff_cm:.0f}cm"
        )
        backoff_t0 = time.perf_counter()
        pos_xy = _standoff_backoff_move(
            ucv,
            robot_name,
            pos_xy,
            standoff.obstacle_xy,
            backoff_cm,
        )
        _invalidate_pose_after_standoff_motion(pose_cache)
        pos_xy, robot_yaw_deg, ucv = _fetch_nav_pose(
            ucv, robot_name, nav_timing, pose_cache, force=True
        )
        if nav_timing is not None:
            nav_timing.backoff_ms += (time.perf_counter() - backoff_t0) * 1000.0
        did_backoff = True
    elif depth_violation and forward_depth_cm is not None:
        _depth_backoff_move(
            ucv,
            robot_name,
            forward_depth_cm=forward_depth_cm,
            carry_motion_cb=carry_motion_cb,
            nav_timing=nav_timing,
        )
        _invalidate_pose_after_standoff_motion(pose_cache)
        pos_xy, robot_yaw_deg, ucv = _fetch_nav_pose(
            ucv, robot_name, nav_timing, pose_cache, force=True
        )
        did_backoff = True

    if did_backoff and depth_invalidate_fn is not None:
        depth_invalidate_fn("standoff_backoff")

    if did_backoff:
        if depth_refresh_fn is not None:
            forward_depth_cm = depth_refresh_fn()
    elif forward_depth_cm is None and depth_refresh_fn is not None:
        forward_depth_cm = depth_refresh_fn()

    if carry_motion_cb is not None:
        carry_motion_cb()
    if nav_timing is not None:
        nav_timing.standoff_ms += (time.perf_counter() - standoff_t0) * 1000.0
        nav_timing.standoff_events += 1

    _maybe_evict_stale_l2_for_depth(
        ucv,
        robot_name,
        pos_xy,
        layers,
        forward_depth_cm=forward_depth_cm,
        l2_seen_cells=l2_seen_cells,
        registry_positions=registry_positions,
        robot_yaw_deg=robot_yaw_deg,
    )

    recheck = check_perception_standoff(
        pos_xy,
        layers,
        registry_positions=registry_positions,
        standoff_cm=PERCEPTION_STANDOFF_CM,
        forward_depth_cm=forward_depth_cm,
    )
    nearest_dist = recheck.nearest_dist_cm
    depth_violation = (
        forward_depth_cm is not None and forward_depth_cm < PERCEPTION_STANDOFF_CM - 5.0
    )
    if recheck.needs_backoff(PERCEPTION_STANDOFF_CM) or depth_violation:
        print(
            f"  [SiteNav] move standoff: still too close "
            f"(map={recheck.nearest_dist_cm:.0f}cm depth={forward_depth_cm})"
        )
        return False, pos_xy, forward_depth_cm, nearest_dist
    return True, pos_xy, forward_depth_cm, nearest_dist


def _open_loop_move_target(
    pos_xy: WorldXY,
    waypoint_xy: WorldXY,
    *,
    max_move_cm: float = SITE_MAX_OPEN_LOOP_MOVE_CM,
) -> WorldXY:
    dist_wp = dist2d(pos_xy, waypoint_xy)
    if dist_wp < 1e-3:
        return waypoint_xy
    move_cm = min(dist_wp, max_move_cm)
    t = move_cm / dist_wp
    return (
        pos_xy[0] + t * (waypoint_xy[0] - pos_xy[0]),
        pos_xy[1] + t * (waypoint_xy[1] - pos_xy[1]),
    )


def _smooth_segment_command(
    pos_xy: WorldXY,
    yaw_deg: float,
    waypoint_xy: WorldXY,
) -> Optional[SegmentCommand]:
    distance_cm = dist2d(pos_xy, waypoint_xy)
    if distance_cm < 1e-3:
        return None
    target_yaw = yaw_to_target(pos_xy, waypoint_xy)
    angle_diff = _normalize_angle(target_yaw - yaw_deg)
    abs_angle = abs(angle_diff)
    clockwise = 1 if angle_diff < 0.0 else -1
    if abs_angle > SITE_SMOOTH_TURN_MOVE_DEG:
        return SegmentCommand(turn_deg=abs_angle, turn_clockwise=clockwise, move_cm=0.0)
    if abs_angle > SITE_ROTATE_THR_DEG:
        return SegmentCommand(
            turn_deg=abs_angle,
            turn_clockwise=clockwise,
            move_cm=min(distance_cm, SITE_MAX_OPEN_LOOP_MOVE_CM * 0.55),
        )
    return SegmentCommand(
        turn_deg=0.0,
        turn_clockwise=1,
        move_cm=min(distance_cm, SITE_MAX_OPEN_LOOP_MOVE_CM),
    )


def _nearest_traversable_world_xy(
    costmap: Costmap2D,
    pos_xy: WorldXY,
    *,
    max_radius_cells: int = 10,
) -> WorldXY:
    start = costmap.world_xy_to_grid(pos_xy, clamp=True)
    if start is None:
        return pos_xy
    if costmap.is_traversable(start):
        return pos_xy
    sx, sy = start
    best: Optional[WorldXY] = None
    best_dist = float("inf")
    for r in range(1, max_radius_cells + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                gx, gy = sx + dx, sy + dy
                if gx < 0 or gy < 0 or gx >= costmap.width_cells or gy >= costmap.height_cells:
                    continue
                if not costmap.is_traversable((gx, gy)):
                    continue
                candidate = costmap.grid_to_world_xy_center((gx, gy))
                dist = dist2d(candidate, pos_xy)
                if dist < best_dist:
                    best_dist = dist
                    best = candidate
        if best is not None:
            return best
    return pos_xy


def _safe_replan_astar(costmap: Costmap2D, pos_xy: WorldXY, goal_xy: WorldXY):
    try:
        return plan_astar_waypoints(costmap, pos_xy, goal_xy)
    except (ValueError, RuntimeError):
        alt = _nearest_traversable_world_xy(costmap, pos_xy)
        if alt == pos_xy:
            raise ValueError(f"No traversable path from {pos_xy} to {goal_xy}") from None
        try:
            return plan_astar_waypoints(costmap, alt, goal_xy)
        except (ValueError, RuntimeError) as exc:
            raise ValueError(f"No traversable path from {pos_xy} to {goal_xy}") from exc


def _mark_cells_disk_on_l2(
    layers: LayeredCostmap,
    center: Tuple[int, int],
    l2_seen_cells: Set[Tuple[int, int]],
    *,
    radius_cells: int,
    exclude_center: bool = False,
) -> int:
    cx, cy = center
    marked = 0
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            gx, gy = cx + dx, cy + dy
            if exclude_center and gx == cx and gy == cy:
                continue
            if (gx, gy) in l2_seen_cells:
                continue
            layers.set_l2_cell(gx, gy, L2_LETHAL_COST)
            l2_seen_cells.add((gx, gy))
            marked += 1
    return marked


def _mark_stuck_corridor_on_l2(
    layers: LayeredCostmap,
    pos_xy: WorldXY,
    toward_xy: WorldXY,
    l2_seen_cells: Set[Tuple[int, int]],
    *,
    length_cm: float = STUCK_CORRIDOR_LENGTH_CM,
    half_width_cells: int = STUCK_CORRIDOR_HALF_WIDTH_CELLS,
) -> int:
    """Mark cells ahead of the robot toward the blocked waypoint as lethal."""
    costmap = layers.to_costmap2d()
    dx = toward_xy[0] - pos_xy[0]
    dy = toward_xy[1] - pos_xy[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        return 0
    ux, uy = dx / dist, dy / dist
    marked = 0
    step = max(costmap.resolution_cm * 0.5, 10.0)
    traveled = step
    while traveled <= length_cm:
        wx = pos_xy[0] + ux * traveled
        wy = pos_xy[1] + uy * traveled
        center = costmap.world_xy_to_grid((wx, wy), clamp=True)
        if center is None:
            break
        cx, cy = center
        for ddx in range(-half_width_cells, half_width_cells + 1):
            for ddy in range(-half_width_cells, half_width_cells + 1):
                gx, gy = cx + ddx, cy + ddy
                if (gx, gy) in l2_seen_cells:
                    continue
                layers.set_l2_cell(gx, gy, L2_LETHAL_COST)
                l2_seen_cells.add((gx, gy))
                marked += 1
        traveled += step
    return marked


def _mark_stuck_hotspot_on_l2(
    layers: LayeredCostmap,
    pos_xy: WorldXY,
    l2_seen_cells: Set[Tuple[int, int]],
    *,
    radius_cells: int = STUCK_HOTSPOT_RADIUS_CELLS,
) -> int:
    costmap = layers.to_costmap2d()
    center = costmap.world_xy_to_grid(pos_xy, clamp=True)
    if center is None:
        return 0
    return _mark_cells_disk_on_l2(
        layers,
        center,
        l2_seen_cells,
        radius_cells=radius_cells,
        exclude_center=True,
    )


def _mark_stuck_cells_on_l2(
    layers: LayeredCostmap,
    pos_xy: WorldXY,
    l2_seen_cells: Set[Tuple[int, int]],
    *,
    radius_cells: int = 1,
) -> int:
    costmap = layers.to_costmap2d()
    center = costmap.world_xy_to_grid(pos_xy, clamp=True)
    if center is None:
        return 0
    return _mark_cells_disk_on_l2(
        layers,
        center,
        l2_seen_cells,
        radius_cells=radius_cells,
        exclude_center=True,
    )


def _replan_on_merged_layers(
    layers: LayeredCostmap,
    pos_xy: WorldXY,
    goal_xy: WorldXY,
    *,
    reason: str,
    trace: Optional[NavTrace],
    l2_seen_cells: Set[Tuple[int, int]],
    nav_kpi: Optional[Any] = None,
) -> Optional[list]:
    """Replan using planner_server (Nav2 planner_server equivalent)."""
    from nav_stack.planner_server import replan_on_merged_layers  # noqa: WPS433

    removed_self = _exclude_robot_self_from_l2(layers, pos_xy, l2_seen_cells)
    if removed_self:
        print(f"  [SiteNav] L2 self-exclude {removed_self} cells before replan")
    result = replan_on_merged_layers(
        layers,
        pos_xy,
        goal_xy,
        planning_clearance_cm=SITE_PLANNING_CLEARANCE_CM,
        planning_clearance_cost=SITE_PLANNING_CLEARANCE_COST,
        l2_seen_cells=l2_seen_cells,
    )
    if nav_kpi is not None:
        nav_kpi.record_replan(success=bool(result.waypoints))
    if result.stage == "tight_merged":
        print("  [SiteNav] replan using tight merged L0+L1+L2 clearance")
    elif result.stage == "l0_l1":
        print("  [SiteNav] replan using L0+L1 (L2 ignored)")
    elif result.stage == "l0_only":
        print("  [SiteNav] replan skipped: L0-only path ignores L1 forbidden zones")
    elif result.stage == "failed":
        print("  [SiteNav] tight merged replan failed")
    if result.waypoints and result.stage != "l0_only":
        if trace is not None:
            trace.record_plan(result.waypoints, reason=reason)
            trace.l2_cell_count = len(l2_seen_cells)
        return result.waypoints
    return None


def _l2_occupied_count(layers: LayeredCostmap) -> int:
    return int((layers.l2 > 0).sum())


def _sync_seen_cells_from_l2(
    layers: LayeredCostmap,
    l2_seen_cells: Set[Tuple[int, int]],
) -> None:
    height, width = layers.l2.shape
    for gy in range(height):
        for gx in range(width):
            if layers.l2[gy, gx] > 0:
                l2_seen_cells.add((gx, gy))


def _exclude_robot_self_from_l2(
    layers: LayeredCostmap,
    robot_xy: WorldXY,
    l2_seen_cells: Set[Tuple[int, int]],
    *,
    radius_cm: float = ROBOT_L2_SELF_EXCLUDE_RADIUS_CM,
) -> int:
    costmap = layers.to_costmap2d()
    center = costmap.world_xy_to_grid(robot_xy, clamp=True)
    if center is None:
        return 0
    radius_cells = max(1, int(math.ceil(radius_cm / layers.resolution_cm)))
    cx, cy = center
    removed = 0
    for gy in range(max(0, cy - radius_cells), min(layers.height_cells, cy + radius_cells + 1)):
        for gx in range(max(0, cx - radius_cells), min(layers.width_cells, cx + radius_cells + 1)):
            wx, wy = costmap.grid_to_world_xy_center((gx, gy))
            if math.hypot(wx - robot_xy[0], wy - robot_xy[1]) > radius_cm:
                continue
            if layers.l2[gy, gx] <= 0:
                continue
            layers.clear_l2_cell(gx, gy)
            l2_seen_cells.discard((gx, gy))
            removed += 1
    return removed


def _unstuck_backup(ucv: UnrealCV, robot_name: str, backup_cm: float = UNSTUCK_BACKUP_CM) -> None:
    duration_s = max(0.25, backup_cm / UNSTUCK_BACK_SPEED)
    _site_dog_move(ucv, robot_name, -UNSTUCK_BACK_SPEED, duration_s, direction=0)



def _standoff_backoff_move(
    ucv: UnrealCV,
    robot_name: str,
    robot_xy: WorldXY,
    obstacle_xy: WorldXY,
    backoff_cm: float,
) -> WorldXY:
    """Turn away from obstacle and move to restore standoff distance."""
    from perception_standoff import away_bearing_deg  # noqa: WPS433

    move_cm = min(STANDOFF_BACKOFF_MAX_CM, max(20.0, backoff_cm))
    away_deg = away_bearing_deg(robot_xy, obstacle_xy)
    try:
        robot_yaw = get_yaw(ucv, robot_name)
    except (ConnectionError, OSError, ValueError, RuntimeError):
        robot_yaw = away_deg
    turn_delta = _normalize_angle(away_deg - robot_yaw)
    if abs(turn_delta) > SITE_ROTATE_THR_DEG:
        clockwise = 1 if turn_delta > 0 else 0
        _dog_rotate_chunked(ucv, robot_name, abs(turn_delta), clockwise)
    duration_s = max(0.25, move_cm / STANDOFF_BACKOFF_SPEED)
    _site_dog_move(ucv, robot_name, STANDOFF_BACKOFF_SPEED, duration_s, direction=0)
    return robot_xy


def _gate_perception_standoff(
    ucv: UnrealCV,
    robot_name: str,
    pos_xy: WorldXY,
    layers: LayeredCostmap,
    *,
    registry_positions: Sequence[WorldXY],
    carry_motion_cb: Optional[CarrySyncFn],
    nav_timing: Optional[NavTimingAccumulator],
    forward_depth_cm: Optional[float] = None,
    depth_refresh_fn: Optional[Callable[[], Optional[float]]] = None,
    l2_seen_cells: Optional[Set[Tuple[int, int]]] = None,
    depth_invalidate_fn: Optional[Callable[[str], None]] = None,
    robot_yaw_deg: Optional[float] = None,
    pose_cache: Optional[PoseCache] = None,
) -> Tuple[bool, WorldXY]:
    """Backoff when too close to obstacles; return whether to run perceive."""
    from perception_standoff import check_perception_standoff  # noqa: WPS433

    if PERCEPTION_STANDOFF_CM <= 0.0:
        return True, pos_xy

    if forward_depth_cm is None and depth_refresh_fn is not None:
        forward_depth_cm = depth_refresh_fn()

    _maybe_evict_stale_l2_for_depth(
        ucv,
        robot_name,
        pos_xy,
        layers,
        forward_depth_cm=forward_depth_cm,
        l2_seen_cells=l2_seen_cells,
        registry_positions=registry_positions,
        robot_yaw_deg=robot_yaw_deg,
    )

    standoff = check_perception_standoff(
        pos_xy,
        layers,
        registry_positions=registry_positions,
        standoff_cm=PERCEPTION_STANDOFF_CM,
        forward_depth_cm=forward_depth_cm,
    )
    if not standoff.needs_backoff(PERCEPTION_STANDOFF_CM):
        return True, pos_xy

    backoff_cm = standoff.backoff_cm(
        PERCEPTION_STANDOFF_CM,
        max_cm=STANDOFF_BACKOFF_MAX_CM,
    )
    print(
        f"  [SiteNav] standoff: {standoff.nearest_dist_cm:.0f}cm "
        f"< {PERCEPTION_STANDOFF_CM:.0f}cm ({standoff.source}) "
        f"→ backoff {backoff_cm:.0f}cm before perceive"
    )
    standoff_t0 = time.perf_counter()
    backoff_t0 = time.perf_counter()
    pos_xy = _standoff_backoff_move(
        ucv,
        robot_name,
        pos_xy,
        standoff.obstacle_xy,  # type: ignore[arg-type]
        backoff_cm,
    )
    _invalidate_pose_after_standoff_motion(pose_cache)
    pos_xy, robot_yaw_deg, ucv = _fetch_nav_pose(
        ucv, robot_name, nav_timing, pose_cache, force=True
    )
    if nav_timing is not None:
        nav_timing.backoff_ms += (time.perf_counter() - backoff_t0) * 1000.0
    if carry_motion_cb is not None:
        carry_motion_cb()
    if depth_invalidate_fn is not None:
        depth_invalidate_fn("perceive_standoff_backoff")
    if nav_timing is not None:
        nav_timing.standoff_ms += (time.perf_counter() - standoff_t0) * 1000.0
        nav_timing.standoff_events += 1

    if depth_refresh_fn is not None:
        forward_depth_cm = depth_refresh_fn()

    recheck = check_perception_standoff(
        pos_xy,
        layers,
        registry_positions=registry_positions,
        standoff_cm=PERCEPTION_STANDOFF_CM,
        forward_depth_cm=forward_depth_cm,
    )
    if recheck.needs_backoff(PERCEPTION_STANDOFF_CM):
        print(
            f"  [SiteNav] standoff: still {recheck.nearest_dist_cm:.0f}cm after backoff; "
            f"defer perceive"
        )
        return False, pos_xy
    return True, pos_xy

def _normalize_angle(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def _find_escape_step_candidates(
    layers: LayeredCostmap,
    pos_xy: WorldXY,
    goal_xy: WorldXY,
    yaw_deg: float,
    *,
    stuck_hotspots: Sequence[WorldXY] = (),
) -> List[WorldXY]:
    """Rank short traversable escape targets; prefer away from prior stuck hotspots."""
    costmap = _planning_costmap(layers)
    start = costmap.world_xy_to_grid(pos_xy, clamp=True)
    if start is None:
        return []
    sx, sy = start
    max_r = max(1, int(ESCAPE_STEP_MAX_CM / costmap.resolution_cm))
    min_dist = ESCAPE_STEP_MIN_CM
    scored: List[Tuple[float, WorldXY]] = []
    for dy in range(-max_r, max_r + 1):
        for dx in range(-max_r, max_r + 1):
            gx, gy = sx + dx, sy + dy
            if gx < 0 or gy < 0 or gx >= costmap.width_cells or gy >= costmap.height_cells:
                continue
            if not costmap.is_traversable((gx, gy)):
                continue
            candidate = costmap.grid_to_world_xy_center((gx, gy))
            if not world_segment_is_traversable(
                costmap, pos_xy, candidate, skip_start_cell=True
            ):
                continue
            step_dist = dist2d(candidate, pos_xy)
            if step_dist < min_dist or step_dist > ESCAPE_STEP_MAX_CM:
                continue
            turn = abs(_normalize_angle(yaw_to_target(pos_xy, candidate) - yaw_deg))
            if turn > ESCAPE_MAX_TURN_DEG:
                continue
            hotspot_clear = 0.0
            if stuck_hotspots:
                hotspot_clear = min(dist2d(candidate, hot) for hot in stuck_hotspots)
            score = (
                dist2d(candidate, goal_xy)
                + turn * 1.5
                - step_dist * 0.15
                - hotspot_clear * 0.35
            )
            scored.append((score, candidate))
    scored.sort(key=lambda item: item[0])
    out: List[WorldXY] = []
    seen: Set[Tuple[int, int]] = set()
    for _, candidate in scored:
        key = (int(round(candidate[0])), int(round(candidate[1])))
        if key in seen:
            continue
        seen.add(key)
        out.append(candidate)
        if len(out) >= ESCAPE_CANDIDATE_LIMIT:
            break
    return out


def _find_escape_step_world_xy(
    layers: LayeredCostmap,
    pos_xy: WorldXY,
    goal_xy: WorldXY,
    yaw_deg: float,
    *,
    stuck_hotspots: Sequence[WorldXY] = (),
) -> Optional[WorldXY]:
    candidates = _find_escape_step_candidates(
        layers, pos_xy, goal_xy, yaw_deg, stuck_hotspots=stuck_hotspots
    )
    return candidates[0] if candidates else None


def _move_toward_escape_target(
    ucv: UnrealCV,
    robot_name: str,
    escape_xy: WorldXY,
    *,
    on_after_motion: Optional[CarrySyncFn] = None,
    max_iters: int = 3,
) -> WorldXY:
    for _ in range(max_iters):
        cur_xy, _ = _safe_get_pos2d(ucv, robot_name)
        if dist2d(cur_xy, escape_xy) <= PATH_WP_REACH_TOLERANCE_CM:
            return cur_xy
        command = _smooth_segment_command(
            cur_xy, get_yaw(ucv, robot_name), escape_xy
        )
        if command is None:
            return cur_xy
        _execute_segment_command(
            ucv,
            command,
            robot_name,
            on_after_motion=on_after_motion,
        )
        tick_settle(ucv, settle_s=POST_MOTION_SETTLE_S, ticks=1)
    cur_xy, _ = _safe_get_pos2d(ucv, robot_name)
    return cur_xy


def _lateral_unstuck_rotate_backup(
    ucv: UnrealCV,
    robot_name: str,
    *,
    on_after_motion: Optional[CarrySyncFn] = None,
) -> None:
    """Rotate ~90° then back up when forward escape cannot displace the robot."""
    for turn_deg, clockwise in ((90.0, -1), (90.0, 1)):
        _dog_rotate_chunked(
            ucv,
            robot_name,
            turn_deg,
            clockwise,
            on_after_motion=on_after_motion,
        )
        _unstuck_backup(ucv, robot_name)
        if on_after_motion is not None:
            on_after_motion()


def _execute_escape_step(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    robot_name: str,
    pos_xy: WorldXY,
    goal_xy: WorldXY,
    *,
    stuck_hotspots: Sequence[WorldXY] = (),
    on_after_motion: Optional[CarrySyncFn] = None,
) -> Optional[WorldXY]:
    try:
        yaw_deg = get_yaw(ucv, robot_name)
    except (ConnectionError, OSError, ValueError, RuntimeError):
        return None
    candidates = _find_escape_step_candidates(
        layers, pos_xy, goal_xy, yaw_deg, stuck_hotspots=stuck_hotspots
    )
    for escape_xy in candidates:
        print(
            f"  [SiteNav] escape try → local={world_xy_to_local(*escape_xy)} "
            f"dist={dist2d(pos_xy, escape_xy):.0f}cm"
        )
        after_xy = _move_toward_escape_target(
            ucv,
            robot_name,
            escape_xy,
            on_after_motion=on_after_motion,
        )
        if dist2d(after_xy, pos_xy) >= ESCAPE_MIN_DISPLACEMENT_CM:
            print(
                f"  [SiteNav] escape OK actual local={world_xy_to_local(*after_xy)} "
                f"moved={dist2d(after_xy, pos_xy):.0f}cm"
            )
            return after_xy
    print("  [SiteNav] escape candidates failed — lateral rotate+backup")
    _lateral_unstuck_rotate_backup(ucv, robot_name, on_after_motion=on_after_motion)
    after_xy, _ = _safe_get_pos2d(ucv, robot_name)
    if dist2d(after_xy, pos_xy) >= ESCAPE_MIN_DISPLACEMENT_CM:
        print(
            f"  [SiteNav] lateral escape OK local={world_xy_to_local(*after_xy)} "
            f"moved={dist2d(after_xy, pos_xy):.0f}cm"
        )
        return after_xy
    print(
        f"  [SiteNav] escape still blocked local={world_xy_to_local(*after_xy)} "
        f"moved={dist2d(after_xy, pos_xy):.0f}cm"
    )
    return after_xy


def _dog_rotate_chunked(
    ucv: UnrealCV,
    robot_name: str,
    turn_deg: float,
    clockwise: int,
    *,
    diag: bool = False,
    on_after_motion: Optional[CarrySyncFn] = None,
) -> None:
    """Split large rotations — single large dog_rotate crashes UE on Level PIE."""
    remaining = float(turn_deg)
    min_step_deg = 1.0
    while remaining > min_step_deg:
        step_deg = min(remaining, MAX_TURN_DEG_PER_STEP)
        if diag:
            print(
                f"  [SiteNav] UE-RISK dog_rotate {step_deg:.1f}° "
                f"cw={clockwise} (chunk of {turn_deg:.1f}°)"
            )
        turn_duration_s = max(0.25, ROBOT_TURN_DUR_S * step_deg / 90.0)
        _site_dog_rotate(ucv, robot_name, turn_duration_s, step_deg, clockwise)
        if on_after_motion is not None:
            on_after_motion()
        remaining -= step_deg
    if remaining > 1e-3:
        step_deg = remaining
        if diag:
            print(
                f"  [SiteNav] UE-RISK dog_rotate {step_deg:.1f}° "
                f"cw={clockwise} (remainder of {turn_deg:.1f}°)"
            )
        turn_duration_s = max(0.25, ROBOT_TURN_DUR_S * step_deg / 90.0)
        _site_dog_rotate(ucv, robot_name, turn_duration_s, step_deg, clockwise)
        if on_after_motion is not None:
            on_after_motion()


def _prime_first_motion(ucv: UnrealCV, robot_name: str) -> None:
    """Short forward move before first turn — stabilizes SpotDog physics after spawn."""
    duration_s = max(SITE_MOVE_SLICE_S, FIRST_MOVE_PRIME_CM / SITE_ROBOT_SPEED)
    print(f"  [SiteNav] prime dog_move {FIRST_MOVE_PRIME_CM:.0f}cm before first turn")
    _site_dog_move(ucv, robot_name, SITE_ROBOT_SPEED * 0.6, duration_s, direction=0)


def _execute_segment_command(
    ucv: UnrealCV,
    command: SegmentCommand,
    robot_name: str,
    *,
    diag: bool = False,
    on_after_motion: Optional[CarrySyncFn] = None,
    nav_timing: Optional[NavTimingAccumulator] = None,
    pose_cache: Optional[PoseCache] = None,
) -> None:
    """Site-tuned open-loop move (slower speed; uses actual robot_name)."""
    did_motion = False
    if command.turn_deg > 1.0:
        rot_t0 = time.perf_counter()
        _dog_rotate_chunked(
            ucv,
            robot_name,
            command.turn_deg,
            command.turn_clockwise,
            diag=diag,
            on_after_motion=on_after_motion,
        )
        if nav_timing is not None:
            nav_timing.rotate_ms += (time.perf_counter() - rot_t0) * 1000.0
        did_motion = True
    if command.move_cm > 1e-3:
        if diag:
            print(f"  [SiteNav] UE-RISK dog_move {command.move_cm:.1f}cm")
        move_duration_s = max(SITE_MOVE_SLICE_S, command.move_cm / SITE_ROBOT_SPEED)
        trans_t0 = time.perf_counter()
        _site_dog_move(ucv, robot_name, SITE_ROBOT_SPEED, move_duration_s, direction=0)
        if nav_timing is not None:
            nav_timing.translate_ms += (time.perf_counter() - trans_t0) * 1000.0
        if on_after_motion is not None:
            on_after_motion()
        did_motion = True
    if did_motion:
        invalidate_robot_pose(pose_cache, reason="segment_motion")


def _execute_rpp_closed_loop(
    ucv: UnrealCV,
    robot_name: str,
    pos_xy: WorldXY,
    yaw_deg: float,
    waypoint_xy: WorldXY,
    waypoints: Sequence[WorldXY],
    wp_index: int,
    *,
    max_move_cm: float,
    diag: bool = False,
    on_after_motion: Optional[CarrySyncFn] = None,
    nav_timing: Optional[NavTimingAccumulator] = None,
    on_move_cm_fn: Optional[Callable[[float], None]] = None,
    pose_cache: Optional[PoseCache] = None,
) -> Tuple[WorldXY, float, UnrealCV, bool]:
    """RPP closed-loop chunks toward ``waypoint_xy``; returns (pos, yaw, ucv, stalled)."""
    server = _get_controller_server()
    state = {"ucv": ucv, "xy": pos_xy, "yaw": yaw_deg}

    def _get_pose() -> Tuple[WorldXY, float]:
        xy, yaw, ucv_out = _fetch_nav_pose(
            state["ucv"],
            robot_name,
            nav_timing,
            pose_cache,
            force=False,
        )
        state["ucv"] = ucv_out
        state["xy"] = xy
        state["yaw"] = yaw
        return xy, yaw

    def _compute_command(current_xy: WorldXY, current_yaw: float) -> Optional[SegmentCommand]:
        return server.compute_segment_command(
            current_xy,
            current_yaw,
            waypoint_xy,
            waypoints,
            wp_index,
            legacy_command_fn=_smooth_segment_command,
        )

    def _execute_chunk(command: SegmentCommand) -> None:
        _execute_segment_command(
            state["ucv"],
            command,
            robot_name,
            diag=diag,
            on_after_motion=on_after_motion,
            nav_timing=nav_timing,
            pose_cache=pose_cache,
        )
        if command.move_cm > 1e-3 and on_move_cm_fn is not None:
            on_move_cm_fn(command.move_cm)

    result = server.execute_closed_loop(
        target_xy=waypoint_xy,
        get_pose=_get_pose,
        compute_command=_compute_command,
        execute_chunk=_execute_chunk,
    )
    return result.final_pos, result.final_yaw_deg, state["ucv"], result.stalled


def _timed_get_pos2d(
    ucv: UnrealCV,
    robot_name: str,
    nav_timing: Optional[NavTimingAccumulator],
) -> Tuple[WorldXY, UnrealCV]:
    t0 = time.perf_counter()
    pos_xy, ucv_out = _safe_get_pos2d(ucv, robot_name)
    if nav_timing is not None:
        nav_timing.pose_query_ms += (time.perf_counter() - t0) * 1000.0
    return pos_xy, ucv_out


def _timed_get_yaw(
    ucv: UnrealCV,
    robot_name: str,
    nav_timing: Optional[NavTimingAccumulator],
) -> float:
    t0 = time.perf_counter()
    yaw_deg = get_yaw(ucv, robot_name)
    if nav_timing is not None:
        nav_timing.pose_query_ms += (time.perf_counter() - t0) * 1000.0
    return yaw_deg


def _safe_get_pos2d(ucv, robot_name: str):
    try:
        return get_pos2d(ucv, robot_name), ucv
    except (ConnectionError, AttributeError, OSError, ValueError, TypeError) as exc:
        print(f"  [SiteNav] get_pos2d failed ({exc}) — reconnect")
        try:
            ucv = ensure_live_or_reconnect(ucv, reason="get_pos2d")
            return get_pos2d(ucv, robot_name), ucv
        except (ConnectionError, OSError, RuntimeError, PieSessionLost) as exc2:
            raise PieSessionLost(f"UnrealCV reconnect failed during nav: {exc2}") from exc2


@dataclass
class _StuckRecoveryOutcome:
    ucv: UnrealCV
    pos_xy: WorldXY
    waypoints: list
    wp_index: int
    steps_on_wp: int
    unstuck_attempts: int
    stuck_hotspots: List[WorldXY]
    mission_failed: bool


def _stuck_recovery_callbacks(
    *,
    soft_reset_fn: Optional[Callable[..., None]] = None,
) -> "StuckRecoveryCallbacks":
    from nav_stack.stuck_recovery import StuckRecoveryCallbacks  # noqa: WPS433

    def _mark(session: "StuckRecoverySession") -> Tuple[int, int, int]:
        mark_radius = min(4, 1 + (session.unstuck_attempts + 1) // 2)
        n_marked = _mark_stuck_cells_on_l2(
            session.layers,
            session.stuck_xy,
            session.l2_seen_cells,
            radius_cells=mark_radius,
        )
        n_hotspot = _mark_stuck_hotspot_on_l2(
            session.layers, session.stuck_xy, session.l2_seen_cells
        )
        block_xy = (
            session.waypoint_xy
            if session.wp_index < len(session.waypoints)
            else session.goal_xy
        )
        n_corridor = _mark_stuck_corridor_on_l2(
            session.layers, session.stuck_xy, block_xy, session.l2_seen_cells
        )
        return n_marked, n_hotspot, n_corridor

    def _record_plan(session: "StuckRecoverySession", waypoints: list, reason: str) -> None:
        if session.trace is not None:
            session.trace.record_plan(waypoints, reason=reason)
            session.trace.l2_cell_count = len(session.l2_seen_cells)

    def _clear_local(session: "StuckRecoverySession") -> int:
        before = _l2_occupied_count(session.layers)
        if soft_reset_fn is not None:
            soft_reset_fn(session.l2_seen_cells, session.stuck_xy, aggressive=False)
        after = _l2_occupied_count(session.layers)
        return max(0, before - after)

    return StuckRecoveryCallbacks(
        mark_stuck_cells=_mark,
        unstuck_backup=lambda s: _unstuck_backup(s.ucv, s.robot_name, backup_cm=s.backup_cm),
        safe_get_pos2d=lambda s: _safe_get_pos2d(s.ucv, s.robot_name),
        execute_escape_step=lambda s, backup_xy: _execute_escape_step(
            s.ucv,
            s.layers,
            s.robot_name,
            backup_xy,
            s.goal_xy,
            stuck_hotspots=s.stuck_hotspots,
            on_after_motion=s.carry_motion_cb,
        ),
        world_to_local=lambda xy: world_xy_to_local(*xy),
        record_plan=_record_plan,
        spin_backup=lambda s: _lateral_unstuck_rotate_backup(
            s.ucv, s.robot_name, on_after_motion=s.carry_motion_cb
        ),
        clear_local_l2=_clear_local if soft_reset_fn is not None else None,
        wait_settle=lambda s: _nav_settle(s.ucv, settle_s=0.5, ticks=2, nav_timing=None),
    )


def _run_last_resort_recovery(
    *,
    layers: LayeredCostmap,
    pos_xy: WorldXY,
    goal_xy: WorldXY,
    l2_seen_cells: Set[Tuple[int, int]],
    stuck_xy: WorldXY,
    soft_reset_fn: Optional[Callable[..., None]],
    nav_ctx: Optional[Any],
) -> Optional[Tuple[list, int, int]]:
    from nav_stack.last_resort_recovery import try_last_resort_recovery  # noqa: WPS433

    flush_count = nav_ctx.l2_flush_count if nav_ctx is not None else 0
    outcome = try_last_resort_recovery(
        layers=layers,
        pos_xy=pos_xy,
        goal_xy=goal_xy,
        l2_seen_cells=l2_seen_cells,
        l2_flush_count=flush_count,
        stuck_xy=stuck_xy,
        soft_reset_fn=soft_reset_fn,
        replan_fn=lambda start_xy, end_xy: _safe_replan_astar(
            layers.to_costmap2d(), start_xy, end_xy
        ),
        nearest_wp_index_fn=_nearest_waypoint_index_ahead,
        max_flush_count=MAX_L2_FLUSH_COUNT,
        perceive_pause_steps=LAST_RESORT_PERCEIVE_PAUSE_STEPS,
    )
    if outcome is None:
        return None
    if nav_ctx is not None:
        nav_ctx.l2_flush_count = outcome.l2_flush_count
    if not outcome.success:
        return None
    return outcome.waypoints, outcome.wp_index, outcome.perceive_pause_steps


def _apply_stuck_recovery(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    robot_name: str,
    goal_xy: WorldXY,
    *,
    stuck_xy: WorldXY,
    waypoints: list,
    wp_index: int,
    waypoint_xy: WorldXY,
    stuck_hotspots: List[WorldXY],
    unstuck_attempts: int,
    l2_seen_cells: Set[Tuple[int, int]],
    trace: Optional[NavTrace],
    carry_motion_cb: Optional[CarrySyncFn],
    soft_reset_fn: Optional[Callable[..., None]] = None,
    nav_kpi: Optional[Any] = None,
) -> _StuckRecoveryOutcome:
    from nav_stack.stuck_recovery import (  # noqa: WPS433
        StuckRecoverySession,
        run_site_stuck_recovery,
    )

    if nav_kpi is not None:
        nav_kpi.record_stuck()

    session = StuckRecoverySession(
        ucv=ucv,
        layers=layers,
        robot_name=robot_name,
        goal_xy=goal_xy,
        stuck_xy=stuck_xy,
        waypoints=waypoints,
        wp_index=wp_index,
        waypoint_xy=waypoint_xy,
        stuck_hotspots=list(stuck_hotspots),
        unstuck_attempts=unstuck_attempts,
        l2_seen_cells=l2_seen_cells,
        trace=trace,
        carry_motion_cb=carry_motion_cb,
        planning_clearance_cm=SITE_PLANNING_CLEARANCE_CM,
        planning_clearance_cost=SITE_PLANNING_CLEARANCE_COST,
        backup_cm=UNSTUCK_BACKUP_CM,
        stuck_move_threshold_cm=STUCK_MOVE_THRESHOLD_CM,
        escape_min_displacement_cm=ESCAPE_MIN_DISPLACEMENT_CM,
        max_unstuck_attempts=MAX_UNSTUCK_ATTEMPTS,
        nearest_wp_index_fn=_nearest_waypoint_index_ahead,
    )
    outcome = run_site_stuck_recovery(
        session,
        callbacks=_stuck_recovery_callbacks(soft_reset_fn=soft_reset_fn),
    )
    return _StuckRecoveryOutcome(
        ucv=outcome.ucv,
        pos_xy=outcome.pos_xy,
        waypoints=outcome.waypoints,
        wp_index=outcome.wp_index,
        steps_on_wp=outcome.steps_on_wp,
        unstuck_attempts=outcome.unstuck_attempts,
        stuck_hotspots=outcome.stuck_hotspots,
        mission_failed=outcome.mission_failed,
    )


def navigate_layered_with_fusion(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    goal_local_xy: Tuple[float, float],
    *,
    perceive_fn: PerceiveFn,
    soft_reset_fn: Optional[Callable[..., None]] = None,
    robot_name: str = ROBOT_ACTOR,
    nav_actor: Optional[str] = None,
    tolerance_cm: float = 120.0,
    label: str = "",
    perception_interval_s: float = SITE_DEFAULT_PERCEPTION_INTERVAL_S,
    max_total_steps: int = PATH_MAX_TOTAL_STEPS,
    trace: Optional[NavTrace] = None,
    carry_sync_name: Optional[str] = None,
    on_pose_sample: Optional[PoseSampleFn] = None,
    nav_timing: Optional[NavTimingAccumulator] = None,
    extra_obstacle_positions_fn: Optional[Callable[[], Sequence[WorldXY]]] = None,
    forward_depth_cm_fn: Optional[Callable[[], Optional[float]]] = None,
    depth_refresh_fn: Optional[Callable[[], Optional[float]]] = None,
    depth_invalidate_fn: Optional[Callable[[str], None]] = None,
    on_move_cm_fn: Optional[Callable[[float], None]] = None,
    depth_prefetch_fn: Optional[Callable[..., None]] = None,
    on_nav_iter_start_fn: Optional[Callable[[], None]] = None,
    pose_cache: Optional[PoseCache] = None,
    nav_ctx: Optional[Any] = None,
    perceive_depth_refresh_fn: Optional[Callable[[], Optional[float]]] = None,
) -> bool:
    require_live_ucv(ucv, context="site transport fusion nav")
    stack_cfg = _stack_config_from_ctx(nav_ctx)
    nav_kpi = _kpi_from_ctx(nav_ctx)
    goal_xy = local_xy_to_world(*goal_local_xy)
    l2_seen_cells: Set[Tuple[int, int]] = set()
    _sync_seen_cells_from_l2(layers, l2_seen_cells)
    use_carry_sync = bool(carry_sync_name) and not is_carry_ue_attached()

    def _sync_carry() -> None:
        if not use_carry_sync or not carry_sync_name:
            return
        if geh.actor_exists(ucv, carry_sync_name):
            sync_carry_pose(ucv, carry_sync_name, robot_name, refresh_collision=False)

    carry_motion_cb: Optional[CarrySyncFn] = _sync_carry if use_carry_sync else None

    start_xy, ucv = _safe_get_pos2d(ucv, robot_name)
    plan_t0 = time.perf_counter()
    plan = _safe_replan_astar(_planning_costmap(layers), start_xy, goal_xy)
    if nav_timing is not None:
        nav_timing.replan_ms += (time.perf_counter() - plan_t0) * 1000.0
    waypoints = plan.waypoints_xy
    if trace is not None:
        trace.record_plan(waypoints, reason="initial")
        trace.record_position(world_xy_to_local(*start_xy))
    if on_pose_sample is not None:
        on_pose_sample(start_xy, time.time())
    wp_index = 0
    steps_on_wp = 0
    total_steps = 0
    last_perception_t = -1e9
    last_progress_xy = start_xy
    moves_since_progress = 0
    unstuck_attempts = 0
    stuck_hotspots: List[WorldXY] = []
    best_dist_goal = dist2d(start_xy, goal_xy)
    turn_only_steps = 0
    nav_t0 = time.time()
    moves_executed = 0
    last_motion_t = nav_t0
    _l2_flush_count = nav_ctx.l2_flush_count if nav_ctx is not None else 0
    _l2_perceive_pause_steps = 0
    last_l2_replan_xy = start_xy
    last_l2_replan_dist_goal = best_dist_goal
    l2_replan_thrash_count = 0
    goal_stall_anchor_dist = best_dist_goal
    goal_stall_steps = 0

    def _l2_replan_thrash_blocked(pos: WorldXY, dist_goal_cm: float) -> bool:
        nonlocal l2_replan_thrash_count, last_l2_replan_xy, last_l2_replan_dist_goal
        pos_moved = dist2d(pos, last_l2_replan_xy) >= L2_REPLAN_THRASH_POS_CM
        goal_moved = abs(dist_goal_cm - last_l2_replan_dist_goal) >= L2_REPLAN_THRASH_GOAL_CM
        if pos_moved or goal_moved:
            l2_replan_thrash_count = 0
            last_l2_replan_xy = pos
            last_l2_replan_dist_goal = dist_goal_cm
            return False
        l2_replan_thrash_count += 1
        if l2_replan_thrash_count >= L2_REPLAN_THRASH_MAX:
            print(
                "  [SiteNav] L2 replan thrash suppressed "
                f"({l2_replan_thrash_count} cycles, dist_goal={dist_goal_cm:.0f}cm)"
            )
            return True
        return False

    def _note_l2_replan(pos: WorldXY, dist_goal_cm: float) -> None:
        nonlocal l2_replan_thrash_count, last_l2_replan_xy, last_l2_replan_dist_goal
        l2_replan_thrash_count = 0
        last_l2_replan_xy = pos
        last_l2_replan_dist_goal = dist_goal_cm

    print(
        f"  [SiteNav]{f' {label}' if label else ''} goal_local={goal_local_xy} "
        f"waypoints={len(waypoints)} cost={plan.total_cost:.1f}"
    )
    print(f"  [SiteNav] warmup {NAV_WARMUP_SETTLE_S:.1f}s before first dog_move")
    _nav_settle(ucv, settle_s=NAV_WARMUP_SETTLE_S, ticks=3, nav_timing=nav_timing)
    if nav_actor and is_robot_tipped(ucv, robot_name):
        pos_xy, ucv = _safe_get_pos2d(ucv, robot_name)
        recover_robot_upright(ucv, robot_name, pos_xy, nav_actor=nav_actor)
        _nav_settle(ucv, settle_s=1.0, ticks=2, nav_timing=nav_timing)
    if nav_timing is not None:
        prime_t0 = time.perf_counter()
        _prime_first_motion(ucv, robot_name)
        nav_timing.move_ms += (time.perf_counter() - prime_t0) * 1000.0
    else:
        _prime_first_motion(ucv, robot_name)
    _nav_settle(ucv, settle_s=1.0, ticks=2, nav_timing=nav_timing)
    if use_carry_sync:
        _sync_carry()

    def _run_perceive_cycle(now: float) -> None:
        nonlocal pos_xy, waypoints, wp_index, last_perception_t
        perceive_t0 = time.perf_counter()
        l2_count_before = _l2_occupied_count(layers)
        l2_snapshot = layers.l2.copy()
        l2_seen_snapshot = set(l2_seen_cells)
        dist_goal_now = dist2d(pos_xy, goal_xy)
        try:
            outcome = _invoke_perceive(
                perceive_fn, layers=layers, l2_seen_cells=l2_seen_cells
            )
        except Exception as exc:
            print(f"  [SiteNav] perceive error: {exc}")
            outcome = PerceiveOutcome(detections=[], l2_applied=False)
        detections = outcome.detections
        l2_count_after = _l2_occupied_count(layers)

        def _rollback_l2_snapshot() -> None:
            layers.l2[:, :] = l2_snapshot
            l2_seen_cells.clear()
            l2_seen_cells.update(l2_seen_snapshot)

        if outcome.l2_applied:
            if outcome.l2_changed and _l2_cell_delta_warrants_replan(
                outcome.cells_added, outcome.cells_removed
            ):
                if _l2_replan_thrash_blocked(pos_xy, dist_goal_now):
                    _rollback_l2_snapshot()
                else:
                    summary = detections_summary(detections) if detections else {}
                    new_wps = _timed_replan_on_merged_layers(
                        layers,
                        pos_xy,
                        goal_xy,
                        reason="l2_depth",
                        trace=trace,
                        l2_seen_cells=l2_seen_cells,
                        nav_timing=nav_timing,
                        nav_kpi=nav_kpi,
                    )
                    if new_wps is not None:
                        waypoints = new_wps
                        wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                        _note_l2_replan(pos_xy, dist_goal_now)
                        print(
                            f"  [SiteNav] L2 sight +{outcome.cells_added}/-{outcome.cells_removed} "
                            f"cells detect={list(summary.keys())} "
                            f"→ replan {len(waypoints)} WP (merged L0+L1+L2)"
                        )
                    else:
                        _rollback_l2_snapshot()
                        print(
                            f"  [SiteNav] L2 sight +{outcome.cells_added}/-{outcome.cells_removed} "
                            f"cells → replan failed; rolled back latest L2 update"
                        )
            elif outcome.l2_changed:
                print(
                    f"  [SiteNav] L2 sight delta +{outcome.cells_added}/-{outcome.cells_removed} "
                    f"below replan threshold ({L2_REPLAN_CELL_DELTA_THRESHOLD}); skip replan"
                )
            else:
                print(
                    f"  [SiteNav] L2 sight: no map change "
                    f"(active={len(detections)}, l2_cells={len(l2_seen_cells)})"
                )
        elif detections:
            try:
                robot_yaw_deg = get_yaw(ucv, robot_name)
            except (ConnectionError, OSError, ValueError, RuntimeError) as exc:
                print(f"  [SiteNav] L2 apply skipped (get_yaw): {exc}")
                detections = []
            if detections:
                n_cells = apply_l2_from_fusion_detections(
                    layers,
                    detections,
                    robot_xy=pos_xy,
                    robot_yaw_deg=robot_yaw_deg,
                    known_cells=l2_seen_cells,
                )
                summary = detections_summary(detections)
                if n_cells > 0 and _l2_cell_delta_warrants_replan(n_cells):
                    if _l2_replan_thrash_blocked(pos_xy, dist_goal_now):
                        _rollback_l2_snapshot()
                    else:
                        new_wps = _timed_replan_on_merged_layers(
                            layers,
                            pos_xy,
                            goal_xy,
                            reason="l2_perception",
                            trace=trace,
                            l2_seen_cells=l2_seen_cells,
                            nav_timing=nav_timing,
                            nav_kpi=nav_kpi,
                        )
                        if new_wps is not None:
                            waypoints = new_wps
                            wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                            _note_l2_replan(pos_xy, dist_goal_now)
                            print(
                                f"  [SiteNav] L2 +{n_cells} cells detect={list(summary.keys())} "
                                f"→ replan {len(waypoints)} WP (merged L0+L1+L2)"
                            )
                        else:
                            print(
                                f"  [SiteNav] L2 +{n_cells} cells detect={list(summary.keys())} "
                                f"→ replan failed (merged map)"
                            )
                elif n_cells > 0:
                    print(
                        f"  [SiteNav] L2 +{n_cells} cells below replan threshold "
                        f"({L2_REPLAN_CELL_DELTA_THRESHOLD}); skip replan"
                    )
                else:
                    print(
                        f"  [SiteNav] L2 detect={list(summary.keys())} "
                        f"(no new cells, l2_cells={len(l2_seen_cells)})"
                    )
        elif l2_count_after > l2_count_before:
            depth_delta = l2_count_after - l2_count_before
            _sync_seen_cells_from_l2(layers, l2_seen_cells)
            if _l2_cell_delta_warrants_replan(depth_delta):
                if _l2_replan_thrash_blocked(pos_xy, dist_goal_now):
                    _rollback_l2_snapshot()
                else:
                    new_wps = _timed_replan_on_merged_layers(
                        layers,
                        pos_xy,
                        goal_xy,
                        reason="l2_depth",
                        trace=trace,
                        l2_seen_cells=l2_seen_cells,
                        nav_timing=nav_timing,
                        nav_kpi=nav_kpi,
                    )
                    if new_wps is not None:
                        waypoints = new_wps
                        wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                        _note_l2_replan(pos_xy, dist_goal_now)
                        print(
                            f"  [SiteNav] L2 depth +{depth_delta} cells "
                            f"→ replan {len(waypoints)} WP (merged L0+L1+L2)"
                        )
                    else:
                        _rollback_l2_snapshot()
                        print(
                            f"  [SiteNav] L2 depth +{depth_delta} cells "
                            f"→ replan failed; rolled back latest L2 update"
                        )
            else:
                print(
                    f"  [SiteNav] L2 depth +{depth_delta} cells below replan threshold "
                    f"({L2_REPLAN_CELL_DELTA_THRESHOLD}); skip replan"
                )
        else:
            print(f"  [SiteNav] L2 perceive: 0 detections (l2_cells={len(l2_seen_cells)})")
        if nav_timing is not None:
            nav_timing.perceive_ms += (time.perf_counter() - perceive_t0) * 1000.0
        last_perception_t = now


    def _finalize_nav_timing() -> None:
        if nav_timing is None:
            return
        wall_ms = (time.time() - nav_t0) * 1000.0
        gap = wall_ms - nav_timing.accounted_ms()
        if gap > 0.0:
            nav_timing.loop_residual_ms += gap
            nav_timing.loop_overhead_ms += gap

    pos_xy = start_xy
    pos_xy, robot_yaw_deg, ucv = _fetch_nav_pose(
        ucv, robot_name, nav_timing, pose_cache, force=True
    )

    while total_steps < max_total_steps:
        loop_t0 = time.perf_counter()
        accounted_at_start = nav_timing.accounted_ms() if nav_timing is not None else 0.0
        loop_bucket_at_start = (
            _loop_bucket_total_ms(nav_timing) if nav_timing is not None else 0.0
        )
        depth_fresh_this_iter = False

        def _refresh_depth_once() -> Optional[float]:
            nonlocal depth_fresh_this_iter
            if depth_refresh_fn is None:
                return forward_depth_cm_fn() if forward_depth_cm_fn is not None else None
            if depth_fresh_this_iter:
                return forward_depth_cm_fn() if forward_depth_cm_fn is not None else None
            depth_fresh_this_iter = True
            return depth_refresh_fn()

        require_live_ucv(ucv, context=f"site nav step {total_steps}")
        if on_nav_iter_start_fn is not None:
            on_nav_iter_start_fn()
        pos_xy, robot_yaw_deg, ucv = _fetch_nav_pose(
            ucv, robot_name, nav_timing, pose_cache
        )
        if trace is not None:
            trace.record_position(world_xy_to_local(*pos_xy))
        if on_pose_sample is not None:
            on_pose_sample(pos_xy, time.time())
        if nav_kpi is not None and waypoints:
            from nav_stack.nav_kpi import cross_track_error_cm  # noqa: WPS433

            seg_start = waypoints[wp_index - 1] if wp_index > 0 else pos_xy
            seg_end = waypoints[wp_index] if wp_index < len(waypoints) else goal_xy
            nav_kpi.record_cross_track(
                cross_track_error_cm(pos_xy, seg_start, seg_end)
            )
        dist_goal = dist2d(pos_xy, goal_xy)
        if dist_goal < goal_stall_anchor_dist - GOAL_DIST_STALL_IMPROVE_CM:
            goal_stall_anchor_dist = dist_goal
            goal_stall_steps = 0
        else:
            goal_stall_steps += 1
        if goal_stall_steps >= GOAL_DIST_STALL_STEPS and dist_goal > tolerance_cm:
            print(
                f"  [SiteNav] goal-distance stall ({goal_stall_steps} steps) "
                f"dist={dist_goal:.0f}cm → stuck recovery"
            )
            recovery = _apply_stuck_recovery(
                ucv,
                layers,
                robot_name,
                goal_xy,
                stuck_xy=pos_xy,
                waypoints=waypoints,
                wp_index=wp_index,
                waypoint_xy=waypoints[wp_index] if wp_index < len(waypoints) else goal_xy,
                stuck_hotspots=stuck_hotspots,
                unstuck_attempts=unstuck_attempts,
                l2_seen_cells=l2_seen_cells,
                trace=trace,
                carry_motion_cb=carry_motion_cb,
                soft_reset_fn=soft_reset_fn,
                nav_kpi=nav_kpi,
            )
            if recovery.mission_failed:
                last_resort = _run_last_resort_recovery(
                    layers=layers,
                    pos_xy=pos_xy,
                    goal_xy=goal_xy,
                    l2_seen_cells=l2_seen_cells,
                    stuck_xy=pos_xy,
                    soft_reset_fn=soft_reset_fn,
                    nav_ctx=nav_ctx,
                )
                if last_resort is None:
                    _finalize_nav_timing()
                    return False
                waypoints, wp_index, _l2_perceive_pause_steps = last_resort
                _l2_flush_count = (
                    nav_ctx.l2_flush_count if nav_ctx is not None else _l2_flush_count + 1
                )
            else:
                ucv = recovery.ucv
                waypoints = recovery.waypoints
                wp_index = recovery.wp_index
                steps_on_wp = recovery.steps_on_wp
                unstuck_attempts = recovery.unstuck_attempts
                stuck_hotspots = recovery.stuck_hotspots
                pos_xy = recovery.pos_xy
            goal_stall_steps = 0
            goal_stall_anchor_dist = dist_goal
            moves_since_progress = 0
            last_progress_xy = pos_xy
            last_motion_t = time.time()
            continue
        if dist_goal + 40.0 < best_dist_goal:
            best_dist_goal = dist_goal
            turn_only_steps = 0
        elif dist_goal > best_dist_goal + PROGRESS_REGRESS_THRESHOLD_CM:
            new_wps = _timed_replan_on_merged_layers(
                layers,
                pos_xy,
                goal_xy,
                reason="progress_regress",
                trace=trace,
                l2_seen_cells=l2_seen_cells,
                nav_timing=nav_timing,
                nav_kpi=nav_kpi,
            )
            if new_wps is not None:
                waypoints = new_wps
                wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                print(
                    f"  [SiteNav] progress regress dist={dist_goal:.0f}cm "
                    f"best={best_dist_goal:.0f}cm → replan {len(waypoints)} WP"
                )
                best_dist_goal = dist_goal
                turn_only_steps = 0
        if dist_goal <= tolerance_cm:
            print(f"  [SiteNav] Arrived dist={dist2d(pos_xy, goal_xy):.1f}cm")
            if trace is not None:
                trace.arrived = True
                trace.l2_cell_count = len(l2_seen_cells)
            _sync_carry()
            _finalize_nav_timing()
            if nav_timing is not None:
                print(
                    f"  [SiteNav] timing_ms perceive={nav_timing.perceive_ms:.0f} "
                    f"move={nav_timing.move_ms:.0f} replan={nav_timing.replan_ms:.0f} "
                    f"settle={nav_timing.settle_ms:.0f} total={nav_timing.total_ms():.0f}"
                )
            return True

        now = time.time()
        motion_quiet = now - last_motion_t >= MOTION_SETTLE_BEFORE_PERCEIVE_S
        if _l2_perceive_pause_steps > 0:
            _l2_perceive_pause_steps -= 1
        if (
            motion_quiet
            and now - last_perception_t >= perception_interval_s
            and now - nav_t0 >= PERCEPTION_START_DELAY_S
            and _l2_perceive_pause_steps <= 0
        ):
            registry_positions: Sequence[WorldXY] = ()
            if extra_obstacle_positions_fn is not None:
                registry_positions = extra_obstacle_positions_fn()
            perceive_depth_fn = perceive_depth_refresh_fn or depth_refresh_fn
            gate_forward_depth: Optional[float] = None
            if perceive_depth_fn is not None:
                gate_forward_depth = perceive_depth_fn()
                depth_fresh_this_iter = True
            run_perceive, pos_xy = _gate_perception_standoff(
                ucv,
                robot_name,
                pos_xy,
                layers,
                registry_positions=registry_positions,
                carry_motion_cb=carry_motion_cb,
                nav_timing=nav_timing,
                forward_depth_cm=gate_forward_depth,
                depth_refresh_fn=perceive_depth_fn,
                l2_seen_cells=l2_seen_cells,
                depth_invalidate_fn=depth_invalidate_fn,
                robot_yaw_deg=robot_yaw_deg,
                pose_cache=pose_cache,
            )
            if not run_perceive:
                last_perception_t = now
            else:
                if PERCEPTION_STANDOFF_CM > 0.0:
                    last_motion_t = time.time()
                    _nav_settle(
                        ucv,
                        settle_s=POST_MOTION_SETTLE_S,
                        ticks=1,
                        nav_timing=nav_timing,
                    )
                _run_perceive_cycle(now)
        for _ in range(MOVES_PER_CYCLE):
            if total_steps >= max_total_steps:
                break
            if dist2d(pos_xy, goal_xy) <= tolerance_cm:
                print(f"  [SiteNav] Arrived dist={dist2d(pos_xy, goal_xy):.1f}cm")
                if trace is not None:
                    trace.arrived = True
                    trace.l2_cell_count = len(l2_seen_cells)
                _sync_carry()
                _finalize_nav_timing()
                return True

            with _loop_bucket(nav_timing, "waypoint_select_ms"):
                if wp_index >= len(waypoints):
                    waypoint_xy = goal_xy
                else:
                    waypoint_xy = waypoints[wp_index]
                    if dist2d(pos_xy, waypoint_xy) <= PATH_WP_REACH_TOLERANCE_CM:
                        wp_index += 1
                        steps_on_wp = 0
                        continue

                command = _segment_command_for_waypoint(
                    pos_xy,
                    robot_yaw_deg,
                    waypoint_xy,
                    waypoints=waypoints,
                    wp_index=wp_index,
                )
                if command is None:
                    if wp_index >= len(waypoints):
                        if dist2d(pos_xy, goal_xy) <= tolerance_cm:
                            _finalize_nav_timing()
                            return True
                    else:
                        wp_index += 1
                    steps_on_wp = 0
                    continue

            if command.move_cm > 1e-3:
                registry_positions: Sequence[WorldXY] = ()
                if extra_obstacle_positions_fn is not None:
                    registry_positions = extra_obstacle_positions_fn()
                nearest_pre = _nearest_standoff_dist_cm(
                    pos_xy,
                    layers,
                    registry_positions,
                    nav_timing=nav_timing,
                )
                forward_depth_cm: Optional[float] = None
                if PERCEPTION_STANDOFF_CM > 0.0:
                    if depth_refresh_fn is not None:
                        forward_depth_cm = _refresh_depth_once()
                    elif forward_depth_cm_fn is not None:
                        forward_depth_cm = forward_depth_cm_fn()
                ok_move, pos_xy, forward_depth_cm, nearest_dist = _ensure_move_standoff(
                    ucv,
                    robot_name,
                    pos_xy,
                    layers,
                    registry_positions=registry_positions,
                    forward_depth_cm=forward_depth_cm,
                    carry_motion_cb=carry_motion_cb,
                    nav_timing=nav_timing,
                    depth_refresh_fn=_refresh_depth_once if depth_refresh_fn is not None else None,
                    l2_seen_cells=l2_seen_cells,
                    depth_invalidate_fn=depth_invalidate_fn,
                    robot_yaw_deg=robot_yaw_deg,
                    pose_cache=pose_cache,
                )
                if not ok_move:
                    spin_t0 = time.perf_counter()
                    last_motion_t = time.time()
                    _sync_pose_cache(pose_cache, pos_xy, robot_yaw_deg)
                    if nav_timing is not None:
                        nav_timing.move_gate_spin_ms += (time.perf_counter() - spin_t0) * 1000.0
                    continue
                with _loop_bucket(nav_timing, "nav_branch_ms"):
                    server = _get_controller_server()
                    allowed_move = server.allowed_move_cm(nearest_dist, forward_depth_cm)
                    if USE_RPP_CONTROLLER:
                        from nav_stack.local_costmap import build_local_costmap  # noqa: WPS433

                        local_map = build_local_costmap(
                            layers,
                            pos_xy,
                            size_cm=stack_cfg.local_costmap_size_cm,
                            resolution_cm=stack_cfg.local_costmap_resolution_cm,
                        )
                        if nav_kpi is not None:
                            nav_kpi.record_local_costmap_update()
                        local_nearest = local_map.nearest_lethal_dist_cm(pos_xy)
                        allowed_move = min(
                            allowed_move,
                            _dynamic_max_move_cm(local_nearest, forward_depth_cm),
                        )
                    if forward_depth_cm is not None:
                        depth_cap = max(
                            0.0,
                            forward_depth_cm - ROBOT_BODY_CLEARANCE_CM - 10.0,
                        )
                        allowed_move = min(allowed_move, depth_cap)
                    command = _clamp_segment_move(command, allowed_move)
                if command.move_cm <= 1e-3:
                    if nav_timing is not None:
                        nav_timing.move_gate_spin_ms += 50.0
                    total_steps += 1
                    continue
                with _loop_bucket(nav_timing, "replan_decision_ms"):
                    costmap = _timed_planning_costmap(layers, nav_timing)
                    move_target = _open_loop_move_target(pos_xy, waypoint_xy)
                    segment_blocked = not world_segment_is_traversable(
                        costmap, pos_xy, move_target, skip_start_cell=True
                    )
                if segment_blocked:
                    moves_since_progress += 1
                    total_steps += 1
                    new_wps = _timed_replan_on_merged_layers(
                        layers,
                        pos_xy,
                        goal_xy,
                        reason="move_segment_blocked",
                        trace=trace,
                        l2_seen_cells=l2_seen_cells,
                        nav_timing=nav_timing,
                        nav_kpi=nav_kpi,
                    )
                    if new_wps is not None:
                        waypoints = new_wps
                        wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, 0)
                    else:
                        wp_index = min(wp_index + 1, len(waypoints))
                    steps_on_wp = 0
                    if moves_since_progress >= STUCK_CHECK_MOVES:
                        with _loop_bucket(nav_timing, "stuck_check_ms"):
                            stuck_xy = pos_xy
                            stuck = dist2d(stuck_xy, last_progress_xy) < STUCK_MOVE_THRESHOLD_CM
                        if stuck:
                            recovery = _apply_stuck_recovery(
                                ucv,
                                layers,
                                robot_name,
                                goal_xy,
                                stuck_xy=stuck_xy,
                                waypoints=waypoints,
                                wp_index=wp_index,
                                waypoint_xy=waypoint_xy,
                                stuck_hotspots=stuck_hotspots,
                                unstuck_attempts=unstuck_attempts,
                                l2_seen_cells=l2_seen_cells,
                                trace=trace,
                                carry_motion_cb=carry_motion_cb,
                                soft_reset_fn=soft_reset_fn,
                                nav_kpi=nav_kpi,
                            )
                            if recovery.mission_failed:
                                last_resort = _run_last_resort_recovery(
                                    layers=layers,
                                    pos_xy=pos_xy,
                                    goal_xy=goal_xy,
                                    l2_seen_cells=l2_seen_cells,
                                    stuck_xy=stuck_xy,
                                    soft_reset_fn=soft_reset_fn,
                                    nav_ctx=nav_ctx,
                                )
                                if last_resort is None:
                                    return False
                                waypoints, wp_index, _l2_perceive_pause_steps = last_resort
                                _l2_flush_count = (
                                    nav_ctx.l2_flush_count if nav_ctx is not None else _l2_flush_count + 1
                                )
                                unstuck_attempts = 0
                                moves_since_progress = 0
                                stuck_hotspots = []
                                ucv = recovery.ucv
                                last_motion_t = time.time()
                                last_progress_xy = pos_xy
                                continue
                            ucv = recovery.ucv
                            waypoints = recovery.waypoints
                            wp_index = recovery.wp_index
                            steps_on_wp = recovery.steps_on_wp
                            unstuck_attempts = recovery.unstuck_attempts
                            stuck_hotspots = recovery.stuck_hotspots
                            last_progress_xy = recovery.pos_xy
                            last_motion_t = time.time()
                        else:
                            unstuck_attempts = 0
                            last_progress_xy = stuck_xy
                        moves_since_progress = 0
                    continue

            move_t0 = time.perf_counter()
            rpp_stalled = False
            pos_before_move = pos_xy
            if USE_RPP_CONTROLLER and command.move_cm > 1e-3:
                pos_xy, robot_yaw_deg, ucv, rpp_stalled = _execute_rpp_closed_loop(
                    ucv,
                    robot_name,
                    pos_xy,
                    robot_yaw_deg,
                    waypoint_xy,
                    waypoints,
                    wp_index,
                    max_move_cm=allowed_move if command.move_cm > 1e-3 else SITE_MAX_OPEN_LOOP_MOVE_CM,
                    diag=(moves_executed == 0),
                    on_after_motion=carry_motion_cb,
                    nav_timing=nav_timing,
                    on_move_cm_fn=on_move_cm_fn,
                    pose_cache=pose_cache,
                )
            else:
                _execute_segment_command(
                    ucv,
                    command,
                    robot_name,
                    diag=(moves_executed == 0),
                    on_after_motion=carry_motion_cb,
                    nav_timing=nav_timing,
                    pose_cache=pose_cache,
                )
                if command.move_cm > 1e-3 and on_move_cm_fn is not None:
                    on_move_cm_fn(command.move_cm)
            if nav_timing is not None:
                nav_timing.move_ms += (time.perf_counter() - move_t0) * 1000.0
            if rpp_stalled:
                moves_since_progress += 1
            if command.move_cm <= 1e-3 and command.turn_deg > SITE_ROTATE_THR_DEG:
                with _loop_bucket(nav_timing, "nav_branch_ms"):
                    turn_only_steps += 1
            else:
                turn_only_steps = 0
            if turn_only_steps >= MAX_TURN_ONLY_STEPS:
                new_wps = _timed_replan_on_merged_layers(
                    layers,
                    pos_xy,
                    goal_xy,
                    reason="turn_only_stall",
                    trace=trace,
                    l2_seen_cells=l2_seen_cells,
                    nav_timing=nav_timing,
                    nav_kpi=nav_kpi,
                )
                if new_wps is not None:
                    waypoints = new_wps
                    wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                    print(
                        f"  [SiteNav] turn-only stall ({turn_only_steps} steps) "
                        f"→ replan {len(waypoints)} WP"
                    )
                turn_only_steps = 0
            last_motion_t = time.time()
            if POST_MOTION_SETTLE_S > 0:
                _nav_settle(
                    ucv,
                    settle_s=POST_MOTION_SETTLE_S,
                    ticks=1,
                    nav_timing=nav_timing,
                )
            if command.move_cm > 1e-3 and depth_prefetch_fn is not None:
                perceive_due = (time.time() - last_perception_t) >= max(
                    0.0,
                    perception_interval_s - 0.35,
                )
                if perceive_due:
                    try:
                        depth_prefetch_fn(perceive_due=True)
                    except TypeError:
                        depth_prefetch_fn()
            pos_xy, robot_yaw_deg, ucv = _fetch_nav_pose(
                ucv, robot_name, nav_timing, pose_cache
            )
            if nav_kpi is not None and command.move_cm > 1e-3:
                nav_kpi.update_open_loop_scale(
                    command.move_cm, dist2d(pos_before_move, pos_xy)
                )
            moves_executed += 1
            total_steps += 1
            steps_on_wp += 1
            moves_since_progress += 1

            if moves_since_progress >= STUCK_CHECK_MOVES:
                with _loop_bucket(nav_timing, "stuck_check_ms"):
                    new_xy = pos_xy
                    stuck = dist2d(new_xy, last_progress_xy) < STUCK_MOVE_THRESHOLD_CM
                if stuck:
                    recovery = _apply_stuck_recovery(
                        ucv,
                        layers,
                        robot_name,
                        goal_xy,
                        stuck_xy=new_xy,
                        waypoints=waypoints,
                        wp_index=wp_index,
                        waypoint_xy=waypoint_xy,
                        stuck_hotspots=stuck_hotspots,
                        unstuck_attempts=unstuck_attempts,
                        l2_seen_cells=l2_seen_cells,
                        trace=trace,
                        carry_motion_cb=carry_motion_cb,
                        soft_reset_fn=soft_reset_fn,
                        nav_kpi=nav_kpi,
                    )
                    if recovery.mission_failed:
                        last_resort = _run_last_resort_recovery(
                            layers=layers,
                            pos_xy=new_xy,
                            goal_xy=goal_xy,
                            l2_seen_cells=l2_seen_cells,
                            stuck_xy=new_xy,
                            soft_reset_fn=soft_reset_fn,
                            nav_ctx=nav_ctx,
                        )
                        if last_resort is None:
                            return False
                        waypoints, wp_index, _l2_perceive_pause_steps = last_resort
                        _l2_flush_count = (
                            nav_ctx.l2_flush_count if nav_ctx is not None else _l2_flush_count + 1
                        )
                        unstuck_attempts = 0
                        moves_since_progress = 0
                        stuck_hotspots = []
                        ucv = recovery.ucv
                        last_motion_t = time.time()
                        last_progress_xy = new_xy
                        continue
                    ucv = recovery.ucv
                    waypoints = recovery.waypoints
                    wp_index = recovery.wp_index
                    steps_on_wp = recovery.steps_on_wp
                    unstuck_attempts = recovery.unstuck_attempts
                    stuck_hotspots = recovery.stuck_hotspots
                    new_xy = recovery.pos_xy
                    last_motion_t = time.time()
                else:
                    unstuck_attempts = 0
                last_progress_xy = new_xy
                moves_since_progress = 0

            if steps_on_wp >= PATH_MAX_STEPS_PER_WP and wp_index < len(waypoints):
                with _loop_bucket(nav_timing, "waypoint_select_ms"):
                    wp_index += 1
                    steps_on_wp = 0

        _account_outer_loop_slice(
            loop_t0, accounted_at_start, loop_bucket_at_start, nav_timing
        )

        if total_steps % 25 == 0:
            print(
                f"  [SiteNav] step={total_steps} dist_goal={dist2d(pos_xy, goal_xy):.0f}cm "
                f"wp={wp_index + 1}/{len(waypoints)} l2_cells={len(l2_seen_cells)}"
            )

    print(f"  [SiteNav] ERROR: exceeded max_total_steps={max_total_steps}")
    if trace is not None:
        trace.l2_cell_count = len(l2_seen_cells)
    _finalize_nav_timing()
    if nav_timing is not None:
        print(
            f"  [SiteNav] timing_ms perceive={nav_timing.perceive_ms:.0f} "
            f"move={nav_timing.move_ms:.0f} replan={nav_timing.replan_ms:.0f} "
            f"settle={nav_timing.settle_ms:.0f} total={nav_timing.total_ms():.0f}"
        )
    return False


def navigate_to_slot(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    slot_id: str,
    *,
    object_registry: object,
    perceive_fn: PerceiveFn,
    standoff_fn: Optional[Callable[[WorldXY, WorldXY], WorldXY]] = None,
    fallback_goal_local: Optional[Tuple[float, float]] = None,
    soft_reset_fn: Optional[Callable[..., None]] = None,
    robot_name: str = ROBOT_ACTOR,
    nav_actor: Optional[str] = None,
    tolerance_cm: float = 120.0,
    label: str = "",
    perception_interval_s: float = SITE_DEFAULT_PERCEPTION_INTERVAL_S,
    max_total_steps: int = PATH_MAX_TOTAL_STEPS,
    trace: Optional[NavTrace] = None,
    on_pose_sample: Optional[PoseSampleFn] = None,
    nav_timing: Optional[NavTimingAccumulator] = None,
    extra_obstacle_positions_fn: Optional[Callable[[], Sequence[WorldXY]]] = None,
    forward_depth_cm_fn: Optional[Callable[[], Optional[float]]] = None,
    depth_refresh_fn: Optional[Callable[[], Optional[float]]] = None,
    depth_invalidate_fn: Optional[Callable[[str], None]] = None,
    on_move_cm_fn: Optional[Callable[[float], None]] = None,
    depth_prefetch_fn: Optional[Callable[..., None]] = None,
    on_nav_iter_start_fn: Optional[Callable[[], None]] = None,
    pose_cache: Optional[PoseCache] = None,
    nav_ctx: Optional[Any] = None,
    perceive_depth_refresh_fn: Optional[Callable[[], Optional[float]]] = None,
) -> bool:
    from carry import pickup_standoff_xy  # noqa: WPS433

    goal_xy = object_registry.goal_xy(slot_id) if hasattr(object_registry, "goal_xy") else None
    if goal_xy is None and fallback_goal_local is not None:
        goal_xy = local_xy_to_world(*fallback_goal_local)
    if goal_xy is None:
        print(f"  [SiteNav] navigate_to_slot: unknown slot {slot_id}")
        return False

    robot_xy = get_pos2d(ucv, robot_name)
    if standoff_fn is not None:
        approach_xy = standoff_fn(goal_xy, robot_xy)
    else:
        approach_xy = pickup_standoff_xy(goal_xy, robot_xy)
    approach_local = world_xy_to_local(*approach_xy)
    nav_label = label or f"to-slot-{slot_id}"
    print(f"  [SiteNav] navigate_to_slot {slot_id} goal={goal_xy} approach={approach_xy}")
    return navigate_layered_with_fusion(
        ucv,
        layers,
        approach_local,
        perceive_fn=perceive_fn,
        soft_reset_fn=soft_reset_fn,
        robot_name=robot_name,
        nav_actor=nav_actor,
        tolerance_cm=tolerance_cm,
        label=nav_label,
        perception_interval_s=perception_interval_s,
        max_total_steps=max_total_steps,
        trace=trace,
        on_pose_sample=on_pose_sample,
        nav_timing=nav_timing,
        extra_obstacle_positions_fn=extra_obstacle_positions_fn,
        forward_depth_cm_fn=forward_depth_cm_fn,
        depth_refresh_fn=depth_refresh_fn,
        depth_invalidate_fn=depth_invalidate_fn,
        on_move_cm_fn=on_move_cm_fn,
        depth_prefetch_fn=depth_prefetch_fn,
        on_nav_iter_start_fn=on_nav_iter_start_fn,
        pose_cache=pose_cache,
        nav_ctx=nav_ctx,
        perceive_depth_refresh_fn=perceive_depth_refresh_fn,
    )


def deliver_to(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    slot_id: str,
    *,
    object_registry: object,
    perceive_fn: PerceiveFn,
    fallback_goal_local: Optional[Tuple[float, float]] = None,
    soft_reset_fn: Optional[Callable[..., None]] = None,
    robot_name: str = ROBOT_ACTOR,
    nav_actor: Optional[str] = None,
    tolerance_cm: float = 120.0,
    label: str = "",
    perception_interval_s: float = SITE_DEFAULT_PERCEPTION_INTERVAL_S,
    max_total_steps: int = PATH_MAX_TOTAL_STEPS,
    trace: Optional[NavTrace] = None,
    carry_sync_name: Optional[str] = None,
    on_pose_sample: Optional[PoseSampleFn] = None,
    nav_timing: Optional[NavTimingAccumulator] = None,
    extra_obstacle_positions_fn: Optional[Callable[[], Sequence[WorldXY]]] = None,
    forward_depth_cm_fn: Optional[Callable[[], Optional[float]]] = None,
    depth_refresh_fn: Optional[Callable[[], Optional[float]]] = None,
    depth_invalidate_fn: Optional[Callable[[str], None]] = None,
    on_move_cm_fn: Optional[Callable[[float], None]] = None,
    depth_prefetch_fn: Optional[Callable[..., None]] = None,
    on_nav_iter_start_fn: Optional[Callable[[], None]] = None,
    pose_cache: Optional[PoseCache] = None,
    nav_ctx: Optional[Any] = None,
    perceive_depth_refresh_fn: Optional[Callable[[], Optional[float]]] = None,
) -> bool:
    """Navigate directly to semantic slot goal (e.g. humanoid delivery point)."""
    goal_local = object_registry.goal_local(slot_id) if hasattr(object_registry, "goal_local") else None
    if goal_local is None:
        goal_local = fallback_goal_local
    if goal_local is None:
        print(f"  [SiteNav] deliver_to: unknown slot {slot_id}")
        return False
    nav_label = label or f"deliver-to-{slot_id}"
    print(f"  [SiteNav] deliver_to {slot_id} goal_local={goal_local}")
    return navigate_layered_with_fusion(
        ucv,
        layers,
        goal_local,
        perceive_fn=perceive_fn,
        soft_reset_fn=soft_reset_fn,
        robot_name=robot_name,
        nav_actor=nav_actor,
        tolerance_cm=tolerance_cm,
        label=nav_label,
        perception_interval_s=perception_interval_s,
        max_total_steps=max_total_steps,
        trace=trace,
        carry_sync_name=carry_sync_name,
        on_pose_sample=on_pose_sample,
        nav_timing=nav_timing,
        extra_obstacle_positions_fn=extra_obstacle_positions_fn,
        forward_depth_cm_fn=forward_depth_cm_fn,
        depth_refresh_fn=depth_refresh_fn,
        depth_invalidate_fn=depth_invalidate_fn,
        on_move_cm_fn=on_move_cm_fn,
        depth_prefetch_fn=depth_prefetch_fn,
        on_nav_iter_start_fn=on_nav_iter_start_fn,
        pose_cache=pose_cache,
        nav_ctx=nav_ctx,
        perceive_depth_refresh_fn=perceive_depth_refresh_fn,
    )
