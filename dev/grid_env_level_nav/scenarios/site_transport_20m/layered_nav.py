#!/usr/bin/env python3
"""L0+L1+L2 navigation with L2 perception updates, carry sync, and motion sampling."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Set, Tuple, Union

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
from costmap_layers import LayeredCostmap  # noqa: E402
from path_planning_costmap import Costmap2D  # noqa: E402
from grid_env_10k_pie_patrol import (  # noqa: E402
    PATH_MAX_STEPS_PER_WP,
    PATH_MAX_TOTAL_STEPS,
    PATH_WP_REACH_TOLERANCE_CM,
    ROBOT_TURN_DUR_S,
    ROTATE_THR_DEG,
    SegmentCommand,
    _nearest_waypoint_index_ahead,
    dist2d,
    get_pos2d,
    get_yaw,
    plan_astar_waypoints,
    segment_command_toward_waypoint,
    yaw_to_target,
)
from carry import sync_carry_pose  # noqa: E402
from l2_fusion import apply_l2_from_fusion_detections, detections_summary  # noqa: E402
from level_coords import local_xy_to_world, world_xy_to_local  # noqa: E402
from perception_layer import L2_LETHAL_COST  # noqa: E402
from pie_safety import PieSessionLost, require_live_ucv, tick_settle  # noqa: E402
from pie_spawn_safety import ensure_live_or_reconnect  # noqa: E402
from viz import NavTrace  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

WorldXY = Tuple[float, float]
GridCell = Tuple[int, int]
ROBOT_ACTOR = geh.ROBOT_ACTOR_NAME
PerceiveFn = Callable[..., Union[list, "PerceiveOutcome"]]
PoseSampleFn = Callable[[WorldXY, float], None]
CarrySyncFn = Callable[[], None]


@dataclass(frozen=True)
class PerceiveOutcome:
    detections: list
    cells_added: int = 0
    cells_removed: int = 0
    l2_applied: bool = False

    @property
    def l2_changed(self) -> bool:
        return self.cells_added > 0 or self.cells_removed > 0


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
PERCEPTION_START_DELAY_S = 0.0
MOTION_SETTLE_BEFORE_PERCEIVE_S = 0.3
POST_MOTION_SETTLE_S = 0.6
MOVES_PER_CYCLE = 1
SITE_ROBOT_SPEED = 130.0
SITE_MOVE_SLICE_S = 0.3
NAV_WARMUP_SETTLE_S = 6.0
FIRST_MOVE_PRIME_CM = 20.0
MAX_TURN_DEG_PER_STEP = 20.0
TURN_SLEEP_FRAC = 0.35
STUCK_MOVE_THRESHOLD_CM = 14.0
STUCK_CHECK_MOVES = 5
UNSTUCK_BACKUP_CM = 100.0
UNSTUCK_BACK_SPEED = 100.0
ESCAPE_STEP_MIN_CM = 70.0
ESCAPE_STEP_MAX_CM = 150.0
ESCAPE_MAX_TURN_DEG = 135.0
MAX_UNSTUCK_ATTEMPTS = 16


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
    cx, cy = center
    marked = 0
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            gx, gy = cx + dx, cy + dy
            if (gx, gy) in l2_seen_cells:
                continue
            layers.set_l2_cell(gx, gy, L2_LETHAL_COST)
            l2_seen_cells.add((gx, gy))
            marked += 1
    return marked


def _replan_on_merged_layers(
    layers: LayeredCostmap,
    pos_xy: WorldXY,
    goal_xy: WorldXY,
    *,
    reason: str,
    trace: Optional[NavTrace],
    l2_seen_cells: Set[Tuple[int, int]],
) -> Optional[list]:
    """Replan using merged L0+L1+L2 costmap."""
    costmap = layers.to_costmap2d()
    try:
        replan = _safe_replan_astar(costmap, pos_xy, goal_xy)
    except ValueError:
        return None
    if not replan.waypoints_xy:
        return None
    if trace is not None:
        trace.record_plan(replan.waypoints_xy, reason=reason)
        trace.l2_cell_count = len(l2_seen_cells)
    return replan.waypoints_xy


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


def _unstuck_backup(ucv: UnrealCV, robot_name: str, backup_cm: float = UNSTUCK_BACKUP_CM) -> None:
    duration_s = max(0.25, backup_cm / UNSTUCK_BACK_SPEED)
    ucv.dog_move(robot_name, [-UNSTUCK_BACK_SPEED, duration_s, 0])
    time.sleep(duration_s * 0.35)


def _normalize_angle(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def _find_escape_step_world_xy(
    layers: LayeredCostmap,
    pos_xy: WorldXY,
    goal_xy: WorldXY,
    yaw_deg: float,
) -> Optional[WorldXY]:
    """Choose a short traversable escape target on merged L0+L1+L2."""
    costmap = layers.to_costmap2d()
    start = costmap.world_xy_to_grid(pos_xy, clamp=True)
    if start is None:
        return None
    sx, sy = start
    max_r = max(1, int(ESCAPE_STEP_MAX_CM / costmap.resolution_cm))
    min_dist = ESCAPE_STEP_MIN_CM
    best: Optional[WorldXY] = None
    best_score = float("inf")
    for dy in range(-max_r, max_r + 1):
        for dx in range(-max_r, max_r + 1):
            gx, gy = sx + dx, sy + dy
            if gx < 0 or gy < 0 or gx >= costmap.width_cells or gy >= costmap.height_cells:
                continue
            if not costmap.is_traversable((gx, gy)):
                continue
            candidate = costmap.grid_to_world_xy_center((gx, gy))
            step_dist = dist2d(candidate, pos_xy)
            if step_dist < min_dist or step_dist > ESCAPE_STEP_MAX_CM:
                continue
            turn = abs(_normalize_angle(yaw_to_target(pos_xy, candidate) - yaw_deg))
            if turn > ESCAPE_MAX_TURN_DEG:
                continue
            # Prefer useful progress, but allow sideways escape when the direct path is blocked.
            score = dist2d(candidate, goal_xy) + turn * 1.5 - step_dist * 0.1
            if score < best_score:
                best_score = score
                best = candidate
    return best


def _execute_escape_step(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    robot_name: str,
    pos_xy: WorldXY,
    goal_xy: WorldXY,
    *,
    on_after_motion: Optional[CarrySyncFn] = None,
) -> Optional[WorldXY]:
    try:
        yaw_deg = get_yaw(ucv, robot_name)
    except (ConnectionError, OSError, ValueError, RuntimeError):
        return None
    escape_xy = _find_escape_step_world_xy(layers, pos_xy, goal_xy, yaw_deg)
    if escape_xy is None:
        return None
    print(
        f"  [SiteNav] escape step → local={world_xy_to_local(*escape_xy)} "
        f"dist={dist2d(pos_xy, escape_xy):.0f}cm"
    )
    for _ in range(2):
        cur_xy, _ = _safe_get_pos2d(ucv, robot_name)
        if dist2d(cur_xy, escape_xy) <= PATH_WP_REACH_TOLERANCE_CM:
            return cur_xy
        command = segment_command_toward_waypoint(cur_xy, get_yaw(ucv, robot_name), escape_xy)
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
    while remaining > ROTATE_THR_DEG:
        step_deg = min(remaining, MAX_TURN_DEG_PER_STEP)
        if diag:
            print(
                f"  [SiteNav] UE-RISK dog_rotate {step_deg:.1f}° "
                f"cw={clockwise} (chunk of {turn_deg:.1f}°)"
            )
        turn_duration_s = max(0.25, ROBOT_TURN_DUR_S * step_deg / 90.0)
        ucv.dog_rotate(robot_name, [turn_duration_s, step_deg, clockwise])
        time.sleep(turn_duration_s * TURN_SLEEP_FRAC)
        if on_after_motion is not None:
            on_after_motion()
        remaining -= step_deg


def _prime_first_motion(ucv: UnrealCV, robot_name: str) -> None:
    """Short forward move before first turn — stabilizes SpotDog physics after spawn."""
    duration_s = max(SITE_MOVE_SLICE_S, FIRST_MOVE_PRIME_CM / SITE_ROBOT_SPEED)
    print(f"  [SiteNav] prime dog_move {FIRST_MOVE_PRIME_CM:.0f}cm before first turn")
    ucv.dog_move(robot_name, [SITE_ROBOT_SPEED * 0.6, duration_s, 0])
    time.sleep(duration_s * 0.4)


def _execute_segment_command(
    ucv: UnrealCV,
    command: SegmentCommand,
    robot_name: str,
    *,
    diag: bool = False,
    on_after_motion: Optional[CarrySyncFn] = None,
) -> None:
    """Site-tuned open-loop move (slower speed; uses actual robot_name)."""
    if command.turn_deg > ROTATE_THR_DEG:
        _dog_rotate_chunked(
            ucv,
            robot_name,
            command.turn_deg,
            command.turn_clockwise,
            diag=diag,
            on_after_motion=on_after_motion,
        )
    if command.move_cm > 1e-3:
        if diag:
            print(f"  [SiteNav] UE-RISK dog_move {command.move_cm:.1f}cm")
        move_duration_s = max(SITE_MOVE_SLICE_S, command.move_cm / SITE_ROBOT_SPEED)
        ucv.dog_move(robot_name, [SITE_ROBOT_SPEED, move_duration_s, 0])
        time.sleep(move_duration_s * 0.15)
        if on_after_motion is not None:
            on_after_motion()


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


def navigate_layered_with_fusion(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    goal_local_xy: Tuple[float, float],
    *,
    perceive_fn: PerceiveFn,
    robot_name: str = ROBOT_ACTOR,
    tolerance_cm: float = 120.0,
    label: str = "",
    perception_interval_s: float = SITE_DEFAULT_PERCEPTION_INTERVAL_S,
    max_total_steps: int = PATH_MAX_TOTAL_STEPS,
    trace: Optional[NavTrace] = None,
    carry_sync_name: Optional[str] = None,
    on_pose_sample: Optional[PoseSampleFn] = None,
) -> bool:
    require_live_ucv(ucv, context="site transport fusion nav")
    goal_xy = local_xy_to_world(*goal_local_xy)
    l2_seen_cells: Set[Tuple[int, int]] = set()

    def _sync_carry() -> None:
        if carry_sync_name and geh.actor_exists(ucv, carry_sync_name):
            sync_carry_pose(ucv, carry_sync_name, robot_name)

    start_xy, ucv = _safe_get_pos2d(ucv, robot_name)
    plan = _safe_replan_astar(layers.to_costmap2d(), start_xy, goal_xy)
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
    nav_t0 = time.time()
    moves_executed = 0
    last_motion_t = nav_t0

    print(
        f"  [SiteNav]{f' {label}' if label else ''} goal_local={goal_local_xy} "
        f"waypoints={len(waypoints)} cost={plan.total_cost:.1f}"
    )
    print(f"  [SiteNav] warmup {NAV_WARMUP_SETTLE_S:.1f}s before first dog_move")
    tick_settle(ucv, settle_s=NAV_WARMUP_SETTLE_S, ticks=3)
    _prime_first_motion(ucv, robot_name)
    tick_settle(ucv, settle_s=1.0, ticks=2)
    _sync_carry()

    while total_steps < max_total_steps:
        require_live_ucv(ucv, context=f"site nav step {total_steps}")
        pos_xy, ucv = _safe_get_pos2d(ucv, robot_name)
        if trace is not None:
            trace.record_position(world_xy_to_local(*pos_xy))
        if on_pose_sample is not None:
            on_pose_sample(pos_xy, time.time())
        if dist2d(pos_xy, goal_xy) <= tolerance_cm:
            print(f"  [SiteNav] Arrived dist={dist2d(pos_xy, goal_xy):.1f}cm")
            if trace is not None:
                trace.arrived = True
                trace.l2_cell_count = len(l2_seen_cells)
            _sync_carry()
            return True

        now = time.time()
        motion_quiet = now - last_motion_t >= MOTION_SETTLE_BEFORE_PERCEIVE_S
        if (
            motion_quiet
            and now - last_perception_t >= perception_interval_s
            and now - nav_t0 >= PERCEPTION_START_DELAY_S
        ):
            l2_count_before = _l2_occupied_count(layers)
            try:
                outcome = _invoke_perceive(
                    perceive_fn, layers=layers, l2_seen_cells=l2_seen_cells
                )
            except Exception as exc:
                print(f"  [SiteNav] perceive error: {exc}")
                outcome = PerceiveOutcome(detections=[], l2_applied=False)
            detections = outcome.detections
            l2_count_after = _l2_occupied_count(layers)
            if outcome.l2_applied:
                if outcome.l2_changed:
                    summary = detections_summary(detections) if detections else {}
                    new_wps = _replan_on_merged_layers(
                        layers,
                        pos_xy,
                        goal_xy,
                        reason="l2_sight",
                        trace=trace,
                        l2_seen_cells=l2_seen_cells,
                    )
                    if new_wps is not None:
                        waypoints = new_wps
                        wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                        print(
                            f"  [SiteNav] L2 sight +{outcome.cells_added}/-{outcome.cells_removed} "
                            f"cells detect={list(summary.keys())} "
                            f"→ replan {len(waypoints)} WP (merged L0+L1+L2)"
                        )
                    else:
                        print(
                            f"  [SiteNav] L2 sight +{outcome.cells_added}/-{outcome.cells_removed} "
                            f"cells → replan failed (merged map)"
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
                    if n_cells > 0:
                        new_wps = _replan_on_merged_layers(
                            layers,
                            pos_xy,
                            goal_xy,
                            reason="l2_perception",
                            trace=trace,
                            l2_seen_cells=l2_seen_cells,
                        )
                        if new_wps is not None:
                            waypoints = new_wps
                            wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                            print(
                                f"  [SiteNav] L2 +{n_cells} cells detect={list(summary.keys())} "
                                f"→ replan {len(waypoints)} WP (merged L0+L1+L2)"
                            )
                        else:
                            print(
                                f"  [SiteNav] L2 +{n_cells} cells detect={list(summary.keys())} "
                                f"→ replan failed (merged map)"
                            )
                    else:
                        print(
                            f"  [SiteNav] L2 detect={list(summary.keys())} "
                            f"(no new cells, l2_cells={len(l2_seen_cells)})"
                        )
            elif l2_count_after > l2_count_before:
                _sync_seen_cells_from_l2(layers, l2_seen_cells)
                new_wps = _replan_on_merged_layers(
                    layers,
                    pos_xy,
                    goal_xy,
                    reason="l2_depth",
                    trace=trace,
                    l2_seen_cells=l2_seen_cells,
                )
                if new_wps is not None:
                    waypoints = new_wps
                    wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, wp_index)
                    print(
                        f"  [SiteNav] L2 depth +{l2_count_after - l2_count_before} cells "
                        f"→ replan {len(waypoints)} WP (merged L0+L1+L2)"
                    )
                else:
                    print(
                        f"  [SiteNav] L2 depth +{l2_count_after - l2_count_before} cells "
                        f"→ replan failed"
                    )
            else:
                print(f"  [SiteNav] L2 perceive: 0 detections (l2_cells={len(l2_seen_cells)})")
            last_perception_t = now

        for _ in range(MOVES_PER_CYCLE):
            if total_steps >= max_total_steps:
                break
            pos_xy, ucv = _safe_get_pos2d(ucv, robot_name)
            if dist2d(pos_xy, goal_xy) <= tolerance_cm:
                print(f"  [SiteNav] Arrived dist={dist2d(pos_xy, goal_xy):.1f}cm")
                if trace is not None:
                    trace.arrived = True
                    trace.l2_cell_count = len(l2_seen_cells)
                _sync_carry()
                return True

            _sync_carry()

            if wp_index >= len(waypoints):
                waypoint_xy = goal_xy
            else:
                waypoint_xy = waypoints[wp_index]
                if dist2d(pos_xy, waypoint_xy) <= PATH_WP_REACH_TOLERANCE_CM:
                    wp_index += 1
                    steps_on_wp = 0
                    continue

            command = segment_command_toward_waypoint(
                pos_xy,
                get_yaw(ucv, robot_name),
                waypoint_xy,
            )
            if command is None:
                if wp_index >= len(waypoints):
                    if dist2d(pos_xy, goal_xy) <= tolerance_cm:
                        return True
                else:
                    wp_index += 1
                steps_on_wp = 0
                continue

            _execute_segment_command(
                ucv,
                command,
                robot_name,
                diag=(moves_executed == 0),
                on_after_motion=_sync_carry,
            )
            last_motion_t = time.time()
            tick_settle(ucv, settle_s=POST_MOTION_SETTLE_S, ticks=1)
            _sync_carry()
            moves_executed += 1
            total_steps += 1
            steps_on_wp += 1
            moves_since_progress += 1

            if moves_since_progress >= STUCK_CHECK_MOVES:
                new_xy, ucv = _safe_get_pos2d(ucv, robot_name)
                if dist2d(new_xy, last_progress_xy) < STUCK_MOVE_THRESHOLD_CM:
                    if unstuck_attempts >= MAX_UNSTUCK_ATTEMPTS:
                        print(
                            f"  [SiteNav] FAIL: stuck at local={world_xy_to_local(*new_xy)} "
                            f"after {MAX_UNSTUCK_ATTEMPTS} backup+replan attempts"
                        )
                        if trace is not None:
                            trace.l2_cell_count = len(l2_seen_cells)
                        return False
                    unstuck_attempts += 1
                    mark_radius = min(4, 1 + unstuck_attempts // 2)
                    n_marked = _mark_stuck_cells_on_l2(
                        layers, new_xy, l2_seen_cells, radius_cells=mark_radius
                    )
                    print(
                        f"  [SiteNav] STUCK @ local={world_xy_to_local(*new_xy)} "
                        f"mark_l2={n_marked} → backup {UNSTUCK_BACKUP_CM:.0f}cm + replan "
                        f"(attempt {unstuck_attempts}/{MAX_UNSTUCK_ATTEMPTS})"
                    )
                    _unstuck_backup(ucv, robot_name)
                    last_motion_t = time.time()
                    _sync_carry()
                    backup_xy, ucv = _safe_get_pos2d(ucv, robot_name)
                    if dist2d(backup_xy, new_xy) < STUCK_MOVE_THRESHOLD_CM * 2.0:
                        escape_xy = _execute_escape_step(
                            ucv,
                            layers,
                            robot_name,
                            backup_xy,
                            goal_xy,
                            on_after_motion=_sync_carry,
                        )
                        if escape_xy is not None:
                            backup_xy = escape_xy
                            last_motion_t = time.time()
                            _sync_carry()
                    new_wps = _replan_on_merged_layers(
                        layers,
                        backup_xy,
                        goal_xy,
                        reason="unstuck_replan",
                        trace=trace,
                        l2_seen_cells=l2_seen_cells,
                    )
                    if new_wps is not None:
                        waypoints = new_wps
                        wp_index = _nearest_waypoint_index_ahead(
                            backup_xy, waypoints, wp_index
                        )
                        print(
                            f"  [SiteNav] unstuck replan → {len(waypoints)} WP "
                            f"(merged L0+L1+L2)"
                        )
                    steps_on_wp = 0
                    new_xy = backup_xy
                else:
                    unstuck_attempts = 0
                last_progress_xy = new_xy
                moves_since_progress = 0

            if steps_on_wp >= PATH_MAX_STEPS_PER_WP and wp_index < len(waypoints):
                wp_index += 1
                steps_on_wp = 0

        if total_steps % 25 == 0:
            pos_xy, ucv = _safe_get_pos2d(ucv, robot_name)
            print(
                f"  [SiteNav] step={total_steps} dist_goal={dist2d(pos_xy, goal_xy):.0f}cm "
                f"wp={wp_index + 1}/{len(waypoints)} l2_cells={len(l2_seen_cells)}"
            )

    print(f"  [SiteNav] ERROR: exceeded max_total_steps={max_total_steps}")
    if trace is not None:
        trace.l2_cell_count = len(l2_seen_cells)
    return False
