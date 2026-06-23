#!/usr/bin/env python3
"""L0+L1+L2 navigation with L2 perception updates, carry sync, and motion sampling."""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Set, Tuple, Union

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
STUCK_CORRIDOR_LENGTH_CM = 200.0
STUCK_CORRIDOR_HALF_WIDTH_CELLS = 3
STUCK_HOTSPOT_RADIUS_CELLS = 4
ESCAPE_MIN_DISPLACEMENT_CM = 35.0
ESCAPE_CANDIDATE_LIMIT = 8

PROGRESS_REGRESS_THRESHOLD_CM = 350.0
MAX_TURN_ONLY_STEPS = 12
MAX_L2_FLUSH_COUNT = 3
LAST_RESORT_PERCEIVE_PAUSE_STEPS = 40
SITE_PLANNING_CLEARANCE_CM = 40.0
SITE_PLANNING_CLEARANCE_COST = 300.0
ROBOT_L2_SELF_EXCLUDE_RADIUS_CM = 70.0


def _planning_costmap(layers: LayeredCostmap) -> Costmap2D:
    """Merged map plus soft 1m clearance cost around lethal cells for planning."""
    base = layers.to_costmap2d()
    radius_cells = max(0, int(math.ceil(SITE_PLANNING_CLEARANCE_CM / base.resolution_cm)))
    if radius_cells <= 0:
        return base
    costs = base.costs.copy()
    lethal = costs >= base.lethal_cost * 0.5
    lethal_ys, lethal_xs = np.nonzero(lethal)
    if lethal_xs.size == 0:
        return base
    for cx, cy in zip(lethal_xs, lethal_ys):
        gx0 = max(0, int(cx) - radius_cells)
        gx1 = min(base.width_cells, int(cx) + radius_cells + 1)
        gy0 = max(0, int(cy) - radius_cells)
        gy1 = min(base.height_cells, int(cy) + radius_cells + 1)
        for gy in range(gy0, gy1):
            for gx in range(gx0, gx1):
                if lethal[gy, gx]:
                    continue
                dist_cm = math.hypot(gx - int(cx), gy - int(cy)) * base.resolution_cm
                if dist_cm <= SITE_PLANNING_CLEARANCE_CM:
                    costs[gy, gx] = max(float(costs[gy, gx]), SITE_PLANNING_CLEARANCE_COST)
    for gy in range(base.height_cells):
        for gx in range(base.width_cells):
            if lethal[gy, gx]:
                continue
            border_dist_cells = min(gx, gy, base.width_cells - 1 - gx, base.height_cells - 1 - gy)
            border_dist_cm = border_dist_cells * base.resolution_cm
            if border_dist_cm <= SITE_PLANNING_CLEARANCE_CM:
                costs[gy, gx] = max(float(costs[gy, gx]), SITE_PLANNING_CLEARANCE_COST)
    return Costmap2D(
        costs=costs,
        origin_xy=base.origin_xy,
        resolution_cm=base.resolution_cm,
        lethal_cost=base.lethal_cost,
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
) -> Optional[list]:
    """Replan using merged L0+L1+L2 costmap."""
    removed_self = _exclude_robot_self_from_l2(layers, pos_xy, l2_seen_cells)
    if removed_self:
        print(f"  [SiteNav] L2 self-exclude {removed_self} cells before replan")
    costmap = _planning_costmap(layers)
    replan = None
    try:
        replan = _safe_replan_astar(costmap, pos_xy, goal_xy)
    except (ValueError, RuntimeError):
        try:
            replan = _safe_replan_astar(layers.to_costmap2d(), pos_xy, goal_xy)
            print("  [SiteNav] replan using tight merged L0+L1+L2 clearance")
        except (ValueError, RuntimeError) as exc:
            print(f"  [SiteNav] tight merged replan failed: {exc}")
            replan = None
    if replan is not None and replan.waypoints_xy:
        if trace is not None:
            trace.record_plan(replan.waypoints_xy, reason=reason)
            trace.l2_cell_count = len(l2_seen_cells)
        return replan.waypoints_xy
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
    while remaining > 1.0:
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
) -> None:
    """Site-tuned open-loop move (slower speed; uses actual robot_name)."""
    if command.turn_deg > SITE_ROTATE_THR_DEG:
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
        _site_dog_move(ucv, robot_name, SITE_ROBOT_SPEED, move_duration_s, direction=0)
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
) -> _StuckRecoveryOutcome:
    stuck_hotspots = list(stuck_hotspots)
    stuck_hotspots.append(stuck_xy)
    pending_attempt = unstuck_attempts + 1
    mark_radius = min(4, 1 + pending_attempt // 2)
    n_marked = _mark_stuck_cells_on_l2(
        layers, stuck_xy, l2_seen_cells, radius_cells=mark_radius
    )
    n_hotspot = _mark_stuck_hotspot_on_l2(layers, stuck_xy, l2_seen_cells)
    block_xy = waypoint_xy if wp_index < len(waypoints) else goal_xy
    n_corridor = _mark_stuck_corridor_on_l2(
        layers, stuck_xy, block_xy, l2_seen_cells
    )
    print(
        f"  [SiteNav] STUCK @ local={world_xy_to_local(*stuck_xy)} "
        f"mark_l2={n_marked} hotspot={n_hotspot} corridor={n_corridor} "
        f"→ backup {UNSTUCK_BACKUP_CM:.0f}cm + replan "
        f"(attempt {pending_attempt}/{MAX_UNSTUCK_ATTEMPTS})"
    )
    _unstuck_backup(ucv, robot_name)
    if carry_motion_cb is not None:
        carry_motion_cb()
    backup_xy, ucv = _safe_get_pos2d(ucv, robot_name)
    if dist2d(backup_xy, stuck_xy) < STUCK_MOVE_THRESHOLD_CM * 2.0:
        escape_xy = _execute_escape_step(
            ucv,
            layers,
            robot_name,
            backup_xy,
            goal_xy,
            stuck_hotspots=stuck_hotspots,
            on_after_motion=carry_motion_cb,
        )
        if escape_xy is not None:
            backup_xy = escape_xy
            if carry_motion_cb is not None:
                carry_motion_cb()
    displacement = dist2d(backup_xy, stuck_xy)
    if displacement >= ESCAPE_MIN_DISPLACEMENT_CM:
        print(f"  [SiteNav] unstuck displacement={displacement:.0f}cm")
        unstuck_attempts = 0
    else:
        unstuck_attempts = pending_attempt
        if displacement < 5.0 and pending_attempt >= 3:
            print(
                f"  [SiteNav] zero-displacement for {pending_attempt} attempts"
                " — fast-track to LAST RESORT"
            )
            unstuck_attempts = MAX_UNSTUCK_ATTEMPTS
        if unstuck_attempts >= MAX_UNSTUCK_ATTEMPTS:
            print(
                f"  [SiteNav] FAIL: stuck at local={world_xy_to_local(*backup_xy)} "
                f"after {MAX_UNSTUCK_ATTEMPTS} backup+replan attempts"
            )
            if trace is not None:
                trace.l2_cell_count = len(l2_seen_cells)
            return _StuckRecoveryOutcome(
                ucv=ucv,
                pos_xy=backup_xy,
                waypoints=waypoints,
                wp_index=wp_index,
                steps_on_wp=0,
                unstuck_attempts=unstuck_attempts,
                stuck_hotspots=stuck_hotspots,
                mission_failed=True,
            )
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
        wp_index = _nearest_waypoint_index_ahead(backup_xy, waypoints, wp_index)
        print(
            f"  [SiteNav] unstuck replan → {len(waypoints)} WP (merged L0+L1+L2)"
        )
    else:
        print(
            f"  [SiteNav] unstuck replan failed at "
            f"local={world_xy_to_local(*backup_xy)}"
        )
    return _StuckRecoveryOutcome(
        ucv=ucv,
        pos_xy=backup_xy,
        waypoints=waypoints,
        wp_index=wp_index,
        steps_on_wp=0,
        unstuck_attempts=unstuck_attempts,
        stuck_hotspots=stuck_hotspots,
        mission_failed=False,
    )


def navigate_layered_with_fusion(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    goal_local_xy: Tuple[float, float],
    *,
    perceive_fn: PerceiveFn,
    soft_reset_fn: Optional[Callable[[Set[Tuple[int, int]], WorldXY], None]] = None,
    robot_name: str = ROBOT_ACTOR,
    nav_actor: Optional[str] = None,
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
    _sync_seen_cells_from_l2(layers, l2_seen_cells)
    use_carry_sync = bool(carry_sync_name) and not is_carry_ue_attached()

    def _sync_carry() -> None:
        if not use_carry_sync or not carry_sync_name:
            return
        if geh.actor_exists(ucv, carry_sync_name):
            sync_carry_pose(ucv, carry_sync_name, robot_name, refresh_collision=False)

    carry_motion_cb: Optional[CarrySyncFn] = _sync_carry if use_carry_sync else None

    start_xy, ucv = _safe_get_pos2d(ucv, robot_name)
    plan = _safe_replan_astar(_planning_costmap(layers), start_xy, goal_xy)
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
    _l2_flush_count = 0
    _l2_perceive_pause_steps = 0

    print(
        f"  [SiteNav]{f' {label}' if label else ''} goal_local={goal_local_xy} "
        f"waypoints={len(waypoints)} cost={plan.total_cost:.1f}"
    )
    print(f"  [SiteNav] warmup {NAV_WARMUP_SETTLE_S:.1f}s before first dog_move")
    tick_settle(ucv, settle_s=NAV_WARMUP_SETTLE_S, ticks=3)
    if nav_actor and is_robot_tipped(ucv, robot_name):
        pos_xy, ucv = _safe_get_pos2d(ucv, robot_name)
        recover_robot_upright(ucv, robot_name, pos_xy, nav_actor=nav_actor)
        tick_settle(ucv, settle_s=1.0, ticks=2)
    _prime_first_motion(ucv, robot_name)
    tick_settle(ucv, settle_s=1.0, ticks=2)
    if use_carry_sync:
        _sync_carry()

    while total_steps < max_total_steps:
        require_live_ucv(ucv, context=f"site nav step {total_steps}")
        pos_xy, ucv = _safe_get_pos2d(ucv, robot_name)
        if trace is not None:
            trace.record_position(world_xy_to_local(*pos_xy))
        if on_pose_sample is not None:
            on_pose_sample(pos_xy, time.time())
        dist_goal = dist2d(pos_xy, goal_xy)
        if dist_goal + 40.0 < best_dist_goal:
            best_dist_goal = dist_goal
            turn_only_steps = 0
        elif dist_goal > best_dist_goal + PROGRESS_REGRESS_THRESHOLD_CM:
            new_wps = _replan_on_merged_layers(
                layers,
                pos_xy,
                goal_xy,
                reason="progress_regress",
                trace=trace,
                l2_seen_cells=l2_seen_cells,
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
            l2_count_before = _l2_occupied_count(layers)
            l2_snapshot = layers.l2.copy()
            l2_seen_snapshot = set(l2_seen_cells)
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
                        reason="l2_depth",
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
                        layers.l2[:, :] = l2_snapshot
                        l2_seen_cells.clear()
                        l2_seen_cells.update(l2_seen_snapshot)
                        print(
                            f"  [SiteNav] L2 sight +{outcome.cells_added}/-{outcome.cells_removed} "
                            f"cells → replan failed; rolled back latest L2 update"
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
                    layers.l2[:, :] = l2_snapshot
                    l2_seen_cells.clear()
                    l2_seen_cells.update(l2_seen_snapshot)
                    print(
                        f"  [SiteNav] L2 depth +{l2_count_after - l2_count_before} cells "
                        f"→ replan failed; rolled back latest L2 update"
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

            if wp_index >= len(waypoints):
                waypoint_xy = goal_xy
            else:
                waypoint_xy = waypoints[wp_index]
                if dist2d(pos_xy, waypoint_xy) <= PATH_WP_REACH_TOLERANCE_CM:
                    wp_index += 1
                    steps_on_wp = 0
                    continue

            command = _smooth_segment_command(
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

            if command.move_cm > 1e-3:
                costmap = _planning_costmap(layers)
                move_target = _open_loop_move_target(pos_xy, waypoint_xy)
                if not world_segment_is_traversable(
                    costmap, pos_xy, move_target, skip_start_cell=True
                ):
                    moves_since_progress += 1
                    total_steps += 1
                    new_wps = _replan_on_merged_layers(
                        layers,
                        pos_xy,
                        goal_xy,
                        reason="move_segment_blocked",
                        trace=trace,
                        l2_seen_cells=l2_seen_cells,
                    )
                    if new_wps is not None:
                        waypoints = new_wps
                        wp_index = _nearest_waypoint_index_ahead(pos_xy, waypoints, 0)
                    else:
                        wp_index += 1
                    steps_on_wp = 0
                    if moves_since_progress >= STUCK_CHECK_MOVES:
                        stuck_xy, ucv = _safe_get_pos2d(ucv, robot_name)
                        if dist2d(stuck_xy, last_progress_xy) < STUCK_MOVE_THRESHOLD_CM:
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
                            )
                            if recovery.mission_failed:
                                if _l2_flush_count < MAX_L2_FLUSH_COUNT:
                                    print(
                                        f"  [SiteNav] LAST RESORT #{_l2_flush_count + 1}:"
                                        " flush all L2 cells, replan on L0+L1 only"
                                    )
                                    if soft_reset_fn is not None:
                                        soft_reset_fn(l2_seen_cells, stuck_xy)
                                    else:
                                        layers.l2[:, :] = 0
                                        l2_seen_cells.clear()
                                    _l2_flush_count += 1
                                    _l2_perceive_pause_steps = LAST_RESORT_PERCEIVE_PAUSE_STEPS
                                    unstuck_attempts = 0
                                    moves_since_progress = 0
                                    stuck_hotspots = []
                                    ucv = recovery.ucv
                                    last_motion_t = time.time()
                                    try:
                                        flush_plan = _safe_replan_astar(
                                            layers.to_costmap2d(), pos_xy, goal_xy
                                        )
                                        waypoints = flush_plan.waypoints_xy
                                        wp_index = _nearest_waypoint_index_ahead(
                                            pos_xy, waypoints, 0
                                        )
                                        last_progress_xy = pos_xy
                                        print(
                                            f"  [SiteNav] L2 flush replan"
                                            f" → {len(waypoints)} WP on L0+L1"
                                            f" (perception paused {LAST_RESORT_PERCEIVE_PAUSE_STEPS} steps)"
                                        )
                                    except (ValueError, RuntimeError) as exc:
                                        print(
                                            f"  [SiteNav] L2 flush replan also failed: {exc}"
                                        )
                                        return False
                                    continue  # skip recovery attr overwrite below
                                else:
                                    return False
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

            _execute_segment_command(
                ucv,
                command,
                robot_name,
                diag=(moves_executed == 0),
                on_after_motion=carry_motion_cb,
            )
            if command.move_cm <= 1e-3 and command.turn_deg > SITE_ROTATE_THR_DEG:
                turn_only_steps += 1
            else:
                turn_only_steps = 0
            if turn_only_steps >= MAX_TURN_ONLY_STEPS:
                new_wps = _replan_on_merged_layers(
                    layers,
                    pos_xy,
                    goal_xy,
                    reason="turn_only_stall",
                    trace=trace,
                    l2_seen_cells=l2_seen_cells,
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
                tick_settle(ucv, settle_s=POST_MOTION_SETTLE_S, ticks=1)
            moves_executed += 1
            total_steps += 1
            steps_on_wp += 1
            moves_since_progress += 1

            if moves_since_progress >= STUCK_CHECK_MOVES:
                new_xy, ucv = _safe_get_pos2d(ucv, robot_name)
                if dist2d(new_xy, last_progress_xy) < STUCK_MOVE_THRESHOLD_CM:
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
                    )
                    if recovery.mission_failed:
                        if _l2_flush_count < MAX_L2_FLUSH_COUNT:
                            print(
                                f"  [SiteNav] LAST RESORT #{_l2_flush_count + 1}:"
                                " flush all L2 cells, replan on L0+L1 only"
                            )
                            if soft_reset_fn is not None:
                                soft_reset_fn(l2_seen_cells, new_xy)
                            else:
                                layers.l2[:, :] = 0
                                l2_seen_cells.clear()
                            _l2_flush_count += 1
                            _l2_perceive_pause_steps = LAST_RESORT_PERCEIVE_PAUSE_STEPS
                            unstuck_attempts = 0
                            moves_since_progress = 0
                            stuck_hotspots = []
                            ucv = recovery.ucv
                            last_motion_t = time.time()
                            try:
                                flush_plan = _safe_replan_astar(
                                    layers.to_costmap2d(), new_xy, goal_xy
                                )
                                waypoints = flush_plan.waypoints_xy
                                wp_index = _nearest_waypoint_index_ahead(
                                    new_xy, waypoints, 0
                                )
                                last_progress_xy = new_xy
                                print(
                                    f"  [SiteNav] L2 flush replan"
                                    f" → {len(waypoints)} WP on L0+L1"
                                    f" (perception paused {LAST_RESORT_PERCEIVE_PAUSE_STEPS} steps)"
                                )
                            except (ValueError, RuntimeError) as exc:
                                print(
                                    f"  [SiteNav] L2 flush replan also failed: {exc}"
                                )
                                return False
                            continue  # skip recovery attr overwrite below
                        else:
                            return False
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


def navigate_to_slot(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    slot_id: str,
    *,
    object_registry: object,
    perceive_fn: PerceiveFn,
    standoff_fn: Optional[Callable[[WorldXY, WorldXY], WorldXY]] = None,
    fallback_goal_local: Optional[Tuple[float, float]] = None,
    soft_reset_fn: Optional[Callable[[Set[Tuple[int, int]], WorldXY], None]] = None,
    robot_name: str = ROBOT_ACTOR,
    nav_actor: Optional[str] = None,
    tolerance_cm: float = 120.0,
    label: str = "",
    perception_interval_s: float = SITE_DEFAULT_PERCEPTION_INTERVAL_S,
    max_total_steps: int = PATH_MAX_TOTAL_STEPS,
    trace: Optional[NavTrace] = None,
    on_pose_sample: Optional[PoseSampleFn] = None,
) -> bool:
    """Navigate to semantic slot via ObjectRegistry goal + optional standoff."""
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
    )


def deliver_to(
    ucv: UnrealCV,
    layers: LayeredCostmap,
    slot_id: str,
    *,
    object_registry: object,
    perceive_fn: PerceiveFn,
    fallback_goal_local: Optional[Tuple[float, float]] = None,
    soft_reset_fn: Optional[Callable[[Set[Tuple[int, int]], WorldXY], None]] = None,
    robot_name: str = ROBOT_ACTOR,
    nav_actor: Optional[str] = None,
    tolerance_cm: float = 120.0,
    label: str = "",
    perception_interval_s: float = SITE_DEFAULT_PERCEPTION_INTERVAL_S,
    max_total_steps: int = PATH_MAX_TOTAL_STEPS,
    trace: Optional[NavTrace] = None,
    carry_sync_name: Optional[str] = None,
    on_pose_sample: Optional[PoseSampleFn] = None,
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
    )
