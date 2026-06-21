#!/usr/bin/env python3
"""NavMesh path following with perception sampling (NavFindPath via NavQueryService)."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

from ground_truth import GroundTruthObservation, ground_truth_all_props, normalize_angle_deg
from pie_safety import MAX_LEG_DURATION_S, NAV_MAX_STEPS_DEFAULT, PERCEPTION_MIN_INTERVAL_S

import nav_query as nq  # noqa: E402
from level_coords import NAV_PROJECT_PROBE_Z_CM  # noqa: E402
from simple_nav import (  # noqa: E402
    ConnectionCheck,
    NavigationRunResult,
    SampleCallback,
    TimeSeriesSample,
    _gt_to_dict,
    dist2d,
    yaw_to_target,
)

WorldXY = Tuple[float, float]

ROBOT_SPEED = 180.0
ROBOT_MOVE_SLICE_S = 0.22
ROBOT_TURN_SLICE_S = 0.22
ROTATE_THR_DEG = 6.0
GOAL_TOLERANCE_CM = 120.0
MAX_TURN_DEG_PER_STEP = 22.0
PATH_WP_REACH_TOLERANCE_CM = 80.0
PATH_REPLAN_STUCK_STEPS = 12
PATH_MAX_OPEN_LOOP_MOVE_CM = 320.0


@dataclass(frozen=True)
class _SegmentCommand:
    turn_deg: float
    turn_clockwise: int
    move_cm: float


def _segment_command(
    pos_xy: WorldXY,
    yaw_deg: float,
    waypoint_xy: WorldXY,
) -> Optional[_SegmentCommand]:
    distance_cm = dist2d(pos_xy, waypoint_xy)
    if distance_cm < 1e-3:
        return None
    target_yaw = yaw_to_target(pos_xy, waypoint_xy)
    angle_diff = normalize_angle_deg(target_yaw - yaw_deg)
    if abs(angle_diff) > ROTATE_THR_DEG:
        clockwise = 1 if angle_diff < 0.0 else -1
        return _SegmentCommand(
            turn_deg=min(abs(angle_diff), MAX_TURN_DEG_PER_STEP),
            turn_clockwise=clockwise,
            move_cm=0.0,
        )
    move_cm = min(distance_cm, PATH_MAX_OPEN_LOOP_MOVE_CM)
    return _SegmentCommand(turn_deg=0.0, turn_clockwise=1, move_cm=move_cm)


def _execute_segment(ucv, robot_name: str, command: _SegmentCommand) -> None:
    if command.turn_deg > ROTATE_THR_DEG:
        ucv.dog_rotate(
            robot_name,
            [ROBOT_TURN_SLICE_S, command.turn_deg, command.turn_clockwise],
        )
        time.sleep(ROBOT_TURN_SLICE_S * 0.35)
    if command.move_cm > 1e-3:
        move_duration_s = max(ROBOT_MOVE_SLICE_S, command.move_cm / ROBOT_SPEED)
        ucv.dog_move(robot_name, [ROBOT_SPEED, move_duration_s, 0])
        time.sleep(move_duration_s * 0.35)


def _robot_foot_xyz(ucv, robot_name: str) -> Tuple[float, float, float]:
    loc = ucv.get_location(robot_name)
    return float(loc[0]), float(loc[1]), float(loc[2])


def _plan_navmesh_path(
    ucv,
    nav_actor: str,
    start_xyz: Tuple[float, float, float],
    goal_xyz: Tuple[float, float, float],
) -> List[WorldXY]:
    raw = nq.nav_find_path(ucv, nav_actor, start_xyz, goal_xyz)
    points = nq.path_points_xy(raw)
    if not points:
        return [(goal_xyz[0], goal_xyz[1])]
    return points


def navigate_to_target_navmesh(
    ucv,
    robot_name: str,
    goal_xy: WorldXY,
    *,
    nav_actor: str,
    registry,
    fov_deg: float,
    perceive_fn: Callable[[], dict],
    get_pose_fn: Callable[[], Tuple[WorldXY, float]],
    target_prop_type_id: str,
    t0: float,
    goal_z_cm: Optional[float] = None,
    on_sample: Optional[SampleCallback] = None,
    connection_check: Optional[ConnectionCheck] = None,
    max_steps: int = NAV_MAX_STEPS_DEFAULT,
    max_leg_duration_s: float = MAX_LEG_DURATION_S,
    sample_interval_s: float = PERCEPTION_MIN_INTERVAL_S,
) -> NavigationRunResult:
    """Follow UE NavMesh polyline to goal_xy while sampling perception."""
    result = NavigationRunResult(target_prop_type_id=target_prop_type_id)
    last_sample_t = -1e9
    leg_start = time.time()
    steps_on_wp = 0
    wp_index = 0

    start_xyz = _robot_foot_xyz(ucv, robot_name)
    gz = goal_z_cm if goal_z_cm is not None else NAV_PROJECT_PROBE_Z_CM
    goal_xyz = (goal_xy[0], goal_xy[1], gz)
    waypoints = _plan_navmesh_path(ucv, nav_actor, start_xyz, goal_xyz)
    print(
        f"[NavMesh] {target_prop_type_id}: {len(waypoints)} waypoints "
        f"start=({start_xyz[0]:.0f},{start_xyz[1]:.0f}) goal=({goal_xy[0]:.0f},{goal_xy[1]:.0f})"
    )

    for step_i in range(max_steps):
        if connection_check is not None and not connection_check():
            result.aborted = True
            result.abort_reason = "UnrealCV connection lost"
            break
        if time.time() - leg_start > max_leg_duration_s:
            result.aborted = True
            result.abort_reason = f"leg timeout ({max_leg_duration_s:.0f}s)"
            break

        pos_xy, yaw_deg = get_pose_fn()
        if dist2d(pos_xy, goal_xy) <= GOAL_TOLERANCE_CM:
            result.reached = True
            break

        now = time.time()
        if now - last_sample_t >= sample_interval_s:
            est = perceive_fn()
            gt_map = ground_truth_all_props(pos_xy, yaw_deg, registry, fov_deg=fov_deg)
            sample = TimeSeriesSample(
                t_s=now - t0,
                robot_xy=pos_xy,
                robot_yaw_deg=yaw_deg,
                estimates=est,
                ground_truth={k: _gt_to_dict(v) for k, v in gt_map.items()},
            )
            result.samples.append(sample)
            if on_sample is not None:
                on_sample(sample)
            last_sample_t = now

        if wp_index >= len(waypoints):
            waypoint_xy = goal_xy
        else:
            waypoint_xy = waypoints[wp_index]
            if dist2d(pos_xy, waypoint_xy) <= PATH_WP_REACH_TOLERANCE_CM:
                wp_index += 1
                steps_on_wp = 0
                continue

        command = _segment_command(pos_xy, yaw_deg, waypoint_xy)
        if command is None:
            wp_index += 1
            steps_on_wp = 0
            continue

        _execute_segment(ucv, robot_name, command)
        steps_on_wp += 1

        if steps_on_wp >= PATH_REPLAN_STUCK_STEPS:
            pos_xy = get_pose_fn()[0]
            if dist2d(pos_xy, goal_xy) <= GOAL_TOLERANCE_CM:
                result.reached = True
                break
            cur_xyz = _robot_foot_xyz(ucv, robot_name)
            waypoints = _plan_navmesh_path(ucv, nav_actor, cur_xyz, goal_xyz)
            wp_index = 0
            steps_on_wp = 0
            print(
                f"[NavMesh] replan @ ({pos_xy[0]:.0f},{pos_xy[1]:.0f}) "
                f"→ {len(waypoints)} waypoints (step {step_i})"
            )

    if connection_check is None or connection_check():
        pos_xy, yaw_deg = get_pose_fn()
        est = perceive_fn()
        gt_map = ground_truth_all_props(pos_xy, yaw_deg, registry, fov_deg=fov_deg)
        result.samples.append(
            TimeSeriesSample(
                t_s=time.time() - t0,
                robot_xy=pos_xy,
                robot_yaw_deg=yaw_deg,
                estimates=est,
                ground_truth={k: _gt_to_dict(v) for k, v in gt_map.items()},
            )
        )
    return result
