#!/usr/bin/env python3
"""Simple turn-then-go navigation with perception sampling."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from ground_truth import GroundTruthObservation, ground_truth_all_props, normalize_angle_deg
from pie_safety import MAX_LEG_DURATION_S, NAV_MAX_STEPS_DEFAULT, PERCEPTION_MIN_INTERVAL_S
from prop_placement import PlacementRegistry

WorldXY = Tuple[float, float]

ROBOT_SPEED = 120.0
ROBOT_MOVE_SLICE_S = 0.24
ROBOT_ROTATE_SLICE_S = 0.22
ROTATE_THR_DEG = 6.0
GOAL_TOLERANCE_CM = 90.0
MAX_TURN_DEG_PER_STEP = 22.0
STUCK_MOVE_THRESHOLD_CM = 8.0
STUCK_CHECK_INTERVAL = 8
UNSTUCK_TURN_DEG = 75.0


@dataclass
class TimeSeriesSample:
    t_s: float
    robot_xy: WorldXY
    robot_yaw_deg: float
    estimates: Dict[str, Dict[str, float]]
    ground_truth: Dict[str, Dict[str, float]]


@dataclass
class NavigationRunResult:
    target_prop_type_id: str
    samples: List[TimeSeriesSample] = field(default_factory=list)
    reached: bool = False
    aborted: bool = False
    abort_reason: str = ""


SampleCallback = Callable[[TimeSeriesSample], None]
ConnectionCheck = Callable[[], bool]


def yaw_to_target(from_xy: WorldXY, to_xy: WorldXY) -> float:
    return math.degrees(math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0]))


def dist2d(a: WorldXY, b: WorldXY) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _gt_to_dict(gt: GroundTruthObservation) -> Dict[str, float]:
    return {
        "distance_m": gt.distance_m,
        "bearing_deg": gt.bearing_deg,
        "in_fov": float(gt.in_fov),
    }


def navigate_to_target(
    ucv,
    robot_name: str,
    goal_xy: WorldXY,
    *,
    registry: PlacementRegistry,
    fov_deg: float,
    perceive_fn: Callable[[], Dict[str, Dict[str, float]]],
    get_pose_fn: Callable[[], Tuple[WorldXY, float]],
    target_prop_type_id: str,
    t0: float,
    on_sample: Optional[SampleCallback] = None,
    connection_check: Optional[ConnectionCheck] = None,
    max_steps: int = NAV_MAX_STEPS_DEFAULT,
    max_leg_duration_s: float = MAX_LEG_DURATION_S,
    sample_interval_s: float = PERCEPTION_MIN_INTERVAL_S,
) -> NavigationRunResult:
    result = NavigationRunResult(target_prop_type_id=target_prop_type_id)
    last_sample_t = -1e9
    leg_start = time.time()
    stuck_checks = 0
    last_stuck_xy: Optional[WorldXY] = None

    for _ in range(max_steps):
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

        target_yaw = yaw_to_target(pos_xy, goal_xy)
        angle_diff = normalize_angle_deg(target_yaw - yaw_deg)
        if abs(angle_diff) > ROTATE_THR_DEG:
            clockwise = 1 if angle_diff < 0.0 else -1
            turn_deg = min(abs(angle_diff), MAX_TURN_DEG_PER_STEP)
            ucv.dog_rotate(robot_name, [ROBOT_ROTATE_SLICE_S, turn_deg, clockwise])
            time.sleep(ROBOT_ROTATE_SLICE_S * 0.4)
        else:
            move_cm = min(dist2d(pos_xy, goal_xy), ROBOT_SPEED * ROBOT_MOVE_SLICE_S)
            if move_cm > 2.0:
                ucv.dog_move(robot_name, [ROBOT_SPEED, ROBOT_MOVE_SLICE_S, 0])
                time.sleep(ROBOT_MOVE_SLICE_S * 0.4)

        stuck_checks += 1
        if stuck_checks >= STUCK_CHECK_INTERVAL:
            stuck_checks = 0
            if last_stuck_xy is not None and dist2d(pos_xy, last_stuck_xy) < STUCK_MOVE_THRESHOLD_CM:
                ucv.dog_rotate(robot_name, [ROBOT_ROTATE_SLICE_S, UNSTUCK_TURN_DEG, 1])
                time.sleep(ROBOT_ROTATE_SLICE_S * 0.4)
            last_stuck_xy = pos_xy

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
