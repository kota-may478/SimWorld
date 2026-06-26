"""Regulated Pure Pursuit controller (Nav2 RPP-inspired, UE SegmentCommand output)."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths()

from grid_env_10k_pie_patrol import (  # noqa: E402
    SegmentCommand,
    dist2d,
    normalize_angle,
    yaw_to_target,
)

WorldXY = Tuple[float, float]


@dataclass(frozen=True)
class RppConfig:
    lookahead_cm: float = 80.0
    min_lookahead_cm: float = 40.0
    max_lookahead_cm: float = 150.0
    regulated_linear_scaling_min_radius_cm: float = 120.0
    regulated_linear_scaling_min_speed_frac: float = 0.4
    rotate_to_heading_threshold_deg: float = 35.0
    rotate_thr_deg: float = 12.0
    smooth_turn_move_frac: float = 0.55


def _interpolate_segment(a: WorldXY, b: WorldXY, t: float) -> WorldXY:
    return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))


def find_lookahead_point(
    pos_xy: WorldXY,
    waypoints: Sequence[WorldXY],
    wp_index: int,
    lookahead_cm: float,
) -> WorldXY:
    """Point on the path polyline ``lookahead_cm`` ahead of the robot."""
    if not waypoints:
        return pos_xy
    start = min(max(wp_index, 0), len(waypoints) - 1)
    path: list[WorldXY] = [pos_xy, *list(waypoints[start:])]
    accumulated = 0.0
    for idx in range(len(path) - 1):
        seg_a = path[idx]
        seg_b = path[idx + 1]
        seg_len = dist2d(seg_a, seg_b)
        if seg_len < 1e-6:
            continue
        if accumulated + seg_len >= lookahead_cm:
            remain = lookahead_cm - accumulated
            t = remain / seg_len
            return _interpolate_segment(seg_a, seg_b, t)
        accumulated += seg_len
    return path[-1]


def _turning_radius_cm(pos_xy: WorldXY, yaw_deg: float, target_xy: WorldXY) -> float:
    dist = dist2d(pos_xy, target_xy)
    if dist < 1e-3:
        return float("inf")
    target_yaw = yaw_to_target(pos_xy, target_xy)
    angle_diff = abs(normalize_angle(target_yaw - yaw_deg))
    if angle_diff < 1e-3:
        return float("inf")
    half = math.radians(angle_diff / 2.0)
    return dist / (2.0 * math.sin(half))


def _regulated_move_cap(
    pos_xy: WorldXY,
    yaw_deg: float,
    lookahead_xy: WorldXY,
    *,
    config: RppConfig,
    max_move_cm: float,
) -> float:
    radius = _turning_radius_cm(pos_xy, yaw_deg, lookahead_xy)
    min_radius = config.regulated_linear_scaling_min_radius_cm
    if not math.isfinite(radius) or radius >= min_radius:
        return max_move_cm
    ratio = max(0.0, min(1.0, radius / min_radius))
    min_frac = config.regulated_linear_scaling_min_speed_frac
    frac = min_frac + (1.0 - min_frac) * ratio
    return max_move_cm * frac


def compute_rpp_command(
    pos_xy: WorldXY,
    yaw_deg: float,
    waypoints: Sequence[WorldXY],
    wp_index: int,
    *,
    config: RppConfig,
    max_move_cm: float,
) -> Optional[SegmentCommand]:
    """Compute one SegmentCommand toward the lookahead point on the global path."""
    if not waypoints and wp_index <= 0:
        return None

    lookahead_xy = find_lookahead_point(
        pos_xy,
        waypoints,
        wp_index,
        config.lookahead_cm,
    )
    distance_cm = dist2d(pos_xy, lookahead_xy)
    if distance_cm < 1e-3:
        return None

    target_yaw = yaw_to_target(pos_xy, lookahead_xy)
    angle_diff = normalize_angle(target_yaw - yaw_deg)
    abs_angle = abs(angle_diff)
    clockwise = 1 if angle_diff < 0.0 else -1

    move_cap = _regulated_move_cap(
        pos_xy,
        yaw_deg,
        lookahead_xy,
        config=config,
        max_move_cm=max_move_cm,
    )

    if abs_angle > config.rotate_to_heading_threshold_deg:
        return SegmentCommand(turn_deg=abs_angle, turn_clockwise=clockwise, move_cm=0.0)
    if abs_angle > config.rotate_thr_deg:
        return SegmentCommand(
            turn_deg=abs_angle,
            turn_clockwise=clockwise,
            move_cm=min(distance_cm, move_cap * config.smooth_turn_move_frac),
        )
    return SegmentCommand(
        turn_deg=0.0,
        turn_clockwise=1,
        move_cm=min(distance_cm, move_cap),
    )
