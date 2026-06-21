#!/usr/bin/env python3
"""Ground-truth distance and bearing from robot pose to registered props."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from prop_placement import PlacementRegistry, PropPlacement

WorldXY = Tuple[float, float]


def normalize_angle_deg(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def yaw_to_target_deg(from_xy: WorldXY, to_xy: WorldXY) -> float:
    return math.degrees(math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0]))


def bearing_relative_to_forward_deg(
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    target_xy: WorldXY,
) -> float:
    """Bearing to target relative to robot forward (+X at yaw=0 convention)."""
    target_yaw = yaw_to_target_deg(robot_xy, target_xy)
    return normalize_angle_deg(target_yaw - robot_yaw_deg)


def horizontal_distance_m(robot_xy: WorldXY, target_xy: WorldXY) -> float:
    return math.hypot(target_xy[0] - robot_xy[0], target_xy[1] - robot_xy[1]) / 100.0


@dataclass(frozen=True)
class GroundTruthObservation:
    prop_type_id: str
    slot_id: str
    distance_m: float
    bearing_deg: float
    in_fov: bool


def ground_truth_for_prop(
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    prop: PropPlacement,
    *,
    fov_deg: float,
) -> GroundTruthObservation:
    if prop.world_xyz_cm is None:
        raise ValueError(f"prop {prop.slot_id} has no world_xyz_cm in registry")
    target_xy = (prop.world_xyz_cm[0], prop.world_xyz_cm[1])
    bearing = bearing_relative_to_forward_deg(robot_xy, robot_yaw_deg, target_xy)
    half_fov = fov_deg * 0.5
    in_fov = abs(bearing) <= half_fov
    return GroundTruthObservation(
        prop_type_id=prop.prop_type_id,
        slot_id=prop.slot_id,
        distance_m=horizontal_distance_m(robot_xy, target_xy),
        bearing_deg=bearing,
        in_fov=in_fov,
    )


def ground_truth_all_props(
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    registry: PlacementRegistry,
    *,
    fov_deg: float,
) -> Dict[str, GroundTruthObservation]:
    out: Dict[str, GroundTruthObservation] = {}
    for prop in registry.props:
        out[prop.prop_type_id] = ground_truth_for_prop(
            robot_xy,
            robot_yaw_deg,
            prop,
            fov_deg=fov_deg,
        )
    return out


def rmse(values_a: list[float], values_b: list[float]) -> Optional[float]:
    if not values_a or len(values_a) != len(values_b):
        return None
    sq = 0.0
    for a, b in zip(values_a, values_b):
        d = a - b
        sq += d * d
    return math.sqrt(sq / len(values_a))
