#!/usr/bin/env python3
"""Geometric L2 perception: FOV cone + range check against placement registry (no UE camera)."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

import level_coords as lc  # noqa: E402
from depth_object_perception import ObjectEstimate  # noqa: E402
from grid_env_10k_pie_patrol import dist2d, normalize_angle, yaw_to_target  # noqa: E402
from prop_placement import PlacementRegistry, PropPlacement  # noqa: E402
from robot_sensor import (  # noqa: E402
    SENSOR_CAM_FORWARD_OFFSET_CM,
    SENSOR_FOV_DEG,
)
from zones import ROADBLOCK_BP_NAME  # noqa: E402

WorldXY = Tuple[float, float]


@dataclass(frozen=True)
class GeomPerceptionConfig:
    fov_deg: float = SENSOR_FOV_DEG
    max_range_cm: float = 650.0
    sensor_forward_cm: float = SENSOR_CAM_FORWARD_OFFSET_CM
    min_confidence: float = 0.3


def _prop_world_xy(prop: PropPlacement) -> WorldXY:
    if prop.world_xyz_cm is not None:
        return float(prop.world_xyz_cm[0]), float(prop.world_xyz_cm[1])
    return lc.local_xy_to_world(*prop.local_xy_cm)


def _sensor_origin_xy(robot_xy: WorldXY, robot_yaw_deg: float, forward_cm: float) -> WorldXY:
    yaw_rad = math.radians(robot_yaw_deg)
    return (
        robot_xy[0] + forward_cm * math.cos(yaw_rad),
        robot_xy[1] + forward_cm * math.sin(yaw_rad),
    )


def _bearing_deg_robot_frame(
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    target_xy: WorldXY,
    *,
    sensor_forward_cm: float,
) -> float:
    sensor_xy = _sensor_origin_xy(robot_xy, robot_yaw_deg, sensor_forward_cm)
    world_bearing = yaw_to_target(sensor_xy, target_xy)
    return normalize_angle(world_bearing - robot_yaw_deg)


def _in_fov_cone(bearing_deg: float, fov_deg: float) -> bool:
    return abs(bearing_deg) <= (fov_deg * 0.5) + 1e-6


def _is_l2_candidate(prop: PropPlacement) -> bool:
    if prop.bp_name == ROADBLOCK_BP_NAME:
        return False
    if prop.prop_type_id.startswith("roadblock"):
        return False
    return True


def visible_props_from_geom(
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    placement_reg: PlacementRegistry,
    *,
    config: Optional[GeomPerceptionConfig] = None,
) -> List[Tuple[PropPlacement, float, float]]:
    """Return visible props as (prop, distance_m, bearing_deg) sorted by distance."""
    cfg = config or GeomPerceptionConfig()
    visible: List[Tuple[PropPlacement, float, float]] = []
    for prop in placement_reg.props:
        if not _is_l2_candidate(prop):
            continue
        prop_xy = _prop_world_xy(prop)
        dist_cm = dist2d(robot_xy, prop_xy)
        if dist_cm > cfg.max_range_cm:
            continue
        bearing = _bearing_deg_robot_frame(
            robot_xy,
            robot_yaw_deg,
            prop_xy,
            sensor_forward_cm=cfg.sensor_forward_cm,
        )
        if not _in_fov_cone(bearing, cfg.fov_deg):
            continue
        visible.append((prop, dist_cm / 100.0, bearing))
    visible.sort(key=lambda item: item[1])
    return visible


def geom_detections(
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    placement_reg: PlacementRegistry,
    *,
    config: Optional[GeomPerceptionConfig] = None,
) -> List[ObjectEstimate]:
    cfg = config or GeomPerceptionConfig()
    estimates: List[ObjectEstimate] = []
    for prop, distance_m, bearing_deg in visible_props_from_geom(
        robot_xy, robot_yaw_deg, placement_reg, config=cfg
    ):
        confidence = max(
            cfg.min_confidence,
            min(1.0, 1.0 - (distance_m * 100.0) / max(cfg.max_range_cm, 1.0)),
        )
        estimates.append(
            ObjectEstimate(
                prop_type_id=prop.prop_type_id,
                slot_id=prop.slot_id,
                distance_m=distance_m,
                bearing_deg=bearing_deg,
                mask_pixels=100,
                confidence=confidence,
            )
        )
    return estimates
