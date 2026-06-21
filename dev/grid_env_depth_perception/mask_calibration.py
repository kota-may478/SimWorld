#!/usr/bin/env python3
"""Calibrate object_mask colors after spawn (UnrealCV returns BGR)."""

from __future__ import annotations

import math
from collections import Counter
from typing import List, Optional, Tuple

import numpy as np

from pie_safety import (
    POST_TELEPORT_SETTLE_S,
    require_live_ucv,
    soft_teleport_robot,
    tick_settle,
)
from prop_placement import PlacementRegistry, PropPlacement, _copy_prop, save_registry
from robot_sensor import update_sensor_camera_pose

WorldXY = Tuple[float, float]
STANDOFF_CM = 450.0
MIN_COLOR_PIXELS = 80
CALIB_SETTLE_S = 0.35


def _yaw_toward(from_xy: WorldXY, to_xy: WorldXY) -> float:
    return math.degrees(math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0]))


def _dominant_bgr_in_center(mask_bgr: np.ndarray) -> Optional[Tuple[int, int, int]]:
    h, w = mask_bgr.shape[:2]
    roi = mask_bgr[h // 4 : 3 * h // 4, w // 4 : 3 * w // 4]
    if roi.size == 0:
        return None
    flat = roi.reshape(-1, 3)
    sums = flat.sum(axis=1)
    valid = flat[sums > 40]
    if valid.size == 0:
        return None
    quant = (valid // 8) * 8
    keys = [tuple(int(v) for v in row) for row in quant]
    color, count = Counter(keys).most_common(1)[0]
    if count < MIN_COLOR_PIXELS:
        return None
    return color  # type: ignore[return-value]


def _standoff_pose(prop: PropPlacement) -> Tuple[Tuple[float, float, float], float]:
    if prop.world_xyz_cm is None:
        raise ValueError(f"{prop.slot_id} missing world pose")
    target = (prop.world_xyz_cm[0], prop.world_xyz_cm[1])
    spawn_hint = (target[0] - 200.0, target[1] - 200.0)
    yaw = _yaw_toward(spawn_hint, target)
    rad = math.radians(yaw)
    standoff = (
        target[0] - STANDOFF_CM * math.cos(rad),
        target[1] - STANDOFF_CM * math.sin(rad),
        prop.world_xyz_cm[2],
    )
    return standoff, yaw


def calibrate_mask_colors(
    ucv,
    communicator,
    camera_id: int,
    robot_name: str,
    registry: PlacementRegistry,
) -> PlacementRegistry:
    from robot_sensor import fetch_mask_rgb

    updated: List[PropPlacement] = []
    for prop in registry.props:
        require_live_ucv(ucv, context=f"mask calib {prop.slot_id}")
        if prop.world_xyz_cm is None:
            updated.append(prop)
            continue
        loc, yaw = _standoff_pose(prop)
        soft_teleport_robot(ucv, robot_name, loc, yaw)
        tick_settle(ucv, settle_s=CALIB_SETTLE_S, ticks=2)
        update_sensor_camera_pose(ucv, robot_name, camera_id)
        tick_settle(ucv, settle_s=POST_TELEPORT_SETTLE_S, ticks=1)
        mask = fetch_mask_rgb(communicator, camera_id)
        if mask is None:
            print(f"[MaskCalib] WARN: no mask for {prop.slot_id}")
            updated.append(prop)
            continue
        bgr = _dominant_bgr_in_center(mask)
        if bgr is None:
            print(f"[MaskCalib] WARN: no dominant color for {prop.slot_id}")
            updated.append(prop)
            continue
        set_rgb = prop.mask_color_set_rgb or prop.mask_color_rgb
        print(
            f"[MaskCalib] {prop.prop_type_id} observed BGR={bgr} "
            f"(set RGB={set_rgb})"
        )
        updated.append(
            _copy_prop(
                prop,
                mask_color_observed_bgr=bgr,
                mask_color_rgb=bgr[::-1],
            )
        )
    out = PlacementRegistry(
        version=registry.version,
        seed=registry.seed,
        prop_count=registry.prop_count,
        region_x_max_cm=registry.region_x_max_cm,
        region_y_max_cm=registry.region_y_max_cm,
        exclusion_cm=registry.exclusion_cm,
        spotdog_spawn_local_cm=registry.spotdog_spawn_local_cm,
        props=tuple(updated),
    )
    save_registry(out)
    return out
