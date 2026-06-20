#!/usr/bin/env python3
"""Apply FusionCam object_mask detections to L2 costmap cells."""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Set, Tuple

from costmap_layers import LayeredCostmap
from perception_layer import EgocentricPerceptionConfig, apply_l2_obstacle_cells
from work_region import world_xy_to_cell

WorldXY = Tuple[float, float]
L2_PROP_RADIUS_CM = 90.0


def estimate_world_xy_from_detection(
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    *,
    distance_m: float,
    bearing_deg: float,
    camera_offset_forward_cm: float = 22.0,
) -> WorldXY:
    yaw_rad = math.radians(robot_yaw_deg + bearing_deg)
    forward_cm = distance_m * 100.0 + camera_offset_forward_cm * math.cos(math.radians(bearing_deg))
    wx = robot_xy[0] + forward_cm * math.cos(yaw_rad)
    wy = robot_xy[1] + forward_cm * math.sin(yaw_rad)
    return wx, wy


def l2_cells_for_world_disk(
    layers: LayeredCostmap,
    center_xy: WorldXY,
    *,
    radius_cm: float = L2_PROP_RADIUS_CM,
) -> List[Tuple[int, int]]:
    res = layers.resolution_cm
    gx0, gy0 = world_xy_to_cell(center_xy[0], center_xy[1], res, clamp=True)
    if gx0 is None or gy0 is None:
        return []
    cell_r = max(1, int(math.ceil(radius_cm / res)))
    cells: List[Tuple[int, int]] = []
    for dgy in range(-cell_r, cell_r + 1):
        for dgx in range(-cell_r, cell_r + 1):
            gx, gy = gx0 + dgx, gy0 + dgy
            if 0 <= gx < layers.width_cells and 0 <= gy < layers.height_cells:
                cells.append((gx, gy))
    return cells


def apply_l2_from_fusion_detections(
    layers: LayeredCostmap,
    detections: Sequence[object],
    *,
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    known_cells: Optional[Set[Tuple[int, int]]] = None,
    camera_offset_forward_cm: float = 22.0,
) -> int:
    seen = known_cells if known_cells is not None else set()
    new_cells: List[Tuple[int, int]] = []
    for det in detections:
        prop_type = getattr(det, "prop_type_id", None)
        if not prop_type:
            continue
        wx, wy = estimate_world_xy_from_detection(
            robot_xy,
            robot_yaw_deg,
            distance_m=float(det.distance_m),
            bearing_deg=float(det.bearing_deg),
            camera_offset_forward_cm=camera_offset_forward_cm,
        )
        for cell in l2_cells_for_world_disk(layers, (wx, wy)):
            if cell in seen:
                continue
            seen.add(cell)
            new_cells.append(cell)
    if not new_cells:
        return 0
    return apply_l2_obstacle_cells(
        layers,
        new_cells,
        config=EgocentricPerceptionConfig(use_lethal=True),
    )


def detections_summary(detections: Sequence[object]) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for det in detections:
        pid = str(getattr(det, "prop_type_id", ""))
        if pid:
            out[pid] = float(getattr(det, "distance_m", 0.0))
    return out
