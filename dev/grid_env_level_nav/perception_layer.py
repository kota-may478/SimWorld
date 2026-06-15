#!/usr/bin/env python3
"""L2 egocentric perception: depth FOV → costmap cells (MVP)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from costmap_layers import LayeredCostmap
from level_coords import FLOOR_REF_Z_CM
from work_region import world_xy_to_cell

WorldXY = Tuple[float, float]

L2_OBSTACLE_COST = 80.0
L2_LETHAL_COST = 1.0e9
DEFAULT_FOV_DEG = 90.0
DEFAULT_MAX_RANGE_CM = 800.0
DEFAULT_MIN_HEIGHT_CM = 25.0
DEFAULT_STRIDE_PX = 4


@dataclass(frozen=True)
class EgocentricPerceptionConfig:
    fov_deg: float = DEFAULT_FOV_DEG
    max_range_cm: float = DEFAULT_MAX_RANGE_CM
    min_obstacle_height_cm: float = DEFAULT_MIN_HEIGHT_CM
    stride_px: int = DEFAULT_STRIDE_PX
    use_lethal: bool = False
    camera_offset_forward_cm: float = 40.0
    camera_height_cm: float = 60.0


def _normalize_angle(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def depth_m_to_world_points(
    depth_m: np.ndarray,
    *,
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    fov_deg: float,
    camera_offset_forward_cm: float,
    camera_height_cm: float,
    floor_z_cm: float = FLOOR_REF_Z_CM,
    stride_px: int = DEFAULT_STRIDE_PX,
) -> List[Tuple[float, float, float]]:
    """Pinhole inverse projection: depth image → world (x,y,z) hit points."""
    if depth_m.ndim != 2:
        return []
    h, w = depth_m.shape
    if h < 2 or w < 2:
        return []
    yaw_rad = math.radians(robot_yaw_deg)
    fx = (w / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    cx = (w - 1) / 2.0
    cam_x = robot_xy[0] + camera_offset_forward_cm * math.cos(yaw_rad)
    cam_y = robot_xy[1] + camera_offset_forward_cm * math.sin(yaw_rad)
    cam_z = floor_z_cm + camera_height_cm
    points: List[Tuple[float, float, float]] = []
    for v in range(0, h, max(1, stride_px)):
        for u in range(0, w, max(1, stride_px)):
            d_m = float(depth_m[v, u])
            if not math.isfinite(d_m) or d_m <= 0.05:
                continue
            d_cm = d_m * 100.0
            x_cam = (u - cx) * d_cm / fx
            y_cam = d_cm
            z_cam = -(v - (h - 1) / 2.0) * d_cm / fx
            # camera frame: +Y forward, +X right, +Z up → world yaw rotate
            wx = cam_x + y_cam * math.cos(yaw_rad) - x_cam * math.sin(yaw_rad)
            wy = cam_y + y_cam * math.sin(yaw_rad) + x_cam * math.cos(yaw_rad)
            wz = cam_z + z_cam
            points.append((wx, wy, wz))
    return points


def obstacle_cells_from_depth(
    depth_m: np.ndarray,
    layers: LayeredCostmap,
    *,
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    config: Optional[EgocentricPerceptionConfig] = None,
    floor_z_cm: float = FLOOR_REF_Z_CM,
) -> List[Tuple[int, int]]:
    """Return grid cells hit by obstacles above floor threshold in robot FOV."""
    cfg = config or EgocentricPerceptionConfig()
    points = depth_m_to_world_points(
        depth_m,
        robot_xy=robot_xy,
        robot_yaw_deg=robot_yaw_deg,
        fov_deg=cfg.fov_deg,
        camera_offset_forward_cm=cfg.camera_offset_forward_cm,
        camera_height_cm=cfg.camera_height_cm,
        floor_z_cm=floor_z_cm,
        stride_px=cfg.stride_px,
    )
    cells: List[Tuple[int, int]] = []
    seen: set[Tuple[int, int]] = set()
    for wx, wy, wz in points:
        if math.hypot(wx - robot_xy[0], wy - robot_xy[1]) > cfg.max_range_cm:
            continue
        height_above_floor = wz - floor_z_cm
        if height_above_floor < cfg.min_obstacle_height_cm:
            continue
        cell = world_xy_to_cell(wx, wy, layers.resolution_cm, clamp=True)
        if cell is None or cell in seen:
            continue
        seen.add(cell)
        cells.append(cell)
    return cells


def apply_l2_obstacle_cells(
    layers: LayeredCostmap,
    cells: Sequence[Tuple[int, int]],
    *,
    config: Optional[EgocentricPerceptionConfig] = None,
) -> int:
    cfg = config or EgocentricPerceptionConfig()
    cost = L2_LETHAL_COST if cfg.use_lethal else L2_OBSTACLE_COST
    n = 0
    for gx, gy in cells:
        layers.set_l2_cell(gx, gy, cost)
        n += 1
    return n


def update_l2_from_depth_image(
    depth_m: np.ndarray,
    layers: LayeredCostmap,
    *,
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    config: Optional[EgocentricPerceptionConfig] = None,
) -> int:
    cells = obstacle_cells_from_depth(
        depth_m,
        layers,
        robot_xy=robot_xy,
        robot_yaw_deg=robot_yaw_deg,
        config=config,
    )
    return apply_l2_obstacle_cells(layers, cells, config=config)


def fetch_robot_depth_m(ucv, robot_name: str, camera_id: int = 0) -> Optional[np.ndarray]:
    """Best-effort depth capture from UnrealCV (returns meters, HxW)."""
    try:
        raw = ucv.get_image(camera_id, "depth", "png")
    except Exception:
        return None
    if raw is None:
        return None
    try:
        from io import BytesIO
        from PIL import Image

        img = Image.open(BytesIO(raw))
        arr = np.asarray(img, dtype=np.float32)
        if arr.ndim == 3:
            arr = arr[..., 0]
        # UE depth PNG often encodes linear depth; scale heuristic if values look like 0-255
        if arr.max() > 50.0:
            arr = arr / 255.0 * 20.0
        return arr
    except Exception:
        return None
