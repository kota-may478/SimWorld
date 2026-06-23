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
DEFAULT_MIN_DEPTH_CLEARANCE_CM = 100.0
DEFAULT_ROBOT_BODY_CLEARANCE_CM = 45.0


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


def _dilate_cells(
    cells: Sequence[Tuple[int, int]],
    layers: LayeredCostmap,
    *,
    radius_cells: int,
) -> List[Tuple[int, int]]:
    if radius_cells <= 0:
        return list(dict.fromkeys(cells))
    out: List[Tuple[int, int]] = []
    seen: set[Tuple[int, int]] = set()
    for cx, cy in cells:
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if math.hypot(dx, dy) > radius_cells + 0.25:
                    continue
                gx, gy = cx + dx, cy + dy
                if gx < 0 or gy < 0 or gx >= layers.width_cells or gy >= layers.height_cells:
                    continue
                cell = (gx, gy)
                if cell in seen:
                    continue
                seen.add(cell)
                out.append(cell)
    return out


def obstacle_cells_from_depth_gated_by_detections(
    depth_m: np.ndarray,
    layers: LayeredCostmap,
    *,
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    detections: Sequence[object],
    config: Optional[EgocentricPerceptionConfig] = None,
    bearing_margin_deg: float = 12.0,
    depth_band_cm: float = 140.0,
    dilate_cells: int = 1,
    floor_z_cm: float = FLOOR_REF_Z_CM,
) -> List[Tuple[int, int]]:
    """Return depth obstacle cells only in sectors confirmed by object detections."""
    if not detections:
        return []
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
    candidates: list[tuple[Tuple[int, int], float, float]] = []
    for wx, wy, wz in points:
        dist_cm = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
        if dist_cm > cfg.max_range_cm:
            continue
        if wz - floor_z_cm < cfg.min_obstacle_height_cm:
            continue
        bearing = _normalize_angle(math.degrees(math.atan2(wy - robot_xy[1], wx - robot_xy[0])) - robot_yaw_deg)
        cell = world_xy_to_cell(wx, wy, layers.resolution_cm, clamp=True)
        if cell is None:
            continue
        candidates.append((cell, bearing, dist_cm))

    matched: list[Tuple[int, int]] = []
    for det in detections:
        det_bearing = float(getattr(det, "bearing_deg", 0.0))
        det_dist_cm = max(0.0, float(getattr(det, "distance_m", 0.0)) * 100.0)
        sector = [
            item
            for item in candidates
            if abs(_normalize_angle(item[1] - det_bearing)) <= bearing_margin_deg
        ]
        if not sector:
            continue
        if det_dist_cm > 1.0:
            band = max(depth_band_cm, det_dist_cm * 0.35)
            ranged = [item for item in sector if abs(item[2] - det_dist_cm) <= band]
            if ranged:
                sector = ranged
        nearest_cm = min(item[2] for item in sector)
        for cell, _bearing, dist_cm in sector:
            if dist_cm <= nearest_cm + depth_band_cm:
                matched.append(cell)
    return _dilate_cells(matched, layers, radius_cells=dilate_cells)


def close_range_keepout_cells_from_depth(
    depth_m: np.ndarray,
    layers: LayeredCostmap,
    *,
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    config: Optional[EgocentricPerceptionConfig] = None,
    min_clearance_cm: float = DEFAULT_MIN_DEPTH_CLEARANCE_CM,
    keepout_radius_cm: float = DEFAULT_MIN_DEPTH_CLEARANCE_CM,
    robot_body_clearance_cm: float = DEFAULT_ROBOT_BODY_CLEARANCE_CM,
    camera_pitch_deg: float = 0.0,
    floor_z_cm: float = FLOOR_REF_Z_CM,
) -> List[Tuple[int, int]]:
    """Inflated L2 cells for depth obstacles closer than the requested clearance.

    Uses pitch-aware 3-D projection and a height threshold to distinguish
    floor hits from real obstacles, replacing the fixed row-cutoff heuristic.
    """
    cfg = config or EgocentricPerceptionConfig()
    obstacle_cells: list[Tuple[int, int]] = []
    seen: set[Tuple[int, int]] = set()
    if depth_m.ndim != 2:
        return []
    h, w = depth_m.shape
    if h < 2 or w < 2:
        return []
    yaw_rad = math.radians(robot_yaw_deg)
    pitch_rad = math.radians(camera_pitch_deg)
    cos_pitch = math.cos(pitch_rad)
    sin_pitch = math.sin(pitch_rad)
    fx = (w / 2.0) / math.tan(math.radians(cfg.fov_deg) / 2.0)
    cx = (w - 1) / 2.0
    cy_center = (h - 1) / 2.0
    cam_x = robot_xy[0] + cfg.camera_offset_forward_cm * math.cos(yaw_rad)
    cam_y = robot_xy[1] + cfg.camera_offset_forward_cm * math.sin(yaw_rad)
    cam_z = floor_z_cm + cfg.camera_height_cm
    for v in range(0, h, max(1, cfg.stride_px)):
        for u in range(0, w, max(1, cfg.stride_px)):
            d_m = float(depth_m[v, u])
            if not math.isfinite(d_m) or d_m <= 0.05:
                continue
            d_cm = d_m * 100.0
            if d_cm > min_clearance_cm:
                continue
            x_cam = (u - cx) * d_cm / fx
            y_cam = d_cm
            z_cam = -(v - cy_center) * d_cm / fx
            # Apply camera pitch (rotation about camera X axis).
            y_fwd = y_cam * cos_pitch - z_cam * sin_pitch
            z_up = y_cam * sin_pitch + z_cam * cos_pitch
            # World height of hit point — skip floor/ground returns.
            wz = cam_z + z_up
            if wz - floor_z_cm < cfg.min_obstacle_height_cm:
                continue
            wx = cam_x + y_fwd * math.cos(yaw_rad) - x_cam * math.sin(yaw_rad)
            wy = cam_y + y_fwd * math.sin(yaw_rad) + x_cam * math.cos(yaw_rad)
            cell = world_xy_to_cell(wx, wy, layers.resolution_cm, clamp=True)
            if cell is None or cell in seen:
                continue
            seen.add(cell)
            obstacle_cells.append(cell)

    if not obstacle_cells:
        return []

    radius_cells = max(1, int(math.ceil(keepout_radius_cm / layers.resolution_cm)))
    inflated = _dilate_cells(obstacle_cells, layers, radius_cells=radius_cells)
    return [
        cell
        for cell in inflated
        if math.hypot(
            layers.origin_xy[0] + (cell[0] + 0.5) * layers.resolution_cm - robot_xy[0],
            layers.origin_xy[1] + (cell[1] + 0.5) * layers.resolution_cm - robot_xy[1],
        )
        > robot_body_clearance_cm
    ]


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
