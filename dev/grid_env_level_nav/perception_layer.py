#!/usr/bin/env python3
"""L2_depth egocentric perception: depth FOV → costmap cells via log-odds."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Set, Tuple

import numpy as np

from costmap_layers import (
    L2_LOG_ODDS_HIT,
    L2_LOG_ODDS_MISS,
    LayeredCostmap,
)
from level_coords import FLOOR_REF_Z_CM
from work_region import world_xy_to_cell

WorldXY = Tuple[float, float]
GridCell = Tuple[int, int]

L2_OBSTACLE_COST = 80.0
L2_LETHAL_COST = 1.0e9
DEFAULT_FOV_DEG = 90.0
DEFAULT_MAX_RANGE_CM = 800.0
DEFAULT_MIN_HEIGHT_CM = 25.0
DEFAULT_STRIDE_PX = 4
DEFAULT_MIN_DEPTH_CLEARANCE_CM = 100.0
DEFAULT_ROBOT_BODY_CLEARANCE_CM = 45.0
DEFAULT_SELF_EXCLUDE_RADIUS_CM = 70.0
DEFAULT_CAMERA_PITCH_DEG = 0.0


@dataclass(frozen=True)
class EgocentricPerceptionConfig:
    fov_deg: float = DEFAULT_FOV_DEG
    max_range_cm: float = DEFAULT_MAX_RANGE_CM
    min_obstacle_height_cm: float = DEFAULT_MIN_HEIGHT_CM
    stride_px: int = DEFAULT_STRIDE_PX
    use_lethal: bool = False
    camera_offset_forward_cm: float = 40.0
    camera_height_cm: float = 60.0
    camera_pitch_deg: float = DEFAULT_CAMERA_PITCH_DEG
    self_exclude_radius_cm: float = DEFAULT_SELF_EXCLUDE_RADIUS_CM
    use_log_odds: bool = True
    latch_static: bool = True


@dataclass(frozen=True)
class DepthHit:
    world_xy: WorldXY
    world_z_cm: float
    distance_cm: float
    cell: GridCell


def _normalize_angle(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def bresenham_line(gx0: int, gy0: int, gx1: int, gy1: int) -> List[GridCell]:
    """Integer grid line from (gx0,gy0) to (gx1,gy1) inclusive."""
    cells: List[GridCell] = []
    dx = abs(gx1 - gx0)
    dy = abs(gy1 - gy0)
    sx = 1 if gx0 < gx1 else -1
    sy = 1 if gy0 < gy1 else -1
    err = dx - dy
    x, y = gx0, gy0
    while True:
        cells.append((x, y))
        if x == gx1 and y == gy1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells


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
    camera_pitch_deg: float = DEFAULT_CAMERA_PITCH_DEG,
) -> List[Tuple[float, float, float]]:
    """Pinhole inverse projection (pitch-aware): depth image → world (x,y,z) hit points."""
    if depth_m.ndim != 2:
        return []
    h, w = depth_m.shape
    if h < 2 or w < 2:
        return []
    yaw_rad = math.radians(robot_yaw_deg)
    pitch_rad = math.radians(camera_pitch_deg)
    cos_pitch = math.cos(pitch_rad)
    sin_pitch = math.sin(pitch_rad)
    fx = (w / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    cx = (w - 1) / 2.0
    cy_center = (h - 1) / 2.0
    cam_x = robot_xy[0] + camera_offset_forward_cm * math.cos(yaw_rad)
    cam_y = robot_xy[1] + camera_offset_forward_cm * math.sin(yaw_rad)
    cam_z = floor_z_cm + camera_height_cm
    points: List[Tuple[float, float, float]] = []
    for v in range(0, h, max(1, stride_px)):
        for u in range(0, w, max(1, stride_px)):
            d_m = float(depth_m[v, u])
            if not math.isfinite(d_m) or d_m <= 0.05 or d_m > 80.0:
                continue
            d_cm = d_m * 100.0
            x_cam = (u - cx) * d_cm / fx
            y_cam = d_cm
            z_cam = -(v - cy_center) * d_cm / fx
            y_fwd = y_cam * cos_pitch - z_cam * sin_pitch
            z_up = y_cam * sin_pitch + z_cam * cos_pitch
            wx = cam_x + y_fwd * math.cos(yaw_rad) - x_cam * math.sin(yaw_rad)
            wy = cam_y + y_fwd * math.sin(yaw_rad) + x_cam * math.cos(yaw_rad)
            wz = cam_z + z_up
            points.append((wx, wy, wz))
    return points


def depth_hits_from_image(
    depth_m: np.ndarray,
    layers: LayeredCostmap,
    *,
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    config: Optional[EgocentricPerceptionConfig] = None,
    floor_z_cm: float = FLOOR_REF_Z_CM,
) -> List[DepthHit]:
    """Return obstacle hit points above floor threshold within robot FOV."""
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
        camera_pitch_deg=cfg.camera_pitch_deg,
    )
    hits: List[DepthHit] = []
    seen: Set[GridCell] = set()
    for wx, wy, wz in points:
        dist_cm = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
        if dist_cm > cfg.max_range_cm:
            continue
        if dist_cm <= cfg.self_exclude_radius_cm:
            continue
        height_above_floor = wz - floor_z_cm
        if height_above_floor < cfg.min_obstacle_height_cm:
            continue
        cell = world_xy_to_cell(wx, wy, layers.resolution_cm, clamp=True)
        if cell is None or cell in seen:
            continue
        seen.add(cell)
        hits.append(
            DepthHit(
                world_xy=(wx, wy),
                world_z_cm=wz,
                distance_cm=dist_cm,
                cell=cell,
            )
        )
    return hits


def obstacle_cells_from_depth(
    depth_m: np.ndarray,
    layers: LayeredCostmap,
    *,
    robot_xy: WorldXY,
    robot_yaw_deg: float,
    config: Optional[EgocentricPerceptionConfig] = None,
    floor_z_cm: float = FLOOR_REF_Z_CM,
) -> List[GridCell]:
    """Return grid cells hit by obstacles above floor threshold in robot FOV."""
    return [hit.cell for hit in depth_hits_from_image(
        depth_m,
        layers,
        robot_xy=robot_xy,
        robot_yaw_deg=robot_yaw_deg,
        config=config,
        floor_z_cm=floor_z_cm,
    )]


def _robot_cell(layers: LayeredCostmap, robot_xy: WorldXY) -> Optional[GridCell]:
    return world_xy_to_cell(robot_xy[0], robot_xy[1], layers.resolution_cm, clamp=True)


def apply_depth_ray_update(
    layers: LayeredCostmap,
    hits: Sequence[DepthHit],
    *,
    robot_xy: WorldXY,
    config: Optional[EgocentricPerceptionConfig] = None,
    preserve_cells: Optional[Set[GridCell]] = None,
) -> Tuple[int, int]:
    """Ray-clear free space + log-odds hit/miss updates. Returns (hits, clears)."""
    cfg = config or EgocentricPerceptionConfig()
    preserve = preserve_cells or set()
    robot_cell = _robot_cell(layers, robot_xy)
    if robot_cell is None:
        return 0, 0
    hit_cells: Set[GridCell] = {hit.cell for hit in hits}
    cleared: Set[GridCell] = set()
    hit_count = 0

    for hit in hits:
        gx, gy = hit.cell
        for ray_cell in bresenham_line(robot_cell[0], robot_cell[1], gx, gy):
            if ray_cell == hit.cell:
                break
            if ray_cell in preserve or ray_cell in hit_cells:
                continue
            if ray_cell in cleared:
                continue
            if cfg.use_log_odds:
                layers.update_l2_log_odds_cell(
                    ray_cell[0], ray_cell[1], L2_LOG_ODDS_MISS, latch_static=False
                )
            else:
                layers.clear_l2_cell(ray_cell[0], ray_cell[1])
            cleared.add(ray_cell)

    for hit in hits:
        gx, gy = hit.cell
        if cfg.use_log_odds:
            layers.update_l2_log_odds_cell(
                gx, gy, L2_LOG_ODDS_HIT, latch_static=cfg.latch_static
            )
        else:
            cost = L2_LETHAL_COST if cfg.use_lethal else L2_OBSTACLE_COST
            layers.set_l2_cell(gx, gy, cost)
        hit_count += 1

    return hit_count, len(cleared)


def _dilate_cells(
    cells: Sequence[GridCell],
    layers: LayeredCostmap,
    *,
    radius_cells: int,
) -> List[GridCell]:
    if radius_cells <= 0:
        return list(dict.fromkeys(cells))
    out: List[GridCell] = []
    seen: Set[GridCell] = set()
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
) -> List[GridCell]:
    """Inflated L2 cells for depth obstacles closer than the requested clearance."""
    cfg = config or EgocentricPerceptionConfig()
    if camera_pitch_deg != 0.0:
        cfg = EgocentricPerceptionConfig(
            fov_deg=cfg.fov_deg,
            max_range_cm=cfg.max_range_cm,
            min_obstacle_height_cm=cfg.min_obstacle_height_cm,
            stride_px=cfg.stride_px,
            use_lethal=cfg.use_lethal,
            camera_offset_forward_cm=cfg.camera_offset_forward_cm,
            camera_height_cm=cfg.camera_height_cm,
            camera_pitch_deg=camera_pitch_deg,
            self_exclude_radius_cm=cfg.self_exclude_radius_cm,
            use_log_odds=cfg.use_log_odds,
            latch_static=cfg.latch_static,
        )
    obstacle_cells: list[GridCell] = []
    seen: Set[GridCell] = set()
    if depth_m.ndim != 2:
        return []
    h, w = depth_m.shape
    if h < 2 or w < 2:
        return []
    yaw_rad = math.radians(robot_yaw_deg)
    pitch_rad = math.radians(cfg.camera_pitch_deg)
    cos_pitch = math.cos(pitch_rad)
    sin_pitch = math.sin(pitch_rad)
    fx = (w / 2.0) / math.tan(math.radians(cfg.fov_deg) / 2.0)
    cx = (w - 1) / 2.0
    cy_center = (h - 1) / 2.0
    cam_x = robot_xy[0] + cfg.camera_offset_forward_cm * math.cos(yaw_rad)
    cam_y = robot_xy[1] + cfg.camera_offset_forward_cm * math.sin(yaw_rad)
    # Skip bottom band — floor dominates depth there and causes false keepout.
    max_v = max(1, int(h * 0.82))
    for v in range(0, max_v, max(1, cfg.stride_px)):
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
            y_fwd = y_cam * cos_pitch - z_cam * sin_pitch
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
    cells: Sequence[GridCell],
    *,
    config: Optional[EgocentricPerceptionConfig] = None,
    latch_static: Optional[bool] = None,
) -> int:
    cfg = config or EgocentricPerceptionConfig()
    latch = cfg.latch_static if latch_static is None else latch_static
    cost = L2_LETHAL_COST if cfg.use_lethal else L2_OBSTACLE_COST
    n = 0
    for gx, gy in cells:
        if cfg.use_log_odds:
            layers.update_l2_log_odds_cell(gx, gy, L2_LOG_ODDS_HIT, latch_static=latch)
        else:
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
    preserve_cells: Optional[Set[GridCell]] = None,
) -> int:
    """Full FOV depth → L2_depth with ray clearing and optional carry-forward mask."""
    cfg = config or EgocentricPerceptionConfig()
    hits = depth_hits_from_image(
        depth_m,
        layers,
        robot_xy=robot_xy,
        robot_yaw_deg=robot_yaw_deg,
        config=cfg,
    )
    hit_count, _cleared = apply_depth_ray_update(
        layers,
        hits,
        robot_xy=robot_xy,
        config=cfg,
        preserve_cells=preserve_cells,
    )
    return hit_count


def min_forward_depth_m(
    depth_m: np.ndarray,
    *,
    fov_deg: float = DEFAULT_FOV_DEG,
    cone_half_deg: float = 22.0,
) -> Optional[float]:
    """Minimum valid slant-range depth (m) in the forward cone (lower image band)."""
    if depth_m.ndim != 2:
        return None
    h, w = depth_m.shape
    if h < 2 or w < 2:
        return None
    cx = (w - 1) / 2.0
    half_fov_rad = math.radians(fov_deg / 2.0)
    if half_fov_rad <= 1e-6:
        return None
    half_width_px = (w / 2.0) * math.tan(math.radians(cone_half_deg)) / math.tan(half_fov_rad)
    u0 = max(0, int(cx - half_width_px))
    u1 = min(w, int(cx + half_width_px) + 1)
    v0 = h // 3
    v1 = max(v0 + 1, int(h * 0.82))
    roi = depth_m[v0:v1, u0:u1]
    valid = roi[np.isfinite(roi) & (roi > 0.05) & (roi < 80.0)]
    if valid.size == 0:
        return None
    return float(np.min(valid))


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
        if arr.max() > 50.0:
            arr = arr / 255.0 * 20.0
        return arr
    except Exception:
        return None
