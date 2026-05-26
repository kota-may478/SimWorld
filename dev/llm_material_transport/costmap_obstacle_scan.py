"""
UE 上の障害物（正方形の柱）を 2D コストマップへ反映する。

スポーン後、マップ中心真上からカメラ高度を上げ、30 m 四方が画角に収まる位置で
depth のみを取得。透視逆投影で各画素を世界格子へ落とし、正方形クラスタだけを採用。
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import binary_dilation, label, maximum_filter

from path_planning_costmap import (
    COSTMAP_LETHAL_COST,
    Costmap2D,
    GridCell,
    WorldXY,
)

WorldXYZ = Tuple[float, float, float]
PillarObstacle = Tuple[float, float, float]  # x_cm, y_cm, half_extent_cm (square)


COSTMAP_PROBE_NAME = "MT_CostmapProbe"
COSTMAP_PROBE_BP_PATH = "/Game/CityDatabase/blueprints/BP_Box.BP_Box_C"
COSTMAP_PROBE_SCALE = (0.15, 0.15, 0.5)

OBSTACLE_SCAN_STRIDE_CELLS = 15
OBSTACLE_SCAN_CELL_COST = 80.0
OBSTACLE_SCAN_USE_LETHAL = False
OBSTACLE_SCAN_SETTLE_S = 0.02

# 真上 depth スキャン
TOPDOWN_CAMERA_ID = 0
TOPDOWN_CAM_PITCH_DEG = -90.0
TOPDOWN_CAM_YAW_DEG = 0.0
TOPDOWN_CAM_FOV_DEG = 90.0
TOPDOWN_CAPTURE_RESOLUTION = (512, 512)
TOPDOWN_CAPTURE_SETTLE_S = 0.4
TOPDOWN_MIN_VALID_DEPTH_M = 0.5

# カメラ高度探索: 低い位置から上げ、30 m 四方の四隅が画角内に入る最初の高度
TOPDOWN_HEIGHT_SEARCH_START_CM = 600.0
TOPDOWN_HEIGHT_SEARCH_STEP_CM = 150.0
TOPDOWN_HEIGHT_SEARCH_MAX_CM = 5500.0
TOPDOWN_FOV_EDGE_MARGIN_FRAC = 0.98
TOPDOWN_HEIGHT_SAFETY_FACTOR = 1.12

# depth → 柱（床との距離差）
TOPDOWN_PILLAR_DEPTH_MARGIN_M = 1.26
TOPDOWN_LOCAL_MAX_FILTER_PX = 31
TOPDOWN_DEPTH_VOTES_PER_CELL = 1

# 正方形柱フィルタ（格子単位）
TOPDOWN_SQUARE_MIN_AREA_CELLS = 20
TOPDOWN_SQUARE_MAX_AREA_CELLS = 8000
TOPDOWN_SQUARE_MIN_SIDE_CELLS = 4
TOPDOWN_SQUARE_MAX_SIDE_CELLS = 120
TOPDOWN_SQUARE_MIN_ASPECT = 0.55
TOPDOWN_SQUARE_MAX_ASPECT = 1.85
TOPDOWN_SQUARE_BBOX_PAD_CELLS = 1
TOPDOWN_AGENT_EXCLUDE_RADIUS_CM = 300.0

MANUAL_PILLAR_OBSTACLES_CM: List[PillarObstacle] = []

_LAST_TOPDOWN_CAPTURE: dict = {}


@dataclass(frozen=True)
class ObstacleScanResult:
    sampled_cells: int
    hit_cells: int
    manual_pillars: int
    inflated_regions: int
    scan_method: str = "topdown_depth_nadir"
    pillar_count: int = 0
    camera_height_cm: float = 0.0


def parse_collision_counts(raw_response: object) -> dict:
    if isinstance(raw_response, dict):
        return raw_response
    if isinstance(raw_response, str):
        text = raw_response.strip()
        if text.startswith("error"):
            return {}
        return json.loads(text)
    return {}


def collision_indicates_static_obstacle(counts: dict) -> bool:
    building = int(counts.get("BuildingCollision", 0) or 0)
    obj = int(counts.get("ObjectCollision", 0) or 0)
    return (building + obj) > 0


def _first_numeric_camera_id(ucv) -> int:
    tokens = str(ucv.get_cameras()).replace(",", " ").split()
    for token in tokens:
        try:
            return int(token)
        except ValueError:
            continue
    return TOPDOWN_CAMERA_ID


def _fetch_depth_npy(ucv, camera_id: int) -> np.ndarray:
    cmd = f"vget /camera/{camera_id}/depth npy"
    with ucv.lock:
        payload = ucv.client.request(cmd)
    depth = np.load(BytesIO(payload))
    if depth.ndim != 2:
        raise ValueError(f"expected 2D depth, got shape {depth.shape}")
    return depth.astype(np.float32, copy=False)


def _costmap_center_xy(costmap: Costmap2D) -> WorldXY:
    return (
        costmap.origin_xy[0] + costmap.size_x_cm * 0.5,
        costmap.origin_xy[1] + costmap.size_y_cm * 0.5,
    )


def _costmap_corners_xy(costmap: Costmap2D) -> List[WorldXY]:
    ox, oy = costmap.origin_xy
    sx, sy = costmap.size_x_cm, costmap.size_y_cm
    return [
        (ox, oy),
        (ox + sx, oy),
        (ox, oy + sy),
        (ox + sx, oy + sy),
    ]


def _focal_length_px(width_px: int, fov_deg: float) -> float:
    half_fov = math.radians(fov_deg * 0.5)
    return (width_px * 0.5) / max(math.tan(half_fov), 1e-3)


def _world_xy_to_pixel_nadir(
    wx: float,
    wy: float,
    *,
    cam_x: float,
    cam_y: float,
    cam_z: float,
    ground_z_cm: float,
    width_px: int,
    height_px: int,
    fov_deg: float,
) -> Tuple[int, int, bool]:
    """真下カメラ: 世界 XY [cm] → 画像ピクセル (u, v)。"""
    cam_h_m = max((cam_z - ground_z_cm) / 100.0, 0.5)
    dx_m = (wx - cam_x) / 100.0
    dy_m = (wy - cam_y) / 100.0
    focal = _focal_length_px(width_px, fov_deg)
    pu = int(round(width_px * 0.5 + (dx_m / cam_h_m) * focal))
    pv = int(round(height_px * 0.5 - (dy_m / cam_h_m) * focal))
    margin = int((1.0 - TOPDOWN_FOV_EDGE_MARGIN_FRAC) * min(width_px, height_px) * 0.5)
    inside = (
        pu >= margin
        and pu < width_px - margin
        and pv >= margin
        and pv < height_px - margin
    )
    return pu, pv, inside


def _analytic_min_camera_height_cm(costmap: Costmap2D, fov_deg: float) -> float:
    """四隅が画角に収まる高度の理論下限 [cm]（安全係数付き）。"""
    half_x = costmap.size_x_cm * 0.5
    half_y = costmap.size_y_cm * 0.5
    corner_horiz_m = math.hypot(half_x, half_y) / 100.0
    half_fov = math.radians(fov_deg * 0.5)
    min_h_m = corner_horiz_m / max(math.tan(half_fov), 1e-3)
    return min_h_m * 100.0 * TOPDOWN_HEIGHT_SAFETY_FACTOR


def _map_corners_fit_in_image(
    costmap: Costmap2D,
    *,
    cam_x: float,
    cam_y: float,
    cam_z: float,
    ground_z_cm: float,
    width_px: int,
    height_px: int,
    fov_deg: float,
) -> bool:
    for wx, wy in _costmap_corners_xy(costmap):
        _, _, inside = _world_xy_to_pixel_nadir(
            wx,
            wy,
            cam_x=cam_x,
            cam_y=cam_y,
            cam_z=cam_z,
            ground_z_cm=ground_z_cm,
            width_px=width_px,
            height_px=height_px,
            fov_deg=fov_deg,
        )
        if not inside:
            return False
    return True


def find_camera_height_for_full_map(
    ucv,
    costmap: Costmap2D,
    *,
    ground_z_cm: float,
    camera_id: int,
    width_px: int,
    height_px: int,
    fov_deg: float = TOPDOWN_CAM_FOV_DEG,
) -> float:
    """
    マップ中心真上・真下のまま高度を上げ、30 m 四方の四隅が画角内に入る最初の高度 [cm]。
    理論値から開始し、UE へは数回だけカメラを置いて確認する。
    """
    cx, cy = _costmap_center_xy(costmap)
    analytic_cm = _analytic_min_camera_height_cm(costmap, fov_deg)
    start_cm = max(TOPDOWN_HEIGHT_SEARCH_START_CM, analytic_cm * 0.95)
    height_cm = start_cm

    ucv.set_camera_rotation(
        camera_id,
        (TOPDOWN_CAM_PITCH_DEG, TOPDOWN_CAM_YAW_DEG, 0.0),
    )

    while height_cm <= TOPDOWN_HEIGHT_SEARCH_MAX_CM:
        cam_z = ground_z_cm + height_cm
        if _map_corners_fit_in_image(
            costmap,
            cam_x=cx,
            cam_y=cy,
            cam_z=cam_z,
            ground_z_cm=ground_z_cm,
            width_px=width_px,
            height_px=height_px,
            fov_deg=fov_deg,
        ):
            ucv.set_camera_location(camera_id, (cx, cy, cam_z))
            return float(height_cm)
        height_cm += TOPDOWN_HEIGHT_SEARCH_STEP_CM

    ucv.set_camera_location(camera_id, (cx, cy, ground_z_cm + analytic_cm))
    print(
        f"[Costmap] FOV search hit max; using analytic height {analytic_cm:.0f} cm"
    )
    return float(analytic_cm)


def _pixel_to_world_xy_nadir(
    pu: np.ndarray,
    pv: np.ndarray,
    depth_m: np.ndarray,
    *,
    cam_x: float,
    cam_y: float,
    cam_z: float,
    ground_z_cm: float,
    width_px: int,
    height_px: int,
    fov_deg: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    各画素を地面 Z 平面へ逆投影（歪み補正）。

    Returns:
        wx_cm, wy_cm, slant_range_m
    """
    cam_h_m = max((cam_z - ground_z_cm) / 100.0, 0.5)
    focal = _focal_length_px(width_px, fov_deg)
    dx_m = (pu.astype(np.float64) - width_px * 0.5) / focal * cam_h_m
    dy_m = -(pv.astype(np.float64) - height_px * 0.5) / focal * cam_h_m
    wx_cm = cam_x + dx_m * 100.0
    wy_cm = cam_y + dy_m * 100.0
    slant_m = np.sqrt(cam_h_m * cam_h_m + dx_m * dx_m + dy_m * dy_m)
    return wx_cm, wy_cm, slant_m


def _depth_image_to_pillar_votes(
    costmap: Costmap2D,
    depth: np.ndarray,
    *,
    cam_x: float,
    cam_y: float,
    cam_z: float,
    ground_z_cm: float,
    fov_deg: float,
    depth_margin_m: float,
) -> np.ndarray:
    """depth 画素を透視補正して格子へ投票（柱候補セル）。"""
    height_px, width_px = depth.shape
    pu_grid, pv_grid = np.meshgrid(
        np.arange(width_px, dtype=np.int32),
        np.arange(height_px, dtype=np.int32),
    )
    wx_cm, wy_cm, slant_m = _pixel_to_world_xy_nadir(
        pu_grid,
        pv_grid,
        depth,
        cam_x=cam_x,
        cam_y=cam_y,
        cam_z=cam_z,
        ground_z_cm=ground_z_cm,
        width_px=width_px,
        height_px=height_px,
        fov_deg=fov_deg,
    )

    measured = depth.astype(np.float64)
    finite = np.isfinite(measured) & (measured > TOPDOWN_MIN_VALID_DEPTH_M)
    depth_fill = np.where(finite, measured, 0.0)
    local_far = maximum_filter(depth_fill, size=TOPDOWN_LOCAL_MAX_FILTER_PX)
    img_residual = local_far - measured
    pillar_px = finite & (img_residual >= float(depth_margin_m))
    pillar_px = _filter_square_pillar_clusters(pillar_px)

    in_map = (
        (wx_cm >= costmap.origin_xy[0])
        & (wx_cm <= costmap.origin_xy[0] + costmap.size_x_cm)
        & (wy_cm >= costmap.origin_xy[1])
        & (wy_cm <= costmap.origin_xy[1] + costmap.size_y_cm)
    )
    pillar_px = pillar_px & in_map
    if not np.any(pillar_px):
        return np.zeros((costmap.height_cells, costmap.width_cells), dtype=bool)

    gx = np.floor(
        (wx_cm - costmap.origin_xy[0]) / costmap.resolution_cm
    ).astype(np.int32)
    gy = np.floor(
        (wy_cm - costmap.origin_xy[1]) / costmap.resolution_cm
    ).astype(np.int32)
    project = (
        pillar_px
        & (gx >= 0)
        & (gx < costmap.width_cells)
        & (gy >= 0)
        & (gy < costmap.height_cells)
    )

    votes = np.zeros((costmap.height_cells, costmap.width_cells), dtype=np.int32)
    if np.any(project):
        np.add.at(votes, (gy[project], gx[project]), 1)

    return votes >= TOPDOWN_DEPTH_VOTES_PER_CELL


def _eccentricity(xs: np.ndarray, ys: np.ndarray) -> float:
    if xs.size < 3:
        return 99.0
    cov = np.cov(np.vstack([xs - xs.mean(), ys - ys.mean()]))
    eigvals = np.sort(np.linalg.eigvalsh(cov))
    if eigvals[0] < 1e-8:
        return 99.0
    return float(math.sqrt(eigvals[1] / eigvals[0]))


def _filter_square_pillar_clusters(mask: np.ndarray) -> np.ndarray:
    """細長い影・壁片を除き、正方形に近いクラスタのみ残す。"""
    labeled, count = label(mask)
    if count == 0:
        return mask
    keep = np.zeros_like(mask, dtype=bool)
    for comp_id in range(1, count + 1):
        ys, xs = np.where(labeled == comp_id)
        area = xs.size
        if area < TOPDOWN_SQUARE_MIN_AREA_CELLS or area > TOPDOWN_SQUARE_MAX_AREA_CELLS:
            continue
        width = int(xs.max() - xs.min() + 1)
        height = int(ys.max() - ys.min() + 1)
        side_min = min(width, height)
        side_max = max(width, height)
        if side_min < TOPDOWN_SQUARE_MIN_SIDE_CELLS:
            continue
        if side_max > TOPDOWN_SQUARE_MAX_SIDE_CELLS:
            continue
        aspect = side_min / max(side_max, 1)
        if aspect < TOPDOWN_SQUARE_MIN_ASPECT or aspect > TOPDOWN_SQUARE_MAX_ASPECT:
            continue
        keep[labeled == comp_id] = True
    return keep


def _mask_to_square_bboxes(mask: np.ndarray) -> List[Tuple[int, int, int, int]]:
    """各連結成分の軸平行外接正方形（格子 index）。"""
    labeled, count = label(mask)
    boxes: List[Tuple[int, int, int, int]] = []
    for comp_id in range(1, count + 1):
        ys, xs = np.where(labeled == comp_id)
        if xs.size == 0:
            continue
        pad = TOPDOWN_SQUARE_BBOX_PAD_CELLS
        gx0 = max(0, int(xs.min()) - pad)
        gx1 = min(mask.shape[1] - 1, int(xs.max()) + pad)
        gy0 = max(0, int(ys.min()) - pad)
        gy1 = min(mask.shape[0] - 1, int(ys.max()) + pad)
        boxes.append((gx0, gy0, gx1, gy1))
    return boxes


def _apply_square_bboxes_to_mask(
    shape: Tuple[int, int],
    boxes: Sequence[Tuple[int, int, int, int]],
) -> np.ndarray:
    out = np.zeros(shape, dtype=bool)
    for gx0, gy0, gx1, gy1 in boxes:
        out[gy0 : gy1 + 1, gx0 : gx1 + 1] = True
    return out


def _exclude_actor_disks_fast(
    costmap: Costmap2D,
    mask: np.ndarray,
    exclude_actor_xy: Sequence[WorldXY],
    *,
    radius_cm: float = TOPDOWN_AGENT_EXCLUDE_RADIUS_CM,
) -> np.ndarray:
    if not exclude_actor_xy:
        return mask
    gx = np.arange(costmap.width_cells, dtype=np.float64)
    gy = np.arange(costmap.height_cells, dtype=np.float64)
    grid_gx, grid_gy = np.meshgrid(gx, gy)
    wx = costmap.origin_xy[0] + (grid_gx + 0.5) * costmap.resolution_cm
    wy = costmap.origin_xy[1] + (grid_gy + 0.5) * costmap.resolution_cm
    out = mask.copy()
    radius_sq = float(radius_cm) ** 2
    for ax, ay in exclude_actor_xy:
        out[(wx - ax) ** 2 + (wy - ay) ** 2 <= radius_sq] = False
    return out


def scan_obstacles_topdown_depth(
    ucv,
    costmap: Costmap2D,
    *,
    ground_z_cm: float,
    camera_id: Optional[int] = None,
    exclude_actor_xy: Optional[Sequence[WorldXY]] = None,
    pillar_depth_margin_m: Optional[float] = None,
) -> Tuple[np.ndarray, float]:
    """
    マップ中心真上・真下で高度を探索し、depth のみから正方形柱マスクを構築。

    Returns:
        (mask, camera_height_cm above ground)
    """
    margin_m = float(
        TOPDOWN_PILLAR_DEPTH_MARGIN_M
        if pillar_depth_margin_m is None
        else pillar_depth_margin_m
    )
    cam_id = _first_numeric_camera_id(ucv) if camera_id is None else int(camera_id)
    cx, cy = _costmap_center_xy(costmap)
    width_px, height_px = TOPDOWN_CAPTURE_RESOLUTION

    ucv.set_camera_resolution(cam_id, (width_px, height_px))
    ucv.set_camera_fov(cam_id, TOPDOWN_CAM_FOV_DEG)

    height_cm = find_camera_height_for_full_map(
        ucv,
        costmap,
        ground_z_cm=ground_z_cm,
        camera_id=cam_id,
        width_px=width_px,
        height_px=height_px,
    )
    cam_z = float(ground_z_cm) + height_cm
    ucv.set_camera_location(cam_id, (cx, cy, cam_z))
    ucv.set_camera_rotation(
        cam_id,
        (TOPDOWN_CAM_PITCH_DEG, TOPDOWN_CAM_YAW_DEG, 0.0),
    )
    print(
        f"[Costmap] Top-down camera at map center ({cx:.1f}, {cy:.1f}), "
        f"height={height_cm:.0f} cm, pitch={TOPDOWN_CAM_PITCH_DEG:.0f}°"
    )
    if TOPDOWN_CAPTURE_SETTLE_S > 0:
        time.sleep(TOPDOWN_CAPTURE_SETTLE_S)

    depth = _fetch_depth_npy(ucv, cam_id)
    global _LAST_TOPDOWN_CAPTURE
    _LAST_TOPDOWN_CAPTURE = {
        "depth": depth,
        "cam_xyz": (cx, cy, cam_z),
        "ground_z_cm": float(ground_z_cm),
    }
    candidate = _depth_image_to_pillar_votes(
        costmap,
        depth,
        cam_x=cx,
        cam_y=cy,
        cam_z=cam_z,
        ground_z_cm=ground_z_cm,
        fov_deg=TOPDOWN_CAM_FOV_DEG,
        depth_margin_m=margin_m,
    )
    boxes = _mask_to_square_bboxes(candidate)
    candidate = _apply_square_bboxes_to_mask(candidate.shape, boxes)
    candidate = _exclude_actor_disks_fast(
        costmap, candidate, list(exclude_actor_xy or ())
    )
    return candidate, height_cm


def log_actor_alignment_on_costmap(
    costmap: Costmap2D,
    actors: Sequence[Tuple[str, WorldXY]],
    *,
    cam_x: float,
    cam_y: float,
    cam_z: float,
    ground_z_cm: float,
    width_px: int,
    height_px: int,
    fov_deg: float = TOPDOWN_CAM_FOV_DEG,
) -> None:
    """スキャン直後: actor の格子・画素位置をログ（投影ずれの診断用）。"""
    for label, (wx, wy) in actors:
        grid = costmap.world_xy_to_grid((wx, wy), clamp=False)
        pu, pv, inside = _world_xy_to_pixel_nadir(
            wx,
            wy,
            cam_x=cam_x,
            cam_y=cam_y,
            cam_z=cam_z,
            ground_z_cm=ground_z_cm,
            width_px=width_px,
            height_px=height_px,
            fov_deg=fov_deg,
        )
        print(
            f"[Costmap] align {label}: world=({wx:.1f},{wy:.1f}) "
            f"grid={grid} pixel=({pu},{pv}) in_fov={inside}"
        )


def save_topdown_camera_debug(
    depth: np.ndarray,
    costmap: Costmap2D,
    actors: Sequence[Tuple[str, WorldXY]],
    output_dir: Path,
    *,
    cam_x: float,
    cam_y: float,
    cam_z: float,
    ground_z_cm: float,
    fov_deg: float = TOPDOWN_CAM_FOV_DEG,
) -> Path:
    """真上 depth に actor 投影位置を重ねた診断 PNG。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    height_px, width_px = depth.shape
    out = Path(output_dir) / "topdown_scan_camera.png"
    fig, ax = plt.subplots(figsize=(8, 8))
    valid = np.isfinite(depth) & (depth > TOPDOWN_MIN_VALID_DEPTH_M)
    show = np.where(valid, depth, np.nan)
    ax.imshow(show, origin="upper", cmap="viridis")
    for label, (wx, wy) in actors:
        pu, pv, inside = _world_xy_to_pixel_nadir(
            wx,
            wy,
            cam_x=cam_x,
            cam_y=cam_y,
            cam_z=cam_z,
            ground_z_cm=ground_z_cm,
            width_px=width_px,
            height_px=height_px,
            fov_deg=fov_deg,
        )
        if inside:
            ax.plot(pu, pv, "o", markersize=10, label=label)
    ax.legend(loc="upper right")
    ax.set_title("Top-down depth + actor projections")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return out


def save_obstacle_scan_debug(
    costmap: Costmap2D,
    mask: np.ndarray,
    output_dir: Path,
    *,
    title: str = "Costmap obstacles (square pillars)",
) -> Path:
    from path_planning_costmap import plot_costmap_with_paths

    debug_map = costmap_from_mask(costmap, mask)
    out = Path(output_dir) / "costmap_obstacles.png"
    plot_costmap_with_paths(
        debug_map,
        [],
        title=title,
        save_path=str(out),
        show=False,
    )
    return out


def costmap_from_mask(base: Costmap2D, mask: np.ndarray) -> Costmap2D:
    costs = np.full_like(base.costs, 1.0, dtype=np.float64)
    costs[mask] = OBSTACLE_SCAN_CELL_COST
    return Costmap2D(
        costs=costs,
        origin_xy=base.origin_xy,
        resolution_cm=base.resolution_cm,
        lethal_cost=base.lethal_cost,
    )


def spawn_costmap_probe(ucv, location: WorldXYZ) -> None:
    if COSTMAP_PROBE_NAME not in {str(n) for n in ucv.get_objects().tolist()}:
        ucv.spawn_bp_asset(COSTMAP_PROBE_BP_PATH, COSTMAP_PROBE_NAME)
    ucv.set_scale(COSTMAP_PROBE_SCALE, COSTMAP_PROBE_NAME)
    ucv.set_physics(COSTMAP_PROBE_NAME, False)
    ucv.set_collision(COSTMAP_PROBE_NAME, True)
    ucv.set_movable(COSTMAP_PROBE_NAME, True)
    ucv.set_location(location, COSTMAP_PROBE_NAME)
    ucv.set_orientation((0.0, 0.0, 0.0), COSTMAP_PROBE_NAME)


def destroy_costmap_probe(ucv) -> None:
    objects = {str(name) for name in ucv.get_objects().tolist()}
    if COSTMAP_PROBE_NAME in objects:
        ucv.destroy(COSTMAP_PROBE_NAME)


def _grid_centers_for_scan(costmap: Costmap2D, stride_cells: int) -> List[GridCell]:
    stride = max(1, int(stride_cells))
    centers: List[GridCell] = []
    for gy in range(0, costmap.height_cells, stride):
        for gx in range(0, costmap.width_cells, stride):
            centers.append((gx, gy))
    return centers


def scan_obstacles_collision_probe(
    ucv,
    costmap: Costmap2D,
    *,
    ground_z_cm: float,
    stride_cells: int = OBSTACLE_SCAN_STRIDE_CELLS,
    probe_z_offset_cm: float = 40.0,
) -> np.ndarray:
    mask = np.zeros((costmap.height_cells, costmap.width_cells), dtype=bool)
    centers = _grid_centers_for_scan(costmap, stride_cells)
    if not centers:
        return mask

    first_xy = costmap.grid_to_world_xy_center(centers[0])
    spawn_costmap_probe(ucv, (first_xy[0], first_xy[1], ground_z_cm + probe_z_offset_cm))
    time.sleep(0.1)

    for gx, gy in centers:
        wx, wy = costmap.grid_to_world_xy_center((gx, gy))
        probe_loc = (wx, wy, ground_z_cm + probe_z_offset_cm)
        ucv.set_location(probe_loc, COSTMAP_PROBE_NAME)
        if OBSTACLE_SCAN_SETTLE_S > 0:
            time.sleep(OBSTACLE_SCAN_SETTLE_S)
        counts = parse_collision_counts(ucv.get_collision_num(COSTMAP_PROBE_NAME))
        if not collision_indicates_static_obstacle(counts):
            continue
        mask[gy, gx] = True
        inflate = max(0, int(round(55.0 / costmap.resolution_cm)))
        y0 = max(0, gy - inflate)
        y1 = min(costmap.height_cells, gy + inflate + 1)
        x0 = max(0, gx - inflate)
        x1 = min(costmap.width_cells, gx + inflate + 1)
        mask[y0:y1, x0:x1] = True

    destroy_costmap_probe(ucv)
    return mask


def apply_manual_pillar_obstacles(
    costmap: Costmap2D,
    pillars: Sequence[PillarObstacle],
    *,
    cell_cost: float = OBSTACLE_SCAN_CELL_COST,
    use_lethal: bool = OBSTACLE_SCAN_USE_LETHAL,
) -> int:
    value = COSTMAP_LETHAL_COST if use_lethal else cell_cost
    applied = 0
    for x_cm, y_cm, half_extent_cm in pillars:
        half = float(half_extent_cm)
        gx_c, gy_c = costmap.world_xy_to_grid((x_cm, y_cm), clamp=True)
        pad = max(1, int(round(half / costmap.resolution_cm)))
        gx0 = max(0, gx_c - pad)
        gx1 = min(costmap.width_cells - 1, gx_c + pad)
        gy0 = max(0, gy_c - pad)
        gy1 = min(costmap.height_cells - 1, gy_c + pad)
        costmap.costs[gy0 : gy1 + 1, gx0 : gx1 + 1] = (
            value
            if use_lethal
            else np.maximum(costmap.costs[gy0 : gy1 + 1, gx0 : gx1 + 1], value)
        )
        applied += 1
    return applied


def apply_obstacle_mask_to_costmap(
    costmap: Costmap2D,
    mask: np.ndarray,
    *,
    cell_cost: float = OBSTACLE_SCAN_CELL_COST,
    use_lethal: bool = OBSTACLE_SCAN_USE_LETHAL,
) -> int:
    if mask.shape != costmap.costs.shape:
        raise ValueError(f"mask shape {mask.shape} != costmap {costmap.costs.shape}")
    value = float(COSTMAP_LETHAL_COST if use_lethal else cell_cost)
    cells = int(np.count_nonzero(mask))
    if use_lethal:
        costmap.costs[mask] = value
    else:
        costmap.costs[mask] = np.maximum(costmap.costs[mask], value)
    return cells


def enrich_costmap_with_obstacles(
    ucv,
    costmap: Costmap2D,
    *,
    ground_z_cm: float,
    manual_pillars: Optional[Sequence[PillarObstacle]] = None,
    stride_cells: int = OBSTACLE_SCAN_STRIDE_CELLS,
    use_topdown_depth: bool = True,
    exclude_actor_xy: Optional[Sequence[WorldXY]] = None,
    alignment_actors: Optional[Sequence[Tuple[str, WorldXY]]] = None,
    save_debug_dir: Optional[Path] = None,
) -> ObstacleScanResult:
    scan_method = "topdown_depth_nadir"
    camera_height_cm = 0.0
    if use_topdown_depth:
        mask, camera_height_cm = scan_obstacles_topdown_depth(
            ucv,
            costmap,
            ground_z_cm=ground_z_cm,
            exclude_actor_xy=exclude_actor_xy,
        )
        sampled = int(costmap.width_cells * costmap.height_cells)
    else:
        scan_method = "collision_probe"
        mask = scan_obstacles_collision_probe(
            ucv,
            costmap,
            ground_z_cm=ground_z_cm,
            stride_cells=stride_cells,
        )
        sampled = len(_grid_centers_for_scan(costmap, stride_cells))

    capture = _LAST_TOPDOWN_CAPTURE
    if capture and save_debug_dir is not None and alignment_actors:
        cx, cy, cz = capture["cam_xyz"]
        log_actor_alignment_on_costmap(
            costmap,
            alignment_actors,
            cam_x=cx,
            cam_y=cy,
            cam_z=cz,
            ground_z_cm=capture["ground_z_cm"],
            width_px=capture["depth"].shape[1],
            height_px=capture["depth"].shape[0],
        )
        cam_png = save_topdown_camera_debug(
            capture["depth"],
            costmap,
            alignment_actors,
            Path(save_debug_dir),
            cam_x=cx,
            cam_y=cy,
            cam_z=cz,
            ground_z_cm=capture["ground_z_cm"],
        )
        print(f"[Costmap] Saved top-down camera debug: {cam_png}")

    detected = obstacle_mask_to_world_pillars(costmap, mask)
    if detected:
        print(
            "[Costmap] Square pillars (center_x, center_y, half_side_cm): "
            + ", ".join(f"({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})" for p in detected[:12])
        )
    else:
        print("[Costmap] Pillar scan: no square pillar clusters detected.")

    if save_debug_dir is not None:
        save_obstacle_scan_debug(costmap, mask, Path(save_debug_dir))

    hit_cells = apply_obstacle_mask_to_costmap(costmap, mask)
    pillars = list(manual_pillars or MANUAL_PILLAR_OBSTACLES_CM)
    manual_count = apply_manual_pillar_obstacles(costmap, pillars)
    return ObstacleScanResult(
        sampled_cells=sampled,
        hit_cells=hit_cells,
        manual_pillars=manual_count,
        inflated_regions=hit_cells,
        scan_method=scan_method,
        pillar_count=len(detected),
        camera_height_cm=camera_height_cm,
    )


def obstacle_mask_to_world_pillars(
    costmap: Costmap2D,
    mask: np.ndarray,
    *,
    min_cluster_cells: int = TOPDOWN_SQUARE_MIN_AREA_CELLS,
) -> List[PillarObstacle]:
    pillars: List[PillarObstacle] = []
    if not np.any(mask):
        return pillars

    labeled, count = label(mask)
    for comp_id in range(1, count + 1):
        ys, xs = np.where(labeled == comp_id)
        if xs.size < min_cluster_cells:
            continue
        gx0, gx1 = int(xs.min()), int(xs.max())
        gy0, gy1 = int(ys.min()), int(ys.max())
        wx0, wy0 = costmap.grid_to_world_xy_center((gx0, gy0))
        wx1, wy1 = costmap.grid_to_world_xy_center((gx1, gy1))
        cx = (wx0 + wx1) * 0.5
        cy = (wy0 + wy1) * 0.5
        half_side = max(
            abs(wx1 - wx0) * 0.5,
            abs(wy1 - wy0) * 0.5,
            costmap.resolution_cm,
        )
        pillars.append((float(cx), float(cy), float(half_side)))
    return pillars
