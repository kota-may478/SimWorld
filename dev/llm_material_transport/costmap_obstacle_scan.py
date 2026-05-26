"""
UE 上の障害物（柱など）を 2D コストマップへ反映する。

UnrealCV 1.0.1 にはワールド線分 trace が無い。GetCollisionNum は Humanoid / Spot 用で
静止テレポートでは増えない。主手段は真上カメラ depth（1 枚）→ 格子マスク。
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from io import BytesIO
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import binary_dilation, label, maximum_filter, zoom

from path_planning_costmap import (
    COSTMAP_LETHAL_COST,
    Costmap2D,
    GridCell,
    WorldXY,
    add_circular_cost_region,
)

WorldXYZ = Tuple[float, float, float]
PillarObstacle = Tuple[float, float, float]  # x_cm, y_cm, radius_cm


COSTMAP_PROBE_NAME = "MT_CostmapProbe"
COSTMAP_PROBE_BP_PATH = "/Game/CityDatabase/blueprints/BP_Box.BP_Box_C"
COSTMAP_PROBE_SCALE = (0.15, 0.15, 0.5)

OBSTACLE_SCAN_STRIDE_CELLS = 15
OBSTACLE_SCAN_INFLATE_RADIUS_CM = 55.0
OBSTACLE_SCAN_CELL_COST = 80.0
OBSTACLE_SCAN_USE_LETHAL = False
OBSTACLE_SCAN_SETTLE_S = 0.02

# 真上 depth スキャン（推奨）
TOPDOWN_CAMERA_ID = 0
TOPDOWN_CAM_HEIGHT_CM = 2500.0
TOPDOWN_CAM_PITCH_DEG = -90.0
TOPDOWN_CAM_FOV_DEG = 90.0
TOPDOWN_DEPTH_MARGIN_M = 1.26
TOPDOWN_LOCAL_MAX_FILTER_PX = 31
TOPDOWN_CAPTURE_RESOLUTION = (512, 512)
TOPDOWN_MIN_CLUSTER_PX = 80
TOPDOWN_CAPTURE_SETTLE_S = 0.35
TOPDOWN_MIN_VALID_DEPTH_M = 0.5

# 手動で柱中心を追記する場合（UE スキャン結果の校正用）。空ならスキャンのみ。
MANUAL_PILLAR_OBSTACLES_CM: List[PillarObstacle] = []


@dataclass(frozen=True)
class ObstacleScanResult:
    sampled_cells: int
    hit_cells: int
    manual_pillars: int
    inflated_regions: int
    scan_method: str = "topdown_depth"


def parse_collision_counts(raw_response: object) -> dict:
    """GetCollisionNum の JSON / dict を正規化する。"""
    if isinstance(raw_response, dict):
        return raw_response
    if isinstance(raw_response, str):
        text = raw_response.strip()
        if text.startswith("error"):
            return {}
        return json.loads(text)
    return {}


def collision_indicates_static_obstacle(counts: dict) -> bool:
    """柱・壁・建造物など静止障害物との接触（移動後のイベントカウンタ）。"""
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
    size_x_cm = costmap.width_cells * costmap.resolution_cm
    size_y_cm = costmap.height_cells * costmap.resolution_cm
    return (
        costmap.origin_xy[0] + size_x_cm * 0.5,
        costmap.origin_xy[1] + size_y_cm * 0.5,
    )


def scan_obstacles_topdown_depth(
    ucv,
    costmap: Costmap2D,
    *,
    ground_z_cm: float,
    camera_id: Optional[int] = None,
    depth_margin_m: Optional[float] = None,
) -> np.ndarray:
    """
    コストマップ全域を見下ろす depth 1 枚から占有マスクを作る。

    床より depth_margin_m 以上手前（カメラに近い）ピクセルを障害物とみなす。
    解像度は格子数に合わせてリサイズ（おおよそ 1 セル ≈ 1 ピクセル）。
    """
    cam_id = _first_numeric_camera_id(ucv) if camera_id is None else int(camera_id)
    margin_m = float(TOPDOWN_DEPTH_MARGIN_M if depth_margin_m is None else depth_margin_m)
    min_cluster_px = int(TOPDOWN_MIN_CLUSTER_PX)
    cx, cy = _costmap_center_xy(costmap)
    cam_z = float(ground_z_cm) + TOPDOWN_CAM_HEIGHT_CM
    capture_res = TOPDOWN_CAPTURE_RESOLUTION

    ucv.set_camera_resolution(cam_id, capture_res)
    ucv.set_camera_fov(cam_id, TOPDOWN_CAM_FOV_DEG)
    ucv.set_camera_location(cam_id, (cx, cy, cam_z))
    ucv.set_camera_rotation(cam_id, (TOPDOWN_CAM_PITCH_DEG, 0.0, 0.0))
    if TOPDOWN_CAPTURE_SETTLE_S > 0:
        time.sleep(TOPDOWN_CAPTURE_SETTLE_S)

    depth = _fetch_depth_npy(ucv, cam_id)
    valid = np.isfinite(depth) & (depth > TOPDOWN_MIN_VALID_DEPTH_M)
    if not np.any(valid):
        return np.zeros((costmap.height_cells, costmap.width_cells), dtype=bool)

    depth_fill = np.where(valid, depth, 0.0)
    local_far = maximum_filter(depth_fill, size=TOPDOWN_LOCAL_MAX_FILTER_PX)
    obstacle_img = valid & ((local_far - depth) > margin_m)

    labeled, component_count = label(obstacle_img)
    if component_count > 0:
        counts = np.bincount(labeled.ravel())
        keep_ids = {
            idx
            for idx in range(1, len(counts))
            if counts[idx] >= min_cluster_px
        }
        obstacle_img = np.isin(labeled, list(keep_ids))

    zoom_y = costmap.height_cells / obstacle_img.shape[0]
    zoom_x = costmap.width_cells / obstacle_img.shape[1]
    mask_f = zoom(obstacle_img.astype(np.float32), (zoom_y, zoom_x), order=0)
    mask = mask_f > 0.5

    inflate = max(0, int(round(OBSTACLE_SCAN_INFLATE_RADIUS_CM / costmap.resolution_cm)))
    if inflate > 0:
        structure = np.ones((2 * inflate + 1, 2 * inflate + 1), dtype=bool)
        mask = binary_dilation(mask, structure=structure)
    return mask


def spawn_costmap_probe(ucv, location: WorldXYZ) -> None:
    """コストマップ用の小型プローブを生成する。"""
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
    """
    プローブを格子点へ置き、Building/Object コリジョンで占有マスクを作る。

    Returns:
        bool ndarray shape (height_cells, width_cells)
    """
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
        inflate = max(0, int(round(OBSTACLE_SCAN_INFLATE_RADIUS_CM / costmap.resolution_cm)))
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
    """手動指定の柱円領域をコストマップへ書き込む。"""
    value = COSTMAP_LETHAL_COST if use_lethal else cell_cost
    applied = 0
    for x_cm, y_cm, radius_cm in pillars:
        add_circular_cost_region(
            costmap,
            (float(x_cm), float(y_cm)),
            float(radius_cm),
            value,
            replace=use_lethal,
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
    """占有マスクをコストへ反映。"""
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
) -> ObstacleScanResult:
    """UE depth（推奨）またはプローブ走査 + 手動柱をコストマップへ統合。"""
    scan_method = "topdown_depth"
    if use_topdown_depth:
        mask = scan_obstacles_topdown_depth(ucv, costmap, ground_z_cm=ground_z_cm)
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
    detected = obstacle_mask_to_world_pillars(costmap, mask)
    if detected:
        print(
            "[Costmap] Depth scan pillar approx (x_cm, y_cm, radius_cm): "
            + ", ".join(f"({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})" for p in detected[:12])
        )
    hit_cells = apply_obstacle_mask_to_costmap(costmap, mask)
    pillars = list(manual_pillars or MANUAL_PILLAR_OBSTACLES_CM)
    manual_count = apply_manual_pillar_obstacles(costmap, pillars)
    return ObstacleScanResult(
        sampled_cells=sampled,
        hit_cells=hit_cells,
        manual_pillars=manual_count,
        inflated_regions=hit_cells,
        scan_method=scan_method,
    )


def obstacle_mask_to_world_pillars(
    costmap: Costmap2D,
    mask: np.ndarray,
    *,
    min_cluster_cells: int = 4,
) -> List[PillarObstacle]:
    """占有マスクから柱中心の近似リストを抽出（デバッグ・手動登録用）。"""
    pillars: List[PillarObstacle] = []
    if not np.any(mask):
        return pillars

    visited = np.zeros_like(mask, dtype=bool)
    height, width = mask.shape
    for gy in range(height):
        for gx in range(width):
            if not mask[gy, gx] or visited[gy, gx]:
                continue
            stack = [(gx, gy)]
            cluster: List[GridCell] = []
            visited[gy, gx] = True
            while stack:
                cx, cy = stack.pop()
                cluster.append((cx, cy))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < width and 0 <= ny < height and mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        stack.append((nx, ny))
            if len(cluster) < min_cluster_cells:
                continue
            xs = []
            ys = []
            for cx, cy in cluster:
                wx, wy = costmap.grid_to_world_xy_center((cx, cy))
                xs.append(wx)
                ys.append(wy)
            radius_cm = max(
                OBSTACLE_SCAN_INFLATE_RADIUS_CM,
                math.sqrt(len(cluster)) * costmap.resolution_cm * 0.5,
            )
            pillars.append((float(sum(xs) / len(xs)), float(sum(ys) / len(ys)), radius_cm))
    return pillars
