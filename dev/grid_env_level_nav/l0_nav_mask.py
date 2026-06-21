#!/usr/bin/env python3
"""L0 NavMesh mask: build, save, load (costs[gy,gx] lethal outside NavMesh)."""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Callable, Optional, Tuple

import numpy as np

from level_coords import FLOOR_REF_Z_CM, NAV_PROJECT_PROBE_Z_CM, REGION_ORIGIN_WORLD_XY
from work_region import (
    DEFAULT_RESOLUTION_CM,
    DEFAULT_XY_TOLERANCE_CM,
    DEFAULT_Z_TOLERANCE_CM,
    cell_center_world_xy,
    region_height_cells,
    region_width_cells,
)

ProjectFn = Callable[[float, float, float], dict]

COSTMAP_DEFAULT_CELL_COST = 1.0
COSTMAP_LETHAL_COST = 1.0e9


def empty_l0_costs(resolution_cm: float = DEFAULT_RESOLUTION_CM) -> np.ndarray:
    w = region_width_cells(resolution_cm)
    h = region_height_cells(resolution_cm)
    return np.full((h, w), COSTMAP_DEFAULT_CELL_COST, dtype=np.float32)


def projection_xy_distance_cm(result: dict, wx: float, wy: float) -> Optional[float]:
    if not result.get("ok"):
        return None
    try:
        px = float(result["x"])
        py = float(result["y"])
    except (KeyError, TypeError, ValueError):
        return None
    return math.hypot(px - wx, py - wy)


def projection_z_delta_cm(result: dict, floor_z_cm: float) -> Optional[float]:
    if not result.get("ok"):
        return None
    try:
        pz = float(result["z"])
    except (KeyError, TypeError, ValueError):
        return None
    return abs(pz - floor_z_cm)


def project_cell_to_cost(
    result: dict,
    *,
    wx: float,
    wy: float,
    wz: float,
    default_cost: float = COSTMAP_DEFAULT_CELL_COST,
    lethal_cost: float = COSTMAP_LETHAL_COST,
    xy_tolerance_cm: float = DEFAULT_XY_TOLERANCE_CM,
    z_tolerance_cm: float = DEFAULT_Z_TOLERANCE_CM,
    floor_z_cm: float = FLOOR_REF_Z_CM,
) -> float:
    """
    Walkable iff NavMesh projection exists AND (px,py) is near cell center (wx, wy).

    Rejects far snaps (holes/pillars projecting to nearby floor).
    """
    if not result.get("ok"):
        return lethal_cost
    err = str(result.get("error", ""))
    if err:
        return lethal_cost

    xy_dist = projection_xy_distance_cm(result, wx, wy)
    if xy_dist is None or xy_dist > xy_tolerance_cm:
        return lethal_cost

    z_delta = projection_z_delta_cm(result, floor_z_cm)
    if z_delta is not None and z_delta > z_tolerance_cm:
        return lethal_cost

    # wz unused for decision; probe height is only a search hint for UE.
    _ = wz
    return default_cost


def build_l0_mask_from_project_fn(
    project_fn: ProjectFn,
    *,
    resolution_cm: float = DEFAULT_RESOLUTION_CM,
    z_cm: float = NAV_PROJECT_PROBE_Z_CM,
    xy_tolerance_cm: float = DEFAULT_XY_TOLERANCE_CM,
    z_tolerance_cm: float = DEFAULT_Z_TOLERANCE_CM,
    floor_z_cm: float = FLOOR_REF_Z_CM,
    stride: int = 1,
    progress_every: int = 200,
    checkpoint_path: Optional[Path] = None,
    checkpoint_interval: int = 500,
    resume_costs: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Sample each cell center with project_fn(wx, wy, wz) → dict (NavProjectPoint shape)."""
    if stride < 1:
        raise ValueError("stride must be >= 1")
    costs = (
        resume_costs.copy()
        if resume_costs is not None
        else empty_l0_costs(resolution_cm)
    )
    h, w = costs.shape
    sampled = 0
    t0 = time.time()
    for gy in range(0, h, stride):
        for gx in range(0, w, stride):
            wx, wy = cell_center_world_xy(gx, gy, resolution_cm)
            result = project_fn(wx, wy, z_cm)
            costs[gy, gx] = project_cell_to_cost(
                result,
                wx=wx,
                wy=wy,
                wz=z_cm,
                xy_tolerance_cm=xy_tolerance_cm,
                z_tolerance_cm=z_tolerance_cm,
                floor_z_cm=floor_z_cm,
            )
            sampled += 1
            if progress_every and sampled % progress_every == 0:
                elapsed = time.time() - t0
                rate = sampled / max(elapsed, 1e-6)
                print(
                    f"[L0] sampled {sampled} cells ({gy}/{h} rows) "
                    f"{rate:.1f} cells/s elapsed={elapsed:.0f}s"
                )
            if (
                checkpoint_path is not None
                and checkpoint_interval > 0
                and sampled % checkpoint_interval == 0
            ):
                save_l0_mask_npz(
                    checkpoint_path,
                    costs,
                    resolution_cm=resolution_cm,
                    partial=True,
                )
    if stride > 1:
        costs = upsample_strided_l0(costs, stride=stride)
    return costs


def upsample_strided_l0(costs: np.ndarray, *, stride: int) -> np.ndarray:
    """Fill gaps in a strided sample using nearest sampled neighbor."""
    if stride <= 1:
        return costs
    h, w = costs.shape
    out = costs.copy()
    sample_mask = np.zeros((h, w), dtype=bool)
    for gy in range(0, h, stride):
        for gx in range(0, w, stride):
            sample_mask[gy, gx] = True
    for gy in range(h):
        for gx in range(w):
            if sample_mask[gy, gx]:
                continue
            sy = min(range(0, h, stride), key=lambda y: abs(y - gy))
            sx = min(range(0, w, stride), key=lambda x: abs(x - gx))
            out[gy, gx] = costs[sy, sx]
    return out


def save_l0_mask_npz(
    path: Path,
    costs: np.ndarray,
    *,
    resolution_cm: float = DEFAULT_RESOLUTION_CM,
    partial: bool = False,
    xy_tolerance_cm: float = DEFAULT_XY_TOLERANCE_CM,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        costs=costs.astype(np.float32),
        origin_xy=np.array(REGION_ORIGIN_WORLD_XY, dtype=np.float64),
        resolution_cm=np.float64(resolution_cm),
        lethal_cost=np.float64(COSTMAP_LETHAL_COST),
        default_cost=np.float64(COSTMAP_DEFAULT_CELL_COST),
        xy_tolerance_cm=np.float64(xy_tolerance_cm),
        partial=np.array(partial),
    )
    lethal = int(np.sum(costs >= COSTMAP_LETHAL_COST * 0.5))
    print(
        f"[L0] saved {path} shape={costs.shape} "
        f"lethal={lethal}/{costs.size} partial={partial}"
    )


def load_l0_mask_npz(path: Path) -> Tuple[np.ndarray, float, Tuple[float, float], float]:
    data = np.load(path, allow_pickle=False)
    costs = np.asarray(data["costs"], dtype=np.float32)
    resolution_cm = float(data["resolution_cm"])
    origin = tuple(float(x) for x in data["origin_xy"])
    lethal_cost = float(data.get("lethal_cost", COSTMAP_LETHAL_COST))
    return costs, resolution_cm, origin, lethal_cost


def is_l0_cache_complete(path: Path) -> bool:
    if not path.is_file():
        return False
    data = np.load(path, allow_pickle=False)
    return not bool(data.get("partial", True))


def synthetic_l0_corridor(
    *,
    resolution_cm: float = DEFAULT_RESOLUTION_CM,
    corridor_local_y: Tuple[float, float] = (2000.0, 5000.0),
) -> np.ndarray:
    """Unit-test / offline planner mask: lethal outside a local-Y corridor band."""
    from level_coords import world_xy_to_local

    costs = empty_l0_costs(resolution_cm)
    h, w = costs.shape
    for gy in range(h):
        for gx in range(w):
            wx, wy = cell_center_world_xy(gx, gy, resolution_cm)
            _, ly = world_xy_to_local(wx, wy)
            if ly < corridor_local_y[0] or ly > corridor_local_y[1]:
                costs[gy, gx] = COSTMAP_LETHAL_COST
    return costs
