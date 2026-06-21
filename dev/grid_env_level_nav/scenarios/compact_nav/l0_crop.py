#!/usr/bin/env python3
"""Crop a full Level L0 mask to a local rectangular sub-region."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from region import DEFAULT_RESOLUTION_CM, REGION_ORIGIN_WORLD_XY, REGION_SIZE_CM
from costmap_layers import LayeredCostmap
from l0_nav_mask import load_l0_mask_npz


def crop_l0_to_local_region(
    l0_path: Path | str,
    *,
    size_x_cm: float = REGION_SIZE_CM,
    size_y_cm: float = REGION_SIZE_CM,
    origin_xy: tuple[float, float] = REGION_ORIGIN_WORLD_XY,
    resolution_cm: float | None = None,
) -> LayeredCostmap:
    costs, res, file_origin, lethal = load_l0_mask_npz(Path(l0_path))
    if resolution_cm is None:
        resolution_cm = res
    if abs(res - resolution_cm) > 0.01:
        raise ValueError(f"L0 resolution {res}cm != requested {resolution_cm}cm")

    ox_file, oy_file = file_origin
    if abs(ox_file - origin_xy[0]) > 1.0 or abs(oy_file - origin_xy[1]) > 1.0:
        raise ValueError(f"L0 origin {file_origin} != expected {origin_xy}")

    w = int(math.ceil(size_x_cm / resolution_cm))
    h = int(math.ceil(size_y_cm / resolution_cm))
    h = min(h, costs.shape[0])
    w = min(w, costs.shape[1])
    cropped = costs[:h, :w].astype(np.float32, copy=True)
    return LayeredCostmap(
        l0=cropped,
        origin_xy=origin_xy,
        resolution_cm=resolution_cm,
        lethal_cost=lethal,
    )
