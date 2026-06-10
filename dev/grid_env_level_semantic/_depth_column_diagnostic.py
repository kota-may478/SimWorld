#!/usr/bin/env python3
"""Debug depth column filter vs center-patch at rooftop edge."""
from __future__ import annotations

import sys
import time
from io import BytesIO
from pathlib import Path

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
from level_semantic_scan import (  # noqa: E402
    DEPTH_MIN_VALID_CM,
    DEPTH_SKY_THRESHOLD_CM,
    _fetch_depth_npy,
    _is_valid_depth_cm,
    _surface_z_from_column_depth,
)

CAM_ID = 0
CAM_Z = 7430.0
PATCH_R = 2


def center_patch_min(depth: np.ndarray, cam_z: float) -> tuple[float | None, float | None]:
    h, w = depth.shape
    cy, cx = h // 2, w // 2
    patch = depth[
        max(0, cy - PATCH_R) : min(h, cy + PATCH_R + 1),
        max(0, cx - PATCH_R) : min(w, cx + PATCH_R + 1),
    ]
    valid = [float(v) for v in patch.ravel() if _is_valid_depth_cm(float(v), cam_z_cm=cam_z)]
    if not valid:
        return None, None
    d = min(valid)
    return cam_z - d, d


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    fov = float(ucv.get_camera_fov(CAM_ID))
    print(f"fov={fov}")

    xs = list(range(5955, 6376, 30))
    y = 1170.0
    print(f"cam_z={CAM_Z} patch_r={PATCH_R}")
    print("x_cm   center_d  frame_min_d  patch_d  patch_z")
    for x in xs:
        ucv.set_camera_location(CAM_ID, (x, y, CAM_Z))
        ucv.set_camera_rotation(CAM_ID, (-90.0, 0.0, 0.0))
        time.sleep(0.18)
        try:
            ucv.tick()
        except Exception:
            pass
        depth = _fetch_depth_npy(ucv, CAM_ID)
        h, w = depth.shape
        center_d = float(depth[h // 2, w // 2])
        patch_z, patch_d = center_patch_min(depth, CAM_Z)
        frame_valid = [
            float(v)
            for v in depth.ravel()
            if _is_valid_depth_cm(float(v), cam_z_cm=CAM_Z)
        ]
        frame_min = min(frame_valid) if frame_valid else None
        print(
            f"{x:6.0f} {center_d:9.1f} "
            f"{frame_min if frame_min else 'none':>11} "
            f"{patch_d if patch_d else 'none':>8} "
            f"{patch_z if patch_z else 'none':>7}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
