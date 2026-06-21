#!/usr/bin/env python3
"""Sweep camera Z at one rooftop cell."""
from __future__ import annotations

import sys
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
from level_semantic_scan import _fetch_depth_npy, _is_valid_depth_cm  # noqa: E402

CAM_ID = 0
X, Y = 6225.0, 1125.0


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    print(f"xy=({X},{Y}) rot after set:")
    for cam_z in range(6850, 7510, 50):
        ucv.set_camera_location(CAM_ID, (X, Y, float(cam_z)))
        ucv.set_camera_rotation(CAM_ID, (-90.0, 0.0, 0.0))
        time.sleep(0.12)
        try:
            ucv.tick()
        except Exception:
            pass
        rot = ucv.get_camera_rotation(CAM_ID)
        depth = _fetch_depth_npy(ucv, CAM_ID)
        h, w = depth.shape
        c = float(depth[h // 2, w // 2])
        valid = [float(v) for v in depth.ravel() if _is_valid_depth_cm(float(v), cam_z_cm=float(cam_z))]
        fmin = min(valid) if valid else None
        pz = (float(cam_z) - fmin) if fmin else None
        cz = (float(cam_z) - c) if _is_valid_depth_cm(c, cam_z_cm=float(cam_z)) else None
        print(
            f"z={cam_z} rot={rot} center={c:.1f} "
            f"frame_min={fmin if fmin else 'none'} "
            f"surf_center={cz if cz else 'none'} surf_frame={pz if pz else 'none'}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
