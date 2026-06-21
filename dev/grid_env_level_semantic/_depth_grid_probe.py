#!/usr/bin/env python3
"""Depth at each registry block XY."""
from __future__ import annotations

import json
import sys
import time
from io import BytesIO
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
from level_semantic_scan import _fetch_depth_npy, _is_valid_depth_cm  # noqa: E402

CAM_ID = 0
CAM_Z = 7430.0
PATCH_R = 2


def patch_surface_z(depth, cam_z: float):
    h, w = depth.shape
    cy, cx = h // 2, w // 2
    patch = depth[
        max(0, cy - PATCH_R) : min(h, cy + PATCH_R + 1),
        max(0, cx - PATCH_R) : min(w, cx + PATCH_R + 1),
    ]
    valid = [float(v) for v in patch.ravel() if _is_valid_depth_cm(float(v), cam_z_cm=cam_z)]
    if not valid:
        return None
    d = min(valid)
    return cam_z - d


def main() -> int:
    reg = json.loads((THIS_DIR / ".level_semantic_registry.json").read_text())
    ucv, _ = g10k.ensure_connection()
    print(f"cam_z={CAM_Z}")
    print("gx  gy    x      y     center_d  frame_min  patch_z")
    for name, blk in sorted(reg["blocks"].items()):
        gx, gy = blk["gx"], blk["gy"]
        x, y, _ = blk["world_cm"]
        ucv.set_camera_location(CAM_ID, (x, y, CAM_Z))
        ucv.set_camera_rotation(CAM_ID, (-90.0, 0.0, 0.0))
        time.sleep(0.15)
        try:
            ucv.tick()
        except Exception:
            pass
        depth = _fetch_depth_npy(ucv, CAM_ID)
        h, w = depth.shape
        center_d = float(depth[h // 2, w // 2])
        frame_valid = [
            float(v)
            for v in depth.ravel()
            if _is_valid_depth_cm(float(v), cam_z_cm=CAM_Z)
        ]
        frame_min = min(frame_valid) if frame_valid else None
        pz = patch_surface_z(depth, CAM_Z)
        print(
            f"{gx:3d} {gy:3d} {x:7.0f} {y:7.0f} "
            f"{center_d:9.1f} {frame_min if frame_min else 'none':>9} "
            f"{pz if pz else 'none':>7}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
