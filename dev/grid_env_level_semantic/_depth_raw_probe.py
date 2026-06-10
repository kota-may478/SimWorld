#!/usr/bin/env python3
"""Print raw depth npy stats at verify XY (Level PIE)."""
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
from spawn_fixed_height_verify import VERIFY_CENTER_XY_CM  # noqa: E402

CAM_ID = 0


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    x, y = 6300.0, 1170.0
    cam_z = 7430.0
    ucv.set_camera_location(CAM_ID, (x, y, cam_z))
    ucv.set_camera_rotation(CAM_ID, (-90.0, 0.0, 0.0))
    time.sleep(0.3)
    try:
        ucv.tick()
    except Exception:
        pass
    cmd = f"vget /camera/{CAM_ID}/depth npy"
    with ucv.lock:
        payload = ucv.client.request(cmd)
    depth = np.load(BytesIO(payload))
    h, w = depth.shape
    c = depth[h // 2, w // 2]
    print(f"shape={depth.shape} dtype={depth.dtype}")
    print(f"center={c} min={np.nanmin(depth)} max={np.nanmax(depth)} median={np.nanmedian(depth)}")
    print(f"cam_z={cam_z} center_as_m_surface_z={cam_z - float(c) * 100.0}")
    print(f"center_as_cm_surface_z={cam_z - float(c)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
