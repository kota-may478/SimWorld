#!/usr/bin/env python3
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
    _closest_valid_depth_cm,
    _save_camera_state,
    surface_z_cm_at_xy,
)
from spawn_fixed_height_verify import VERIFY_CENTER_XY_CM  # noqa: E402

CAM_ID = 0
ucv, _ = g10k.ensure_connection()
x, y = VERIFY_CENTER_XY_CM
cam_z = 6930.0

state = _save_camera_state(ucv, CAM_ID)
print("saved rot=", state.rotation)
ucv.set_camera_location(CAM_ID, (x, y, cam_z))
ucv.set_camera_rotation(CAM_ID, (-90.0, 0.0, 0.0))
time.sleep(0.3)
print("after set rot=", ucv.get_camera_rotation(CAM_ID))
try:
    ucv.tick()
except Exception:
    pass
cmd = f"vget /camera/{CAM_ID}/depth npy"
with ucv.lock:
    payload = ucv.client.request(cmd)
depth = np.load(BytesIO(payload))
print("global min", float(np.min(depth)), "patch", _closest_valid_depth_cm(depth, radius_px=8))
z = surface_z_cm_at_xy(ucv, x, y, ref_bottom_z_cm=6500.0, block_height_cm=30.0)
print("surface_z_cm_at_xy=", z)
