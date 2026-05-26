#!/usr/bin/env python3
"""Top-down depth snapshot for obstacle detection feasibility."""
from __future__ import annotations

import sys
from io import BytesIO
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from path_planning_costmap import build_uniform_costmap
from simworld.communicator.unrealcv import UnrealCV


def raw_depth(ucv, cam_id: int) -> np.ndarray:
    cmd = f"vget /camera/{cam_id}/depth npy"
    with ucv.lock:
        payload = ucv.client.request(cmd)
    return np.load(BytesIO(payload))


def main() -> int:
    origin = (1425.755, -1711.4)
    ground_z = 3873.0
    size_cm = 3000.0
    cx = origin[0] + size_cm * 0.5
    cy = origin[1] + size_cm * 0.5
    cam_z = ground_z + 2500.0

    ucv = UnrealCV(port=9000, ip="172.20.224.1")
    cams = str(ucv.get_cameras()).replace(",", " ").split()
    cam_id = 0
    for token in cams:
        try:
            cam_id = int(token)
            break
        except ValueError:
            continue
    print("camera id", cam_id)
    ucv.set_camera_resolution(cam_id, (400, 400))
    ucv.set_camera_fov(cam_id, 90.0)
    ucv.set_camera_location(cam_id, (cx, cy, cam_z))
    ucv.set_camera_rotation(cam_id, (-90.0, 0.0, 0.0))
    import time

    time.sleep(0.5)
    depth = raw_depth(ucv, cam_id)
    print("depth shape", depth.shape, "dtype", depth.dtype)
    valid = depth[np.isfinite(depth) & (depth > 0.001)]
    print(
        "valid stats min/med/max",
        float(np.min(valid)),
        float(np.median(valid)),
        float(np.max(valid)),
    )
    p90 = float(np.percentile(valid, 90))
    p10 = float(np.percentile(valid, 10))
    obst = valid[valid < p90 - 50.0]
    print("pixels below p90-50:", obst.size, "of", valid.size)
    out = Path(__file__).resolve().parent / "topdown_depth.npy"
    np.save(out, depth)
    print("saved", out)
    ucv.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
