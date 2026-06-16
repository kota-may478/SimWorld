#!/usr/bin/env python3
"""Compare ThirdPerson vs FusionCam pose and object_mask (PIE)."""

from __future__ import annotations

import sys
from collections import Counter
from io import BytesIO
from pathlib import Path

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
ROOT = THIS_DIR.parent.parent
for p in (
    str(ROOT),
    str(THIS_DIR),
    str(ROOT / "dev" / "grid_env_hri"),
    str(ROOT / "dev" / "grid_env_10k"),
    str(ROOT / "dev" / "grid_env_level_nav"),
):
    if p not in sys.path:
        sys.path.insert(0, p)

import grid_env_10k as g10k  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
from mask_calibration import _standoff_pose  # noqa: E402
from pie_safety import soft_teleport_robot, tick_settle  # noqa: E402
from prop_placement import load_registry  # noqa: E402
from robot_sensor import (  # noqa: E402
    FUSION_CAM_NAME_SUBSTR,
    MASK_BG_BGR,
    list_camera_names,
    resolve_sensor_camera_id,
    restore_editor_viewmode_lit,
)


def _top_bgr(mask: np.ndarray, n: int = 5) -> list:
    flat = mask.reshape(-1, 3)
    sums = flat.sum(axis=1)
    valid = flat[sums > 30]
    if valid.size == 0:
        return []
    quant = (valid // 4) * 4
    keys = [tuple(int(v) for v in row) for row in quant]
    return Counter(keys).most_common(n)


def _fetch_mask(ucv, cam_id: int) -> np.ndarray | None:
    import PIL.Image

    cmd = f"vget /camera/{cam_id}/object_mask png"
    try:
        with ucv.lock:
            payload = ucv.client.request(cmd)
        img = np.asarray(PIL.Image.open(BytesIO(payload)))
        if img.ndim == 3 and img.shape[2] == 4:
            img = img[:, :, :3]
        return img
    finally:
        restore_editor_viewmode_lit(ucv)


def _non_bg_fraction(mask: np.ndarray, tol: int = 8) -> float:
    bg = np.array(MASK_BG_BGR, dtype=np.int16)
    diff = np.abs(mask.astype(np.int16) - bg)
    non_bg = np.any(diff > tol, axis=2)
    return float(non_bg.mean())


def main() -> int:
    registry = load_registry()
    prop = registry.visit_order_props()[0]
    ucv, _ = g10k.ensure_connection()
    ok, robot = lnr.soft_reset_level_spotdog(ucv, registry.spotdog_spawn_local_cm)
    if not ok:
        print("no robot")
        return 1
    loc, yaw = _standoff_pose(prop)
    soft_teleport_robot(ucv, robot, loc, yaw)
    tick_settle(ucv, settle_s=0.5, ticks=2)

    names = list_camera_names(ucv)
    fusion_id = resolve_sensor_camera_id(ucv)
    third_id = next((i for i, n in enumerate(names) if "ThirdPerson" in n), 1)
    print("cameras:", list(enumerate(names)))
    print(f"fusion_id={fusion_id} third_id={third_id} prop={prop.slot_id}")

    for cam_id in (third_id, fusion_id):
        name = names[cam_id] if cam_id < len(names) else "?"
        loc3 = ucv.get_camera_location(cam_id)
        rot = ucv.get_camera_rotation(cam_id)
        print(f"\n--- cam {cam_id} ({name}) ---")
        print(f"  location={loc3} rotation={rot}")
        try:
            mask = _fetch_mask(ucv, cam_id)
            print(f"  mask shape={mask.shape} non_bg%={_non_bg_fraction(mask)*100:.2f}")
            print(f"  top BGR={_top_bgr(mask)}")
            canonical = prop.detection_bgr()
            hit = int(np.sum(np.all(np.abs(mask.astype(int) - np.array(canonical)) <= 6, axis=2)))
            print(f"  canonical {canonical} pixels (tol 6)={hit}")
        except Exception as exc:
            print(f"  mask error: {exc}")

    if fusion_id < len(names) and FUSION_CAM_NAME_SUBSTR in names[fusion_id]:
        print("\nFusionCam detected — mask should use head camera after UE fix.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
