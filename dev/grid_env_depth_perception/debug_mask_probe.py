#!/usr/bin/env python3
"""One-shot mask debug: teleport near prop, dump colors (PIE only)."""

from __future__ import annotations

import sys
from collections import Counter
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

from io import BytesIO

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
from depth_object_perception import (  # noqa: E402
    PerceptionConfig,
    _mask_for_bgr,
    detect_objects,
    depth_npy_to_meters,
    fetch_depth_npy,
)
from mask_calibration import _standoff_pose  # noqa: E402
from pie_safety import soft_teleport_robot, tick_settle  # noqa: E402
from prop_placement import load_registry  # noqa: E402
from robot_sensor import (  # noqa: E402
    configure_sensor_camera,
    fetch_mask_rgb,
    resolve_sensor_camera_id,
    update_sensor_camera_pose,
)
from simworld.communicator.communicator import Communicator  # noqa: E402

try:
    import cv2
except ImportError:
    cv2 = None


def _top_colors(mask: np.ndarray, n: int = 12) -> list:
    flat = mask.reshape(-1, 3)
    sums = flat.sum(axis=1)
    valid = flat[sums > 30]
    if valid.size == 0:
        return []
    quant = (valid // 4) * 4
    keys = [tuple(int(v) for v in row) for row in quant]
    return Counter(keys).most_common(n)


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
    camera_id = resolve_sensor_camera_id(ucv)
    for try_cam in (camera_id, 0, 1):
        cmd = f"vget /camera/{try_cam}/object_mask png"
        try:
            with ucv.lock:
                payload = ucv.client.request(cmd)
            import PIL.Image

            img = np.asarray(PIL.Image.open(BytesIO(payload)))
            if img.shape[2] == 4:
                img = img[:, :, :3]
            tops = _top_colors(img)
            print(f"cam {try_cam} raw RGB top: {tops[:3]}")
        except Exception as exc:
            print(f"cam {try_cam} mask err: {exc}")
    configure_sensor_camera(ucv, camera_id)
    update_sensor_camera_pose(ucv, robot, camera_id)
    tick_settle(ucv, settle_s=0.4, ticks=1)

    def _raw_mask_rgb():
        cmd = f"vget /camera/{camera_id}/object_mask png"
        with ucv.lock:
            payload = ucv.client.request(cmd)
        import PIL.Image

        img = np.asarray(PIL.Image.open(BytesIO(payload)))
        if img.shape[2] == 4:
            img = img[:, :, :3]
        return img

    try:
        geh._ue_request(ucv, "vset /viewmode object_mask", timeout_s=10.0)  # noqa: SLF001
        tick_settle(ucv, settle_s=0.3, ticks=1)
        print("set viewmode object_mask")
    except Exception as exc:
        print(f"viewmode failed: {exc}")

    raw_rgb = _raw_mask_rgb()
    print(f"raw RGB top colors: {_top_colors(raw_rgb)}")
    comm = Communicator(ucv)
    lit = comm.get_camera_observation(camera_id, "lit", mode="direct")
    mask = fetch_mask_rgb(comm, camera_id)
    depth_raw = fetch_depth_npy(ucv, camera_id)
    print(f"robot loc={loc} yaw={yaw}")
    if lit is not None:
        print(f"lit shape={lit.shape} mean={lit.mean():.1f} std={lit.std():.1f} min={lit.min()} max={lit.max()}")
    if mask is None:
        print("mask is None")
        return 1
    print(f"mask shape={mask.shape} dtype={mask.dtype}")
    print(f"canonical detection_bgr={prop.detection_bgr()}")
    for tol in (3, 8, 16, 24):
        m = _mask_for_bgr(mask, prop.detection_bgr(), tol)
        print(f"tolerance {tol}: pixels={int(m.sum())}")
    print("top BGR colors:", _top_colors(mask))
    depth_m = depth_npy_to_meters(depth_raw) if depth_raw is not None else None
    if depth_m is not None:
        print(f"depth shape={depth_m.shape} center={depth_m[depth_m.shape[0]//2, depth_m.shape[1]//2]:.3f}")
        print(f"depth finite%={np.isfinite(depth_m).mean()*100:.1f} min={np.nanmin(depth_m):.3f} max={np.nanmax(depth_m):.3f}")
    est = detect_objects(mask, depth_m, registry, config=PerceptionConfig())
    print("detect_objects:", est)
    out = THIS_DIR / "cache" / "debug_mask.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    if cv2 is not None:
        cv2.imwrite(str(out), mask)
        print(f"wrote {out}")
        if lit is not None:
            lit_out = THIS_DIR / "cache" / "debug_lit.png"
            cv2.imwrite(str(lit_out), lit)
            print(f"wrote {lit_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
