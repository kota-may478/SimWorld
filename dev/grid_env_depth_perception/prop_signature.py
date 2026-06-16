#!/usr/bin/env python3
"""Per-prop visual signatures at standoff (Approach C + lit/depth fallback)."""

from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
_GEH_DIR = _THIS_DIR.parent / "grid_env_hri"
if str(_GEH_DIR) not in sys.path:
    sys.path.insert(0, str(_GEH_DIR))

import grid_env_hri_simulation as geh  # noqa: E402
from depth_object_perception import depth_npy_to_meters, fetch_depth_npy  # noqa: E402
from mask_calibration import _dominant_bgr_in_center, _standoff_pose  # noqa: E402
from pie_safety import POST_TELEPORT_SETTLE_S, soft_teleport_robot, tick_settle  # noqa: E402
from prop_placement import PlacementRegistry, PropPlacement, _copy_prop, save_registry  # noqa: E402
from robot_sensor import (  # noqa: E402
    fetch_lit_bgr,
    update_sensor_camera_pose,
)

BGR = Tuple[int, int, int]
MASK_BG_BGR = (76, 76, 76)
MASK_BG_TOLERANCE = 6
LIT_DEPTH_MIN_M = 1.5
LIT_DEPTH_MAX_M = 12.0


def mask_segmentation_active(mask_bgr: np.ndarray) -> bool:
    """True when object_mask shows non-background labeled pixels."""
    if mask_bgr is None or mask_bgr.size == 0:
        return False
    bg = _mask_for_bgr_simple(mask_bgr, MASK_BG_BGR, MASK_BG_TOLERANCE)
    non_bg = int((~bg).sum())
    return non_bg > mask_bgr.shape[0] * mask_bgr.shape[1] * 0.005


def _mask_for_bgr_simple(mask_bgr: np.ndarray, color_bgr: BGR, tolerance: int) -> np.ndarray:
    target = np.array(color_bgr, dtype=np.int16)
    region = np.ones(mask_bgr.shape[:2], dtype=bool)
    for ch in range(3):
        lo = target[ch] - tolerance
        hi = target[ch] + tolerance
        region &= (mask_bgr[..., ch].astype(np.int16) >= lo) & (
            mask_bgr[..., ch].astype(np.int16) <= hi
        )
    return region


def _dominant_bgr_depth_gated(
    lit_bgr: np.ndarray,
    depth_m: np.ndarray,
    *,
    d_min_m: float = LIT_DEPTH_MIN_M,
    d_max_m: float = LIT_DEPTH_MAX_M,
) -> Optional[BGR]:
    if lit_bgr.ndim != 3 or depth_m.ndim != 2:
        return None
    h, w = depth_m.shape
    gate = np.isfinite(depth_m) & (depth_m >= d_min_m) & (depth_m <= d_max_m)
    y0, y1 = h // 4, 3 * h // 4
    x0, x1 = w // 4, 3 * w // 4
    roi_gate = gate[y0:y1, x0:x1]
    roi = lit_bgr[y0:y1, x0:x1]
    flat = roi[roi_gate]
    if flat.size < 120:
        flat = roi.reshape(-1, 3)
        sums = flat.sum(axis=1)
        flat = flat[sums > 40]
    if flat.size == 0:
        return None
    quant = (flat.astype(np.int16) // 8) * 8
    keys = [tuple(int(v) for v in row) for row in quant]
    color, count = Counter(keys).most_common(1)[0]
    if count < 80:
        return None
    return color  # type: ignore[return-value]


def observe_prop_signature_at_standoff(
    ucv,
    communicator,
    camera_id: int,
    robot_name: str,
    prop: PropPlacement,
) -> PropPlacement:
    """Capture mask/lit signatures facing the prop (~4.5 m standoff)."""
    if prop.world_xyz_cm is None:
        return prop
    loc, yaw = _standoff_pose(prop)
    soft_teleport_robot(ucv, robot_name, loc, yaw)
    tick_settle(ucv, settle_s=POST_TELEPORT_SETTLE_S, ticks=2)
    update_sensor_camera_pose(ucv, robot_name, camera_id)
    tick_settle(ucv, settle_s=0.35, ticks=1)

    from robot_sensor import fetch_mask_rgb  # noqa: WPS433

    mask = fetch_mask_rgb(communicator, camera_id)
    lit = fetch_lit_bgr(communicator, camera_id)
    depth_raw = fetch_depth_npy(ucv, camera_id)
    depth_m = depth_npy_to_meters(depth_raw) if depth_raw is not None else None

    updates: dict = {}
    if mask is not None and mask_segmentation_active(mask):
        bgr = _dominant_bgr_in_center(mask)
        if bgr is not None:
            print(f"[PropSig] {prop.prop_type_id} mask observed BGR={bgr}")
            updates["mask_color_observed_bgr"] = bgr
    elif mask is not None:
        print(f"[PropSig] {prop.prop_type_id} object_mask inactive (gray buffer)")

    if lit is not None and depth_m is not None:
        lit_bgr = _dominant_bgr_depth_gated(lit, depth_m)
        if lit_bgr is not None:
            print(f"[PropSig] {prop.prop_type_id} lit observed BGR={lit_bgr}")
            updates["lit_color_observed_bgr"] = lit_bgr

    if not updates:
        print(f"[PropSig] WARN: no signature for {prop.prop_type_id}")
        return prop
    return _copy_prop(prop, **updates)


def sync_registry_detection_signatures(
    ucv,
    communicator,
    camera_id: int,
    robot_name: str,
    registry: PlacementRegistry,
) -> PlacementRegistry:
    updated: List[PropPlacement] = []
    for prop in registry.props:
        if not geh.actor_exists(ucv, prop.slot_id):
            updated.append(prop)
            continue
        observed = observe_prop_signature_at_standoff(
            ucv, communicator, camera_id, robot_name, prop
        )
        updated.append(observed)
    out = PlacementRegistry(
        version=registry.version,
        seed=registry.seed,
        prop_count=registry.prop_count,
        region_x_max_cm=registry.region_x_max_cm,
        region_y_max_cm=registry.region_y_max_cm,
        exclusion_cm=registry.exclusion_cm,
        spotdog_spawn_local_cm=registry.spotdog_spawn_local_cm,
        props=tuple(updated),
    )
    save_registry(out)
    return out
