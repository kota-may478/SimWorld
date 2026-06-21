#!/usr/bin/env python3
"""Depth + object_mask egocentric object perception for SpotDog."""

from __future__ import annotations

import math
from dataclasses import dataclass
from io import BytesIO
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from prop_placement import PlacementRegistry, PropPlacement

DEFAULT_FOV_DEG = 90.0
DEFAULT_MIN_MASK_PIXELS = 48
DEFAULT_COLOR_TOLERANCE = 6
DEFAULT_LIT_COLOR_TOLERANCE = 20
LIT_DEPTH_MIN_M = 0.8
LIT_DEPTH_MAX_M = 25.0
MAX_LIT_FRAME_FRACTION = 0.22


@dataclass(frozen=True)
class PerceptionConfig:
    fov_deg: float = DEFAULT_FOV_DEG
    min_mask_pixels: int = DEFAULT_MIN_MASK_PIXELS
    color_tolerance: int = DEFAULT_COLOR_TOLERANCE
    lit_color_tolerance: int = DEFAULT_LIT_COLOR_TOLERANCE
    allow_lit_fallback: bool = False
    camera_offset_forward_cm: float = 22.0
    camera_height_cm: float = 45.0
    camera_pitch_deg: float = -5.0


@dataclass(frozen=True)
class ObjectEstimate:
    prop_type_id: str
    slot_id: str
    distance_m: float
    bearing_deg: float
    mask_pixels: int
    confidence: float


def depth_npy_to_meters(depth: np.ndarray) -> np.ndarray:
    """Convert UnrealCV depth npy to meters (handles cm vs m heuristics)."""
    out = depth.astype(np.float64, copy=True)
    finite = np.isfinite(out) & (out > 0.0)
    # Sky / invalid sentinel in Level scans
    out[finite & (out > 5000.0)] = np.nan
    # Values typically < 20 are meters; larger values are cm in SpotDog demos.
    meter_mask = finite & (out < 20.0)
    cm_mask = finite & ~meter_mask
    out[meter_mask] = out[meter_mask]
    out[cm_mask] = out[cm_mask] / 100.0
    return out


def pixel_bearing_deg(center_x: float, frame_w: int, fov_deg: float) -> float:
    norm = (center_x - 0.5 * frame_w) / (0.5 * frame_w)
    return float(norm * (fov_deg * 0.5))


def _mask_for_bgr(
    mask_bgr: np.ndarray,
    color_bgr: Sequence[int],
    tolerance: int,
) -> np.ndarray:
    """Per-channel tolerance match (UnrealCV GT tutorial style), on BGR image."""
    if mask_bgr.ndim != 3 or mask_bgr.shape[2] < 3:
        return np.zeros((0, 0), dtype=bool)
    target = np.array(color_bgr[:3], dtype=np.int16)
    region = np.ones(mask_bgr.shape[:2], dtype=bool)
    for ch in range(3):
        lo = target[ch] - tolerance
        hi = target[ch] + tolerance
        region &= (mask_bgr[..., ch].astype(np.int16) >= lo) & (
            mask_bgr[..., ch].astype(np.int16) <= hi
        )
    return region


def _bbox_from_mask(mask: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
    ys, xs = np.where(mask)
    if xs.size == 0:
        return None
    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    return x0, y0, x1 - x0 + 1, y1 - y0 + 1


def _centroid_x_from_mask(mask: np.ndarray) -> Optional[float]:
    ys, xs = np.where(mask)
    if xs.size == 0:
        return None
    return float(xs.mean())


def estimate_depth_m_at_bbox(depth_m: np.ndarray, bbox: Tuple[int, int, int, int]) -> Optional[float]:
    x, y, w, h = bbox
    if w <= 0 or h <= 0:
        return None
    cx = int(x + 0.5 * w)
    cy = int(y + 0.60 * h)
    rx = max(2, int(0.14 * w))
    ry = max(2, int(0.14 * h))
    x0 = max(0, cx - rx)
    x1 = min(depth_m.shape[1], cx + rx)
    y0 = max(0, cy - ry)
    y1 = min(depth_m.shape[0], cy + ry)
    roi = depth_m[y0:y1, x0:x1]
    valid = roi[np.isfinite(roi) & (roi > 0.05) & (roi < 80.0)]
    if valid.size < 8:
        return None
    return float(np.percentile(valid, 35))


def slant_range_to_horizontal_m(
    slant_m: float,
    bearing_deg: float,
    *,
    camera_height_m: float,
    camera_pitch_deg: float,
) -> float:
    """Approximate horizontal ground distance from camera slant range."""
    pitch_rad = math.radians(camera_pitch_deg)
    bearing_rad = math.radians(bearing_deg)
    along_ray = slant_m * math.cos(pitch_rad)
    horiz = along_ray * math.cos(bearing_rad)
    if horiz <= 0.05:
        return max(slant_m * 0.5, 0.05)
    return horiz


def _center_roi_mask(h: int, w: int) -> np.ndarray:
    roi = np.zeros((h, w), dtype=bool)
    roi[h // 4 : 3 * h // 4, w // 4 : 3 * w // 4] = True
    return roi


def _estimate_from_region(
    prop: PropPlacement,
    region: np.ndarray,
    depth_m: np.ndarray,
    cfg: PerceptionConfig,
    *,
    max_frame_fraction: Optional[float] = None,
) -> Optional[ObjectEstimate]:
    h, w = depth_m.shape[:2]
    pixels = int(region.sum())
    if max_frame_fraction is not None and pixels > max_frame_fraction * h * w:
        region = region & _center_roi_mask(h, w)
        pixels = int(region.sum())
    if pixels < cfg.min_mask_pixels:
        return None
    bbox = _bbox_from_mask(region)
    if bbox is None:
        return None
    x, y, bw, bh = bbox
    center_x = _centroid_x_from_mask(region)
    if center_x is None:
        center_x = x + 0.5 * bw
    bearing = pixel_bearing_deg(center_x, w, cfg.fov_deg)
    if abs(bearing) > cfg.fov_deg * 0.5 + 0.5:
        return None
    if max_frame_fraction is not None:
        if abs(center_x - 0.5 * w) > 0.38 * w:
            return None
    slant_m = estimate_depth_m_at_bbox(depth_m, bbox)
    if slant_m is None:
        return None
    horiz_m = slant_range_to_horizontal_m(
        slant_m,
        bearing,
        camera_height_m=cfg.camera_height_cm / 100.0,
        camera_pitch_deg=cfg.camera_pitch_deg,
    )
    # Camera is forward of robot center; adjust to robot-frame ground distance.
    bearing_rad = math.radians(bearing)
    horiz_m += (cfg.camera_offset_forward_cm / 100.0) * math.cos(bearing_rad)
    horiz_m = max(horiz_m, 0.05)
    confidence = min(1.0, pixels / max(cfg.min_mask_pixels * 4, 1))
    return ObjectEstimate(
        prop_type_id=prop.prop_type_id,
        slot_id=prop.slot_id,
        distance_m=horiz_m,
        bearing_deg=bearing,
        mask_pixels=pixels,
        confidence=confidence,
    )


def detect_objects(
    mask_bgr: np.ndarray,
    depth_m: np.ndarray,
    registry: PlacementRegistry,
    *,
    lit_bgr: Optional[np.ndarray] = None,
    only_prop_type_id: Optional[str] = None,
    config: Optional[PerceptionConfig] = None,
) -> List[ObjectEstimate]:
    cfg = config or PerceptionConfig()
    props = registry.props
    if only_prop_type_id is not None:
        props = tuple(p for p in props if p.prop_type_id == only_prop_type_id)
    estimates: List[ObjectEstimate] = []
    for prop in props:
        mask = _mask_for_bgr(mask_bgr, prop.detection_bgr(), cfg.color_tolerance)
        est = _estimate_from_region(prop, mask, depth_m, cfg)
        if est is not None:
            estimates.append(est)
            continue
        if not cfg.allow_lit_fallback:
            continue
        lit_color = prop.detection_lit_bgr()
        if lit_bgr is None or lit_color is None:
            continue
        depth_gate = np.isfinite(depth_m) & (depth_m >= LIT_DEPTH_MIN_M) & (depth_m <= LIT_DEPTH_MAX_M)
        lit_region = _mask_for_bgr(lit_bgr, lit_color, cfg.lit_color_tolerance) & depth_gate
        est_lit = _estimate_from_region(
            prop,
            lit_region,
            depth_m,
            cfg,
            max_frame_fraction=MAX_LIT_FRAME_FRACTION,
        )
        if est_lit is not None:
            estimates.append(est_lit)
    return _resolve_detection_conflicts(estimates)


def _resolve_detection_conflicts(
    estimates: List[ObjectEstimate],
    *,
    bearing_sep_deg: float = 8.0,
) -> List[ObjectEstimate]:
    """When two props share similar bearing, keep the higher mask-pixel match."""
    if len(estimates) < 2:
        return estimates
    kept: List[ObjectEstimate] = []
    for est in sorted(estimates, key=lambda e: e.mask_pixels, reverse=True):
        conflict = False
        for other in kept:
            if abs(est.bearing_deg - other.bearing_deg) < bearing_sep_deg:
                conflict = True
                break
        if not conflict:
            kept.append(est)
    return kept


def estimates_by_prop_type(estimates: Sequence[ObjectEstimate]) -> Dict[str, ObjectEstimate]:
    return {e.prop_type_id: e for e in estimates}


def fetch_depth_npy(ucv, camera_id: int) -> Optional[np.ndarray]:
    cmd = f"vget /camera/{camera_id}/depth npy"
    try:
        with ucv.lock:
            payload = ucv.client.request(cmd)
        depth = np.load(BytesIO(payload))
        if depth.ndim != 2:
            return None
        return depth
    except Exception:
        return None
