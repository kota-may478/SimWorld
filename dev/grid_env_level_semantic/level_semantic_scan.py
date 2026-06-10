#!/usr/bin/env python3
"""Semantic labeling on Level geometry (PIE).

**Approach C (preferred):** ``BP_SemanticCollisionProbe.ProbePointHit(X,Y,Z,RadiusCm)``
sphere sweep (radius = half cube edge = 15 cm for 0.3 m cells):

1. At initial block bottom: cube center, r = 0.15 m → wall if hit.
2. Else same (gx, gy) with bottom −0.30 m: cube center, r = 0.15 m → floor if hit, else air.

Height scan: lower −30 cm while all air; on first wall → initial bottom = that height + 30 cm.

**Depth fallback:** nadir camera when the collision probe BP is unavailable.
"""

from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

BlockIndex = Tuple[int, int]

_SEM_DIR = Path(__file__).resolve().parent.parent / "grid_env_10k_semantic"
if str(_SEM_DIR) not in sys.path:
    sys.path.insert(0, str(_SEM_DIR))

from block_semantic_scan import BlockSemantic, classify_semantic  # noqa: E402

import level_collision_probe as lcp  # noqa: E402
from level_region import HEIGHT_STEP_CM  # noqa: E402

DEPTH_CAMERA_ID = 0
DEPTH_CAPTURE_SETTLE_S = 0.18
# UnrealCV depth npy on Level is slant range in **cm** (65504 ≈ sky / no hit).
DEPTH_MIN_VALID_CM = 50.0
DEPTH_SKY_THRESHOLD_CM = 60_000.0
# Per-cell labels: center pixel only (sky → air). Wider radii are calibration-only.
DEPTH_CENTER_PATCH_RADIUS_PX = 0
# Calibration fallback: frame min only when setting region Z0, never for per-cell labels.
DEPTH_CALIBRATION_PATCH_RADIUS_PX = 32
DEPTH_PROBE_CLEARANCE_CM = 400.0
DEPTH_Z_TOLERANCE_CM = 3.0
NADIR_PITCH_DEG = -90.0


@dataclass(frozen=True)
class _CameraState:
    location: Tuple[float, float, float]
    rotation: Tuple[float, float, float]
    fov: float


def parse_collision_counts(raw_response: object) -> dict:
    if isinstance(raw_response, dict):
        return raw_response
    if isinstance(raw_response, str):
        text = raw_response.strip()
        if text.startswith("error"):
            return {}
        return json.loads(text)
    return {}


def _parse_vec3(raw_value) -> Tuple[float, float, float]:
    if isinstance(raw_value, str):
        tokens = raw_value.replace(",", " ").split()
        return (float(tokens[0]), float(tokens[1]), float(tokens[2]))
    return (float(raw_value[0]), float(raw_value[1]), float(raw_value[2]))


def _save_camera_state(ucv, camera_id: int) -> _CameraState:
    loc = _parse_vec3(ucv.get_camera_location(camera_id))
    rot = _parse_vec3(ucv.get_camera_rotation(camera_id))
    fov = float(ucv.get_camera_fov(camera_id))
    return _CameraState(location=loc, rotation=rot, fov=fov)


def _restore_camera_state(ucv, camera_id: int, state: _CameraState) -> None:
    # get_camera_rotation returns (yaw, pitch, roll); set expects (pitch, yaw, roll).
    yaw, pitch, roll = state.rotation
    ucv.set_camera_location(camera_id, state.location)
    ucv.set_camera_rotation(camera_id, (pitch, yaw, roll))
    ucv.set_camera_fov(camera_id, state.fov)


def _fetch_depth_npy(ucv, camera_id: int) -> np.ndarray:
    cmd = f"vget /camera/{camera_id}/depth npy"
    with ucv.lock:
        payload = ucv.client.request(cmd)
    depth = np.load(BytesIO(payload))
    if depth.ndim != 2:
        raise ValueError(f"expected 2D depth, got shape {depth.shape}")
    return depth.astype(np.float32, copy=False)


def _is_valid_depth_cm(value: float, *, cam_z_cm: float) -> bool:
    """Reject sky and non-finite samples. Nadir slant range must be below the camera."""
    if not math.isfinite(value):
        return False
    if value < DEPTH_MIN_VALID_CM or value >= DEPTH_SKY_THRESHOLD_CM:
        return False
    if value >= cam_z_cm - DEPTH_MIN_VALID_CM:
        return False
    return True


def _min_valid_depth_in_center_patch(
    depth: np.ndarray,
    *,
    cam_z_cm: float,
    radius_px: int,
) -> Optional[float]:
    """Minimum slant range [cm] in a square patch around the image center."""
    h, w = depth.shape
    cy, cx = h // 2, w // 2
    r = max(0, int(radius_px))
    best: Optional[float] = None
    for pv in range(max(0, cy - r), min(h, cy + r + 1)):
        for pu in range(max(0, cx - r), min(w, cx + r + 1)):
            depth_cm = float(depth[pv, pu])
            if not _is_valid_depth_cm(depth_cm, cam_z_cm=cam_z_cm):
                continue
            if best is None or depth_cm < best:
                best = depth_cm
    return best


def _surface_z_from_nadir_center(
    depth: np.ndarray,
    *,
    cam_z_cm: float,
    patch_radius_px: int = DEPTH_CENTER_PATCH_RADIUS_PX,
) -> Optional[float]:
    """World Z [cm] of the surface directly below camera XY (nadir column only).

    Uses the nearest valid depth in a small center patch. Does **not** fall back to
    frame-wide minimum — that picks oblique FOV hits from nearby rooftops and makes
    every cliff-edge cell ``floor``.
    """
    depth_cm = _min_valid_depth_in_center_patch(
        depth,
        cam_z_cm=cam_z_cm,
        radius_px=patch_radius_px,
    )
    if depth_cm is None:
        return None
    surface_z = cam_z_cm - depth_cm
    if not math.isfinite(surface_z) or surface_z < 0.0 or surface_z > cam_z_cm:
        return None
    return surface_z


def _surface_z_calibration_fallback(
    depth: np.ndarray,
    *,
    cam_z_cm: float,
) -> Optional[float]:
    """One-shot Z0 helper: center pixel, wider patch, then full frame (last resort)."""
    for radius in (
        0,
        DEPTH_CALIBRATION_PATCH_RADIUS_PX,
    ):
        z = _surface_z_from_nadir_center(depth, cam_z_cm=cam_z_cm, patch_radius_px=radius)
        if z is not None:
            return z
    depth_cm = _min_valid_depth_in_center_patch(
        depth,
        cam_z_cm=cam_z_cm,
        radius_px=max(depth.shape) // 2,
    )
    if depth_cm is None:
        return None
    return cam_z_cm - depth_cm


def surface_z_cm_at_xy(
    ucv,
    x_cm: float,
    y_cm: float,
    *,
    sample_cam_z_cm: float,
) -> Optional[float]:
    """Topmost surface world Z [cm] at (x,y), or None if depth invalid."""
    camera_id = DEPTH_CAMERA_ID
    cam_z = sample_cam_z_cm
    state = _save_camera_state(ucv, camera_id)
    try:
        ucv.set_camera_location(camera_id, (x_cm, y_cm, cam_z))
        ucv.set_camera_rotation(camera_id, (NADIR_PITCH_DEG, 0.0, 0.0))
        if DEPTH_CAPTURE_SETTLE_S > 0:
            time.sleep(DEPTH_CAPTURE_SETTLE_S)
        try:
            ucv.tick()
        except Exception:
            pass
        depth = _fetch_depth_npy(ucv, camera_id)
        return _surface_z_from_nadir_center(depth, cam_z_cm=cam_z)
    finally:
        _restore_camera_state(ucv, camera_id, state)


def reference_surface_z_cm(
    ucv,
    x_cm: float,
    y_cm: float,
    *,
    sample_cam_z_cm: float,
) -> Optional[float]:
    """Surface Z for height calibration (allows wider patch / frame min fallback)."""
    camera_id = DEPTH_CAMERA_ID
    cam_z = sample_cam_z_cm
    state = _save_camera_state(ucv, camera_id)
    try:
        ucv.set_camera_location(camera_id, (x_cm, y_cm, cam_z))
        ucv.set_camera_rotation(camera_id, (NADIR_PITCH_DEG, 0.0, 0.0))
        if DEPTH_CAPTURE_SETTLE_S > 0:
            time.sleep(DEPTH_CAPTURE_SETTLE_S)
        try:
            ucv.tick()
        except Exception:
            pass
        depth = _fetch_depth_npy(ucv, camera_id)
        return _surface_z_calibration_fallback(depth, cam_z_cm=cam_z)
    finally:
        _restore_camera_state(ucv, camera_id, state)


def reference_surface_z_for_cells(
    ucv,
    cells: List[BlockIndex],
    *,
    cell_center_xy_cm_fn: Callable[[int, int], Tuple[float, float]],
    sample_cam_z_cm: float,
) -> Optional[float]:
    """Highest reference surface among cells (for shared Z0 on sloped sites)."""
    best: Optional[float] = None
    for gx, gy in cells:
        x_cm, y_cm = cell_center_xy_cm_fn(gx, gy)
        surface_z = reference_surface_z_cm(
            ucv,
            x_cm,
            y_cm,
            sample_cam_z_cm=sample_cam_z_cm,
        )
        if surface_z is None:
            continue
        if best is None or surface_z > best:
            best = surface_z
    return best


def depth_band_hits(
    surface_z_cm: Optional[float],
    bottom_z_cm: float,
    block_height_cm: float,
) -> bool:
    if surface_z_cm is None:
        return False
    tol = DEPTH_Z_TOLERANCE_CM
    z_min = bottom_z_cm - tol
    z_max = bottom_z_cm + block_height_cm + tol
    return z_min <= surface_z_cm <= z_max


def compute_depth_sample_cam_z_cm(z_reference_bottom_cm: float, block_height_cm: float) -> float:
    """Fixed nadir camera height for a height-scan session (do not tie to lowered bottom_z)."""
    return z_reference_bottom_cm + block_height_cm + DEPTH_PROBE_CLEARANCE_CM


def classify_cell_depth(
    ucv,
    x_cm: float,
    y_cm: float,
    *,
    z_initial_bottom_cm: float,
    block_height_cm: float,
    depth_sample_cam_z_cm: float,
) -> Tuple[BlockSemantic, Optional[float]]:
    z_lower_cm = z_initial_bottom_cm - block_height_cm
    surface_z = surface_z_cm_at_xy(
        ucv,
        x_cm,
        y_cm,
        sample_cam_z_cm=depth_sample_cam_z_cm,
    )
    hit_initial = depth_band_hits(surface_z, z_initial_bottom_cm, block_height_cm)
    hit_lower = depth_band_hits(surface_z, z_lower_cm, block_height_cm)
    return classify_semantic(hit_at_z_initial=hit_initial, hit_at_z_lower=hit_lower), surface_z


PROBE_ACTOR = lcp.PROBE_ACTOR


def ensure_collision_probe(ucv, **kwargs) -> Tuple[bool, str]:
    return lcp.ensure_collision_probe(ucv, **kwargs)


def destroy_collision_probe(ucv) -> None:
    lcp.destroy_collision_probe(ucv)


def collision_probe_available(ucv) -> bool:
    return lcp.probe_bp_available(ucv)


def cube_inscribed_probe_radius_cm(block_height_cm: float) -> float:
    """Inscribed sphere radius at cube center [cm] (= half edge = 0.15 m for 0.3 m cell)."""
    return block_height_cm / 2.0


def cube_center_z_cm(z_bottom_cm: float, block_height_cm: float) -> float:
    return z_bottom_cm + block_height_cm / 2.0


def classify_semantic_from_center_tiers(
    *,
    hit_at_initial_center: bool,
    hit_at_lower_center: bool,
) -> BlockSemantic:
    if hit_at_initial_center:
        return "wall"
    if hit_at_lower_center:
        return "floor"
    return "air"


def classify_cell_collision(
    ucv,
    x_cm: float,
    y_cm: float,
    *,
    z_initial_bottom_cm: float,
    block_height_cm: float,
    probe_actor: str = PROBE_ACTOR,
) -> Tuple[BlockSemantic, bool]:
    radius_cm = cube_inscribed_probe_radius_cm(block_height_cm)
    center_z = cube_center_z_cm(z_initial_bottom_cm, block_height_cm)
    hit_high = lcp.probe_point_hit(
        ucv,
        x_cm,
        y_cm,
        center_z,
        actor=probe_actor,
        radius_cm=radius_cm,
    )
    if hit_high:
        return "wall", True
    lower_bottom = z_initial_bottom_cm - HEIGHT_STEP_CM
    lower_center_z = cube_center_z_cm(lower_bottom, block_height_cm)
    hit_low = lcp.probe_point_hit(
        ucv,
        x_cm,
        y_cm,
        lower_center_z,
        actor=probe_actor,
        radius_cm=radius_cm,
    )
    sem = classify_semantic_from_center_tiers(
        hit_at_initial_center=False,
        hit_at_lower_center=hit_low,
    )
    return sem, hit_low


def scan_region_collision(
    ucv,
    cells: List[BlockIndex],
    *,
    cell_center_xy_cm_fn: Callable[[int, int], Tuple[float, float]],
    z_initial_bottom_cm: float,
    block_height_cm: float,
    depth_sample_cam_z_cm: Optional[float] = None,
    progress_every: int = 50,
    manage_probe: bool = True,
    probe_actor: str = PROBE_ACTOR,
    use_collision_probe: Optional[bool] = None,
) -> Tuple[Dict[BlockIndex, BlockSemantic], int]:
    """Scan cells; return (semantics, geometry_hit_count).

    ``geometry_hit_count`` feeds height-scan limits: collision path counts cells
    with any probe hit; depth path counts cells with valid nadir surface Z.
    """
    if use_collision_probe is None:
        use_collision_probe = (
            lcp.probe_bp_available(ucv) if manage_probe else True
        )

    spawned = False
    if use_collision_probe and manage_probe:
        ok, probe_actor = ensure_collision_probe(ucv)
        if not ok:
            raise RuntimeError(
                f"collision probe spawn failed — create {lcp.PROBE_BP_PATH} "
                "(run create_semantic_collision_probe_editor.py in UE Editor)"
            )
        spawned = True

    try:
        if use_collision_probe:
            return _scan_region_collision_probe(
                ucv,
                cells,
                cell_center_xy_cm_fn=cell_center_xy_cm_fn,
                z_initial_bottom_cm=z_initial_bottom_cm,
                block_height_cm=block_height_cm,
                progress_every=progress_every,
                probe_actor=probe_actor,
            )
        return _scan_region_depth(
            ucv,
            cells,
            cell_center_xy_cm_fn=cell_center_xy_cm_fn,
            z_initial_bottom_cm=z_initial_bottom_cm,
            block_height_cm=block_height_cm,
            depth_sample_cam_z_cm=depth_sample_cam_z_cm,
            progress_every=progress_every,
        )
    finally:
        # Session reuse — destroying the probe here causes destroy→respawn PIE crashes.
        pass


def _scan_region_collision_probe(
    ucv,
    cells: List[BlockIndex],
    *,
    cell_center_xy_cm_fn: Callable[[int, int], Tuple[float, float]],
    z_initial_bottom_cm: float,
    block_height_cm: float,
    progress_every: int,
    probe_actor: str,
) -> Tuple[Dict[BlockIndex, BlockSemantic], int]:
    results: Dict[BlockIndex, BlockSemantic] = {}
    geometry_hits = 0
    total = len(cells)
    t0 = time.monotonic()
    for i, (gx, gy) in enumerate(cells, start=1):
        x_cm, y_cm = cell_center_xy_cm_fn(gx, gy)
        sem, had_hit = classify_cell_collision(
            ucv,
            x_cm,
            y_cm,
            z_initial_bottom_cm=z_initial_bottom_cm,
            block_height_cm=block_height_cm,
            probe_actor=probe_actor,
        )
        results[(gx, gy)] = sem
        if had_hit:
            geometry_hits += 1
        if progress_every > 0 and (i % progress_every == 0 or i == total):
            elapsed = time.monotonic() - t0
            print(
                f"[LevelSemanticScan/collision] {i}/{total} "
                f"last=({gx},{gy})->{sem} "
                f"z0={z_initial_bottom_cm:.1f} "
                f"geom_hits={geometry_hits} "
                f"elapsed={elapsed:.1f}s"
            )
    return results, geometry_hits


def _scan_region_depth(
    ucv,
    cells: List[BlockIndex],
    *,
    cell_center_xy_cm_fn: Callable[[int, int], Tuple[float, float]],
    z_initial_bottom_cm: float,
    block_height_cm: float,
    depth_sample_cam_z_cm: Optional[float],
    progress_every: int,
) -> Tuple[Dict[BlockIndex, BlockSemantic], int]:
    cam_z = (
        depth_sample_cam_z_cm
        if depth_sample_cam_z_cm is not None
        else compute_depth_sample_cam_z_cm(z_initial_bottom_cm, block_height_cm)
    )
    results: Dict[BlockIndex, BlockSemantic] = {}
    surface_hits = 0
    total = len(cells)
    t0 = time.monotonic()
    for i, (gx, gy) in enumerate(cells, start=1):
        x_cm, y_cm = cell_center_xy_cm_fn(gx, gy)
        sem, surface_z = classify_cell_depth(
            ucv,
            x_cm,
            y_cm,
            z_initial_bottom_cm=z_initial_bottom_cm,
            block_height_cm=block_height_cm,
            depth_sample_cam_z_cm=cam_z,
        )
        results[(gx, gy)] = sem
        if surface_z is not None:
            surface_hits += 1
        if progress_every > 0 and (i % progress_every == 0 or i == total):
            elapsed = time.monotonic() - t0
            surf_s = f"{surface_z:.1f}" if surface_z is not None else "none"
            print(
                f"[LevelSemanticScan/depth] {i}/{total} "
                f"last=({gx},{gy})->{sem} surface_z={surf_s} "
                f"z0={z_initial_bottom_cm:.1f} "
                f"elapsed={elapsed:.1f}s"
            )
    return results, surface_hits
