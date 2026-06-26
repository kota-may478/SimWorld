#!/usr/bin/env python3
"""Robot pose queries with movement-token cache and optional UE batch vbp."""

from __future__ import annotations

import json
import time
from typing import Any, MutableMapping, Optional, Tuple

from grid_env_10k_pie_patrol import get_pos2d, get_yaw
from metrics import NavTimingAccumulator
from simworld.communicator.unrealcv import UnrealCV

WorldXY = Tuple[float, float]
PoseCache = MutableMapping[str, Any]

POSE_STALE_KEY = "pose_stale"
POSE_BATCH_MODE_KEY = "pose_batch_mode"

VBP_POSE_COMMANDS = (
    "GetPose2dJson",
    "GetNavPoseJson",
)


def init_pose_cache(cache: PoseCache) -> None:
    """Mark pose cache empty until first fetch."""
    cache[POSE_STALE_KEY] = True
    cache.pop("xy", None)
    cache.pop("yaw", None)
    cache.pop(POSE_BATCH_MODE_KEY, None)


def invalidate_robot_pose(cache: Optional[PoseCache], *, reason: str = "") -> None:
    """Movement token: next fetch must contact UE."""
    if cache is None:
        return
    cache[POSE_STALE_KEY] = True
    if reason:
        cache["pose_invalidate_reason"] = reason


def sync_robot_pose_cache(cache: Optional[PoseCache], pos_xy: WorldXY, yaw_deg: float) -> None:
    if cache is None:
        return
    cache["xy"] = pos_xy
    cache["yaw"] = float(yaw_deg)
    cache[POSE_STALE_KEY] = False


def _unwrap_vbp_payload(payload: dict) -> object:
    if "ReturnValue" not in payload:
        return payload
    inner = payload["ReturnValue"]
    if isinstance(inner, dict):
        return inner
    if isinstance(inner, str):
        text = inner.strip()
        if not text:
            return payload
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return {"error": text}
    return inner


def _parse_pose2d_payload(raw: object) -> Optional[Tuple[WorldXY, float]]:
    if raw is None:
        return None
    text = str(raw).strip()
    if not text or text.lower().startswith("error"):
        return None
    try:
        payload: object = json.loads(text)
    except json.JSONDecodeError:
        return None
    if isinstance(payload, dict):
        payload = _unwrap_vbp_payload(payload)
    if not isinstance(payload, dict):
        return None
    if "error" in payload:
        return None
    try:
        x = float(payload.get("x", payload.get("X")))
        y = float(payload.get("y", payload.get("Y")))
        yaw = float(payload.get("yaw", payload.get("Yaw", payload.get("yaw_deg"))))
    except (TypeError, ValueError):
        return None
    return (x, y), yaw


def _fetch_pose_via_vbp(ucv: UnrealCV, robot_name: str) -> Optional[Tuple[WorldXY, float]]:
    for cmd in VBP_POSE_COMMANDS:
        try:
            raw = ucv.client.request(f"vbp {robot_name} {cmd}")
        except (ConnectionError, OSError, ValueError, RuntimeError, AttributeError):
            continue
        parsed = _parse_pose2d_payload(raw)
        if parsed is not None:
            return parsed
    return None


def _fetch_pose_split(
    ucv: UnrealCV,
    robot_name: str,
    nav_timing: Optional[NavTimingAccumulator],
) -> Tuple[WorldXY, float, UnrealCV]:
    t0 = time.perf_counter()
    pos_xy = get_pos2d(ucv, robot_name)
    yaw_deg = get_yaw(ucv, robot_name)
    if nav_timing is not None:
        nav_timing.pose_query_ms += (time.perf_counter() - t0) * 1000.0
        nav_timing.pose_batch_split_fetches += 1
    return pos_xy, yaw_deg, ucv


def fetch_robot_pose2d(
    ucv: UnrealCV,
    robot_name: str,
    nav_timing: Optional[NavTimingAccumulator],
    *,
    force: bool = False,
) -> Tuple[WorldXY, float, UnrealCV, str]:
    """Fetch robot XY+yaw; prefer one vbp round-trip, else location+orientation."""
    t0 = time.perf_counter()
    batch = _fetch_pose_via_vbp(ucv, robot_name)
    if batch is not None:
        pos_xy, yaw_deg = batch
        if nav_timing is not None:
            nav_timing.pose_query_ms += (time.perf_counter() - t0) * 1000.0
            nav_timing.pose_batch_vbp_fetches += 1
        return pos_xy, yaw_deg, ucv, "vbp"

    pos_xy, yaw_deg, ucv_out = _fetch_pose_split(ucv, robot_name, nav_timing)
    return pos_xy, yaw_deg, ucv_out, "split"


def fetch_nav_pose(
    ucv: UnrealCV,
    robot_name: str,
    nav_timing: Optional[NavTimingAccumulator],
    pose_cache: Optional[PoseCache],
    *,
    force: bool = False,
) -> Tuple[WorldXY, float, UnrealCV]:
    """Movement-token pose fetch: skip UE when cache is fresh and robot has not moved."""
    if not force and pose_cache is not None and not pose_cache.get(POSE_STALE_KEY, True):
        xy = pose_cache.get("xy")
        yaw = pose_cache.get("yaw")
        if xy is not None and yaw is not None:
            if nav_timing is not None:
                nav_timing.pose_cache_hits += 1
            return xy, float(yaw), ucv

    pos_xy, yaw_deg, ucv_out, mode = fetch_robot_pose2d(
        ucv,
        robot_name,
        nav_timing,
        force=True,
    )
    sync_robot_pose_cache(pose_cache, pos_xy, yaw_deg)
    if pose_cache is not None:
        pose_cache[POSE_BATCH_MODE_KEY] = mode
    return pos_xy, yaw_deg, ucv_out
