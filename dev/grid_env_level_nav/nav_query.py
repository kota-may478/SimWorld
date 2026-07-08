#!/usr/bin/env python3
"""UnrealCV vbp wrapper for BP_NavQueryService (NavProjectPoint / NavFindPath)."""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

_THIS_DIR = Path(__file__).resolve().parent
_GEH_DIR = _THIS_DIR.parent / "grid_env_hri"
for _p in (_THIS_DIR, _GEH_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import grid_env_hri_simulation as geh  # noqa: E402
from level_coords import NAV_PROJECT_PROBE_Z_CM, local_xy_to_world  # noqa: E402

NAV_QUERY_ACTOR = "NavQueryService"
DEFAULT_PROBE_LOCAL_XY = (1500.0, 1500.0)
DEFAULT_PROBE_WORLD_XYZ = (
    local_xy_to_world(*DEFAULT_PROBE_LOCAL_XY)[0],
    local_xy_to_world(*DEFAULT_PROBE_LOCAL_XY)[1],
    NAV_PROJECT_PROBE_Z_CM,
)
NAV_QUERY_BP_PATH = "/Game/CustomAssets/BP_NavQueryService.BP_NavQueryService_C"
NAV_QUERY_SPAWN_SETTLE_S = 0.25
NAV_QUERY_DESTROY_SETTLE_S = 1.0

WorldXYZ = Tuple[float, float, float]


def _unwrap_return_value(payload: dict) -> dict:
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
    return payload


def parse_nav_json(raw_response: object) -> dict:
    if isinstance(raw_response, dict):
        return _unwrap_return_value(raw_response)
    if isinstance(raw_response, str):
        text = raw_response.strip()
        if not text or text.lower().startswith("error"):
            return {"error": text}
        try:
            return _unwrap_return_value(json.loads(text))
        except json.JSONDecodeError:
            return {"error": text}
    return {}


def _vbp_nav(ucv, actor: str, method: str, *args: object) -> dict:
    parts = " ".join(str(a) for a in args)
    cmd = f"vbp {actor} {method} {parts}".strip()
    raw = geh._ue_request(ucv, cmd, timeout_s=30.0)
    if raw is None:
        return {"error": "ue_request_failed"}
    try:
        return parse_nav_json(json.loads(raw))
    except json.JSONDecodeError:
        return parse_nav_json(raw)


def nav_find_path(
    ucv,
    actor: str,
    start_xyz: WorldXYZ,
    end_xyz: WorldXYZ,
    *,
    agent_radius_cm: Optional[float] = None,
) -> dict:
    sx, sy, sz = start_xyz
    ex, ey, ez = end_xyz
    if agent_radius_cm is not None:
        return _vbp_nav(
            ucv,
            actor,
            "NavFindPathWithRadius",
            sx,
            sy,
            sz,
            ex,
            ey,
            ez,
            agent_radius_cm,
        )
    return _vbp_nav(ucv, actor, "NavFindPath", sx, sy, sz, ex, ey, ez)


def nav_find_path_validated(
    ucv,
    actor: str,
    start_xyz: WorldXYZ,
    end_xyz: WorldXYZ,
    *,
    agent_radius_cm: float,
    min_center_clearance_cm: float,
    resample_spacing_cm: float,
) -> dict:
    sx, sy, sz = start_xyz
    ex, ey, ez = end_xyz
    return _vbp_nav(
        ucv,
        actor,
        "NavFindPathValidated",
        sx,
        sy,
        sz,
        ex,
        ey,
        ez,
        agent_radius_cm,
        min_center_clearance_cm,
        resample_spacing_cm,
    )


def nav_register_planning_obstacle(
    ucv,
    actor: str,
    obstacle_id: str,
    center_x: float,
    center_y: float,
    half_x: float,
    half_y: float,
) -> dict:
    return _vbp_nav(
        ucv,
        actor,
        "NavRegisterPlanningObstacle",
        obstacle_id,
        center_x,
        center_y,
        half_x,
        half_y,
    )


def get_actor_bounds(ucv, actor: str, target_actor: str) -> dict:
    """Return {ok, cx, cy, cz, half_x, half_y, half_z} for target_actor."""
    return _vbp_nav(ucv, actor, "GetActorBoundsJson", target_actor)


def nav_register_box_obstacle(
    ucv,
    actor: str,
    obstacle_id: str,
    center_xyz: WorldXYZ,
    half_extents_xyz: WorldXYZ,
) -> dict:
    cx, cy, cz = center_xyz
    hx, hy, hz = half_extents_xyz
    return _vbp_nav(
        ucv,
        actor,
        "NavRegisterBoxObstacle",
        obstacle_id,
        cx,
        cy,
        cz,
        hx,
        hy,
        hz,
    )


def nav_clear_box_obstacles(ucv, actor: str) -> dict:
    return _vbp_nav(ucv, actor, "NavClearBoxObstacles")


def nav_rebuild(ucv, actor: str) -> dict:
    return _vbp_nav(ucv, actor, "NavRebuild")


def nav_rebuild_dirty_region(
    ucv,
    actor: str,
    min_xyz: WorldXYZ,
    max_xyz: WorldXYZ,
    *,
    margin_cm: float,
) -> dict:
    min_x, min_y, min_z = min_xyz
    max_x, max_y, max_z = max_xyz
    return _vbp_nav(
        ucv,
        actor,
        "NavRebuildDirtyRegion",
        min_x,
        min_y,
        min_z,
        max_x,
        max_y,
        max_z,
        margin_cm,
    )


def nav_local_rebuild_api_available(ucv, actor: str) -> bool:
    """True when NavRebuildDirtyRegion is compiled in."""
    raw = nav_rebuild_dirty_region(
        ucv,
        actor,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        margin_cm=0.0,
    )
    if raw.get("ok"):
        return True
    err = str(raw.get("error", "")).lower()
    if "invalid_dirty_region" in err:
        return True
    if (
        "argument invalid" in err
        or "not found" in err
        or "unknown" in err
    ):
        return False
    return False


def nav_runtime_api_available(ucv, actor: str) -> bool:
    """True when extended NavQueryService API (GetActorBoundsJson) is compiled in."""
    raw = get_actor_bounds(ucv, actor, "__nav_api_probe__")
    if raw.get("ok"):
        return True
    err = str(raw.get("error", "")).lower()
    if "actor_not_found" in err:
        return True
    if "not found" in err or "unknown" in err or "argument invalid" in err:
        return False
    return False


def nav_validated_api_available(ucv, actor: str) -> bool:
    """True when NavFindPathValidated / NavRegisterPlanningObstacle are compiled in."""
    raw = nav_register_planning_obstacle(
        ucv, actor, "__nav_validated_api_probe__", 0.0, 0.0, 1.0, 1.0
    )
    if raw.get("ok"):
        return True
    err = str(raw.get("error", "")).lower()
    if (
        "argument invalid" in err
        or "not found" in err
        or "unknown" in err
    ):
        return False
    return False


def nav_project_point(ucv, actor: str, x_cm: float, y_cm: float, z_cm: float) -> dict:
    return _vbp_nav(ucv, actor, "NavProjectPoint", x_cm, y_cm, z_cm)


def nav_is_reachable(
    ucv,
    actor: str,
    start_xyz: WorldXYZ,
    end_xyz: WorldXYZ,
) -> dict:
    sx, sy, sz = start_xyz
    ex, ey, ez = end_xyz
    return _vbp_nav(ucv, actor, "NavIsReachable", sx, sy, sz, ex, ey, ez)


def path_points_xy(result: dict) -> List[Tuple[float, float]]:
    if not result.get("ok"):
        return []
    points: List[Tuple[float, float]] = []
    for pt in result.get("points", []):
        if isinstance(pt, dict):
            points.append((float(pt["x"]), float(pt["y"])))
    return points


def path_points_xyz(result: dict) -> List[WorldXYZ]:
    if not result.get("ok"):
        return []
    points: List[WorldXYZ] = []
    for pt in result.get("points", []):
        if isinstance(pt, dict):
            points.append((float(pt["x"]), float(pt["y"]), float(pt.get("z", 0.0))))
    return points


def _actor_present(ucv, name: str) -> bool:
    return name in geh.actor_names(ucv)


def find_nav_query_actor(ucv) -> Optional[str]:
    """Prefer Actor Label ``NavQueryService``; else first ``BP_NavQueryService_C_*``."""
    names = geh.actor_names(ucv)
    if NAV_QUERY_ACTOR in names:
        return NAV_QUERY_ACTOR
    candidates = sorted(
        n for n in names if n.startswith("BP_NavQueryService_C")
    )
    return candidates[0] if candidates else None


def _probe_responds(ucv, actor: str, x: float, y: float, z: float) -> bool:
    """True when the actor answers vbp (NavMesh may still reject the probe point)."""
    raw = nav_project_point(ucv, actor, x, y, z)
    err = str(raw.get("error", ""))
    if err and (
        "Invalid" in err
        or "not found" in err.lower()
        or "can not find actor" in err.lower()
        or err in ("no_world", "no_navsys")
    ):
        return False
    return "ok" in raw


def destroy_nav_query_service(ucv, actor: str = NAV_QUERY_ACTOR) -> None:
    if not _actor_present(ucv, actor):
        return
    geh._ue_request(ucv, f"vset /object/{actor}/destroy", timeout_s=20.0)
    geh.wait_until_actor_gone(ucv, actor, timeout_s=10.0)
    geh._prepare_ue_spawn(ucv)
    if NAV_QUERY_DESTROY_SETTLE_S > 0:
        time.sleep(NAV_QUERY_DESTROY_SETTLE_S)


def ensure_nav_query_service(
    ucv,
    *,
    actor: str = NAV_QUERY_ACTOR,
    probe_xyz: WorldXYZ = DEFAULT_PROBE_WORLD_XYZ,
    force_respawn: bool = False,
) -> Tuple[bool, str]:
    """Return (ok, actor_name). Reuses level-placed or existing PIE actor."""
    if not force_respawn:
        existing = find_nav_query_actor(ucv)
        if existing and _probe_responds(ucv, existing, *probe_xyz):
            print(f"[NavQuery] reusing existing {existing!r}")
            return True, existing

    if force_respawn and _actor_present(ucv, actor):
        destroy_nav_query_service(ucv, actor)

    if _actor_present(ucv, actor):
        print(f"[NavQuery] {actor!r} still listed after destroy — reuse")
        return True, actor

    geh.wait_until_actor_gone(ucv, actor, timeout_s=12.0)
    geh._prepare_ue_spawn(ucv)

    if not geh.spawn_bp(ucv, NAV_QUERY_BP_PATH, actor):
        fallback = find_nav_query_actor(ucv)
        if fallback and _probe_responds(ucv, fallback, *probe_xyz):
            print(f"[NavQuery] spawn reported fail but {fallback!r} exists — reuse")
            return True, fallback
        return False, actor

    if NAV_QUERY_SPAWN_SETTLE_S > 0:
        time.sleep(NAV_QUERY_SPAWN_SETTLE_S)
    try:
        ucv.tick()
    except Exception:
        pass
    if _probe_responds(ucv, actor, *probe_xyz):
        return True, actor
    fallback = find_nav_query_actor(ucv)
    if fallback and _probe_responds(ucv, fallback, *probe_xyz):
        return True, fallback
    return False, actor
