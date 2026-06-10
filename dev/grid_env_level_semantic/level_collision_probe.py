#!/usr/bin/env python3
"""Level semantic collision probe via UnrealCV ``vbp`` (Approach C).

Requires ``BP_SemanticCollisionProbe`` (or compatible actor) exposing::

    ProbePointHit(float X, float Y, float Z, float RadiusCm=0) -> JSON string
        {"hit": true/false, "building": N, "object": N}
        RadiusCm <= 0 uses actor default (15 cm = inscribed sphere in 0.3 m cube).

Spawn **once** per PIE session when possible. Avoid destroy+respawn cycles and
physics/scale tweaks on the native probe actor (both can crash SimWorld PIE).
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

_THIS_DIR = Path(__file__).resolve().parent
_GEH_DIR = _THIS_DIR.parent / "grid_env_hri"
for _p in (_THIS_DIR, _GEH_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import grid_env_hri_simulation as geh  # noqa: E402

PROBE_ACTOR = "level_sem_collision_probe"
PROBE_BP_PATH = "/Game/CustomAssets/BP_SemanticCollisionProbe.BP_SemanticCollisionProbe_C"
PROBE_SPAWN_SETTLE_S = 0.25
PROBE_DESTROY_SETTLE_S = 1.2
PROBE_RESPAWN_SETTLE_S = 1.5
# Level rooftop band (cm) — reject landscape / void hits outside this range.
LEVEL_CALIB_Z_MIN_CM = 1000.0
LEVEL_CALIB_Z_MAX_CM = 9000.0


def _unwrap_return_value(payload: dict) -> dict:
    """UnrealCV vbp wraps Blueprint return values as ``{"ReturnValue": ...}``."""
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


def parse_probe_hit(raw_response: object) -> dict:
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


def probe_point_blocks(raw: dict) -> bool:
    if raw.get("error"):
        return False
    if "hit" in raw:
        hit = raw.get("hit")
        if isinstance(hit, str):
            return hit.lower() == "true"
        return bool(hit)
    building = int(raw.get("BuildingCollision", raw.get("building", 0)) or 0)
    obj = int(raw.get("ObjectCollision", raw.get("object", 0)) or 0)
    return (building + obj) > 0


def _vbp_probe_hit(
    ucv,
    actor: str,
    x_cm: float,
    y_cm: float,
    z_cm: float,
    *,
    radius_cm: Optional[float] = None,
) -> dict:
    if radius_cm is not None and radius_cm > 0:
        cmd = f"vbp {actor} ProbePointHit {x_cm} {y_cm} {z_cm} {radius_cm}"
    else:
        cmd = f"vbp {actor} ProbePointHit {x_cm} {y_cm} {z_cm}"
    raw = geh._ue_request(ucv, cmd, timeout_s=10.0)
    return parse_probe_hit(raw)


def _probe_present(ucv) -> bool:
    """Object list membership (location vget can fail on bare C++ actors)."""
    return PROBE_ACTOR in geh.actor_names(ucv)


def _probe_responds(ucv, actor: str) -> bool:
    raw = _vbp_probe_hit(ucv, actor, 6285.0, 1185.0, 6873.5)
    err = str(raw.get("error", ""))
    if err and ("Invalid" in err or "not found" in err.lower()):
        return False
    # Any vbp reply (incl. hit=false JSON) means the actor is alive.
    return (
        "hit" in raw
        or "building" in raw
        or "BuildingCollision" in raw
        or (not err and bool(raw))
    )


def _prepare_safe_probe_spawn(ucv) -> None:
    """Wait for stale probe teardown before spawn (duplicate name crashes PIE)."""
    geh.wait_until_actor_gone(ucv, PROBE_ACTOR, timeout_s=12.0)
    geh._prepare_ue_spawn(ucv)
    if PROBE_RESPAWN_SETTLE_S > 0:
        time.sleep(PROBE_RESPAWN_SETTLE_S)


def ensure_collision_probe(
    ucv,
    *,
    bp_path: str = PROBE_BP_PATH,
    force_respawn: bool = False,
) -> Tuple[bool, str]:
    """Spawn semantic probe BP once; reuse existing actor to avoid PIE crashes.

    Duplicate ``spawn_bp`` on an actor that already exists in the world is a
    known SimWorld PIE crash — never respawn when ``vget /objects`` lists the probe.
    """
    if not force_respawn and (_probe_present(ucv) or _probe_responds(ucv, PROBE_ACTOR)):
        print(f"[CollisionProbe] reusing existing {PROBE_ACTOR!r}")
        return True, PROBE_ACTOR

    if force_respawn and _probe_present(ucv):
        destroy_collision_probe(ucv)

    if _probe_present(ucv):
        print(f"[CollisionProbe] {PROBE_ACTOR!r} still listed after destroy — reuse")
        return True, PROBE_ACTOR

    _prepare_safe_probe_spawn(ucv)

    if not geh.spawn_bp(ucv, bp_path, PROBE_ACTOR):
        if _probe_present(ucv):
            print(f"[CollisionProbe] spawn reported fail but {PROBE_ACTOR!r} exists — reuse")
            return True, PROBE_ACTOR
        return False, PROBE_ACTOR
    # Native probe is vbp-only: do NOT set scale/physics/collision (crashes without mesh).
    if PROBE_SPAWN_SETTLE_S > 0:
        time.sleep(PROBE_SPAWN_SETTLE_S)
    try:
        ucv.tick()
    except Exception:
        pass
    return True, PROBE_ACTOR


def probe_point_hit(
    ucv,
    x_cm: float,
    y_cm: float,
    z_cm: float,
    *,
    actor: str = PROBE_ACTOR,
    radius_cm: Optional[float] = None,
) -> bool:
    """True if blocking geometry overlaps sphere at (x,y,z) with ``radius_cm``."""
    return probe_point_blocks(
        _vbp_probe_hit(ucv, actor, x_cm, y_cm, z_cm, radius_cm=radius_cm),
    )


def destroy_collision_probe(ucv) -> None:
    """Gentle destroy — skip physics/collision disable that crashes bare C++ actors."""
    if not _probe_present(ucv):
        return
    geh._ue_request(ucv, f"vset /object/{PROBE_ACTOR}/destroy", timeout_s=20.0)
    geh.wait_until_actor_gone(ucv, PROBE_ACTOR, timeout_s=10.0)
    geh._prepare_ue_spawn(ucv)
    if PROBE_DESTROY_SETTLE_S > 0:
        time.sleep(PROBE_DESTROY_SETTLE_S)


def probe_bp_available(ucv, *, bp_path: str = PROBE_BP_PATH) -> bool:
    """Check ProbePointHit; leaves probe alive on success for session reuse."""
    ok, name = ensure_collision_probe(ucv, bp_path=bp_path)
    if not ok:
        return False
    return _probe_responds(ucv, name)


def _z_in_calib_band(z_cm: float) -> bool:
    return LEVEL_CALIB_Z_MIN_CM <= z_cm <= LEVEL_CALIB_Z_MAX_CM


def reference_surface_z_collision(
    ucv,
    x_cm: float,
    y_cm: float,
    *,
    z_top_cm: float,
    z_bottom_cm: float,
    step_cm: float = 15.0,
    actor: str = PROBE_ACTOR,
) -> Optional[float]:
    """Highest Z [cm] with solid overlap when stepping down (floor top estimate)."""
    z = min(z_top_cm, LEVEL_CALIB_Z_MAX_CM)
    z_bottom = max(z_bottom_cm, LEVEL_CALIB_Z_MIN_CM)
    step = max(1.0, float(step_cm))
    while z >= z_bottom:
        if _z_in_calib_band(z) and probe_point_hit(ucv, x_cm, y_cm, z, actor=actor):
            return z
        z -= step
    return None


def reference_surface_z_for_cells_collision(
    ucv,
    cells,
    *,
    cell_center_xy_cm_fn,
    z_top_cm: float,
    z_bottom_cm: float,
    step_cm: float = 15.0,
    actor: str = PROBE_ACTOR,
) -> Optional[float]:
    """Highest floor surface among cells (collision vertical sweep)."""
    best: Optional[float] = None
    for gx, gy in cells:
        x_cm, y_cm = cell_center_xy_cm_fn(gx, gy)
        surface_z = reference_surface_z_collision(
            ucv,
            x_cm,
            y_cm,
            z_top_cm=z_top_cm,
            z_bottom_cm=z_bottom_cm,
            step_cm=step_cm,
            actor=actor,
        )
        if surface_z is None:
            continue
        if best is None or surface_z > best:
            best = surface_z
    return best
