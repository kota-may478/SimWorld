#!/usr/bin/env python3
"""UnrealCV vbp wrappers for BP_SpotRobot NavMove Phase 5 (SpotDogNavController)."""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

_THIS_DIR = Path(__file__).resolve().parent
_GEH_DIR = _THIS_DIR.parent / "grid_env_hri"
for _p in (_THIS_DIR, _GEH_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import grid_env_hri_simulation as geh  # noqa: E402

WorldXYZ = Tuple[float, float, float]


def moveto_use_ue_controller() -> bool:
    """True when Python should dispatch NavFollowPathJson (requires working UE tick)."""
    return os.environ.get("NAV_MOVETO_UE", "").strip().lower() in ("1", "true", "yes")


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


def parse_nav_move_json(raw_response: object) -> dict:
    if isinstance(raw_response, dict):
        return _unwrap_return_value(raw_response)
    if isinstance(raw_response, str):
        text = raw_response.strip()
        if not text or text.lower().startswith("error"):
            return {"error": text}
        try:
            return _unwrap_return_value(json.loads(text))
        except json.JSONDecodeError:
            return parse_nav_move_json({"ReturnValue": text})
    return {"error": "empty_response"}


def _vbp_nav_move(ucv, robot_name: str, method: str, *args: object) -> dict:
    parts = " ".join(str(a) for a in args)
    cmd = f"vbp {robot_name} {method} {parts}".strip()
    raw = geh._ue_request(ucv, cmd, timeout_s=30.0)  # noqa: SLF001
    if raw is None:
        return {"error": "ue_request_failed"}
    text = str(raw).strip()
    if text.lower().startswith("error"):
        return {"error": text}
    try:
        return parse_nav_move_json(json.loads(text))
    except json.JSONDecodeError:
        return parse_nav_move_json(text)


def path_points_to_json(points: Sequence[WorldXYZ]) -> str:
    payload = {
        "points": [
            {"x": float(x), "y": float(y), "z": float(z)}
            for x, y, z in points
        ]
    }
    return json.dumps(payload, separators=(",", ":"))


def nav_move_api_available(ucv, robot_name: str) -> bool:
    raw = _vbp_nav_move(ucv, robot_name, "GetNavMoveStatusJson")
    if raw.get("status"):
        return True
    err = str(raw.get("error", "")).lower()
    return "not found" not in err and "unknown" not in err and "invalid" not in err


def nav_follow_path_json(
    ucv,
    robot_name: str,
    points: Sequence[WorldXYZ],
) -> dict:
    if not points:
        return {"ok": False, "error": "empty_path"}
    return _vbp_nav_move(
        ucv,
        robot_name,
        "NavFollowPathJson",
        path_points_to_json(points),
    )


def nav_move_to_goal(
    ucv,
    robot_name: str,
    goal_xyz: WorldXYZ,
    *,
    acceptance_radius_cm: float = 130.0,
) -> dict:
    gx, gy, gz = goal_xyz
    return _vbp_nav_move(
        ucv,
        robot_name,
        "NavMoveToGoal",
        gx,
        gy,
        gz,
        acceptance_radius_cm,
    )


def nav_stop_move(ucv, robot_name: str) -> dict:
    return _vbp_nav_move(ucv, robot_name, "NavStopMove")


def get_nav_move_status(ucv, robot_name: str) -> dict:
    return _vbp_nav_move(ucv, robot_name, "GetNavMoveStatusJson")


def wait_nav_move_complete(
    ucv,
    robot_name: str,
    *,
    timeout_s: float = 180.0,
    poll_s: float = 0.25,
) -> dict:
    deadline = time.perf_counter() + timeout_s
    last: dict = {"status": "idle"}
    while time.perf_counter() < deadline:
        last = get_nav_move_status(ucv, robot_name)
        status = str(last.get("status", "")).lower()
        if status in ("success", "failed", "idle"):
            return last
        time.sleep(poll_s)
    last["error"] = "timeout"
    return last
