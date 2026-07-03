#!/usr/bin/env python3
"""Phase 5 diagnostic: NavFollowPathJson motion + UE log hints."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_move as nm  # noqa: E402
import nav_query as nq  # noqa: E402
from grid_env_10k_pie_patrol import get_yaw  # noqa: E402

ROBOT = geh.ROBOT_ACTOR_NAME
UE_LOG = Path("/mnt/c/UEProjects/SimWorld/Saved/Logs/SimWorld.log")


def _tail_spotdog_log() -> list[str]:
    if not UE_LOG.is_file():
        return [f"(log not found: {UE_LOG})"]
    lines = UE_LOG.read_text(encoding="utf-8", errors="replace").splitlines()
    patterns = (
        "SpotDogNavController",
        "rotate_vbp",
        "move_vbp",
        "Rotate SetTimer passed",
        "Divide by zero",
    )
    hits = [ln for ln in lines[-4000:] if any(p in ln for p in patterns)]
    return hits[-25:] if hits else ["(no SpotDogNavController / Rotate log lines in recent log)"]


def _test_rotate_vbp(ucv, robot: str) -> tuple[bool, float]:
    """Return (yaw_changed, delta_deg) for a short Rotate_Angle probe."""
    yaw0 = get_yaw(ucv, robot)
    geh._ue_request(ucv, f"vbp {robot} Rotate_Angle 1.0 30 -1", timeout_s=10.0)
    time.sleep(1.2)
    yaw1 = get_yaw(ucv, robot)
    return abs(yaw1 - yaw0) >= 3.0, yaw1 - yaw0


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    names = geh.actor_names(ucv)
    ctrls = [n for n in names if "SpotDogAIController" in n]
    print("=== Phase 5 NavMove Diagnostic ===")
    print(f"robot present: {ROBOT in names}")
    print(f"controllers: {ctrls}")
    if ROBOT not in names:
        print("FAIL: robot not in level")
        return 1

    loc0 = ucv.get_location(ROBOT)
    yaw0 = get_yaw(ucv, ROBOT)
    print(f"robot loc=({loc0[0]:.0f},{loc0[1]:.0f}) yaw={yaw0:.1f}")

    rotate_ok, rotate_delta = _test_rotate_vbp(ucv, ROBOT)
    print(
        f"Rotate_Angle vbp probe: {'OK' if rotate_ok else 'BROKEN'} "
        f"(delta={rotate_delta:.1f}°)"
    )
    if not rotate_ok:
        print(
            "  → Rotate_Angle BP fails (Output Log: Divide by zero / SetTimer zero). "
            "Rebuild with latest SpotDogNavController (bUseDirectYawRotation)."
        )

    nm.nav_stop_move(ucv, ROBOT)
    time.sleep(0.2)
    _, nav_actor = nq.ensure_nav_query_service(
        ucv, probe_xyz=(0.0, 0.0, lc.NAV_PROJECT_PROBE_Z_CM)
    )
    x, y = float(loc0[0]), float(loc0[1])

    def proj(ax: float, ay: float) -> tuple[float, float, float]:
        raw = nq.nav_project_point(ucv, nav_actor, ax, ay, lc.NAV_PROJECT_PROBE_Z_CM)
        return (
            float(raw["x"]),
            float(raw["y"]),
            float(raw.get("z", lc.NAV_PROJECT_PROBE_Z_CM)),
        )

    start = proj(x, y)
    goal = proj(x + 300.0, y)
    path = nq.nav_find_path(ucv, nav_actor, start, goal)
    pts = [(float(p["x"]), float(p["y"]), float(p["z"])) for p in path["points"]]
    follow = nm.nav_follow_path_json(ucv, ROBOT, pts)
    print(f"NavFollowPathJson: {follow}")
    if not follow.get("ok"):
        return 1

    moved_max = 0.0
    last_status = ""
    for i in range(40):
        st = nm.get_nav_move_status(ucv, ROBOT)
        loc = ucv.get_location(ROBOT)
        yaw = get_yaw(ucv, ROBOT)
        moved = math.hypot(loc[0] - loc0[0], loc[1] - loc0[1])
        moved_max = max(moved_max, moved)
        status = str(st.get("status", ""))
        if status != last_status or i % 4 == 0:
            print(
                f"  t={i * 0.5:.1f}s status={status} "
                f"moved={moved:.0f}cm yaw={yaw:.0f} "
                f"dist={st.get('dist_remaining_cm')}"
            )
            last_status = status
        if status.lower() in ("success", "failed"):
            break
        time.sleep(0.5)

    print(f"max moved: {moved_max:.0f}cm")
    print("--- UE log (SpotDogNavController) ---")
    for ln in _tail_spotdog_log():
        print(ln)

    if last_status.lower() == "success" and moved_max >= 50.0:
        print("PASS")
        return 0
    print("FAIL")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
