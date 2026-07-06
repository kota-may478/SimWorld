#!/usr/bin/env python3
"""Phase 5 prerequisite checklist probe (requires PIE on /Game/Maps/Level)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (
    THIS_DIR,
    THIS_DIR.parent / "grid_env_hri",
    THIS_DIR.parent / "grid_env_10k",
):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402

ROBOT = geh.ROBOT_ACTOR_NAME
PROBE_VBPS = (
    "Move_Speed",
    "Rotate_Angle",
    "GetPose2dJson",
    "GetVisibleSightTargetsJson",
    "AttachCarryActor",
)


def _vbp_probe(ucv, actor: str, cmd: str) -> str:
    try:
        raw = geh._ue_request(ucv, f"vbp {actor} {cmd}", timeout_s=10.0)  # noqa: SLF001
    except Exception as exc:
        return f"ERR:{exc}"
    text = str(raw).strip() if raw is not None else ""
    return text[:200] if text else "EMPTY"


def _classify_vbp(raw: str) -> str:
    low = raw.lower()
    if not raw or raw == "EMPTY":
        return "NO_RESPONSE"
    if low.startswith("error") or "not found" in low or "unknown" in low:
        return "MISSING"
    if "argument invalid" in low:
        return "EXISTS_BAD_ARGS"
    return "EXISTS"


def main() -> int:
    results: list[tuple[str, str, str]] = []
    ucv, _ = g10k.ensure_connection()
    try:
        ok_nav, nav_actor = nq.ensure_nav_query_service(
            ucv,
            probe_xyz=(
                lc.local_xy_to_world(1500.0, 1500.0)[0],
                lc.local_xy_to_world(1500.0, 1500.0)[1],
                lc.NAV_PROJECT_PROBE_Z_CM,
            ),
        )
        results.append(("1", "NavQueryService", "PASS" if ok_nav else "FAIL"))

        names = geh.actor_names(ucv)
        robot_present = ROBOT in names
        results.append(("4a", f"Robot {ROBOT} spawned", "PASS" if robot_present else "FAIL"))

        controllers = sorted(n for n in names if "SpotDogAIController" in n)
        generic_ai = sorted(n for n in names if n.endswith("AIController_0"))
        if controllers:
            ctrl_status = f"PASS ({controllers[0]})"
        elif generic_ai:
            ctrl_status = f"WARN (generic {generic_ai[0]} only)"
        else:
            ctrl_status = "FAIL (no AI controller)"
        results.append(("3", "BP_SpotDogAIController in level", ctrl_status))

        for vbp in PROBE_VBPS:
            probe_arg = "1.0 30 -1" if vbp == "Rotate_Angle" else "180 0.01 0" if vbp == "Move_Speed" else ""
            if vbp == "AttachCarryActor":
                probe_arg = "__probe__"
            raw = _vbp_probe(ucv, ROBOT, f"{vbp} {probe_arg}".strip())
            results.append(("4b", f"vbp {vbp}", _classify_vbp(raw)))

        if robot_present:
            loc = ucv.get_location(ROBOT)
            start = (
                float(loc[0]),
                float(loc[1]),
                float(loc[2]),
            )
            proj = nq.nav_project_point(ucv, nav_actor, start[0], start[1], lc.NAV_PROJECT_PROBE_Z_CM)
            if proj.get("ok"):
                start_xyz = (float(proj["x"]), float(proj["y"]), float(proj.get("z", lc.NAV_PROJECT_PROBE_Z_CM)))
            else:
                start_xyz = start
            goal_xyz = lc.foot_world_xyz_from_local_xy(3000.0, 3000.0)
            goal_proj = nq.nav_project_point(ucv, nav_actor, goal_xyz[0], goal_xyz[1], lc.NAV_PROJECT_PROBE_Z_CM)
            if goal_proj.get("ok"):
                goal_xyz = (
                    float(goal_proj["x"]),
                    float(goal_proj["y"]),
                    float(goal_proj.get("z", lc.NAV_PROJECT_PROBE_Z_CM)),
                )
            path = nq.nav_find_path(
                ucv,
                nav_actor,
                start_xyz,
                goal_xyz,
                agent_radius_cm=100.0,
            )
            if path.get("ok"):
                n_pts = len(path.get("points", []))
                results.append(("2", "NavFindPathWithRadius (robot→goal)", f"PASS ({n_pts} pts)"))
            else:
                results.append(
                    ("2", "NavFindPathWithRadius (robot→goal)", f"FAIL ({path.get('error', path)})"),
                )
        else:
            interior = lc.foot_world_xyz_from_local_xy(1500.0, 1500.0)
            path = nq.nav_find_path(ucv, nav_actor, interior, lc.foot_world_xyz_from_local_xy(3000.0, 3000.0))
            if path.get("ok"):
                results.append(("2", "NavFindPath (interior)", f"PASS ({len(path.get('points', []))} pts)"))
            else:
                results.append(("2", "NavFindPath (interior)", f"FAIL ({path.get('error', path)})"))

        ext_api = nq.nav_runtime_api_available(ucv, nav_actor)
        results.append(("1b", "Extended NavQueryService API", "PASS" if ext_api else "FAIL"))

        rebuild = nq.nav_rebuild(ucv, nav_actor)
        results.append(("1c", "NavRebuild", "PASS" if rebuild.get("ok") else f"FAIL ({rebuild})"))

        metrics_path = (
            THIS_DIR
            / "scenarios"
            / "site_transport_20m"
            / "out"
            / "latest_metrics_json.json"
        )
        if metrics_path.is_file():
            payload = json.loads(metrics_path.read_text(encoding="utf-8"))
            navmesh = payload.get("profile") == "navmesh" or payload.get("nav_profile") == "navmesh"
            layout = payload.get("layout_id", "?")
            success = bool(payload.get("success"))
            if navmesh and layout == "layout_01" and success:
                results.append(("5", "layout_01 navmesh PASS (artifact)", "PASS"))
            else:
                results.append(
                    (
                        "5",
                        "layout_01 navmesh PASS (artifact)",
                        f"STALE (layout={layout} success={success} profile={payload.get('profile')})",
                    ),
                )
        else:
            results.append(("5", "layout_01 navmesh PASS (artifact)", "NO_ARTIFACT"))

    finally:
        geh.release_connection(ucv)

    print("=== Phase 5 Prerequisite Check ===")
    fails = 0
    warns = 0
    for num, label, status in results:
        mark = "OK" if status.startswith("PASS") else ("WARN" if status.startswith("WARN") or status.startswith("STALE") or status.startswith("EXISTS") else "NG")
        if mark == "NG":
            fails += 1
        elif mark == "WARN":
            warns += 1
        print(f"[{mark}] #{num} {label}: {status}")

    print(f"--- summary: {fails} fail, {warns} warn, {len(results) - fails - warns} pass ---")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
