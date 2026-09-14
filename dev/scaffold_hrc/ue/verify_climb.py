#!/usr/bin/env python3
"""PIE check: leftover header beams, Recast on 1F→3F, Spot step-through.

Does not destroy ScaffoldHrc_truck. Soft-resets SpotDog if already in the level.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
ROOT = PKG.parent.parent
NAV = ROOT / "dev" / "grid_env_level_nav"
GEH = ROOT / "dev" / "grid_env_hri"
DEPTH = ROOT / "dev" / "grid_env_depth_perception"
for path in (str(ROOT), str(NAV), str(GEH), str(DEPTH), str(PKG)):
    if path not in sys.path:
        sys.path.insert(0, path)

import ue_client_guard  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402
from pie_safety import tick_settle  # noqa: E402
from level_nav_robot import ensure_level_spotdog  # noqa: E402
from ue.layout import STAGING_LOCAL_XY_CM, scaffold_xyz_to_local_cm  # noqa: E402
from ue.placement import (  # noqa: E402
    ACTOR_PREFIX,
    STALE_ACTORS,
    TRUCK_ACTOR,
    spot_ramps,
    stair_landings,
)

YARD_HUMAN = f"{ACTOR_PREFIX}_yard_human"
FORBIDDEN_NAMES = (
    f"{ACTOR_PREFIX}_rail_f2_0",
    f"{ACTOR_PREFIX}_rail_f2_1",
    f"{ACTOR_PREFIX}_rail_f3_0",
    f"{ACTOR_PREFIX}_rail_f3_1",
    f"{ACTOR_PREFIX}_transom_f2_0",
    f"{ACTOR_PREFIX}_transom_f3_0",
    f"{ACTOR_PREFIX}_landing_f3_north",
    *(f"{ACTOR_PREFIX}_tread_L0_{i}" for i in range(10)),
    *(f"{ACTOR_PREFIX}_tread_L1_{i}" for i in range(10)),
)
SPOT_FOOT_Z_CM = 50.0
SPOT_HOP_TOL_Z_CM = 80.0
HOP_END_Z_TOL_CM = 40.0


def _project(ucv, nav_actor: str, local_cm) -> dict:
    wx, wy, wz = lc.local_xyz_to_world(*local_cm)
    return nq.nav_project_point(ucv, nav_actor, wx, wy, wz + 20.0)


def _ok(raw: dict) -> bool:
    return bool(raw.get("ok"))


def _project_xyz(ucv, nav_actor: str, x_m: float, y_m: float, z_m: float) -> dict:
    local = scaffold_xyz_to_local_cm(x_m, y_m, z_m)
    wx, wy, wz = lc.local_xyz_to_world(*local)
    return nq.nav_project_point(ucv, nav_actor, wx, wy, wz + 20.0)


def _first_nav_point(ucv, nav_actor: str, samples: list) -> tuple:
    for label, xyz in samples:
        raw = _project_xyz(ucv, nav_actor, *xyz)
        print("probe", label, json.dumps(raw, default=str)[:180])
        if _ok(raw):
            return label, (float(raw["x"]), float(raw["y"]), float(raw["z"]))
    return None, None


def _diagnose_fail(ucv, nav_actor: str, labels: list[str], probes: dict) -> None:
    names = set(geh.actor_names(ucv))
    leftover = [n for n in FORBIDDEN_NAMES if n in names]
    stale = [n for n in STALE_ACTORS if n in names]
    print("diagnose leftover_header", leftover)
    print("diagnose stale", stale)
    for name in (
        "NavMeshBoundsVolume_1",
        "NavMeshBoundsVolume",
        "RecastNavMesh_0",
    ):
        if name not in names:
            continue
        loc = ucv.get_location(name)
        bounds = nq.get_actor_bounds(ucv, nav_actor, name)
        print("diagnose", name, "loc", loc, "bounds", json.dumps(bounds, default=str)[:300])
    for key in labels:
        raw = probes.get(key, {})
        if _ok(raw):
            continue
        print("diagnose no_projection", key, json.dumps(raw, default=str)[:240])


WALK_SPEED = 80.0
WALK_MAX_SLICE_S = 0.45


def _yaw_toward(src, dst) -> float:
    return math.degrees(math.atan2(dst[1] - src[1], dst[0] - src[0]))


def _follow_nav_path(ucv, robot_name: str, points) -> dict:
    ucv.set_physics(robot_name, False)
    if not points:
        return {"ok": False, "error": "empty_path"}
    for index, pt in enumerate(points):
        stand = (pt[0], pt[1], pt[2] + SPOT_FOOT_Z_CM)
        if index + 1 < len(points):
            nxt = points[index + 1]
            ucv.set_orientation(
                [0.0, _yaw_toward(stand, (nxt[0], nxt[1], nxt[2])), 0.0],
                robot_name,
            )
        ucv.set_location([stand[0], stand[1], stand[2]], robot_name)
        tick_settle(ucv, settle_s=0.08, ticks=1)
        if index + 1 < len(points):
            nxt = points[index + 1]
            dist = math.hypot(nxt[0] - stand[0], nxt[1] - stand[1])
            duration = min(WALK_MAX_SLICE_S, max(0.12, dist / 200.0))
            ucv.dog_move(robot_name, [WALK_SPEED, duration, 0])
    loc = ucv.get_location(robot_name)
    goal = (points[-1][0], points[-1][1], points[-1][2] + SPOT_FOOT_Z_CM)
    dist_xy = math.hypot(loc[0] - goal[0], loc[1] - goal[1])
    dz = loc[2] - goal[2]
    rise = points[-1][2] - points[0][2]
    ok = dist_xy <= 80.0 and abs(dz) <= SPOT_HOP_TOL_Z_CM and rise >= 300.0
    return {
        "ok": ok,
        "loc": [float(loc[0]), float(loc[1]), float(loc[2])],
        "goal": list(goal),
        "dist_xy": dist_xy,
        "dz": float(dz),
        "rise": float(rise),
        "n_points": len(points),
    }


def main() -> int:
    ucv, _ = ue_client_guard.prepare_ue_connection()
    if not geh._ping_ucv(ucv):  # noqa: SLF001
        print("FAIL: UnrealCV ping. Start PIE on /Game/Maps/Level first.")
        return 1
    names = set(geh.actor_names(ucv))
    required = [
        TRUCK_ACTOR,
        YARD_HUMAN,
        f"{ACTOR_PREFIX}_deck_f1_b0_r0",
        f"{ACTOR_PREFIX}_deck_f2_b0_r0",
        f"{ACTOR_PREFIX}_deck_f3_b0_r0",
        f"{ACTOR_PREFIX}_landing_f3_south_0",
        f"{ACTOR_PREFIX}_landing_f3_yard_r0",
        f"{ACTOR_PREFIX}_ramp_L0_0",
        f"{ACTOR_PREFIX}_ramp_L1_5",
        f"{ACTOR_PREFIX}_cheek_L0_s_0",
        f"{ACTOR_PREFIX}_cheek_L0_n_0",
        f"{ACTOR_PREFIX}_cheek_L1_s_0",
        f"{ACTOR_PREFIX}_cheek_L1_n_0",
    ]
    missing = [n for n in required if n not in names]
    leftover = [n for n in FORBIDDEN_NAMES if n in names]
    print("missing", missing)
    print("leftover_header", leftover)
    if leftover:
        print("FAIL: leftover ghost stairs or header beams in PIE. Re-run spawn_scaffold_pie.py --keep")
        return 3

    ok, nav_actor = nq.ensure_nav_query_service(ucv)
    print("nav_actor", ok, nav_actor)
    if not ok:
        print("FAIL: NavQueryService. Recast/BP_NavQueryService is not ready.")
        return 1
    if "NavMeshBoundsVolume_1" in names:
        loc = ucv.get_location("NavMeshBoundsVolume_1")
        ucv.set_location([float(loc[0]), float(loc[1]), 6680.0], "NavMeshBoundsVolume_1")
        print("raised NavMeshBoundsVolume_1 z", loc[2], "->", 6680.0)
    rebuild = nq.nav_rebuild(ucv, nav_actor)
    print("rebuild", json.dumps(rebuild, default=str)[:240])

    probes = {}
    for box in (*stair_landings(), *spot_ramps()):
        if box.kind not in {"landing", "tread", "ramp"}:
            continue
        probes[box.actor] = _project(ucv, nav_actor, box.local_cm)
    for floor, xyz in (
        (1, (1.0, 0.6, 0.15)),
        (2, (1.0, 0.6, 1.95)),
        (3, (1.0, 0.6, 3.75)),
    ):
        probes[f"deck_f{floor}"] = _project(
            ucv, nav_actor, scaffold_xyz_to_local_cm(*xyz)
        )

    failed = [k for k, raw in probes.items() if not _ok(raw)]
    print("nav_fail", failed)
    print("nav_ok", len(probes) - len(failed), "/", len(probes))
    for key in (
        "ScaffoldHrc_ramp_L0_5",
        "ScaffoldHrc_landing_f2_sill_r0",
        "ScaffoldHrc_ramp_L1_5",
        "ScaffoldHrc_landing_f3_south_0",
        "deck_f2",
        "deck_f3",
    ):
        print("nav", key, json.dumps(probes.get(key, {}), default=str))

    start_label, start_w = _first_nav_point(
        ucv,
        nav_actor,
        [
            ("deck_f1_mid", (5.0, 0.6, 0.20)),
            ("deck_f1_mouth", (0.5, 0.60, 0.20)),
            ("f1_north", (-0.90, 2.00, 0.20)),
            ("ramp_l0_low", (-6.70, 0.55, 0.45)),
        ],
    )
    goal_label, goal_w = _first_nav_point(
        ucv,
        nav_actor,
        [
            ("deck_f3", (3.0, 0.6, 3.75)),
            ("landing_f3_south", (-0.50, 0.60, 3.75)),
            ("landing_f3_yard", (-7.60, 0.60, 3.75)),
        ],
    )
    print("path_ends", start_label, start_w, "->", goal_label, goal_w)
    chain = [
        ("deck_f1_mouth", (0.40, 0.55, 0.20)),
        ("l0_low", (-6.70, 0.55, 0.45)),
        ("l0_high", (-1.70, 0.55, 1.95)),
        ("deck_f2", (3.0, 0.6, 1.95)),
        ("l1_low", (-1.70, 1.85, 2.25)),
        ("l1_high", (-6.70, 1.85, 3.75)),
        ("f3_yard", (-7.60, 0.60, 3.75)),
        ("f3_south", (-0.50, 0.60, 3.75)),
        ("deck_f3", (3.0, 0.6, 3.75)),
    ]
    chain_pts = []
    for label, xyz in chain:
        raw = _project_xyz(ucv, nav_actor, *xyz)
        print("chain", label, json.dumps(raw, default=str)[:180])
        if _ok(raw):
            chain_pts.append((label, (float(raw["x"]), float(raw["y"]), float(raw["z"]))))
    stitched: list = []
    all_segs_ok = True
    for (a, pa), (b, pb) in zip(chain_pts, chain_pts[1:]):
        hop = nq.nav_find_path(ucv, nav_actor, pa, pb)
        pts = nq.path_points_xyz(hop)
        rise = (pts[-1][2] - pts[0][2]) if pts else 0.0
        print(
            "seg",
            a,
            "->",
            b,
            "ok",
            bool(hop.get("ok")),
            "n",
            len(pts),
            "rise",
            rise,
        )
        if not hop.get("ok") or not pts:
            all_segs_ok = False
            continue
        end_z = pts[-1][2]
        if abs(end_z - pb[2]) > HOP_END_Z_TOL_CM:
            all_segs_ok = False
            print("seg snapped_away", a, "->", b, "end_z", end_z, "want", pb[2])
            continue
        if not stitched:
            stitched.extend(pts)
        else:
            stitched.extend(pts[1:])
    path_raw = {"ok": False, "error": "no_projected_ends"}
    points = []
    if start_w and goal_w:
        path_raw = nq.nav_find_path(ucv, nav_actor, start_w, goal_w)
        points = nq.path_points_xyz(path_raw)
    direct_rise = (points[-1][2] - points[0][2]) if points else 0.0
    print("path_ok", bool(path_raw.get("ok")), "n", len(points), "rise", direct_rise)
    print("path_raw", json.dumps(path_raw, default=str)[:400])
    stitch_rise = (stitched[-1][2] - stitched[0][2]) if stitched else 0.0
    print("stitch", all_segs_ok, "n", len(stitched), "rise", stitch_rise)
    if direct_rise < 300.0 and all_segs_ok and stitch_rise >= 300.0:
        points = stitched
        path_raw = {"ok": True, "stitched": True}

    start_local = (STAGING_LOCAL_XY_CM[0] + 250.0, STAGING_LOCAL_XY_CM[1] + 250.0)
    robot_ok, robot_name = ensure_level_spotdog(ucv, start_local)
    print("spotdog", robot_ok, robot_name)
    walk_raw = {"ok": False, "error": "no_robot"}
    if robot_ok and points:
        walk_raw = _follow_nav_path(ucv, robot_name, points)
        print("nav_follow", json.dumps(walk_raw, default=str))

    if missing:
        print("FAIL: spawn missing actors first (spawn_scaffold_pie.py)")
        return 1
    if not robot_ok:
        print("FAIL: SpotDog is not in the level")
        return 2
    if not path_raw.get("ok") or not points:
        _diagnose_fail(ucv, nav_actor, failed, probes)
        print("FAIL: Recast has no 1F→3F path", path_raw)
        return 2
    if not walk_raw.get("ok"):
        print("FAIL: Spot did not follow NavMesh 1F→3F", walk_raw)
        return 2
    print("OK: Spot followed NavMesh 1F→3F")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
