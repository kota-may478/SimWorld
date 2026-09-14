#!/usr/bin/env python3
"""Spawn the parametric 3F scaffold on /Game/Maps/Level (PIE required).

Usage (PIE running, UnrealCV on :9000):
  conda run -n simworld python dev/scaffold_hrc/ue/spawn_scaffold_pie.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
ROOT = PKG.parent.parent
NAV = ROOT / "dev" / "grid_env_level_nav"
GEH = ROOT / "dev" / "grid_env_hri"
DEPTH = ROOT / "dev" / "grid_env_depth_perception"
for path in (str(ROOT), str(PKG), str(NAV), str(GEH), str(DEPTH)):
    if path not in sys.path:
        sys.path.insert(0, path)

import ue_client_guard  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import level_coords as lc  # noqa: E402
from pie_spawn_safety import (  # noqa: E402
    destroy_actor_level,
    destroy_by_prefix,
    spawn_bp_resilient,
)
from pie_safety import tick_settle  # noqa: E402
from level_nav_robot import ensure_level_spotdog  # noqa: E402
from ue.placement import (  # noqa: E402
    ACTOR_PREFIX,
    ASSEMBLER_ACTOR,
    PERSISTENT_ACTORS,
    PIPE_KINDS,
    SPOT_YARD_LOCAL_XY_CM,
    STALE_ACTORS,
    SpawnBox,
    TRUCK_ACTOR,
    TRUCK_BP_CANDIDATES,
    TRUCK_UNIFORM_SCALE,
    TRUCK_YAW_DEG,
    YARD_HUMAN_ACTOR,
    YARD_HUMAN_ALONG_CM,
    YARD_HUMAN_SIDE_CM,
    YARD_HUMAN_YAW_DEG,
    all_spawn_boxes,
)
from ue.layout import STAGING_LOCAL_XY_CM  # noqa: E402
from ue.mission_poses import (  # noqa: E402
    ASSEMBLER_FOOT_Z_CM,
    ASSEMBLER_SCALE,
    assembler_stand_local_cm,
    extra_humanoid_actor_names,
    floor_drop_slot_local_cm,
    standing_local_cm,
    SPOT_STAND_Z_CM,
)

NAV_VOLUME = "NavMeshBoundsVolume_1"
NAV_VOLUME_CENTER_Z_CM = 6680.0
CUBE_BP = geh.CUBE_BP
BATCH_PAUSE_EVERY = 10
PIPE_COLOR = (168, 172, 176)
YARD_HUMAN = YARD_HUMAN_ACTOR
# Extra idle after member destroys before any spawn_bp (reuse path usually skips spawn).
BARE_POST_DESTROY_EXTRA_S = 8.0
HUMAN_BP_CANDIDATES = (
    geh.HUMAN_BP,
    "/Game/TrafficSystem/Pedestrian/Base_Pedestrian.Base_Pedestrian_C",
)


def _place(ucv, box: SpawnBox) -> None:
    wx, wy, wz = lc.local_xyz_to_world(*box.local_cm)
    ucv.set_physics(box.actor, False)
    geh.set_cube_blocking_mode(
        ucv, box.actor, blocking=True, apply_tint=True
    )
    if box.kind in PIPE_KINDS:
        try:
            ucv.set_color(box.actor, list(PIPE_COLOR))
        except Exception:
            pass
    ucv.set_location([wx, wy, wz], box.actor)
    ucv.set_scale(list(box.scale), box.actor)
    ucv.set_orientation([box.pitch_deg, box.yaw_deg, box.roll_deg], box.actor)


def spawn_boxes(ucv, boxes: tuple[SpawnBox, ...]) -> tuple[int, object]:
    placed = 0
    for index, box in enumerate(boxes, start=1):
        if geh.actor_exists(ucv, box.actor):
            _place(ucv, box)
        else:
            ok, ucv = spawn_bp_resilient(ucv, CUBE_BP, box.actor)
            if not ok:
                print(f"[ScaffoldSpawn] FAIL {box.actor}")
                return placed, ucv
            _place(ucv, box)
        placed += 1
        if index % BATCH_PAUSE_EVERY == 0:
            tick_settle(ucv, settle_s=0.25, ticks=1)
            print(f"[ScaffoldSpawn] placed {placed}/{len(boxes)}")
    return placed, ucv


def _raise_nav_volume(ucv) -> None:
    """Keep 1F–3F inside the Recast volume (3F walk ~6812 cm)."""
    if not geh.actor_exists(ucv, NAV_VOLUME):
        return
    loc = ucv.get_location(NAV_VOLUME)
    ucv.set_location([float(loc[0]), float(loc[1]), NAV_VOLUME_CENTER_Z_CM], NAV_VOLUME)
    print(f"[ScaffoldSpawn] {NAV_VOLUME} z {loc[2]} -> {NAV_VOLUME_CENTER_Z_CM}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Spawn Stage-1 scaffold cubes in Level PIE.")
    parser.add_argument("--keep", action="store_true", help="do not destroy previous ScaffoldHrc_* actors")
    parser.add_argument("--truck-only", action="store_true", help="spawn/move the yard vehicle BP only")
    parser.add_argument(
        "--bare",
        action="store_true",
        help=(
            "bare ground: destroy scaffold members/carry only; "
            "reuse truck + yard human + assembler (never destroy→respawn those BPs)"
        ),
    )
    args = parser.parse_args()

    ucv, _ = ue_client_guard.prepare_ue_connection()
    if not geh._ping_ucv(ucv):  # noqa: SLF001
        print("FAIL: UnrealCV is not listening on :9000. Start PIE on /Game/Maps/Level first.")
        return 1
    if args.truck_only:
        _spawn_yard_truck(ucv)
        return 0
    if args.bare:
        removed, ucv = destroy_by_prefix(
            ucv, f"{ACTOR_PREFIX}_", skip=PERSISTENT_ACTORS
        )
        kept = ", ".join(PERSISTENT_ACTORS)
        print(f"[ScaffoldSpawn] bare: destroyed {removed} (kept {kept})")
        if removed:
            tick_settle(ucv, settle_s=BARE_POST_DESTROY_EXTRA_S, ticks=2)
        _spawn_yard_truck(ucv)
        _spawn_yard_human(ucv)
        _spawn_yard_assembler(ucv)
        _park_extra_humanoids(ucv)
        _raise_nav_volume(ucv)
        _spawn_yard_spot(ucv)
        ok = _report_yard_actors(ucv)
        print("[ScaffoldSpawn] bare yard ready (no scaffold members)")
        return 0 if ok else 1
    if not args.keep:
        removed, ucv = destroy_by_prefix(
            ucv, f"{ACTOR_PREFIX}_", skip=PERSISTENT_ACTORS
        )
        kept = ", ".join(PERSISTENT_ACTORS)
        print(f"[ScaffoldSpawn] destroyed {removed} previous actors (kept {kept})")
    boxes = all_spawn_boxes()
    placed, ucv = spawn_boxes(ucv, boxes)
    print(f"[ScaffoldSpawn] placed {placed}/{len(boxes)}")
    for leftover in STALE_ACTORS:
        if geh.actor_exists(ucv, leftover):
            ok, ucv = destroy_actor_level(ucv, leftover)
            print(f"[ScaffoldSpawn] removed leftover {leftover} ok={ok}")
    _spawn_yard_truck(ucv)
    _spawn_yard_human(ucv)
    _raise_nav_volume(ucv)
    return 0 if placed == len(boxes) else 1


def _place_yard_truck(ucv, wx: float, wy: float, wz: float) -> None:
    scale = TRUCK_UNIFORM_SCALE
    ucv.set_physics(TRUCK_ACTOR, False)
    ucv.set_location([wx, wy, wz], TRUCK_ACTOR)
    ucv.set_orientation([0.0, TRUCK_YAW_DEG, 0.0], TRUCK_ACTOR)
    ucv.set_scale([scale, scale, scale], TRUCK_ACTOR)


def _spawn_yard_truck(ucv) -> None:
    # Do not UnrealCV-destroy BP_KeiTruck / humanoids. Destroy-then-spawn crashed
    # UE 5.3.2 (connection reset on spawn_bp after vset .../destroy). See PERSISTENT_ACTORS.
    sx, sy = STAGING_LOCAL_XY_CM
    wx, wy, wz = lc.local_xyz_to_world(sx, sy, 0.0)
    if geh.actor_exists(ucv, TRUCK_ACTOR):
        _place_yard_truck(ucv, wx, wy, wz)
        print(f"[ScaffoldSpawn] reused {TRUCK_ACTOR}")
        return
    for bp_path in TRUCK_BP_CANDIDATES:
        ok, ucv = spawn_bp_resilient(ucv, bp_path, TRUCK_ACTOR, timeout_s=120.0)
        if not ok:
            print(f"[ScaffoldSpawn] truck candidate failed {bp_path}")
            continue
        _place_yard_truck(ucv, wx, wy, wz)
        print(f"[ScaffoldSpawn] yard truck {bp_path} at {(wx, wy, wz)}")
        return
    print("[ScaffoldSpawn] no in-project vehicle BP spawned")


def _spawn_yard_human(ucv) -> None:
    sx, sy = STAGING_LOCAL_XY_CM
    wx, wy, wz = lc.local_xyz_to_world(
        sx + YARD_HUMAN_SIDE_CM, sy + YARD_HUMAN_ALONG_CM, ASSEMBLER_FOOT_Z_CM
    )
    scale = [ASSEMBLER_SCALE, ASSEMBLER_SCALE, ASSEMBLER_SCALE]
    if geh.actor_exists(ucv, YARD_HUMAN):
        _ground_human(ucv, YARD_HUMAN)
        try:
            ucv.set_scale(scale, YARD_HUMAN)
        except Exception:
            pass
        ucv.set_location([wx, wy, wz], YARD_HUMAN)
        ucv.set_orientation([0.0, YARD_HUMAN_YAW_DEG, 0.0], YARD_HUMAN)
        print(f"[ScaffoldSpawn] reused {YARD_HUMAN}")
        return
    for bp_path in HUMAN_BP_CANDIDATES:
        ok, _ucv = spawn_bp_resilient(ucv, bp_path, YARD_HUMAN, timeout_s=120.0)
        if not ok:
            print(f"[ScaffoldSpawn] humanoid candidate failed {bp_path}")
            continue
        _ground_human(ucv, YARD_HUMAN)
        try:
            ucv.set_scale(scale, YARD_HUMAN)
        except Exception:
            pass
        ucv.set_location([wx, wy, wz], YARD_HUMAN)
        ucv.set_orientation([0.0, YARD_HUMAN_YAW_DEG, 0.0], YARD_HUMAN)
        print(f"[ScaffoldSpawn] yard humanoid {bp_path} at {(wx, wy, wz)}")
        return
    print("[ScaffoldSpawn] no humanoid BP spawned")


def _ground_human(ucv, name: str) -> None:
    for fn in (
        lambda: ucv.set_physics(name, False),
        lambda: ucv.set_movable(name, True),
        lambda: ucv.set_collision(name, True),
        lambda: ucv.enable_controller(name, False),
    ):
        try:
            fn()
        except Exception:
            pass


def _spawn_yard_assembler(ucv) -> None:
    local = assembler_stand_local_cm(floor_drop_slot_local_cm(1, 0))
    wx, wy, wz = lc.local_xyz_to_world(*local)
    scale = [ASSEMBLER_SCALE, ASSEMBLER_SCALE, ASSEMBLER_SCALE]
    if geh.actor_exists(ucv, ASSEMBLER_ACTOR):
        _ground_human(ucv, ASSEMBLER_ACTOR)
        try:
            ucv.set_scale(scale, ASSEMBLER_ACTOR)
        except Exception:
            pass
        ucv.set_location([wx, wy, wz], ASSEMBLER_ACTOR)
        ucv.set_orientation([0.0, 180.0, 0.0], ASSEMBLER_ACTOR)
        print(f"[ScaffoldSpawn] reused {ASSEMBLER_ACTOR} at {(wx, wy, wz)}")
        return
    for bp_path in HUMAN_BP_CANDIDATES:
        ok, _ucv = spawn_bp_resilient(ucv, bp_path, ASSEMBLER_ACTOR, timeout_s=120.0)
        if not ok:
            print(f"[ScaffoldSpawn] assembler candidate failed {bp_path}")
            continue
        _ground_human(ucv, ASSEMBLER_ACTOR)
        try:
            ucv.set_scale(scale, ASSEMBLER_ACTOR)
        except Exception:
            pass
        ucv.set_location([wx, wy, wz], ASSEMBLER_ACTOR)
        ucv.set_orientation([0.0, 180.0, 0.0], ASSEMBLER_ACTOR)
        print(f"[ScaffoldSpawn] assembler {bp_path} at {(wx, wy, wz)}")
        return
    print("[ScaffoldSpawn] no assembler BP spawned")


def _park_extra_humanoids(ucv) -> None:
    extras = extra_humanoid_actor_names(
        geh.actor_names(ucv), (YARD_HUMAN, ASSEMBLER_ACTOR)
    )
    for name in extras:
        loc = geh.try_get_location_cm(ucv, name)
        if loc is None:
            try:
                loc = ucv.get_location(name)
            except Exception:
                continue
        try:
            ucv.set_physics(name, False)
            ucv.set_collision(name, False)
        except Exception:
            pass
        wx, wy, _wz = lc.local_xyz_to_world(-50_000.0, -50_000.0, -50_000.0)
        ucv.set_location([wx, wy, lc.FLOOR_REF_Z_CM - 50_000.0], name)
        print(
            f"[ScaffoldSpawn] parked leftover humanoid {name} "
            f"was=({float(loc[0]):.1f},{float(loc[1]):.1f},{float(loc[2]):.1f})"
        )


def _spawn_yard_spot(ucv) -> None:
    ok, name = ensure_level_spotdog(ucv, SPOT_YARD_LOCAL_XY_CM)
    if not ok:
        print("[ScaffoldSpawn] FAIL SpotDog not placed in the yard")
        return
    stand = standing_local_cm(
        (SPOT_YARD_LOCAL_XY_CM[0], SPOT_YARD_LOCAL_XY_CM[1], 0.0),
        SPOT_STAND_Z_CM,
    )
    wx, wy, wz = lc.local_xyz_to_world(*stand)
    try:
        ucv.set_physics(name, False)
        ucv.set_collision(name, False)
    except Exception:
        pass
    ucv.set_location([wx, wy, wz], name)
    loc = ucv.get_location(name)
    print(f"[ScaffoldSpawn] yard Spot {name} at {tuple(round(v, 1) for v in loc)}")


def _report_yard_actors(ucv) -> bool:
    names = set(geh.actor_names(ucv))
    spot = geh.ROBOT_ACTOR_NAME if geh.ROBOT_ACTOR_NAME in names else None
    if spot is None:
        spot = next((n for n in names if "SpotRobot" in n), None)
    checks = [
        ("truck", TRUCK_ACTOR in names, TRUCK_ACTOR),
        ("yard_human", YARD_HUMAN in names, YARD_HUMAN),
        ("spot", spot is not None, spot or geh.ROBOT_ACTOR_NAME),
    ]
    ok = True
    for label, present, actor in checks:
        if not present:
            print(f"[Yard] {label} MISSING ({actor})")
            ok = False
            continue
        loc = ucv.get_location(actor)
        lx, ly, lz = lc.world_xyz_to_local(*loc)
        print(
            f"[Yard] {label} OK {actor} "
            f"world=({loc[0]:.1f},{loc[1]:.1f},{loc[2]:.1f}) "
            f"local=({lx:.1f},{ly:.1f},{lz:.1f})"
        )
    assembler = ASSEMBLER_ACTOR in names
    if assembler:
        loc = ucv.get_location(ASSEMBLER_ACTOR)
        lx, ly, lz = lc.world_xyz_to_local(*loc)
        print(
            f"[Yard] assembler OK {ASSEMBLER_ACTOR} "
            f"world=({loc[0]:.1f},{loc[1]:.1f},{loc[2]:.1f}) "
            f"local=({lx:.1f},{ly:.1f},{lz:.1f})"
        )
    else:
        print(f"[Yard] assembler MISSING {ASSEMBLER_ACTOR}")
        ok = False
    return ok


if __name__ == "__main__":
    raise SystemExit(main())
