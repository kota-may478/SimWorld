#!/usr/bin/env python3
"""PIE four-room scenario: layout (1,1)-(30,30), SpotDog round trip to (20,20).

Prerequisites: UE Editor grid_100x100 map, PIE running.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Set, Tuple

BlockIndex = Tuple[int, int]


def _find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / "setup.py").exists() and (candidate / "simworld").is_dir():
            return candidate
    return here.parent.parent


ROOT = _find_project_root()
G10K_DIR = ROOT / "dev" / "grid_env_10k"
GEH_DIR = ROOT / "dev" / "grid_env_hri"
for p in (ROOT, GEH_DIR, G10K_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_hri_simulation as geh  # noqa: E402
import grid_env_10k as g10k  # noqa: E402
import grid_env_10k_four_rooms_layout as layout  # noqa: E402
import grid_env_10k_pie_patrol as patrol  # noqa: E402
from simworld.communicator.communicator import Communicator  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

ROBOT_START_CELL = layout.ROBOT_START_CELL
ROBOT_GOAL_CELL = layout.ENTITY_GOAL_CELL
GOAL_DWELL_S = 3.0


@dataclass
class FourRoomsResult:
    layout_ok: bool
    layout_errors: tuple[str, ...]
    robot_spawned: bool
    outbound_arrived: bool
    return_arrived: bool
    start_xy: patrol.WorldXY
    goal_xy: patrol.WorldXY
    final_xy: patrol.WorldXY
    return_dist_cm: float


def apply_four_rooms_layout_pie(
    ucv: UnrealCV,
    registry: Dict[BlockIndex, str],
    *,
    room_layout: layout.FourRoomsLayout,
    reset_region_to_f: bool = False,
) -> Tuple[int, int]:
    """Apply walls/pillar/entity as T; optionally reset whole region to F first."""
    total_fail = 0
    if reset_region_to_f:
        _, fail_n = patrol.set_blocks_mode_pie(
            ucv,
            registry,
            list(room_layout.region_cells),
            "F",
            progress_every=100,
            label="four_rooms reset F",
        )
        total_fail += fail_n

    ok_n, fail_n = patrol.set_blocks_mode_pie(
        ucv,
        registry,
        list(room_layout.ue_solid_cells),
        "T",
        progress_every=50,
        label="four_rooms solid T",
    )
    total_fail += fail_n
    return ok_n, total_fail


def ensure_single_ue_session(
    *,
    ucv: Optional[UnrealCV] = None,
    communicator: Optional[Communicator] = None,
    force_new: bool = False,
) -> Tuple[UnrealCV, Communicator]:
    """One UnrealCV TCP client on :9000 (release stale session when ``force_new``)."""
    if force_new:
        g10k.release_connection(ucv, communicator=communicator)
        ucv = None
        communicator = None
    return g10k.ensure_connection(
        force_new=force_new,
        ucv=ucv,
        communicator=communicator,
    )


def run_four_rooms_scenario(
    *,
    robot_start_cell: BlockIndex = ROBOT_START_CELL,
    robot_goal_cell: BlockIndex = ROBOT_GOAL_CELL,
    goal_dwell_s: float = GOAL_DWELL_S,
    skip_layout_if_already_applied: bool = False,
    skip_robot_probe: bool = True,
    force_new_connection: bool = False,
    ucv: Optional[UnrealCV] = None,
    communicator: Optional[Communicator] = None,
) -> FourRoomsResult:
    room_layout = layout.build_four_rooms_layout(entity_cell=robot_goal_cell)
    layout_errors = tuple(layout.validate_room_adjacency(room_layout))
    layout_ok = len(layout_errors) == 0
    if layout_errors:
        print("[Layout] validation errors:")
        for msg in layout_errors:
            print(f"  - {msg}")
    else:
        print(
            f"[Layout] OK region=({layout.REGION_GX0},{layout.REGION_GY0})-"
            f"({layout.REGION_GX1},{layout.REGION_GY1}) "
            f"walls={len(room_layout.wall_cells)} "
            f"entity={room_layout.entity_cells} "
            f"door={layout.DOOR_WIDTH_CM:.0f}cm"
        )

    ucv, communicator = ensure_single_ue_session(
        ucv=ucv,
        communicator=communicator,
        force_new=force_new_connection,
    )
    patrol.prepare_pie_rerun(ucv)

    scripts_dir = str(G10K_DIR / "scripts")
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    from mount_simworld_runtime_paks_pie import (  # noqa: WPS433
        mount_paks,
        probe_robot_spawn,
    )

    if not mount_paks(ucv):
        raise RuntimeError(patrol.robot_bp_setup_hint())
    if skip_robot_probe:
        print("[Robot] skip BP probe (reuse session)")
    elif not probe_robot_spawn(ucv):
        raise RuntimeError(patrol.robot_bp_setup_hint())

    start_xy = patrol.block_index_to_world_xy_cm(*robot_start_cell)
    goal_xy = patrol.block_index_to_world_xy_cm(*robot_goal_cell)

    registry = patrol.build_pie_block_registry(
        ucv,
        cells=set(room_layout.region_cells),
    )

    if skip_layout_if_already_applied:
        print("[Scenario] skip layout apply (assume region already configured)")
    else:
        apply_four_rooms_layout_pie(
            ucv,
            registry,
            room_layout=room_layout,
            reset_region_to_f=False,
        )

    blocking: Set[BlockIndex] = set(room_layout.costmap_lethal_cells)
    costmap = patrol.build_costmap_from_blocking_cells(blocking)

    robot_spawned = patrol.spawn_robot_at(ucv, robot_start_cell)
    if not robot_spawned:
        return FourRoomsResult(
            layout_ok=layout_ok,
            layout_errors=layout_errors,
            robot_spawned=False,
            outbound_arrived=False,
            return_arrived=False,
            start_xy=start_xy,
            goal_xy=goal_xy,
            final_xy=start_xy,
            return_dist_cm=float("inf"),
        )

    time.sleep(0.5)
    print(f"[Scenario] outbound {robot_start_cell} -> {robot_goal_cell}")
    outbound_arrived = patrol.robot_navigate_astar(
        ucv,
        costmap,
        goal_xy,
        tolerance_cm=patrol.ARRIVE_TOLERANCE_CM,
        label="outbound",
    )

    if outbound_arrived and goal_dwell_s > 0:
        print(f"[Scenario] dwell {goal_dwell_s:.1f}s at goal ...")
        time.sleep(goal_dwell_s)

    print(f"[Scenario] return {robot_goal_cell} -> {robot_start_cell}")
    return_arrived = patrol.robot_navigate_astar(
        ucv,
        costmap,
        start_xy,
        tolerance_cm=patrol.RETURN_ARRIVE_TOLERANCE_CM,
        label="return",
    )

    final_xy = patrol.get_pos2d(ucv, geh.ROBOT_ACTOR_NAME)
    return_dist = patrol.dist2d(final_xy, start_xy)
    print(
        f"[Scenario] done layout_ok={layout_ok} outbound={outbound_arrived} "
        f"return={return_arrived} final_dist={return_dist:.1f} cm"
    )
    return FourRoomsResult(
        layout_ok=layout_ok,
        layout_errors=layout_errors,
        robot_spawned=True,
        outbound_arrived=outbound_arrived,
        return_arrived=return_arrived,
        start_xy=start_xy,
        goal_xy=goal_xy,
        final_xy=final_xy,
        return_dist_cm=return_dist,
    )


def four_rooms_success(result: FourRoomsResult) -> bool:
    return (
        result.layout_ok
        and result.robot_spawned
        and result.outbound_arrived
        and result.return_arrived
        and result.return_dist_cm <= patrol.RETURN_ARRIVE_TOLERANCE_CM
    )


def main() -> int:
    import os

    wait_s = float(os.environ.get("UE_WAIT_S", "180"))
    wait_script = G10K_DIR / "wait_for_ue_port.py"
    py = sys.executable
    import subprocess

    g10k.release_connection()
    print(f"[UE] waiting for PIE UnrealCV on :9000 (up to {wait_s:.0f}s) ...")
    proc = subprocess.run(
        [py, str(wait_script), str(wait_s)],
        capture_output=True,
        text=True,
    )
    print(proc.stdout.strip() or proc.stderr.strip())
    if proc.returncode != 0:
        print(
            "[UE] TIMEOUT — PIE を Play したうえで WSL の :9000 接続が 1 本だけであることを確認してください。"
        )
        return 1

    ucv: Optional[UnrealCV] = None
    communicator: Optional[Communicator] = None
    try:
        ucv, communicator = ensure_single_ue_session(force_new=True)
        result = run_four_rooms_scenario(
            skip_robot_probe=True,
            skip_layout_if_already_applied=False,
            force_new_connection=False,
            ucv=ucv,
            communicator=communicator,
        )
        ok = four_rooms_success(result)
        print(f"[Result] SUCCESS={ok}")
        return 0 if ok else 1
    finally:
        g10k.release_connection(ucv, communicator=communicator)
        print("[UE] session released (single client closed)")


if __name__ == "__main__":
    raise SystemExit(main())
