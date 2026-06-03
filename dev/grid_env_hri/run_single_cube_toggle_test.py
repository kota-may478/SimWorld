#!/usr/bin/env python3
"""単一 TransparentCube の SetBlocking OFF/ON 切替を Robot 通過試験で検証する。"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path

os.environ.setdefault("SPAWN_DEMO_MODE_CUBES", "0")
os.environ.setdefault("GRID_N", "0")

ROOT = Path(__file__).resolve().parent.parent.parent
GEH_DIR = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(GEH_DIR) not in sys.path:
    sys.path.insert(0, str(GEH_DIR))

import grid_env_hri_simulation as geh  # noqa: E402


def main() -> int:
    importlib.reload(geh)
    print(
        f"[SingleCubeTest] cube={geh.SINGLE_TOGGLE_CUBE_NAME!r} "
        f"map=({geh.SINGLE_TOGGLE_MAP_X_M}, {geh.SINGLE_TOGGLE_MAP_Y_M}) m"
    )

    ucv, _communicator = geh.ensure_connection()
    if not ucv.client.isconnected():
        print("[SingleCubeTest] UE not connected", file=sys.stderr)
        return 1

    if not geh.spawn_fixed_floor(ucv):
        print("[SingleCubeTest] floor spawn failed", file=sys.stderr)
        return 1

    if not geh.spawn_robot(ucv):
        print("[SingleCubeTest] robot spawn failed", file=sys.stderr)
        return 1

    ok = geh.run_single_cube_toggle_passage_suite(ucv)

    geh.destroy_if_exists(ucv, geh.SINGLE_TOGGLE_CUBE_NAME)
    geh.destroy_if_exists(ucv, geh.ROBOT_ACTOR_NAME)
    geh.destroy_if_exists(ucv, geh.FLOOR_ACTOR_NAME)

    if ok:
        print("[SingleCubeTest] ALL PASS — OFF/ON/OFF toggle verified with SpotDog")
        return 0
    print("[SingleCubeTest] FAILED — see [PassageTest] logs above", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
