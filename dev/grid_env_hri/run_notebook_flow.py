#!/usr/bin/env python3
"""Notebook 相当の全セル（クリーンアップ含む）を順に実行して検証する。"""

from __future__ import annotations

import importlib
import os
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

# 小規模テスト
os.environ.setdefault("GRID_N", "3")
os.environ.setdefault("SETTLE_AFTER_SPAWN_S", "3")

ROOT = Path(__file__).resolve().parent.parent.parent
GEH_DIR = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(GEH_DIR) not in sys.path:
    sys.path.insert(0, str(GEH_DIR))

import grid_env_hri_simulation as geh  # noqa: E402
from simworld.communicator.communicator import Communicator  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402


def main() -> int:
    importlib.reload(geh)
    print(f"[Flow] hosts={geh._ue_host_candidates()}")

    ucv, communicator = geh.ensure_connection()
    assert ucv.client.isconnected()

    if not geh.spawn_fixed_floor(ucv):
        print("[Flow] floor failed", file=sys.stderr)
        return 1

    cube_registry = geh.spawn_cubes(ucv, geh.GRID_N)
    marker_registry: dict = {}
    if geh.SPAWN_DEMO_MODE_CUBES:
        marker_registry = geh.spawn_demo_mode_cubes(ucv)
    human_name = geh.spawn_humanoid(communicator, ucv)
    robot_ok = geh.spawn_robot(ucv)

    if geh.SETTLE_AFTER_SPAWN_S > 0:
        print(f"[Flow] settle {geh.SETTLE_AFTER_SPAWN_S}s")
        time.sleep(geh.SETTLE_AFTER_SPAWN_S)

    geh.report_spawn_state(
        ucv, cube_registry, human_name, marker_registry=marker_registry or None
    )

    print("[Flow] cleanup ...")
    geh.cleanup_spawned(
        ucv, cube_registry.keys(), marker_ids=marker_registry.keys()
    )
    if human_name:
        geh.destroy_if_exists(ucv, human_name)

    print(
        f"[Flow] OK cubes={len(cube_registry)} human={human_name} robot_ok={robot_ok}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
