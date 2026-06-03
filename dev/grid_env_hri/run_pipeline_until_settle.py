#!/usr/bin/env python3
"""Notebook と同じスポーン手順をクリーンアップ手前まで実行（検証用）。"""

from __future__ import annotations

import os
import sys

# 小規模既定（環境変数 GRID_N で上書き可）
os.environ.setdefault("GRID_N", "3")

import grid_env_hri_simulation as geh  # noqa: E402


def main() -> int:
    print(
        f"[Pipeline] GRID_N={geh.GRID_N}, hosts={geh._ue_host_candidates()}, "
        "stop before cleanup"
    )
    ucv, communicator = geh.ensure_connection()

    if not geh.spawn_fixed_floor(ucv):
        print("[Pipeline] floor spawn failed", file=sys.stderr)
        return 1

    cube_registry = geh.spawn_cubes(ucv, geh.GRID_N)
    human_name = geh.spawn_humanoid(communicator, ucv)
    robot_ok = geh.spawn_robot(ucv)

    if geh.SETTLE_AFTER_SPAWN_S > 0:
        print(f"[Pipeline] settle {geh.SETTLE_AFTER_SPAWN_S}s ...")
        import time

        time.sleep(geh.SETTLE_AFTER_SPAWN_S)

    geh.report_spawn_state(ucv, cube_registry, human_name)
    print("[Pipeline] OK (cleanup skipped)")
    print(f"  cubes={len(cube_registry)}, human={human_name}, robot_ok={robot_ok}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
