#!/usr/bin/env python3
"""Debug GetCollisionNum on robot vs probe box."""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import time

from simworld.communicator.unrealcv import UnrealCV

from costmap_obstacle_scan import COSTMAP_PROBE_NAME, parse_collision_counts, spawn_costmap_probe

ROBOT_BP = "/Game/Robot_Dog/Blueprint/BP_SpotRobot.BP_SpotRobot_C"
ROBOT_NAME = "MT_SpotRobot"


def main() -> int:
    ucv = UnrealCV(port=9000, ip="172.20.224.1")
    objs = {str(n) for n in ucv.get_objects().tolist()}
    print("total objects", len(objs))
    if ROBOT_NAME not in objs:
        ucv.spawn_bp_asset(ROBOT_BP, ROBOT_NAME)
        ucv.set_collision(ROBOT_NAME, True)
        ucv.set_movable(ROBOT_NAME, True)
        time.sleep(0.3)

    loc = (2000.0, -1500.0, 3873.0)
    ucv.set_location(loc, ROBOT_NAME)
    time.sleep(0.25)
    raw = ucv.get_collision_num(ROBOT_NAME)
    print(f"{ROBOT_NAME} at {loc}: {raw!r} -> {parse_collision_counts(raw)}")

    spawn_costmap_probe(ucv, loc)
    time.sleep(0.25)
    raw = ucv.get_collision_num(COSTMAP_PROBE_NAME)
    print(f"{COSTMAP_PROBE_NAME}: {raw!r} -> {parse_collision_counts(raw)}")
    ucv.destroy(COSTMAP_PROBE_NAME)
    ucv.destroy(ROBOT_NAME)
    ucv.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
