#!/usr/bin/env python3
"""Non-destructive yaw sweep for AI Perception sight debugging."""

from __future__ import annotations

import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT / "grid_env_hri"))
sys.path.insert(0, str(_ROOT / "grid_env_level_nav"))
sys.path.insert(0, str(_ROOT / "grid_env_level_nav" / "scenarios" / "site_transport_20m"))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import level_nav_robot as lnr  # noqa: E402
import ue_client_guard  # noqa: E402
from grid_env_10k_pie_patrol import get_pos2d, get_yaw  # noqa: E402
from l2_geom import GeomPerceptionConfig, geom_detections  # noqa: E402
from l2_sight import fetch_ue_sight_targets  # noqa: E402
from placement import build_registry, to_placement_registry  # noqa: E402
from pie_safety import tick_settle  # noqa: E402
from robot_sensor import SENSOR_CAM_FORWARD_OFFSET_CM, SENSOR_FOV_DEG  # noqa: E402
from runtime_sight_sources import ensure_runtime_site20_sight_sources  # noqa: E402

ROBOT = "GridEnv_SpotRobot"


def main() -> int:
    reg = build_registry()
    placement = to_placement_registry(reg)
    cfg = GeomPerceptionConfig(
        fov_deg=SENSOR_FOV_DEG,
        max_range_cm=650.0,
        sensor_forward_cm=SENSOR_CAM_FORWARD_OFFSET_CM,
    )
    yaws = (0.0, 30.0, 45.0, 60.0, 90.0, -30.0, -45.0, -60.0)

    with ue_client_guard.exclusive_ue_client_lock():
        ucv, _ = ue_client_guard.prepare_ue_connection(force_new=True)
        lnr.ensure_spotdog_sight_controller(ucv, ROBOT)
        ensure_runtime_site20_sight_sources(ucv)
        for yaw in yaws:
            ucv.set_orientation((0.0, yaw, 0.0), ROBOT)
            tick_settle(ucv, settle_s=1.5, ticks=8)
            robot_xy = get_pos2d(ucv, ROBOT)
            robot_yaw = get_yaw(ucv, ROBOT)
            geom = geom_detections(robot_xy, robot_yaw, placement, config=cfg)
            parsed = fetch_ue_sight_targets(ucv, ROBOT)
            print(
                f"YAW {yaw:+.1f} actual={robot_yaw:+.1f} "
                f"geom={[d.slot_id for d in geom[:5]]} sight={parsed}"
            )
            time.sleep(0.5)
        geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
