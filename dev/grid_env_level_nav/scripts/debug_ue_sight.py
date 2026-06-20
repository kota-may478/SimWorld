#!/usr/bin/env python3
"""Debug AI Perception: compare geom FOV vs UE sight vbp."""

from __future__ import annotations

import sys
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
from l2_geom import geom_detections, GeomPerceptionConfig  # noqa: E402
from l2_sight import fetch_ue_sight_targets  # noqa: E402
from placement import build_registry, to_placement_registry  # noqa: E402
from pie_safety import tick_settle  # noqa: E402
from robot_sensor import SENSOR_CAM_FORWARD_OFFSET_CM, SENSOR_FOV_DEG  # noqa: E402
from runtime_sight_sources import ensure_runtime_site20_sight_sources  # noqa: E402
from region import ROBOT_START_LOCAL_CM  # noqa: E402

ROBOT = "GridEnv_SpotRobot"


def main() -> int:
    reg = build_registry()
    placement = to_placement_registry(reg)
    cfg = GeomPerceptionConfig(
        fov_deg=SENSOR_FOV_DEG,
        max_range_cm=650.0,
        sensor_forward_cm=SENSOR_CAM_FORWARD_OFFSET_CM,
    )

    with ue_client_guard.exclusive_ue_client_lock():
        ucv, _ = ue_client_guard.prepare_ue_connection(force_new=True)
        tick_settle(ucv, settle_s=3.0, ticks=15)
        lnr.soft_reset_level_spotdog(ucv, ROBOT_START_LOCAL_CM)
        lnr.ensure_spotdog_sight_controller(ucv, ROBOT)
        ensure_runtime_site20_sight_sources(ucv)
        ucv.set_orientation((0.0, 0.0, 0.0), ROBOT)
        tick_settle(ucv, settle_s=2.0, ticks=10)

        names = geh.actor_names(ucv)
        controllers = [n for n in names if "SpotDogAIController" in n or "AIController" in n]
        site20 = [n for n in names if n.startswith("site20_")]
        print(f"CONTROLLERS ({len(controllers)}):", controllers[:5])
        print(f"SITE20_ACTORS ({len(site20)}):", site20[:8], "...")

        robot_xy = get_pos2d(ucv, ROBOT)
        robot_yaw = get_yaw(ucv, ROBOT)
        print(f"ROBOT xy={robot_xy} yaw={robot_yaw:.1f}°")

        geom = geom_detections(robot_xy, robot_yaw, placement, config=cfg)
        print(f"GEOM_VISIBLE ({len(geom)}):", [d.slot_id for d in geom[:12]])

        raw_pawn = ucv.client.request(f"vbp {ROBOT} GetVisibleSightTargetsJson")
        print("VBP_PAWN:", raw_pawn[:500])

        for ctrl in controllers[:3]:
            try:
                raw_ctrl = ucv.client.request(
                    f"vbp {ctrl} GetVisibleSightTargetsJson"
                )
                print(f"VBP_CTRL {ctrl}:", raw_ctrl[:500])
            except Exception as exc:
                print(f"VBP_CTRL {ctrl} ERR:", exc)

        parsed = fetch_ue_sight_targets(ucv, ROBOT)
        print("PARSED_SIGHT:", parsed)

        geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
