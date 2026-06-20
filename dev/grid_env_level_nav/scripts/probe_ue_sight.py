#!/usr/bin/env python3
"""Probe AI Perception sight vbp (PIE must be running). Uses exclusive UE lock + retries."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT / "grid_env_hri"))
sys.path.insert(0, str(_ROOT / "grid_env_level_nav"))
sys.path.insert(0, str(_ROOT / "grid_env_level_nav" / "scenarios" / "site_transport_20m"))

import grid_env_hri_simulation as geh  # noqa: E402
import ue_client_guard  # noqa: E402
from l2_sight import fetch_ue_sight_targets  # noqa: E402
from pie_safety import tick_settle  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description="Probe GetVisibleSightTargetsJson via UnrealCV")
    p.add_argument("--robot", default="GridEnv_SpotRobot")
    p.add_argument("--ticks", type=int, default=6, help="UE ticks before vbp (AI Perception warmup)")
    p.add_argument("--settle-s", type=float, default=2.0)
    args = p.parse_args()

    with ue_client_guard.exclusive_ue_client_lock():
        ucv, _ = ue_client_guard.prepare_ue_connection(force_new=True)
        tick_settle(ucv, settle_s=args.settle_s, ticks=args.ticks)
        raw = ucv.client.request(f"vbp {args.robot} GetVisibleSightTargetsJson")
        print("RAW:", raw[:1200] if len(raw) > 1200 else raw)
        parsed = fetch_ue_sight_targets(ucv, args.robot)
        print("PARSED:", parsed)
        try:
            loc = ucv.get_location(args.robot)
            print("ROBOT_LOC:", loc)
        except Exception as exc:
            print("ROBOT_LOC_ERR:", exc)
        geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
