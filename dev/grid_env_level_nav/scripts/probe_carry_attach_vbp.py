#!/usr/bin/env python3
"""Probe BP_SpotRobot carry attach vbp (PIE + GridEnv_SpotRobot required)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import ue_client_guard  # noqa: E402
from carry import (  # noqa: E402
    CARRY_SOCKET_NAME,
    VBP_ATTACH_CARRY,
    VBP_PROBE_ATTACH,
    attach_carry_to_robot_bone,
    probe_carry_attach_vbp,
    reset_carry_attach_state,
)

ROBOT = geh.ROBOT_ACTOR_NAME


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Probe AttachCarryActor vbp on SpotDog")
    p.add_argument("--robot", default=ROBOT)
    p.add_argument("--carry", default="site20_carry", help="Try attach if actor exists")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    reset_carry_attach_state()
    ucv, _ = ue_client_guard.prepare_ue_connection(force_new=True)
    if ucv is None:
        print("[carry-probe] FAIL: no UnrealCV connection (start PIE on Level)")
        return 1

    for cmd_name in (VBP_PROBE_ATTACH, VBP_ATTACH_CARRY):
        try:
            raw = geh._ue_request(ucv, f"vbp {args.robot} {cmd_name} __probe__", timeout_s=10.0)  # noqa: SLF001
        except Exception as exc:
            print(f"[carry-probe] {cmd_name}: ERROR {exc}")
            continue
        print(f"[carry-probe] {cmd_name} raw={raw!r}")

    available = probe_carry_attach_vbp(ucv, args.robot)
    print(
        f"[carry-probe] AttachCarryActor vbp: {'AVAILABLE' if available else 'NOT FOUND'} "
        f"(socket={CARRY_SOCKET_NAME!r})"
    )
    if available and geh.actor_exists(ucv, args.carry):
        ok = attach_carry_to_robot_bone(ucv, args.carry, args.robot)
        print(f"[carry-probe] trial attach {args.carry!r}: {'OK' if ok else 'FAIL'}")
    elif available:
        print(f"[carry-probe] skip trial attach — {args.carry!r} not in level")
    return 0 if available else 2


if __name__ == "__main__":
    raise SystemExit(main())
