#!/usr/bin/env python3
"""PIE 接続・SpotDog BP・外周サンプル SetBlocking の短い診断。"""
from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for p in (ROOT, ROOT / "dev" / "grid_env_10k", ROOT / "dev" / "grid_env_hri"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
import grid_env_10k_pie_patrol as patrol  # noqa: E402

ROBOT_CANDIDATES = [
    geh.ROBOT_BP,
    "/Game/Robot_Dog/Blueprint/BP_SpotRobot_Child.BP_SpotRobot_Child_C",
    "/Game/Robot_Dog/Mesh/BP_Robot_Dog.BP_Robot_Dog_C",
]

def probe_robot_bps(ucv) -> list[tuple[str, bool]]:
    results: list[tuple[str, bool]] = []
    for bp in ROBOT_CANDIDATES:
        name = f"__probe_{bp.rsplit('/', 1)[-1].replace('.', '_')}__"
        geh.destroy_if_exists(ucv, name)
        ok = geh.spawn_bp(ucv, bp, name, timeout_s=30.0)
        if ok:
            geh.destroy_if_exists(ucv, name)
        results.append((bp, ok))
    return results


def probe_setblocking_one_cube(ucv, cube_names: list[str]) -> tuple[bool, str]:
    """最初に位置が取れた TransparentCube で SetBlocking を 1 回試す（全走査しない）。"""
    for name in cube_names[:80]:
        if geh.try_get_location_cm(ucv, name) is None:
            continue
        off = geh.set_cube_blocking_mode(ucv, name, blocking=False)
        on = geh.set_cube_blocking_mode(ucv, name, blocking=True)
        return off and on, name
    return False, ""


def main() -> int:
    print("=== PIE diagnostic ===")
    ucv, communicator = g10k.ensure_connection()
    if not ucv.client.isconnected():
        print("FAIL: UnrealCV not connected")
        return 1
    print("OK: UnrealCV connected")

    names = geh.actor_names(ucv)
    cubes = [n for n in names if "TransparentCube" in n or n.startswith("block_")]
    print(f"INFO: {len(names)} actors, {len(cubes)} cube-like")

    print("\n--- Robot BP spawn probes ---")
    robot_results = probe_robot_bps(ucv)
    any_robot = False
    for bp, ok in robot_results:
        status = "OK" if ok else "FAIL"
        print(f"  {status}: {bp}")
        any_robot = any_robot or ok

    print("\n--- Humanoid spawn probe ---")
    patrol.cleanup_runtime_agents(ucv)
    human = patrol.spawn_humanoid_at(communicator, ucv, patrol.HUMAN_CELL)
    print(f"  {'OK' if human else 'FAIL'}: humanoid -> {human}")

    print("\n--- SetBlocking probe (1 cube, max 80 actors) ---")
    t0 = time.monotonic()
    sb_ok, sb_name = probe_setblocking_one_cube(ucv, cubes)
    print(
        f"  {'OK' if sb_ok else 'FAIL'}: actor={sb_name or 'none'} "
        f"elapsed={time.monotonic() - t0:.1f}s"
    )

    print("\n=== Summary ===")
    print("  connection: OK")
    print(f"  robot_bp: {'OK' if any_robot else 'FAIL'}")
    print(f"  humanoid: {'OK' if human else 'FAIL'}")
    print(f"  set_blocking: {'OK' if sb_ok else 'FAIL'}")
    return 0 if any_robot and human and sb_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
