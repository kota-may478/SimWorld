#!/usr/bin/env python3
"""Level PIE: which actors/BPs support GetCollisionNum and related vbp calls."""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
from level_semantic_scan import parse_collision_counts  # noqa: E402

PROBE_LOC = (6285.0, 1185.0, 6873.5)
CANDIDATES = [
    ("/Game/CityDatabase/blueprints/BP_Box.BP_Box_C", "level_diag_box"),
    (geh.CUBE_BP, "level_diag_cube"),
    ("/Game/Robot_Dog/Blueprint/BP_SpotRobot.BP_SpotRobot_C", "level_diag_robot"),
]


def try_vbp(ucv, name: str, fn: str, *args: str) -> str:
    arg_s = " ".join(str(a) for a in args)
    cmd = f"vbp {name} {fn} {arg_s}".strip()
    try:
        return str(geh._ue_request(ucv, cmd, timeout_s=8.0))
    except Exception as exc:
        return f"error {exc}"


def test_bp(ucv, bp: str, name: str) -> None:
    geh.destroy_if_exists(ucv, name)
    if not geh.spawn_bp(ucv, bp, name):
        print(f"  {name}: spawn_failed bp={bp}")
        return
    ucv.set_scale((0.15, 0.15, 0.15), name)
    ucv.set_physics(name, False)
    ucv.set_collision(name, True)
    ucv.set_movable(name, True)
    ucv.set_location(PROBE_LOC, name)
    time.sleep(0.15)
    try:
        ucv.tick()
    except Exception:
        pass
    time.sleep(0.1)
    raw = ucv.get_collision_num(name)
    counts = parse_collision_counts(raw)
    total = sum(int(v) for v in counts.values()) if counts else 0
    print(f"  {name}: GetCollisionNum raw={raw!r} total={total}")
    for fn in ("LineTraceDown", "ProbeOverlap", "SemanticProbe", "GetHit"):
        resp = try_vbp(ucv, name, fn)
        if "error" not in resp.lower() or "Invalid" not in resp:
            print(f"    {fn}: {resp[:120]}")
    geh.destroy_if_exists(ucv, name)


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    objs = {str(o) for o in ucv.get_objects().tolist()}
    print(f"objects={len(objs)} probe_loc={PROBE_LOC}")
    for bp, name in CANDIDATES:
        print(f"\n=== {bp} ===")
        test_bp(ucv, bp, name)
    # Existing scene actors that might already expose GetCollisionNum
    print("\n=== existing actors (sample) ===")
    for name in sorted(objs):
        if any(k in name for k in ("Robot", "Pawn", "Spot", "Dog", "Probe", "Box")):
            raw = ucv.get_collision_num(name)
            if raw and "Invalid" not in str(raw):
                print(f"  {name}: {raw!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
