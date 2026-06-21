#!/usr/bin/env python3
"""Smoke test: NavProjectPoint on corner A (requires PIE + BP_NavQueryService)."""
from __future__ import annotations

import json
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import level_coords as lc  # noqa: E402
import nav_query as nq  # noqa: E402

# Interior point (walkable on Level NavMesh) + nav probe Z (ProjectExtentCm ≈ 30).
TEST_LOCAL_XY = (1500.0, 1500.0)
TEST_WORLD_XYZ = (
    lc.local_xy_to_world(*TEST_LOCAL_XY)[0],
    lc.local_xy_to_world(*TEST_LOCAL_XY)[1],
    lc.NAV_PROJECT_PROBE_Z_CM,
)


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    ok, name = nq.ensure_nav_query_service(ucv, probe_xyz=TEST_WORLD_XYZ)
    if not ok:
        print("FAIL: NavQueryService unavailable — complete Phase 2 native install + BP")
        return 1

    x, y, z = TEST_WORLD_XYZ
    raw = nq.nav_project_point(ucv, name, x, y, z)
    print(f"NavProjectPoint raw={json.dumps(raw, ensure_ascii=False)}")
    if raw.get("ok"):
        print(f"OK: projected ({raw['x']}, {raw['y']}, {raw['z']})")
        return 0

    print(f"FAIL: {raw.get('error', raw)}")
    if raw.get("error") == "no_projection":
        print("hint: Build Paths (Phase 1-4) and check Nav Mesh Bounds Volume")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
