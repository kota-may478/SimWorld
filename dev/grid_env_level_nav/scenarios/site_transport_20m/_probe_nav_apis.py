#!/usr/bin/env python3
"""Quick probe: NavRebuildDirtyRegion + local rebuild availability."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from bootstrap import setup_paths  # noqa: E402

setup_paths(scenario="site_transport_20m")

import grid_env_hri_simulation as geh  # noqa: E402
import nav_query as nq  # noqa: E402


def main() -> int:
    ucv, _ = geh.reconnect_if_needed()
    ok, nav_actor = nq.ensure_nav_query_service(ucv)
    if not ok:
        print("NavQueryService unavailable")
        return 1
    dirty_probe = nq.nav_rebuild_dirty_region(
        ucv,
        nav_actor,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        margin_cm=0.0,
    )
    local_ok = nq.nav_local_rebuild_api_available(ucv, nav_actor)
    validated_ok = nq.nav_validated_api_available(ucv, nav_actor)
    runtime_ok = nq.nav_runtime_api_available(ucv, nav_actor)
    print("[Probe] nav_actor:", nav_actor)
    print("[Probe] dirty_region_probe:", json.dumps(dirty_probe, ensure_ascii=False))
    print("[Probe] local_rebuild_api_available:", local_ok)
    print("[Probe] validated_api_available:", validated_ok)
    print("[Probe] runtime_api_available:", runtime_ok)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
