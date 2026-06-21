#!/usr/bin/env python3
"""Spot-check NavProjectPoint + XY tolerance (PIE required)."""

from __future__ import annotations

import sys
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
_ROOT = _THIS_DIR.parent.parent
for _p in (_THIS_DIR, _ROOT / "dev" / "grid_env_hri", _ROOT / "dev" / "grid_env_10k"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import grid_env_10k as g10k  # noqa: E402
import nav_query as nq  # noqa: E402
from level_coords import NAV_PROJECT_PROBE_Z_CM, local_xy_to_world  # noqa: E402
from l0_nav_mask import (  # noqa: E402
    COSTMAP_LETHAL_COST,
    projection_xy_distance_cm,
    project_cell_to_cost,
)
from work_region import DEFAULT_XY_TOLERANCE_CM  # noqa: E402

Z_CM = NAV_PROJECT_PROBE_Z_CM

# label, local_xy — edit hole/pillar coords after measuring in Editor if needed
SPOTS = [
    ("floor_corner_A", (0.0, 0.0)),
    ("floor_center", (3500.0, 3950.0)),
    ("smoke_start", (500.0, 500.0)),
    ("smoke_goal", (5000.0, 6000.0)),
    # TODO: replace with measured hole/pillar centers from your Level
    ("hole_candidate", (2000.0, 3000.0)),
    ("pillar_candidate", (2500.0, 3500.0)),
]


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    ok, actor = nq.ensure_nav_query_service(ucv)
    if not ok:
        print("[spot] ERROR: NavQueryService unavailable")
        return 1

    print(f"[spot] actor={actor!r} xy_tol={DEFAULT_XY_TOLERANCE_CM}cm z_probe={Z_CM}")
    print(f"{'label':<18} {'ok':>4} {'xy_dist':>8} {'cost':>12}  local_xy")
    failures = 0
    for label, (lx, ly) in SPOTS:
        wx, wy = local_xy_to_world(lx, ly)
        raw = nq.nav_project_point(ucv, actor, wx, wy, Z_CM)
        xy_dist = projection_xy_distance_cm(raw, wx, wy)
        cost = project_cell_to_cost(raw, wx=wx, wy=wy, wz=Z_CM)
        walkable = cost < COSTMAP_LETHAL_COST * 0.5
        dist_s = f"{xy_dist:.1f}" if xy_dist is not None else "—"
        print(
            f"{label:<18} {str(raw.get('ok')):>4} {dist_s:>8} "
            f"{'walkable' if walkable else 'LETHAL':>12}  ({lx:.0f},{ly:.0f})"
        )
        if xy_dist is not None and xy_dist > 100.0 and raw.get("ok"):
            print(
                f"  WARN: large snap {xy_dist:.0f}cm — "
                f"set BP_NavQueryService ProjectExtentCm≈30"
            )
            failures += 1
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
