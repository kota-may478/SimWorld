#!/usr/bin/env python3
"""UE Play 中に柱スキャンだけ実行し costmap_obstacles.png を保存する。"""
from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from path_planning_costmap import build_uniform_costmap, plot_costmap_with_paths
from costmap_obstacle_scan import enrich_costmap_with_obstacles, obstacle_mask_to_world_pillars

from simworld.communicator.unrealcv import UnrealCV


def main() -> int:
    origin = (1425.755, -1711.4)
    ground_z = 3873.0
    ucv = UnrealCV(port=9000, ip="172.20.224.1")
    costmap = build_uniform_costmap(origin_xy=origin)
    result = enrich_costmap_with_obstacles(ucv, costmap, ground_z_cm=ground_z)
    print(result)
  # re-scan mask for pillar list (lightweight re-run not stored; print high-cost cells count)
    high = int((costmap.costs > 10).sum())
    print(f"high_cost_cells={high}")
    out = Path(__file__).resolve().parent / "costmap_obstacles.png"
    plot_costmap_with_paths(
        costmap,
        [],
        title="Costmap + obstacle scan",
        save_path=str(out),
        show=False,
    )
    print(f"saved {out}")
    ucv.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
