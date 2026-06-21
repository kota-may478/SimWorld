#!/usr/bin/env python3
"""Regenerate compact-nav PNG from saved npz + trajectory JSON."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="compact_nav")

from costmap_layers import LayeredCostmap  # noqa: E402
from paths import COMPACT_NAV_RUN_DIR  # noqa: E402
from placement import ensure_registry  # noqa: E402
from viz import NavTrace, save_compact_nav_artifacts  # noqa: E402


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--npz", type=Path, required=True)
    p.add_argument("--trajectory", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, default=COMPACT_NAV_RUN_DIR)
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    data = np.load(args.npz)
    layers = LayeredCostmap(
        l0=data["l0"],
        origin_xy=(float(data["origin_xy"][0]), float(data["origin_xy"][1])),
        resolution_cm=float(data["resolution_cm"]),
        lethal_cost=float(data["lethal_cost"]),
    )
    layers.l1 = data["l1"].astype(np.float32)
    layers.l2 = data["l2"].astype(np.float32)

    raw = json.loads(args.trajectory.read_text(encoding="utf-8"))
    trace = NavTrace(
        trajectory_local_cm=[tuple(p) for p in raw["trajectory_local_cm"]],
        planned_paths_local_cm=[[tuple(p) for p in path] for path in raw["planned_paths_local_cm"]],
        replan_events=raw.get("replan_events", []),
        l2_cell_count=int(raw.get("l2_cell_count", 0)),
        arrived=bool(raw.get("arrived", False)),
    )
    registry = ensure_registry()
    paths = save_compact_nav_artifacts(layers, registry, trace, output_dir=args.output_dir)
    print(paths.get("costmap_png"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
