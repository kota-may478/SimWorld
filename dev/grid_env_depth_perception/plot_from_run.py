#!/usr/bin/env python3
"""Regenerate plots (+ RMSE) from a saved depth_recognition_*.json run."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from plot_results import plot_distance_and_bearing, summarize_rmse  # noqa: E402
from prop_placement import PlacementRegistry, PropPlacement  # noqa: E402
from simple_nav import NavigationRunResult, TimeSeriesSample  # noqa: E402


def _load_run(path: Path) -> tuple[PlacementRegistry, list[NavigationRunResult]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    props = tuple(PropPlacement.from_dict(p) for p in data["props"])
    registry = PlacementRegistry(
        version=1,
        seed=int(data.get("registry_seed", 42)),
        prop_count=len(props),
        region_x_max_cm=3000.0,
        region_y_max_cm=3000.0,
        exclusion_cm=500.0,
        spotdog_spawn_local_cm=(100.0, 100.0),
        props=props,
    )
    runs: list[NavigationRunResult] = []
    for raw in data.get("runs", []):
        samples = [
            TimeSeriesSample(
                t_s=float(s["t_s"]),
                robot_xy=(float(s["robot_xy"][0]), float(s["robot_xy"][1])),
                robot_yaw_deg=float(s["robot_yaw_deg"]),
                estimates=s.get("estimates", {}),
                ground_truth=s.get("ground_truth", {}),
            )
            for s in raw.get("samples", [])
        ]
        runs.append(
            NavigationRunResult(
                target_prop_type_id=str(raw["target_prop_type_id"]),
                samples=samples,
                reached=bool(raw.get("reached", False)),
            )
        )
    return registry, runs


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("json_path", type=Path)
    args = parser.parse_args()
    registry, runs = _load_run(args.json_path)
    rmse = summarize_rmse(runs, registry)
    stem = args.json_path.with_suffix("")
    dist_png = Path(f"{stem}_distance.png")
    bear_png = Path(f"{stem}_bearing.png")
    plot_distance_and_bearing(runs, registry, dist_png, bear_png, rmse)
    rmse_path = Path(f"{stem}_rmse.json")
    rmse_path.write_text(json.dumps(rmse, indent=2), encoding="utf-8")
    print(f"Wrote {dist_png}")
    print(f"Wrote {bear_png}")
    print(f"RMSE: {rmse}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
