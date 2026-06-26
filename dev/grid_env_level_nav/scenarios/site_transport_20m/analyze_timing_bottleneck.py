#!/usr/bin/env python3
"""Compare timing JSON files and print bottleneck breakdown."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Tuple


def _load(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _pct(part_ms: float, wall_ms: float) -> float:
    if wall_ms <= 0:
        return 0.0
    return 100.0 * part_ms / wall_ms


def _leg_row(timing: Dict[str, Any], label: str) -> Dict[str, Any]:
    for row in timing.get("per_leg", []):
        if row.get("label") == label:
            return row
    return {}


def _exclusive_buckets(row: Dict[str, Any]) -> List[Tuple[str, float, str]]:
    """Buckets that are mostly non-overlapping for wall-time attribution."""
    items = [
        ("translate+rotate (move sub)", row.get("translate_ms", 0) + row.get("rotate_ms", 0), "UE dog_move / dog_rotate"),
        ("settle", row.get("settle_ms", 0), "tick_settle after moves / warmup"),
        ("standoff backoff", row.get("standoff_ms", 0), "perception/move standoff backoffs"),
        ("replan (A*)", row.get("replan_ms", 0), "planner_server A* only"),
        ("sight_registry", row.get("sight_registry_ms", 0), "ObjectRegistry FOV update"),
        ("l2_depth update", row.get("l2_update_ms", 0), "update_l2_depth rasterize"),
        ("depth_refresh", row.get("depth_refresh_ms", 0), "depth cache miss outside perceive label"),
        ("depth_fetch (in_perceive flag)", row.get("depth_fetch_ms", 0), "depth fetch tagged in_perceive"),
        ("camera_settle", row.get("camera_settle_ms", 0), "sensor camera pose settle"),
        ("pose_query", row.get("pose_query_ms", 0), "get_pos2d/get_yaw (may overlap other buckets)"),
        ("prefetch_hit", row.get("prefetch_hit_ms", 0), "async depth prefetch bookkeeping"),
        ("loop_residual", row.get("loop_residual_ms", 0), "unbucketed loop time"),
    ]
    return [(name, float(ms), note) for name, ms, note in items if float(ms) > 0.0]


def _report(label: str, timing: Dict[str, Any]) -> None:
    print(f"\n{'=' * 72}")
    print(f"  {label}")
    print(f"{'=' * 72}")
    totals = timing.get("totals", {})
    for leg_name in ("leg1", "leg2"):
        row = _leg_row(timing, leg_name)
        if not row:
            continue
        wall_s = row.get("wall_time_s")
        if wall_s is None:
            key = f"{leg_name}_time_s"
            wall_s = timing.get(key)
        wall_ms = float(wall_s or 0) * 1000.0
        perceive_ms = float(row.get("perceive_ms", 0))
        accounted_ms = float(row.get("accounted_ms", 0))
        print(f"\n--- {leg_name} wall={wall_s:.1f}s accounted={accounted_ms/1000:.1f}s ---")
        print(
            f"perceive_ms={perceive_ms/1000:.1f}s ({_pct(perceive_ms, wall_ms):.0f}% of wall) "
            f"| move_ms={row.get('move_ms', 0)/1000:.1f}s "
            f"| settle_ms={row.get('settle_ms', 0)/1000:.1f}s"
        )
        print(
            f"depth_misses={row.get('depth_cache_misses')} prefetch_hits={row.get('prefetch_hits')} "
            f"standoff_events={row.get('standoff_events')}"
        )
        print("\nAttribution (note: pose_query/depth overlap perceive wall-clock):")
        buckets = sorted(_exclusive_buckets(row), key=lambda x: x[1], reverse=True)
        for name, ms, note in buckets[:8]:
            print(f"  {ms/1000:7.1f}s ({_pct(ms, wall_ms):5.1f}%)  {name:<28}  {note}")

    print("\n--- mission totals ---")
    nav_wall = timing.get("nav_wall_time_s")
    if nav_wall:
        print(f"nav_wall_time_s={nav_wall}")
    t = totals
    print(
        f"perceive={t.get('perceive_ms', 0)/1000:.1f}s move={t.get('move_ms', 0)/1000:.1f}s "
        f"settle={t.get('settle_ms', 0)/1000:.1f}s pose_query={t.get('pose_query_ms', 0)/1000:.1f}s"
    )


def _delta(fast: Dict[str, Any], slow: Dict[str, Any]) -> None:
    print(f"\n{'=' * 72}")
    print("  DELTA (slow - fast)")
    print(f"{'=' * 72}")
    for leg in ("leg1", "leg2"):
        f = _leg_row(fast, leg)
        s = _leg_row(slow, leg)
        f_wall = float(f.get("wall_time_s", 0)) * 1000
        s_wall = float(s.get("wall_time_s", 0)) * 1000
        print(f"\n{leg}: wall +{(s_wall - f_wall)/1000:.1f}s")
        for key in (
            "perceive_ms",
            "move_ms",
            "settle_ms",
            "standoff_ms",
            "translate_ms",
            "rotate_ms",
            "sight_registry_ms",
            "depth_refresh_ms",
            "pose_query_ms",
            "replan_ms",
            "loop_residual_ms",
        ):
            dv = float(s.get(key, 0)) - float(f.get(key, 0))
            if abs(dv) < 500:
                continue
            print(f"  {key}: +{dv/1000:.1f}s")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--fast", type=Path, required=True)
    p.add_argument("--slow", type=Path, required=True)
    args = p.parse_args()
    fast = _load(args.fast)
    slow = _load(args.slow)
    _report("FAST baseline", fast)
    _report("SLOW (recent)", slow)
    _delta(fast, slow)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
