#!/usr/bin/env python3
"""Spawn one layout variant in PIE for visual review."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from layout_variants import layout_id_for_index, layout_summary  # noqa: E402
from paths import site_transport_registry_path  # noqa: E402
from placement import ensure_registry  # noqa: E402
from spawn_pie import spawn_site_transport_scene  # noqa: E402
from pie_safety import PieSessionLost  # noqa: E402


def _print_layout_brief(registry) -> None:
    summary = layout_summary(registry)
    print(f"\n=== Layout {summary['layout_id']} ===")
    print(f"  transport: {summary['transport_bp']} @ {summary['transport_xy']}")
    print(f"  pit rect:  {summary['pit_rect']}")
    print(f"  props: sw={summary['sw_prop_count']} mid={summary['mid_prop_count']} "
          f"yard={summary['yard_decor_count']} roadblocks={summary['roadblock_count']}")
    print("  clutter:")
    for item in summary["props"]:
        if item["cluster"] == "no_entry_roadblock":
            continue
        flag = " [CARRY]" if item["transport"] else ""
        print(f"    {item['bp']:32s} {item['cluster']:18s} ({item['xy'][0]:.0f},{item['xy'][1]:.0f}){flag}")


def main() -> int:
    p = argparse.ArgumentParser(description="PIE preview of one site_transport layout")
    p.add_argument("--layout-id", default="layout_01", help="e.g. layout_01 .. layout_10")
    p.add_argument("--layout-index", type=int, default=None, help="1..10 (overrides --layout-id)")
    p.add_argument("--force-respawn", action="store_true", help="Destroy and re-spawn props")
    p.add_argument("--summary-only", action="store_true", help="Print JSON summary without PIE")
    p.add_argument("--write-summary", type=Path, default=None, help="Write layout summary JSON path")
    args = p.parse_args()

    layout_id = layout_id_for_index(args.layout_index) if args.layout_index else args.layout_id
    path = site_transport_registry_path(layout_id)
    if not path.is_file():
        print(f"[LayoutPreview] missing registry {path}; run generate_layouts.py first")
        return 1

    registry = ensure_registry(layout_id=layout_id)
    _print_layout_brief(registry)

    if args.write_summary is not None:
        args.write_summary.parent.mkdir(parents=True, exist_ok=True)
        args.write_summary.write_text(
            json.dumps(layout_summary(registry), indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        print(f"[LayoutPreview] summary written: {args.write_summary}")

    if args.summary_only:
        return 0

    try:
        rc, _ = spawn_site_transport_scene(
            layout_id=layout_id,
            force_respawn=args.force_respawn,
        )
    except PieSessionLost as exc:
        print(f"[LayoutPreview] ABORT: {exc}")
        return 3

    if rc == 0:
        print(f"[LayoutPreview] PIE spawn OK for {layout_id}")
    else:
        print(f"[LayoutPreview] PIE spawn finished with code {rc}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
