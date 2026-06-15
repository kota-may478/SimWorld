#!/usr/bin/env python3
"""Print zone catalog picklist (labels + coordinates + cell counts)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from zone_catalog import ZoneCatalog  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--catalog",
        type=Path,
        default=_THIS_DIR / "cache" / "zone_catalog.template.json",
    )
    p.add_argument("--resolution-cm", type=float, default=30.0)
    p.add_argument("--json", action="store_true")
    args = p.parse_args()

    catalog = ZoneCatalog.load(args.catalog)
    rows = catalog.picklist_table(args.resolution_cm)
    if args.json:
        print(json.dumps(rows, indent=2, ensure_ascii=False))
        return 0

    print(f"Zone picklist @ {args.resolution_cm} cm — {args.catalog}")
    print(f"{'zone_id':<12} {'kind':<12} {'cells':>6}  local_xy_cm / world_xy_cm")
    print("-" * 72)
    for row in rows:
        geom = row.get("local_xy_cm") or row.get("world_xy_cm") or "—"
        print(
            f"{row['zone_id']:<12} {row['kind']:<12} {row['cell_count']:>6}  {geom}  {row.get('note','')}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
