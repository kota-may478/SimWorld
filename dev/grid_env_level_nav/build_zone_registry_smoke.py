#!/usr/bin/env python3
"""Generate zone_registry.json from local rectangles (smoke / manual zones)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from work_region import DEFAULT_RESOLUTION_CM  # noqa: E402
from zone_registry import ZoneRegistry  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--output",
        type=Path,
        default=_THIS_DIR / "cache" / "zone_registry.json",
    )
    p.add_argument("--resolution-cm", type=float, default=DEFAULT_RESOLUTION_CM)
    args = p.parse_args()

    reg = ZoneRegistry(resolution_cm=args.resolution_cm)
    # Example RoomD band across likely corridor (adjust after level_semantic scan).
    reg.add_rect_zone(
        "RoomD",
        2800.0,
        3200.0,
        3400.0,
        4000.0,
        closed_cost=1.0e9,
        note="smoke: manual rect; replace with level_semantic RoomD cells",
    )
    reg.save(args.output)
    print(f"[zones] wrote {args.output} RoomD cells={len(reg.zones['RoomD'].cells)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
