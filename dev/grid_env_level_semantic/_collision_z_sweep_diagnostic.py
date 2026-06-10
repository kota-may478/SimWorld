#!/usr/bin/env python3
"""Probe hit vs Z at rooftop test XY (diagnostic)."""
from __future__ import annotations

import json
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
for p in (THIS_DIR, THIS_DIR.parent / "grid_env_hri", THIS_DIR.parent / "grid_env_10k"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import level_collision_probe as lcp  # noqa: E402

XY = (6285.0, 1185.0)
Z_SAMPLES = list(range(7200, 6400, -30))


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    ok, name = lcp.ensure_collision_probe(ucv)
    if not ok:
        print("spawn failed")
        return 1
    x, y = XY
    hits = []
    for z in Z_SAMPLES:
        raw = lcp.parse_probe_hit(lcp._vbp_probe_hit(ucv, name, x, y, float(z)))  # noqa: SLF001
        if lcp.probe_point_blocks(raw):
            hits.append((z, raw))
    print(f"XY={XY} hits={len(hits)}")
    for z, raw in hits[:10]:
        print(f"  z={z}: {json.dumps(raw)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
