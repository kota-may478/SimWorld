#!/usr/bin/env python3
"""Smoke test: spawn BP_SemanticCollisionProbe and call ProbePointHit."""
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

TEST_XY_Z = (6285.0, 1185.0, 6873.5)


def main() -> int:
    ucv, _ = g10k.ensure_connection()
    ok, name = lcp.ensure_collision_probe(ucv)
    if not ok:
        print("FAIL: spawn BP_SemanticCollisionProbe — run create_semantic_collision_probe_editor.py")
        return 1
    x, y, z = TEST_XY_Z
    raw = lcp._vbp_probe_hit(ucv, name, x, y, z)  # noqa: SLF001
    hit = lcp.probe_point_hit(ucv, x, y, z, actor=name)
    print(f"ProbePointHit raw={json.dumps(raw, ensure_ascii=False)} hit={hit}")
    print(f"(probe {name!r} left in PIE for session reuse)")
    if raw.get("error") and "Invalid" in str(raw["error"]):
        print("FAIL: ProbePointHit not implemented on BP")
        return 2
    print("OK: collision probe responds")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
