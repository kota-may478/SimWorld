#!/usr/bin/env python3
"""CLI: Level semantic block layer (PIE)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

import grid_env_level_semantic as lvl  # noqa: E402
import level_camera_probe as lcp  # noqa: E402


def _parse_subgrid(raw: str | None) -> tuple[int, int, int, int] | None:
    if raw is None or raw.lower() in {"none", ""}:
        return None
    parts = [int(p.strip()) for p in raw.split(",")]
    if len(parts) != 4:
        raise ValueError("subgrid must be gx0,gy0,gx1,gy1")
    return parts[0], parts[1], parts[2], parts[3]


def main() -> int:
    parser = argparse.ArgumentParser(description="Level semantic block layer (PIE)")
    parser.add_argument(
        "--subgrid",
        default="1,1,5,5",
        help="gx0,gy0,gx1,gy1 for PIE test; use 'none' for full region",
    )
    parser.add_argument(
        "--allow-large-region",
        action="store_true",
        help="Required when subgrid=none (full ~74k cells)",
    )
    parser.add_argument("--no-cleanup", action="store_true")
    parser.add_argument("--wait-ue", type=float, default=120.0)
    args = parser.parse_args()

    subgrid = _parse_subgrid(args.subgrid)
    if not lcp.wait_for_ue_port(args.wait_ue):
        print("ERROR: UnrealCV not reachable on any candidate host", file=sys.stderr)
        return 2
    ucv, _ = lvl.ensure_connection()
    lvl.run_level_semantic_layer(
        ucv,
        cleanup_before=not args.no_cleanup,
        allow_large_region=args.allow_large_region,
        pie_subgrid=subgrid,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
