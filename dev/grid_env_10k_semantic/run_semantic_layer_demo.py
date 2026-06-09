#!/usr/bin/env python3
"""Run elevated semantic-layer demo on grid_100x100 PIE."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

import grid_env_10k_semantic as sem  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Semantic block layer demo (grid_100x100 PIE)")
    parser.add_argument(
        "--no-demo-wall",
        action="store_true",
        help="Skip elevated wall obstacle at (4,4)",
    )
    parser.add_argument(
        "--blocking",
        action="store_true",
        help="Spawn blocks as solid (T) instead of translucent (F)",
    )
    parser.add_argument(
        "--no-cleanup-first",
        action="store_true",
        help="Keep existing sem_* actors from a previous run",
    )
    parser.add_argument(
        "--cleanup-only",
        action="store_true",
        help="Remove sem_* actors and exit",
    )
    args = parser.parse_args()

    ucv, _ = sem.ensure_connection()
    if not ucv.client.isconnected():
        print("FAIL: UnrealCV not connected — open grid_100x100 and start PIE.", file=sys.stderr)
        return 1

    if args.cleanup_only:
        sem.cleanup_semantic_layer(ucv)
        return 0

    result = sem.run_semantic_layer_demo(
        ucv,
        spawn_demo_wall=not args.no_demo_wall,
        default_block_mode="T" if args.blocking else "F",
        cleanup_before=not args.no_cleanup_first,
    )
    counts = sem.summarize_semantics(result.semantics)
    print(
        f"SUCCESS: placed={len(result.blocks)} "
        f"wall/floor/air={counts} registry={result.registry_path}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
