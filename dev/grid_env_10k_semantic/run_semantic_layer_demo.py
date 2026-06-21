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
        "--uniform-mode",
        choices=("F", "T"),
        help="Use one mode for all blocks instead of floor=F / air=T",
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
    parser.add_argument(
        "--wait-ue-s",
        type=float,
        default=180.0,
        help="Seconds to wait for UnrealCV port 9000 before failing (default 180)",
    )
    args = parser.parse_args()

    if not sem._wait_for_ue_port(args.wait_ue_s):
        print(
            f"FAIL: port 9000 not reachable after {args.wait_ue_s:.0f}s — start grid_100x100 PIE.",
            file=sys.stderr,
        )
        return 1

    ucv, _ = sem.ensure_connection()
    if not ucv.client.isconnected():
        print("FAIL: UnrealCV not connected — open grid_100x100 and start PIE.", file=sys.stderr)
        return 1

    if args.cleanup_only:
        sem.cleanup_semantic_layer(ucv)
        return 0

    result = sem.run_semantic_layer_demo(
        ucv,
        use_semantic_modes=args.uniform_mode is None,
        default_block_mode=args.uniform_mode or sem.DEFAULT_BLOCK_MODE,
        cleanup_before=not args.no_cleanup_first,
    )
    counts = {"wall": 0, "floor": 0, "air": 0}
    for s in result.semantics.values():
        counts[s] += 1
    print(
        f"SUCCESS: placed={len(result.blocks)} "
        f"wall/floor/air={counts} registry={result.registry_path}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
