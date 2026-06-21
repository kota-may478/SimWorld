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
from level_region import LOCKED_BLOCK_BOTTOM_Z_CM  # noqa: E402


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
    parser.add_argument(
        "--fixed-bottom-z",
        type=float,
        default=None,
        help="Skip height scan; label and place at this block bottom Z [cm]",
    )
    parser.add_argument(
        "--use-locked-z",
        action="store_true",
        help=f"Use calibrated LOCKED_BLOCK_BOTTOM_Z_CM ({LOCKED_BLOCK_BOTTOM_Z_CM} cm)",
    )
    parser.add_argument(
        "--labels-only",
        action="store_true",
        help="Phase 1 only: label + JSON checkpoints (no PIE spawn)",
    )
    parser.add_argument(
        "--spawn-only",
        action="store_true",
        help="Phase 2 only: spawn blocks from registry (labels must be complete)",
    )
    parser.add_argument(
        "--checkpoint-every",
        type=int,
        default=500,
        help="Save registry every N labeled cells (0=disable)",
    )
    parser.add_argument(
        "--no-resume",
        action="store_true",
        help="Ignore existing registry checkpoint",
    )
    parser.add_argument("--wait-ue", type=float, default=120.0)
    args = parser.parse_args()

    subgrid = _parse_subgrid(args.subgrid)
    if not lcp.wait_for_ue_port(args.wait_ue):
        print("ERROR: UnrealCV not reachable on any candidate host", file=sys.stderr)
        return 2
    fixed_z = args.fixed_bottom_z
    if args.use_locked_z:
        fixed_z = LOCKED_BLOCK_BOTTOM_Z_CM

    ucv, _ = lvl.ensure_connection()
    lvl.run_level_semantic_layer(
        ucv,
        cleanup_before=not args.no_cleanup,
        allow_large_region=args.allow_large_region,
        pie_subgrid=subgrid,
        fixed_block_bottom_z_cm=fixed_z,
        labels_only=args.labels_only,
        spawn_only=args.spawn_only,
        checkpoint_every=args.checkpoint_every,
        resume=not args.no_resume,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
