#!/usr/bin/env python3
"""フェーズ1: 10,000 半透明ブロック + 床 + Humanoid + Robot をスポーンして検証。"""

from __future__ import annotations

import os
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

import grid_env_10k as g10k  # noqa: E402


def main() -> int:
    dry = int(os.environ.get("BLOCK_SPAWN_DRY_RUN_N", "0"))
    grid_n = dry if dry > 0 else g10k.BLOCK_GRID_N
    if dry > 0:
        print(f"[run_phase1] dry-run: {grid_n}×{grid_n} = {grid_n * grid_n} blocks")
    ok = g10k.run_phase1_spawn(grid_n=grid_n)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
