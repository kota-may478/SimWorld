#!/usr/bin/env python3
"""CLI: generate all site_transport_20m layout variants."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from layout_variants import (  # noqa: E402
    LAYOUT_COUNT,
    VARIANT_BASE_SEED,
    generate_all_layouts,
)


def main() -> int:
    p = argparse.ArgumentParser(description="Generate site_transport_20m layout variants")
    p.add_argument("--count", type=int, default=LAYOUT_COUNT)
    p.add_argument("--start-index", type=int, default=1)
    p.add_argument("--base-seed", type=int, default=VARIANT_BASE_SEED)
    args = p.parse_args()
    generate_all_layouts(
        base_seed=args.base_seed,
        count=args.count,
        start_index=args.start_index,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
