#!/usr/bin/env python3
"""Spawn layout_01 .. layout_10 in PIE sequentially (one prop at a time per layout)."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from layout_variants import layout_id_for_index, layout_summary  # noqa: E402
from paths import site_transport_registry_path  # noqa: E402
from placement import ensure_registry  # noqa: E402
from pie_safety import PieSessionLost  # noqa: E402
from spawn_pie import spawn_site_transport_scene  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description="Spawn all site_transport layouts in PIE")
    p.add_argument("--start-index", type=int, default=1, help="First layout index (default 1)")
    p.add_argument("--end-index", type=int, default=10, help="Last layout index (default 10)")
    p.add_argument("--force-respawn", action="store_true", help="Re-spawn props each layout")
    p.add_argument(
        "--pause-between-s",
        type=float,
        default=1.0,
        help="Seconds to wait between layouts (default 1.0)",
    )
    args = p.parse_args()

    if args.start_index < 1 or args.end_index < args.start_index:
        print("FAIL: invalid index range")
        return 1

    results: list[tuple[str, int, str]] = []
    for index in range(args.start_index, args.end_index + 1):
        layout_id = layout_id_for_index(index)
        path = site_transport_registry_path(layout_id)
        if not path.is_file():
            print(f"[AllLayouts] SKIP {layout_id}: missing registry {path}")
            results.append((layout_id, 2, "missing registry"))
            continue

        summary = layout_summary(ensure_registry(layout_id=layout_id))
        print(
            f"\n[AllLayouts] === {layout_id} "
            f"decor={summary['decor_prop_count']} roadblocks={summary['roadblock_count']} "
            f"transport={summary['transport_bp']} ==="
        )
        t0 = time.monotonic()
        try:
            rc, _ = spawn_site_transport_scene(
                layout_id=layout_id,
                force_respawn=args.force_respawn,
            )
        except PieSessionLost as exc:
            print(f"[AllLayouts] ABORT {layout_id}: {exc}")
            results.append((layout_id, 3, str(exc)))
            break

        elapsed = time.monotonic() - t0
        status = "OK" if rc == 0 else f"code {rc}"
        print(f"[AllLayouts] {layout_id} {status} ({elapsed:.1f}s)")
        results.append((layout_id, rc, status))

        if index < args.end_index and args.pause_between_s > 0:
            time.sleep(args.pause_between_s)

    print("\n[AllLayouts] summary:")
    for layout_id, rc, status in results:
        print(f"  {layout_id}: {status} (rc={rc})")

    failed = [r for r in results if r[1] != 0]
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
