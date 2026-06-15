#!/usr/bin/env python3
"""Visualize L0 / merged layered costmap."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib.pyplot as plt
import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
_MT = _THIS_DIR.parent / "llm_material_transport"
for _p in (_THIS_DIR, _MT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from costmap_layers import LayeredCostmap  # noqa: E402
from path_planning_costmap import apply_fixed_costmap_axes  # noqa: E402
from zone_registry import ZoneRegistry  # noqa: E402


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--l0", type=Path, required=True)
    p.add_argument("--zones", type=Path, default=None)
    p.add_argument("--close-zone", action="append", default=[])
    p.add_argument("--output", type=Path, default=None)
    p.add_argument("--no-show", action="store_true")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    layers = LayeredCostmap.from_l0_cache(args.l0)
    if args.zones:
        registry = ZoneRegistry.load(args.zones)
        for zid in args.close_zone:
            layers.close_zone(zid, registry)

    costmap = layers.to_costmap2d()
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    titles = ("L0", "L1", "Merged")
    arrays = (layers.l0, layers.l1, layers.merged_costs())
    for ax, title, arr in zip(axes, titles, arrays):
        lethal = float(costmap.lethal_cost)
        display = np.where(arr >= lethal * 0.5, lethal, arr)
        im = ax.imshow(
            display.T,
            origin="lower",
            extent=costmap.plot_extent_y_horizontal(),
            cmap="RdYlGn_r",
            vmin=0,
            vmax=max(10.0, lethal),
        )
        ax.set_title(title)
        apply_fixed_costmap_axes(ax, costmap)
        fig.colorbar(im, ax=ax, fraction=0.046)
    fig.suptitle(str(args.l0))
    plt.tight_layout()
    if args.output:
        fig.savefig(args.output, dpi=150)
        print(f"[viz] saved {args.output}")
    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
