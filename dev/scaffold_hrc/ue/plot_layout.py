#!/usr/bin/env python3
"""Plan and elevation of the Stage-1 scaffold (paper Fig. 2, no UE)."""

from __future__ import annotations

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from paths import make_run_dir
from scene.geometry import STAGE1_GEOM
from scene.scaffold_grammar import build_scaffold
from ue.layout import ENTRANCE_LOCAL_XY_CM, STAGING_LOCAL_XY_CM, footprint_local_cm
from ue.placement import DROP_XY_M, REFUGE_XY_M


def write_layout_png(path: Path) -> Path:
    geom = STAGE1_GEOM
    spec = build_scaffold(geom)
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2))
    plan, elev = axes

    x0, y0, x1, y1 = geom.deck_xy_bounds()
    sx0, sx1, sy0, sy1 = geom.stair_xy_bounds()
    plan.add_patch(
        Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, linewidth=1.4, label="deck")
    )
    plan.add_patch(
        Rectangle((sx0, sy0), sx1 - sx0, sy1 - sy0, fill=False, linewidth=1.0, linestyle="--", label="stair bay")
    )
    store_x, store_y = geom.storage_xy()
    plan.scatter([store_x], [store_y], marker="s", s=40, label="storage")
    plan.scatter([DROP_XY_M[0]], [DROP_XY_M[1]], marker="o", s=30, label="drop")
    plan.scatter([REFUGE_XY_M[0]], [REFUGE_XY_M[1]], marker="^", s=40, label="refuge")
    plan.set_aspect("equal")
    plan.set_xlabel("x (m)")
    plan.set_ylabel("y (m)")
    plan.set_title("plan")
    plan.legend(loc="upper right", fontsize=7)

    for floor in range(1, geom.n_floors + 1):
        z = geom.floor_z_m(floor)
        elev.plot([0.0, geom.deck_length_m], [z, z], color="0.2", linewidth=1.6)
        elev.text(geom.deck_length_m + 0.2, z, f"{floor}F", va="center", fontsize=8)
    for m in spec.modules:
        if m.kind == "post":
            plan.plot(m.x_m, m.y_m, "k.", markersize=5)
    treads = [m for m in spec.modules if m.kind == "stair_tread"]
    elev.scatter([m.x_m for m in treads], [m.z_m for m in treads], s=8, color="0.4")
    elev.set_xlabel("x (m)")
    elev.set_ylabel("z (m)")
    elev.set_title("elevation")
    elev.set_aspect("equal")

    fig.suptitle(
        f"Stage-1 scaffold  {geom.deck_length_m:.1f}x{geom.deck_width_m:.1f} m, "
        f"3F, entrance local {ENTRANCE_LOCAL_XY_CM} cm, yard {STAGING_LOCAL_XY_CM} cm",
        fontsize=10,
    )
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=140)
    plt.close(fig)
    _ = footprint_local_cm()
    return path


def main() -> int:
    run_dir = make_run_dir()
    out = write_layout_png(run_dir / "fig2_layout.png")
    print("wrote", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
