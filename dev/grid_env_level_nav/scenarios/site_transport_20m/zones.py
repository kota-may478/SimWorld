#!/usr/bin/env python3
"""L1 forbidden zones for site_transport_20m (inside BP barrier enclosures)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple

from costmap_layers import LayeredCostmap
from l0_nav_mask import COSTMAP_LETHAL_COST
from work_region import local_rect_to_cells

RectLocal = Tuple[float, float, float, float]

ROADBLOCK_BP_NAME = "BP_Roadblock_03b"
ROADBLOCKS_PER_SIDE = 3
# The catalog stores asset paths but not physical dimensions. Keep the width in
# one place so it can be replaced by measured UE bounds when available.
ROADBLOCK_03B_WIDTH_CM = 120.0
ROADBLOCK_L1_SIDE_CM = ROADBLOCK_03B_WIDTH_CM * ROADBLOCKS_PER_SIDE
NO_ENTRY_CENTER_LOCAL_CM = (1450.0, 1090.0)


def _centered_rect(center_xy: Tuple[float, float], side_cm: float) -> RectLocal:
    cx, cy = center_xy
    half = side_cm * 0.5
    return (cx - half, cy - half, cx + half, cy + half)


@dataclass(frozen=True)
class ForbiddenZone:
    zone_id: str
    rect_local_cm: RectLocal
    note: str = ""


# SW prop cluster: dumpster(400,450) lightgenerator(550,600) cablereel(650,400)
#   portapotty(350,750) watertank(500,850).  All are physical obstacles that the
#   robot cannot traverse.  Blocking them in L1 prevents LAST-RESORT replans from
#   routing through the cluster when L2 is temporarily cleared.
SW_CLUSTER_RECT_LOCAL_CM: RectLocal = (300.0, 250.0, 800.0, 950.0)

# Inner work pit enclosed by fence/pylon/barrier props in layout_01.
FORBIDDEN_ZONES_LAYOUT_01: Tuple[ForbiddenZone, ...] = (
    ForbiddenZone(
        zone_id="no_entry_pit",
        rect_local_cm=_centered_rect(NO_ENTRY_CENTER_LOCAL_CM, ROADBLOCK_L1_SIDE_CM),
        note=(
            f"Excavation / machinery zone enclosed by "
            f"{ROADBLOCKS_PER_SIDE * 4} {ROADBLOCK_BP_NAME} actors"
        ),
    ),
    ForbiddenZone(
        zone_id="sw_prop_cluster",
        rect_local_cm=SW_CLUSTER_RECT_LOCAL_CM,
        note=(
            "Physical prop cluster in SW quadrant: dumpster, lightgenerator, "
            "cablereel, portapotty, watertank.  Keeps LAST-RESORT L0+L1 replans "
            "from routing through impassable obstacles."
        ),
    ),
)


def point_in_forbidden_local(
    lx: float,
    ly: float,
    zones: Sequence[ForbiddenZone],
) -> bool:
    for zone in zones:
        x0, y0, x1, y1 = zone.rect_local_cm
        if min(x0, x1) <= lx <= max(x0, x1) and min(y0, y1) <= ly <= max(y0, y1):
            return True
    return False


def sw_cluster_rect_from_points(
    points: Sequence[Tuple[float, float]],
    *,
    padding_cm: float = 80.0,
) -> RectLocal:
    """Axis-aligned bounding rect for SW clutter props (L1 last-resort block)."""
    if not points:
        return SW_CLUSTER_RECT_LOCAL_CM
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (
        min(xs) - padding_cm,
        min(ys) - padding_cm,
        max(xs) + padding_cm,
        max(ys) + padding_cm,
    )


def forbidden_zones_for_layout(
  pit_rect: RectLocal,
  sw_cluster_rect: RectLocal,
) -> Tuple[ForbiddenZone, ...]:
    """Build L1 forbidden zones for a layout variant."""
    return (
        ForbiddenZone(
            zone_id="no_entry_pit",
            rect_local_cm=pit_rect,
            note=(
                f"Excavation / machinery zone enclosed by "
                f"{ROADBLOCKS_PER_SIDE * 4} {ROADBLOCK_BP_NAME} actors"
            ),
        ),
        ForbiddenZone(
            zone_id="sw_prop_cluster",
            rect_local_cm=sw_cluster_rect,
            note="SW quadrant prop cluster (auto bounds from variant placement)",
        ),
    )


def apply_forbidden_zones_l1(
    layers: LayeredCostmap,
    zones: Sequence[ForbiddenZone],
    *,
    lethal_cost: float = COSTMAP_LETHAL_COST,
) -> int:
    """Rasterize forbidden rects onto L1 as lethal cells."""
    count = 0
    res = layers.resolution_cm
    for zone in zones:
        x0, y0, x1, y1 = zone.rect_local_cm
        for gx, gy in local_rect_to_cells(x0, y0, x1, y1, res):
            if 0 <= gx < layers.width_cells and 0 <= gy < layers.height_cells:
                layers.l1[gy, gx] = lethal_cost
                count += 1
    return count
