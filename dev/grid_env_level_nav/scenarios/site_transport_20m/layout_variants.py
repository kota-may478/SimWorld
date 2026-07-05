#!/usr/bin/env python3
"""Random site_transport_20m layout variants (10 patterns incl. curated layout_01)."""

from __future__ import annotations

import json
import math
import random
import sys
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from paths import REGISTRY_DIR, site_transport_registry_path  # noqa: E402
from region import REGION_SIZE_CM, ROBOT_START_LOCAL_CM, HUMANOID_LOCAL_CM  # noqa: E402
from zones import (  # noqa: E402
    ROADBLOCK_BP_NAME,
    ROADBLOCK_L1_SIDE_CM,
    ROADBLOCKS_PER_SIDE,
    ForbiddenZone,
    forbidden_zones_for_layout,
    sw_cluster_rect_from_points,
    _centered_rect,
)

from placement import (  # noqa: E402
    CARDINAL_YAW_DEG,
    DEFAULT_SEED,
    LayoutEntry,
    TRANSPORT_LOCAL_CM,
    _CURATED_LAYOUT,
    _roadblock_perimeter_layout,
    build_registry_from_layout,
    save_registry,
)

LAYOUT_COUNT = 10
VARIANT_BASE_SEED = 20260629
MARGIN_CM = 80.0
MIN_PROP_SEP_CM = 110.0
MIN_CORRIDOR_CLEAR_CM = MIN_PROP_SEP_CM
ANCHOR_CLEAR_CM = 140.0
PICKUP_EXCLUSION_CM = 95.0

# Slot structure mirrors curated layout (without roadblocks).
_VARIABLE_SLOT_SPECS: Tuple[Tuple[str, str], ...] = (
    ("facilities_sw", "waste_dumpster"),
    ("equipment_sw", "light_tower"),
    ("equipment_sw", "cable_spool"),
    ("facilities_sw", "portable_toilet"),
    ("facilities_sw", "water_tank"),
    ("mid_site", "barrel"),
    ("mid_site", "drywall"),
    ("mid_site", "cinder_blocks"),
    ("mid_site", "cinder_blocks"),
    ("mid_site", "pallet"),
    ("mid_site", "safety_pylon"),
    ("material_yard", "yard_pallet"),
    ("material_yard", "cardboard_boxes"),
    ("material_yard", "brick_pallet"),
    ("material_yard", "bagged_concrete"),
    ("material_yard", "rebar_bundle"),
)

_SW_BP_POOL = (
    "BP_Dumpster",
    "BP_LightGenerator_01a",
    "BP_CableReel",
    "BP_Portapotty_01",
    "BP_Portapotty_02",
    "BP_WaterTank_01a",
    "BP_Tarp_01",
    "BP_Tarp_02",
    "BP_PlasticBarrel_01a",
    "BP_Trafficbarrier_01",
)

_MID_BP_POOL = (
    "BP_Barrel_01",
    "BP_Drywall_01a",
    "BP_Drywall_01b",
    "BP_CinderStack_01a",
    "BP_CinderStack_01b",
    "BP_CinderStack_02a",
    "BP_woodenpalette_01",
    "BP_ConstructionPylons_01a",
    "BP_ConstructionPylons_01d",
    "BP_ConcretePipe_01",
    "BP_Spool_01a",
    "BP_Tarp_03",
)

_YARD_BP_POOL = (
    "BP_woodenpalette_01",
    "BP_Boxes_01a",
    "BP_Boxes_02a",
    "BP_Boxes_03a",
    "BP_BrickPaletteStack_01a",
    "BP_BrickPaletteStack_01b",
    "BP_ConcreteBag_01a",
    "BP_ConcreteBag_02a",
    "BP_Rebar_01a",
    "BP_Rebar_01b",
    "BP_CinderStack_Pallete_01a",
)

_TRANSPORT_BP_POOL = (
    "BP_Crate_01a",
    "BP_Boxes_03a",
    "BP_Boxes_04a",
    "BP_Barrel_01",
    "BP_ConcreteBag_01a",
    "BP_ConcreteBag_03a",
    "BP_BrickPaletteStack_01c",
    "BP_PlasticBarrel_01a",
    "BP_Spool_01a",
)

_PIT_CENTER_X_RANGE = (1050.0, 1580.0)
_PIT_CENTER_Y_RANGE = (720.0, 1280.0)

_SW_XY_RANGE = ((300.0, 820.0), (280.0, 980.0))
_MID_XY_RANGE = ((720.0, 1480.0), (260.0, 1320.0))
# Base yard decor anchors (shuffled + jittered per variant; pickup fixed separately).
_YARD_BASE_POSITIONS: Tuple[Tuple[float, float], ...] = (
    (1720.0, 1780.0),
    (1780.0, 1910.0),
    (1910.0, 1720.0),
    (1680.0, 1880.0),
    (1880.0, 1880.0),
    (1650.0, 1750.0),
    (1860.0, 1780.0),
    (1750.0, 1910.0),
)
_YARD_XY_RANGE = ((1620.0, 1970.0), (1620.0, 1970.0))


def layout_id_for_index(index: int) -> str:
    if not 1 <= index <= LAYOUT_COUNT:
        raise ValueError(f"layout index must be 1..{LAYOUT_COUNT}, got {index}")
    return f"layout_{index:02d}"


def _catalog_names() -> set[str]:
    from prop_catalog import ensure_catalog  # noqa: WPS433

    return {e.bp_name for e in ensure_catalog()}


def _filter_pool(pool: Sequence[str], catalog: set[str]) -> List[str]:
    return [bp for bp in pool if bp in catalog]


def _dist_point_to_segment(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    denom = abx * abx + aby * aby
    if denom < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / denom))
    cx, cy = ax + t * abx, ay + t * aby
    return math.hypot(px - cx, py - cy)


def _point_in_rect(px: float, py: float, rect: Tuple[float, float, float, float]) -> bool:
    x0, y0, x1, y1 = rect
    return min(x0, x1) <= px <= max(x0, x1) and min(y0, y1) <= py <= max(y0, y1)


def _pit_inner_rect(pit_center: Tuple[float, float]) -> Tuple[float, float, float, float]:
    return _centered_rect(pit_center, ROADBLOCK_L1_SIDE_CM)


def _valid_xy(
    x: float,
    y: float,
    placed: Sequence[Tuple[float, float]],
    *,
    pit_inner: Tuple[float, float, float, float],
    sw_rect: Tuple[float, float, float, float] | None,
    mid_rect: Tuple[float, float, float, float] | None,
    yard_rect: Tuple[float, float, float, float] | None,
    region: str,
) -> bool:
    if not (MARGIN_CM <= x <= REGION_SIZE_CM - MARGIN_CM):
        return False
    if not (MARGIN_CM <= y <= REGION_SIZE_CM - MARGIN_CM):
        return False
    if math.hypot(x - ROBOT_START_LOCAL_CM[0], y - ROBOT_START_LOCAL_CM[1]) < ANCHOR_CLEAR_CM:
        return False
    if math.hypot(x - HUMANOID_LOCAL_CM[0], y - HUMANOID_LOCAL_CM[1]) < ANCHOR_CLEAR_CM:
        return False
    if math.hypot(x - TRANSPORT_LOCAL_CM[0], y - TRANSPORT_LOCAL_CM[1]) < PICKUP_EXCLUSION_CM:
        return False
    if _point_in_rect(x, y, pit_inner):
        return False
    if region != "yard":
        corridor_dist = _dist_point_to_segment(
            x,
            y,
            ROBOT_START_LOCAL_CM[0],
            ROBOT_START_LOCAL_CM[1],
            TRANSPORT_LOCAL_CM[0],
            TRANSPORT_LOCAL_CM[1],
        )
        if corridor_dist < MIN_CORRIDOR_CLEAR_CM:
            return False
    for ox, oy in placed:
        if math.hypot(x - ox, y - oy) < MIN_PROP_SEP_CM:
            return False
    if region == "sw" and sw_rect is not None:
        if not _point_in_rect(x, y, sw_rect):
            return False
    if region == "mid" and mid_rect is not None:
        if not _point_in_rect(x, y, mid_rect):
            return False
    if region == "yard" and yard_rect is not None:
        if not _point_in_rect(x, y, yard_rect):
            return False
    return True


def _sample_xy(
    rng: random.Random,
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    placed: List[Tuple[float, float]],
    *,
    pit_inner: Tuple[float, float, float, float],
    region: str,
) -> Tuple[float, float]:
    sw_rect = _SW_XY_RANGE[0][0], _SW_XY_RANGE[1][0], _SW_XY_RANGE[0][1], _SW_XY_RANGE[1][1]
    mid_rect = _MID_XY_RANGE[0][0], _MID_XY_RANGE[1][0], _MID_XY_RANGE[0][1], _MID_XY_RANGE[1][1]
    yard_rect = _YARD_XY_RANGE[0][0], _YARD_XY_RANGE[1][0], _YARD_XY_RANGE[0][1], _YARD_XY_RANGE[1][1]
    for _ in range(400):
        x = rng.uniform(*x_range)
        y = rng.uniform(*y_range)
        if _valid_xy(
            x,
            y,
            placed,
            pit_inner=pit_inner,
            sw_rect=sw_rect,
            mid_rect=mid_rect,
            yard_rect=yard_rect,
            region=region,
        ):
            return (x, y)
    raise RuntimeError(f"failed to place prop in region={region} after 400 attempts")


def _sample_yard_xy(
    rng: random.Random,
    placed: List[Tuple[float, float]],
    *,
    pit_inner: Tuple[float, float, float, float],
    yard_index: int,
) -> Tuple[float, float]:
    bases = list(_YARD_BASE_POSITIONS)
    rng.shuffle(bases)
    base = bases[yard_index % len(bases)]
    for _ in range(40):
        x = min(REGION_SIZE_CM - MARGIN_CM, max(MARGIN_CM, base[0] + rng.uniform(-45.0, 45.0)))
        y = min(REGION_SIZE_CM - MARGIN_CM, max(MARGIN_CM, base[1] + rng.uniform(-45.0, 45.0)))
        if _valid_xy(
            x,
            y,
            placed,
            pit_inner=pit_inner,
            sw_rect=None,
            mid_rect=None,
            yard_rect=None,
            region="yard",
        ):
            return (x, y)
    return base


def _pool_for_cluster(cluster: str, pools: Dict[str, List[str]]) -> List[str]:
    if cluster in {"facilities_sw", "equipment_sw"}:
        return pools["sw"]
    if cluster == "mid_site":
        return pools["mid"]
    if cluster == "material_yard":
        return pools["yard"]
    raise KeyError(cluster)


def generate_random_layout_entries(
    variant_index: int,
    *,
    base_seed: int = VARIANT_BASE_SEED,
) -> Tuple[List[LayoutEntry], Tuple[float, float], Tuple[ForbiddenZone, ...]]:
    """Build prop layout entries for layout_02..layout_10."""
    if variant_index == 1:
        raise ValueError("use curated _CURATED_LAYOUT for layout_01")
    catalog = _catalog_names()
    pools = {
        "sw": _filter_pool(_SW_BP_POOL, catalog),
        "mid": _filter_pool(_MID_BP_POOL, catalog),
        "yard": _filter_pool(_YARD_BP_POOL, catalog),
        "transport": _filter_pool(_TRANSPORT_BP_POOL, catalog),
    }
    for key, items in pools.items():
        if not items:
            raise RuntimeError(f"empty BP pool for {key}")

    rng = random.Random(base_seed + variant_index * 9973)
    pit_cx = rng.uniform(*_PIT_CENTER_X_RANGE)
    pit_cy = rng.uniform(*_PIT_CENTER_Y_RANGE)
    pit_center = (pit_cx, pit_cy)
    pit_inner = _pit_inner_rect(pit_center)
    pit_rect = pit_inner

    placed: List[Tuple[float, float]] = []
    entries: List[LayoutEntry] = []
    used_bps: set[str] = set()

    yard_slot = 0
    for cluster, role in _VARIABLE_SLOT_SPECS:
        pool = _pool_for_cluster(cluster, pools)
        rng.shuffle(pool)
        bp_name = next(bp for bp in pool if bp not in used_bps or len(used_bps) >= len(pool))
        used_bps.add(bp_name)
        if cluster in {"facilities_sw", "equipment_sw"}:
            xy = _sample_xy(rng, *_SW_XY_RANGE, placed, pit_inner=pit_inner, region="sw")
        elif cluster == "mid_site":
            xy = _sample_xy(rng, *_MID_XY_RANGE, placed, pit_inner=pit_inner, region="mid")
        else:
            xy = _sample_yard_xy(rng, placed, pit_inner=pit_inner, yard_index=yard_slot)
            yard_slot += 1
        placed.append(xy)
        yaw = rng.choice(CARDINAL_YAW_DEG)
        entries.append((bp_name, cluster, role, xy, yaw, False))

    transport_pool = list(pools["transport"])
    rng.shuffle(transport_pool)
    transport_bp = transport_pool[(variant_index - 2) % len(transport_pool)]
    entries.append(
        (
            transport_bp,
            "material_yard",
            "shipping_crate",
            TRANSPORT_LOCAL_CM,
            rng.choice(CARDINAL_YAW_DEG),
            True,
        )
    )

    roadblocks = _roadblock_perimeter_layout(pit_rect)
    entries.extend(roadblocks)

    sw_points = [e[3] for e in entries if e[1] in {"facilities_sw", "equipment_sw"}]
    sw_rect = sw_cluster_rect_from_points(sw_points)
    zones = forbidden_zones_for_layout(pit_rect, sw_rect)
    return entries, pit_center, zones


def build_layout_registry(variant_index: int, *, base_seed: int = VARIANT_BASE_SEED):
    layout_id = layout_id_for_index(variant_index)
    if variant_index == 1:
        from zones import FORBIDDEN_ZONES_LAYOUT_01  # noqa: WPS433

        return build_registry_from_layout(
            list(_CURATED_LAYOUT),
            seed=DEFAULT_SEED,
            layout_id=layout_id,
            forbidden_zones=FORBIDDEN_ZONES_LAYOUT_01,
        )

    entries, pit_center, zones = generate_random_layout_entries(
        variant_index, base_seed=base_seed
    )
    seed = base_seed + variant_index
    registry = build_registry_from_layout(
        entries,
        seed=seed,
        layout_id=layout_id,
        forbidden_zones=zones,
    )
    return registry


def layout_summary(registry) -> Dict[str, object]:
    transport = registry.transport_slot()
    sw_props = [p for p in registry.props if p.cluster_id in {"facilities_sw", "equipment_sw"}]
    mid_props = [p for p in registry.props if p.cluster_id == "mid_site"]
    yard_props = [
        p
        for p in registry.props
        if p.cluster_id == "material_yard" and not p.is_transport_target
    ]
    barriers = [p for p in registry.props if p.cluster_id == "no_entry_roadblock"]
    pit_zone = next(z for z in registry.forbidden_zones if z.zone_id == "no_entry_pit")
    return {
        "layout_id": registry.layout_id,
        "seed": registry.seed,
        "transport_bp": transport.bp_name if transport else None,
        "transport_xy": list(registry.material_pickup_local_cm),
        "pit_rect": list(pit_zone.rect_local_cm),
        "roadblock_count": len(barriers),
        "sw_prop_count": len(sw_props),
        "mid_prop_count": len(mid_props),
        "yard_decor_count": len(yard_props),
        "props": [
            {
                "bp": p.bp_name,
                "cluster": p.cluster_id,
                "xy": list(p.local_xy_cm),
                "transport": p.is_transport_target,
            }
            for p in registry.props
        ],
    }


def generate_all_layouts(
    *,
    base_seed: int = VARIANT_BASE_SEED,
    count: int = LAYOUT_COUNT,
) -> Path:
    """Write layout_01..layout_NN JSON registries and a manifest."""
    REGISTRY_DIR.mkdir(parents=True, exist_ok=True)
    manifest: Dict[str, object] = {
        "version": 1,
        "base_seed": base_seed,
        "layout_count": count,
        "layouts": [],
    }
    for index in range(1, count + 1):
        layout_id = layout_id_for_index(index)
        registry = build_layout_registry(index, base_seed=base_seed)
        path = site_transport_registry_path(layout_id)
        save_registry(registry, path)
        summary = layout_summary(registry)
        manifest["layouts"].append(
            {
                "index": index,
                "layout_id": layout_id,
                "registry_path": str(path),
                "transport_bp": summary["transport_bp"],
                "pit_rect": summary["pit_rect"],
            }
        )
        print(f"[LayoutGen] wrote {layout_id} transport={summary['transport_bp']}")

    manifest_path = REGISTRY_DIR / "site_transport_20m_layout_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"[LayoutGen] manifest: {manifest_path}")
    return manifest_path


if __name__ == "__main__":
    generate_all_layouts()
