#!/usr/bin/env python3
"""Structured site_transport_20m layout variants (3×4 m clusters, 12 decor props)."""

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
    ForbiddenZone,
    _centered_rect,
    forbidden_zones_for_layout,
    sw_cluster_rect_from_points,
)

from placement import (  # noqa: E402
    CARDINAL_YAW_DEG,
    LayoutEntry,
    TRANSPORT_LOCAL_CM,
    _roadblock_perimeter_layout,
    build_registry_from_layout,
    save_registry,
)

LAYOUT_COUNT = 10
VARIANT_BASE_SEED = 20260708
GRID_STEP_CM = 100.0
CLUSTER_COUNT = 3
PROPS_PER_CLUSTER = 4
DECOR_PROP_COUNT = CLUSTER_COUNT * PROPS_PER_CLUSTER
# Each cluster occupies a 4 m × 4 m footprint; edges are >= 4 m apart.
CLUSTER_SIDE_CM = 400.0
CLUSTER_HALF_CM = CLUSTER_SIDE_CM * 0.5
INTER_CLUSTER_GAP_CM = 400.0
SITE_EDGE_CM = 0.0
SITE_EDGE_HI_CM = REGION_SIZE_CM
MARGIN_CM = 80.0
# Four props per cluster inset 80 cm from each cluster edge (keeps site margin).
_CLUSTER_PROP_INSET_CM = MARGIN_CM
_CLUSTER_PROP_OFFSET_MAG_CM = CLUSTER_HALF_CM - _CLUSTER_PROP_INSET_CM
CLUSTER_PROP_OFFSETS_CM: Tuple[Tuple[float, float], ...] = (
    (-_CLUSTER_PROP_OFFSET_MAG_CM, -_CLUSTER_PROP_OFFSET_MAG_CM),
    (_CLUSTER_PROP_OFFSET_MAG_CM, -_CLUSTER_PROP_OFFSET_MAG_CM),
    (-_CLUSTER_PROP_OFFSET_MAG_CM, _CLUSTER_PROP_OFFSET_MAG_CM),
    (_CLUSTER_PROP_OFFSET_MAG_CM, _CLUSTER_PROP_OFFSET_MAG_CM),
)
ANCHOR_CLEAR_CM = 140.0
# Conservative axis-aligned footprint radius for decor props (placement only).
DECOR_PROP_HALF_EXTENT_CM = 85.0
MIN_PROP_CENTER_SEP_CM = DECOR_PROP_HALF_EXTENT_CM * 2.0
MIN_INTRA_CLUSTER_SEP_CM = 220.0
PICKUP_EXCLUSION_CM = max(200.0, MIN_PROP_CENTER_SEP_CM)
MIN_CORRIDOR_CLEAR_CM = 100.0

# Reference pit rect (legacy layout_01); all variants now use random pit centers.
LAYOUT_01_PIT_RECT: Tuple[float, float, float, float] = _centered_rect(
    (1450.0, 1090.0), ROADBLOCK_L1_SIDE_CM
)

_SUBCOMPONENT_MARKERS: Tuple[str, ...] = (
    "_Base_",
    "_Lid_",
    "_Lights_",
    "_Brick_",
    "_connectors_",
    "_support_",
    "_Single_",
)


def layout_id_for_index(index: int) -> str:
    if not 1 <= index <= LAYOUT_COUNT:
        raise ValueError(f"layout index must be 1..{LAYOUT_COUNT}, got {index}")
    return f"layout_{index:02d}"


def _catalog_names() -> set[str]:
    from prop_catalog import ensure_catalog  # noqa: WPS433

    return {e.bp_name for e in ensure_catalog()}


def _is_spawnable_decor_bp(bp_name: str) -> bool:
    """Standalone decor props from the 73-object catalog (excl. fence sub-parts)."""
    if bp_name.startswith("BP_Roadblock_"):
        return False
    if bp_name.startswith("BP_Z"):
        return False
    return not any(marker in bp_name for marker in _SUBCOMPONENT_MARKERS)


def _spawnable_bp_pool(catalog: set[str]) -> List[str]:
    return sorted(bp for bp in catalog if _is_spawnable_decor_bp(bp))


def _grid_cell_centers() -> List[Tuple[float, float]]:
    lo = GRID_STEP_CM
    hi = REGION_SIZE_CM - GRID_STEP_CM
    cells: List[Tuple[float, float]] = []
    x = lo
    while x <= hi + 1e-6:
        y = lo
        while y <= hi + 1e-6:
            cells.append((x, y))
            y += GRID_STEP_CM
        x += GRID_STEP_CM
    return cells


def _dist_point_to_rect(
    px: float,
    py: float,
    rect: Tuple[float, float, float, float],
) -> float:
    x0, y0, x1, y1 = rect
    x_min, x_max = min(x0, x1), max(x0, x1)
    y_min, y_max = min(y0, y1), max(y0, y1)
    if x_min <= px <= x_max and y_min <= py <= y_max:
        return 0.0
    dx = max(x_min - px, 0.0, px - x_max)
    dy = max(y_min - py, 0.0, py - y_max)
    return math.hypot(dx, dy)


def _point_clear_of_pit(
    px: float,
    py: float,
    pit_rect: Tuple[float, float, float, float],
    *,
    margin_cm: float,
) -> bool:
    """True when a decor prop centered at (px,py) does not overlap the pit interior."""
    return _dist_point_to_rect(px, py, pit_rect) >= margin_cm


def _props_overlap(
    a: Tuple[float, float],
    b: Tuple[float, float],
    *,
    min_sep_cm: float = MIN_PROP_CENTER_SEP_CM,
) -> bool:
    return math.hypot(a[0] - b[0], a[1] - b[1]) < min_sep_cm


def _point_in_rect(px: float, py: float, rect: Tuple[float, float, float, float]) -> bool:
    x0, y0, x1, y1 = rect
    return min(x0, x1) <= px <= max(x0, x1) and min(y0, y1) <= py <= max(y0, y1)


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


def validate_decor_placement(
    points: Sequence[Tuple[float, float]],
    pit_rect: Tuple[float, float, float, float],
    *,
    cluster_ids: Sequence[int] | None = None,
) -> None:
    """Raise ValueError when decor props overlap or intrude into the pit."""
    for index, xy in enumerate(points):
        if not _point_clear_of_pit(
            xy[0], xy[1], pit_rect, margin_cm=DECOR_PROP_HALF_EXTENT_CM
        ):
            raise ValueError(f"decor {index} at {xy} overlaps pit {pit_rect}")
        for other_index, other in enumerate(points[index + 1 :], index + 1):
            min_sep = MIN_INTRA_CLUSTER_SEP_CM
            if cluster_ids is not None and cluster_ids[index] != cluster_ids[other_index]:
                min_sep = MIN_PROP_CENTER_SEP_CM
            if math.hypot(xy[0] - other[0], xy[1] - other[1]) < min_sep:
                raise ValueError(f"decor overlap: {xy} vs {other}")


def _rects_overlap(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> bool:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    return not (ax1 <= bx0 or bx1 <= ax0 or ay1 <= by0 or by1 <= ay0)


def _cluster_rect(center_xy: Tuple[float, float]) -> Tuple[float, float, float, float]:
    cx, cy = center_xy
    return (
        cx - CLUSTER_HALF_CM,
        cy - CLUSTER_HALF_CM,
        cx + CLUSTER_HALF_CM,
        cy + CLUSTER_HALF_CM,
    )


def _prop_positions_for_cluster(
    center_xy: Tuple[float, float],
) -> List[Tuple[float, float]]:
    cx, cy = center_xy
    return [(cx + dx, cy + dy) for dx, dy in CLUSTER_PROP_OFFSETS_CM]


def _cluster_inside_region(center_xy: Tuple[float, float]) -> bool:
    rect = _cluster_rect(center_xy)
    return (
        rect[0] >= SITE_EDGE_CM - 1e-6
        and rect[1] >= SITE_EDGE_CM - 1e-6
        and rect[2] <= SITE_EDGE_HI_CM + 1e-6
        and rect[3] <= SITE_EDGE_HI_CM + 1e-6
    )


def _single_cluster_anchor_clear(center_xy: Tuple[float, float]) -> bool:
    if not _cluster_inside_region(center_xy):
        return False
    for px, py in _prop_positions_for_cluster(center_xy):
        if not (MARGIN_CM <= px <= REGION_SIZE_CM - MARGIN_CM):
            return False
        if not _cluster_anchor_clear(px, py):
            return False
    return True


def _cluster_layout_usable(cluster_centers: Sequence[Tuple[float, float]]) -> bool:
    if len(cluster_centers) != CLUSTER_COUNT:
        return False
    if len(set(cluster_centers)) != CLUSTER_COUNT:
        return False
    try:
        validate_cluster_layout(cluster_centers)
    except ValueError:
        return False
    for center in cluster_centers:
        if not _single_cluster_anchor_clear(center):
            return False
    return True


def _cluster_center_candidates_no_pit() -> List[Tuple[float, float]]:
    lo = CLUSTER_HALF_CM
    hi = REGION_SIZE_CM - CLUSTER_HALF_CM
    candidates: List[Tuple[float, float]] = []
    cx = lo
    while cx <= hi + 1e-6:
        cy = lo
        while cy <= hi + 1e-6:
            if _single_cluster_anchor_clear((cx, cy)):
                candidates.append((cx, cy))
            cy += GRID_STEP_CM
        cx += GRID_STEP_CM
    return candidates


def _sample_cluster_centers(rng: random.Random) -> List[Tuple[float, float]]:
    candidates = _cluster_center_candidates_no_pit()
    if len(candidates) < CLUSTER_COUNT:
        raise RuntimeError(
            f"only {len(candidates)} single-cluster centers; need {CLUSTER_COUNT}"
        )
    shuffled = list(candidates)
    rng.shuffle(shuffled)
    for i, first in enumerate(shuffled):
        for j in range(i + 1, len(shuffled)):
            for k in range(j + 1, len(shuffled)):
                trial = [first, shuffled[j], shuffled[k]]
                if _cluster_layout_usable(trial):
                    return trial
    for _ in range(8000):
        trial = rng.sample(candidates, CLUSTER_COUNT)
        if _cluster_layout_usable(trial):
            return trial
    raise RuntimeError("no usable random cluster layout with 4 m inter-area gaps")


def inter_cluster_edge_gap_cm(
    center_a: Tuple[float, float],
    center_b: Tuple[float, float],
) -> float:
    """Minimum edge-to-edge gap between two axis-aligned 4 m cluster footprints."""
    ax0, ay0, ax1, ay1 = _cluster_rect(center_a)
    bx0, by0, bx1, by1 = _cluster_rect(center_b)
    gap_x = 0.0
    if ax1 < bx0:
        gap_x = bx0 - ax1
    elif bx1 < ax0:
        gap_x = ax0 - bx1
    gap_y = 0.0
    if ay1 < by0:
        gap_y = by0 - ay1
    elif by1 < ay0:
        gap_y = ay0 - by1
    if gap_x > 0.0 and gap_y > 0.0:
        return math.hypot(gap_x, gap_y)
    return max(gap_x, gap_y)


def validate_cluster_layout(
    cluster_centers: Sequence[Tuple[float, float]],
    *,
    min_inter_cluster_gap_cm: float = INTER_CLUSTER_GAP_CM,
) -> None:
    if len(cluster_centers) != CLUSTER_COUNT:
        raise ValueError(f"expected {CLUSTER_COUNT} clusters, got {len(cluster_centers)}")
    for i, center_a in enumerate(cluster_centers):
        for center_b in cluster_centers[i + 1 :]:
            gap = inter_cluster_edge_gap_cm(center_a, center_b)
            if gap + 1e-6 < min_inter_cluster_gap_cm:
                raise ValueError(
                    f"cluster gap {gap:.1f}cm < {min_inter_cluster_gap_cm:.1f}cm "
                    f"between {center_a} and {center_b}"
                )


def _cluster_anchor_clear(px: float, py: float) -> bool:
    if math.hypot(px - ROBOT_START_LOCAL_CM[0], py - ROBOT_START_LOCAL_CM[1]) < ANCHOR_CLEAR_CM:
        return False
    if math.hypot(px - HUMANOID_LOCAL_CM[0], py - HUMANOID_LOCAL_CM[1]) < ANCHOR_CLEAR_CM:
        return False
    if math.hypot(px - TRANSPORT_LOCAL_CM[0], py - TRANSPORT_LOCAL_CM[1]) < PICKUP_EXCLUSION_CM:
        return False
    return True


def _sample_pit_center_for_clusters(
    rng: random.Random,
    cluster_centers: Sequence[Tuple[float, float]],
) -> Tuple[float, float]:
    candidates = [
        (cx, cy)
        for cx, cy in _pit_center_candidates()
        if _pit_center_usable_against_clusters(cx, cy, cluster_centers)
    ]
    if not candidates:
        raise RuntimeError("no usable pit centers clear of decor clusters")
    return rng.choice(candidates)


def _assign_props_to_clusters(
    rng: random.Random,
    cluster_centers: Sequence[Tuple[float, float]],
    decor_bps: Sequence[str],
) -> List[Tuple[str, Tuple[float, float], int]]:
    if len(decor_bps) != DECOR_PROP_COUNT:
        raise ValueError(f"expected {DECOR_PROP_COUNT} decor BPs, got {len(decor_bps)}")
    shuffled = list(decor_bps)
    rng.shuffle(shuffled)
    placements: List[Tuple[str, Tuple[float, float], int]] = []
    prop_index = 0
    for cluster_id, center in enumerate(cluster_centers):
        area_bps = shuffled[cluster_id * PROPS_PER_CLUSTER : (cluster_id + 1) * PROPS_PER_CLUSTER]
        for bp_name, (px, py) in zip(area_bps, _prop_positions_for_cluster(center)):
            placements.append((bp_name, (px, py), cluster_id))
            prop_index += 1
    assert prop_index == DECOR_PROP_COUNT
    return placements


def _pit_rect_from_center(center_xy: Tuple[float, float]) -> Tuple[float, float, float, float]:
    return _centered_rect(center_xy, ROADBLOCK_L1_SIDE_CM)


def _pit_center_candidates() -> List[Tuple[float, float]]:
    """1 m-grid centers where the 12-roadblock square fits inside the site."""
    half = ROADBLOCK_L1_SIDE_CM * 0.5
    lo = half + MARGIN_CM
    hi = REGION_SIZE_CM - MARGIN_CM - half
    centers: List[Tuple[float, float]] = []
    cx = math.ceil(lo / GRID_STEP_CM) * GRID_STEP_CM
    while cx <= hi + 1e-6:
        cy = math.ceil(lo / GRID_STEP_CM) * GRID_STEP_CM
        while cy <= hi + 1e-6:
            if _pit_center_usable(cx, cy):
                centers.append((cx, cy))
            cy += GRID_STEP_CM
        cx += GRID_STEP_CM
    return centers


def _pit_center_usable(cx: float, cy: float) -> bool:
    rect = _pit_rect_from_center((cx, cy))
    for ax, ay in (ROBOT_START_LOCAL_CM, HUMANOID_LOCAL_CM, TRANSPORT_LOCAL_CM):
        if _point_in_rect(ax, ay, rect):
            return False
        if math.hypot(cx - ax, cy - ay) < ANCHOR_CLEAR_CM:
            return False
    return True


def _pit_center_usable_against_clusters(
    cx: float,
    cy: float,
    cluster_centers: Sequence[Tuple[float, float]],
) -> bool:
    if not _pit_center_usable(cx, cy):
        return False
    pit_rect = _pit_rect_from_center((cx, cy))
    for center in cluster_centers:
        if _rects_overlap(_cluster_rect(center), pit_rect):
            return False
    return True


def _sample_pit_center(rng: random.Random) -> Tuple[float, float]:
    candidates = _pit_center_candidates()
    if not candidates:
        raise RuntimeError("no usable pit centers on 1 m grid")
    return rng.choice(candidates)


def _sample_unique_bps(rng: random.Random, pool: Sequence[str], count: int) -> List[str]:
    if len(pool) < count:
        raise RuntimeError(f"BP pool size {len(pool)} < required {count}")
    return rng.sample(list(pool), count)


def generate_random_layout_entries(
    variant_index: int,
    *,
    base_seed: int = VARIANT_BASE_SEED,
) -> Tuple[List[LayoutEntry], Tuple[float, float], Tuple[ForbiddenZone, ...]]:
    """Build clustered prop layout entries for layout_01 .. layout_10."""

    catalog = _catalog_names()
    pool = _spawnable_bp_pool(catalog)
    if len(pool) < DECOR_PROP_COUNT + 1:
        raise RuntimeError(f"spawnable BP pool too small: {len(pool)}")

    rng = random.Random(base_seed + variant_index * 9973)
    cluster_centers = _sample_cluster_centers(rng)
    validate_cluster_layout(cluster_centers)
    pit_center = _sample_pit_center_for_clusters(rng, cluster_centers)
    pit_rect = _pit_rect_from_center(pit_center)

    decor_bps = _sample_unique_bps(rng, pool, DECOR_PROP_COUNT)
    placements = _assign_props_to_clusters(rng, cluster_centers, decor_bps)
    decor_xy = [xy for _bp, xy, _cluster_id in placements]
    cluster_ids = [cluster_id for _bp, _xy, cluster_id in placements]
    validate_decor_placement(decor_xy, pit_rect, cluster_ids=cluster_ids)

    entries: List[LayoutEntry] = []
    for index, (bp_name, xy, _cluster_id) in enumerate(placements):
        entries.append(
            (
                bp_name,
                "site_grid",
                f"prop_{index:02d}",
                xy,
                rng.choice(CARDINAL_YAW_DEG),
                False,
            )
        )

    transport_candidates = [bp for bp in pool if bp not in decor_bps]
    if not transport_candidates:
        transport_candidates = list(pool)
    transport_bp = rng.choice(transport_candidates)
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

    entries.extend(_roadblock_perimeter_layout(pit_rect))

    sw_points = [
        xy
        for xy in decor_xy
        if xy[0] <= REGION_SIZE_CM * 0.5 and xy[1] <= REGION_SIZE_CM * 0.5
    ]
    sw_rect = sw_cluster_rect_from_points(sw_points)
    zones = forbidden_zones_for_layout(pit_rect, sw_rect)
    return entries, pit_center, zones


def build_layout_registry(variant_index: int, *, base_seed: int = VARIANT_BASE_SEED):
    layout_id = layout_id_for_index(variant_index)
    entries, _pit_center, zones = generate_random_layout_entries(
        variant_index, base_seed=base_seed
    )
    seed = base_seed + variant_index
    return build_registry_from_layout(
        entries,
        seed=seed,
        layout_id=layout_id,
        forbidden_zones=zones,
    )


def layout_summary(registry) -> Dict[str, object]:
    transport = registry.transport_slot()
    decor_props = [
        p
        for p in registry.props
        if p.cluster_id not in {"no_entry_roadblock", "material_yard"}
        or (p.cluster_id == "material_yard" and not p.is_transport_target)
    ]
    decor_props = [p for p in registry.props if p.cluster_id == "site_grid"]
    barriers = [p for p in registry.props if p.cluster_id == "no_entry_roadblock"]
    pit_zone = next(z for z in registry.forbidden_zones if z.zone_id == "no_entry_pit")
    return {
        "layout_id": registry.layout_id,
        "seed": registry.seed,
        "transport_bp": transport.bp_name if transport else None,
        "transport_xy": list(registry.material_pickup_local_cm),
        "pit_rect": list(pit_zone.rect_local_cm),
        "roadblock_count": len(barriers),
        "decor_prop_count": len(decor_props),
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
    start_index: int = 1,
) -> Path:
    """Write layout JSON registries and update the manifest."""
    if not 1 <= start_index <= count:
        raise ValueError(f"start_index must be 1..{count}, got {start_index}")

    REGISTRY_DIR.mkdir(parents=True, exist_ok=True)
    manifest_path = REGISTRY_DIR / "site_transport_20m_layout_manifest.json"
    existing_manifest: Dict[str, object] = {}
    if manifest_path.is_file() and start_index > 1:
        existing_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    layouts_meta: List[Dict[str, object]] = []
    if start_index > 1 and existing_manifest.get("layouts"):
        for row in existing_manifest["layouts"]:  # type: ignore[union-attr]
            if int(row["index"]) < start_index:  # type: ignore[index]
                layouts_meta.append(row)

    for index in range(start_index, count + 1):
        layout_id = layout_id_for_index(index)
        registry = build_layout_registry(index, base_seed=base_seed)
        path = site_transport_registry_path(layout_id)
        save_registry(registry, path)
        summary = layout_summary(registry)
        layouts_meta.append(
            {
                "index": index,
                "layout_id": layout_id,
                "registry_path": str(path),
                "transport_bp": summary["transport_bp"],
                "pit_rect": summary["pit_rect"],
            }
        )
        pit_rect = summary["pit_rect"]
        pit_cx = (float(pit_rect[0]) + float(pit_rect[2])) * 0.5
        pit_cy = (float(pit_rect[1]) + float(pit_rect[3])) * 0.5
        print(
            f"[LayoutGen] wrote {layout_id} "
            f"transport={summary['transport_bp']} decor={summary['decor_prop_count']} "
            f"pit_center=({pit_cx:.0f},{pit_cy:.0f})"
        )

    manifest: Dict[str, object] = {
        "version": 1,
        "base_seed": base_seed,
        "layout_count": count,
        "layouts": layouts_meta,
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"[LayoutGen] manifest: {manifest_path}")
    return manifest_path


if __name__ == "__main__":
    generate_all_layouts(start_index=1)
