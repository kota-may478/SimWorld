#!/usr/bin/env python3
"""Tests for site_transport_20m layout variant generation."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from layout_variants import (  # noqa: E402
    CLUSTER_COUNT,
    DECOR_PROP_COUNT,
    INTER_CLUSTER_GAP_CM,
    LAYOUT_01_PIT_RECT,
    LAYOUT_COUNT,
    PROPS_PER_CLUSTER,
    _spawnable_bp_pool,
    build_layout_registry,
    generate_random_layout_entries,
    inter_cluster_edge_gap_cm,
    layout_id_for_index,
    validate_cluster_layout,
    validate_decor_placement,
)
from zones import ROADBLOCK_L1_SIDE_CM  # noqa: E402
from placement import (  # noqa: E402
    TRANSPORT_LOCAL_CM,
    build_registry_from_layout,
    quantize_prop_yaw_deg,
    roadblock_yaw_deg_for_role,
)
from zones import FORBIDDEN_ZONES_LAYOUT_01, forbidden_zones_for_layout, sw_cluster_rect_from_points  # noqa: E402


def _cluster_centers_from_decor_props(
    decor: list,
) -> list[tuple[float, float]]:
    ordered = sorted(decor, key=lambda prop: prop.role)
    centers: list[tuple[float, float]] = []
    for cluster_index in range(CLUSTER_COUNT):
        group = ordered[
            cluster_index * PROPS_PER_CLUSTER : (cluster_index + 1) * PROPS_PER_CLUSTER
        ]
        assert len(group) == PROPS_PER_CLUSTER
        centers.append(
            (
                sum(prop.local_xy_cm[0] for prop in group) / PROPS_PER_CLUSTER,
                sum(prop.local_xy_cm[1] for prop in group) / PROPS_PER_CLUSTER,
            )
        )
    return centers


def test_layout_id_for_index() -> None:
    assert layout_id_for_index(1) == "layout_01"
    assert layout_id_for_index(10) == "layout_10"


def test_spawnable_pool_large_enough() -> None:
    from layout_variants import _catalog_names  # noqa: WPS433

    pool = _spawnable_bp_pool(_catalog_names())
    assert len(pool) >= DECOR_PROP_COUNT + 1
    assert all(not bp.startswith("BP_Roadblock_") for bp in pool)


def test_generate_ten_distinct_variants() -> None:
    transports = []
    pit_rects = []
    cluster_center_sets: list[tuple[tuple[float, float], ...]] = []
    for index in range(1, LAYOUT_COUNT + 1):
        reg = build_layout_registry(index)
        assert reg.layout_id == layout_id_for_index(index)
        transport = reg.transport_slot()
        assert transport is not None
        assert transport.is_transport_target
        assert transport.local_xy_cm == TRANSPORT_LOCAL_CM
        targets = [p for p in reg.props if p.is_transport_target]
        assert len(targets) == 1
        roadblocks = [p for p in reg.props if p.cluster_id == "no_entry_roadblock"]
        assert len(roadblocks) == 12
        assert all(p.bp_name == "BP_Roadblock_03b" for p in roadblocks)
        transports.append(transport.bp_name)
        pit = next(z for z in reg.forbidden_zones if z.zone_id == "no_entry_pit")
        pit_rects.append(tuple(pit.rect_local_cm))
        decor = [p for p in reg.props if p.cluster_id == "site_grid"]
        assert len(decor) == DECOR_PROP_COUNT
        assert len({p.bp_name for p in decor}) == DECOR_PROP_COUNT
        ordered_decor = sorted(decor, key=lambda prop: prop.role)
        decor_xy = [p.local_xy_cm for p in ordered_decor]
        cluster_centers = _cluster_centers_from_decor_props(ordered_decor)
        cluster_ids = [index // PROPS_PER_CLUSTER for index in range(DECOR_PROP_COUNT)]
        validate_decor_placement(
            decor_xy,
            pit.rect_local_cm,
            cluster_ids=cluster_ids,
        )
        validate_cluster_layout(cluster_centers)
        cluster_center_sets.append(tuple(cluster_centers))
        for prop in decor:
            assert prop.yaw_deg in {0.0, 90.0, 180.0, -90.0}
    assert len(set(transports)) >= 5
    assert len(set(pit_rects)) >= 2
    assert len(set(cluster_center_sets)) >= 3
    for index in range(2, LAYOUT_COUNT + 1):
        pit = pit_rects[index - 1]
        cx = (pit[0] + pit[2]) * 0.5
        cy = (pit[1] + pit[3]) * 0.5
        assert abs((pit[2] - pit[0]) - ROADBLOCK_L1_SIDE_CM) < 1e-6
        assert abs((pit[3] - pit[1]) - ROADBLOCK_L1_SIDE_CM) < 1e-6
        _ = cx, cy


def test_random_entries_respect_pickup_exclusion() -> None:
    entries, _, zones = generate_random_layout_entries(3)
    pit = next(z for z in zones if z.zone_id == "no_entry_pit")
    decor = sorted(
        [
            entry
            for entry in entries
            if entry[1] == "site_grid"
        ],
        key=lambda entry: entry[2],
    )
    decor_xy = [entry[3] for entry in decor]
    assert len(decor_xy) == DECOR_PROP_COUNT
    cluster_centers = [
        (
            sum(entry[3][0] for entry in decor[i * PROPS_PER_CLUSTER : (i + 1) * PROPS_PER_CLUSTER])
            / PROPS_PER_CLUSTER,
            sum(entry[3][1] for entry in decor[i * PROPS_PER_CLUSTER : (i + 1) * PROPS_PER_CLUSTER])
            / PROPS_PER_CLUSTER,
        )
        for i in range(CLUSTER_COUNT)
    ]
    validate_cluster_layout(cluster_centers)
    validate_decor_placement(decor_xy, pit.rect_local_cm)
    for bp, cluster, _role, xy, yaw, is_target in entries:
        if is_target:
            assert xy == TRANSPORT_LOCAL_CM
            assert yaw in {0.0, 90.0, 180.0, -90.0}
        else:
            assert cluster != "no_entry_roadblock" or bp == "BP_Roadblock_03b"
            if cluster == "site_grid":
                assert yaw in {0.0, 90.0, 180.0, -90.0}
    assert len(zones) == 2


def test_cluster_gap_at_least_four_meters() -> None:
    entries, _, _zones = generate_random_layout_entries(1)
    decor = sorted(
        [entry for entry in entries if entry[1] == "site_grid"],
        key=lambda entry: entry[2],
    )
    centers = [
        (
            sum(entry[3][0] for entry in decor[i * PROPS_PER_CLUSTER : (i + 1) * PROPS_PER_CLUSTER])
            / PROPS_PER_CLUSTER,
            sum(entry[3][1] for entry in decor[i * PROPS_PER_CLUSTER : (i + 1) * PROPS_PER_CLUSTER])
            / PROPS_PER_CLUSTER,
        )
        for i in range(CLUSTER_COUNT)
    ]
    for i, center_a in enumerate(centers):
        for center_b in centers[i + 1 :]:
            assert inter_cluster_edge_gap_cm(center_a, center_b) + 1e-6 >= INTER_CLUSTER_GAP_CM


def test_quantize_prop_yaw_snaps_to_cardinals() -> None:
    assert quantize_prop_yaw_deg(20.0) == 0.0
    assert quantize_prop_yaw_deg(50.0) == 90.0
    assert quantize_prop_yaw_deg(-95.0) == -90.0
    assert quantize_prop_yaw_deg(170.0) == 180.0
    assert quantize_prop_yaw_deg(180.0) == 180.0
    assert quantize_prop_yaw_deg(-180.0) == 180.0


def test_roadblock_perimeter_yaws_form_square() -> None:
    entries = __import__("placement", fromlist=["_roadblock_perimeter_layout"])._roadblock_perimeter_layout(
        LAYOUT_01_PIT_RECT
    )
    by_role: dict[str, float] = {}
    for _bp, _cluster, role, _xy, yaw, _target in entries:
        by_role[role] = yaw
    assert by_role["roadblock_south"] == 0.0
    assert by_role["roadblock_north"] == 180.0
    assert by_role["roadblock_west"] == 90.0
    assert by_role["roadblock_east"] == -90.0
    entries = [
        ("BP_Barrel_01", "site_grid", "prop_00", (900.0, 700.0), 0.0, False),
        ("BP_Crate_01a", "material_yard", "shipping_crate", TRANSPORT_LOCAL_CM, 0.0, True),
    ]
    entries.extend(
        __import__("placement", fromlist=["_roadblock_perimeter_layout"])._roadblock_perimeter_layout(
            LAYOUT_01_PIT_RECT
        )
    )
    reg = build_registry_from_layout(
        entries,
        seed=1,
        layout_id="layout_test",
        forbidden_zones=[],
    )
    for prop in reg.props:
        if prop.cluster_id != "no_entry_roadblock":
            continue
        assert prop.yaw_deg == roadblock_yaw_deg_for_role(prop.role)


def test_sw_cluster_rect_from_points() -> None:
    rect = sw_cluster_rect_from_points([(400.0, 450.0), (500.0, 850.0)], padding_cm=50.0)
    assert rect[0] <= 400.0
    assert rect[2] >= 500.0


def test_build_registry_from_layout_requires_catalog() -> None:
    entries = [
        ("BP_Barrel_01", "site_grid", "prop_00", (900.0, 700.0), 0.0, False),
        ("BP_Crate_01a", "material_yard", "shipping_crate", TRANSPORT_LOCAL_CM, 0.0, True),
    ]
    entries.extend(
        __import__("placement", fromlist=["_roadblock_perimeter_layout"])._roadblock_perimeter_layout(
            LAYOUT_01_PIT_RECT
        )
    )
    zones = forbidden_zones_for_layout(
        LAYOUT_01_PIT_RECT,
        sw_cluster_rect_from_points([]),
    )
    reg = build_registry_from_layout(
        entries,
        seed=1,
        layout_id="layout_test",
        forbidden_zones=zones,
    )
    assert reg.material_pickup_local_cm == TRANSPORT_LOCAL_CM
