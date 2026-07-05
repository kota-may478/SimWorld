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
    LAYOUT_COUNT,
    build_layout_registry,
    generate_random_layout_entries,
    layout_id_for_index,
)
from placement import (  # noqa: E402
    TRANSPORT_LOCAL_CM,
    build_registry_from_layout,
    quantize_prop_yaw_deg,
    roadblock_yaw_deg_for_role,
)
from zones import forbidden_zones_for_layout, sw_cluster_rect_from_points  # noqa: E402


def test_layout_id_for_index() -> None:
    assert layout_id_for_index(1) == "layout_01"
    assert layout_id_for_index(10) == "layout_10"


def test_generate_ten_distinct_variants() -> None:
    transports = []
    pit_rects = []
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
    assert len(set(transports)) >= 5
    assert len(set(pit_rects)) >= 5


def test_random_entries_respect_pickup_exclusion() -> None:
    entries, _, zones = generate_random_layout_entries(3)
    for bp, cluster, _role, xy, yaw, is_target in entries:
        if is_target:
            assert xy == TRANSPORT_LOCAL_CM
            assert yaw in {0.0, 90.0, 180.0, -90.0}
        else:
            assert cluster != "no_entry_roadblock" or bp == "BP_Roadblock_03b"
            if cluster != "no_entry_roadblock":
                assert yaw in {0.0, 90.0, 180.0, -90.0}
    assert len(zones) == 2


def test_quantize_prop_yaw_snaps_to_cardinals() -> None:
    assert quantize_prop_yaw_deg(20.0) == 0.0
    assert quantize_prop_yaw_deg(45.0) == 90.0
    assert quantize_prop_yaw_deg(-95.0) == -90.0
    assert quantize_prop_yaw_deg(170.0) == 180.0
    assert quantize_prop_yaw_deg(180.0) == 180.0
    assert quantize_prop_yaw_deg(-180.0) == 180.0


def test_roadblock_perimeter_yaws_form_square() -> None:
    entries = __import__("placement", fromlist=["_roadblock_perimeter_layout"])._roadblock_perimeter_layout(
        (1270.0, 910.0, 1630.0, 1270.0)
    )
    by_role: dict[str, float] = {}
    for _bp, _cluster, role, _xy, yaw, _target in entries:
        by_role[role] = yaw
    assert by_role["roadblock_south"] == 0.0
    assert by_role["roadblock_north"] == 180.0
    assert by_role["roadblock_west"] == 90.0
    assert by_role["roadblock_east"] == -90.0
    reg = build_registry_from_layout(
        entries,
        seed=1,
        layout_id="layout_test",
        forbidden_zones=[],
    )
    for prop in reg.props:
        assert prop.yaw_deg == roadblock_yaw_deg_for_role(prop.role)


def test_sw_cluster_rect_from_points() -> None:
    rect = sw_cluster_rect_from_points([(400.0, 450.0), (500.0, 850.0)], padding_cm=50.0)
    assert rect[0] <= 400.0
    assert rect[2] >= 500.0


def test_build_registry_from_layout_requires_catalog() -> None:
    entries = [
        ("BP_Barrel_01", "mid_site", "barrel", (900.0, 700.0), 0.0, False),
        ("BP_Crate_01a", "material_yard", "shipping_crate", TRANSPORT_LOCAL_CM, 0.0, True),
    ]
    entries.extend(
        __import__("placement", fromlist=["_roadblock_perimeter_layout"])._roadblock_perimeter_layout(
            (1270.0, 910.0, 1630.0, 1270.0)
        )
    )
    zones = forbidden_zones_for_layout(
        (1270.0, 910.0, 1630.0, 1270.0),
        sw_cluster_rect_from_points([]),
    )
    reg = build_registry_from_layout(
        entries,
        seed=1,
        layout_id="layout_test",
        forbidden_zones=zones,
    )
    assert reg.material_pickup_local_cm == TRANSPORT_LOCAL_CM
