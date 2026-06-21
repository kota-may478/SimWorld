#!/usr/bin/env python3
"""Unit tests for object_mask_color parsing and detection BGR mapping."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from depth_object_perception import _mask_for_bgr, detect_objects  # noqa: E402
from object_mask_color import parse_unreal_color_response  # noqa: E402
from prop_placement import PlacementRegistry, PropPlacement  # noqa: E402


def test_parse_unreal_color_response_paren():
    raw = "(R=152,G=206,B=66,A=255)"
    assert parse_unreal_color_response(raw) == (152, 206, 66)


def test_parse_unreal_color_response_spaces():
    raw = "152 206 66 255"
    assert parse_unreal_color_response(raw) == (152, 206, 66)


def test_detection_bgr_from_canonical():
    prop = PropPlacement(
        slot_id="depth_test_prop_001",
        catalog_index=0,
        prop_type_id="boxes_03a",
        bp_name="BP_Boxes_03a",
        bp_path="/Game/x.BP_x",
        mask_color_rgb=(115, 153, 195),
        local_xy_cm=(100.0, 200.0),
        mask_color_canonical_rgb=(120, 150, 200),
    )
    assert prop.detection_bgr() == (200, 150, 120)


def test_detection_bgr_prefers_canonical_over_observed():
    prop = PropPlacement(
        slot_id="depth_test_prop_001",
        catalog_index=0,
        prop_type_id="boxes_03a",
        bp_name="BP_Boxes_03a",
        bp_path="/Game/x.BP_x",
        mask_color_rgb=(115, 153, 195),
        local_xy_cm=(100.0, 200.0),
        mask_color_canonical_rgb=(120, 150, 200),
        mask_color_observed_bgr=(99, 88, 77),
    )
    assert prop.detection_bgr() == (200, 150, 120)


def test_mask_match_per_channel_tolerance():
    mask = np.zeros((4, 4, 3), dtype=np.uint8)
    mask[1:3, 1:3] = (66, 206, 152)  # BGR for RGB 152,206,66
    region = _mask_for_bgr(mask, (66, 206, 152), tolerance=8)
    assert region.sum() == 4


def test_detect_objects_synthetic():
    prop = PropPlacement(
        slot_id="depth_test_prop_001",
        catalog_index=0,
        prop_type_id="boxes_03a",
        bp_name="BP_Boxes_03a",
        bp_path="/Game/x.BP_x",
        mask_color_rgb=(152, 206, 66),
        local_xy_cm=(100.0, 200.0),
        mask_color_canonical_rgb=(152, 206, 66),
    )
    registry = PlacementRegistry(
        version=1,
        seed=42,
        prop_count=1,
        region_x_max_cm=3000.0,
        region_y_max_cm=3000.0,
        exclusion_cm=500.0,
        spotdog_spawn_local_cm=(100.0, 100.0),
        props=(prop,),
    )
    h, w = 96, 128
    mask = np.zeros((h, w, 3), dtype=np.uint8)
    mask[30:70, 50:90] = (66, 206, 152)
    depth_m = np.full((h, w), 3.5, dtype=np.float64)
    estimates = detect_objects(mask, depth_m, registry)
    assert len(estimates) == 1
    assert estimates[0].prop_type_id == "boxes_03a"
    assert estimates[0].distance_m > 0.5
