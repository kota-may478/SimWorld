#!/usr/bin/env python3
"""Unit tests for UnrealCV depth npy unit conversion."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT / "dev" / "grid_env_depth_perception") not in sys.path:
    sys.path.insert(0, str(_ROOT / "dev" / "grid_env_depth_perception"))

from depth_object_perception import depth_npy_to_meters, depth_npy_unit_hint  # noqa: E402


def test_ue_cm_depth_converts_to_meters() -> None:
    depth_cm = np.full((32, 32), 150.0, dtype=np.float32)
    assert depth_npy_unit_hint(depth_cm) == "cm"
    depth_m = depth_npy_to_meters(depth_cm)
    assert abs(float(np.nanmin(depth_m)) - 1.5) < 1e-6


def test_synthetic_meter_depth_unchanged() -> None:
    depth_m_in = np.full((32, 32), 2.0, dtype=np.float32)
    assert depth_npy_unit_hint(depth_m_in) == "m"
    depth_m = depth_npy_to_meters(depth_m_in)
    assert abs(float(np.nanmin(depth_m)) - 2.0) < 1e-6


def test_close_ue_cm_not_misread_as_meters() -> None:
    depth_cm = np.full((32, 32), 85.0, dtype=np.float32)
    depth_m = depth_npy_to_meters(depth_cm)
    assert float(np.nanmin(depth_m)) < 1.0
