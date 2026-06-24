#!/usr/bin/env python3
"""Unit tests for forward depth cone helper."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_PKG = Path(__file__).resolve().parent
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from perception_layer import min_forward_depth_m  # noqa: E402


def test_min_forward_depth_m_center_obstacle() -> None:
    depth = np.full((64, 64), 3.0, dtype=np.float32)
    depth[40:50, 28:36] = 0.7
    assert min_forward_depth_m(depth, fov_deg=90.0) == pytest.approx(0.7)
