#!/usr/bin/env python3
"""Unit tests for perception standoff distance checks."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from costmap_layers import LayeredCostmap  # noqa: E402
from perception_layer import L2_LETHAL_COST  # noqa: E402
from perception_standoff import (  # noqa: E402
    StandoffCheck,
    check_perception_standoff,
    nearest_l2_obstacle_cm,
)
from site_transport_config import FAST_PROFILE, PERCEPTION_STANDOFF_CM  # noqa: E402


def _empty_layers() -> LayeredCostmap:
    costs = np.zeros((40, 40), dtype=np.float32)
    return LayeredCostmap(l0=costs, origin_xy=(0.0, 0.0), resolution_cm=50.0)


def test_fast_profile_standoff_constant() -> None:
    assert PERCEPTION_STANDOFF_CM == 100.0
    assert FAST_PROFILE.perception_standoff_cm == 100.0


def test_nearest_l2_obstacle_cm() -> None:
    layers = _empty_layers()
    layers.l2[10, 10] = L2_LETHAL_COST
    obstacle_xy, dist = nearest_l2_obstacle_cm((525.0, 525.0), layers)
    assert obstacle_xy == (525.0, 525.0)
    assert dist == 0.0


def test_check_perception_standoff_registry() -> None:
    layers = _empty_layers()
    robot_xy = (1000.0, 1000.0)
    near_prop = (1080.0, 1000.0)
    result = check_perception_standoff(
        robot_xy,
        layers,
        registry_positions=[near_prop],
        standoff_cm=100.0,
    )
    assert isinstance(result, StandoffCheck)
    assert result.source == "registry"
    assert result.needs_backoff(100.0)
    assert result.backoff_cm(100.0) > 0.0


def test_check_perception_standoff_clear() -> None:
    layers = _empty_layers()
    result = check_perception_standoff(
        (500.0, 500.0),
        layers,
        registry_positions=[(800.0, 800.0)],
        standoff_cm=100.0,
    )
    assert not result.needs_backoff(100.0)
