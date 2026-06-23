#!/usr/bin/env python3
"""Unit tests for site transport nav profiles and timing summary."""

from __future__ import annotations

import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from metrics import NavTimingAccumulator, build_timing_summary  # noqa: E402
from site_transport_config import FAST_PROFILE, apply_profile_to_layered_nav, resolve_profile  # noqa: E402
import layered_nav as ln  # noqa: E402


def test_resolve_fast_profile() -> None:
    profile = resolve_profile("fast")
    assert profile.name == "fast"
    assert profile.perception_interval_s == 6.5
    assert profile.depth_stride_px == 16
    assert profile.standoff_backoff_max_cm == 110.0
    assert profile.l2_replan_cell_delta_threshold == 10


def test_apply_fast_profile_updates_layered_nav() -> None:
    apply_profile_to_layered_nav(FAST_PROFILE)
    assert ln.MOVES_PER_CYCLE == 4
    assert ln.SITE_ROBOT_SPEED == 290.0
    assert ln.STANDOFF_BACKOFF_MAX_CM == 110.0
    assert ln.STANDOFF_BACKOFF_SPEED == 180.0
    assert ln.L2_REPLAN_CELL_DELTA_THRESHOLD == 10
    assert ln.PERCEPTION_STANDOFF_CM == 100.0
    apply_profile_to_layered_nav(resolve_profile("default"))
    assert ln.MOVES_PER_CYCLE == 2
    assert ln.PERCEPTION_STANDOFF_CM == 50.0


def test_build_timing_summary() -> None:
    leg1 = NavTimingAccumulator(label="leg1", move_ms=1000.0, perceive_ms=200.0)
    leg2 = NavTimingAccumulator(label="leg2", move_ms=800.0, replan_ms=50.0)
    summary = build_timing_summary(
        legs=[leg1, leg2],
        leg1_time_s=120.5,
        leg2_time_s=95.2,
        profile="fast",
    )
    assert summary["profile"] == "fast"
    assert summary["leg1_time_s"] == 120.5
    assert summary["totals"]["move_ms"] == 1800.0
    assert summary["per_leg"][0]["wall_time_s"] == 120.5
