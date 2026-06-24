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
    assert profile.perception_interval_s == 5.5
    assert profile.depth_stride_px == 12
    assert profile.standoff_backoff_max_cm == 100.0
    assert profile.l2_replan_cell_delta_threshold == 10


def test_apply_fast_profile_updates_layered_nav() -> None:
    apply_profile_to_layered_nav(FAST_PROFILE)
    assert ln.MOVES_PER_CYCLE == 3
    assert ln.SITE_ROBOT_SPEED == 285.0
    assert ln.STANDOFF_BACKOFF_MAX_CM == 100.0
    assert ln.STANDOFF_BACKOFF_SPEED == 140.0
    assert ln.SITE_PLANNING_CLEARANCE_CM == 100.0
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
    assert "accounted_ms" in summary["totals"]
    assert "accounting_note" in summary
    assert summary["per_leg"][0]["wall_time_s"] == 120.5


def test_timing_breakdown_rows() -> None:
    from metrics import timing_breakdown_rows  # noqa: WPS433

    metrics = {
        "success": True,
        "total_time_s": 584.81,
        "leg1_time_s": 300.0,
        "leg2_time_s": 280.0,
        "violations": {"tracked_motion_time_s": 580.0},
        "timing_summary": {
            "profile": "fast",
            "nav_wall_time_s": 580.0,
            "leg1_time_s": 300.0,
            "leg2_time_s": 280.0,
            "totals": {
                "move_ms": 120000.0,
                "perceive_ms": 45000.0,
                "replan_ms": 8000.0,
                "settle_ms": 12000.0,
                "standoff_ms": 3000.0,
            },
        },
    }
    rows = dict(timing_breakdown_rows(metrics))
    assert rows["Total mission time"] == "584.81 s"
    assert rows["Nav wall time (leg1+leg2)"] == "580.00 s"
    assert rows["Movement"] == "120.00 s"
    assert rows["Mapping / perception (SLAM/L2)"] == "45.00 s"
    assert rows["Leg 1 wall time"] == "300.00 s"
