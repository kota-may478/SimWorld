#!/usr/bin/env python3
"""Unit tests for proximity violation metrics."""

from __future__ import annotations

import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from metrics import MissionRecorder, PROXIMITY_VIOLATION_CM  # noqa: E402
from zones import ForbiddenZone  # noqa: E402


def test_proximity_violation_rate_uses_total_time() -> None:
    zones = [
        ForbiddenZone(
            zone_id="z0",
            rect_local_cm=(0.0, 0.0, 100.0, 100.0),
            note="test",
        )
    ]
    recorder = MissionRecorder(mission_t0=0.0, forbidden_zones=zones)
    recorder.record_pose((100.0, 100.0), now=0.0, proximity_dist_cm=80.0)
    recorder.record_pose((200.0, 200.0), now=5.0, proximity_dist_cm=200.0)
    recorder.record_pose((300.0, 300.0), now=10.0, proximity_dist_cm=200.0)
    metrics = recorder.finalize(
        success=True,
        mission_end_t=20.0,
        layout_id="test",
    )
    viol = metrics["violations"]
    assert viol["proximity_violation_threshold_cm"] == PROXIMITY_VIOLATION_CM
    assert viol["proximity_violation_denominator"] == "total_time_s"
    assert viol["proximity_violation_time_s"] == 5.0
    assert viol["proximity_violation_rate"] == 0.25
