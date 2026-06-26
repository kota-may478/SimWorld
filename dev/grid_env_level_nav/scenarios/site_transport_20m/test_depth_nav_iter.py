#!/usr/bin/env python3
"""Unit tests for per-nav-iter depth fetch budget."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from depth_frame_cache import DepthFrameCache  # noqa: E402
from depth_nav_iter import DepthNavIterBudget  # noqa: E402


def test_iter_budget_reuses_second_call() -> None:
    cache = DepthFrameCache(ttl_s=5.0, pose_delta_max_cm=100.0)
    budget = DepthNavIterBudget()
    budget.begin_iter()
    pose = (0.0, 0.0)
    calls = {"n": 0}

    def fetch() -> np.ndarray:
        calls["n"] += 1
        return np.array([[1.0]], dtype=np.float32)

    def record(_raw: np.ndarray, depth_m: np.ndarray) -> float:
        return 100.0

    cache.refresh_forward_depth_cm(pose, fetch, record, force=True)
    budget.note_ue_fetch()
    assert calls["n"] == 1
    assert budget.should_fetch_ue(cache, pose, max_age_s=cache.ttl_s) is False
    reused = cache.reuse_cached_forward_cm()
    assert reused == 100.0
    assert cache.hits == 1


def test_invalidate_resets_budget() -> None:
    budget = DepthNavIterBudget()
    budget.begin_iter()
    budget.note_ue_fetch()
    budget.on_invalidate()
    assert budget.ue_fetches_this_iter == 0
