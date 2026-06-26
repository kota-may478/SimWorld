#!/usr/bin/env python3
"""Unit tests for DepthFrameCache."""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from bootstrap import setup_paths

setup_paths(scenario="site_transport_20m")

from depth_frame_cache import DepthFrameCache  # noqa: E402


def test_cache_hit_on_same_pose() -> None:
    cache = DepthFrameCache(ttl_s=1.0)
    pose = (100.0, 200.0)
    calls = {"n": 0}

    def fetch() -> np.ndarray:
        calls["n"] += 1
        return np.array([[1.5]], dtype=np.float32)

    def record(_raw: np.ndarray, depth_m: np.ndarray) -> float:
        return float(np.min(depth_m) * 100.0)

    first = cache.refresh_forward_depth_cm(pose, fetch, record)
    second = cache.refresh_forward_depth_cm(pose, fetch, record)
    assert first == 150.0
    assert second == 150.0
    assert calls["n"] == 1
    assert cache.hits == 1
    assert cache.misses == 1


def test_invalidate_on_move() -> None:
    cache = DepthFrameCache(move_invalidate_cm=30.0)
    pose = (0.0, 0.0)

    def fetch() -> np.ndarray:
        return np.array([[2.0]], dtype=np.float32)

    def record(_raw: np.ndarray, depth_m: np.ndarray) -> float:
        return 200.0

    cache.refresh_forward_depth_cm(pose, fetch, record)
    cache.note_move_cm(35.0)
    assert cache.get_depth_m() is None
    cache.refresh_forward_depth_cm(pose, fetch, record)
    assert cache.misses == 2


def test_try_get_fresh_forward_cm() -> None:
    cache = DepthFrameCache(ttl_s=1.0)
    pose = (1.0, 2.0)
    calls = {"n": 0}

    def fetch() -> np.ndarray:
        calls["n"] += 1
        return np.array([[2.0]], dtype=np.float32)

    def record(_raw: np.ndarray, depth_m: np.ndarray) -> float:
        return 200.0

    cache.refresh_forward_depth_cm(pose, fetch, record, force=True)
    assert cache.try_get_fresh_forward_cm(pose) == 200.0
    assert calls["n"] == 1
    assert cache.hits == 1


def test_prefetch_async_hit() -> None:
    cache = DepthFrameCache(ttl_s=1.0)
    pose = (10.0, 20.0)
    calls = {"n": 0}

    def fetch() -> np.ndarray:
        calls["n"] += 1
        return np.array([[3.0]], dtype=np.float32)

    def record(_raw: np.ndarray, depth_m: np.ndarray) -> float:
        return 300.0

    cache.prefetch_async(pose, fetch, record)
    result = cache.get_or_wait(
        pose,
        fetch,
        record,
        max_wait_s=0.5,
        force=False,
        max_age_s=cache.ttl_s,
    )
    assert result == 300.0
    assert calls["n"] == 1
    assert cache.prefetch_hits >= 1
