#!/usr/bin/env python3
"""Unit tests for ue_client_guard (no UE)."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

import ue_client_guard as guard


class UeClientGuardTest(unittest.TestCase):
    def tearDown(self) -> None:
        guard.release_ue_client_lock()

    def test_lock_acquire_release(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            lock_path = Path(tmp) / "ue.lock"
            with mock.patch.object(guard, "_LOCK_PATH", lock_path):
                self.assertTrue(guard.acquire_ue_client_lock(blocking=True))
                self.assertTrue(lock_path.read_text(encoding="utf-8").strip().isdigit())
                guard.release_ue_client_lock()
                self.assertIsNone(guard._lock_fd)

    def test_exclusive_context_manager(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            lock_path = Path(tmp) / "ue.lock"
            with mock.patch.object(guard, "_LOCK_PATH", lock_path):
                with guard.exclusive_ue_client_lock():
                    self.assertIsNotNone(guard._lock_fd)
                self.assertIsNone(guard._lock_fd)


if __name__ == "__main__":
    unittest.main()
