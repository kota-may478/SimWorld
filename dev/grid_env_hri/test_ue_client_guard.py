#!/usr/bin/env python3
"""Unit tests for ue_client_guard (no UE)."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

_HRI_DIR = Path(__file__).resolve().parent
if str(_HRI_DIR) not in sys.path:
    sys.path.insert(0, str(_HRI_DIR))

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

    def test_wait_for_tcp_port_idle_no_ss(self) -> None:
        with mock.patch.object(guard, "_wsl_tcp_lines_on_port", return_value=[]):
            self.assertTrue(guard.wait_for_tcp_port_idle(timeout_s=0.1))

    def test_describe_port_conflicts_filters_pid(self) -> None:
        my_pid = 99999
        with mock.patch.object(
            guard,
            "_python_tcp_lines_on_port",
            return_value=["line-a"],
        ) as mock_lines:
            out = guard.describe_port_9000_conflicts(except_pid=my_pid)
            mock_lines.assert_called_once()
            self.assertEqual(out, ["line-a"])


if __name__ == "__main__":
    unittest.main()
