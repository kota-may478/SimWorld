#!/usr/bin/env python3
"""UE condition-card batch resume / protocol matching (no PIE)."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from scene.erect_plan import build_erect_sequence
from scene.geometry import STAGE1_GEOM
from ue.run_condition_card_missions import mission_matches_protocol


class UeBatchResumeTests(unittest.TestCase):
    n_full = len(build_erect_sequence(STAGE1_GEOM))
    def _write(self, payload: dict) -> Path:
        tmp = Path(tempfile.mkdtemp()) / "mission.json"
        tmp.write_text(json.dumps(payload) + "\n", encoding="utf-8")
        return tmp

    def test_smoke_ok_is_not_full(self) -> None:
        path = self._write(
            {
                "ok": True,
                "n_items": 3,
                "n_filled": 3,
                "smin_m": None,
                "protocol": "smoke",
            }
        )
        self.assertTrue(mission_matches_protocol(path, "smoke"))
        self.assertFalse(mission_matches_protocol(path, "full"))
        self.assertFalse(mission_matches_protocol(path, "smin"))

    def test_full_requires_smin_and_all_items(self) -> None:
        path = self._write(
            {
                "ok": True,
                "n_items": self.n_full,
                "n_filled": self.n_full,
                "smin_m": 1.52,
                "protocol": "full",
            }
        )
        self.assertTrue(mission_matches_protocol(path, "full"))

    def test_full_without_smin_does_not_skip(self) -> None:
        path = self._write(
            {
                "ok": True,
                "n_items": self.n_full,
                "n_filled": self.n_full,
                "smin_m": None,
                "protocol": "full",
            }
        )
        self.assertFalse(mission_matches_protocol(path, "full"))


if __name__ == "__main__":
    unittest.main()
