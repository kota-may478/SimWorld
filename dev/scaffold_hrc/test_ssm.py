#!/usr/bin/env python3
"""Unit tests for ISO/TS 15066 SSM helpers."""

from __future__ import annotations

import unittest

from oracle.ssm import apply_ssm, protective_separation_m, safety_index


class SsmHelperTest(unittest.TestCase):
    def test_stopped_robot_keeps_only_intrusion_distance(self) -> None:
        self.assertAlmostEqual(protective_separation_m(0.0), 0.15)
        self.assertGreater(protective_separation_m(1.0), protective_separation_m(0.2))

    def test_si_is_sep_over_sp(self) -> None:
        self.assertAlmostEqual(safety_index(1.2, 0.6), 2.0)
        self.assertLess(safety_index(0.4, 0.8), 1.0)

    def test_stop_when_si_below_one(self) -> None:
        speed, mode = apply_ssm(0.20, 1.0, on_site=True)
        self.assertEqual(mode, "stop")
        self.assertEqual(speed, 0.0)

    def test_above_iso_sp_does_not_reduce(self) -> None:
        sp = protective_separation_m(1.0)
        speed, mode = apply_ssm(1.10 * sp, 1.0, on_site=True)
        self.assertEqual(mode, "free")
        self.assertAlmostEqual(speed, 1.0)

    def test_extra_margin_stops_before_iso_sp(self) -> None:
        sp = protective_separation_m(1.0)
        sep = sp + 0.10
        speed, mode = apply_ssm(sep, 1.0, on_site=True, extra_margin_m=0.0)
        self.assertEqual(mode, "free")
        speed, mode = apply_ssm(sep, 1.0, on_site=True, extra_margin_m=0.30)
        self.assertEqual(mode, "stop")
        self.assertEqual(speed, 0.0)

    def test_corridor_is_free_but_receding_still_stops(self) -> None:
        speed, mode = apply_ssm(0.10, 1.0, on_site=False)
        self.assertEqual(mode, "free")
        self.assertAlmostEqual(speed, 1.0)
        speed, mode = apply_ssm(0.10, 1.0, on_site=True, receding=True)
        self.assertEqual(mode, "stop")
        self.assertEqual(speed, 0.0)


if __name__ == "__main__":
    unittest.main()
