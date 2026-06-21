#!/usr/bin/env python3
"""Entry point: site_transport_20m E2E test (PIE required)."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parent
sys.path.insert(0, str(_PKG))
_TARGET = _PKG / "scenarios" / "site_transport_20m" / "run_test.py"
runpy.run_path(str(_TARGET), run_name="__main__")
