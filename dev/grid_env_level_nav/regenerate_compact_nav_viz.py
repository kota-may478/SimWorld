#!/usr/bin/env python3
"""Backward-compatible entry point → scenarios/compact_nav/regenerate_viz.py"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parent
sys.path.insert(0, str(_PKG))
_TARGET = _PKG / "scenarios" / "compact_nav" / "regenerate_viz.py"
runpy.run_path(str(_TARGET), run_name="__main__")
