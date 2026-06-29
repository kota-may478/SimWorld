#!/usr/bin/env python3
"""Entry point: preview one site_transport_20m layout in PIE."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parent
sys.path.insert(0, str(_PKG))
_TARGET = _PKG / "scenarios" / "site_transport_20m" / "preview_layout_pie.py"
runpy.run_path(str(_TARGET), run_name="__main__")
