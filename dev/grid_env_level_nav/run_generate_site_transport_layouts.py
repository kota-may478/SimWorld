#!/usr/bin/env python3
"""Generate 10 site_transport_20m layout variant JSON registries."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parent
sys.path.insert(0, str(_PKG))
_TARGET = _PKG / "scenarios" / "site_transport_20m" / "generate_layouts.py"
runpy.run_path(str(_TARGET), run_name="__main__")
