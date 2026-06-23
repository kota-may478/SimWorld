#!/usr/bin/env python3
"""Re-export canonical placement module from package root.

Prefer: ``from construction_site_placement import ...`` (with ``bootstrap`` / package root on ``sys.path``).
"""

from __future__ import annotations

import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from construction_site_placement import *  # noqa: F401,F403
