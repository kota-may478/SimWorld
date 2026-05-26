#!/usr/bin/env python3
"""Headless integration run for material_transport_llm (simworld env, UE required)."""
from __future__ import annotations

import os
import runpy
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = Path(__file__).resolve().parent / "material_transport_llm.py"

os.environ.setdefault("MPLBACKEND", "Agg")


def main() -> int:
    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(SCRIPT.parent))
    print(f"Running integration script: {SCRIPT}")
    try:
        runpy.run_path(str(SCRIPT), run_name="__main__")
    except Exception:
        traceback.print_exc()
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
