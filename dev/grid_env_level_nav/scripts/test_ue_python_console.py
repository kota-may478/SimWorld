#!/usr/bin/env python3
"""Smoke-test UE Python console through UnrealCV vrun."""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT / "grid_env_hri"))

import grid_env_hri_simulation as geh  # noqa: E402
import ue_client_guard  # noqa: E402


def main() -> int:
    with ue_client_guard.exclusive_ue_client_lock():
        ucv, _ = ue_client_guard.prepare_ue_connection(force_new=True)
        raw = ucv.client.request("vrun py unreal.log('SimWorld UE Python console OK')")
        print("VRUN_PY:", raw)
        geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
