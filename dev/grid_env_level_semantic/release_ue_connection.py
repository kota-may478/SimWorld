#!/usr/bin/env python3
"""Release stale UnrealCV TCP sessions from this machine (WSL).

UnrealCV accepts only one client. A Jupyter kernel left running after a notebook
cell often keeps port 9000 busy and blocks CLI runs.

Usage:
  conda activate simworld
  python release_ue_connection.py

Before CLI / a fresh notebook run:
  1. Kernel → Restart in any open SimWorld notebooks, OR run this script
  2. If PIE was stopped, start Play again on Level
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
GEH_DIR = THIS_DIR.parent / "grid_env_hri"
if str(GEH_DIR) not in sys.path:
    sys.path.insert(0, str(GEH_DIR))

import grid_env_hri_simulation as geh  # noqa: E402
from ue_client_guard import (  # noqa: E402
    cleanup_port_before_connect,
    release_ue_client_lock,
    windows_tcp_states_on_port,
)


def main() -> int:
    release_ue_client_lock()
    cleanup_port_before_connect(except_pid=os.getpid())
    print("[release] module-level UnrealCV session cleared")

    states = windows_tcp_states_on_port()
    if states:
        print(f"[release] Windows :9000 states: {states}")
    for host in geh._ue_host_candidates():
        ok = geh._probe_unrealcv_endpoint(host, geh.UE_PORT, timeout_s=3.0)
        print(f"[release] probe {host}:{geh.UE_PORT} -> {'OK' if ok else 'no'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
