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

import subprocess
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
GEH_DIR = THIS_DIR.parent / "grid_env_hri"
if str(GEH_DIR) not in sys.path:
    sys.path.insert(0, str(GEH_DIR))

import grid_env_hri_simulation as geh  # noqa: E402


def _wsl_clients_on_9000() -> list[str]:
    try:
        out = subprocess.check_output(
            ["ss", "-tnp", "state", "established", "(", "dport", "=", ":9000", ")"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return []
    lines = []
    for line in out.strip().splitlines()[1:]:
        if "python" in line.lower():
            lines.append(line.strip())
    return lines


def main() -> int:
    clients = _wsl_clients_on_9000()
    if clients:
        print("[release] WSL Python still connected to :9000:")
        for line in clients:
            print(f"  {line}")
        print("[release] → Restart Jupyter Kernel or close the notebook, then re-run.")
    else:
        print("[release] No WSL Python ESTABLISHED on :9000")

    geh.release_connection()
    print("[release] module-level UnrealCV session cleared")

    for host in geh._ue_host_candidates():
        ok = geh._probe_unrealcv_endpoint(host, geh.UE_PORT, timeout_s=3.0)
        print(f"[release] probe {host}:{geh.UE_PORT} -> {'OK' if ok else 'no'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
