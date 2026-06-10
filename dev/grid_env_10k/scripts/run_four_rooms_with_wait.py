#!/usr/bin/env python3
"""Wait for UnrealCV, then run four-room scenario with one TCP client."""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = Path(__file__).resolve().parent
G10K = ROOT / "dev" / "grid_env_10k"
PY = Path(sys.executable)
PROBE = SCRIPTS / "_probe_ue_port.py"
SCENARIO = G10K / "grid_env_10k_four_rooms_pie.py"

WAIT_S = float(__import__("os").environ.get("UE_WAIT_S", "300"))
POLL_S = 5.0


def probe() -> bool:
    r = subprocess.run([str(PY), str(PROBE)], capture_output=True, text=True)
    print(r.stdout.strip() or r.stderr.strip())
    return r.returncode == 0


def main() -> int:
    deadline = time.monotonic() + WAIT_S
    print(f"[Wait] UnrealCV on 127.0.0.1:9000 (up to {WAIT_S:.0f}s) ...")
    while time.monotonic() < deadline:
        if probe():
            break
        time.sleep(POLL_S)
    else:
        print(
            "[Wait] TIMEOUT — UE Editor で PIE Stop → Play。"
            "WSL: ss -tnp state established '( dport = :9000 )' で python 接続が残っていれば kill。"
        )
        return 1

    print("[Run] four-room scenario (single UnrealCV client) ...")
    return subprocess.call([str(PY), "-u", str(SCENARIO)])


if __name__ == "__main__":
    raise SystemExit(main())
