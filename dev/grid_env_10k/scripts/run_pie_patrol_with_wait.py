#!/usr/bin/env python3
"""Wait for UnrealCV, mount paks, run patrol (single process)."""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = Path(__file__).resolve().parent
PY = Path(sys.executable)
PROBE = SCRIPTS / "_probe_ue_port.py"
MOUNT = SCRIPTS / "mount_simworld_runtime_paks_pie.py"
PATROL = ROOT / "dev" / "grid_env_10k" / "grid_env_10k_pie_patrol.py"

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
            "[Wait] TIMEOUT — UE Editor で PIE を一度 Stop → 再 Play してください。"
            "（古い UnrealCV 接続が残ると新規接続できません）"
        )
        return 1

    print("[Run] mount + probe ...")
    r = subprocess.run([str(PY), str(MOUNT)])
    if r.returncode != 0:
        return r.returncode

    print("[Run] patrol scenario ...")
    return subprocess.call([str(PY), str(PATROL)])


if __name__ == "__main__":
    raise SystemExit(main())
