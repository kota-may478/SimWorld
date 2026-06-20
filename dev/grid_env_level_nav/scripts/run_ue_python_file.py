#!/usr/bin/env python3
"""Run a Windows-side UE Python file through UnrealCV ``vrun py``."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT / "grid_env_hri"))

import grid_env_hri_simulation as geh  # noqa: E402
import ue_client_guard  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("windows_path")
    args = parser.parse_args()

    command = f"vrun py exec(open(r'{args.windows_path}', encoding='utf-8').read())"
    with ue_client_guard.exclusive_ue_client_lock():
        ucv, _ = ue_client_guard.prepare_ue_connection(force_new=True)
        raw = ucv.client.request(command, timeout=60)
        print("VRUN_PY_FILE:", raw)
        geh.release_connection(ucv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
