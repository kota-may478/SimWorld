#!/usr/bin/env python3
"""Release UnrealCV session held by the active Jupyter kernel (single-client UE)."""
from __future__ import annotations

import json
import sys
from pathlib import Path

from jupyter_client import BlockingKernelClient
from jupyter_client.connect import find_connection_file


def _kernel_json() -> Path:
    runtime = Path("/run/user/1000/jupyter/runtime")
    if not runtime.is_dir():
        raise SystemExit(f"no jupyter runtime dir: {runtime}")
    files = sorted(runtime.glob("kernel-*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not files:
        raise SystemExit("no kernel-*.json in jupyter runtime")
    return files[0]


def main() -> int:
    conn_path = _kernel_json()
    print(f"[kernel] using {conn_path}")
    info = json.loads(conn_path.read_text(encoding="utf-8"))
    client = BlockingKernelClient()
    client.load_connection_file(str(conn_path))
    client.start_channels()
    try:
        msg_id = client.execute(
            "import grid_env_hri_simulation as geh\n"
            "geh.release_connection()\n"
            "print('KERNEL_UE_RELEASED')",
            silent=False,
            store_history=False,
        )
        while True:
            msg = client.get_iopub_msg(timeout=30)
            if msg["parent_header"].get("msg_id") != msg_id:
                continue
            msg_type = msg["header"]["msg_type"]
            if msg_type == "stream" and msg["content"].get("name") == "stdout":
                text = msg["content"].get("text", "")
                sys.stdout.write(text)
                if "KERNEL_UE_RELEASED" in text:
                    break
            elif msg_type == "error":
                print(msg["content"], file=sys.stderr)
                return 1
            elif msg_type == "status" and msg["content"].get("execution_state") == "idle":
                break
    finally:
        client.stop_channels()
    print("[kernel] release_connection done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
