"""Simulate Cursor 'Existing Jupyter Server' kernel connection."""
from __future__ import annotations

import json
import sys
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:8899"
TOKEN = "simworld-cursor"


def api(path: str, method: str = "GET", body: dict | None = None) -> dict:
    url = f"{BASE}{path}"
    if "?" in url:
        url += f"&token={TOKEN}"
    else:
        url += f"?token={TOKEN}"
    data = None
    headers = {"Content-Type": "application/json"}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=30) as resp:
        raw = resp.read().decode("utf-8")
        return json.loads(raw) if raw else {}


def main() -> int:
    try:
        api("/api")
    except urllib.error.URLError as exc:
        print(f"Jupyter server not reachable at {BASE}: {exc}", file=sys.stderr)
        print("Start it with: dev/llm_material_transport/start_jupyter_for_cursor.sh", file=sys.stderr)
        return 1

    specs = api("/api/kernelspecs")
    names = sorted(specs.get("kernelspecs", {}))
    print("kernelspecs:", names)
    if "simworld" not in names:
        print("simworld kernel missing on server", file=sys.stderr)
        return 1

    session = api(
        "/api/sessions",
        method="POST",
        body={
            "kernel": {"name": "simworld"},
            "path": "dev/llm_material_transport/_cursor_kernel_test.ipynb",
            "type": "notebook",
        },
    )
    kernel_id = session["kernel"]["id"]
    session_id = session["id"]
    print("kernel id:", kernel_id)
    print("session id:", session_id)

    api(f"/api/sessions/{session_id}", method="DELETE")
    print("server connect ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
