#!/usr/bin/env python3
"""Wait until 127.0.0.1:9000 accepts TCP (SimWorld UnrealCV)."""
import socket
import sys
import time

host, port = "127.0.0.1", 9000
deadline = time.monotonic() + float(sys.argv[1] if len(sys.argv) > 1 else "120")
while time.monotonic() < deadline:
    try:
        with socket.create_connection((host, port), timeout=2.0):
            print(f"OK {host}:{port}")
            raise SystemExit(0)
    except OSError:
        time.sleep(2.0)
print(f"TIMEOUT {host}:{port}", file=sys.stderr)
raise SystemExit(1)
