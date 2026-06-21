#!/usr/bin/env python3
import socket
import sys

host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
port = 9000
s = socket.socket()
s.settimeout(5)
try:
    s.connect((host, port))
    banner = s.recv(128)
    print(f"OK {host}:{port} {banner!r}")
except OSError as exc:
    print(f"FAIL {host}:{port} {exc}")
    raise SystemExit(1)
finally:
    s.close()
