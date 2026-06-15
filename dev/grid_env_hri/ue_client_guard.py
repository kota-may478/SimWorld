#!/usr/bin/env python3
"""Single-client guard for UnrealCV (:9000).

UnrealCV accepts one TCP client. A second WSL Python process (Jupyter kernel + CLI)
or a reconnect while the old socket is in CLOSE-WAIT often causes:
  - Connection reset by peer during spawn_bp
  - Fail to send message, client is not connected

Use ``ensure_exclusive_ue_session()`` before PIE scripts / long L0 builds.
"""

from __future__ import annotations

import atexit
import fcntl
import os
import signal
import subprocess
import sys
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator, Optional, Tuple

from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV

UE_PORT = 9000
_LOCK_PATH = Path(os.environ.get("SIMWORLD_UE_LOCK", "/tmp/simworld_ue9000.lock"))
_lock_fd: Optional[int] = None


def _wsl_tcp_lines_on_port(*states: str, port: int = UE_PORT) -> list[str]:
    try:
        out = subprocess.check_output(
            [
                "ss",
                "-tnp",
                "state",
                *states,
                "(",
                "dport",
                "=",
                f":{port}",
                ")",
            ],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return []
    lines: list[str] = []
    for line in out.strip().splitlines()[1:]:
        if "python" in line.lower():
            lines.append(line.strip())
    return lines


def terminate_stale_wsl_python_clients_on_port(
    port: int = UE_PORT,
    *,
    except_pid: Optional[int] = None,
) -> int:
    """SIGTERM other WSL Python processes holding :port (ESTABLISHED / CLOSE-WAIT)."""
    my_pid = os.getpid() if except_pid is None else except_pid
    killed = 0
    for line in _wsl_tcp_lines_on_port(
        "established",
        "close-wait",
        "fin-wait-1",
        "fin-wait-2",
        port=port,
    ):
        if "pid=" not in line:
            continue
        pid_part = line.split("pid=")[1].split(",")[0]
        try:
            pid = int(pid_part)
        except ValueError:
            continue
        if pid == my_pid:
            continue
        try:
            os.kill(pid, signal.SIGTERM)
            killed += 1
            print(f"[UE guard] SIGTERM stale client pid={pid} ({line[:72]}...)")
        except OSError as exc:
            print(f"[UE guard] SIGTERM pid={pid} failed: {exc}")
            try:
                os.kill(pid, signal.SIGKILL)
                killed += 1
                print(f"[UE guard] SIGKILL pid={pid}")
            except OSError as exc2:
                print(f"[UE guard] SIGKILL pid={pid} failed: {exc2}")
    return killed


def acquire_ue_client_lock(*, blocking: bool = True) -> bool:
    """Process-wide file lock so only one script owns the UE client at a time."""
    global _lock_fd
    if _lock_fd is not None:
        return True
    _LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(str(_LOCK_PATH), os.O_CREAT | os.O_RDWR, 0o644)
    flags = fcntl.LOCK_EX if blocking else fcntl.LOCK_EX | fcntl.LOCK_NB
    try:
        fcntl.flock(fd, flags)
    except BlockingIOError:
        os.close(fd)
        holder = ""
        try:
            holder = _LOCK_PATH.read_text(encoding="utf-8").strip()
        except OSError:
            pass
        print(
            f"[UE guard] another process holds {_LOCK_PATH}"
            f"{f' (pid={holder})' if holder else ''}"
        )
        return False
    os.ftruncate(fd, 0)
    os.write(fd, str(os.getpid()).encode())
    _lock_fd = fd
    atexit.register(release_ue_client_lock)
    return True


def release_ue_client_lock() -> None:
    global _lock_fd
    if _lock_fd is None:
        return
    try:
        fcntl.flock(_lock_fd, fcntl.LOCK_UN)
    except OSError:
        pass
    try:
        os.close(_lock_fd)
    except OSError:
        pass
    _lock_fd = None


@contextmanager
def exclusive_ue_client_lock() -> Iterator[None]:
    if not acquire_ue_client_lock(blocking=True):
        raise RuntimeError(
            f"Could not acquire UE client lock ({_LOCK_PATH}). "
            "Stop other SimWorld notebooks/CLI or run release_ue_connection.py."
        )
    try:
        yield
    finally:
        release_ue_client_lock()


def ensure_exclusive_ue_session(
    *,
    force_new: bool = False,
    kill_stale_clients: bool = True,
    acquire_lock: bool = True,
    ucv: Optional[UnrealCV] = None,
    communicator: Optional[Communicator] = None,
) -> Tuple[UnrealCV, Communicator]:
    """One client on :9000: optional stale kill, file lock, then connect."""
    import grid_env_hri_simulation as geh

    if kill_stale_clients:
        n = terminate_stale_wsl_python_clients_on_port(except_pid=os.getpid())
        if n:
            print(f"[UE guard] cleared {n} stale WSL Python client(s) on :{UE_PORT}")
    if acquire_lock and not acquire_ue_client_lock(blocking=True):
        raise RuntimeError(
            "Another SimWorld script holds the UE client lock. "
            "Restart Jupyter kernels or wait for the other script to finish."
        )
    if force_new:
        geh.release_connection(ucv, communicator=communicator)
    return geh.reconnect_if_needed(ucv=ucv, communicator=communicator)
