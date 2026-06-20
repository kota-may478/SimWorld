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
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator, Optional, Tuple

from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV

UE_PORT = 9000
_LOCK_PATH = Path(os.environ.get("SIMWORLD_UE_LOCK", "/tmp/simworld_ue9000.lock"))
_PORT_IDLE_WAIT_S = float(os.environ.get("UE_PORT_IDLE_WAIT_S", "8.0"))
_PORT_IDLE_POLL_S = 0.35
_LOCK_WAIT_TIMEOUT_S = float(os.environ.get("UE_LOCK_WAIT_TIMEOUT_S", "6.0"))
_shutdown_installed = False
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


def _python_tcp_lines_on_port(
    *states: str,
    port: int = UE_PORT,
    except_pid: Optional[int] = None,
) -> list[str]:
    lines = _wsl_tcp_lines_on_port(*states, port=port)
    if except_pid is None:
        return lines
    filtered: list[str] = []
    for line in lines:
        if "pid=" not in line:
            continue
        pid_part = line.split("pid=")[1].split(",")[0]
        try:
            pid = int(pid_part)
        except ValueError:
            filtered.append(line)
            continue
        if pid != except_pid:
            filtered.append(line)
    return filtered


def wait_for_tcp_port_idle(
    port: int = UE_PORT,
    *,
    timeout_s: float = _PORT_IDLE_WAIT_S,
    except_pid: Optional[int] = None,
) -> bool:
    """Wait until no other WSL Python holds :port in non-idle TCP states."""
    deadline = time.time() + max(0.0, timeout_s)
    while time.time() < deadline:
        busy = _python_tcp_lines_on_port(
            "established",
            "close-wait",
            "fin-wait-1",
            "fin-wait-2",
            "syn-sent",
            port=port,
            except_pid=except_pid,
        )
        if not busy:
            return True
        time.sleep(_PORT_IDLE_POLL_S)
    return False


def describe_port_9000_conflicts(*, except_pid: Optional[int] = None) -> list[str]:
    return _python_tcp_lines_on_port(
        "established",
        "close-wait",
        "fin-wait-1",
        "fin-wait-2",
        "syn-sent",
        port=UE_PORT,
        except_pid=except_pid,
    )


def install_graceful_ue_shutdown() -> None:
    """Release UnrealCV on SIGINT/SIGTERM/atexit so :9000 does not stay CLOSE-WAIT."""
    global _shutdown_installed
    if _shutdown_installed:
        return
    import grid_env_hri_simulation as geh

    def _shutdown(*_args) -> None:
        try:
            geh.release_connection()
        except Exception:
            pass
        try:
            release_ue_client_lock()
        except Exception:
            pass

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    atexit.register(_shutdown)
    _shutdown_installed = True


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


def _pid_command(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/cmdline").read_text(encoding="utf-8").replace("\x00", " ").strip()
    except OSError:
        return ""


def _terminate_pid(pid: int, *, reason: str) -> bool:
    if pid <= 0 or pid == os.getpid():
        return False
    cmd = _pid_command(pid)
    try:
        os.kill(pid, signal.SIGTERM)
        print(f"[UE guard] SIGTERM {reason} pid={pid} cmd={cmd[:96]!r}")
        return True
    except OSError as exc:
        print(f"[UE guard] SIGTERM {reason} pid={pid} failed: {exc}")
        return False


def _lock_holder_pid() -> Optional[int]:
    try:
        raw = _LOCK_PATH.read_text(encoding="utf-8").strip()
    except OSError:
        return None
    try:
        return int(raw)
    except ValueError:
        return None


def _cleanup_stale_lock_holder() -> bool:
    holder = _lock_holder_pid()
    if holder is None or holder == os.getpid():
        return False
    cmd = _pid_command(holder)
    if not cmd:
        return False
    cmd_lower = cmd.lower()
    if "simworld" not in cmd_lower and "run_site_transport" not in cmd_lower:
        return False
    return _terminate_pid(holder, reason="UE lock holder")


def _windows_tcp_rows_on_port(port: int = UE_PORT) -> list[str]:
    """Return Windows Get-NetTCPConnection rows for :port when running under WSL."""
    try:
        out = subprocess.check_output(
            [
                "powershell.exe",
                "-NoProfile",
                "-Command",
                (
                    f"Get-NetTCPConnection -LocalPort {port} -ErrorAction SilentlyContinue "
                    "| Select-Object LocalAddress,LocalPort,State,OwningProcess "
                    "| ConvertTo-Csv -NoTypeInformation"
                ),
            ],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=4.0,
        )
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
        return []
    return [line.strip() for line in out.splitlines()[1:] if line.strip()]


def diagnose_windows_port_9000_state() -> None:
    rows = _windows_tcp_rows_on_port()
    if not rows:
        return
    states = [row.split(",")[2].strip('"').lower() for row in rows if len(row.split(",")) >= 3]
    if "closewait" in states or states.count("established") > 1:
        print(f"[UE guard] Windows :{UE_PORT} TCP state before connect:")
        for row in rows:
            print(f"  {row}")
        print(
            "[UE guard] cleaning WSL Python clients before opening a new UnrealCV session"
        )
        terminate_stale_wsl_python_clients_on_port(except_pid=os.getpid())
        wait_for_tcp_port_idle(except_pid=os.getpid(), timeout_s=2.0)


def acquire_ue_client_lock(*, blocking: bool = True) -> bool:
    """Process-wide file lock so only one script owns the UE client at a time."""
    global _lock_fd
    if _lock_fd is not None:
        return True
    _LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(str(_LOCK_PATH), os.O_CREAT | os.O_RDWR, 0o644)
    deadline = time.time() + (_LOCK_WAIT_TIMEOUT_S if blocking else 0.0)
    while True:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            break
        except BlockingIOError:
            holder = _lock_holder_pid()
            print(
                f"[UE guard] another process holds {_LOCK_PATH}"
                f"{f' (pid={holder})' if holder else ''}"
            )
            terminate_stale_wsl_python_clients_on_port(except_pid=os.getpid())
            _cleanup_stale_lock_holder()
            if not blocking or time.time() >= deadline:
                os.close(fd)
                return False
            time.sleep(0.5)
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


def prepare_ue_connection(
    *,
    force_new: bool = True,
    kill_stale_clients: bool = True,
    acquire_lock: bool = True,
    wait_port_idle: bool = True,
    ucv: Optional[UnrealCV] = None,
    communicator: Optional[Communicator] = None,
) -> Tuple[UnrealCV, Communicator]:
    """Acquire lock, clear stale :9000 clients, release old session, then connect once."""
    return ensure_exclusive_ue_session(
        force_new=force_new,
        kill_stale_clients=kill_stale_clients,
        acquire_lock=acquire_lock,
        wait_port_idle=wait_port_idle,
        ucv=ucv,
        communicator=communicator,
    )


def ensure_exclusive_ue_session(
    *,
    force_new: bool = False,
    kill_stale_clients: bool = True,
    acquire_lock: bool = True,
    wait_port_idle: bool = True,
    ucv: Optional[UnrealCV] = None,
    communicator: Optional[Communicator] = None,
) -> Tuple[UnrealCV, Communicator]:
    """One client on :9000: optional stale kill, file lock, then connect."""
    import grid_env_hri_simulation as geh

    install_graceful_ue_shutdown()

    if acquire_lock and not acquire_ue_client_lock(blocking=True):
        raise RuntimeError(
            "Another SimWorld script holds the UE client lock. "
            "Restart Jupyter kernels or wait for the other script to finish."
        )

    if kill_stale_clients:
        n = terminate_stale_wsl_python_clients_on_port(except_pid=os.getpid())
        if n:
            print(f"[UE guard] cleared {n} stale WSL Python client(s) on :{UE_PORT}")
        diagnose_windows_port_9000_state()

    # Always drop any module/notebook session before opening a new TCP client.
    geh.release_connection(ucv, communicator=communicator)

    if wait_port_idle:
        idle = wait_for_tcp_port_idle(except_pid=os.getpid())
        if not idle:
            conflicts = describe_port_9000_conflicts(except_pid=os.getpid())
            print(
                f"[UE guard] WARN: :{UE_PORT} still busy after wait "
                f"({len(conflicts)} foreign python socket(s))"
            )
            for line in conflicts[:4]:
                print(f"  {line}")

    return geh.ensure_connection(force_new=True)
