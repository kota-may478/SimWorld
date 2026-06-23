#!/usr/bin/env python3
"""30 m グリッド床 + 透明箱 10,000 + Humanoid + SpotDog（empty.umap 用）。

前提:
  1. Windows: C:\\SimWorldServer で SimWorld.exe を起動
       .\\SimWorld.exe -windowed -log /Game/Maps/empty.umap
  2. pakchunk9002 に BP_Floor_30x30 / BP_TransparentCube が含まれている
  3. WSL: conda activate simworld

座標系（UE cm、empty マップ）:
  - 床 30 m 四方の **左下隅** が (x, y) = (0, 0)
  - 床上面の高さ = FLOOR_TOP_Z_CM（既定 100 cm = empty 原点から 1 m）
  - 床は物理 OFF・固定。箱は床上に直接配置（既定）または短い物理落下（オプション）
  - Humanoid / Robot spawn on floor grid with AGENT_SPAWN_ABOVE_FLOOR_CM clearance
  - 箱の Actor Z は CUBE_PIVOT_AT_CENTER で下面/中心 pivot を切替（浮き見え時は 0=下面）

使い方:
  python grid_env_hri_simulation.py              # GRID_N=100（10,000 箱）
  GRID_N=3 python grid_env_hri_simulation.py    # 小規模テスト
"""

from __future__ import annotations

import json
import math
import os
import socket
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Iterable, Iterator, List, NamedTuple, Optional, Tuple

# ---- SimWorld ルートをパスに追加 ----
def _find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / "setup.py").exists() and (candidate / "simworld").is_dir():
            return candidate
    return here.parent.parent


ROOT = _find_project_root()
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simworld.agent.humanoid import Humanoid
from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV
from simworld.utils.vector import Vector

# ==============================================================
# アセット / 定数
# ==============================================================

FLOOR_BP = "/Game/CustomAssets/BP_Floor_30x30.BP_Floor_30x30_C"
# 床左下の格子全体（0.3 m ピッチ）。既定は非表示に近い → SetBlocking + 色付けで薄く表示
CUBE_BP = "/Game/CustomAssets/BP_TransparentCube.BP_TransparentCube_C"
HUMAN_BP = "/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C"
ROBOT_BP = "/Game/Robot_Dog/Blueprint/BP_SpotRobot.BP_SpotRobot_C"

FLOOR_ACTOR_NAME = "grid_floor_main"
ROBOT_ACTOR_NAME = "GridEnv_SpotRobot"

FLOOR_SIZE_M = 30.0
CUBE_SIZE_M = 0.3
CUBE_SIZE_CM = CUBE_SIZE_M * 100.0
CUBE_HALF_CM = CUBE_SIZE_CM / 2.0

# 床左下隅（マップ座標原点）[cm]
MAP_ORIGIN_XY_CM: Tuple[float, float] = (0.0, 0.0)
FLOOR_HALF_CM = FLOOR_SIZE_M * 50.0  # 1500 cm

# empty 原点から床上面まで 1 m
FLOOR_TOP_Z_CM = 100.0
# 床メッシュ pivot: spawn_grid 検証時、actor Z=0 で上面 ≈ Z=0 → 上面を FLOOR_TOP_Z_CM に合わせる
FLOOR_ACTOR_Z_CM = FLOOR_TOP_Z_CM

# 箱のみ短い落下（大きい値だと角の格子から床外へ弾き飛ばされる）
CUBE_SPAWN_ABOVE_FLOOR_CM = float(os.environ.get("CUBE_SPAWN_ABOVE_FLOOR_CM", "5.0"))
# BP root がメッシュ中心のとき 1。下面合わせのとき 0（約半辺分の浮きを解消）
CUBE_PIVOT_AT_CENTER = os.environ.get("CUBE_PIVOT_AT_CENTER", "0") not in {
    "0",
    "false",
    "False",
}
# 床上面との隙間 [cm]（貫通防止の微小オフセット）
CUBE_ON_FLOOR_EPS_CM = float(os.environ.get("CUBE_ON_FLOOR_EPS_CM", "0.5"))
# デモ立方体（通過試験4個）を床上から短く落下させて設置
DEMO_CUBE_PHYSICS_DROP = os.environ.get("DEMO_CUBE_PHYSICS_DROP", "0") not in {
    "0",
    "false",
    "False",
}
# Spawn agents slightly above floor top so capsule/mesh does not intersect on placement.
AGENT_SPAWN_ABOVE_FLOOR_CM = float(os.environ.get("AGENT_SPAWN_ABOVE_FLOOR_CM", "50.0"))
# Base_User_Agent root is at feet (same Z convention as hri_spotdog Z=100 on floor top).
# Set HUMANOID_ROOT_AT_FEET=0 only if your BP uses capsule-center pivot.
HUMANOID_ROOT_AT_FEET = os.environ.get("HUMANOID_ROOT_AT_FEET", "1") not in {
    "0",
    "false",
    "False",
}
HUMANOID_CAPSULE_HALF_HEIGHT_CM = float(
    os.environ.get("HUMANOID_CAPSULE_HALF_HEIGHT_CM", "88.0")
)


def resolve_floor_top_z_cm(ucv: UnrealCV) -> float:
    """Floor top Z [cm] from ``grid_floor_main`` when present, else ``FLOOR_TOP_Z_CM``."""
    if actor_exists(ucv, FLOOR_ACTOR_NAME):
        loc = try_get_location_cm(ucv, FLOOR_ACTOR_NAME)
        if loc is not None:
            return loc[2]
    return FLOOR_TOP_Z_CM


def robot_spawn_z_on_floor_cm(*, floor_top_z_cm: Optional[float] = None) -> float:
    """UE Z for SpotDog root (pivot near ground contact)."""
    top = FLOOR_TOP_Z_CM if floor_top_z_cm is None else floor_top_z_cm
    return top + AGENT_SPAWN_ABOVE_FLOOR_CM


def humanoid_spawn_z_on_floor_cm(*, floor_top_z_cm: Optional[float] = None) -> float:
    """UE Z for Humanoid root (feet on floor + clearance, or capsule-center pivot)."""
    if HUMANOID_ROOT_AT_FEET:
        return robot_spawn_z_on_floor_cm(floor_top_z_cm=floor_top_z_cm)
    top = FLOOR_TOP_Z_CM if floor_top_z_cm is None else floor_top_z_cm
    return top + HUMANOID_CAPSULE_HALF_HEIGHT_CM + AGENT_SPAWN_ABOVE_FLOOR_CM


def agent_spawn_z_on_floor_cm() -> float:
    """Backward-compatible default (robot-style ground pivot)."""
    return robot_spawn_z_on_floor_cm()


HUMAN_SPAWN_Z_CM = float(
    os.environ.get("HUMAN_SPAWN_Z_CM", str(humanoid_spawn_z_on_floor_cm()))
)
ROBOT_SPAWN_Z_CM = float(
    os.environ.get("ROBOT_SPAWN_Z_CM", str(robot_spawn_z_on_floor_cm()))
)
PHYSICS_ENABLE_DELAY_S = 0.08
SETTLE_AFTER_SPAWN_S = 6.0

# Human / Robot マップ座標 [m]（左下原点、material_transport grid_map と同じ比率）
HUMAN_MAP_XY_M = (1.0, 1.0)
ROBOT_MAP_XY_M = (1.0, 3.0)

# pakchunk9002: BP_TransparentCube の SetBlocking デモ（床面上・マップ座標 [m]、左下原点）
# 実体モード（SetBlocking True）— 従来 BP_Box 目印があった付近
DEMO_SOLID_MAP_XY_M: Tuple[Tuple[float, float], ...] = (
    (8.0, 8.0),
    (10.0, 8.0),
    (8.0, 10.0),
)
# 半透明モード（SetBlocking False）— Humanoid 足元ではなく視認しやすい位置
DEMO_TRANSLUCENT_MAP_XY_M: Tuple[Tuple[float, float], ...] = (
    (5.5, 5.5),
)
# 格子全体を SetBlocking True にしたときの薄い色（任意）
TRANSPARENT_CUBE_TINT = (
    int(os.environ.get("TRANSPARENT_CUBE_TINT_R", "120")),
    int(os.environ.get("TRANSPARENT_CUBE_TINT_G", "180")),
    int(os.environ.get("TRANSPARENT_CUBE_TINT_B", "255")),
)
SPAWN_DEMO_MODE_CUBES = os.environ.get("SPAWN_DEMO_MODE_CUBES", "1") not in {
    "0",
    "false",
    "False",
}
# 後方互換
SPAWN_VISIBLE_MARKERS = SPAWN_DEMO_MODE_CUBES

# SpotDog によるデモ立方体通過試験（ログ判定）
RUN_DEMO_PASSAGE_TESTS = os.environ.get("RUN_DEMO_PASSAGE_TESTS", "1") not in {
    "0",
    "false",
    "False",
}
RUN_TRANSLUCENT_TOGGLE_PASSAGE_TESTS = os.environ.get(
    "RUN_TRANSLUCENT_TOGGLE_PASSAGE_TESTS", "1"
) not in {"0", "false", "False"}
# 単一 TransparentCube で SetBlocking OFF/ON を Robot 通過試験のみ行う
SINGLE_TOGGLE_CUBE_NAME = os.environ.get("SINGLE_TOGGLE_CUBE_NAME", "toggle_test_cube")
SINGLE_TOGGLE_MAP_X_M = float(os.environ.get("SINGLE_TOGGLE_MAP_X_M", "5.5"))
SINGLE_TOGGLE_MAP_Y_M = float(os.environ.get("SINGLE_TOGGLE_MAP_Y_M", "5.5"))
# 通過試験などで marker_registry 外にスポーンする Actor（cleanup_spawned が常に削除）
GRID_ENV_EXTRA_CLEANUP_ACTORS: Tuple[str, ...] = (SINGLE_TOGGLE_CUBE_NAME,)
PASSAGE_AXIS_OFFSET_M = float(os.environ.get("PASSAGE_AXIS_OFFSET_M", "1.5"))
PASSAGE_ROBOT_SPEED = float(os.environ.get("PASSAGE_ROBOT_SPEED", "220"))
PASSAGE_MOVE_SLICE_S = float(os.environ.get("PASSAGE_MOVE_SLICE_S", "0.45"))
PASSAGE_MAX_MOVE_S = float(os.environ.get("PASSAGE_MAX_MOVE_S", "18"))
PASSAGE_ARRIVE_CM = float(os.environ.get("PASSAGE_ARRIVE_CM", "35"))
PASS_THROUGH_MIN_PROGRESS_RATIO = float(
    os.environ.get("PASS_THROUGH_MIN_PROGRESS_RATIO", "0.82")
)
PASS_THROUGH_MAX_MISS_CM = float(
    os.environ.get("PASS_THROUGH_MAX_MISS_CM", str(CUBE_HALF_CM + 45.0))
)
BLOCKED_MAX_PROGRESS_RATIO = float(os.environ.get("BLOCKED_MAX_PROGRESS_RATIO", "0.62"))
BLOCKED_MIN_PROGRESS_RATIO = float(os.environ.get("BLOCKED_MIN_PROGRESS_RATIO", "0.15"))

GRID_N = int(os.environ.get("GRID_N", "100"))
SPAWN_INTERVAL_S = float(os.environ.get("SPAWN_INTERVAL_S", "0.005"))
# 格子箱: 既定は床上・透過モード（SetBlocking False）・物理シミュ OFF（床突き抜け防止）
GRID_CUBE_BLOCKING = os.environ.get("GRID_CUBE_BLOCKING", "0") not in {
    "0",
    "false",
    "False",
}
# 1 にすると SetBlocking True のあと物理落下（床コリジョンが弱いと沈むことがある）
CUBE_ENABLE_PHYSICS = os.environ.get("CUBE_ENABLE_PHYSICS", "0") not in {
    "0",
    "false",
    "False",
}
# Humanoid / Robot に Simulated Physics を有効にするとラグドール化するため既定 OFF
AGENT_ENABLE_PHYSICS = os.environ.get("AGENT_ENABLE_PHYSICS", "0") not in {"0", "false", "False"}

# UnrealCV: client.request の既定 timeout は 5s。BP 初回ロードは遅いことがある。
UE_REQUEST_TIMEOUT_S = float(os.environ.get("UE_REQUEST_TIMEOUT_S", "30"))
UE_SPAWN_TIMEOUT_S = float(os.environ.get("UE_SPAWN_TIMEOUT_S", "180"))
UE_SPAWN_POLL_S = float(os.environ.get("UE_SPAWN_POLL_S", "1.0"))
UE_SPAWN_POLL_MAX = int(os.environ.get("UE_SPAWN_POLL_MAX", "60"))
UE_PORT = 9000
UE_TCP_PROBE_TIMEOUT_S = float(os.environ.get("UE_TCP_PROBE_TIMEOUT_S", "3"))
UE_UNREALCV_MAX_ATTEMPTS = int(os.environ.get("UE_UNREALCV_MAX_ATTEMPTS", "5"))
UE_UNREALCV_RETRY_INTERVAL_S = float(os.environ.get("UE_UNREALCV_RETRY_INTERVAL_S", "0.5"))
UE_CONNECT_RETRIES = int(os.environ.get("UE_CONNECT_RETRIES", "10"))
UE_CONNECT_RETRY_WAIT_S = float(os.environ.get("UE_CONNECT_RETRY_WAIT_S", "3.0"))
UE_RELEASE_SETTLE_S = float(os.environ.get("UE_RELEASE_SETTLE_S", "1.5"))
UE_PING_TIMEOUT_S = float(os.environ.get("UE_PING_TIMEOUT_S", "2.0"))
# When 1, TCP connect without UnrealCV banner still counts as reachable (PIE busy).
UE_PROBE_RELAXED_TCP = os.environ.get("UE_PROBE_RELAXED_TCP", "1") not in {
    "0",
    "false",
    "False",
}

_UE_SESSION_UCV: Optional[UnrealCV] = None
_UE_SESSION_COMM: Optional[Communicator] = None
_UE_SESSION_HOST: Optional[str] = None


# ==============================================================
# UE 接続（Notebook / CLI 共通）
# ==============================================================

def _is_wsl() -> bool:
    version_path = Path("/proc/version")
    if not version_path.exists():
        return False
    text = version_path.read_text(encoding="utf-8").lower()
    return "microsoft" in text or "wsl" in text


def _wsl_default_gateway_ip() -> Optional[str]:
    try:
        result = subprocess.run(
            ["ip", "route", "show", "default"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    parts = result.stdout.split()
    if len(parts) >= 3 and parts[0] == "default" and parts[1] == "via":
        return parts[2]
    return None


def _windows_host_ip_from_resolv() -> Optional[str]:
    resolv_path = Path("/etc/resolv.conf")
    if not resolv_path.exists():
        return None
    for line in resolv_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line.startswith("nameserver"):
            parts = line.split()
            if len(parts) >= 2:
                return parts[1]
    return None


def _port_listener_hint(port: int = UE_PORT) -> str:
    try:
        result = subprocess.run(
            ["ss", "-tlnp"],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    needle = f":{port}"
    lines = [line for line in result.stdout.splitlines() if needle in line and "LISTEN" in line]
    return "; ".join(lines[:2])


def _probe_unrealcv_endpoint(
    host: str,
    port: int = UE_PORT,
    timeout_s: Optional[float] = None,
) -> bool:
    """TCP で接続し UnrealCV バナー（connected）を確認。到達不能ホストは数秒で諦める。"""
    timeout = UE_TCP_PROBE_TIMEOUT_S if timeout_s is None else timeout_s
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect((host, port))
        banner = b""
        deadline = time.monotonic() + timeout
        while len(banner) < 128 and time.monotonic() < deadline:
            try:
                chunk = sock.recv(64)
            except socket.timeout:
                break
            if not chunk:
                break
            banner += chunk
            if b"connected" in banner.lower():
                return True
        if b"connected" in banner.lower():
            return True
        if UE_PROBE_RELAXED_TCP:
            print(
                f"[UE] probe relaxed OK: {host}:{port} TCP open "
                f"(banner={banner[:48]!r})"
            )
            return True
        return False
    except OSError as exc:
        print(f"[UE] probe TCP failed {host}:{port} — {exc}")
        return False
    finally:
        sock.close()


def _ping_ucv(ucv: UnrealCV) -> bool:
    """Lightweight liveness check for an existing UnrealCV client."""
    try:
        if not ucv.client.isconnected():
            return False
        with ucv.lock:
            ucv.client.request("vget /version", timeout=UE_PING_TIMEOUT_S)
        return True
    except Exception:
        return False


def release_connection(
    ucv: Optional[UnrealCV] = None,
    *,
    communicator: Optional[Communicator] = None,
) -> None:
    """Disconnect module-level UnrealCV session (call before Kernel restart / PIE reset).

    Pass notebook ``ucv`` when ``importlib.reload`` cleared the module session but the
    kernel still holds the live TCP client.
    """
    global _UE_SESSION_UCV, _UE_SESSION_COMM, _UE_SESSION_HOST
    for client in (ucv, _UE_SESSION_UCV):
        if client is None:
            continue
        try:
            import ue_client_guard as _guard

            _guard._shutdown_ucv_socket(client)
        except Exception:
            try:
                client.disconnect()
            except Exception:
                pass
    _UE_SESSION_UCV = None
    _UE_SESSION_COMM = None
    _UE_SESSION_HOST = None
    if UE_RELEASE_SETTLE_S > 0:
        time.sleep(UE_RELEASE_SETTLE_S)
    print("[UE] session released")


def reconnect_if_needed(
    *,
    ucv: Optional[UnrealCV] = None,
    communicator: Optional[Communicator] = None,
    force_new: bool = False,
) -> Tuple[UnrealCV, Communicator]:
    """Reuse a live client; reconnect only when ping fails or ``force_new``."""
    if force_new:
        release_connection(ucv, communicator=communicator)
        return ensure_connection(force_new=True)

    if ucv is not None and _ping_ucv(ucv):
        comm = communicator if communicator is not None else Communicator(ucv)
        host = _UE_SESSION_HOST or getattr(ucv, "ip", None) or "127.0.0.1"
        return _store_ue_session(ucv, comm, host)

    if not force_new and _UE_SESSION_UCV is not None and _ping_ucv(_UE_SESSION_UCV):
        if _UE_SESSION_COMM is None:
            _UE_SESSION_COMM = Communicator(_UE_SESSION_UCV)
        return _UE_SESSION_UCV, _UE_SESSION_COMM

    if ucv is not None or _UE_SESSION_UCV is not None:
        print("[UE] UnrealCV session not responding — reconnecting")
    release_connection(ucv, communicator=communicator)
    return ensure_connection(force_new=True)


def settle_after_actor_destroy(
    ucv: UnrealCV,
    *,
    settle_s: float = 2.0,
    run_clean_garbage: bool = False,
) -> None:
    """Let UE finish destroy/GC before the next spawn.

    Avoid ``clean_garbage`` here — forcing GC right after pawn destroy has crashed
  PIE on /Game/Maps/Level (SpotDog Character + camera + controller).
    """
    for _ in range(3):
        try:
            ucv.tick()
        except Exception:
            break
        time.sleep(0.1)
    if run_clean_garbage:
        _prepare_ue_spawn(ucv)
    if settle_s > 0:
        time.sleep(settle_s)


def adopt_ue_session(
    ucv: UnrealCV,
    communicator: Optional[Communicator] = None,
) -> Tuple[UnrealCV, Communicator]:
    """Re-register a live notebook client after ``importlib.reload(geh)``."""
    if not _ping_ucv(ucv):
        raise ConnectionError("notebook ucv is not responding")
    comm = communicator if communicator is not None else Communicator(ucv)
    host = _UE_SESSION_HOST or getattr(ucv, "ip", None) or "127.0.0.1"
    print(f"[UE] Adopted existing UnrealCV session at {host}:{UE_PORT}")
    return _store_ue_session(ucv, comm, host)


def _store_ue_session(
    ucv: UnrealCV,
    communicator: Communicator,
    host: str,
) -> Tuple[UnrealCV, Communicator]:
    global _UE_SESSION_UCV, _UE_SESSION_COMM, _UE_SESSION_HOST
    _UE_SESSION_UCV = ucv
    _UE_SESSION_COMM = communicator
    _UE_SESSION_HOST = host
    return ucv, communicator


def _wsl_localhost_port_shadowed_by_python(port: int = UE_PORT) -> bool:
    """WSL の 127.0.0.1:port が Python に占有され UE ではないか（malformat magic 255 の典型原因）。"""
    hint = _port_listener_hint(port)
    if not hint:
        return False
    for line in hint.split(";"):
        line_lower = line.lower()
        if "python" not in line_lower:
            continue
        if "127.0.0.1" in line or "0.0.0.0" in line:
            return True
    return False


def _ue_host_candidates() -> List[str]:
    hosts: List[str] = []
    override = os.environ.get("UE_HOST", "").strip()
    if override:
        return [override]

    if _is_wsl():
        # WSL2 mirrored / localhost 転送時は 127.0.0.1 が Windows 上の SimWorld に届くことが多い
        if not _wsl_localhost_port_shadowed_by_python(UE_PORT):
            hosts.append("127.0.0.1")
        for candidate in (_wsl_default_gateway_ip(), _windows_host_ip_from_resolv()):
            if candidate and candidate not in hosts:
                hosts.append(candidate)
        return hosts

    if "127.0.0.1" not in hosts:
        hosts.append("127.0.0.1")
    resolv_ip = _windows_host_ip_from_resolv()
    if resolv_ip and resolv_ip not in hosts:
        hosts.append(resolv_ip)
    return hosts


@contextmanager
def _limited_unrealcv_check_connection() -> Iterator[None]:
    """到達不能ホストで 30 回×1s リトライしないよう check_connection を一時的に制限。"""
    import simworld.communicator.unrealcv as sucv_module

    original = sucv_module.UnrealCV.check_connection
    max_attempts = UE_UNREALCV_MAX_ATTEMPTS
    retry_interval = UE_UNREALCV_RETRY_INTERVAL_S

    def limited_check(
        self: UnrealCV,
        max_attempts: int = max_attempts,
        retry_interval: float = retry_interval,
    ) -> None:
        return original(self, max_attempts=max_attempts, retry_interval=retry_interval)

    sucv_module.UnrealCV.check_connection = limited_check  # type: ignore[method-assign]
    try:
        yield
    finally:
        sucv_module.UnrealCV.check_connection = original  # type: ignore[method-assign]


def _connect_unrealcv(host: str, port: int = UE_PORT) -> UnrealCV:
    with _limited_unrealcv_check_connection():
        return UnrealCV(port=port, ip=host)


def ensure_connection(
    *,
    force_new: bool = False,
    ucv: Optional[UnrealCV] = None,
    communicator: Optional[Communicator] = None,
) -> Tuple[UnrealCV, Communicator]:
    """UnrealCV / Communicator を初期化（短い TCP プローブ後に接続）。

    Reuses the module-level session when still alive (notebook re-runs).
    Pass ``ucv`` / ``communicator`` from the notebook to reuse globals explicitly.
    Set ``force_new=True`` to drop any stale TCP session before reconnecting.
    """
    global _UE_SESSION_UCV, _UE_SESSION_COMM

    if ucv is not None:
        if _ping_ucv(ucv):
            comm = communicator if communicator is not None else Communicator(ucv)
            host = _UE_SESSION_HOST or getattr(ucv, "ip", None) or "127.0.0.1"
            print(f"[UE] Reusing notebook UnrealCV session at {host}:{UE_PORT}")
            return _store_ue_session(ucv, comm, host)
        print("[UE] notebook ucv not responding — opening a new session")
        try:
            ucv.disconnect()
        except Exception:
            pass

    if force_new:
        release_connection()

    if not force_new and _UE_SESSION_UCV is not None:
        if _ping_ucv(_UE_SESSION_UCV):
            if _UE_SESSION_COMM is None:
                _UE_SESSION_COMM = Communicator(_UE_SESSION_UCV)
            print(
                f"[UE] Reusing UnrealCV session at "
                f"{_UE_SESSION_HOST or _UE_SESSION_UCV.ip}:{UE_PORT}"
            )
            return _UE_SESSION_UCV, _UE_SESSION_COMM
        print("[UE] Stale UnrealCV session — disconnecting before reconnect")
        release_connection()

    if _is_wsl() and _wsl_localhost_port_shadowed_by_python(UE_PORT):
        print(
            "[UE] 127.0.0.1:9000 は WSL 内 Python が LISTEN 中のためスキップします。"
            f" ({_port_listener_hint(UE_PORT)})"
        )

    candidates = _ue_host_candidates()
    print(f"[UE] Probing UnrealCV on {candidates} (timeout={UE_TCP_PROBE_TIMEOUT_S:g}s each) ...")

    try:
        import ue_client_guard as _guard

        _guard.install_graceful_ue_shutdown()
        busy = _guard.describe_port_9000_conflicts(except_pid=os.getpid())
        if busy:
            print(f"[UE] WARN: {len(busy)} other Python socket(s) on :{UE_PORT} before connect")
            for line in busy[:3]:
                print(f"  {line}")
            _guard.terminate_stale_wsl_python_clients_on_port(except_pid=os.getpid())
            _guard.wait_for_tcp_port_idle(except_pid=os.getpid(), timeout_s=2.0)
    except Exception:
        pass

    errors: List[str] = []
    reachable: List[str] = []
    for host in candidates:
        if host == "127.0.0.1" and _wsl_localhost_port_shadowed_by_python(UE_PORT):
            errors.append(
                f"{host}:{UE_PORT} — skipped (WSL Python shadows port; "
                f"listener: {_port_listener_hint(UE_PORT)})"
            )
            continue
        if _probe_unrealcv_endpoint(host, UE_PORT):
            reachable.append(host)
            print(f"[UE] probe OK: {host}:{UE_PORT}")
        else:
            extra = ""
            if host == "127.0.0.1":
                hint = _port_listener_hint(UE_PORT)
                if hint:
                    extra = f" (listener: {hint})"
            errors.append(f"{host}:{UE_PORT} — not reachable / not UnrealCV{extra}")
            print(f"[UE] probe skip: {host}:{UE_PORT}")

    if not reachable:
        errors.append(
            "no host answered with UnrealCV banner — SimWorld が 9000 で LISTEN していないか、"
            "WSL から Windows へ届いていません（.wslconfig の localhostForwarding 等を確認）"
        )
    else:
        for host in reachable:
            for attempt in range(1, UE_CONNECT_RETRIES + 1):
                if attempt > 1:
                    wait_s = UE_CONNECT_RETRY_WAIT_S * (attempt - 1)
                    print(
                        f"[UE] connect retry {attempt}/{UE_CONNECT_RETRIES} "
                        f"on {host}:{UE_PORT} (wait {wait_s:.0f}s) ..."
                    )
                    time.sleep(wait_s)
                    try:
                        import ue_client_guard as _guard

                        _guard.wait_for_tcp_port_idle(
                            except_pid=os.getpid(),
                            timeout_s=min(wait_s + 2.0, 15.0),
                        )
                    except Exception:
                        pass
                try:
                    ucv = _connect_unrealcv(host)
                    communicator = Communicator(ucv)
                    print(f"[UE] Connected via UnrealCV at {host}:{UE_PORT}")
                    return _store_ue_session(ucv, communicator, host)
                except Exception as exc:
                    err = f"{host}:{UE_PORT} attempt {attempt} — {exc}"
                    errors.append(err)
                    print(f"[UE] connect failed: {err}")

    shadow_hint = _port_listener_hint(UE_PORT)
    shadow_note = ""
    if shadow_hint and "python" in shadow_hint.lower():
        shadow_note = (
            "\n\n[WSL] 127.0.0.1:9000 が WSL 内の Python に占有されています"
            f" ({shadow_hint})。"
            " Jupyter Kernel → Restart、または `kill` で該当 Python を終了後、"
            " Windows で SimWorld.exe を起動してから再接続してください。"
            " 応急処置: `export UE_HOST=<WindowsホストIP>`（ゲートウェイ / resolv の nameserver）。"
        )

    stale_note = (
        "\n\n[単一クライアント] UnrealCV は TCP 1 本のみ。"
        " `Listen`（UE）+ `Established` 1 本（Python）は正常で、2 本目の接続は開けません。"
        " 1) `release_ue_connection()` または Kernel → Restart、"
        " 2) Cell 2 → Cell 5 を再実行。"
        " PIE Stop → Play が必要なのは UE クラッシュ時や CloseWait が複数残ったときだけです。"
    )

    raise ConnectionError(
        "Unreal Engine (UnrealCV) に接続できませんでした。\n"
        "1. Windows で SimWorld を起動:\n"
        "     cd C:\\SimWorldServer\n"
        "     .\\SimWorld.exe -windowed -log /Game/Maps/empty.umap\n"
        "   （Editor PIE の場合は grid_100x100 を Play）\n"
        "2. ログに `Start listening on port 9000` があるか確認\n"
        "3. Kernel → Restart 後、初期設定 → UE 接続の順で再実行\n"
        "試行結果:\n  - " + "\n  - ".join(errors)
        + shadow_note
        + stale_note
    )


# ==============================================================
# 座標変換
# ==============================================================

def floor_center_xy_cm() -> Tuple[float, float]:
    """床 Static Mesh 中心（左下隅が MAP_ORIGIN 時）。"""
    ox, oy = MAP_ORIGIN_XY_CM
    return ox + FLOOR_HALF_CM, oy + FLOOR_HALF_CM


def map_xy_m_to_world_cm(map_xy_m: Tuple[float, float]) -> Tuple[float, float]:
    """マップ座標 [m]（左下原点）→ UE 世界 XY [cm]。"""
    mx, my = map_xy_m
    ox, oy = MAP_ORIGIN_XY_CM
    return ox + mx * 100.0, oy + my * 100.0


def cube_bottom_z_on_floor_cm() -> float:
    """床上面に箱の下面を載せる Actor Z（pivot=下面）。"""
    return FLOOR_TOP_Z_CM + CUBE_ON_FLOOR_EPS_CM


def cube_center_z_on_floor_cm() -> float:
    """床上面に箱の中心を載せる Actor Z（pivot=中心）。"""
    return FLOOR_TOP_Z_CM + CUBE_HALF_CM + CUBE_ON_FLOOR_EPS_CM


def cube_actor_z_on_floor_cm() -> float:
    """CUBE_PIVOT_AT_CENTER に応じた床上 Actor Z [cm]。"""
    if CUBE_PIVOT_AT_CENTER:
        return cube_center_z_on_floor_cm()
    return cube_bottom_z_on_floor_cm()


def cube_actor_location_on_floor_cm(x: float, y: float) -> Tuple[float, float, float]:
    """map 平面座標 [cm] と床上 Z で Actor location を返す。"""
    return x, y, cube_actor_z_on_floor_cm()


def cube_actor_location_physics_drop_cm(x: float, y: float) -> Tuple[float, float, float]:
    """床上位置から CUBE_SPAWN_ABOVE_FLOOR_CM だけ上げた落下開始位置 [cm]。"""
    x, y, z = cube_actor_location_on_floor_cm(x, y)
    return x, y, z + CUBE_SPAWN_ABOVE_FLOOR_CM


def cube_center_cm(
    row: int,
    col: int,
    *,
    above_floor_cm: Optional[float] = None,
    on_floor: bool = False,
) -> Tuple[float, float, float]:
    """格子 (row, col) の箱 Actor location [cm]（下面 or 中心 pivot）。"""
    ox, oy = MAP_ORIGIN_XY_CM
    x = ox + col * CUBE_SIZE_CM + CUBE_HALF_CM
    y = oy + row * CUBE_SIZE_CM + CUBE_HALF_CM
    if on_floor or not CUBE_ENABLE_PHYSICS:
        return cube_actor_location_on_floor_cm(x, y)
    drop_cm = CUBE_SPAWN_ABOVE_FLOOR_CM if above_floor_cm is None else above_floor_cm
    return x, y, cube_actor_z_on_floor_cm() + drop_cm


def agent_spawn_xyz_cm(
    map_xy_m: Tuple[float, float],
    *,
    spawn_z_cm: Optional[float] = None,
    agent_kind: str = "robot",
    floor_top_z_cm: Optional[float] = None,
) -> Tuple[float, float, float]:
    """Humanoid / Robot spawn XYZ [cm] on the floor grid (kinematic placement)."""
    x, y = map_xy_m_to_world_cm(map_xy_m)
    if spawn_z_cm is not None:
        z = spawn_z_cm
    elif agent_kind == "humanoid":
        z = humanoid_spawn_z_on_floor_cm(floor_top_z_cm=floor_top_z_cm)
    else:
        z = robot_spawn_z_on_floor_cm(floor_top_z_cm=floor_top_z_cm)
    return x, y, z


def humanoid_feet_z_cm(actor_z_cm: float) -> float:
    """Approximate foot height from Humanoid actor Z."""
    if HUMANOID_ROOT_AT_FEET:
        return actor_z_cm
    return actor_z_cm - HUMANOID_CAPSULE_HALF_HEIGHT_CM


def configure_humanoid_kinematic(
    ucv: UnrealCV,
    communicator: Communicator,
    human_name: str,
    loc: Tuple[float, float, float],
    *,
    human_id: Optional[int] = None,
) -> None:
    """Place Humanoid without sim physics; stop locomotion / montage."""
    ucv.set_physics(human_name, False)
    ucv.set_movable(human_name, True)
    ucv.set_location(list(loc), human_name)
    ucv.set_orientation((0.0, 0.0, 0.0), human_name)
    ucv.set_collision(human_name, True)
    for cmd in (
        lambda: ucv.humanoid_stop(human_name),
        lambda: ucv.humanoid_stop_current_action(human_name),
    ):
        try:
            cmd()
        except Exception:
            pass
    try:
        if human_id is not None:
            communicator.humanoid_set_speed(human_id, 0.0)
        else:
            ucv.humanoid_set_speed(human_name, 0.0)
    except Exception:
        pass
    time.sleep(PHYSICS_ENABLE_DELAY_S)


def verify_humanoid_on_floor(
    ucv: UnrealCV,
    human_name: str,
    *,
    floor_top_z_cm: float = FLOOR_TOP_Z_CM,
) -> Tuple[bool, Optional[Tuple[float, float, float]]]:
    """True when estimated feet are on/above the floor top."""
    loc = try_get_location_cm(ucv, human_name)
    if loc is None:
        return False, None
    feet_z = humanoid_feet_z_cm(loc[2])
    ok = feet_z >= floor_top_z_cm - 5.0
    return ok, loc


def place_humanoid_on_floor(
    ucv: UnrealCV,
    communicator: Communicator,
    human_name: str,
    base_loc: Tuple[float, float, float],
    *,
    human_id: Optional[int] = None,
    floor_top_z_cm: Optional[float] = None,
) -> bool:
    """Configure Humanoid at ``base_loc``; bump Z and retry until feet clear the floor."""
    floor_z = FLOOR_TOP_Z_CM if floor_top_z_cm is None else floor_top_z_cm
    for attempt in range(5):
        z_bump = attempt * 30.0
        loc = (base_loc[0], base_loc[1], base_loc[2] + z_bump)
        configure_humanoid_kinematic(
            ucv, communicator, human_name, loc, human_id=human_id
        )
        ok, actual = verify_humanoid_on_floor(
            ucv, human_name, floor_top_z_cm=floor_z
        )
        if ok:
            feet = humanoid_feet_z_cm(actual[2]) if actual else float("nan")
            suffix = f" (+{z_bump:.0f}cm retry)" if z_bump > 0 else ""
            print(
                f"[Humanoid] on-floor OK {human_name} actor={_fmt_xyz(actual)} "
                f"feet≈{feet:.1f}cm floor_top={floor_z:.1f}cm{suffix}"
            )
            return True
        if actual:
            feet = humanoid_feet_z_cm(actual[2])
            print(
                f"[Humanoid] on-floor retry {attempt + 1}/5: "
                f"actor={_fmt_xyz(actual)} feet≈{feet:.1f}cm "
                f"(need >={floor_z - 5:.1f}cm)"
            )
    return False


# ==============================================================
# UnrealCV ヘルパ
# ==============================================================

def _parse_location_response(raw: Optional[str]) -> Optional[Tuple[float, float, float]]:
    if raw is None:
        return None
    parts = str(raw).strip().split()
    if len(parts) < 3:
        return None
    try:
        return float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        return None


def _ue_request(ucv: UnrealCV, cmd: str, *, timeout_s: Optional[float] = None) -> Optional[str]:
    """UnrealCV 同期リクエスト。失敗時は None（UnrealCV 1.2.x は timeout 未実装）。"""
    del timeout_s
    try:
        with ucv.lock:
            return ucv.client.request(cmd)
    except Exception as exc:
        print(f"[UE] request failed ({cmd[:72]!r}): {exc}")
        return None


def try_get_location_cm(ucv: UnrealCV, name: str) -> Optional[Tuple[float, float, float]]:
    """vget /object/{name}/location — 全オブジェクト列挙より軽い。"""
    if not name:
        return None
    raw = _ue_request(ucv, f"vget /object/{name}/location", timeout_s=min(15.0, UE_REQUEST_TIMEOUT_S))
    return _parse_location_response(raw)


def actor_names(ucv: UnrealCV) -> set[str]:
    raw = _ue_request(ucv, "vget /objects", timeout_s=max(UE_REQUEST_TIMEOUT_S, 60.0))
    if raw is None:
        print("[UE] warn: vget /objects failed — returning empty set")
        return set()
    return {str(name) for name in raw.split()}


def actor_exists(ucv: UnrealCV, name: str) -> bool:
    return try_get_location_cm(ucv, name) is not None


def _prepare_ue_spawn(ucv: UnrealCV) -> None:
    raw = _ue_request(ucv, "vset /action/clean_garbage", timeout_s=30.0)
    if raw is None:
        print("[UE] warn: clean_garbage failed")
    time.sleep(0.15)


def wait_until_actor_gone(
    ucv: UnrealCV,
    name: str,
    *,
    timeout_s: float = 3.0,
    poll_s: float = 0.15,
) -> bool:
    """Poll until ``name`` is no longer reachable (post-destroy rename safety)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not actor_exists(ucv, name):
            return True
        time.sleep(poll_s)
    return not actor_exists(ucv, name)


def _disable_actor_for_destroy(ucv: UnrealCV, name: str) -> None:
    """Best-effort shutdown before destroy (controller / physics / collision)."""
    try:
        ucv.enable_controller(name, False)
    except Exception:
        pass
    _ue_request(ucv, f"vset /object/{name}/physics 0", timeout_s=15.0)
    _ue_request(ucv, f"vset /object/{name}/collision 0", timeout_s=15.0)


def prepare_pawn_for_destroy(
    ucv: UnrealCV,
    name: str,
    *,
    stash_xyz: Optional[Tuple[float, float, float]] = None,
) -> None:
    """Shut down SpotDog / Character pawn before destroy (reduces PIE crashes)."""
    _disable_actor_for_destroy(ucv, name)
    try:
        ucv.set_movable(name, True)
    except Exception:
        pass
    if stash_xyz is not None:
        try:
            ucv.set_location(list(stash_xyz), name)
        except Exception:
            pass
    try:
        ucv.tick()
    except Exception:
        pass
    time.sleep(0.35)


def destroy_actor_safely(
    ucv: UnrealCV,
    name: str,
    *,
    timeout_s: float = 12.0,
    max_attempts: int = 4,
    profile: str = "default",
) -> bool:
    """Destroy ``name`` with retries and GC; return True when absent or gone.

    ``profile='pawn'``: single destroy attempt, no clean_garbage retries — for
    SpotDog / Character actors that have crashed UE when hammered with GC.
    """
    if not actor_exists(ucv, name):
        return True

    if profile == "pawn":
        print(f"[UE] destroy pawn {name!r} (single attempt, no clean_garbage) ...")
        prepare_pawn_for_destroy(ucv, name)
        _ue_request(ucv, f"vset /object/{name}/destroy", timeout_s=30.0)
        gone = wait_until_actor_gone(ucv, name, timeout_s=max(3.0, timeout_s))
        if not gone:
            print(
                f"[UE] warn: pawn {name!r} still present after destroy "
                "(reuse via teleport instead of respawn)"
            )
        return gone

    per_attempt_timeout = max(2.0, timeout_s / max_attempts)
    for attempt in range(1, max_attempts + 1):
        if not actor_exists(ucv, name):
            return True
        print(f"[UE] destroy {name!r} attempt {attempt}/{max_attempts} ...")
        _disable_actor_for_destroy(ucv, name)
        _ue_request(ucv, f"vset /object/{name}/destroy", timeout_s=30.0)
        if wait_until_actor_gone(ucv, name, timeout_s=per_attempt_timeout):
            return True
        _prepare_ue_spawn(ucv)
    gone = not actor_exists(ucv, name)
    if not gone:
        print(
            f"[UE] warn: {name!r} still present after {max_attempts} destroy attempts "
            "(reuse/teleport instead of respawn)"
        )
    return gone


def destroy_pawn_safely(ucv: UnrealCV, name: str, *, timeout_s: float = 12.0) -> bool:
    """Destroy a Character / SpotDog pawn with the crash-safe profile."""
    return destroy_actor_safely(ucv, name, timeout_s=timeout_s, profile="pawn")


def destroy_if_exists(ucv: UnrealCV, name: str) -> None:
    destroy_actor_safely(ucv, name)


def _spawn_bp_class_path(bp_path: str) -> str:
    """Stock UnrealCV ``vset /objects/spawn`` expects a UClass path (often *_C)."""
    if "." in bp_path and bp_path.endswith("_C"):
        return bp_path
    if "." in bp_path:
        return f"{bp_path}_C"
    return bp_path


def _spawn_bp_no_handler(res_text: str) -> bool:
    lower = res_text.lower()
    return "no handler found" in lower and "spawn_bp_asset" in lower


def spawn_bp(
    ucv: UnrealCV,
    bp_path: str,
    name: str,
    *,
    timeout_s: Optional[float] = None,
) -> bool:
    if not _ping_ucv(ucv):
        print(f"[UE] spawn_bp {name!r}: UnrealCV client not connected (call reconnect_if_needed)")
        return False
    timeout = UE_SPAWN_TIMEOUT_S if timeout_s is None else timeout_s
    class_path = _spawn_bp_class_path(bp_path)
    spawn_cmds: list[tuple[str, str]] = [
        (f"vset /objects/spawn_bp_asset {bp_path} {name}", "spawn_bp_asset"),
        (f"vset /objects/spawn {class_path} {name}", "spawn (stock UnrealCV)"),
    ]

    res_text = ""
    for cmd, label in spawn_cmds:
        print(f"[UE] {label} {name!r} (timeout={timeout:g}s) ...")
        res = _ue_request(ucv, cmd, timeout_s=timeout)
        if res is None:
            if not _ping_ucv(ucv):
                print(
                    f"[UE] {label}: connection lost during spawn "
                    "(UE may still be processing destroy — wait and reconnect_if_needed)"
                )
            else:
                print(
                    f"[UE] {label}: no response within {timeout:g}s — "
                    "SimWorld を前面に出す / pakchunk9002 / BP パスを確認"
                )
            return False
        res_text = str(res).strip()
        if res_text.lower().startswith("error"):
            if label == spawn_cmds[0][1] and _spawn_bp_no_handler(res_text):
                print(f"[UE] {label} unavailable — trying {spawn_cmds[1][1]} ...")
                continue
            if "renaming blocked" in res_text.lower() or "still in use" in res_text.lower():
                print(f"[UE] {label}: rename race — destroy + retry ...")
                if not destroy_actor_safely(ucv, name):
                    if actor_exists(ucv, name):
                        print(
                            f"[UE] {label}: abort spawn; {name!r} still alive "
                            "(caller should reuse/teleport)"
                        )
                        return False
                _prepare_ue_spawn(ucv)
                res = _ue_request(ucv, cmd, timeout_s=timeout)
                if res is None:
                    return False
                res_text = str(res).strip()
                if not res_text.lower().startswith("error"):
                    break
            print(f"[UE] {label} error: {res_text}")
            return False
        break
    else:
        return False

    for attempt in range(UE_SPAWN_POLL_MAX):
        if actor_exists(ucv, name):
            if attempt > 0:
                print(f"[UE] {name!r} appeared after {attempt + 1} poll(s)")
            return True
        time.sleep(UE_SPAWN_POLL_S)

    print(
        f"[UE] spawn_bp_asset: command returned but {name!r} has no location — "
        f"BP path ok? ({bp_path})"
    )
    return False


def _set_blocking_vbp_tokens(blocking: bool) -> Tuple[str, str]:
    """UnrealCV → BP Custom Event 向けの bool 表記（UE 実装差のフォールバック）。"""
    if blocking:
        return "True", "1"
    return "False", "0"


def set_cube_blocking_mode(
    ucv: UnrealCV,
    cube_id: str,
    *,
    blocking: bool,
    apply_tint: bool = False,
) -> bool:
    """BP_TransparentCube の Custom Event SetBlocking（見た目は BP マテリアル、当たりは blocking）。"""
    primary, fallback = _set_blocking_vbp_tokens(blocking)
    raw = _ue_request(
        ucv,
        f"vbp {cube_id} SetBlocking {primary}",
        timeout_s=15.0,
    )
    ok = raw is not None and not str(raw).strip().lower().startswith("error")
    if not ok:
        raw_fb = _ue_request(
            ucv,
            f"vbp {cube_id} SetBlocking {fallback}",
            timeout_s=15.0,
        )
        ok = raw_fb is not None and not str(raw_fb).strip().lower().startswith("error")
        if ok:
            print(f"  note: SetBlocking {fallback!r} ok for {cube_id} (primary {primary!r} failed)")
    if not ok:
        print(
            f"  warn: vbp SetBlocking {blocking} failed for {cube_id!r} — "
            f"BP の Custom Event / pakchunk9002 を確認"
        )

    ucv.set_collision(cube_id, blocking)
    if blocking and apply_tint:
        try:
            ucv.set_color(cube_id, list(TRANSPARENT_CUBE_TINT))
        except Exception as exc:
            print(f"  warn: set_color {cube_id}: {exc}")
    return ok


def enable_cube_blocking(ucv: UnrealCV, cube_id: str) -> None:
    """格子用: 実体モード + 薄い色付け（コリジョン ON）。"""
    set_cube_blocking_mode(ucv, cube_id, blocking=True, apply_tint=True)


def spawn_with_physics_drop(
    ucv: UnrealCV,
    bp_path: str,
    name: str,
    location: Tuple[float, float, float],
    *,
    enable_physics: bool = True,
    use_set_blocking: bool = False,
) -> bool:
    """物理 OFF で上空配置 → コリジョン ON → 物理 ON（material_transport と同順）。"""
    if not spawn_bp(ucv, bp_path, name):
        return False

    ucv.set_physics(name, False)
    ucv.set_collision(name, False)
    ucv.set_movable(name, True)
    ucv.set_location(list(location), name)
    ucv.set_orientation((0.0, 0.0, 0.0), name)
    time.sleep(PHYSICS_ENABLE_DELAY_S)

    if use_set_blocking:
        enable_cube_blocking(ucv, name)
    else:
        ucv.set_collision(name, True)

    if enable_physics:
        time.sleep(PHYSICS_ENABLE_DELAY_S)
        ucv.set_physics(name, True)
    return True


def spawn_fixed_floor(ucv: UnrealCV) -> bool:
    """30 m 床を 1 m 高度に固定配置（物理 OFF、落ちない）。"""
    cx, cy = floor_center_xy_cm()
    loc = (cx, cy, FLOOR_ACTOR_Z_CM)

    print("[Floor] prepare UE (clean_garbage) ...")
    _prepare_ue_spawn(ucv)
    print("[Floor] remove stale actor if any ...")
    destroy_if_exists(ucv, FLOOR_ACTOR_NAME)
    print(f"[Floor] spawn {FLOOR_BP} ...")
    if not spawn_bp(ucv, FLOOR_BP, FLOOR_ACTOR_NAME):
        print("[Floor] spawn failed — PAK / BP パス / SimWorld 再起動を確認")
        return False

    ucv.set_physics(FLOOR_ACTOR_NAME, False)
    ucv.set_movable(FLOOR_ACTOR_NAME, False)
    ucv.set_collision(FLOOR_ACTOR_NAME, True)
    ucv.set_location(list(loc), FLOOR_ACTOR_NAME)
    ucv.set_orientation((0.0, 0.0, 0.0), FLOOR_ACTOR_NAME)
    print(f"[Floor] fixed at center={loc} (corner @ {MAP_ORIGIN_XY_CM}, top Z≈{FLOOR_TOP_Z_CM} cm)")
    return True


def demo_cube_center_cm(map_xy_m: Tuple[float, float]) -> Tuple[float, float, float]:
    """BP_TransparentCube（30 cm）を床面上に静置する Actor location [cm]。"""
    x, y = map_xy_m_to_world_cm(map_xy_m)
    return cube_actor_location_on_floor_cm(x, y)


def _place_cube_kinematic_on_floor(
    ucv: UnrealCV,
    name: str,
    loc: Tuple[float, float, float],
    *,
    blocking: bool,
    apply_tint: bool,
) -> bool:
    """スポーン済み Actor を床上に固定配置（物理 OFF）。"""
    ucv.set_physics(name, False)
    ucv.set_collision(name, False)
    ucv.set_movable(name, True)
    ucv.set_location(list(loc), name)
    ucv.set_orientation((0.0, 0.0, 0.0), name)
    time.sleep(PHYSICS_ENABLE_DELAY_S)
    return set_cube_blocking_mode(ucv, name, blocking=blocking, apply_tint=apply_tint)


def spawn_transparent_cube_on_floor(
    ucv: UnrealCV,
    name: str,
    map_xy_m: Tuple[float, float],
    *,
    blocking: bool,
    apply_tint: bool = False,
    use_physics_drop: bool = False,
) -> Tuple[bool, Tuple[float, float, float]]:
    """BP_TransparentCube を床上面に設置（直接配置 or 短い物理落下）。"""
    x, y = map_xy_m_to_world_cm(map_xy_m)
    destroy_if_exists(ucv, name)

    if use_physics_drop:
        loc = cube_actor_location_physics_drop_cm(x, y)
        ok = spawn_with_physics_drop(
            ucv,
            CUBE_BP,
            name,
            loc,
            enable_physics=True,
            use_set_blocking=blocking,
        )
        if ok and not blocking:
            set_cube_blocking_mode(ucv, name, blocking=False, apply_tint=apply_tint)
        return ok, loc

    loc = cube_actor_location_on_floor_cm(x, y)
    if not spawn_bp(ucv, CUBE_BP, name):
        return False, loc
    ok = _place_cube_kinematic_on_floor(
        ucv, name, loc, blocking=blocking, apply_tint=apply_tint and blocking
    )
    return ok, loc


def _spawn_demo_transparent_cube(
    ucv: UnrealCV,
    name: str,
    map_xy_m: Tuple[float, float],
    *,
    blocking: bool,
) -> bool:
    """BP_TransparentCube デモ立方体（通過試験用）。"""
    ok, _loc = spawn_transparent_cube_on_floor(
        ucv,
        name,
        map_xy_m,
        blocking=blocking,
        apply_tint=blocking,
        use_physics_drop=DEMO_CUBE_PHYSICS_DROP,
    )
    return ok


def spawn_demo_mode_cubes(ucv: UnrealCV) -> dict[str, dict]:
    """pakchunk9002 の BP_TransparentCube を実体 / 半透明で床上に配置（モード視認用）。"""
    registry: dict[str, dict] = {}
    place_mode = (
        f"physics_drop +{CUBE_SPAWN_ABOVE_FLOOR_CM:.0f}cm"
        if DEMO_CUBE_PHYSICS_DROP
        else f"on_floor kinematic (pivot={'center' if CUBE_PIVOT_AT_CENTER else 'bottom'})"
    )
    print(
        f"[DemoCubes] solid(SetBlocking True) @ {list(DEMO_SOLID_MAP_XY_M)} m, "
        f"translucent(False) @ {list(DEMO_TRANSLUCENT_MAP_XY_M)} m "
        f"(BP={CUBE_BP}, placement={place_mode})"
    )

    for idx, map_xy in enumerate(DEMO_SOLID_MAP_XY_M):
        name = f"demo_solid_{idx:02d}"
        ok, loc = spawn_transparent_cube_on_floor(
            ucv, name, map_xy, blocking=True, apply_tint=True, use_physics_drop=DEMO_CUBE_PHYSICS_DROP
        )
        if not ok:
            print(f"  warn: demo solid spawn failed {name!r} — pakchunk9002 / BP を確認")
            continue
        registry[name] = {
            "map_xy_m": map_xy,
            "world_cm": loc,
            "blocking": True,
            "mode": "solid",
        }
        print(
            f"  {name} solid (blocking) map={map_xy} m → {_fmt_xyz(loc)} "
            "— 見た目は半透明グレー、当たりあり"
        )

    for idx, map_xy in enumerate(DEMO_TRANSLUCENT_MAP_XY_M):
        name = f"demo_translucent_{idx:02d}"
        ok, loc = spawn_transparent_cube_on_floor(
            ucv,
            name,
            map_xy,
            blocking=False,
            use_physics_drop=DEMO_CUBE_PHYSICS_DROP,
        )
        if not ok:
            print(f"  warn: demo translucent spawn failed {name!r}")
            continue
        registry[name] = {
            "map_xy_m": map_xy,
            "world_cm": loc,
            "blocking": False,
            "mode": "translucent",
        }
        print(
            f"  {name} pass-through map={map_xy} m → {_fmt_xyz(loc)} "
            "(SetBlocking False — 当たりなし・カメラ/エージェントが通過可)"
        )

    print(f"[DemoCubes] done: {len(registry)}")
    return registry


def spawn_visible_marker_cubes(ucv: UnrealCV) -> dict[str, dict]:
    """後方互換エイリアス（旧 BP_Box 目印 → BP_TransparentCube デモ）。"""
    return spawn_demo_mode_cubes(ucv)


def spawn_grid_cube_on_floor(
    ucv: UnrealCV,
    cube_id: str,
    row: int,
    col: int,
    *,
    blocking: bool,
) -> Tuple[bool, Tuple[float, float, float]]:
    """BP_TransparentCube を床上面に静置（デモ立方体と同系）。"""
    loc = cube_center_cm(row, col, on_floor=True)
    destroy_if_exists(ucv, cube_id)
    if not spawn_bp(ucv, CUBE_BP, cube_id):
        return False, loc
    ok = _place_cube_kinematic_on_floor(
        ucv, cube_id, loc, blocking=blocking, apply_tint=blocking
    )
    return ok, loc


def spawn_cubes(ucv: UnrealCV, grid_n: int) -> dict[str, dict]:
    registry: dict[str, dict] = {}
    total = grid_n * grid_n
    extent_m = grid_n * CUBE_SIZE_M
    use_physics_drop = CUBE_ENABLE_PHYSICS and GRID_CUBE_BLOCKING
    mode_label = (
        "blocking+physics_drop"
        if use_physics_drop
        else ("blocking on floor" if GRID_CUBE_BLOCKING else "pass-through on floor")
    )
    print(
        f"[Cubes] spawning {total} grid cubes (GRID_N={grid_n}, mode={mode_label})"
    )
    print(
        f"  grid covers map x,y in [0, {extent_m:.1f}] m (floor corner); "
        f"agents @ human {HUMAN_MAP_XY_M} m, robot {ROBOT_MAP_XY_M} m — "
        "camera near agents may not show the grid"
    )
    if not GRID_CUBE_BLOCKING:
        print(
            "  GRID_CUBE_BLOCKING=0: SetBlocking False (透過/通過可), "
            f"z≈{cube_actor_z_on_floor_cm():.1f} cm on floor top "
            f"(pivot={'center' if CUBE_PIVOT_AT_CENTER else 'bottom'})"
        )

    for row in range(grid_n):
        for col in range(grid_n):
            cube_id = f"cube_{row:03d}_{col:03d}"
            if use_physics_drop:
                loc = cube_center_cm(row, col)
                ok = spawn_with_physics_drop(
                    ucv,
                    CUBE_BP,
                    cube_id,
                    loc,
                    enable_physics=True,
                    use_set_blocking=True,
                )
            else:
                ok, loc = spawn_grid_cube_on_floor(
                    ucv,
                    cube_id,
                    row,
                    col,
                    blocking=GRID_CUBE_BLOCKING,
                )
            if ok:
                registry[cube_id] = {
                    "row": row,
                    "col": col,
                    "x_cm": loc[0],
                    "y_cm": loc[1],
                    "spawn_z_cm": loc[2],
                    "blocking": GRID_CUBE_BLOCKING,
                }
            else:
                print(f"  warn: failed {cube_id}")
            time.sleep(SPAWN_INTERVAL_S)

        if (row + 1) % 10 == 0 or row == grid_n - 1:
            print(f"  row {row + 1}/{grid_n} ({len(registry)}/{total})")

    print(f"[Cubes] done: {len(registry)}")
    return registry


def spawn_humanoid(communicator: Communicator, ucv: UnrealCV) -> Optional[str]:
    """Spawn Humanoid on the floor grid (kinematic, capsule-center Z)."""
    floor_z = resolve_floor_top_z_cm(ucv)
    loc = agent_spawn_xyz_cm(
        HUMAN_MAP_XY_M, agent_kind="humanoid", floor_top_z_cm=floor_z
    )
    human = Humanoid(position=Vector(loc[0], loc[1]), direction=Vector(1, 0))
    human_name = communicator.get_humanoid_name(human.id)
    destroy_actor_safely(ucv, human_name)
    if not spawn_bp(ucv, HUMAN_BP, human_name):
        print(f"[Humanoid] spawn_bp failed ({HUMAN_BP})")
        return None
    if not place_humanoid_on_floor(
        ucv,
        communicator,
        human_name,
        loc,
        human_id=human.id,
        floor_top_z_cm=floor_z,
    ):
        print(f"[Humanoid] placement failed for {human_name}")
        return None
    if AGENT_ENABLE_PHYSICS:
        print(
            "  warn: AGENT_ENABLE_PHYSICS=True is ignored for Humanoid (ragdoll risk)"
        )
    print(
        f"[Humanoid] {human_name} spawn @ {loc} "
        f"(capsule_half={HUMANOID_CAPSULE_HALF_HEIGHT_CM:.0f}cm, "
        f"clearance={AGENT_SPAWN_ABOVE_FLOOR_CM:.0f}cm)"
    )
    return human_name


def spawn_robot(ucv: UnrealCV) -> bool:
    """SpotDog を material_transport と同様に配置しコントローラを有効化。"""
    floor_z = resolve_floor_top_z_cm(ucv)
    loc = agent_spawn_xyz_cm(
        ROBOT_MAP_XY_M, agent_kind="robot", floor_top_z_cm=floor_z
    )
    destroy_pawn_safely(ucv, ROBOT_ACTOR_NAME)
    if not spawn_bp(ucv, ROBOT_BP, ROBOT_ACTOR_NAME):
        print("[Robot] spawn failed")
        return False

    ucv.set_physics(ROBOT_ACTOR_NAME, False)
    ucv.set_movable(ROBOT_ACTOR_NAME, True)
    ucv.set_location(list(loc), ROBOT_ACTOR_NAME)
    ucv.set_orientation((0.0, 0.0, 0.0), ROBOT_ACTOR_NAME)
    ucv.set_collision(ROBOT_ACTOR_NAME, True)
    ucv.enable_controller(ROBOT_ACTOR_NAME, True)
    if AGENT_ENABLE_PHYSICS:
        print(
            f"  warn: AGENT_ENABLE_PHYSICS=True は SpotDog を転倒させるため無視します"
        )
    print(
        f"[Robot] {ROBOT_ACTOR_NAME} @ {loc} "
        f"(controller on, z_clearance={AGENT_SPAWN_ABOVE_FLOOR_CM:.0f}cm)"
    )
    return True


def _fmt_xyz(loc: Tuple[float, float, float]) -> str:
    return f"({loc[0]:.1f}, {loc[1]:.1f}, {loc[2]:.1f})"


def settle_after_cube_spawn_if_needed(*, demo_physics_drop: bool = False) -> None:
    """格子箱またはデモ立方体の物理落下後に SETTLE_AFTER_SPAWN_S 待機。"""
    need_wait = SETTLE_AFTER_SPAWN_S > 0 and (
        (CUBE_ENABLE_PHYSICS and GRID_CUBE_BLOCKING) or demo_physics_drop
    )
    if need_wait:
        label = "demo cubes" if demo_physics_drop else "grid cubes"
        print(f"[Settle] waiting {SETTLE_AFTER_SPAWN_S}s for {label} physics ...")
        time.sleep(SETTLE_AFTER_SPAWN_S)
    elif SETTLE_AFTER_SPAWN_S > 0:
        print(
            "[Settle] skip cube physics wait "
            "(on-floor kinematic placement)"
        )


def report_spawn_state(
    ucv: UnrealCV,
    cube_registry: dict[str, dict],
    human_name: Optional[str],
    *,
    marker_registry: Optional[dict[str, dict]] = None,
    floor_z_min_cm: float = FLOOR_TOP_Z_CM - 5.0,
) -> None:
    """スポーン後の位置をログ出力（箱の床外落下・エージェント転倒の確認用）。"""
    pivot_label = "center" if CUBE_PIVOT_AT_CENTER else "bottom"
    expected_z = cube_actor_z_on_floor_cm()
    print(
        f"[Verify] actor locations after settle "
        f"(cube pivot={pivot_label}, expected actor Z≈{expected_z:.1f} cm, "
        f"floor top Z={FLOOR_TOP_Z_CM:.1f} cm):"
    )
    if actor_exists(ucv, FLOOR_ACTOR_NAME):
        floor_loc = ucv.get_location(FLOOR_ACTOR_NAME)
        print(f"  floor {FLOOR_ACTOR_NAME}: {_fmt_xyz(tuple(floor_loc))}")

    if human_name and actor_exists(ucv, human_name):
        loc = tuple(ucv.get_location(human_name))
        feet_z = humanoid_feet_z_cm(loc[2])
        ok = feet_z >= floor_z_min_cm
        print(
            f"  humanoid {human_name}: {_fmt_xyz(loc)} "
            f"feet≈{feet_z:.1f}cm {'OK' if ok else 'LOW-Z?'}"
        )
    elif human_name:
        print(f"  humanoid {human_name}: MISSING")

    if actor_exists(ucv, ROBOT_ACTOR_NAME):
        loc = tuple(ucv.get_location(ROBOT_ACTOR_NAME))
        ok = loc[2] >= floor_z_min_cm
        print(f"  robot {ROBOT_ACTOR_NAME}: {_fmt_xyz(loc)} {'OK' if ok else 'LOW-Z?'}")
    else:
        print(f"  robot {ROBOT_ACTOR_NAME}: MISSING")

    on_floor = 0
    below_floor = 0
    missing = 0
    sample_ids = sorted(cube_registry.keys())[:3]
    for cube_id in cube_registry:
        if not actor_exists(ucv, cube_id):
            missing += 1
            continue
        loc = tuple(ucv.get_location(cube_id))
        if loc[2] >= floor_z_min_cm:
            on_floor += 1
        else:
            below_floor += 1
    print(
        f"  cubes: on/above floor={on_floor}, below floor z={below_floor}, "
        f"missing={missing}, total={len(cube_registry)}"
    )
    for cube_id in sample_ids:
        if actor_exists(ucv, cube_id):
            loc = tuple(ucv.get_location(cube_id))
            print(f"    sample {cube_id}: {_fmt_xyz(loc)}")

    if marker_registry:
        print(f"  demo_cubes: {len(marker_registry)}")
        for demo_id, meta in sorted(marker_registry.items()):
            if actor_exists(ucv, demo_id):
                loc = tuple(ucv.get_location(demo_id))
                mode = meta.get("mode", "?")
                dz = loc[2] - expected_z
                z_note = "OK" if abs(dz) <= 3.0 else f"ZΔ{dz:+.1f}?"
                print(
                    f"    {demo_id} {mode} map={meta.get('map_xy_m')} "
                    f"→ {_fmt_xyz(loc)} {z_note}"
                )
            else:
                print(f"    {demo_id}: MISSING")


class PassageTrialVerdict(NamedTuple):
    demo_id: str
    expects_pass_through: bool
    passed: bool
    message: str
    progress_cm: float
    goal_distance_cm: float
    min_dist_obstacle_cm: float
    max_object_collision: int
    crossed_obstacle: bool


def parse_collision_counts(raw_response: object) -> dict:
    if isinstance(raw_response, dict):
        return raw_response
    if isinstance(raw_response, str):
        text = raw_response.strip()
        if not text or text.lower().startswith("error"):
            return {}
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return {}
    return {}


def max_object_collision_count(counts: dict) -> int:
    return int(counts.get("ObjectCollision", 0) or 0) + int(
        counts.get("BuildingCollision", 0) or 0
    )


def _normalize_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _robot_xy_cm(ucv: UnrealCV) -> Tuple[float, float]:
    loc = ucv.get_location(ROBOT_ACTOR_NAME)
    return float(loc[0]), float(loc[1])


def _robot_yaw_deg(ucv: UnrealCV) -> float:
    ori = ucv.get_orientation(ROBOT_ACTOR_NAME)
    return float(ori[1])


def _dist_xy_cm(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _yaw_toward_deg(from_xy: Tuple[float, float], to_xy: Tuple[float, float]) -> float:
    return math.degrees(math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0]))


def _projection_on_segment(
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    point_xy: Tuple[float, float],
) -> float:
    ax = goal_xy[0] - start_xy[0]
    ay = goal_xy[1] - start_xy[1]
    length = math.hypot(ax, ay)
    if length < 1e-6:
        return 0.0
    ux, uy = ax / length, ay / length
    px = point_xy[0] - start_xy[0]
    py = point_xy[1] - start_xy[1]
    return px * ux + py * uy


def passage_progress_along_segment(
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    final_xy: Tuple[float, float],
) -> float:
    """start→goal 軸方向への移動量 [cm]（負にならないようクリップ）。"""
    return max(0.0, _projection_on_segment(start_xy, goal_xy, final_xy))


def crossed_obstacle_plane(
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    obstacle_xy: Tuple[float, float],
    final_xy: Tuple[float, float],
    *,
    half_extent_cm: float = CUBE_HALF_CM,
    margin_cm: float = 10.0,
) -> bool:
    """最終位置が障害物立方体の「向こう側」平面を越えたか（直進試験用）。"""
    obs_proj = _projection_on_segment(start_xy, goal_xy, obstacle_xy)
    final_proj = _projection_on_segment(start_xy, goal_xy, final_xy)
    goal_proj = _projection_on_segment(start_xy, goal_xy, goal_xy)
    direction_sign = 1.0 if goal_proj >= obs_proj else -1.0
    threshold = obs_proj + direction_sign * (half_extent_cm + margin_cm)
    if direction_sign > 0:
        return final_proj >= threshold
    return final_proj <= threshold


def judge_passage_trial(
    *,
    expects_pass_through: bool,
    goal_distance_cm: float,
    progress_cm: float,
    min_dist_to_obstacle_cm: float,
    max_object_collision: int,
    crossed_obstacle: bool,
) -> Tuple[bool, str]:
    """通過可/不可の期待に対するログ判定（目視不要）。"""
    if goal_distance_cm < 1e-3:
        return False, "goal_distance_cm is zero"

    progress_ratio = progress_cm / goal_distance_cm

    if expects_pass_through:
        if not crossed_obstacle:
            return (
                False,
                f"did not cross obstacle plane (final progress {progress_cm:.0f} cm, "
                f"min_dist={min_dist_to_obstacle_cm:.0f} cm)",
            )
        if progress_ratio < PASS_THROUGH_MIN_PROGRESS_RATIO:
            return (
                False,
                f"progress {progress_cm:.0f}/{goal_distance_cm:.0f} cm "
                f"({progress_ratio:.2f} < {PASS_THROUGH_MIN_PROGRESS_RATIO})",
            )
        return (
            True,
            f"passed through (crossed_plane=True, progress_ratio={progress_ratio:.2f}, "
            f"min_dist={min_dist_to_obstacle_cm:.0f} cm, max_obj_coll={max_object_collision})",
        )

    if crossed_obstacle:
        return (
            False,
            "robot crossed obstacle plane — SetBlocking True / BP collision "
            "not effective (re-cook pakchunk9002 BP_TransparentCube SetBlocking event)",
        )
    if progress_ratio >= BLOCKED_MAX_PROGRESS_RATIO:
        return (
            False,
            f"progress {progress_cm:.0f}/{goal_distance_cm:.0f} cm "
            f"({progress_ratio:.2f} >= {BLOCKED_MAX_PROGRESS_RATIO}, should be blocked)",
        )
    if progress_ratio < BLOCKED_MIN_PROGRESS_RATIO:
        return (
            False,
            f"robot did not advance toward obstacle "
            f"(progress_ratio={progress_ratio:.2f} < {BLOCKED_MIN_PROGRESS_RATIO})",
        )
    return (
        True,
        f"blocked before obstacle plane (crossed_plane=False, progress_ratio={progress_ratio:.2f}, "
        f"min_dist={min_dist_to_obstacle_cm:.0f} cm, max_obj_coll={max_object_collision})",
    )


def reapply_demo_cube_blocking_modes(
    ucv: UnrealCV,
    demo_registry: dict[str, dict],
) -> None:
    """通過試験前にデモ立方体へ SetBlocking を再適用（スポーン直後の取りこぼし対策）。"""
    for demo_id, meta in sorted(demo_registry.items()):
        if not actor_exists(ucv, demo_id):
            print(f"[PassageTest] warn: {demo_id} missing before blocking reapply")
            continue
        blocking = meta.get("mode") != "translucent" and meta.get("blocking") is not False
        ok = set_cube_blocking_mode(ucv, demo_id, blocking=blocking, apply_tint=blocking)
        print(f"[PassageTest] reapply SetBlocking {blocking} on {demo_id}: {'ok' if ok else 'FAIL'}")


def passage_segment_for_demo(
    map_xy_m: Tuple[float, float],
    *,
    expects_pass_through: bool,
    segment_axis: str = "auto",
) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
    """障害物中心を貫く start / goal [cm] と障害物中心 [cm]。

    segment_axis:
      - auto: 通過可は南北（y）、BLOCK は東西（x）
      - through: 常に南北（SetBlocking 切替試験で同一経路を使う）
    """
    ox, oy = map_xy_m_to_world_cm(map_xy_m)
    offset_cm = PASSAGE_AXIS_OFFSET_M * 100.0
    obstacle_xy = (ox, oy)
    use_through_axis = (
        segment_axis == "through"
        or (segment_axis == "auto" and expects_pass_through)
    )
    if use_through_axis:
        start_xy = (ox, oy - offset_cm)
        goal_xy = (ox, oy + offset_cm)
    else:
        start_xy = (ox - offset_cm, oy)
        goal_xy = (ox + offset_cm, oy)
    return start_xy, goal_xy, obstacle_xy


def place_robot_and_face(
    ucv: UnrealCV,
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
) -> None:
    """SpotDog を start に置き、goal 方向を向かせる。"""
    if not actor_exists(ucv, ROBOT_ACTOR_NAME):
        raise RuntimeError(f"{ROBOT_ACTOR_NAME!r} is not spawned")

    ucv.set_physics(ROBOT_ACTOR_NAME, False)
    ucv.set_movable(ROBOT_ACTOR_NAME, True)
    ucv.set_collision(ROBOT_ACTOR_NAME, True)
    ucv.set_location(
        [start_xy[0], start_xy[1], ROBOT_SPAWN_Z_CM],
        ROBOT_ACTOR_NAME,
    )
    ucv.enable_controller(ROBOT_ACTOR_NAME, True)
    time.sleep(PHYSICS_ENABLE_DELAY_S)

    target_yaw = _yaw_toward_deg(start_xy, goal_xy)
    current_yaw = _robot_yaw_deg(ucv)
    angle_diff = _normalize_angle_deg(target_yaw - current_yaw)
    if abs(angle_diff) > 5.0:
        clockwise = 1 if angle_diff < 0.0 else -1
        rotate_s = min(1.2, max(0.35, abs(angle_diff) / 90.0))
        ucv.dog_rotate(ROBOT_ACTOR_NAME, [rotate_s, abs(angle_diff), clockwise])
        time.sleep(rotate_s + 0.05)


def drive_robot_through_segment(
    ucv: UnrealCV,
    start_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    obstacle_xy: Tuple[float, float],
) -> Tuple[Tuple[float, float], float, int]:
    """goal へ前進し、最終位置・障害物への最小距離・最大 ObjectCollision を返す。"""
    place_robot_and_face(ucv, start_xy, goal_xy)

    min_dist = float("inf")
    max_obj_coll = 0
    elapsed = 0.0

    while elapsed < PASSAGE_MAX_MOVE_S:
        pos = _robot_xy_cm(ucv)
        min_dist = min(min_dist, _dist_xy_cm(pos, obstacle_xy))
        counts = parse_collision_counts(ucv.get_collision_num(ROBOT_ACTOR_NAME))
        max_obj_coll = max(max_obj_coll, max_object_collision_count(counts))

        if _dist_xy_cm(pos, goal_xy) <= PASSAGE_ARRIVE_CM:
            break

        ucv.dog_move(
            ROBOT_ACTOR_NAME,
            [PASSAGE_ROBOT_SPEED, PASSAGE_MOVE_SLICE_S, 0],
        )
        elapsed += PASSAGE_MOVE_SLICE_S

    final_xy = _robot_xy_cm(ucv)
    min_dist = min(min_dist, _dist_xy_cm(final_xy, obstacle_xy))
    counts = parse_collision_counts(ucv.get_collision_num(ROBOT_ACTOR_NAME))
    max_obj_coll = max(max_obj_coll, max_object_collision_count(counts))

    if min_dist == float("inf"):
        min_dist = _dist_xy_cm(final_xy, obstacle_xy)
    return final_xy, min_dist, max_obj_coll


def run_demo_passage_test(
    ucv: UnrealCV,
    demo_id: str,
    meta: dict,
    *,
    segment_axis: str = "auto",
) -> PassageTrialVerdict:
    """1 個のデモ立方体に対する通過試験。"""
    expects_pass = meta.get("mode") == "translucent" or meta.get("blocking") is False
    map_xy = meta.get("map_xy_m")
    if map_xy is None:
        return PassageTrialVerdict(
            demo_id,
            expects_pass,
            False,
            "missing map_xy_m in registry",
            0.0,
            0.0,
            float("inf"),
            0,
            False,
        )

    start_xy, goal_xy, obstacle_xy = passage_segment_for_demo(
        tuple(map_xy),
        expects_pass_through=expects_pass,
        segment_axis=segment_axis,
    )
    goal_distance_cm = _dist_xy_cm(start_xy, goal_xy)

    print(
        f"[PassageTest] {demo_id} expect={'PASS' if expects_pass else 'BLOCK'} "
        f"start={_fmt_xyz((start_xy[0], start_xy[1], 0))} "
        f"goal={_fmt_xyz((goal_xy[0], goal_xy[1], 0))} "
        f"obstacle={_fmt_xyz((obstacle_xy[0], obstacle_xy[1], 0))}"
    )

    final_xy, min_dist_cm, max_obj_coll = drive_robot_through_segment(
        ucv,
        start_xy,
        goal_xy,
        obstacle_xy,
    )
    progress_cm = passage_progress_along_segment(start_xy, goal_xy, final_xy)
    crossed = crossed_obstacle_plane(start_xy, goal_xy, obstacle_xy, final_xy)

    ok, message = judge_passage_trial(
        expects_pass_through=expects_pass,
        goal_distance_cm=goal_distance_cm,
        progress_cm=progress_cm,
        min_dist_to_obstacle_cm=min_dist_cm,
        max_object_collision=max_obj_coll,
        crossed_obstacle=crossed,
    )

    verdict = PassageTrialVerdict(
        demo_id=demo_id,
        expects_pass_through=expects_pass,
        passed=ok,
        message=message,
        progress_cm=progress_cm,
        goal_distance_cm=goal_distance_cm,
        min_dist_obstacle_cm=min_dist_cm,
        max_object_collision=max_obj_coll,
        crossed_obstacle=crossed,
    )
    status = "PASS" if ok else "FAIL"
    print(
        f"[PassageTest] {demo_id} {status}: {message} | "
        f"final=({final_xy[0]:.0f},{final_xy[1]:.0f}) "
        f"progress={progress_cm:.0f}/{goal_distance_cm:.0f} cm "
        f"crossed_plane={crossed} min_dist={min_dist_cm:.0f} cm "
        f"max_obj_coll={max_obj_coll}"
    )
    return verdict


def run_demo_passage_test_with_blocking(
    ucv: UnrealCV,
    demo_id: str,
    meta: dict,
    *,
    blocking: bool,
    segment_axis: str = "auto",
) -> PassageTrialVerdict:
    """指定した SetBlocking 状態を適用してから通過試験を実行する。"""
    label = "solid(blocking)" if blocking else "translucent(pass-through)"
    print(f"[PassageTest] {demo_id} apply SetBlocking {blocking} ({label}) ...")
    ok_apply = set_cube_blocking_mode(ucv, demo_id, blocking=blocking, apply_tint=blocking)
    if not ok_apply:
        print(f"[PassageTest] warn: SetBlocking {blocking} apply failed for {demo_id!r}")
    time.sleep(PHYSICS_ENABLE_DELAY_S)
    trial_meta = {
        **meta,
        "blocking": blocking,
        "mode": "solid" if blocking else "translucent",
    }
    return run_demo_passage_test(
        ucv, demo_id, trial_meta, segment_axis=segment_axis
    )


def spawn_single_toggle_test_cube(ucv: UnrealCV) -> dict[str, dict]:
    """通過切替試験用: TransparentCube を 1 個だけ床上にスポーン（初期は OFF / 透過）。"""
    name = SINGLE_TOGGLE_CUBE_NAME
    map_xy = (SINGLE_TOGGLE_MAP_X_M, SINGLE_TOGGLE_MAP_Y_M)
    print(
        f"[SingleCube] spawn 1x {CUBE_BP} as {name!r} @ map={map_xy} m "
        f"(on floor, initial SetBlocking False)"
    )
    ok, loc = spawn_transparent_cube_on_floor(
        ucv, name, map_xy, blocking=False, use_physics_drop=DEMO_CUBE_PHYSICS_DROP
    )
    if not ok:
        print(f"[SingleCube] spawn failed for {name!r}")
        return {}
    print(f"[SingleCube] placed at {_fmt_xyz(loc)}")
    registry = {
        name: {
            "map_xy_m": map_xy,
            "world_cm": loc,
            "blocking": False,
            "mode": "translucent",
        }
    }
    print(f"[SingleCube] ready — use vbp {name} SetBlocking True/False to toggle ON/OFF")
    return registry


def run_blocking_toggle_passage_tests(
    ucv: UnrealCV,
    cube_id: str,
    meta: dict,
    *,
    phases: Optional[Tuple[Tuple[bool, str], ...]] = None,
) -> bool:
    """同一立方体で SetBlocking OFF/ON を切り替えながら Robot 通過試験を行う。

    blocking=False → OFF（透過・通過可）→ PASS 期待
    blocking=True  → ON（実体・非通過）→ BLOCK 期待
    """
    if phases is None:
        phases = (
            (False, "OFF (SetBlocking False → pass-through)"),
            (True, "ON (SetBlocking True → blocked)"),
            (False, "OFF again (SetBlocking False → pass-through)"),
        )
    print(
        f"[PassageTest] Single-object toggle on {cube_id!r}: "
        + " → ".join(label for _, label in phases)
    )
    verdicts: List[PassageTrialVerdict] = []
    for blocking, phase_label in phases:
        print(f"[PassageTest] --- {phase_label} ---")
        verdicts.append(
            run_demo_passage_test_with_blocking(
                ucv,
                cube_id,
                meta,
                blocking=blocking,
                segment_axis="through",
            )
        )

    passed_n = sum(1 for v in verdicts if v.passed)
    total = len(verdicts)
    all_ok = passed_n == total
    print(
        f"[PassageTest] Toggle SUMMARY {passed_n}/{total} "
        f"{'ALL PASS' if all_ok else 'SOME FAILED'} ({cube_id})"
    )
    if not all_ok:
        for v in verdicts:
            if not v.passed:
                print(
                    f"  FAILED {v.demo_id} "
                    f"SetBlocking={'True' if not v.expects_pass_through else 'False'} "
                    f"expect={'PASS' if v.expects_pass_through else 'BLOCK'}: {v.message}"
                )
    return all_ok


def run_translucent_blocking_toggle_passage_tests(
    ucv: UnrealCV,
    demo_id: str,
    meta: dict,
) -> bool:
    """同一 TransparentCube で透過↔実体を切り替え、Robot 通過試験で両モードを検証。"""
    return run_blocking_toggle_passage_tests(ucv, demo_id, meta)


def run_single_cube_toggle_passage_suite(ucv: UnrealCV) -> bool:
    """床 + 立方体 1 個 + SpotDog で SetBlocking OFF/ON 切替通過試験のみ実行。"""
    if not actor_exists(ucv, ROBOT_ACTOR_NAME):
        print("[SingleCube] FAIL: robot not spawned — spawn_robot() first")
        return False

    registry = spawn_single_toggle_test_cube(ucv)
    if not registry:
        return False

    cube_id = SINGLE_TOGGLE_CUBE_NAME
    meta = registry[cube_id]
    return run_blocking_toggle_passage_tests(ucv, cube_id, meta)


def run_all_demo_passage_tests(
    ucv: UnrealCV,
    demo_registry: dict[str, dict],
) -> bool:
    """全デモ立方体の通過試験。全件 PASS なら True。"""
    if not demo_registry:
        print("[PassageTest] skip: empty demo_registry")
        return True

    if not actor_exists(ucv, ROBOT_ACTOR_NAME):
        print("[PassageTest] FAIL: robot not spawned")
        return False

    reapply_demo_cube_blocking_modes(ucv, demo_registry)

    verdicts: List[PassageTrialVerdict] = []
    for demo_id in sorted(demo_registry.keys()):
        verdicts.append(run_demo_passage_test(ucv, demo_id, demo_registry[demo_id]))

    passed_n = sum(1 for v in verdicts if v.passed)
    total = len(verdicts)
    all_ok = passed_n == total
    print(
        f"[PassageTest] SUMMARY {passed_n}/{total} "
        f"{'ALL PASS' if all_ok else 'SOME FAILED'}"
    )
    if not all_ok:
        for v in verdicts:
            if not v.passed:
                print(
                    f"  FAILED {v.demo_id} expect={'PASS' if v.expects_pass_through else 'BLOCK'}: "
                    f"{v.message}"
                )
        solid_crossed = [
            v.demo_id
            for v in verdicts
            if not v.expects_pass_through and v.crossed_obstacle
        ]
        if solid_crossed:
            print(
                "[PassageTest] HINT: solid cube(s) "
                f"{solid_crossed} were crossed with max_obj_coll=0 — "
                "BP_TransparentCube SetBlocking(True) is not enabling collision in UE. "
                "Fix in Editor (D-3b): Branch True → Query and Physics on mesh, "
                "repackage pakchunk9002, restart SimWorld."
            )

    if RUN_TRANSLUCENT_TOGGLE_PASSAGE_TESTS:
        toggle_targets = [
            (demo_id, demo_registry[demo_id])
            for demo_id in sorted(demo_registry.keys())
            if demo_registry[demo_id].get("mode") == "translucent"
            or demo_registry[demo_id].get("blocking") is False
        ]
        if not toggle_targets:
            print("[PassageTest] toggle skip: no translucent demo cube in registry")
        else:
            for demo_id, meta in toggle_targets:
                toggle_ok = run_translucent_blocking_toggle_passage_tests(
                    ucv, demo_id, meta
                )
                all_ok = all_ok and toggle_ok

    return all_ok


def grid_env_extra_cleanup_actor_names(
    *,
    extra_ids: Optional[Iterable[str]] = None,
) -> Tuple[str, ...]:
    """格子・デモ registry に含まれないが grid_env_hri でスポーンし得る Actor 名。"""
    names: List[str] = list(GRID_ENV_EXTRA_CLEANUP_ACTORS)
    if extra_ids is not None:
        names.extend(extra_ids)
    # 重複除去（順序維持）
    seen: set[str] = set()
    unique: List[str] = []
    for name in names:
        if name and name not in seen:
            seen.add(name)
            unique.append(name)
    return tuple(unique)


def cleanup_spawned(
    ucv: UnrealCV,
    cube_ids: Iterable[str],
    *,
    marker_ids: Optional[Iterable[str]] = None,
    extra_ids: Optional[Iterable[str]] = None,
) -> None:
    """床・Robot・格子箱・デモ箱・通過試験用単体箱などを削除。"""
    cube_set = {cid for cid in cube_ids}
    marker_set = {mid for mid in (marker_ids or ())}
    extras = grid_env_extra_cleanup_actor_names(extra_ids=extra_ids)

    destroy_if_exists(ucv, FLOOR_ACTOR_NAME)
    destroy_if_exists(ucv, ROBOT_ACTOR_NAME)
    for cid in cube_set:
        destroy_if_exists(ucv, cid)
    for mid in marker_set:
        destroy_if_exists(ucv, mid)
    for eid in extras:
        if eid in cube_set or eid in marker_set:
            continue
        if actor_exists(ucv, eid):
            print(f"[Cleanup] extra actor {eid!r} (not in cube/marker registry)")
        destroy_if_exists(ucv, eid)
    try:
        ucv.clean_garbage()
    except Exception:
        pass


# ==============================================================
# メイン
# ==============================================================

def main() -> None:
    print(
        f"[GridEnvHRI] map={ROOT.name}, GRID_N={GRID_N}, "
        f"floor_top_z={FLOOR_TOP_Z_CM} cm, origin={MAP_ORIGIN_XY_CM}"
    )
    print(
        "  Launch: SimWorld.exe -windowed -log /Game/Maps/empty.umap"
    )

    ucv, communicator = ensure_connection()

    if not spawn_fixed_floor(ucv):
        return

    cube_registry = spawn_cubes(ucv, GRID_N)
    marker_registry: dict[str, dict] = {}
    if SPAWN_DEMO_MODE_CUBES:
        marker_registry = spawn_demo_mode_cubes(ucv)
    human_name = spawn_humanoid(communicator, ucv)
    robot_ok = spawn_robot(ucv)

    settle_after_cube_spawn_if_needed(
        demo_physics_drop=DEMO_CUBE_PHYSICS_DROP and bool(marker_registry)
    )

    report_spawn_state(
        ucv, cube_registry, human_name, marker_registry=marker_registry or None
    )

    if RUN_DEMO_PASSAGE_TESTS and robot_ok and marker_registry:
        passage_ok = run_all_demo_passage_tests(ucv, marker_registry)
        if not passage_ok:
            print("[PassageTest] collision behavior check FAILED — see logs above")

    print("[Done]")
    print(f"  floor: {FLOOR_ACTOR_NAME}")
    print(f"  cubes: {len(cube_registry)}")
    print(f"  demo_cubes: {len(marker_registry)}")
    print(f"  humanoid: {human_name}")
    print(f"  robot: {ROBOT_ACTOR_NAME if robot_ok else 'FAILED'}")
    print(
        f"  human map=({HUMAN_MAP_XY_M[0]}m,{HUMAN_MAP_XY_M[1]}m) "
        f"robot map=({ROBOT_MAP_XY_M[0]}m,{ROBOT_MAP_XY_M[1]}m)"
    )


if __name__ == "__main__":
    main()
