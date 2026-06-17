#!/usr/bin/env python3
"""Level PIE destroy-then-spawn helpers (mitigate UE Editor crashes)."""

from __future__ import annotations

import os
import time
from typing import List, Tuple

import grid_env_hri_simulation as geh
from pie_safety import (
    DESTROY_BETWEEN_S,
    PieSessionLost,
    cooldown_before_spawn_batch,
    require_live_ucv,
    settle_after_destroy_batch,
    tick_settle,
)

RECONNECT_BACKOFF_S = 5.0
RECONNECT_INITIAL_WAIT_S = 12.0
RECONNECT_PROBE_TIMEOUT_S = 90.0
MAX_SPAWN_ATTEMPTS = 4


def ping_ok(ucv) -> bool:
    try:
        return geh._ping_ucv(ucv)  # noqa: SLF001
    except Exception:
        return False


def recover_ucv(ucv, *, reason: str):
    """Reconnect after UE drops UnrealCV during destroy/spawn (connection reset)."""
    import ue_client_guard

    print(f"[PieSpawn] reconnect ({reason}) ...")
    geh.release_connection(ucv)
    deadline = time.time() + RECONNECT_PROBE_TIMEOUT_S
    last_err: str | None = None
    attempt = 0
    while time.time() < deadline:
        attempt += 1
        ue_client_guard.wait_for_tcp_port_idle(timeout_s=10.0, except_pid=os.getpid())
        wait_s = RECONNECT_INITIAL_WAIT_S if attempt == 1 else RECONNECT_BACKOFF_S
        print(f"[PieSpawn] reconnect probe {attempt} (wait {wait_s:.0f}s) ...")
        time.sleep(wait_s)
        try:
            ucv, _ = ue_client_guard.prepare_ue_connection(
                force_new=True,
                kill_stale_clients=False,
                acquire_lock=False,
                wait_port_idle=True,
            )
            if ping_ok(ucv):
                print("[PieSpawn] reconnect OK")
                return ucv
            last_err = "connected but ping failed"
        except (ConnectionError, OSError, RuntimeError) as exc:
            last_err = str(exc).split("\n")[0]
            print(f"[PieSpawn] reconnect attempt {attempt} failed: {last_err}")
    raise PieSessionLost(
        f"UnrealCV reconnect failed after {reason}"
        f"{f' ({last_err})' if last_err else ''}. "
        "UE Editor may have crashed or PIE was stopped."
    )


def ensure_live_or_reconnect(ucv, *, reason: str):
    if ping_ok(ucv):
        return ucv
    return recover_ucv(ucv, reason=reason)


def spawn_bp_resilient(
    ucv,
    bp_path: str,
    name: str,
    *,
    timeout_s: float = 120.0,
) -> Tuple[bool, object]:
    """spawn_bp with reconnect retries (destroy→spawn often resets :9000 once)."""
    for attempt in range(1, MAX_SPAWN_ATTEMPTS + 1):
        ucv = ensure_live_or_reconnect(ucv, reason=f"spawn {name!r} attempt {attempt}")
        if geh.spawn_bp(ucv, bp_path, name, timeout_s=timeout_s):
            return True, ucv
        print(f"[PieSpawn] spawn_bp retry {attempt}/{MAX_SPAWN_ATTEMPTS} for {name!r}")
        if ping_ok(ucv):
            tick_settle(ucv, settle_s=4.0, ticks=3)
            continue
        ucv = recover_ucv(ucv, reason=f"spawn_bp failed {name!r}")
    return False, ucv


def destroy_actor_level(ucv, name: str) -> Tuple[bool, object]:
    ucv = ensure_live_or_reconnect(ucv, reason=f"before destroy {name}")
    require_live_ucv(ucv, context=f"destroy {name}")
    if not geh.actor_exists(ucv, name):
        return True, ucv
    geh._ue_request(ucv, f"vset /object/{name}/destroy", timeout_s=30.0)  # noqa: SLF001
    gone = geh.wait_until_actor_gone(ucv, name, timeout_s=5.0)
    time.sleep(DESTROY_BETWEEN_S)
    tick_settle(ucv, settle_s=0.0, ticks=1)
    return gone, ucv


def destroy_by_prefix(ucv, prefix: str) -> Tuple[int, object]:
    names: List[str] = sorted(n for n in geh.actor_names(ucv) if n.startswith(prefix))
    removed = 0
    for name in names:
        gone, ucv = destroy_actor_level(ucv, name)
        if gone:
            removed += 1
    if removed:
        settle_after_destroy_batch(ucv)
        ucv = ensure_live_or_reconnect(ucv, reason=f"after {removed} destroy(s)")
        cooldown_before_spawn_batch(ucv, reason=f"after {removed} destroy(s)")
    return removed, ucv


def destroy_named(ucv, names: List[str]) -> Tuple[int, object]:
    removed = 0
    for name in sorted(set(names)):
        gone, ucv = destroy_actor_level(ucv, name)
        if gone:
            removed += 1
    if removed:
        settle_after_destroy_batch(ucv)
        ucv = ensure_live_or_reconnect(ucv, reason=f"after {removed} destroy(s)")
        cooldown_before_spawn_batch(ucv, reason=f"after {removed} destroy(s)")
    return removed, ucv


def require_live_or_raise(ucv, *, context: str) -> object:
    require_live_ucv(ucv, context=context)
    return ucv
