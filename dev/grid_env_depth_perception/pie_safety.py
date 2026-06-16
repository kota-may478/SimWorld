#!/usr/bin/env python3
"""PIE safety helpers for /Game/Maps/Level (avoid UE Editor crashes).

Level PIE is fragile when:
- Many actor destroys are followed immediately by spawn_bp
- Pawn (SpotDog) is destroyed or hard-respawned during play
- clean_garbage is forced after pawn teardown
- UnrealCV is hammered while UE is still processing destroys

Patterns borrowed from grid_env_level_nav/spawn_construction_vol1_props_pie.py and
level_nav_robot.py (soft-reset only; never hard destroy SpotDog during tests).
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Optional, Tuple

_THIS_DIR = Path(__file__).resolve().parent
_ROOT = _THIS_DIR.parent.parent
_GEH_DIR = _ROOT / "dev" / "grid_env_hri"
if str(_GEH_DIR) not in sys.path:
    sys.path.insert(0, str(_GEH_DIR))

import grid_env_hri_simulation as geh  # noqa: E402

# Destroy batch on Level: longer than grid_env defaults.
POST_DESTROY_SETTLE_S = 6.0
DESTROY_BETWEEN_S = 0.55
SPAWN_SETTLE_S = 0.65
BATCH_PAUSE_EVERY = 2
BATCH_PAUSE_S = 2.5
POST_TELEPORT_SETTLE_S = 0.45
PERCEPTION_MIN_INTERVAL_S = 0.45
MAX_LEG_DURATION_S = 240.0
NAV_MAX_STEPS_DEFAULT = 280


class PieSessionLost(RuntimeError):
    """Raised when UnrealCV connection is gone and UE likely crashed or PIE stopped."""


def ping_ok(ucv) -> bool:
    try:
        return geh._ping_ucv(ucv)  # noqa: SLF001
    except Exception:
        return False


def require_live_ucv(ucv, *, context: str):
    """Fail fast instead of reconnecting during spawn/destroy (reconnect hides UE crash)."""
    if not ping_ok(ucv):
        raise PieSessionLost(
            f"UnrealCV connection lost during {context}. "
            "UE Editor may have crashed or PIE was stopped. "
            "Restart PIE on /Game/Maps/Level before re-running."
        )
    return ucv


def tick_settle(ucv, *, settle_s: float = 0.0, ticks: int = 2) -> None:
    """Let UE finish pending work without forcing GC."""
    for _ in range(max(1, ticks)):
        try:
            ucv.tick()
        except Exception:
            break
        time.sleep(0.12)
    if settle_s > 0:
        time.sleep(settle_s)


def settle_after_destroy_batch(ucv) -> None:
    """Post-destroy idle — never run clean_garbage here on Level."""
    tick_settle(ucv, settle_s=POST_DESTROY_SETTLE_S, ticks=3)


def pause_between_spawns(spawned_count: int) -> bool:
    return spawned_count > 0 and spawned_count % BATCH_PAUSE_EVERY == 0


def batch_pause(ucv, *, reason: str) -> None:
    print(f"[PieSafety] batch pause ({reason}) {BATCH_PAUSE_S}s ...")
    require_live_ucv(ucv, context=f"batch pause ({reason})")
    tick_settle(ucv, settle_s=BATCH_PAUSE_S, ticks=2)


def soft_teleport_robot(
    ucv,
    robot_name: str,
    world_xyz: Tuple[float, float, float],
    yaw_deg: float,
) -> None:
    """Teleport pawn without destroy — disable controller first (Level-safe)."""
    require_live_ucv(ucv, context=f"teleport {robot_name}")
    try:
        ucv.enable_controller(robot_name, False)
    except Exception:
        pass
    ucv.set_physics(robot_name, False)
    ucv.set_movable(robot_name, True)
    ucv.set_location(list(world_xyz), robot_name)
    ucv.set_orientation((0.0, yaw_deg, 0.0), robot_name)
    ucv.set_collision(robot_name, True)
    tick_settle(ucv, settle_s=POST_TELEPORT_SETTLE_S, ticks=1)
    ucv.enable_controller(robot_name, True)
    time.sleep(geh.PHYSICS_ENABLE_DELAY_S)
