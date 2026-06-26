"""Recovery behavior plugins (Nav2 behavior_server equivalent)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Protocol, Sequence

from nav_stack.nav_context import NavContext


@dataclass(frozen=True)
class RecoveryResult:
    success: bool
    message: str = ""


class RecoveryAction(Protocol):
    action_id: str

    def run(self, ctx: NavContext) -> RecoveryResult: ...


@dataclass
class RecoveryActionSpec:
    action_id: str
    runner: Callable[[NavContext], RecoveryResult]


class BehaviorServer:
    """Ordered recovery chain (backup → replan → spin → clear L2 → wait)."""

    def __init__(self, actions: Sequence[RecoveryActionSpec]) -> None:
        self._actions = list(actions)

    @property
    def action_ids(self) -> List[str]:
        return [action.action_id for action in self._actions]

    def run_chain(
        self,
        ctx: NavContext,
        *,
        stop_on_success: bool = True,
    ) -> Optional[RecoveryResult]:
        last: Optional[RecoveryResult] = None
        for action in self._actions:
            last = action.runner(ctx)
            if stop_on_success and last.success:
                return last
        return last


def default_recovery_chain(
    *,
    backup_fn: Callable[[NavContext], RecoveryResult],
    replan_fn: Callable[[NavContext], RecoveryResult],
    spin_backup_fn: Callable[[NavContext], RecoveryResult],
    clear_local_l2_fn: Callable[[NavContext], RecoveryResult],
) -> BehaviorServer:
    return BehaviorServer(
        [
            RecoveryActionSpec("backup", backup_fn),
            RecoveryActionSpec("replan", replan_fn),
            RecoveryActionSpec("spin_backup", spin_backup_fn),
            RecoveryActionSpec("clear_local_l2", clear_local_l2_fn),
        ]
    )
