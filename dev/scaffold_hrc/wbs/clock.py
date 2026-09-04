"""Schedule-bound WBS clock. Spoken rules bind to the active zone only."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class WbsTask:
    task_id: str
    zone_id: str
    t_start: float
    t_end: float
    kind: str
    floor: int
    site: str = "A"


@dataclass(frozen=True)
class WbsClock:
    tasks: Tuple[WbsTask, ...]
    cursor: int = 0

    def current(self) -> WbsTask:
        return self.tasks[self.cursor]

    def is_active(self, task_id: str) -> bool:
        return self.current().task_id == task_id

    def active_zone_id(self) -> str:
        return self.current().zone_id

    def complete_current(self) -> "WbsClock":
        nxt = min(self.cursor + 1, len(self.tasks) - 1)
        if nxt == self.cursor:
            return self
        return WbsClock(self.tasks, cursor=nxt)

    def constrains_floor(self, floor: int) -> bool:
        return self.current().floor == floor

    def bind_utterance_zone(self, zone_id: str) -> WbsTask:
        task = self.current()
        if task.zone_id != zone_id:
            raise ValueError(
                f"utterance zone {zone_id!r} is not the active WBS zone {task.zone_id!r}"
            )
        return task


def build_stage1_wbs(*, window_s: float = 100.0) -> WbsClock:
    tasks: list[WbsTask] = []
    t = 0.0
    for floor in (1, 2, 3):
        zone = f"SCAFFOLD_A_F{floor}"
        for kind in ("supply", "erect"):
            tasks.append(
                WbsTask(
                    task_id=f"A_F{floor}_{kind}",
                    zone_id=zone,
                    t_start=t,
                    t_end=t + window_s,
                    kind=kind,
                    floor=floor,
                )
            )
            t += window_s
    return WbsClock(tasks=tuple(tasks), cursor=0)
