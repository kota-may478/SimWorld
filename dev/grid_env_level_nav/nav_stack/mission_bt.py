"""Lightweight mission BT (explicit state machine, no external library)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional, Protocol


class NodeStatus(Enum):
    RUNNING = auto()
    SUCCESS = auto()
    FAILURE = auto()


class MissionNode(Protocol):
    def tick(self) -> NodeStatus: ...


@dataclass
class SequenceNode:
    children: list[MissionNode]

    def tick(self) -> NodeStatus:
        while self.children:
            status = self.children[0].tick()
            if status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            if status == NodeStatus.FAILURE:
                return NodeStatus.FAILURE
            self.children.pop(0)
        return NodeStatus.SUCCESS


@dataclass
class CallableMissionNode:
    name: str
    fn: Callable[[], bool]

    def tick(self) -> NodeStatus:
        return NodeStatus.SUCCESS if self.fn() else NodeStatus.FAILURE


@dataclass
class MissionRunner:
    """Runs Leg1 → carry → Leg2 as a simple sequence."""

    leg1_fn: Callable[[], bool]
    carry_fn: Callable[[], bool]
    leg2_fn: Callable[[], bool]
    _root: Optional[SequenceNode] = None

    def __post_init__(self) -> None:
        self._root = SequenceNode(
            children=[
                CallableMissionNode("navigate_to_material", self.leg1_fn),
                CallableMissionNode("begin_carry", self.carry_fn),
                CallableMissionNode("deliver_to_humanoid", self.leg2_fn),
            ]
        )

    def tick(self) -> NodeStatus:
        if self._root is None:
            return NodeStatus.FAILURE
        return self._root.tick()

    def run_to_completion(self) -> bool:
        while True:
            status = self.tick()
            if status == NodeStatus.SUCCESS:
                return True
            if status == NodeStatus.FAILURE:
                return False
