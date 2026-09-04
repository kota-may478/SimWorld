"""Parametric 枠組-style modules and board sockets."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from scene.geometry import ScaffoldGeom

N_WORKING_BAYS = 5
KINDS = frozenset({"frame", "board", "stair_tread"})


def module_kind(kind: str) -> str:
    if kind not in KINDS:
        raise ValueError(f"unknown module kind: {kind}")
    return kind


@dataclass(frozen=True)
class Module:
    module_id: str
    kind: str
    x_m: float
    y_m: float
    z_m: float
    yaw_deg: float = 0.0
    sx_m: float = 0.05
    sy_m: float = 2.4
    sz_m: float = 1.8


@dataclass(frozen=True)
class Socket:
    socket_id: str
    floor: int
    x_m: float
    y_m: float
    z_m: float
    filled: bool = False


@dataclass(frozen=True)
class ScaffoldSpec:
    n_bays: int
    bay_m: float
    modules: Tuple[Module, ...]
    sockets: Tuple[Socket, ...]

    def sockets_on_floor(self, floor: int) -> Tuple[Socket, ...]:
        return tuple(s for s in self.sockets if s.floor == floor)

    def next_empty_socket(self, floor: int) -> Optional[Socket]:
        for socket in self.sockets_on_floor(floor):
            if not socket.filled:
                return socket
        return None

    def with_placed(self, socket_id: str) -> "ScaffoldSpec":
        if not any(s.socket_id == socket_id for s in self.sockets):
            raise ValueError(f"unknown socket_id: {socket_id}")
        sockets = tuple(
            Socket(
                socket_id=s.socket_id,
                floor=s.floor,
                x_m=s.x_m,
                y_m=s.y_m,
                z_m=s.z_m,
                filled=True if s.socket_id == socket_id else s.filled,
            )
            for s in self.sockets
        )
        return ScaffoldSpec(self.n_bays, self.bay_m, self.modules, sockets)


def build_scaffold(geom: ScaffoldGeom) -> ScaffoldSpec:
    bay_m = geom.deck_length_m / N_WORKING_BAYS
    modules: list[Module] = []
    sockets: list[Socket] = []
    mid_y = geom.deck_width_m * 0.5

    for floor in range(1, geom.n_floors + 1):
        z = geom.floor_z_m(floor)
        for i in range(N_WORKING_BAYS + 1):
            x = i * bay_m
            modules.append(
                Module(
                    module_id=f"frame_f{floor}_{i}",
                    kind="frame",
                    x_m=x,
                    y_m=mid_y,
                    z_m=z,
                    sy_m=geom.deck_width_m,
                    sz_m=geom.lift_m,
                )
            )
        for i in range(N_WORKING_BAYS):
            x = (i + 0.5) * bay_m
            for row, y in enumerate((geom.deck_width_m * 0.25, geom.deck_width_m * 0.75)):
                sockets.append(
                    Socket(
                        socket_id=f"board_f{floor}_b{i}_r{row}",
                        floor=floor,
                        x_m=x,
                        y_m=y,
                        z_m=z,
                    )
                )
                modules.append(
                    Module(
                        module_id=f"boardslot_f{floor}_b{i}_r{row}",
                        kind="board",
                        x_m=x,
                        y_m=y,
                        z_m=z,
                        sx_m=bay_m * 0.95,
                        sy_m=geom.deck_width_m * 0.45,
                        sz_m=0.05,
                    )
                )

    treads_per_lift = 10
    rise = geom.lift_m / treads_per_lift
    for lift in range(geom.n_floors - 1):
        z0 = geom.floor_z_m(lift + 1)
        for t in range(treads_per_lift):
            frac = t / treads_per_lift
            # Switchback: out along +y then back along -y inside the stair bay.
            if lift % 2 == 0:
                y = frac * geom.deck_width_m
            else:
                y = (1.0 - frac) * geom.deck_width_m
            x = -geom.stair_bay_m * (0.25 + 0.5 * (lift % 2))
            modules.append(
                Module(
                    module_id=f"tread_L{lift}_{t}",
                    kind="stair_tread",
                    x_m=x,
                    y_m=y,
                    z_m=z0 + (t + 1) * rise,
                    sx_m=geom.stair_bay_m * 0.45,
                    sy_m=0.28,
                    sz_m=0.04,
                )
            )

    return ScaffoldSpec(
        n_bays=N_WORKING_BAYS,
        bay_m=bay_m,
        modules=tuple(modules),
        sockets=tuple(sockets),
    )
