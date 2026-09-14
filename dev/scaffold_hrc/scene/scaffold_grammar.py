"""Parametric 枠組足場: 建地・布・交差筋違・布板ソケット.

Visual members follow Japanese framed scaffolding (ビティ / 建枠):
  posts (建地) on the two long edges only, so the 2.4 m deck stays walkable;
  ledgers/transoms (布 / 横架材) at floor level;
  X braces (交差筋違) in the long-face planes, not across the walkway;
  steel decks (アンチ) spawned separately in ue.placement.

Sources: 日工セック 枠組足場部材表 (建枠 900×1700 mm, スパン 1.8 m,
交差筋違, 鋼製布板). STAGE1 deck is 10×2.4 m so bays stay 2.0 m.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from scene.field import stair_post_poses, stair_tread_poses
from scene.geometry import ScaffoldGeom

N_WORKING_BAYS = 5
PIPE_M = 0.10
KINDS = frozenset(
    {"post", "ledger", "transom", "brace", "rail", "board", "stair_tread", "frame"}
)


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
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    sx_m: float = 0.05
    sy_m: float = 2.4
    sz_m: float = 1.8
    z_is_center: bool = False


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


def _pipe(module_id: str, kind: str, x: float, y: float, z: float, sx: float, sy: float, sz: float) -> Module:
    return Module(
        module_id=module_id,
        kind=kind,
        x_m=x,
        y_m=y,
        z_m=z,
        sx_m=sx,
        sy_m=sy,
        sz_m=sz,
    )


def _brace(
    module_id: str,
    x: float,
    y: float,
    z_center: float,
    length_m: float,
    roll_deg: float,
) -> Module:
    return Module(
        module_id=module_id,
        kind="brace",
        x_m=x,
        y_m=y,
        z_m=z_center,
        roll_deg=roll_deg,
        sx_m=length_m,
        sy_m=PIPE_M,
        sz_m=PIPE_M,
        z_is_center=True,
    )


def build_scaffold(geom: ScaffoldGeom) -> ScaffoldSpec:
    """Members sized for Spot carry + humanoid install.

    Order that stays physically valid:
      each lift: posts (建地) → X braces → upper ledgers/transoms (布)
      → boards (アンチ) on the deck at the top of this lift → zigzag AABB
      stair treads onto that deck, then the next lift's posts from that deck.
    """
    bay_m = geom.deck_length_m / N_WORKING_BAYS
    xs = tuple(i * bay_m for i in range(N_WORKING_BAYS + 1))
    ys = (0.0, geom.deck_width_m)
    n_lifts = geom.n_floors - 1
    brace_len = math.hypot(bay_m, geom.lift_m)
    brace_roll = math.degrees(math.atan2(geom.lift_m, bay_m))
    modules: list[Module] = []
    sockets: list[Socket] = []

    for lift in range(n_lifts):
        z0 = lift * geom.lift_m
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                modules.append(
                    _pipe(
                        f"post_{i}_{j}_L{lift}",
                        "post",
                        x,
                        y,
                        z0,
                        PIPE_M,
                        PIPE_M,
                        geom.lift_m,
                    )
                )
        for j, y in enumerate(ys):
            modules.append(
                _pipe(
                    f"post_stair_{j}_L{lift}",
                    "post",
                    -geom.stair_bay_m,
                    y,
                    z0,
                    PIPE_M,
                    PIPE_M,
                    geom.lift_m,
                )
            )

    for pose in stair_post_poses(
        lift_m=geom.lift_m,
        deck_width_m=geom.deck_width_m,
        n_lifts=n_lifts,
    ):
        modules.append(
            _pipe(
                f"post_ramp_l{pose.lane}{pose.side}{pose.station}_L{pose.lift}",
                "post",
                pose.x_m,
                pose.y_m,
                pose.z_m,
                PIPE_M,
                PIPE_M,
                pose.sz_m,
            )
        )

    for floor in range(1, geom.n_floors + 1):
        z = geom.floor_z_m(floor)
        for j, y in enumerate(ys):
            for i in range(N_WORKING_BAYS):
                x = (i + 0.5) * bay_m
                modules.append(
                    _pipe(
                        f"ledger_f{floor}_{j}_b{i}",
                        "ledger",
                        x,
                        y,
                        z,
                        bay_m,
                        PIPE_M,
                        PIPE_M,
                    )
                )
        if floor >= 2:
            for i, x in enumerate(xs):
                if x <= 1e-9:
                    continue
                modules.append(
                    _pipe(
                        f"transom_f{floor}_{i}",
                        "transom",
                        x,
                        geom.deck_width_m * 0.5,
                        z,
                        PIPE_M,
                        geom.deck_width_m,
                        PIPE_M,
                    )
                )
        for i in range(N_WORKING_BAYS):
            x = (i + 0.5) * bay_m
            for row, y_board in enumerate(
                (geom.deck_width_m * 0.25, geom.deck_width_m * 0.75)
            ):
                sockets.append(
                    Socket(
                        socket_id=f"board_f{floor}_b{i}_r{row}",
                        floor=floor,
                        x_m=x,
                        y_m=y_board,
                        z_m=z,
                    )
                )
                modules.append(
                    Module(
                        module_id=f"boardslot_f{floor}_b{i}_r{row}",
                        kind="board",
                        x_m=x,
                        y_m=y_board,
                        z_m=z,
                        sx_m=bay_m * 0.95,
                        sy_m=geom.deck_width_m * 0.45,
                        sz_m=0.05,
                    )
                )

    for lift in range(n_lifts):
        z0 = lift * geom.lift_m
        z_mid = z0 + geom.lift_m * 0.5
        for i in range(N_WORKING_BAYS):
            x_mid = (i + 0.5) * bay_m
            for j, y in enumerate(ys):
                modules.append(
                    _brace(
                        f"brace_f{lift}_b{i}_{j}a",
                        x_mid,
                        y,
                        z_mid,
                        brace_len,
                        brace_roll,
                    )
                )
                modules.append(
                    _brace(
                        f"brace_f{lift}_b{i}_{j}b",
                        x_mid,
                        y,
                        z_mid,
                        brace_len,
                        -brace_roll,
                    )
                )

    for pose in stair_tread_poses(
        lift_m=geom.lift_m,
        deck_width_m=geom.deck_width_m,
        n_lifts=n_lifts,
    ):
        modules.append(
            Module(
                module_id=f"tread_L{pose.lift}_{pose.step}",
                kind="stair_tread",
                x_m=pose.x_m,
                y_m=pose.y_m,
                z_m=pose.z_bottom_m,
                sx_m=pose.sx_m,
                sy_m=pose.sy_m,
                sz_m=pose.sz_m,
            )
        )

    return ScaffoldSpec(
        n_bays=N_WORKING_BAYS,
        bay_m=bay_m,
        modules=tuple(modules),
        sockets=tuple(sockets),
    )
