"""Shared Stage-1 mission protocol (UE is the field source of truth).

Kinematic oracle and UE condition-card replays must read these values so
dwell times, keep-out scope, and fixture counts stay identical.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MissionProtocol:
    sockets_per_floor: int = 4
    n_floors: int = 3
    truck_load_s: float = 10.0
    drop_place_s: float = 10.0
    erect_s: float = 10.0
    # Bare-ground full assembly: boards vs posts/braces/stairs/ledgers
    board_erect_s: float = 10.0
    structural_erect_s: float = 10.0
    staging_slots: int = 6
    erect_from_bare: bool = True
    corridor_vmax_mps: float = 1.2
    human_speed_mps: float = 1.2
    human_retreat_mps: float = 1.2
    assembler_pickup_s: float = 5.0
    keepout_agent: str = "assembler"
    yard_keepout: bool = False
    assembler_keepout_on_scaffold: bool = True


STAGE1_PROTOCOL = MissionProtocol()
