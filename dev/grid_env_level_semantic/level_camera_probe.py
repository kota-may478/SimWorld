#!/usr/bin/env python3
"""UnrealCV helpers: list camera transforms on /Game/Maps/Level (PIE)."""

from __future__ import annotations

import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import List, Optional, Tuple

Vec3 = Tuple[float, float, float]


def _find_project_root() -> Path:
    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / "setup.py").exists() and (candidate / "simworld").is_dir():
            return candidate
    return here.parent.parent


ROOT = _find_project_root()
GEH_DIR = ROOT / "dev" / "grid_env_hri"
G10K_DIR = ROOT / "dev" / "grid_env_10k"
THIS_DIR = Path(__file__).resolve().parent
for p in (ROOT, GEH_DIR, G10K_DIR, THIS_DIR):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import grid_env_10k as g10k  # noqa: E402
import grid_env_hri_simulation as geh  # noqa: E402
from simworld.communicator.unrealcv import UnrealCV  # noqa: E402

LEVEL_MAP_PATH = "/Game/Maps/Level"
DEFAULT_EXPORT_PATH = THIS_DIR / ".level_camera_snapshot.json"


@dataclass(frozen=True)
class CameraSnapshot:
    camera_id: int
    name: str
    location_cm: Vec3
    rotation_deg: Vec3


def parse_vector3(raw_value) -> Vec3:
    if isinstance(raw_value, str):
        tokens = raw_value.replace(",", " ").split()
        return (float(tokens[0]), float(tokens[1]), float(tokens[2]))
    return (float(raw_value[0]), float(raw_value[1]), float(raw_value[2]))


def vec3_to_m(v: Vec3) -> Vec3:
    return (v[0] / 100.0, v[1] / 100.0, v[2] / 100.0)


def ensure_connection() -> Tuple[UnrealCV, object]:
    return g10k.ensure_connection()


def wait_for_ue_port(timeout_s: float = 120.0) -> bool:
    """Wait until any WSL-reachable UnrealCV endpoint answers (same probes as grid_env_hri)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for host in geh._ue_host_candidates():
            if geh._probe_unrealcv_endpoint(host, geh.UE_PORT, timeout_s=2.0):
                return True
        time.sleep(2.0)
    return False


def list_camera_snapshots(ucv: UnrealCV) -> List[CameraSnapshot]:
    raw = ucv.get_cameras()
    names = str(raw).split() if raw else []
    out: List[CameraSnapshot] = []
    for camera_id, name in enumerate(names):
        try:
            loc = parse_vector3(ucv.get_camera_location(camera_id))
            rot = parse_vector3(ucv.get_camera_rotation(camera_id))
        except Exception as exc:
            print(f"[Camera] id={camera_id} name={name!r}: transform failed ({exc})")
            continue
        out.append(
            CameraSnapshot(
                camera_id=camera_id,
                name=name,
                location_cm=loc,
                rotation_deg=rot,
            )
        )
    return out


def print_camera_report(snapshots: List[CameraSnapshot]) -> None:
    print(f"[Camera] count={len(snapshots)}")
    for cam in snapshots:
        x_m, y_m, z_m = vec3_to_m(cam.location_cm)
        print(
            f"  id={cam.camera_id:2d} {cam.name:24s} "
            f"loc_cm=({cam.location_cm[0]:10.2f}, {cam.location_cm[1]:10.2f}, {cam.location_cm[2]:10.2f}) "
            f"loc_m=({x_m:8.3f}, {y_m:8.3f}, {z_m:8.3f}) "
            f"rot=({cam.rotation_deg[0]:7.2f}, {cam.rotation_deg[1]:7.2f}, {cam.rotation_deg[2]:7.2f})"
        )


def export_camera_snapshots(
    snapshots: List[CameraSnapshot],
    path: Path = DEFAULT_EXPORT_PATH,
) -> Path:
    payload = {
        "map": LEVEL_MAP_PATH,
        "unit": "cm",
        "cameras": [asdict(cam) for cam in snapshots],
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"[Camera] exported {path}")
    return path


def format_corner_handoff(cam: CameraSnapshot) -> str:
    """Copy-paste friendly snippet for the two corner coordinates you will send later."""
    x_m, y_m, z_m = vec3_to_m(cam.location_cm)
    return (
        f"corner_xy_m=({x_m:.4f}, {y_m:.4f})  # camera id={cam.camera_id} {cam.name}\n"
        f"corner_z_cm={cam.location_cm[2]:.2f}  # sight height reference only"
    )
