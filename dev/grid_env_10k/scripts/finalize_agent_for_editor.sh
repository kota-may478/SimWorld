#!/usr/bin/env bash
# Replace cooked Content/Agent with editor gym copy. UE Editor は必ず終了してから実行。
set -euo pipefail

UE_CONTENT="${UE_CONTENT:-/mnt/c/UEProjects/SimWorld/Content}"
SRC="${UE_CONTENT}/Agent_from_gym"
DST="${UE_CONTENT}/Agent"
BACKUP="${UE_CONTENT}/Agent_cooked_disabled"

if [ ! -f "${SRC}/BP_AgentBase.uasset" ]; then
  echo "Missing ${SRC}/BP_AgentBase.uasset — run prepare_spotdog_pie_assets.sh first." >&2
  exit 1
fi

size=$(stat -c%s "${SRC}/BP_AgentBase.uasset")
if [ "$size" -lt 100000 ]; then
  echo "Agent_from_gym looks too small (${size} bytes) — not editor assets." >&2
  exit 1
fi

if pgrep -fi "UnrealEditor" >/dev/null 2>&1; then
  echo "ERROR: Unreal Editor is still running. Close it completely, then re-run." >&2
  exit 1
fi

if [ -d "$BACKUP" ]; then
  rm -rf "$BACKUP"
fi
if [ -d "$DST" ]; then
  mv "$DST" "$BACKUP"
  echo "Backed up cooked Agent -> Agent_cooked_disabled"
fi

cp -a "$SRC" "$DST"
echo "Installed editor Agent -> ${DST}"
echo "OK. Restart UE Editor, then run create_interactable_stub + compile_robot_dog_editor."
