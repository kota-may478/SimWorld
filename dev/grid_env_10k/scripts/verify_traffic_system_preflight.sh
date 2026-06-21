#!/usr/bin/env bash
# WSL preflight: TrafficSystem files on disk (no UE Editor required).
set -euo pipefail

UE_CONTENT="${UE_CONTENT:-/mnt/c/UEProjects/SimWorld/Content}"
MIN_TRAFFIC_FILES=30

check_file() {
  local rel="$1"
  local min_bytes="$2"
  local path="${UE_CONTENT}/${rel}"
  if [ ! -f "$path" ]; then
    echo "FAIL missing: $rel"
    return 1
  fi
  local size
  size=$(stat -c%s "$path")
  if [ "$size" -lt "$min_bytes" ]; then
    echo "FAIL too small ($size B): $rel"
    return 1
  fi
  echo "OK $rel ($size B)"
}

ok=true
check_file "Agent/BP_AgentBase.uasset" 100000 || ok=false
check_file "TrafficSystem/Pedestrian/Base_User_Agent.uasset" 50000 || ok=false
check_file "TrafficSystem/Pedestrian/Base_Pedestrian.uasset" 50000 || ok=false
check_file "TrafficSystem/Pedestrian/input/IMC_Demo.uasset" 1000 || ok=false

count=$(find "${UE_CONTENT}/TrafficSystem" -type f 2>/dev/null | wc -l)
if [ "$count" -lt "$MIN_TRAFFIC_FILES" ]; then
  echo "FAIL TrafficSystem file count=$count (need >=$MIN_TRAFFIC_FILES)"
  ok=false
else
  echo "OK TrafficSystem file count=$count"
fi

if [ -d "${UE_CONTENT}/Human_Avatar" ]; then
  echo "WARN Human_Avatar still present — run disable_human_avatar_editor.ps1 (Editor closed)"
fi

if $ok; then
  echo "PREFLIGHT OK — open Editor, PIE Play, then run_humanoid_spawn_test.py"
  exit 0
fi
echo "PREFLIGHT FAIL — close Editor, run install_traffic_system_editor.sh"
exit 1
