#!/usr/bin/env bash
# Install TrafficSystem (Humanoid BPs) into UE Editor Content for PIE spawn.
set -euo pipefail

GYM_CONTENT="${GYM_CONTENT:-/mnt/c/SimWorldServer/_research/SimWorld-Studio/SimWorld-Studio-Minimal/gym_citynav/Content}"
UE_CONTENT="${UE_CONTENT:-/mnt/c/UEProjects/SimWorld/Content}"

if [ ! -d "$GYM_CONTENT" ]; then
  echo "gym_citynav Content not found: $GYM_CONTENT" >&2
  exit 1
fi

install_dir() {
  local rel="$1"
  local src="${GYM_CONTENT}/${rel}"
  local dst="${UE_CONTENT}/${rel}"
  if [ ! -d "$src" ]; then
    echo "Missing source: $src" >&2
    exit 1
  fi
  mkdir -p "$(dirname "$dst")"
  rm -rf "$dst"
  cp -a "$src" "$dst"
  echo "Installed $dst"
}

install_dir "TrafficSystem"

# Human_Avatar needs CitySampleCrowd (~6GB) + MetaHuman deps — off by default (PIE compile errors).
if [ "${INSTALL_HUMAN_AVATAR:-0}" = "1" ]; then
  install_dir "Human_Avatar"
  if [ "${INSTALL_CITY_SAMPLE_CROWD:-0}" = "1" ]; then
    install_dir "CitySampleCrowd"
  else
    echo "WARN: Human_Avatar without CitySampleCrowd often fails compile (BPI_Human / BP_Human_Base)."
    echo "      Set INSTALL_CITY_SAMPLE_CROWD=1 for full copy, or use TrafficSystem/Base_User_Agent only."
  fi
fi

cat <<'EOF'

TrafficSystem installed. Next:
  1. WSL: bash dev/grid_env_10k/scripts/verify_traffic_system_preflight.sh
  2. UE Editor open grid_100x100 -> PIE Play
     (optional) verify_traffic_system_editor.py — file/registry only, no BP load
  3. WSL: python dev/grid_env_10k/run_humanoid_spawn_test.py  (authoritative)

Do NOT run compile_traffic_system_editor.py or open TrafficSystem BPs in Blueprint
Editor unless spawn test fails — load_asset/compile can crash UE.

If Human_Avatar blocks PIE: disable_human_avatar_editor.ps1 (Editor closed).

EOF
