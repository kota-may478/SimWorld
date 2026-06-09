#!/usr/bin/env bash
# Install editor SpotDog parent assets without touching locked Content/Agent/.
# sudo は不要 — ロックは UE Editor がファイルを掴んでいるためです。
set -euo pipefail

GYM_CONTENT="${GYM_CONTENT:-/mnt/c/SimWorldServer/_research/SimWorld-Studio/SimWorld-Studio-Minimal/gym_citynav/Content}"
UE_CONTENT="${UE_CONTENT:-/mnt/c/UEProjects/SimWorld/Content}"

install_dir() {
  local rel="$1"
  local dst_name="${2:-$(basename "$rel")}"
  local src="${GYM_CONTENT}/${rel}"
  local dst="${UE_CONTENT}/${dst_name}"
  if [ ! -d "$src" ]; then
    echo "Missing: $src" >&2
    exit 1
  fi
  rm -rf "$dst"
  cp -a "$src" "$dst"
  echo "Installed $dst"
}

# Locked cooked files stay in Content/Agent/ — mount uses Agent_from_gym instead.
install_dir "Agent" "Agent_from_gym"

if [ ! -d "${UE_CONTENT}/CityDatabase/blueprints" ]; then
  install_dir "CityDatabase/blueprints" "CityDatabase/blueprints"
else
  echo "Skip CityDatabase/blueprints (already present)"
fi

cat <<'EOF'

Step A (Editor を開いたまま OK):
  Agent_from_gym をインストール済み

Step B (Editor を完全終了してから):
  bash dev/grid_env_10k/scripts/finalize_agent_for_editor.sh

Step C (Editor 再起動、PIE 前):
  1. create_interactable_stub_editor.py
  2. compile_robot_dog_editor.py
     ※ Content/Agent が cooked のままだと compile は安全に中止します

Step D: PIE on grid_100x100 -> WSL mount + patrol

EOF
