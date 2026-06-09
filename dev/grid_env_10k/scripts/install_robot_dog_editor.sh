#!/usr/bin/env bash
# Install loose SpotDog dependencies into UE Editor Content (fallback path).
# Recommended: use install_simworld_runtime_paks_editor.sh + mount script instead.
set -euo pipefail

GYM_CONTENT="${GYM_CONTENT:-/mnt/c/SimWorldServer/_research/SimWorld-Studio/SimWorld-Studio-Minimal/gym_citynav/Content}"
UE_CONTENT="${UE_CONTENT:-/mnt/c/UEProjects/SimWorld/Content}"

if [ ! -d "$GYM_CONTENT" ]; then
  echo "gym_citynav Content not found: $GYM_CONTENT" >&2
  exit 1
fi

for dir in Agent "CityDatabase/blueprints" Robot_Dog; do
  src="${GYM_CONTENT}/${dir}"
  dst="${UE_CONTENT}/${dir}"
  if [ ! -d "$src" ]; then
    echo "Missing source: $src" >&2
    exit 1
  fi
  mkdir -p "$(dirname "$dst")"
  rm -rf "$dst"
  cp -a "$src" "$dst"
  echo "Installed $dst"
done

echo ""
echo "NOTE: BP_InteractableAssetBase is NOT in gym_citynav loose Content."
echo "      It lives in pakchunk1000 — use:"
echo "  bash dev/grid_env_10k/scripts/install_simworld_runtime_paks_editor.sh"
echo "  then mount_simworld_runtime_paks_editor.py in Editor"
echo ""
echo "Loose Robot_Dog still needs compile_robot_dog_editor.py after pak mount for parents."
