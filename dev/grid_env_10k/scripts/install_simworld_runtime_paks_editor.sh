#!/usr/bin/env bash
# Link SimWorldServer runtime paks for UE Editor (Windows-visible hard links).
set -euo pipefail

PAK_SRC_WIN="${SIMWORLD_PAK_SRC_WIN:-C:/SimWorldServer/SimWorld/Content/Paks}"
PAK_SRC="/mnt/c/SimWorldServer/SimWorld/Content/Paks"
UE_PAKS_WIN="${UE_PAKS_WIN:-C:/UEProjects/SimWorld/Content/Paks}"

PAKS=(
  pakchunk1000-Windows.pak
  pakchunk0-Windows.pak
)

if [ ! -d "$PAK_SRC" ]; then
  echo "Pak source not found: $PAK_SRC" >&2
  exit 1
fi

mkdir -p "/mnt/c/UEProjects/SimWorld/Content/Paks"

for p in "${PAKS[@]}"; do
  src_win="${PAK_SRC_WIN}/${p}"
  dst_win="${UE_PAKS_WIN}/${p}"
  src_wsl="${PAK_SRC}/${p}"
  if [ ! -f "$src_wsl" ]; then
    echo "Missing pak: $src_wsl" >&2
    exit 1
  fi
  # Remove broken WSL symlink if present
  dst_wsl="/mnt/c/UEProjects/SimWorld/Content/Paks/${p}"
  rm -f "$dst_wsl"
  # Windows hard link (visible to UE Editor)
  cmd.exe /c "if exist \"$dst_win\" del /f \"$dst_win\" & mklink /H \"$dst_win\" \"$src_win\""
  echo "Hard-linked $dst_win -> $src_win"
done

echo ""
echo "Next (UE Editor, before PIE):"
echo "  Tools -> Execute Python Script ->"
echo "  dev/grid_env_10k/scripts/mount_simworld_runtime_paks_editor.py"
echo ""
echo "Mount script also reads C:\\SimWorldServer\\... directly (no link required)."
