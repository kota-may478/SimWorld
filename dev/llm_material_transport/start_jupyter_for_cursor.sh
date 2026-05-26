#!/usr/bin/env bash
# Start a local Jupyter Server for Cursor notebook connection (WSL workaround).
# Usage: dev/llm_material_transport/start_jupyter_for_cursor.sh
set -eu

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
CONDA_SH="$HOME/miniforge3/etc/profile.d/conda.sh"

if [ ! -f "$CONDA_SH" ]; then
  echo "Miniforge not found at $CONDA_SH" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "$CONDA_SH"
conda activate simworld

cd "$ROOT"

TOKEN="${JUPYTER_CURSOR_TOKEN:-simworld-cursor}"
PORT="${JUPYTER_CURSOR_PORT:-8899}"

echo "Starting Jupyter Server for Cursor..."
echo "  root:  $ROOT"
echo "  port:  $PORT"
echo "  token: $TOKEN"
echo
echo "In Cursor notebook: Select Kernel -> Existing Jupyter Server -> paste:"
echo "  http://127.0.0.1:${PORT}/?token=${TOKEN}"
echo

if ss -tln | awk '{print $4}' | grep -q ":${PORT}$"; then
  echo "Port ${PORT} is already in use — reusing existing Jupyter Server."
  echo "If connection fails, stop old servers with: pkill -f 'jupyter-lab.*--port=${PORT}'"
  exit 0
fi

exec jupyter lab \
  --no-browser \
  --ip=127.0.0.1 \
  --port="$PORT" \
  --IdentityProvider.token="$TOKEN" \
  --ServerApp.root_dir="$ROOT"
