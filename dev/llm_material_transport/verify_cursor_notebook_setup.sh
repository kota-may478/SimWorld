#!/usr/bin/env bash
# CLI verification for Cursor notebook setup (no UI required).
set -eu

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PY="$HOME/miniforge3/envs/simworld/bin/python"
JUPYTER="$HOME/miniforge3/envs/simworld/bin/jupyter"

echo "== SimWorld Cursor notebook setup check =="
echo "root: $ROOT"
echo

echo "[1/5] Python + simworld import"
"$PY" -c "import simworld; print('  ok:', simworld.__file__)"
echo

echo "[2/5] kernelspec list"
"$JUPYTER" kernelspec list
echo

echo "[3/5] direct kernelspec connect"
"$PY" "$ROOT/dev/llm_material_transport/_test_kernel_connect.py"
echo

echo "[4/5] headless nbconvert (_cursor_kernel_test.ipynb)"
"$JUPYTER" nbconvert \
  --execute "$ROOT/dev/llm_material_transport/_cursor_kernel_test.ipynb" \
  --ExecutePreprocessor.kernel_name=simworld \
  --to notebook \
  --output /tmp/_cursor_kernel_test_out.ipynb
echo "  ok"
echo

echo "[5/5] Jupyter Server API (port 8899)"
if curl -sf -o /dev/null "http://127.0.0.1:8899/api?token=simworld-cursor"; then
  "$PY" "$ROOT/dev/llm_material_transport/_test_jupyter_server_connect.py"
else
  echo "  server not running — start with:"
  echo "    dev/llm_material_transport/start_jupyter_for_cursor.sh"
fi
echo
echo "All CLI checks passed."
