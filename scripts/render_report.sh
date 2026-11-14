#!/usr/bin/env bash
# Render one saved run into a PNG comparison figure (needs the vision extra).
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export PYTHONPATH="$ROOT/src"
cd "$ROOT"
exec "$PYTHON" -B "$ROOT/scripts/render_report.py" "$@"
