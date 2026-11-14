#!/usr/bin/env bash
# Serve synthetic VLX observations for offline mock runs (no hardware).
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export PYTHONPATH="$ROOT/src"
cd "$ROOT"
exec "$PYTHON" -B "$ROOT/scripts/mock_preview.py" "$@"
