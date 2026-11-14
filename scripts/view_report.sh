#!/usr/bin/env bash
# Browse a saved shadow run offline (no camera, no API, no robot).
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export PYTHONPATH="$ROOT/src"
export XDG_CACHE_HOME="$ROOT/cache" TMPDIR="$ROOT/cache/tmp"
mkdir -p "$TMPDIR" "$ROOT/runs"
cd "$ROOT"
exec "$PYTHON" -B -m vlx_async.report_viewer "$@"
