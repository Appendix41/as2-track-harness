#!/usr/bin/env bash
# Dry-run fused-path executor by default; --execute is an explicit opt-in.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export PYTHONPATH="$ROOT/src"
export TMPDIR="$ROOT/cache/tmp"
if [[ -n "${VLX_DDS_LIB:-}" ]]; then
  export LD_LIBRARY_PATH="$VLX_DDS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
mkdir -p "$TMPDIR" "$ROOT/runs"
cd "$ROOT"
exec "$PYTHON" -B "$ROOT/scripts/fusion_trial.py" "$@"
