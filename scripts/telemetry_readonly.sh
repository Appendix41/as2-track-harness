#!/usr/bin/env bash
# Read-only DDS telemetry subscriber (requires the optional Unitree SDK2 deps).
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export PYTHONPATH="$ROOT/src${VLX_SDK_PATH:+:$VLX_SDK_PATH}"
export TMPDIR="$ROOT/cache/tmp"
if [[ -n "${VLX_DDS_LIB:-}" ]]; then
  export LD_LIBRARY_PATH="$VLX_DDS_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
mkdir -p "$TMPDIR" "$ROOT/runs"
cd "$ROOT"
exec "$PYTHON" -B "$ROOT/scripts/telemetry_readonly.py" "$@"
