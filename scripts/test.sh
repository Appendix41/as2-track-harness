#!/usr/bin/env bash
# Run the unit test suite from any working directory.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-python3}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export PYTHONPATH="$ROOT/src"
export TMPDIR="$ROOT/cache/tmp"
mkdir -p "$TMPDIR" "$ROOT/runs"
cd "$ROOT"
exec "$PYTHON" -B -m unittest discover -s tests -v
