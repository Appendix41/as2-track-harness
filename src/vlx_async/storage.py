"""All generated files stay under this repository root."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def confined(path):
    path = Path(path).resolve()
    allowed = [ROOT/name for name in ("runs", "cache", "manifests", "docs", "config")]
    if not any(path.is_relative_to(parent) for parent in allowed):
        raise ValueError("output_must_stay_in_experiment_directory")
    return path


def atomic_json(path, payload):
    path = confined(path)
    temporary = path.with_name(path.name+".tmp")
    confined(temporary)
    temporary.write_text(json.dumps(payload, ensure_ascii=False, allow_nan=False, indent=2)+"\n")
    temporary.replace(path)
