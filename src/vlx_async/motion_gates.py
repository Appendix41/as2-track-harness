"""Operator switches that decide whether measured obstacles stop motion.

Read once per Fusion/CommandGate construction, so a change takes effect on the
next fusion/execution process (the next test run), never mid-run. A missing or
malformed file falls back to stopping, so deleting the file cannot silently
remove the protection.
"""
import json
from pathlib import Path

from .storage import ROOT

GATES_PATH = ROOT/'config/motion_gates.json'
DEFAULTS = {'obstacle_stop_enabled': True}


def load(path=None):
    path = Path(path) if path is not None else GATES_PATH
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return dict(DEFAULTS)
    value = data.get('obstacle_stop_enabled') if isinstance(data, dict) else None
    return {'obstacle_stop_enabled': value if type(value) is bool
            else DEFAULTS['obstacle_stop_enabled']}


def obstacle_stop_enabled(path=None):
    return load(path)['obstacle_stop_enabled']
