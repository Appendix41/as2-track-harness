"""Telemetry and pose history only: no depth, no perception service.

Reads the DDS snapshot written by ``scripts/telemetry_readonly.sh``
(runs/perception_live/telemetry.json) and keeps a short pose history for route
progress and watchdogs. After the depth removal there is no measured obstacle
perception: the RC veto, the watchdog, the speed cap and the command limits in
execution.py are the only bounds.
"""
import json
from dataclasses import asdict
from pathlib import Path
import threading
import time

from .contracts import Pose
from .observation import get_local_json
from .source import PoseBuffer
from .storage import ROOT

TELEMETRY_PATH = ROOT/'runs/perception_live/telemetry.json'
TELEMETRY_MAX_AGE_NS = 250_000_000
OWNER_STATE_URL = 'http://127.0.0.1:8767/manual/state.json'


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def fresh_telemetry(data, boot, now_ns=None):
    """True only for a running same-boot snapshot no older than 250 ms."""
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    stamp = data.get('timestamp_ns')
    return (isinstance(data, dict) and data.get('status') == 'running'
            and data.get('host_boot_id') == boot and type(stamp) is int
            and 0 <= now_ns-stamp < TELEMETRY_MAX_AGE_NS)


class RobotState:
    """Read-only telemetry: sport/low/remote_axes, pose interpolation, owner flags."""

    def __init__(self, telemetry_path=None, owner_url=None):
        self.telemetry_path = Path(telemetry_path) if telemetry_path else TELEMETRY_PATH
        self.owner_url = owner_url or OWNER_STATE_URL
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.buffer = PoseBuffer()
        self.state, self.error = {}, 'waiting_for_telemetry'
        self.last_pose = None
        self.boot = boot_id()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _read(self, now_ns):
        data = json.loads(self.telemetry_path.read_text())
        if not fresh_telemetry(data, self.boot, now_ns):
            raise ValueError('direct_telemetry_stale')
        return data

    def _owner(self):
        try:
            data = get_local_json(self.owner_url, timeout=.10)
            return dict(
                original_manual_enabled=data.get('manual', data).get('enabled'),
                original_vlx_active=data.get('vlx', {}).get('active'),
                motion_counts={k: data.get(k, {}).get('motion_rpc_calls')
                               for k in ('manual', 'vlx')})
        except Exception:
            return dict(original_manual_enabled=None, original_vlx_active=None)

    def _run(self):
        owner, last_owner = {}, 0.
        while not self.stop.is_set():
            begin = time.monotonic()
            try:
                direct = self._read(time.monotonic_ns())
                if time.monotonic()-last_owner >= .5:
                    owner, last_owner = self._owner(), time.monotonic()
                with self.lock:
                    self._ingest(direct)
                    # Never retain or expose the original service's control token.
                    self.state = dict(sport=direct['sport'], low=direct.get('low'),
                                      remote_axes=direct.get('remote_axes'), **owner)
                    self.error = None
            except Exception as exc:
                with self.lock:
                    self.error = type(exc).__name__
            self.stop.wait(max(.005, .025-(time.monotonic()-begin)))

    def _ingest(self, data):
        for p in data.get('pose_history', []):
            self.buffer.add(Pose.parse(p))
        self.last_pose = self.buffer.samples[-1] if self.buffer.samples else None

    def telemetry(self):
        """Latest sport/low/remote_axes snapshot plus error (never a command)."""
        with self.lock:
            return dict(self.state, error=self.error)

    def latest_pose(self):
        with self.lock:
            return self.last_pose

    def pose_at(self, stamp):
        # Refresh directly from the snapshot too, so a slow owner-status read
        # cannot delay interpolation for a camera/prediction frame.
        try:
            data = self._read(time.monotonic_ns())
            with self.lock:
                self._ingest(data)
        except (OSError, ValueError, TypeError, KeyError):
            pass
        with self.lock:
            return self.buffer.at(stamp)

    def pose_history(self):
        with self.lock:
            return [asdict(p) for p in list(self.buffer.samples)[-40:]]

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)


# Backwards-compatible name for older call sites while the runner is rewritten.
RobotReader = RobotState
