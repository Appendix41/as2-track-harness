#!/usr/bin/env python3
"""Synthetic VLX preview and telemetry for offline mock runs.

Serves the two GET endpoints the read-only harness consumes, and writes a
synthetic telemetry snapshot to ``runs/perception_live/telemetry.json`` so a plan
can be placed in a body frame without hardware:

    GET /control.json   synthetic 8x3 waypoints, fresh timestamps
    GET /state.json     the same record plus a tiny placeholder image

The waypoints are a straight synthetic route, not a model prediction, and the
image is a placeholder. Do not point a real execution trial at this server.
"""
import argparse
import base64
import json
import sys
import threading
import time
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from vlx_async.replay import synthetic_observation  # noqa: E402

BOOT = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
# Minimal JPEG marker pair: enough to exercise request_*.jpg, not an image.
TINY_JPEG = base64.b64encode(b'\xff\xd8\xff\xd9').decode()

TELEMETRY_PATH = ROOT / 'runs/perception_live/telemetry.json'


def envelope(now_ns):
    seconds = (now_ns / 1e9) % 1000.
    obs = synthetic_observation(seconds)
    obs = replace(obs, boot_id=BOOT, camera_ns=now_ns, prediction_ns=now_ns,
                  camera_index=now_ns // 50_000_000,
                  prediction_index=now_ns // 50_000_000)
    record = {
        'prediction_index': obs.prediction_index,
        'camera_frame_index': obs.camera_index,
        'instruction': obs.instruction,
        'instruction_revision': obs.revision,
        'valid_history_frames': 31,
        'camera_received_monotonic_ns': obs.camera_ns,
        'prediction_available_monotonic_ns': obs.prediction_ns,
        'waypoints': [list(p) for p in obs.waypoints],
    }
    return {
        'schema': 'vlx-control-v1',
        'host_boot_id': obs.boot_id,
        'stream_id': obs.stream_id,
        'status': 'running',
        'requested_instruction': obs.instruction,
        'requested_instruction_revision': obs.revision,
        'record': record,
    }


def atomic_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(payload, allow_nan=False))
    temporary.replace(path)


def telemetry_snapshot(now_ns):
    pose = {'x': 0., 'y': 0., 'yaw': 0., 'timestamp_ns': now_ns,
            'frame_id': 'synthetic_world'}
    return {
        'status': 'running',
        'host_boot_id': BOOT,
        'timestamp_ns': now_ns,
        'sport': {'received_monotonic_ns': now_ns, 'mode': 3, 'error_code': 0,
                  'position': [0., 0., 0.], 'rpy': [0., 0., 0.],
                  'velocity': [0., 0., 0.], 'yaw_speed': 0.},
        'low': {'received_monotonic_ns': now_ns, 'mode_machine': 1},
        'remote_axes': {'lx': 0., 'ly': 0., 'rx': 0., 'ry': 0., 'keys': 0},
        'pose_history': [pose],
    }


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ('/control.json', '/state.json', '/camera.json'):
            self.send_error(404)
            return
        body = envelope(time.monotonic_ns() - 5_000_000)
        if self.path in ('/state.json', '/camera.json'):
            body['image_base64'] = TINY_JPEG
        raw = json.dumps(body).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *args):
        pass


def telemetry_loop(stop, path):
    while not stop.wait(0.02):
        try:
            atomic_json(path, telemetry_snapshot(time.monotonic_ns()))
        except OSError:
            pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--listen', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--telemetry', default=str(TELEMETRY_PATH))
    parser.add_argument('--no-telemetry', action='store_true')
    args = parser.parse_args(argv)
    if args.listen not in ('127.0.0.1', 'localhost'):
        parser.error('the synthetic preview is loopback-only')
    stop = threading.Event()
    if not args.no_telemetry:
        path = Path(args.telemetry)
        atomic_json(path, telemetry_snapshot(time.monotonic_ns()))
        threading.Thread(target=telemetry_loop, args=(stop, path), daemon=True).start()
    server = ThreadingHTTPServer((args.listen, args.port), Handler)
    server.daemon_threads = True
    print(json.dumps({'url': f'http://{args.listen}:{server.server_port}/',
                      'kind': 'synthetic_vlx_preview',
                      'telemetry': None if args.no_telemetry else args.telemetry,
                      'warning': 'synthetic waypoints and telemetry; never used for real motion'}),
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()


if __name__ == '__main__':
    main()
