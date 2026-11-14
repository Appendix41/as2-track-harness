"""Independent observer with explicit, session-bound test controls."""
import argparse
import base64
from copy import deepcopy
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
from pathlib import Path
import signal
import secrets
import threading
import time
from urllib.parse import urlsplit, parse_qs
import urllib.request
from .source import ReadOnlySource, FastGate
from .storage import ROOT, atomic_json


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None

def get_local_json(url, timeout=.6):
    parsed = urlsplit(url)
    if parsed.scheme != 'http' or parsed.hostname not in {'127.0.0.1', 'localhost'}:
        raise ValueError('loopback_only')
    if parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ValueError('invalid_local_url')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect)
    with opener.open(urllib.request.Request(url, method='GET'), timeout=timeout) as response:
        raw = response.read(2_000_001)
    if len(raw) > 2_000_000:
        raise ValueError('payload_too_large')
    result = json.loads(raw)
    if not isinstance(result, dict):
        raise ValueError('invalid_state')
    return result

def robot_summary(state):
    # Explicit fields only: the original state contains a control token.
    vlx = state.get('vlx') or {}
    robot = vlx.get('robot') or {}
    return {
        'status': state.get('status'), 'enabled': state.get('enabled') is True,
        'ready': state.get('ready') is True, 'blocker': state.get('blocker'),
        'telemetry_age_s': state.get('telemetry_age_s'),
        'mode': robot.get('mode'), 'error_code': robot.get('error_code'),
        'vlx_active': vlx.get('active') is True,
        'command_mps_rps': state.get('command_mps_rps'),
        'speed_limits': state.get('speed_limits'),
        'acceleration_limits': state.get('acceleration_limits'),
        'original_motion_calls': state.get('motion_rpc_calls'),
        'original_vlx_motion_calls': vlx.get('motion_rpc_calls'),
    }

def image_for_request(run, state):
    request_id = state.get('request_id')
    if isinstance(request_id, str) and request_id.isascii() and request_id.isalnum():
        path = (run / ('request_' + request_id + '.jpg')).resolve()
        if path.parent == run.resolve() and path.is_file() and path.stat().st_size < 1_000_000:
            state.update(image_request_id=request_id, image_base64=base64.b64encode(path.read_bytes()).decode())
    return state

def load_archive(run_name):
    if not isinstance(run_name, str) or not run_name.startswith('shadow_') or not all(c.isalnum() or c == '_' for c in run_name):
        raise ValueError('invalid_run')
    run = (ROOT / 'runs' / run_name).resolve()
    if run.parent != (ROOT / 'runs').resolve():
        raise ValueError('invalid_run')
    metadata = json.loads((run / 'metadata.json').read_text())
    state = json.loads((run / 'latest.json').read_text())
    representative = run/'successful_fusion.json'
    if representative.is_file():
        candidate = json.loads(representative.read_text())
        if (candidate.get('selection')=='corrected' and candidate.get('request_id')==state.get('request_id')
            and candidate.get('stream_id')==state.get('stream_id')):
            state = dict(candidate,status='finished',archive_sample='representative_successful_fusion')
    if not state.get('request_id'):
        requests = sorted(run.glob('request_*.json'))
        if requests:
            request = json.loads(requests[-1].read_text())
            state.update(request_id=request['request_id'],archive_incomplete=True,
                         archive_sample='request_without_completed_response')
    state.update(view_mode='archive', archive_run=run_name, archived=True,
                 started_at=metadata.get('started_at'),
                 model=metadata.get('config', {}).get('provider', {}).get('model', metadata.get('provider')))
    return image_for_request(run, state)

def list_archives():
    result = []
    for path in sorted((ROOT / 'runs').glob('shadow_*/metadata.json'), reverse=True):
        try:
            meta = json.loads(path.read_text())
            if not (path.parent / 'latest.json').is_file():
                continue
            model = meta.get('config', {}).get('provider', {}).get('model', meta.get('provider'))
            if model == 'mock':
                continue
            last = json.loads((path.parent/'latest.json').read_text())
            result.append({'run': path.parent.name, 'model': model, 'started_at': meta.get('started_at'),
                           'has_response': bool(last.get('response_anchor_accepted') or last.get('latency_s'))})
        except (OSError, ValueError, TypeError):
            continue
    return result[:30]

def matching_experiment(state, preview, now_ns):
    stamp = state.get('timestamp_ns')
    if state.get('status') != 'shadow' or type(stamp) is not int or not 0 <= (now_ns-stamp)/1e9 < 2:
        return 'experiment_not_live'
    record = preview.get('record') or {}
    if (state.get('stream_id') != preview.get('stream_id')
            or state.get('instruction') != preview.get('requested_instruction')
            or state.get('instruction_revision') != preview.get('requested_instruction_revision')
            or state.get('host_boot_id') != preview.get('host_boot_id')):
        return 'experiment_task_mismatch'
    if record.get('instruction_revision') != preview.get('requested_instruction_revision'):
        return 'instruction_pending'
    return None


class Observer:
    def __init__(self):
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.preview = {}
        self.robot = {}
        self.experiment = {}
        self.preview_at = self.robot_at = self.experiment_at = 0.
        self.preview_error = self.robot_error = None
        self.source = ReadOnlySource()
        self.gate = FastGate(Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        self.threads = [threading.Thread(target=fn, daemon=True) for fn in
                        (self.preview_loop, self.robot_loop, self.experiment_loop)]
        for thread in self.threads:
            thread.start()

    def preview_loop(self):
        while not self.stop.is_set():
            start = time.monotonic()
            try:
                state = self.source.get('/state.json')
                with self.lock:
                    self.preview, self.preview_at, self.preview_error = state, time.monotonic(), None
            except Exception as exc:
                with self.lock:
                    self.preview_error = type(exc).__name__
            self.stop.wait(max(.01, .2-(time.monotonic()-start)))

    def robot_loop(self):
        while not self.stop.is_set():
            try:
                value = robot_summary(get_local_json('http://127.0.0.1:8767/manual/state.json'))
                with self.lock:
                    self.robot, self.robot_at, self.robot_error = value, time.monotonic(), None
            except Exception as exc:
                with self.lock:
                    self.robot_error = type(exc).__name__
            self.stop.wait(.5)

    def experiment_loop(self):
        while not self.stop.is_set():
            found = {}
            for path in sorted((ROOT/'runs').glob('shadow_*/latest.json'), reverse=True)[:20]:
                try:
                    if time.time()-path.stat().st_mtime > 4:
                        continue
                    saved = json.loads(path.read_text())
                    meta = json.loads((path.parent/'metadata.json').read_text())
                    with self.lock:
                        preview = self.preview
                    if matching_experiment(saved, preview, time.monotonic_ns()):
                        continue
                    url = urlsplit(meta['url'])
                    # Discover only the local shadow dashboard; never proxy arbitrary URLs.
                    if url.scheme != 'http' or url.username or url.password or url.path != '/' or url.query or url.fragment:
                        continue
                    allowed_hosts = {'127.0.0.1', 'localhost'}
                    if url.hostname not in allowed_hosts or not url.port or url.port in {8765, 8766, 8767, 8768}:
                        continue
                    target = 'http://127.0.0.1:' + str(url.port) + '/state.json'
                    live = get_local_json(target)
                    if matching_experiment(live, preview, time.monotonic_ns()):
                        continue
                    found = {**live, 'run': path.parent.name, 'view_mode': 'live',
                             'model': meta.get('config', {}).get('provider', {}).get('model', meta.get('provider'))}
                    break
                except (OSError, ValueError, KeyError, TypeError):
                    continue
            with self.lock:
                self.experiment, self.experiment_at = found, time.monotonic()
            self.stop.wait(.5)

    def snapshot(self):
        now, now_ns = time.monotonic(), time.monotonic_ns()
        with self.lock:
            p, robot, exp = deepcopy(self.preview), deepcopy(self.robot), deepcopy(self.experiment)
            age = now-self.preview_at if self.preview_at else None
            robot_age = now-self.robot_at if self.robot_at else None
            source_error, robot_error = self.preview_error, self.robot_error
            gate_error = self.gate.check({**p, 'schema': 'vlx-control-v1'}, now_ns)
        record = p.get('record') or {}
        def age_of(key):
            stamp = record.get(key)
            return (now_ns-stamp)/1e9 if type(stamp) is int else None
        fast = {
            'status': p.get('status', 'unavailable'), 'message': p.get('message'),
            'instruction': p.get('requested_instruction'), 'instruction_revision': p.get('requested_instruction_revision'),
            'stream_id': p.get('stream_id'), 'record': record, 'image_base64': p.get('image_base64'),
            'camera': p.get('camera'), 'camera_info': p.get('camera_info'),
            'camera_age_s': age_of('camera_received_monotonic_ns'),
            'prediction_age_s': age_of('prediction_available_monotonic_ns'),
            'poll_age_s': age, 'valid': not gate_error and not source_error and age is not None and age < .7,
            'reason': source_error or gate_error,
        }
        robot.update(poll_age_s=robot_age, poll_error=robot_error)
        telemetry_age = robot.get('telemetry_age_s')
        robot['connected'] = (not robot_error and robot_age is not None and robot_age < 1.5
                              and type(telemetry_age) in (int,float) and 0 <= telemetry_age + robot_age < 1.)
        reason = matching_experiment(exp, p, now_ns) if exp else 'no_live_experiment'
        if reason:
            exp = {'view_mode': 'inactive', 'reason': reason, 'model': 'deepseek-v4.1-flash'}
        # Depth is gone: the page no longer exposes a perception snapshot, and
        # the task plan (decision['task']) is the only progress source.
        return {'schema': 'vlx-observation-v2', 'server_time': datetime.now().astimezone().isoformat(),
                'fast': fast, 'robot': robot, 'experiment': exp,
                'execution_enabled': False, 'observer_robot_commands_sent': 0, 'observer_api_calls': 0}

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=2)

class ObservationServer(ThreadingHTTPServer):
    daemon_threads = True
    def __init__(self, address, allowed, observer, controller=None):
        self.allowed = set(allowed) | {'127.0.0.1', '::1'}
        self.observer = observer
        self.controller = controller
        super().__init__(address, ObservationHandler)

class ObservationHandler(BaseHTTPRequestHandler):
    def respond(self, status, body, kind='application/json; charset=utf-8'):
        self.send_response(status)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        if self.client_address[0] not in self.server.allowed:
            self.send_error(403)
            return
        host = self.headers.get('Host')
        allowed_hosts = {f'{name}:{self.server.server_port}' for name in
                         [self.server.server_address[0], '127.0.0.1', 'localhost']}
        if host not in allowed_hosts:
            self.send_error(403)
            return
        path = urlsplit(self.path)
        if path.scheme or path.netloc:
            self.send_error(400)
            return
        try:
            if path.path == '/':
                self.respond(200, Path(__file__).with_name('observation.html').read_bytes(), 'text/html; charset=utf-8')
                return
            if path.path == '/trial_controls.js':
                self.respond(200,Path(__file__).with_name('trial_controls.js').read_bytes(),'application/javascript; charset=utf-8')
                return
            if path.path == '/observe.json':
                payload = self.server.observer.snapshot()
                if self.server.controller:
                    payload['test'] = self.server.controller.snapshot()
            elif path.path == '/archives.json':
                payload = list_archives()
            elif path.path == '/archive.json':
                payload = load_archive(parse_qs(path.query).get('run', [''])[0])
            else:
                self.send_error(404)
                return
            self.respond(200, json.dumps(payload, ensure_ascii=False, allow_nan=False).encode())
        except (OSError, ValueError, TypeError):
            self.respond(400, b'{"error":"unavailable_or_invalid_data"}')

    def do_POST(self):
        if self.path not in {'/test/start','/test/stop','/test/heartbeat','/test/limits','/test/clear-blocker','/test/stack/start','/test/stack/stop'} or not self.server.controller:
            self.respond(405,b'{"error":"unsupported_control_route"}')
            return
        host=self.headers.get('Host','')
        allowed_hosts={f'{name}:{self.server.server_port}' for name in
                       [self.server.server_address[0],'127.0.0.1','localhost']}
        if (self.client_address[0] not in self.server.allowed or host not in allowed_hosts
            or self.headers.get('Origin') != 'http://'+host
            or self.headers.get('Content-Type','').split(';')[0] != 'application/json'
            or self.headers.get('Transfer-Encoding')
            or not secrets.compare_digest(self.headers.get('X-Test-Token',''),self.server.controller.token)):
            self.respond(403,b'{"error":"test_request_not_authorized"}')
            return
        try:
            length=int(self.headers.get('Content-Length','0'))
            if not 0 < length <= 4096:
                raise ValueError('invalid_request_length')
            self.connection.settimeout(2)
            payload=json.loads(self.rfile.read(length))
            if self.path=='/test/start':
                result=self.server.controller.start(payload)
            elif self.path=='/test/limits':
                result=self.server.controller.configure_limits(payload)
            elif self.path=='/test/clear-blocker':
                if payload not in ({}, None):
                    raise ValueError('invalid_clear_blocker_payload')
                result=self.server.controller.clear_blocker()
            elif self.path=='/test/stack/start':
                result=self.server.controller.start_stack()
            elif self.path=='/test/stack/stop':
                result=self.server.controller.stop_stack()
            else:
                if not isinstance(payload,dict) or set(payload)!={'session_id'}:
                    raise ValueError('invalid_session_payload')
                method=self.server.controller.stop if self.path=='/test/stop' else self.server.controller.heartbeat
                result=method(payload['session_id'])
            self.respond(200,json.dumps(result).encode())
        except (OSError,ValueError,TypeError) as exc:
            message=str(exc) if isinstance(exc,ValueError) else 'invalid_test_request'
            self.respond(400,json.dumps({'error':message}).encode())

    def log_message(self, *args):
        pass

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--listen', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8768)
    parser.add_argument('--allow-client', action='append', default=[])
    parser.add_argument('--prompt-api-key',action='store_true',help='read a hidden key into memory only')
    args = parser.parse_args(argv)
    address = ipaddress.ip_address(args.listen)
    if address.is_unspecified or address.version != 4 or not 0 <= args.port < 65536:
        parser.error('use a specific IPv4 address and a valid port')
    if not address.is_loopback and not args.allow_client:
        parser.error('LAN access requires explicit --allow-client')
    import os
    from .trial_control import TrialManager
    key=os.environ.pop('DEEPSEEK_API_KEY',None)
    if args.prompt_api_key:
        import getpass
        key=getpass.getpass('DEEPSEEK_API_KEY (hidden, memory only): ').strip()
    controller=TrialManager(key)
    del key
    observer = Observer()
    server = ObservationServer((args.listen, args.port), args.allow_client, observer,controller)
    def stop(*_):
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    metadata = {'url': f'http://{args.listen}:{server.server_port}/', 'pid': __import__('os').getpid(),
                'started_at': datetime.now().astimezone().isoformat(), 'execution_enabled': False,
                'test_controls': True, 'auto_start': False}
    atomic_json(ROOT/'runs/observation_service.json', metadata)
    print(json.dumps(metadata), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        controller.close()
        observer.close()

if __name__ == '__main__':
    main()
