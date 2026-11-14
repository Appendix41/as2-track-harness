"""Read-only dashboard: no action endpoints, robot client, or upstream proxy."""
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading


class Dashboard:
    def __init__(self, host, port, allowed_clients):
        self.lock = threading.Lock()
        self.state = {"status": "starting", "execution_enabled": False, "robot_commands_sent": 0}
        self.allowed = frozenset(allowed_clients) | {"127.0.0.1", "::1"}
        owner = self
        page = Path(__file__).with_name("dashboard.html").read_bytes()
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.client_address[0] not in owner.allowed:
                    self.send_error(403)
                    return
                if self.path == "/":
                    payload, kind = page, "text/html; charset=utf-8"
                elif self.path == "/state.json":
                    with owner.lock:
                        payload = json.dumps(owner.state, ensure_ascii=False, allow_nan=False).encode()
                    kind = "application/json; charset=utf-8"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; frame-ancestors 'none'")
                self.end_headers()
                try:
                    self.wfile.write(payload)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    @property
    def port(self):
        return self.server.server_port

    def update(self, state):
        with self.lock:
            self.state = deepcopy(state)

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1)
