#!/usr/bin/env python3
"""Send exactly one real image request to the configured planner; never moves.

The camera image comes from an existing local VLX preview over loopback. The
key is read from the environment variable named by ``provider.api_key_env``
(default ``DEEPSEEK_API_KEY``) and is never printed or written.
"""
import argparse
import base64
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from vlx_async.contracts import Observation, Request          # noqa: E402
from vlx_async.providers import ResponsesProvider            # noqa: E402
from vlx_async.source import FastGate, ReadOnlySource        # noqa: E402
from vlx_async.storage import atomic_json                    # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'config/deepseek_task.json'))
    parser.add_argument('--source', default='http://127.0.0.1:8765')
    parser.add_argument('--enable-api', action='store_true',
                        help='required: this script sends one real, billable request')
    args = parser.parse_args()
    if not args.enable_api:
        parser.error('pass --enable-api: this sends a real request to the configured endpoint')
    config = json.loads(Path(args.config).read_text())
    provider = ResponsesProvider(config['provider'], enabled=True)
    state = ReadOnlySource(args.source).get('/state.json')
    state['schema'] = 'vlx-control-v1'
    reason = FastGate(Path('/proc/sys/kernel/random/boot_id').read_text().strip()) \
        .check(state, time.monotonic_ns())
    if reason:
        raise SystemExit('source_not_usable: ' + reason)
    obs = Observation.from_state(state)
    request = Request('probe-' + str(time.monotonic_ns()), obs, time.monotonic_ns())
    started = time.monotonic()
    result = dict(model=provider.model, base_url=provider.base_url,
                  reasoning_effort=provider.reasoning_effort,
                  request_id=request.request_id, source_frame=obs.camera_index,
                  robot_commands_sent=0)
    try:
        plan = provider.plan(request, threading.Event())
        result.update(ok=True, plan_frame=plan.coordinate_frame,
                      waypoints=len(plan.waypoints), confidence=plan.confidence,
                      plan_reason=plan.reason)
    except Exception as exc:  # network/API/schema errors are reported, not raised
        result.update(ok=False, error=type(exc).__name__ + ': ' + str(exc)[:500])
    result['latency_s'] = round(time.monotonic() - started, 3)
    run = ROOT / 'runs' / ('probe_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run.mkdir(parents=True)
    if obs.image_base64:
        (run / 'request.jpg').write_bytes(base64.b64decode(obs.image_base64))
    atomic_json(run / 'probe.json', result)
    print(json.dumps({**result, 'output': str(run)}, ensure_ascii=False), flush=True)
    return 0 if result['ok'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
