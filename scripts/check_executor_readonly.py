#!/usr/bin/env python3
"""Observe a shadow session with real command checks and a fake sink; no robot RPC."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from vlx_async.execution import CommandGate, GuardedActuator, Limits, packet_observation
from vlx_async.storage import atomic_json


class FakeSink:
    def __init__(self):
        self.moves = self.stops = 0

    def move(self, *_):
        self.moves += 1
        return 0

    def stop(self):
        self.stops += 1
        return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8768')
    parser.add_argument('--wait', type=float, default=90)
    parser.add_argument('--duration', type=float, default=6)
    args = parser.parse_args()
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    began = time.monotonic()
    session = None
    while time.monotonic()-began < args.wait:
        with urllib.request.urlopen(args.url.rstrip('/')+'/observe.json', timeout=3) as response:
            test = json.load(response)['test']
        if test['active']:
            if test['mode'] != 'shadow':
                raise SystemExit('shadow session required; never attach to execute')
            session, settings = test['session_id'], test['settings']
            break
        time.sleep(.1)
    if session is None:
        raise SystemExit('no shadow session started')
    limits = Limits(max_forward_mps=settings['speed_mps'],
        max_linear_mps=max(settings['speed_mps'], settings['lateral_mps']),
        max_lateral_mps=settings['lateral_mps'], max_yaw_rps=settings['yaw_rps'],
        max_linear_accel_mps2=settings['linear_accel_mps2'], max_yaw_accel_rps2=settings['yaw_accel_rps2'],
        max_observed_speed_mps=settings['observed_speed_limit_mps'])
    sink = FakeSink()
    gate, actuator, records = CommandGate(limits), None, []
    try:
        while time.monotonic()-began < args.wait:
            started = time.monotonic()
            try:
                packet = json.loads((ROOT/'runs/fusion_execution_packet.json').read_text())
                if packet.get('session_id') != session:
                    raise ValueError('different_session')
                if packet.get('status') == 'finished':
                    break
                now = time.monotonic_ns()
                obs = packet_observation(packet, now, boot)
                command = gate.evaluate(packet['decision'], obs, packet['telemetry'], now)
                if actuator is None and command['allowed']:
                    actuator = GuardedActuator(sink, duration_s=args.duration, limits=limits,
                                              allow_perception_hold=True)
                if actuator:
                    actuator.submit(command)
                    records.append(dict(timestamp_ns=now, allowed=command['allowed'], reason=command['reason'],
                                        holding=actuator.holding, fault=actuator.fault))
                    if actuator.fault:
                        break
            except (OSError, ValueError, KeyError) as exc:
                if actuator:
                    records.append(dict(read_error=str(exc)))
                    break
            time.sleep(max(.001, .05-(time.monotonic()-started)))
    finally:
        if actuator:
            actuator.close()
        report = dict(session_id=session, verification='live inputs, fake sink, no controller handoff',
            simulated_move_calls=sink.moves, simulated_stop_calls=sink.stops,
            perception_holds=actuator.hold_count if actuator else 0,
            fault=actuator.fault if actuator else None, records=records, robot_commands_sent=0)
        output = ROOT/'runs'/('executor_live_readonly_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.json')
        report['output'] = str(output)
        atomic_json(output, report)
        print(json.dumps({k:v for k,v in report.items() if k != 'records'}), flush=True)
    return 0 if sink.moves and report['fault'] == 'motion_duration_complete' else 2


if __name__ == '__main__':
    raise SystemExit(main())
