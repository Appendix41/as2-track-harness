#!/usr/bin/env python3
"""Configurable fused-path executor. Default dry-run; exclusive control required."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT/'src'))
# Optional local Unitree SDK2 Python bindings, never vendored in this repo.
if os.environ.get('VLX_SDK_PATH'):
    sys.path.insert(0, os.environ['VLX_SDK_PATH'])
from vlx_async.execution import Limits, CommandGate, AS2Sink, GuardedActuator, packet_observation, wait_for_command
from vlx_async.controller_handoff import ControllerHandoff
from vlx_async.storage import atomic_json, confined
from vlx_async.runtime_limits import load_saved_limits


class TaskComplete(Exception):
    """Fusion reports that the frozen route of this task has been completed."""

    def __init__(self, details=None):
        super().__init__('route_completed')
        self.details = dict(details or {})


class TaskWait(Exception):
    """The task plan is not ready yet; hold still instead of rejecting execution."""

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=60.)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--session-id')
    parser.add_argument('--speed',type=float,default=.05)
    parser.add_argument('--lateral-speed',type=float,default=.03)
    parser.add_argument('--yaw-speed',type=float,default=.10)
    parser.add_argument('--observed-speed-limit',type=float,default=None)
    parser.add_argument('--linear-acceleration',type=float,default=.30)
    parser.add_argument('--yaw-acceleration',type=float,default=.60)
    parser.add_argument('--ready-timeout',type=float,default=60.)
    parser.add_argument('--progress',type=Path)
    args = parser.parse_args()
    if not 0 < args.duration <= 86400:
        parser.error('duration must be (0,86400] seconds')
    if not 0 < args.ready_timeout <= 86400:
        parser.error('ready timeout must be (0,86400] seconds')
    progress = confined(args.progress) if args.progress else None
    run = ROOT/'runs'/('execution_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run.mkdir()
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    observed_limit = load_saved_limits()['observed_speed_limit_mps'] if args.observed_speed_limit is None else args.observed_speed_limit
    limits = Limits(max_forward_mps=args.speed,max_linear_mps=max(args.speed,args.lateral_speed),
        max_lateral_mps=args.lateral_speed,max_yaw_rps=args.yaw_speed,
        max_linear_accel_mps2=args.linear_acceleration,max_yaw_accel_rps2=args.yaw_acceleration,
        max_observed_speed_mps=observed_limit)
    handoff = ControllerHandoff(run)
    gate = CommandGate(limits,controller_lease=handoff.lease)
    actuator = sink = None
    stop = [False]
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_:stop.__setitem__(0,True))
    began = time.monotonic()
    motion_began = None
    motion_ended = None
    summary = dict(output=str(run),session_id=args.session_id,mode='execute' if args.execute else 'dry_run',
        phase='preparing',motion_commands_sent=0,stop_confirmed=None,stop_reason=None,task_completed=False,
        effective_limits=limits.__dict__)

    def publish(**changes):
        summary.update(changes)
        summary.update(motion_commands_sent=actuator.sent if actuator else 0,
            perception_holds=actuator.hold_count if actuator else 0,
            motion_elapsed_s=(motion_ended if motion_ended is not None else time.monotonic())-motion_began if motion_began is not None else 0.)
        if progress:
            atomic_json(progress,summary)

    def next_command():
        packet = json.loads((ROOT/'runs/fusion_execution_packet.json').read_text())
        if args.session_id and packet.get('session_id') != args.session_id:
            raise ValueError('execution_session_mismatch')
        decision = packet.get('decision') or {}
        # New task lifecycle reasons are not execution rejections: only an
        # explicitly rejected plan is fatal, a completed route is success, and
        # everything else while stationary means "plan not ready yet".
        if 'reason' in decision:
            reason = decision.get('reason')
            if reason == 'route_completed':
                raise TaskComplete(dict(task_reason=reason,
                    plan_status=decision.get('plan_status'),
                    remaining_route_m=decision.get('remaining_route_m'),
                    progress_fraction=decision.get('progress_fraction')))
            if isinstance(reason,str) and reason.startswith('plan_rejected'):
                raise ValueError('task_'+reason)
            if reason != 'task_ready' or decision.get('task_execution_ready') is not True:
                raise TaskWait(reason if isinstance(reason,str) and reason else 'task_not_ready')
        now = time.monotonic_ns()
        obs = packet_observation(packet,now,boot)
        return now,gate.evaluate(packet['decision'],obs,packet['telemetry'],now)

    def wait_preflight(timeout_s, report):
        """Wait for a ready plan, staying still while planning is still pending."""
        deadline = time.monotonic()+timeout_s
        while not stop[0]:
            try:
                return wait_for_command(next_command,max(0.,deadline-time.monotonic()),
                                        lambda: stop[0],report)
            except TaskWait as waiting:
                report(waiting.reason)
                if time.monotonic() >= deadline:
                    raise ValueError('current_fusion_wait_timeout:'+waiting.reason)
                time.sleep(min(.025,max(0.,deadline-time.monotonic())))
        raise ValueError('operator_stopped_before_motion')

    print(json.dumps(summary),flush=True)
    pending_command = None
    completion = None
    try:
        if args.execute:
            publish(phase='waiting_current_fusion_before_handoff')
            wait_preflight(args.ready_timeout,lambda reason:publish(last_reason=reason))
            publish(phase='transferring_controller',last_reason=None)
            handoff.acquire(cancelled=lambda:stop[0])
            if stop[0]:
                raise ValueError('operator_stopped_before_rpc')
            sink = AS2Sink(handoff.lease)
            if sink.stop() != 0:
                raise ValueError('initial_stop_rpc_failed')
            # Handoff and read-only RPC take time; consume new evidence afterwards.
            gate = CommandGate(limits,controller_lease=handoff.lease)
            publish(phase='waiting_fresh_fusion_after_handoff',last_reason=None)
            pending_command = wait_preflight(args.ready_timeout,lambda reason:publish(last_reason=reason))
            actuator = GuardedActuator(sink,duration_s=args.duration,limits=limits,
                                      allow_perception_hold=True)
            motion_began = time.monotonic()
            publish(phase='running_motion',last_reason=None)
        else:
            motion_began = time.monotonic()
        with (run/'commands.jsonl').open('w') as log:
            while not stop[0] and time.monotonic()-motion_began<args.duration:
                start = time.monotonic()
                try:
                    now,command = pending_command if pending_command is not None else next_command()
                    pending_command = None
                    if args.execute:
                        if time.monotonic_ns()-now>100_000_000:
                            command=dict(allowed=False,reason='command_expired',command_mps_rps=[0.,0.,0.])
                        actuator.submit(command)
                        if actuator.fault:
                            stop[0]=True
                    log.write(json.dumps(dict(timestamp_ns=now,**command),allow_nan=False)+'\n')
                    log.flush()
                    publish(last_reason=command['reason'],
                            phase='holding_for_perception' if actuator and actuator.holding else 'running_motion' if args.execute else 'dry_run')
                except TaskComplete as done:
                    completion = done
                    log.write(json.dumps(dict(timestamp_ns=time.monotonic_ns(),allowed=False,
                        reason='route_completed',task_completed=True,**done.details),allow_nan=False)+'\n')
                    log.flush()
                    break
                except TaskWait as waiting:
                    # Planning pending while stationary: wait, never reject execution.
                    publish(last_reason=waiting.reason)
                    if actuator and actuator.fault:
                        stop[0]=True
                except Exception as exc:
                    publish(last_reason=str(exc))
                    log.write(json.dumps(dict(timestamp_ns=time.monotonic_ns(),allowed=False,reason=str(exc)))+'\n')
                    log.flush()
                    if args.execute:
                        stop[0]=True
                if not stop[0]:
                    time.sleep(max(0.,.05-(time.monotonic()-start)))
    except TaskComplete as done:
        completion = done
    except Exception as exc:
        publish(last_reason=str(exc),error=type(exc).__name__)
    finally:
        if completion is not None:
            summary.update(stop_reason='task_completed',task_completed=True,
                           completion_reason='route_completed',completion=completion.details)
        motion_ended = time.monotonic() if motion_began is not None else None
        if actuator:
            actuator.close()
            summary.update(stop_codes=actuator.stop_codes,stop_confirmed=bool(actuator.stop_codes and actuator.stop_codes[-1]==0),fault=actuator.fault)
        elif sink:
            try:
                code=sink.stop()
            except Exception:
                code=None
            summary.update(stop_codes=[code],stop_confirmed=code==0)
        else:
            summary['stop_confirmed']=not handoff.released or handoff.original_stop_confirmed
        if handoff.released:
            publish(phase='restoring_idle_controller')
            try:
                summary['restored_controller']=handoff.restore(summary['stop_confirmed'])
            except Exception as exc:
                summary['restore_error']=str(exc)
        handoff.lease.close()
        publish(phase='task_completed' if completion is not None else 'finished',elapsed_s=time.monotonic()-began)
        atomic_json(run/'summary.json',summary)
        print(json.dumps(summary),flush=True)
    completed = summary.get('stop_reason') == 'task_completed'
    return 0 if not args.execute or (summary.get('stop_confirmed') is True
        and not summary.get('restore_error') and not summary.get('error')
        and (completed or (summary['motion_commands_sent']>0
            and summary.get('fault') in ('executor_closed','motion_duration_complete')))) else 2


if __name__=='__main__':
    raise SystemExit(main())
