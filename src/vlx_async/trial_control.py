"""Own only experiment subprocesses; never reconfigure the original service."""
from copy import deepcopy
from datetime import datetime
import json
import math
import os
from pathlib import Path
import secrets
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request
import uuid

from .execution import ControllerLease, CommandGate, Limits, packet_observation
from .storage import ROOT, atomic_json
from .controller_handoff import handoff_candidate
from .depth_obstacles import obstacles_enabled
from .runtime_limits import load_saved_limits, save_limits, observed_speed_limit, PARAMETER_RANGES, OPTIONAL_DEFAULTS, DEFAULT_OBSERVED_SPEED_MPS

# Optional integration with an existing VLX preview deployment. Point
# VLX_STACK_ROOT at it to use the stack start/stop buttons; shadow and
# observation modes work without it.
STACK_ROOT = Path(os.environ.get('VLX_STACK_ROOT', str(Path.home()/'vlx-stack')))
LOCK = Path(os.environ.get('VLX_CONTROLLER_LOCK', str(STACK_ROOT/'reports/vlx-as2.lock')))
PAGE_HOST = os.environ.get('VLX_OBSERVATION_HOST', '127.0.0.1')
ALLOW_CLIENT = os.environ.get('VLX_ALLOW_CLIENT')
STACK_SCRIPTS = dict(start=STACK_ROOT/'scripts/start_vlx_dashboard.sh',
                     stop=STACK_ROOT/'scripts/stop_vlx_preview.sh',
                     telemetry=ROOT/'scripts/telemetry_readonly.sh',
                     depth=(sys.executable, '-B', '-m', 'vlx_async.depth_obstacles'))


def port_open(port, host='127.0.0.1', timeout=.2):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def settings_from_payload(data):
    required = {'mode','speed_mps','lateral_mps','yaw_rps','model_duration_s','motion_duration_s'}
    if (not isinstance(data,dict) or not required <= set(data)
        or set(data)-required-{'observed_speed_limit_mps','tasks'}-set(OPTIONAL_DEFAULTS)
        or data['mode'] not in ('shadow','execute')):
        raise ValueError('invalid_test_settings')
    tasks = data.get('tasks')
    if tasks is not None:
        if not isinstance(tasks, list) or len(tasks) > 20:
            raise ValueError('invalid_task_queue')
        cleaned = []
        for item in tasks:
            if not isinstance(item, str):
                raise ValueError('invalid_task_queue')
            item = item.strip()
            if not item:
                continue
            if len(item) > 500:
                raise ValueError('task_too_long')
            cleaned.append(item)
        data = dict(data, tasks=cleaned)
    for key,(lo,hi) in PARAMETER_RANGES.items():
        if key not in data:
            continue
        value = data[key]
        if type(value) not in (int,float) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError('out_of_range:'+key)
    if 'observed_speed_limit_mps' in data:
        observed_speed_limit(data['observed_speed_limit_mps'])
    return dict(data)


class TrialManager:
    def __init__(self, api_key=None):
        self.key = api_key or None
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.runner = self.executor = None
        self.sensors = []
        self.boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.last_heartbeat = 0.
        self.inhibit_reason = None
        self.saved_limits = load_saved_limits()
        self.stack_thread = None
        self.stack_phase = 'idle'
        self.stack_children = []
        self.stack_telemetry = []
        self.stack_depth = []
        self.stack_cache = None
        self.stack_cache_at = 0.
        self.stack_error = None
        self.state = dict(active=False,phase='idle',reason=None,session_id=None,
                          mode=None,motion_commands_sent=0,execution_started=False)

    def snapshot(self):
        with self.lock:
            return deepcopy(dict(self.state,api_key_ready=bool(self.key),control_token=self.token,
                limits={k:list(v) for k,v in PARAMETER_RANGES.items()},
                saved_execution_limits=self.saved_limits,
                defaults=OPTIONAL_DEFAULTS,startup_timeout_s=90,stack=self.stack_status()))

    def stack_status(self):
        """Is the heavy experiment stack (model preview, manual web, telemetry) up?"""
        now = time.monotonic()
        with self.lock:
            if self.stack_cache is not None and now-self.stack_cache_at < 1.:
                return dict(self.stack_cache)
        try:
            data = json.loads((ROOT/'runs/perception_live/telemetry.json').read_text())
            telemetry = (data.get('status') == 'running'
                         and 0 <= time.monotonic_ns()-data.get('timestamp_ns', 0) < 1_000_000_000)
        except (OSError, ValueError, TypeError):
            telemetry = False
        components = dict(model=port_open(8765), manual=port_open(8767),
                          page=port_open(8766, PAGE_HOST), telemetry=telemetry,
                          depth=self._depth_fresh(), depth_enabled=obstacles_enabled())
        status = dict(running=components['model'] and components['manual'],
                      components=components, phase=self.stack_phase, error=self.stack_error)
        with self.lock:
            self.stack_cache, self.stack_cache_at = status, now
        return dict(status)

    @staticmethod
    def _telemetry_fresh():
        try:
            data = json.loads((ROOT/'runs/perception_live/telemetry.json').read_text())
            return (data.get('status') == 'running'
                    and 0 <= time.monotonic_ns()-data.get('timestamp_ns', 0) < 1_000_000_000)
        except (OSError, ValueError, TypeError):
            return False

    @staticmethod
    def _stop_telemetry_processes():
        """Stop every telemetry reader of this project, tracked or not."""
        script = str(ROOT/'scripts/telemetry_readonly.py').encode()
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                cmdline = (entry/'cmdline').read_bytes()
            except OSError:
                continue
            if script in cmdline:
                try:
                    os.kill(int(entry.name), signal.SIGTERM)
                except OSError:
                    pass

    @staticmethod
    def _depth_fresh():
        try:
            data = json.loads((ROOT/'runs/perception_live/obstacles.json').read_text())
            return (data.get('status') == 'running'
                    and 0 <= time.monotonic_ns()-data.get('timestamp_ns', 0) < 1_000_000_000)
        except (OSError, ValueError, TypeError):
            return False

    def _start_depth(self):
        log = (ROOT/'runs/depth_obstacles_stack.log').open('a')
        child = subprocess.Popen([str(x) for x in STACK_SCRIPTS['depth']]+['--duration', '43200'],
                                 cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True,
                                 env=dict(os.environ, PYTHONPATH=str(ROOT/'src'),
                                          PYTHONDONTWRITEBYTECODE='1', TMPDIR=str(ROOT/'cache/tmp')))
        log.close()
        self.stack_children.append(child)
        self.stack_depth.append(child)
        return child

    @staticmethod
    def _stop_depth_processes():
        marker = b'vlx_async.depth_obstacles'
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                cmdline = (entry/'cmdline').read_bytes()
            except OSError:
                continue
            if marker in cmdline:
                try:
                    os.kill(int(entry.name), signal.SIGTERM)
                except OSError:
                    pass

    def _start_telemetry(self):
        log = (ROOT/'runs/telemetry_stack.log').open('a')
        child = subprocess.Popen(['bash', str(STACK_SCRIPTS['telemetry']), '--duration', '43200'],
                                 cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True,
                                 env=dict(os.environ, PYTHONPATH=str(ROOT/'src'),
                                          PYTHONDONTWRITEBYTECODE='1', TMPDIR=str(ROOT/'cache/tmp')))
        log.close()
        self.stack_children.append(child)
        self.stack_telemetry.append(child)
        return child

    def _run_stack(self, action):
        self.stack_phase = action
        self.stack_error = None
        try:
            if action == 'starting':
                if not port_open(8765) or not port_open(8767):
                    with (ROOT/'runs/stack_start.log').open('a') as log:
                        command = ['bash', str(STACK_SCRIPTS['start'])]
                        if ALLOW_CLIENT:
                            command.extend(['--allow-client', ALLOW_CLIENT])
                        subprocess.run(command, cwd=STACK_ROOT, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=subprocess.STDOUT, timeout=180, check=False)
                if not self._telemetry_fresh() and not any(c.poll() is None for c in self.stack_telemetry):
                    self._start_telemetry()
                if (obstacles_enabled() and not self._depth_fresh()
                        and not any(c.poll() is None for c in self.stack_depth)):
                    self._start_depth()
                deadline = time.monotonic()+90
                while time.monotonic() < deadline:
                    if port_open(8765) and port_open(8767):
                        break
                    time.sleep(1.)
            else:
                if self.state['active']:
                    self.stop(self.state['session_id'])
                    if self.thread:
                        self.thread.join(timeout=25)
                subprocess.run(['bash', str(STACK_SCRIPTS['stop'])], cwd=STACK_ROOT,
                               stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, timeout=60, check=False)
                for child in self.stack_children:
                    if child.poll() is None:
                        child.send_signal(signal.SIGTERM)
                self.stack_children = []
                self.stack_telemetry = []
                self.stack_depth = []
                self._stop_telemetry_processes()
                self._stop_depth_processes()
                for port in (8765, 8766, 8767):
                    deadline = time.monotonic()+10
                    while port_open(port) and time.monotonic() < deadline:
                        time.sleep(.2)
        except Exception as exc:
            self.stack_error = type(exc).__name__+': '+str(exc)[:200]
        finally:
            self.stack_phase = 'idle'
            with self.lock:
                self.stack_cache = None

    def start_stack(self):
        with self.lock:
            if self.stack_thread and self.stack_thread.is_alive():
                return {'accepted': True, 'phase': self.stack_phase}
            if self.stack_status()['running']:
                if not self._telemetry_fresh() and not any(c.poll() is None for c in self.stack_telemetry):
                    self._start_telemetry()
                if not self._depth_fresh() and not any(c.poll() is None for c in self.stack_depth):
                    self._start_depth()
                return {'accepted': True, 'phase': 'already_running'}
            self.stack_phase = 'starting'
            self.stack_thread = threading.Thread(target=self._run_stack, args=('starting',), daemon=True)
            self.stack_thread.start()
        return {'accepted': True, 'phase': 'starting'}

    def stop_stack(self):
        with self.lock:
            if self.stack_thread and self.stack_thread.is_alive():
                return {'accepted': True, 'phase': self.stack_phase}
            if not self.stack_status()['running'] and not self.state['active']:
                return {'accepted': True, 'phase': 'already_stopped'}
            self.stack_phase = 'stopping'
            self.stack_thread = threading.Thread(target=self._run_stack, args=('stopping',), daemon=True)
            self.stack_thread.start()
        return {'accepted': True, 'phase': 'stopping'}

    def configure_limits(self, payload):
        with self.lock:
            if self.state['active'] or (self.thread and self.thread.is_alive()):
                raise ValueError('stop_test_before_changing_limits')
            self.saved_limits = save_limits(payload)
            return dict(accepted=True,settings=dict(self.saved_limits),applies='next_test')

    def _update(self, **values):
        with self.lock:
            self.state.update(values)

    def start(self, payload):
        settings = settings_from_payload(payload)
        settings = dict(OPTIONAL_DEFAULTS, **settings)
        with self.lock:
            settings.setdefault('observed_speed_limit_mps',self.saved_limits['observed_speed_limit_mps'])
            if self.state['active'] or (self.thread and self.thread.is_alive()):
                raise ValueError('test_already_running')
            if self.inhibit_reason:
                raise ValueError(self.inhibit_reason)
            if not self.key:
                raise ValueError('api_key_unavailable_on_server')
            self.stop_event.clear()
            session = uuid.uuid4().hex
            self.last_heartbeat = time.monotonic()
            self.state = dict(active=True,phase='preparing',reason=None,session_id=session,
                              mode=settings['mode'],settings=settings,motion_commands_sent=0,
                              execution_started=False,started_at=datetime.now().astimezone().isoformat())
            self.thread = threading.Thread(target=self._run,args=(session,settings),daemon=True)
            self.thread.start()
            return dict(accepted=True,session_id=session)

    def heartbeat(self, session):
        with self.lock:
            if not self.state['active'] or session != self.state['session_id']:
                raise ValueError('test_session_mismatch')
            self.last_heartbeat = time.monotonic()
        return {'accepted':True}

    def stop(self, session):
        with self.lock:
            if session != self.state['session_id']:
                raise ValueError('test_session_mismatch')
            if self.state['active']:
                self.state.update(phase='stopping',reason='user_stopped')
                self.stop_event.set()
        return {'accepted':True}

    @staticmethod
    def _set_instruction(text):
        """Set the VLX task text; returns the new revision the runner will see."""
        payload = json.dumps({'instruction': text}).encode()
        request = urllib.request.Request('http://127.0.0.1:8765/instruction', payload,
                                         {'Content-Type': 'application/json'}, method='POST')
        try:
            with urllib.request.urlopen(request, timeout=5.) as response:
                result = json.load(response)
        except (OSError, ValueError) as exc:
            raise ValueError('instruction_update_failed') from exc
        revision = result.get('revision')
        if type(revision) is not int:
            raise ValueError('instruction_update_failed')
        return revision

    @staticmethod
    def _executor_completed(run):
        progress = TrialManager._read(run/'execution_progress.json')
        if progress.get('stop_reason') == 'task_completed' or progress.get('phase') == 'task_completed':
            return True
        try:
            for line in (run/'execution.log').read_text().splitlines():
                value = json.loads(line)
                if value.get('stop_reason') == 'task_completed' or value.get('phase') == 'task_completed':
                    return True
        except (OSError, ValueError, TypeError):
            pass
        return False

    def clear_blocker(self):
        """Clear a restore/stop latch only after the controller really is idle."""
        with self.lock:
            if self.state['active'] or (self.thread and self.thread.is_alive()):
                raise ValueError('test_already_running')
            if not self.inhibit_reason:
                return {'cleared': False, 'blocker': None}
            from .controller_handoff import controller_record, read_idle_state
            controller_record()
            read_idle_state()
            previous = self.inhibit_reason
            self.inhibit_reason = None
            self.state.update(restart_blocker=None, reason='blocker_cleared')
            return {'cleared': True, 'previous': previous}

    @staticmethod
    def _read(path):
        try:
            return json.loads(path.read_text())
        except (OSError,ValueError):
            return {}

    def _spawn(self, command, logfile, api=False):
        env = dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',
                   PYTHONPATH=str(ROOT/'src'),TMPDIR=str(ROOT/'cache/tmp'),
                   XDG_CACHE_HOME=str(ROOT/'cache'),OPENBLAS_NUM_THREADS='1')
        env.pop('DEEPSEEK_API_KEY',None)
        if api:
            env['DEEPSEEK_API_KEY'] = self.key
        with logfile.open('x') as output:
            return subprocess.Popen(command,cwd=ROOT,env=env,stdin=subprocess.DEVNULL,
                stdout=output,stderr=subprocess.STDOUT,start_new_session=True)

    def _sensors(self, run, duration):
        # Telemetry only: the D435i depth/perception service is gone for good.
        value = self._read(ROOT/'runs/perception_live/telemetry.json')
        stamp = value.get('timestamp_ns',0)
        if value.get('status')=='running' and type(stamp) is int and 0 <= time.monotonic_ns()-stamp < 2_000_000_000:
            return
        if any(c.poll() is None for c in self.stack_telemetry):
            return
        child = self._spawn(['bash',str(ROOT/'scripts/telemetry_readonly.sh'),'--duration',str(duration)],
                            run/'telemetry.log')
        self.sensors.append(child)

    def _packet(self, session):
        packet = self._read(ROOT/'runs/fusion_execution_packet.json')
        stamp = packet.get('timestamp_ns')
        if (packet.get('session_id') != session or packet.get('status')!='shadow'
            or type(stamp) is not int or not 0 <= time.monotonic_ns()-stamp < 500_000_000):
            return {}
        return packet

    def _execution_reason(self, packet, settings, gate):
        try:
            now=time.monotonic_ns()
            obs=packet_observation(packet,now,self.boot)
            result=gate.evaluate(packet['decision'],obs,packet['telemetry'],now)
            if not result['allowed']:
                return result['reason']
            lease=ControllerLease(LOCK)
            try:
                lease.acquire()
            except (OSError,BlockingIOError):
                try:
                    handoff_candidate()
                except Exception as exc:
                    return 'controller_handoff_unavailable:'+str(exc)
            finally:
                lease.close()
            return None
        except (ValueError,TypeError,KeyError):
            return 'fresh_execution_packet_required'

    def _run(self, session, settings):
        settings = dict(OPTIONAL_DEFAULTS, **settings)
        tasks = [t for t in (settings.get('tasks') or []) if t]
        queue = settings['mode'] == 'execute' and bool(tasks)
        task_total = len(tasks) if queue else 0
        task_index = 0
        completed_tasks = 0
        run=ROOT/'runs'/('trial_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        reason='test_finished'
        try:
            run.mkdir()
            self._update(output=str(run))
            atomic_json(run/'settings.json',dict(session_id=session,**settings))
            config=self._read(ROOT/'config/deepseek_task.json')
            api_timeout_s=float(config.get('provider',{}).get('timeout_s',60.))
            # One plan per task is enforced by SlowWorker; the provider budget
            # says the same thing so a retry can never be spent mid-task.
            config['provider']['max_calls']=1
            config['task']=dict(plan_timeout_s=settings['model_duration_s'],
                                task_timeout_s=settings['motion_duration_s'],
                                max_travel_m=settings['max_travel_m'])
            config['execution_preview_limits']=dict(max_forward_mps=settings['speed_mps'],max_linear_mps=max(settings['speed_mps'],settings['lateral_mps']),
                max_lateral_mps=settings['lateral_mps'],max_yaw_rps=settings['yaw_rps'],
                max_linear_accel_mps2=settings['linear_accel_mps2'],max_yaw_accel_rps2=settings['yaw_accel_rps2'],
                max_observed_speed_mps=settings.get('observed_speed_limit_mps',DEFAULT_OBSERVED_SPEED_MPS))
            atomic_json(run/'runner_config.json',config)
            # Every queued task gets its own plan window and motion window; the
            # runner stays alive across the whole queue and re-plans only when
            # the instruction (task key) actually changes.
            window=max(settings['model_duration_s'],settings['motion_duration_s'])
            runner_duration=window*max(1,task_total)+(20. if task_total>1 else 0.)
            runner_keep_alive_s=20.
            total=min(86400,settings['startup_timeout_s']+runner_duration+api_timeout_s+30.)
            self._sensors(run,total)
            if queue:
                revision=self._set_instruction(tasks[0])
                self._update(instruction=tasks[0],task_index=1,task_total=task_total,
                             expected_revision=revision)
            runner_args=['--config',str(run/'runner_config.json'),'--enable-api','--port','0',
                '--duration',str(runner_duration),
                '--session-id',session,
                '--startup-timeout',str(settings['startup_timeout_s'])]
            if settings['mode']=='execute':
                # The local VLX/fusion loop remains available for the entire
                # motion window; only new DeepSeek requests stop at model_duration_s.
                runner_args.extend(['--keep-alive-for',str(runner_keep_alive_s)])
            self.runner=self._spawn([sys.executable,'-B','-m','vlx_async.runner',
                *runner_args],run/'model.log',api=True)
            self._update(phase='waiting_vlx')
            gate=CommandGate(Limits(max_forward_mps=settings['speed_mps'],max_linear_mps=max(settings['speed_mps'],settings['lateral_mps']),
                                   max_lateral_mps=settings['lateral_mps'],max_yaw_rps=settings['yaw_rps'],
                                   max_linear_accel_mps2=settings['linear_accel_mps2'],max_yaw_accel_rps2=settings['yaw_accel_rps2'],
                                   max_observed_speed_mps=settings.get('observed_speed_limit_mps',DEFAULT_OBSERVED_SPEED_MPS)))
            while not self.stop_event.wait(.05):
                if time.monotonic()-self.last_heartbeat > 2.:
                    reason='browser_heartbeat_lost'
                    break
                if self.executor is not None:
                    progress=self._read(run/'execution_progress.json')
                    if progress.get('session_id')==session:
                        self._update(motion_commands_sent=progress.get('motion_commands_sent',0),
                            phase=progress.get('phase','waiting_execution_checks'),
                            executor_phase=progress.get('phase'),executor_reason=progress.get('last_reason'),
                            motion_elapsed_s=progress.get('motion_elapsed_s',0))
                if self.executor is not None and self.executor.poll() is not None:
                    exit_code=self.executor.returncode
                    completed=self._executor_completed(run)
                    self.executor=None
                    if completed:
                        completed_tasks+=1
                    if completed and queue and task_index+1 < task_total:
                        task_index+=1
                        text=tasks[task_index]
                        revision=self._set_instruction(text)
                        self._update(task_index=task_index+1,instruction=text,
                                     expected_revision=revision,phase='waiting_plan',
                                     reason='task_queued',execution_started=False,
                                     completed_tasks=completed_tasks)
                        continue
                    reason='motion_window_finished' if exit_code==0 else 'execution_rejected_or_stopped'
                    break
                packet=self._packet(session)
                if packet:
                    decision=packet.get('decision') or {}
                    startup=decision.get('startup') or {}
                    self._update(startup=startup,active_elapsed_s=decision.get('active_elapsed_s',0),
                        api_window_open=decision.get('api_window_open'),
                        vlx_keepalive_after_api=decision.get('vlx_keepalive_after_api',False),
                        execution_blockers=decision.get('execution_blockers',[]),
                        shadow_run=packet.get('run'),slow_worker=decision.get('slow_worker'),
                        instruction=decision.get('instruction'),selection=decision.get('selection'))
                    self._update(fusion_diagnostics={k:decision.get(k) for k in
                        ('request_id','response_anchor_accepted','local_verifier_bind_rejection',
                         'response_rejection','reason','anchor_candidate_id','task_execution_ready')})
                    task_block=decision.get('task') or {}
                    if task_block.get('phase')=='blocked' and task_index>0:
                        reason='task_blocked:'+str(task_block.get('blocked_reason'))
                        break
                    if startup.get('phase') in ('failed','plan_rejected','plan_timeout'):
                        reason=startup.get('reason') or 'first_plan_failed'
                        break
                    if startup.get('first_fusion_complete') is not True:
                        self._update(phase=startup.get('phase','waiting_deepseek'),reason=startup.get('reason'))
                    elif settings['mode']=='shadow':
                        self._update(phase='running_shadow',reason='observation_only')
                    elif self.executor is None:
                        active_elapsed=decision.get('active_elapsed_s',0.)
                        worker_busy=(decision.get('slow_worker') or {}).get('busy') is True
                        expected=self.state.get('expected_revision')
                        if expected is not None and decision.get('instruction_revision')!=expected:
                            self._update(phase='waiting_plan',reason='waiting_new_task_plan')
                            if self.runner.poll() is not None:
                                reason='model_process_failed' if self.runner.returncode else 'model_duration_finished'
                                break
                            continue
                        blocker=self._execution_reason(packet,settings,gate)
                        self._update(phase='waiting_execution_checks',reason=blocker)
                        if blocker is None:
                            self.executor=self._spawn(['bash',str(ROOT/'scripts/fusion_trial.sh'),'--execute',
                                '--session-id',session,'--duration',str(settings['motion_duration_s']),
                                '--progress',str(run/'execution_progress.json'),
                                '--speed',str(settings['speed_mps']),'--lateral-speed',str(settings['lateral_mps']),
                                '--yaw-speed',str(settings['yaw_rps']),
                                '--linear-acceleration',str(settings['linear_accel_mps2']),
                                '--yaw-acceleration',str(settings['yaw_accel_rps2']),
                                '--ready-timeout',str(settings['startup_timeout_s']),
                                '--observed-speed-limit',str(settings.get('observed_speed_limit_mps',DEFAULT_OBSERVED_SPEED_MPS))],run/'execution.log')
                            self._update(phase='waiting_execution_checks',execution_started=True,reason=None)
                        elif (decision.get('api_window_open') is False
                              and active_elapsed >= settings['model_duration_s']+10.
                              and (not worker_busy or active_elapsed >= settings['model_duration_s']+api_timeout_s+10.)):
                            reason='model_duration_finished_without_motion_start'
                            break
                if self.runner.poll() is not None:
                    final=self._read(ROOT/'runs/fusion_execution_packet.json')
                    if self.runner.returncode:
                        reason='model_process_failed'
                    elif final.get('session_id')==session and (final.get('startup') or {}).get('phase') in ('failed','plan_rejected','plan_timeout'):
                        reason=final['startup']['reason']
                    elif self.state.get('phase')=='waiting_execution_checks':
                        reason='motion_not_started:'+str(self.state.get('reason'))
                    else:
                        reason='model_duration_finished'
                    break
            if self.stop_event.is_set():
                reason=self.state.get('reason') or 'user_stopped'
        except Exception as exc:
            reason='trial_failed:'+type(exc).__name__
        finally:
            # Stop the command owner before stopping its input. Never re-arm it.
            for child in [self.executor,self.runner,*self.sensors]:
                if child is not None and child.poll() is None:
                    child.send_signal(signal.SIGTERM)
                    try:
                        child.wait(timeout=18 if child is self.executor else 3)
                    except subprocess.TimeoutExpired:
                        if child is self.executor:
                            reason='executor_stop_not_confirmed'
                            self.inhibit_reason=reason
                        else:
                            child.kill()
                            child.wait(timeout=2)
            motion=None
            if (run/'execution.log').exists():
                for line in (run/'execution.log').read_text().splitlines():
                    try:
                        value=json.loads(line)
                        if 'stop_confirmed' in value:
                            motion=value
                    except ValueError:
                        continue
            if self.state.get('execution_started') and (not motion or motion.get('stop_confirmed') is not True):
                self.inhibit_reason='executor_stop_not_confirmed'
                reason=self.inhibit_reason
            elif motion and motion.get('restore_error'):
                self.inhibit_reason='controller_restore_unconfirmed'
                reason=self.inhibit_reason
            model_summary={}
            model_path=Path(self.state.get('shadow_run') or run)
            if model_path.parent==ROOT/'runs' and model_path.name.startswith('shadow_'):
                model_summary=self._read(model_path/'summary.json')
            worker=model_summary.get('slow_worker')
            if worker:
                self._update(slow_worker=dict(worker,busy=False,busy_at_exit=worker.get('busy'),local_process_exited=True))
            self._update(active=False,phase='finished',reason=reason,
                         completed_tasks=completed_tasks,task_index=task_index,
                         motion_commands_sent=(motion or {}).get('motion_commands_sent',0),
                         execution_result=motion,model_summary=str(model_path/'summary.json') if model_summary else None,
                         executor_phase=(motion or {}).get('phase'),executor_reason=(motion or {}).get('last_reason'),
                         motion_elapsed_s=(motion or {}).get('motion_elapsed_s',0),
                         restart_blocker=self.inhibit_reason,finished_at=datetime.now().astimezone().isoformat())
            saved=self.snapshot()
            saved.pop('control_token',None)
            if run.exists():
                atomic_json(run/'summary.json',saved)
            self.runner=self.executor=None
            self.sensors=[]

    def close(self):
        self._update(reason='server_stopping')
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=25)
        self.key=None
