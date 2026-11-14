"""Transfer an idle controller through the existing lock; never remove it."""
from collections import deque
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request

from .storage import ROOT
from .execution import ControllerLease, Limits
from .fusion_step import validate_telemetry, stationary_evidence

STACK_ROOT = Path(os.environ.get('VLX_STACK_ROOT', str(Path.home()/'vlx-stack')))
LOCK = Path(os.environ.get('VLX_CONTROLLER_LOCK', str(STACK_ROOT/'reports/vlx-as2.lock')))
STATE_URL = 'http://127.0.0.1:8767/manual/state.json'


def identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return None if fields[0] == 'Z' else fields[19]
    except FileNotFoundError:
        return None


def read_idle_state():
    with urllib.request.urlopen(STATE_URL, timeout=.3) as response:
        data = json.load(response)
    vlx = data.get('vlx') or {}
    if (data.get('enabled') is not False or data.get('settings_enabled') is not False
            or data.get('busy') is not False or data.get('stop_rpc_acknowledged') is not True
            or vlx.get('active') is not False or vlx.get('stop_pending') is not False):
        raise ValueError('original_controller_not_idle')
    return {k:data.get(k) for k in ('motion_rpc_calls','ready')}


# The controller record belongs to the optional existing VLX deployment.
# Only that deployment's own manual-web controller is recognised; anything
# else is refused instead of being signalled.
CONTROLLERS = (
    (STACK_ROOT/'reports/vlx-as2-current.json',
     STACK_ROOT/'src/as2_manual_web.py', STACK_ROOT/'reports'),
)


def controller_record():
    for path, script, output_root in CONTROLLERS:
        if not path.exists():
            continue
        record = json.loads(path.read_text())
        if type(record.get('pid')) is not int or identity(record['pid']) != record.get('process_start_ticks'):
            continue
        command = Path(f'/proc/{record["pid"]}/cmdline').read_bytes().split(bytes([0]))
        if str(script).encode() not in command:
            raise ValueError('unexpected_controller_process')
        output = Path(record['output']).resolve()
        if not output.is_relative_to(output_root):
            raise ValueError('unexpected_controller_report')
        return record
    raise ValueError('controller_identity_unavailable')


def handoff_candidate():
    record = controller_record()
    read_idle_state()
    return record


class ControllerHandoff:
    def __init__(self, run):
        self.run = Path(run)
        self.lease = ControllerLease(LOCK)
        self.record = None
        self.released = False
        self.original_stop_confirmed = False
        self.boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()

    def stationary(self, timeout=4., cancelled=lambda:False):
        samples = deque()
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if cancelled():
                raise ValueError('operator_stopped_during_handoff')
            try:
                data = json.loads((ROOT/'runs/perception_live/telemetry.json').read_text())
                sport = validate_telemetry(data,time.monotonic_ns(),self.boot,Limits())
                stamp = sport['received_monotonic_ns']
                if not samples or stamp > samples[-1][0]:
                    samples.append((stamp,sport))
                while samples and stamp-samples[0][0] > 3_000_000_000:
                    samples.popleft()
                if stationary_evidence(samples):
                    return True
            except (OSError,ValueError,KeyError,TypeError):
                samples.clear()
            time.sleep(.04)
        raise ValueError('controller_stationarity_unconfirmed')

    def acquire(self, cancelled=lambda:False):
        try:
            self.lease.acquire()
            return self.lease
        except BlockingIOError:
            pass
        self.record = handoff_candidate()
        self.stationary(cancelled=cancelled)
        read_idle_state()
        if cancelled() or identity(self.record['pid']) != self.record['process_start_ticks']:
            raise ValueError('controller_changed_before_handoff')
        os.kill(self.record['pid'],signal.SIGTERM)
        self.released = True
        deadline = time.monotonic()+6
        while identity(self.record['pid']) is not None and time.monotonic() < deadline:
            time.sleep(.05)
        if identity(self.record['pid']) is not None:
            raise ValueError('controller_did_not_exit')
        summary = json.loads((Path(self.record['output'])/'summary.json').read_text())
        if summary.get('stop_confirmed') is not True:
            raise ValueError('original_controller_stop_unconfirmed')
        self.original_stop_confirmed = True
        self.lease.acquire()
        return self.lease

    def restore(self, stop_confirmed):
        if not self.released:
            return None
        if not stop_confirmed:
            raise ValueError('not_restored_stop_unconfirmed')
        stationarity = 'confirmed'
        try:
            self.stationary(timeout=10.)
        except ValueError:
            # StopMove was already confirmed by the actuator; the restored
            # controller starts disarmed, so a still-unprovable stillness is a
            # warning, not a reason to leave the operator without a controller.
            stationarity = 'unconfirmed'
        self.lease.close()
        sdk = os.environ.get('VLX_SDK_PATH')
        paths = [str(ROOT/'cache/python310')] + ([sdk] if sdk else [])
        env = dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',
            PYTHONPATH=os.pathsep.join(paths),
            LD_LIBRARY_PATH=str(ROOT/'cache/dds-lib'),XDG_CACHE_HOME=str(ROOT/'cache'),TMPDIR=str(ROOT/'cache/tmp'))
        env.pop('DEEPSEEK_API_KEY',None)
        script = STACK_ROOT/'src/as2_manual_web.py'
        if not script.is_file():
            raise ValueError('controller_restore_not_configured')
        output = self.run/'restored_controller'
        with (self.run/'restore.log').open('x') as log:
            child = subprocess.Popen([sys.executable,'-B',
                str(script),'--output',str(output)],
                cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            if child.poll() is not None:
                raise ValueError('controller_restore_failed')
            try:
                read_idle_state()
                return dict(pid=child.pid,output=str(output),idle=True,
                            stationarity=stationarity)
            except Exception:
                time.sleep(.1)
        raise ValueError('controller_restore_timeout')
