from dataclasses import replace
import json
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock

import numpy as np

from vlx_async import controller_handoff as handoff
from vlx_async.execution import Limits, CommandGate, GuardedActuator, ControllerLease
from vlx_async.trial_control import settings_from_payload
from vlx_async.startup import StartupBarrier
from vlx_async.storage import ROOT
from test_trial_control import SETTINGS
from test_perception_execution import executable_inputs


class CustomControlsTests(unittest.TestCase):
    def test_executor_uses_current_fusion_after_handoff_without_hardware(self):
        spec=importlib.util.spec_from_file_location('test_fusion_executor',ROOT/'scripts/fusion_trial.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        events=[];clock=[1.];counts=[0]
        class Transfer:
            released=True;original_stop_confirmed=True
            def __init__(self,run):self.lease=Mock()
            def acquire(self,cancelled):events.append('handoff');return self.lease
            def restore(self,confirmed):
                self_confirmed=confirmed
                if not self_confirmed:raise AssertionError('stop required')
                events.append('restore');return {'idle':True}
        class Sink:
            def __init__(self,lease):events.append('sink')
            def stop(self):events.append('stop');return 0
        class Actuator:
            fault=None;sent=0;stop_codes=[];hold_count=0;holding=False
            def __init__(self,sink,duration_s,limits,allow_perception_hold=False):
                self.sink=sink;self.limits=limits
                if not allow_perception_hold:raise AssertionError('trial must support stationary perception holds')
                self.duration=duration_s;events.append('actuator')
                if (limits.max_forward_mps!=.8 or limits.max_linear_accel_mps2!=1.2
                        or duration_s!=30):
                    raise AssertionError('configured limits lost')
            def submit(self,command):
                self.sent+=1;events.append('move');clock[0]+=31;return True
            def close(self):self.stop_codes=[self.sink.stop()];self.fault='executor_closed'
        class Gate:
            def __init__(self,limits,controller_lease):pass
            def evaluate(self,*args):
                counts[0]+=1;events.append('fresh_fusion')
                return dict(allowed=True,reason='bounded_command_validated',command_mps_rps=[.1,0.,0.])
        with tempfile.TemporaryDirectory(dir=ROOT/'cache/tmp') as directory:
            root=Path(directory);(root/'runs').mkdir()
            (root/'runs/fusion_execution_packet.json').write_text(json.dumps(dict(session_id='test',decision={},telemetry={})))
            with (patch.object(module,'ROOT',root),patch.object(module,'ControllerHandoff',Transfer),
                  patch.object(module,'AS2Sink',Sink),patch.object(module,'GuardedActuator',Actuator),
                  patch.object(module,'CommandGate',Gate),patch.object(module,'packet_observation',return_value=object()),
                  patch.object(module.signal,'signal'),patch.object(module.time,'monotonic',side_effect=lambda:clock[0]),
                  patch.object(module.time,'monotonic_ns',side_effect=lambda:int(clock[0]*1e9)),
                  patch.object(module.time,'sleep'),patch.object(module,'atomic_json',side_effect=lambda p,d:p.write_text(json.dumps(d))),
                  patch.object(module.sys,'argv',['fusion_trial.py','--execute','--session-id','test','--speed','.8',
                    '--duration','30','--linear-acceleration','1.2','--observed-speed-limit','2']),
                    patch('builtins.print')):
                self.assertEqual(module.main(),0)
            summary=json.loads(next((root/'runs').glob('execution_*/summary.json')).read_text())
            self.assertEqual(summary['motion_commands_sent'],1)
            self.assertEqual(summary['effective_limits']['max_forward_mps'],.8)
        self.assertGreaterEqual(counts[0],2)
        self.assertLess(events.index('handoff'),events.index('actuator'))
        self.assertIn('fresh_fusion',events[events.index('handoff')+1:events.index('actuator')])
        self.assertEqual(events[-2:],['stop','restore'])

    def test_original_dashboard_numeric_rules_and_longer_session(self):
        settings=dict(SETTINGS,speed_mps=5.,lateral_mps=5.,yaw_rps=5.,linear_accel_mps2=1.,
            yaw_accel_rps2=1.5,observed_speed_limit_mps=6.,model_duration_s=600,
            motion_duration_s=300,startup_timeout_s=240)
        self.assertEqual(settings_from_payload(settings),settings)
        self.assertEqual(StartupBarrier('session',240).timeout_s,240)

    def test_gate_and_sink_share_selected_speed_and_acceleration(self):
        d,o,t,n=executable_inputs()
        limits=Limits(max_forward_mps=.4,max_lateral_mps=.3,max_linear_mps=.4,max_yaw_rps=.6,
                      max_linear_accel_mps2=2.,max_yaw_accel_rps2=1.5,max_observed_speed_mps=1.)
        command=CommandGate(limits).evaluate(d,o,t,n)
        self.assertTrue(command['allowed']);self.assertAlmostEqual(command['command_mps_rps'][0],.1)
        sink=Mock();sink.move.return_value=0;sink.stop.return_value=0
        actuator=GuardedActuator(sink,duration_s=30,limits=limits)
        try:
            self.assertTrue(actuator.submit(command))
            sink.move.assert_called_once_with(.1,0.,0.)
            self.assertFalse(actuator.submit(dict(command,command_mps_rps=[.41,0.,0.])))
            self.assertEqual(actuator.fault,'input_gate_rejected')
        finally:
            actuator.close()

    def test_unknown_http_owner_requires_real_exclusive_lease(self):
        d,o,t,n=executable_inputs()
        t.update(original_manual_enabled=None,original_vlx_active=None)
        self.assertFalse(CommandGate().evaluate(d,o,t,n)['allowed'])
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'controller.lock';path.touch()
            lease=ControllerLease(path)
            gate=CommandGate(controller_lease=lease)
            self.assertFalse(gate.evaluate(d,o,t,n)['allowed'])
            lease.acquire()
            try:
                self.assertTrue(CommandGate(controller_lease=lease).evaluate(d,o,t,n)['allowed'])
                other=ControllerLease(path)
                with self.assertRaises(BlockingIOError):other.acquire()
                t['original_vlx_active']=True
                self.assertFalse(CommandGate(controller_lease=lease).evaluate(d,o,t,n)['allowed'])
            finally:
                lease.close()
            self.assertTrue(path.exists())

    def test_handoff_rejects_busy_controller_without_signal(self):
        state=dict(enabled=True,settings_enabled=False,busy=False,stop_rpc_acknowledged=True,
                   vlx=dict(active=False,stop_pending=False))
        response=Mock();response.__enter__=Mock(return_value=response);response.__exit__=Mock()
        response.read.return_value=json.dumps(state).encode()
        with patch.object(handoff.urllib.request,'urlopen',return_value=response),patch.object(handoff.os,'kill') as kill:
            with self.assertRaisesRegex(ValueError,'not_idle'):handoff.read_idle_state()
            kill.assert_not_called()

    def test_handoff_waits_for_exit_and_confirmed_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);(p/'summary.json').write_text(json.dumps({'stop_confirmed':True}))
            h=handoff.ControllerHandoff(p);h.lease=Mock()
            h.lease.acquire.side_effect=[BlockingIOError(),None]
            record=dict(pid=12345,process_start_ticks='start',output=str(p))
            with (patch.object(handoff,'handoff_candidate',return_value=record),
                  patch.object(handoff,'read_idle_state'),patch.object(h,'stationary'),
                  patch.object(handoff,'identity',side_effect=['start',None,None]),
                  patch.object(handoff.os,'kill') as kill):
                self.assertIs(h.acquire(),h.lease)
                kill.assert_called_once_with(12345,handoff.signal.SIGTERM)
            self.assertTrue(h.original_stop_confirmed)
            self.assertEqual(h.lease.acquire.call_count,2)

    def test_failed_stop_never_restores_controller(self):
        h=handoff.ControllerHandoff(Path('/unused'));h.released=True
        with patch.object(handoff.subprocess,'Popen') as spawn:
            with self.assertRaisesRegex(ValueError,'stop_unconfirmed'):h.restore(False)
            spawn.assert_not_called()

if __name__=='__main__':unittest.main()
