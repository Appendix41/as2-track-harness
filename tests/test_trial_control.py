import copy
import json
import threading
import time
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
import urllib.request
import urllib.error

from vlx_async.startup import StartupBarrier
from vlx_async.trial_control import TrialManager, settings_from_payload
from vlx_async.runner import lifecycle_windows
from vlx_async.execution import packet_observation
from vlx_async.observation import ObservationServer
from vlx_async.runtime_limits import validate_saved_limits
from vlx_async.storage import ROOT


SETTINGS=dict(mode='shadow',speed_mps=.05,lateral_mps=.02,yaw_rps=.1,
              model_duration_s=60,motion_duration_s=3)

def decision(**changes):
    result=dict(host_boot_id='boot',stream_id='stream',instruction_revision=1,instruction='far chair',
                reason='task_ready',task_execution_ready=True,plan_request_id='plan-1',
                task=dict(phase='executing',plan_request_id='plan-1',plan_calls=1),
                selection='plan_aligned',recommended=[[i*.1,0,0] for i in range(8)])
    return dict(result,**changes)


class StartupTests(unittest.TestCase):
    def setUp(self):
        self.clock=[10.]
        self.barrier=StartupBarrier('new-session',90,clock=lambda:self.clock[0])

    def test_no_departure_before_real_first_fusion(self):
        self.assertEqual(self.barrier.update({},False)['phase'],'waiting_vlx')
        for change in [dict(reason='plan_pending'),dict(task_execution_ready=False),
                       dict(plan_request_id=None,task=dict(phase='waiting_plan',plan_calls=0)),
                       dict(recommended=[])]:
            self.assertFalse(self.barrier.update(decision(**change),True)['first_fusion_complete'])

    def test_accepted_plan_fills_the_legacy_ready_field(self):
        self.clock[0]=25.
        state=self.barrier.update(decision(selection='plan_aligned'),True)
        self.assertTrue(state['first_fusion_complete'])
        self.assertTrue(state['plan_ready'])
        self.assertEqual(state['first_request_id'],'plan-1')
        self.assertEqual(state['waiting_elapsed_s'],15.)
        self.clock[0]=30.
        self.assertEqual(self.barrier.snapshot()['waiting_elapsed_s'],15.)

    def test_success_is_latched_but_new_task_requires_new_session(self):
        self.barrier.update(decision(),True)
        self.assertTrue(self.barrier.update(decision(reason='waiting_for_slow_plan'),True)['first_fusion_complete'])
        state=self.barrier.update(decision(instruction_revision=2),True)
        self.assertEqual(state['phase'],'plan_rejected')
        self.assertFalse(state['first_fusion_complete'])

    def test_timeout_cannot_be_unlatched_by_late_success(self):
        self.clock[0]=101.
        self.assertEqual(self.barrier.update(decision(),True)['reason'],'first_plan_timeout')
        self.assertFalse(self.barrier.update(decision(),True)['first_fusion_complete'])

    def test_separate_run_never_inherits_first_fusion(self):
        self.barrier.update(decision(),True)
        other=StartupBarrier('other-session')
        self.assertFalse(other.snapshot()['first_fusion_complete'])


class RunnerWindowTests(unittest.TestCase):
    def test_model_api_window_can_end_while_local_fusion_loop_stays_live(self):
        self.assertEqual(lifecycle_windows(59.9,60.,73.),dict(api_open=True,runner_open=True))
        self.assertEqual(lifecycle_windows(60.,60.,73.),dict(api_open=False,runner_open=True))
        self.assertEqual(lifecycle_windows(132.9,60.,73.),dict(api_open=False,runner_open=True))
        self.assertEqual(lifecycle_windows(133.,60.,73.),dict(api_open=False,runner_open=False))
        for values in [(-1.,60.,1.),(0.,0.,1.),(0.,1.,-1.),(float('nan'),1.,1.)]:
            with self.assertRaisesRegex(ValueError,'lifecycle_window'):
                lifecycle_windows(*values)

    def test_packet_decoder_requires_session_startup_barrier(self):
        now=time.monotonic_ns()
        packet=dict(schema='vlx-fusion-execution-packet-v1',status='shadow',timestamp_ns=now,
                    session_id='new-session',decision={'startup':{'session_id':'old-session','first_fusion_complete':True}})
        with self.assertRaisesRegex(ValueError,'first_fusion_not_complete'):
            packet_observation(packet,now,'boot')


class SettingsTests(unittest.TestCase):
    def test_valid_settings_preserved(self):
        self.assertEqual(settings_from_payload(SETTINGS),SETTINGS)

    def test_nonfinite_and_unsupported_settings_rejected(self):
        for patching in [{'speed_mps':float('nan')},{'model_duration_s':float('inf')},
                         {'motion_duration_s':86401},{'speed_mps':0},{'lateral_mps':-1},
                         {'yaw_rps':1e39},{'mode':'raw'},{'speed_mps':True},
                         {'linear_accel_mps2':0},{'yaw_accel_rps2':float('nan')},
                         {'model_duration_s':0.4},{'startup_timeout_s':0},
                         {'candidate_max_age_s':1.5}]:
            with self.assertRaises(ValueError):
                settings_from_payload(dict(SETTINGS,**patching))
        with self.assertRaises(ValueError):
            settings_from_payload(dict(SETTINGS,disable_safety=True))

    def test_key_not_in_snapshot_and_absent_key_prevents_spawn(self):
        manager=TrialManager('test-private-key')
        self.assertNotIn('test-private-key',json.dumps(manager.snapshot()))
        manager.key=None
        with self.assertRaisesRegex(ValueError,'api_key_unavailable'):
            manager.start(SETTINGS)

    def test_duplicate_start_and_wrong_session_cannot_take_ownership(self):
        manager=TrialManager('test-private-key')
        manager.state.update(active=True,session_id='owner')
        with self.assertRaisesRegex(ValueError,'already_running'):
            manager.start(SETTINGS)
        for method in [manager.stop,manager.heartbeat]:
            with self.assertRaisesRegex(ValueError,'session_mismatch'):
                method('other')
        self.assertFalse(manager.stop_event.is_set())
        manager.stop('owner')
        self.assertTrue(manager.stop_event.is_set())

    def test_unconfirmed_stop_prevents_another_trial(self):
        manager=TrialManager('test-private-key')
        manager.inhibit_reason='executor_stop_not_confirmed'
        with self.assertRaisesRegex(ValueError,'executor_stop_not_confirmed'):
            manager.start(SETTINGS)


class SensorTests(unittest.TestCase):
    def test_only_the_read_only_telemetry_script_is_pulled(self):
        manager=TrialManager('test-private-key')
        with tempfile.TemporaryDirectory() as folder:
            run=Path(folder)/'trial';run.mkdir()
            spawned=[]
            with patch.object(manager,'_read',return_value={}), \
                 patch.object(manager,'_spawn',side_effect=lambda command,log: spawned.append((command,log)) or object()):
                manager._sensors(run,10.)
        self.assertEqual(len(spawned),1)
        command,log=spawned[0]
        self.assertIn('telemetry_readonly.sh',' '.join(command))
        self.assertNotIn('perception.sh',' '.join(command))
        self.assertEqual(log.name,'telemetry.log')

    def test_fresh_telemetry_does_not_spawn_another_subscriber(self):
        manager=TrialManager('test-private-key')
        with tempfile.TemporaryDirectory() as folder:
            run=Path(folder)/'trial';run.mkdir()
            fresh=dict(status='running',timestamp_ns=time.monotonic_ns())
            with patch.object(manager,'_read',return_value=fresh), \
                 patch.object(manager,'_spawn',side_effect=AssertionError('no spawn expected')):
                manager._sensors(run,10.)
        self.assertEqual(manager.sensors,[])


class FakeProcess:
    def __init__(self):
        self.returncode=None
        self.signals=[]
    def poll(self):
        return self.returncode
    def send_signal(self,value):
        self.signals.append(value);self.returncode=0
    def wait(self,timeout=None):
        return self.returncode


class LifecycleTests(unittest.TestCase):
    def run_case(self,mode,ready,stop_by_user):
        with tempfile.TemporaryDirectory(prefix='trial-test-',dir=ROOT/'cache/tmp') as folder:
            root=Path(folder);(root/'runs').mkdir();(root/'config').mkdir()
            (root/'config/deepseek_task.json').write_text(json.dumps({'provider':{}}))
            manager=TrialManager('test-private-key');manager.state.update(session_id='session',active=True,mode=mode)
            manager.last_heartbeat=time.monotonic() if stop_by_user else 0.
            processes=[];commands=[];sensor_durations=[]
            def spawn(command,*args,**kwargs):
                commands.append(list(command));proc=FakeProcess();processes.append(proc);return proc
            packet={'decision':{'startup':{'first_fusion_complete':ready,'phase':'ready' if ready else 'waiting_deepseek'},
                                'execution_blockers':['unobserved_swept_corridor']}}
            if stop_by_user:
                timer=threading.Timer(.2,lambda:manager.stop('session'));timer.start()
            from vlx_async import trial_control
            def atomic(path,value):
                path.write_text(json.dumps(value))
            with (patch.object(trial_control,'ROOT',root),patch.object(trial_control,'atomic_json',atomic),
                 patch.object(manager,'_spawn',side_effect=spawn),
                 patch.object(manager,'_sensors',side_effect=lambda _run,duration:sensor_durations.append(duration)),
                 patch.object(manager,'_packet',return_value=packet),
                 patch.object(manager,'_execution_reason',return_value='geometry_or_timing_unverified')):
                manager._run('session',dict(SETTINGS,mode=mode))
            config=json.loads(next((root/'runs').glob('trial_*/runner_config.json')).read_text())
            self.assertNotIn('max_candidate_age_s',config.get('fusion',{}))
            self.assertNotIn('max_candidate_age_s',config['execution_preview_limits'])
            if stop_by_user:timer.join()
            self.assertEqual(len(processes),1,'execution must never spawn before every gate passes')
            self.assertTrue(processes[0].signals)
            self.assertFalse(manager.state['active'])
            self.assertFalse(manager.state['execution_started'])
            runner=commands[0]
            # The task window now covers the motion window; a short keep-alive
            # only lets the runner publish the completed task before exit.
            duration_index=runner.index('--duration')+1
            self.assertEqual(float(runner[duration_index]),
                             max(SETTINGS['model_duration_s'],SETTINGS['motion_duration_s']))
            # Separate windows: planning from the form, motion from the form.
            self.assertEqual(config['task']['plan_timeout_s'],SETTINGS['model_duration_s'])
            self.assertEqual(config['task']['task_timeout_s'],SETTINGS['motion_duration_s'])
            # One plan per task is enforced twice: worker and provider budget.
            self.assertEqual(config['provider']['max_calls'],1)
            if mode=='execute':
                self.assertIn('--keep-alive-for',runner)
                keep_index=runner.index('--keep-alive-for')+1
                self.assertEqual(float(runner[keep_index]),20.)
                self.assertGreater(sensor_durations[0],SETTINGS['model_duration_s'])
            else:
                self.assertNotIn('--keep-alive-for',runner)
            return manager.state

    def test_heartbeat_loss_stops_model_process(self):
        state=self.run_case('shadow',False,False)
        self.assertEqual(state['reason'],'browser_heartbeat_lost')

    def test_explicit_stop_works_during_first_model_wait(self):
        state=self.run_case('execute',False,True)
        self.assertEqual(state['reason'],'user_stopped')

    def test_first_fusion_does_not_override_coverage_or_timing(self):
        state=self.run_case('execute',True,True)
        self.assertEqual(state['reason'],'user_stopped')


class FakeObserver:
    def snapshot(self):
        return {'schema':'test'}


class FakeController:
    token='test-csrf-token'
    def __init__(self):
        self.calls=[]
    def snapshot(self):
        return dict(active=False,control_token=self.token,api_key_ready=True)
    def start(self,payload):
        self.calls.append(('start',settings_from_payload(payload)))
        return dict(accepted=True,session_id='session')
    def stop(self,payload):
        self.calls.append(('stop',payload))
        return dict(accepted=True)
    def heartbeat(self,payload):
        self.calls.append(('heartbeat',payload))
        return dict(accepted=True)
    def configure_limits(self,payload):
        values=validate_saved_limits(payload)
        self.calls.append(('limits',values))
        return dict(accepted=True,settings=values,applies='next_test')


class ControlHTTPTests(unittest.TestCase):
    def setUp(self):
        self.control=FakeController()
        self.server=ObservationServer(('127.0.0.1',0),[],FakeObserver(),self.control)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()
        self.url='http://127.0.0.1:'+str(self.server.server_port)
    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join()
    def post(self,path,payload,headers=None):
        h={'Origin':self.url,'Content-Type':'application/json','X-Test-Token':self.control.token}
        h.update(headers or {})
        return urllib.request.urlopen(urllib.request.Request(self.url+path,data=json.dumps(payload).encode(),headers=h,method='POST'))
    def test_get_never_starts_a_test(self):
        for path in ['/','/trial_controls.js','/observe.json']:
            with urllib.request.urlopen(self.url+path) as response:
                self.assertEqual(response.status,200)
        self.assertEqual(self.control.calls,[])
    def test_authorized_start_stop_and_heartbeat(self):
        with self.post('/test/start',SETTINGS) as response:
            self.assertTrue(json.load(response)['accepted'])
        for path in ['/test/heartbeat','/test/stop']:
            with self.post(path,{'session_id':'session'}) as response:
                self.assertEqual(response.status,200)
        self.assertEqual([v[0] for v in self.control.calls],['start','heartbeat','stop'])
    def test_cross_origin_or_missing_token_cannot_start(self):
        for h in [{'Origin':'http://evil.example'},{'X-Test-Token':''},{'Host':'evil.example'},{'Content-Type':'text/plain'}]:
            with self.assertRaises(urllib.error.HTTPError) as caught:
                self.post('/test/start',SETTINGS,h)
            self.assertEqual(caught.exception.code,403)
        self.assertEqual(self.control.calls,[])
    def test_unknown_control_route_never_proxies_original(self):
        with self.assertRaises(urllib.error.HTTPError) as caught:
            self.post('/manual/action',{'action':'vlx_start'})
        self.assertEqual(caught.exception.code,405)
        self.assertEqual(self.control.calls,[])
    def test_limit_save_is_authenticated_and_does_not_start_motion(self):
        with self.post('/test/limits',{'observed_speed_limit_mps':.2}) as response:
            self.assertEqual(json.load(response)['settings']['observed_speed_limit_mps'],.2)
        self.assertEqual(self.control.calls,[('limits',{'observed_speed_limit_mps':.2})])
        for headers in [{'Origin':'http://evil.example'},{'X-Test-Token':''}]:
            with self.assertRaises(urllib.error.HTTPError) as caught:
                self.post('/test/limits',{'observed_speed_limit_mps':.2},headers)
            self.assertEqual(caught.exception.code,403)
        with self.assertRaises(urllib.error.HTTPError) as caught:
            self.post('/test/limits',{'observed_speed_limit_mps':-1})
        self.assertEqual(caught.exception.code,400)


if __name__=='__main__':
    unittest.main()
