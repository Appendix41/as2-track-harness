from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from vlx_async import runtime_limits as config
from vlx_async.execution import Limits, CommandGate
from vlx_async.fusion_step import StepLimits, StepController
from vlx_async.trial_control import TrialManager, settings_from_payload
from test_perception_execution import executable_inputs
from test_trial_control import SETTINGS


class RuntimeLimitTests(unittest.TestCase):
    def test_replay_previous_speed_at_two_operator_limits(self):
        initial=dict(position=[0.,0.,0.],rpy=[0.,0.,0.],velocity=[0.,0.,0.],yaw_speed=0.)
        current=dict(initial,velocity=[.152,0.,0.])
        prepared=dict(command_mps_rps=[.03,0.,0.])
        with self.assertRaisesRegex(ValueError,'observed_speed_limit'):
            StepController(prepared,initial,1_000_000_000).evaluate(current,1_050_000_000)
        control=StepController(prepared,initial,1_000_000_000,StepLimits(max_observed_speed_mps=.2))
        self.assertTrue(control.evaluate(current,1_050_000_000)['allowed'])
        with self.assertRaisesRegex(ValueError,'observed_speed_limit'):
            control.evaluate(dict(current,velocity=[.21,0.,0.]),1_100_000_000)
        with self.assertRaises(ValueError):control.evaluate(initial,1_150_000_000)

    def test_web_command_gate_uses_same_limit_and_retains_other_gates(self):
        d,o,t,n=executable_inputs();t['sport']['velocity']=[.152,0.,0.]
        self.assertEqual(CommandGate().evaluate(d,o,t,n)['reason'],'observed_speed_limit')
        limits=Limits(max_observed_speed_mps=.2)
        self.assertTrue(CommandGate(limits).evaluate(d,o,t,n)['allowed'])
        changed=dict(t,remote_axes=dict(t['remote_axes'],keys=1))
        self.assertFalse(CommandGate(limits).evaluate(d,o,changed,n)['allowed'])
        self.assertTrue(CommandGate(limits).evaluate(d,replace(o,context=replace(o.context,time_sync_verified=False)),t,n)['allowed'])

    def test_finite_bounded_values_required_everywhere(self):
        for value in [True,None,'0.2',float('nan'),float('inf'),0.,.009,config.SDK_FLOAT_MAX*2]:
            for validate in [config.observed_speed_limit,lambda v:StepLimits(max_observed_speed_mps=v),
                             lambda v:Limits(max_observed_speed_mps=v),
                             lambda v:settings_from_payload(dict(SETTINGS,observed_speed_limit_mps=v))]:
                with self.subTest(value=value,validate=validate):
                    with self.assertRaises(ValueError):validate(value)
        for value in [.01,.05,.12,.2,.5,5.,20.]:self.assertEqual(config.observed_speed_limit(value),value)

    def test_save_persists_without_starting_and_cannot_change_active_run(self):
        with tempfile.TemporaryDirectory(dir=config.ROOT/'cache/tmp') as folder:
            with patch.object(config,'LIMITS_PATH',Path(folder)/'limits.json'):
                manager=TrialManager('private-key')
                self.assertEqual(manager.saved_limits['observed_speed_limit_mps'],.12)
                with patch.object(manager,'_spawn',side_effect=AssertionError('no subprocess expected')):
                    result=manager.configure_limits({'observed_speed_limit_mps':.2})
                self.assertEqual(result['applies'],'next_test')
                self.assertFalse(manager.state['active']);self.assertIsNone(manager.thread)
                self.assertEqual(TrialManager('private-key').saved_limits['observed_speed_limit_mps'],.2)
                manager.state['active']=True
                with self.assertRaisesRegex(ValueError,'stop_test_before_changing_limits'):
                    manager.configure_limits({'observed_speed_limit_mps':.3})
                self.assertEqual(config.load_saved_limits()['observed_speed_limit_mps'],.2)
                self.assertNotIn('private-key',json.dumps(manager.snapshot()))

    def test_start_snapshots_saved_limit_without_changing_saved_value(self):
        with tempfile.TemporaryDirectory(dir=config.ROOT/'cache/tmp') as folder:
            with patch.object(config,'LIMITS_PATH',Path(folder)/'limits.json'):
                manager=TrialManager('private-key');manager.configure_limits({'observed_speed_limit_mps':.2})
                with patch('vlx_async.trial_control.threading.Thread'):
                    manager.start(SETTINGS)
                self.assertEqual(manager.state['settings']['observed_speed_limit_mps'],.2)
                config.save_limits({'observed_speed_limit_mps':.3})
                self.assertEqual(manager.state['settings']['observed_speed_limit_mps'],.2)

    def test_invalid_saved_file_does_not_silently_expand_limit(self):
        with tempfile.TemporaryDirectory(dir=config.ROOT/'cache/tmp') as folder:
            path=Path(folder)/'limits.json';path.write_text('{"observed_speed_limit_mps":null}')
            with patch.object(config,'LIMITS_PATH',path):
                with self.assertRaises(ValueError):config.load_saved_limits()

if __name__=='__main__':unittest.main()
