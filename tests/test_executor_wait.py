"""Exercise startup races and temporary stops without creating a robot client."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from vlx_async.execution import CommandGate, GuardedActuator, wait_for_command
from test_perception_execution import executable_inputs


def waiting():
    return dict(allowed=False, reason='camera_frame_stale',
                hold_for_perception=True, command_mps_rps=[0., 0., 0.])


def pending():
    return dict(allowed=False, reason='plan_pending',
                hold_for_perception=False, command_mps_rps=[0., 0., 0.])


def valid():
    return dict(allowed=True, reason='bounded_command_validated', command_mps_rps=[.005, 0., 0.])


class Sink:
    def __init__(self, *_):
        self.events = []

    def move(self, *values):
        self.events.append(('move', values))
        return 0

    def stop(self):
        self.events.append(('stop',))
        return 0


class ExecutorWaitTests(unittest.TestCase):
    def test_spawn_race_waits_for_a_new_valid_frame(self):
        clock = [1.]
        read = Mock(side_effect=[(1, waiting()), ValueError('execution_packet_stale_or_inactive'), (3, valid())])
        reports = []
        result = wait_for_command(read, 2., lambda: False, reports.append,
                                  clock=lambda: clock[0], pause=lambda dt: clock.__setitem__(0, clock[0]+dt))
        self.assertEqual(result, (3, valid()))
        self.assertEqual(reports, ['camera_frame_stale', 'execution_packet_stale_or_inactive'])

    def test_plan_pending_is_retryable_but_not_a_perception_hold(self):
        clock = [1.]
        read = Mock(side_effect=[(1, pending()), (2, valid())])
        reports = []
        result = wait_for_command(read, 2., lambda: False, reports.append,
                                  clock=lambda: clock[0], pause=lambda dt: clock.__setitem__(0, clock[0]+dt))
        self.assertEqual(result, (2, valid()))
        self.assertEqual(reports, ['plan_pending'])
        decision, obs, telemetry, now = executable_inputs()
        decision = dict(decision, task_execution_ready=False, reason='plan_pending')
        command = CommandGate().evaluate(decision, obs, telemetry, now)
        self.assertFalse(command['allowed'])
        self.assertEqual(command['reason'], 'plan_pending')
        self.assertFalse(command['hold_for_perception'])
        self.assertEqual(command['command_mps_rps'], [0., 0., 0.])

    def test_startup_wait_is_cancellable_bounded_and_rejects_other_sessions(self):
        with self.assertRaisesRegex(ValueError, 'operator_stopped'):
            wait_for_command(Mock(), 1., lambda: True, Mock())
        with self.assertRaisesRegex(ValueError, 'execution_session_mismatch'):
            wait_for_command(Mock(side_effect=ValueError('execution_session_mismatch')), 1., lambda: False, Mock())
        clock = [0.]
        with self.assertRaisesRegex(ValueError, 'current_fusion_wait_timeout'):
            wait_for_command(lambda: (1, waiting()), .05, lambda: False, Mock(),
                             clock=lambda: clock[0], pause=lambda dt: clock.__setitem__(0, clock[0]+dt))

    def test_perception_hold_brakes_without_stopping_the_gait_then_resumes(self):
        sink = Sink()
        actuator = GuardedActuator(sink, allow_perception_hold=True)
        try:
            self.assertTrue(actuator.submit(valid()))
            self.assertFalse(actuator.submit(waiting()))
            self.assertTrue(actuator.holding)
            self.assertIsNone(actuator.fault)
            actuator.submit(waiting())
            self.assertEqual([x[0] for x in sink.events], ['move', 'move', 'move'])
            self.assertNotIn('stop', [x[0] for x in sink.events])
            self.assertTrue(actuator.submit(valid()))
            self.assertFalse(actuator.holding)
            self.assertEqual([x[0] for x in sink.events], ['move', 'move', 'move', 'move'])
        finally:
            actuator.close()

    def test_hold_move_failure_is_latched_and_default_actuator_does_not_resume(self):
        for allow_hold in (True, False):
            sink = Sink()
            if allow_hold:
                sink.move = Mock(return_value=-1)
            actuator = GuardedActuator(sink, allow_perception_hold=allow_hold)
            try:
                actuator.submit(waiting())
                self.assertIsNotNone(actuator.fault)
                self.assertFalse(actuator.submit(valid()))
                self.assertEqual(actuator.sent, 0)
            finally:
                actuator.close()

    def test_watchdog_and_remote_takeover_cannot_be_hidden_by_perception_hold(self):
        sink = Sink(); clock = [1.]
        actuator = GuardedActuator(sink, allow_perception_hold=True, clock=lambda: clock[0])
        try:
            actuator.submit(waiting())
            clock[0] += .3
            actuator.submit(valid())
            self.assertEqual(actuator.fault, 'control_watchdog_expired')
            self.assertEqual(actuator.sent, 0)
        finally:
            actuator.close()
        decision, obs, telemetry, now = executable_inputs()
        decision = dict(decision, task_execution_ready=False, reason='plan_pending')
        telemetry['remote_axes']['keys'] = 1
        command = CommandGate().evaluate(decision, obs, telemetry, now)
        self.assertEqual(command['reason'], 'remote_input_or_unknown_state')
        self.assertFalse(command['hold_for_perception'])

    def test_late_prediction_stops_without_replaying_it_then_resumes_on_fresh_data(self):
        from dataclasses import replace
        decision, obs, telemetry, now = executable_inputs()
        gate = CommandGate()
        stale = gate.evaluate(decision, replace(obs, prediction_ns=now-400_000_000), telemetry, now)
        self.assertEqual(stale['reason'], 'prediction_stale')
        self.assertTrue(stale['hold_for_perception'])
        self.assertEqual(stale['command_mps_rps'], [0., 0., 0.])
        next_ns = now+50_000_000
        fresh = gate.evaluate(decision, replace(obs, prediction_ns=next_ns), telemetry, next_ns)
        self.assertTrue(fresh['allowed'])
        self.assertLessEqual(fresh['command_mps_rps'][0], .005+1e-12)
        telemetry['remote_axes']['keys'] = 1
        command = CommandGate().evaluate(decision, replace(obs, prediction_ns=now-400_000_000), telemetry, now)
        self.assertEqual(command['reason'], 'remote_input_or_unknown_state')
        self.assertFalse(command['hold_for_perception'])

    def test_actual_executor_entry_waits_handoffs_holds_and_sends_to_fake_sink(self):
        source = Path(__file__).resolve().parents[1]/'scripts/fusion_trial.py'
        spec = importlib.util.spec_from_file_location('fusion_trial_wait_test', source)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        sink = Sink()
        handoff = Mock(released=False)
        handoff.lease = Mock(file=object())
        commands = [waiting(), valid(), waiting(), valid(), waiting(), valid()]
        gate = Mock()
        gate.evaluate.side_effect = lambda *_: commands.pop(0) if commands else valid()
        with tempfile.TemporaryDirectory(dir=source.parents[1]/'runs') as directory:
            root = Path(directory); (root/'runs').mkdir()
            (root/'runs/fusion_execution_packet.json').write_text(json.dumps(
                dict(session_id='test', decision={}, telemetry={})))
            argv = ['fusion_trial.py', '--execute', '--session-id', 'test', '--duration', '.22',
                    '--ready-timeout', '1', '--observed-speed-limit', '.12']
            with patch.object(module,'ROOT',root), patch.object(module.sys,'argv',argv), \
                 patch.object(module,'ControllerHandoff',return_value=handoff), \
                 patch.object(module,'AS2Sink',return_value=sink), \
                 patch.object(module,'CommandGate',return_value=gate), \
                 patch.object(module,'packet_observation',return_value=object()), \
                 patch.object(module.signal,'signal'), patch('builtins.print'):
                result = module.main()
            summary = json.loads(next((root/'runs').glob('execution_*/summary.json')).read_text())
        self.assertEqual(result, 0, summary)
        handoff.acquire.assert_called_once()
        self.assertGreaterEqual(summary['motion_commands_sent'], 2)
        self.assertEqual(summary['perception_holds'], 1)
        self.assertTrue(summary['stop_confirmed'])
        # A hold keeps the gait alive now: only the final stop closes the motion.
        self.assertEqual([x[0] for x in sink.events[:4]], ['stop', 'move', 'move', 'move'])


if __name__ == '__main__':
    unittest.main()
