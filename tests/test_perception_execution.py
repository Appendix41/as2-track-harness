"""Execution gate tests after the depth chain removal.

The depth/target/obstacle evidence that used to gate commands is gone; what
remains is the frozen task plan plus the robot/remote safety vetoes.  The
helpers here are also imported by the other execution-related tests.
"""
from dataclasses import replace, asdict
import json
import tempfile
import time
import unittest

from vlx_async.contracts import Context
from vlx_async.execution import CommandGate, Limits, GuardedActuator, ControllerLease, packet_observation
from vlx_async.replay import synthetic_observation


def executable_inputs():
    """A fresh packet that the plan gate and robot vetoes both accept."""
    obs=synthetic_observation(20);now=obs.camera_ns
    ctx=Context(pose=obs.context.pose,body_transform_verified=True)
    obs=replace(obs,context=ctx,waypoints=tuple((i*.03,0.,0.) for i in range(8)))
    d=dict(task_execution_ready=True,reason='task_ready',recommended=list(obs.waypoints),
           camera_index=obs.camera_index,prediction_index=obs.prediction_index,stream_id=obs.stream_id,
           host_boot_id=obs.boot_id,instruction_revision=obs.revision)
    tele=dict(sport=dict(received_monotonic_ns=now,mode=3,error_code=0,rpy=[0.,0.,0.],velocity=[0.,0.,0.]),
              low=dict(received_monotonic_ns=now,mode_machine=1),remote_axes=dict(lx=0.,ly=0.,rx=0.,ry=0.,keys=0),
              original_manual_enabled=False,original_vlx_active=False,error=None)
    return d,obs,tele,now


class ExecutionTests(unittest.TestCase):
    def test_packet_preserves_stamps_and_rejects_reboot_future_finished(self):
        d,o,t,n=executable_inputs()
        packet=dict(schema='vlx-fusion-execution-packet-v1',status='shadow',timestamp_ns=n,
                    decision=d,observation=asdict(o),telemetry=t)
        result=packet_observation(json.loads(json.dumps(packet)),n,o.boot_id)
        self.assertEqual(result.camera_ns,o.camera_ns)
        self.assertEqual(result.context.pose.timestamp_ns,o.context.pose.timestamp_ns)
        for change in [dict(status='finished'),dict(timestamp_ns=n+1),dict(timestamp_ns=n-250_000_000)]:
            with self.assertRaises(ValueError):packet_observation(dict(packet,**change),n,o.boot_id)
        with self.assertRaises(ValueError):packet_observation(packet,n,'other-boot')

    def test_limits_and_acceleration_are_independent_of_original_web_settings(self):
        d,o,t,n=executable_inputs();g=CommandGate();r=g.evaluate(d,o,t,n)
        self.assertTrue(r['allowed']);self.assertAlmostEqual(r['command_mps_rps'][0],.005)
        self.assertFalse(r['execution_enabled'])
        self.assertEqual(Limits(max_forward_mps=5.).max_forward_mps,5.)
        # Depth evidence age limits are gone with the depth chain.
        for removed in ('max_scene_age_s','max_candidate_age_s','footprint_radius_m'):
            with self.subTest(removed=removed),self.assertRaises(TypeError):
                Limits(**{removed:5.})

    def test_reverse_plan_turns_in_place_then_drives(self):
        d,o,t,n=executable_inputs()
        turn=dict(d,rotate_in_place=True,chosen_angle_rad=-1.)
        r=CommandGate().evaluate(turn,o,t,n)
        self.assertTrue(r['allowed'],r)
        self.assertEqual(r['command_mps_rps'][0],0.)
        self.assertEqual(r['command_mps_rps'][1],0.)
        self.assertLess(r['command_mps_rps'][2],0.)
        straight=dict(d,rotate_in_place=False)
        r=CommandGate().evaluate(straight,o,t,n+50_000_000)
        self.assertTrue(r['allowed'],r)
        self.assertGreater(r['command_mps_rps'][0],0.)

    def test_task_plan_is_the_only_execution_authority(self):
        d,o,t,n=executable_inputs()
        self.assertTrue(CommandGate().evaluate(d,o,t,n)['allowed'])
        pending=dict(d,task_execution_ready=False,reason='plan_pending')
        r=CommandGate().evaluate(pending,o,t,n)
        self.assertFalse(r['allowed']);self.assertEqual(r['reason'],'plan_pending')
        self.assertFalse(r['hold_for_perception']);self.assertEqual(r['command_mps_rps'],[0.,0.,0.])
        missing={k:v for k,v in d.items() if k!='task_execution_ready'}
        self.assertEqual(CommandGate().evaluate(missing,o,t,n)['reason'],'plan_not_ready')
        for reason in ('plan_requires_reverse','route_drift','low_plan_confidence','no_feasible_candidate',
                       'plan_rejected:low_confidence'):
            blocked=dict(d,task_execution_ready=False,reason=reason)
            self.assertEqual(CommandGate().evaluate(blocked,o,t,n)['reason'],reason)

    def test_route_completed_is_a_clean_stop_not_a_fault(self):
        d,o,t,n=executable_inputs()
        done=dict(d,task_execution_ready=False,reason='route_completed')
        r=CommandGate().evaluate(done,o,t,n)
        self.assertFalse(r['allowed']);self.assertEqual(r['reason'],'route_completed')
        self.assertFalse(r['hold_for_perception']);self.assertEqual(r['command_mps_rps'],[0.,0.,0.])

    def test_stale_clock_remote_and_conflicting_owner_stop(self):
        d,o,t,n=executable_inputs()
        variants=[(dict(d,camera_index=o.camera_index+1),o,t,n),
                  (dict(d,task_execution_ready=False),o,t,n),
                  (d,o,dict(t,original_vlx_active=True),n),
                  (d,o,dict(t,remote_axes=dict(t['remote_axes'],keys=1)),n),
                  (d,o,t,n+500_000_000)]
        for args in variants:
            r=CommandGate().evaluate(*args)
            self.assertFalse(r['allowed'],r);self.assertEqual(r['command_mps_rps'],[0.,0.,0.])

    def test_time_alignment_and_near_field_coverage_no_longer_block_commands(self):
        d,o,t,n=executable_inputs()
        for context in (replace(o.context,time_sync_verified=False,coverage_verified=False),
                        replace(o.context,observed_free_cells=())):
            result=CommandGate().evaluate(d,replace(o,context=context),t,n)
            self.assertTrue(result['allowed'],result)

    def test_control_producer_gap_is_rejected(self):
        d,o,t,n=executable_inputs();g=CommandGate();self.assertTrue(g.evaluate(d,o,t,n)['allowed'])
        self.assertEqual(g.evaluate(d,o,t,n+250_000_000)['reason'],'control_watchdog_gap')

    def test_new_trial_cannot_acquire_original_controller_lock(self):
        with tempfile.NamedTemporaryFile() as f:
            a=ControllerLease(f.name).acquire();b=ControllerLease(f.name)
            try:
                with self.assertRaises(BlockingIOError):b.acquire()
            finally:a.close();b.close()
            b.acquire();b.close()

    def test_watchdog_stops_when_producer_disappears(self):
        class Sink:
            def __init__(self):self.moves=[];self.stops=0
            def move(self,*args):self.moves.append(args);return 0
            def stop(self):self.stops+=1;return 0
        sink=Sink();act=GuardedActuator(sink,duration_s=1.,watchdog_s=.05)
        try:
            self.assertTrue(act.submit(dict(allowed=True,command_mps_rps=[.005,0.,0.])))
            deadline=time.monotonic()+.5
            while not sink.stops and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(sink.stops,1)
            self.assertFalse(act.submit(dict(allowed=True,command_mps_rps=[.005,0.,0.])))
            self.assertEqual(len(sink.moves),1)
        finally:act.close()

    def test_rpc_failure_stops_retries_and_does_not_rearm(self):
        class Sink:
            def __init__(self):self.stops=0;self.moves=0
            def move(self,*args):self.moves+=1;return -1
            def stop(self):self.stops+=1;return -1 if self.stops==1 else 0
        sink=Sink();act=GuardedActuator(sink)
        try:
            self.assertFalse(act.submit(dict(allowed=True,command_mps_rps=[.005,0.,0.])))
            self.assertEqual(act.stop_codes,[-1,0]);self.assertEqual(sink.stops,2)
            self.assertFalse(act.submit(dict(allowed=True,command_mps_rps=[.005,0.,0.])))
        finally:act.close()


if __name__=='__main__':unittest.main()
