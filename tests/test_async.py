from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import tempfile
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch
import urllib.request
import urllib.error

from vlx_async.contracts import Context, LocalCandidate, FusionConfig, Pose, Request, SlowPlan, points
from vlx_async.dashboard import Dashboard
from vlx_async.fusion import Fusion
from vlx_async.geometry import align, collision_free, project_image_path, remaining
from vlx_async.providers import MockProvider, ResponsesProvider, SlowWorker, decode_plan, metric_plan, local_api_settings
from vlx_async.replay import synthetic_observation, delay_experiment
from vlx_async.source import FastGate, PoseBuffer, ReadOnlySource, context_from_file
from vlx_async.startup import StartupBarrier
from vlx_async.storage import ROOT, confined
from vlx_async.task_plan import RoutePlan


def route_plan(obs, points=None, **changes):
    data=dict(request_id='plan-1',task_key=obs.task_key,
              points=tuple(points or [(i*.5,0.,0.) for i in range(9)]),
              confidence=1.,description='synthetic ground route',provider='synthetic',
              created_ns=obs.camera_ns,pose=obs.context.pose)
    data.update(changes)
    return RoutePlan(**data)


def envelope(obs):
    return {'schema':'vlx-control-v1','host_boot_id':obs.boot_id,'stream_id':obs.stream_id,
            'status':'running','requested_instruction':obs.instruction,'requested_instruction_revision':obs.revision,
            'record':{'prediction_index':obs.prediction_index,'camera_frame_index':obs.camera_index,
                      'instruction':obs.instruction,'instruction_revision':obs.revision,'valid_history_frames':31,
                      'camera_received_monotonic_ns':obs.camera_ns,
                      'prediction_available_monotonic_ns':obs.prediction_ns,'waypoints':obs.waypoints}}


class GeometryTests(unittest.TestCase):
    def test_robot_translation_compensated(self):
        a=Pose(0,0,0,1,'world');b=Pose(1,0,0,2,'world')
        self.assertEqual(align([(3,2,0)],a,b),[(2,2,0)])

    def test_robot_turn_compensated(self):
        a=Pose(0,0,0,1,'world');b=Pose(0,0,math.pi/2,2,'world')
        p=align([(2,0,0)],a,b)[0]
        self.assertAlmostEqual(p[0],0);self.assertAlmostEqual(p[1],-2)
        self.assertAlmostEqual(p[2],-math.pi/2)

    def test_roundtrip_preserves_route(self):
        a=Pose(3,-2,1,1,'w');b=Pose(-1,4,-2,2,'w');route=[(2,1,.3),(4,-.5,-.4)]
        back=align(align(route,a,b),b,a)
        for p,q in zip(route,back):
            for x,y in zip(p,q):self.assertAlmostEqual(x,y)

    def test_odometry_reset_rejected(self):
        with self.assertRaisesRegex(ValueError,'frame_changed'):
            align([(1,0,0)],Pose(0,0,0,1,'a'),Pose(0,0,0,2,'b'))

    def test_remaining_crops_passed_segment(self):
        path,d=remaining([(-2,0,0),(-1,0,0),(1,0,0),(3,0,0)])
        self.assertEqual(path,[(0.,0.,0.),(1,0,0),(3,0,0)])
        self.assertEqual(d,0)

    def test_exhausted_route(self):
        self.assertEqual(remaining([(-2,0,0),(-1,0,0)])[0],[])

    def test_collision_between_sparse_points(self):
        self.assertFalse(collision_free([(2,0,0)],[(1,0,.1)],.2))
        self.assertTrue(collision_free([(2,1,0)],[(1,-1,.1)],.2))

    def test_projection_requires_finite_in_front(self):
        h=[[1,0,0],[0,1,0],[0,0,1]]
        self.assertEqual(project_image_path([(0,0,0),(1,0,0)],h)[-1],(1.,0.,0.))
        with self.assertRaises(ValueError):project_image_path([(0,0,0)],[[1,0,0],[0,1,0],[0,0,0]])


class FusionTests(unittest.TestCase):
    def setUp(self):
        self.old=synthetic_observation(0)
        self.current=synthetic_observation(1)
        self.fusion=Fusion()

    def loaded(self,plan=None,current=None):
        current=current or self.current
        plan=plan or route_plan(self.old)
        self.assertIsNone(self.fusion.accept(plan,current,current.camera_ns))
        return self.fusion.decide(current,current.camera_ns)

    def test_before_plan_requests_hold(self):
        r=self.fusion.decide(self.old,self.old.camera_ns)
        self.assertEqual(r['reason'],'plan_pending')
        self.assertEqual(r['selection'],'hold')
        self.assertFalse(r['task_execution_ready'])
        self.assertEqual(r['recommended'],list(self.old.waypoints))
        self.assertIsNone(r['plan_request_id'])

    def test_one_plan_per_task_then_task_ready(self):
        r=self.loaded()
        self.assertEqual(r['reason'],'task_ready')
        self.assertTrue(r['task_execution_ready'])
        self.assertEqual(r['plan_request_id'],'plan-1')
        self.assertEqual(r['plan_status'],'following')
        self.assertEqual(len(r['recommended']),8)
        self.assertIn(r['selection'],('plan_aligned','vlx_only'))
        self.assertGreater(len(r['plan_aligned']),1)
        self.assertEqual(len(r['candidates']),11)
        self.assertEqual(r['plan_weight_used'],FusionConfig().plan_weight)

    def test_second_plan_is_refused(self):
        self.assertIsNone(self.fusion.accept(route_plan(self.old),self.old,self.old.camera_ns))
        self.assertEqual(self.fusion.accept(route_plan(self.old),self.old,self.old.camera_ns),
                         'plan_already_active')

    def test_plan_for_another_task_is_obsolete(self):
        current=replace(self.current,revision=2)
        self.assertEqual(self.fusion.accept(route_plan(self.old),current,current.camera_ns),
                         'obsolete_task')
        self.assertIsNone(self.fusion.plan)

    def test_low_confidence_and_stale_plans_rejected(self):
        self.assertEqual(self.fusion.accept(route_plan(self.old,confidence=.4),self.old,self.old.camera_ns),
                         'low_plan_confidence')
        stale=route_plan(self.old,created_ns=1)
        self.assertEqual(self.fusion.accept(stale,self.old,self.old.camera_ns+1000*10**9),
                         'invalid_plan_clock')

    def test_plan_pose_frame_must_match(self):
        pose=replace(self.old.context.pose,frame_id='other_frame')
        self.assertEqual(self.fusion.accept(route_plan(self.old,pose=pose),self.old,self.old.camera_ns),
                         'plan_pose_unavailable')

    def test_short_route_rejected_and_reverse_route_allowed(self):
        short=route_plan(self.old,points=[(.1,0.,0.),(.2,0.,0.)])
        self.assertEqual(self.fusion.accept(short,self.old,self.old.camera_ns),'plan_route_too_short')
        # A target behind the robot is accepted now; the gate turns in place.
        reverse=route_plan(self.old,points=[(1.,0.,0.),(-1.,0.,0.)])
        self.assertIsNone(self.fusion.accept(reverse,self.old,self.old.camera_ns))

    def test_task_switch_discards_frozen_plan(self):
        self.loaded()
        newer=replace(self.current,revision=2,instruction='follow blue person')
        r=self.fusion.decide(newer,newer.camera_ns)
        self.assertEqual(r['reason'],'plan_pending')
        self.assertIsNone(r['plan_request_id'])

    def test_progress_follows_odometry_without_new_requests(self):
        accepted=self.loaded()
        later=synthetic_observation(1.5)
        advanced=self.fusion.decide(later,later.camera_ns)
        self.assertEqual(advanced['reason'],'task_ready')
        self.assertLess(advanced['remaining_route_m'],accepted['remaining_route_m'])
        self.assertGreater(advanced['progress_fraction'],accepted['progress_fraction'])
        self.assertEqual(advanced['plan_request_id'],accepted['plan_request_id'])

    def test_arrival_reports_route_completed(self):
        self.loaded()
        arrived=synthetic_observation(1,pose=Pose(4.,0.,0.,int(101.5e9),'synthetic_world'))
        r=self.fusion.decide(arrived,arrived.camera_ns)
        self.assertEqual(r['reason'],'route_completed')
        self.assertTrue(r['task_completed'])
        self.assertEqual(r['recommended'],[])
        self.assertFalse(r['task_execution_ready'])

    def test_sideways_drift_blocks_following(self):
        self.loaded()
        drifted=synthetic_observation(1,pose=Pose(0.,2.,0.,int(101.5e9),'synthetic_world'))
        r=self.fusion.decide(drifted,drifted.camera_ns)
        self.assertEqual(r['reason'],'route_drift')
        self.assertFalse(r['task_execution_ready'])

    def test_missing_pose_blocks_plan(self):
        self.assertIsNone(self.fusion.accept(route_plan(self.old),self.old,self.old.camera_ns))
        missing=replace(self.current,context=Context())
        r=self.fusion.decide(missing,missing.camera_ns)
        self.assertEqual(r['reason'],'pose_unavailable')
        self.assertEqual(r['plan_status'],'blocked')


class SourceTests(unittest.TestCase):
    def setUp(self):
        self.obs=synthetic_observation(1)
        self.state=envelope(self.obs)
        self.gate=FastGate(self.obs.boot_id)

    def test_valid_and_duplicate_allowed(self):
        self.assertIsNone(self.gate.check(self.state,self.obs.camera_ns))
        self.assertIsNone(self.gate.check(self.state,self.obs.camera_ns+10_000_000))

    def test_frame_staleness_not_relaxed_for_API(self):
        self.assertEqual(self.gate.check(self.state,self.obs.camera_ns+600_000_000),'camera_frame_stale')

    def test_prediction_staleness(self):
        self.assertEqual(self.gate.check(self.state,self.obs.camera_ns+400_000_000),'prediction_stale')

    def test_instruction_pending(self):
        self.state['requested_instruction_revision']=2
        self.assertEqual(self.gate.check(self.state,self.obs.camera_ns),'instruction_pending')

    def test_nan_waypoints_rejected(self):
        self.state['record']['waypoints']=[[float('nan'),0,0]]*8
        self.assertEqual(self.gate.check(self.state,self.obs.camera_ns),'invalid_waypoints')

    def test_order_regression(self):
        self.gate.check(self.state,self.obs.camera_ns)
        self.state['record']['prediction_index']-=1
        self.assertEqual(self.gate.check(self.state,self.obs.camera_ns),'prediction_order_regressed')

    def test_wrong_host(self):
        self.state['host_boot_id']='other'
        self.assertEqual(self.gate.check(self.state,self.obs.camera_ns),'different_host_or_boot')

    def test_source_denies_mutating_paths(self):
        source=ReadOnlySource()
        for path in ['/instruction','/manual/action','/stop','http://evil/']:
            with self.assertRaises(ValueError):source.get(path)

    def test_source_denies_remote_hosts(self):
        with self.assertRaises(ValueError):ReadOnlySource('http://10.0.0.5:8765')

    def test_pose_interpolation_wraps_yaw(self):
        buf=PoseBuffer()
        buf.add(Pose(0,0,math.radians(179),1_000_000_000,'world'))
        buf.add(Pose(2,0,math.radians(-179),1_100_000_000,'world'))
        p=buf.at(1_050_000_000)
        self.assertEqual(p.x,1);self.assertAlmostEqual(abs(p.yaw),math.pi)
        self.assertIsNone(buf.at(1_300_000_000))

    def test_output_cannot_touch_original_or_backup(self):
        for path in ['/var/tmp/outside-root.json',ROOT/'output/test.json']:
            with self.assertRaises(ValueError):confined(path)


class ProviderTests(unittest.TestCase):
    def setUp(self):
        self.obs=synthetic_observation(0)
        self.request=Request('r1',self.obs,self.obs.camera_ns)
        self.data={'request_id':'r1','coordinate_frame':'image_uv','waypoints':[{'u':.5,'v':.9},{'u':.6,'v':.5}],
                   'confidence':.8,'target_description':'far chair','target_id':None,'dynamic_target':False,'reason':'farther'}

    def test_model_image_points_not_metric(self):
        p=decode_plan(self.data,self.request,self.obs.camera_ns+1,'api')
        self.assertFalse(p.metric_verified)
        self.assertIs(metric_plan(p,{'verified':False}),p)

    def test_wrong_request_rejected(self):
        self.data['request_id']='wrong'
        with self.assertRaises(ValueError):decode_plan(self.data,self.request,self.obs.camera_ns+1,'api')

    def test_invented_track_id_rejected(self):
        self.data['target_id']='made_up'
        with self.assertRaisesRegex(ValueError,'invented'):decode_plan(self.data,self.request,self.obs.camera_ns+1,'api')

    def test_extra_fields_tolerated_and_gateway_shape_accepted(self):
        # Extra keys are ignored: the gateway adds its own metadata.
        self.data['execute']=True
        plan=decode_plan(self.data,self.request,self.obs.camera_ns+1,'api')
        self.assertEqual(len(plan.waypoints),2)
        # The gateway's own field names carry the same information.
        gateway={'tracking_id':None,'target':'far chair','confidence':.7,
                 'dynamic_objects':[],'points':[{'u':.4,'v':.9},{'u':.5,'v':.6}],
                 'note':'approach from the left'}
        plan=decode_plan(gateway,self.request,self.obs.camera_ns+1,'api')
        self.assertEqual(len(plan.waypoints),2)
        self.assertIn('far chair',plan.reason)
        self.assertFalse(plan.dynamic_target)
        # Values are still checked strictly.
        # A refusal decodes and is rejected later, not treated as a schema bug.
        refusal=decode_plan(dict(gateway,points=[],confidence=0.,note='target not visible'),
                            self.request,self.obs.camera_ns+1,'api')
        self.assertEqual(refusal.waypoints,())
        for broken in ({'points':[{'u':float('nan'),'v':.5},{'u':.5,'v':.6}]},
                       {'points':'not-a-list'},
                       {'confidence':1.5},
                       {'tracking_id':'made_up'}):
            with self.assertRaises(ValueError):
                decode_plan(dict(gateway,**broken),self.request,self.obs.camera_ns+1,'api')

    def test_API_requires_explicit_enable_and_key(self):
        with self.assertRaisesRegex(ValueError,'disabled'):ResponsesProvider({'model':'deepseek-v4.1-flash'})
        with patch.dict(os.environ,{},clear=True):
            with self.assertRaisesRegex(ValueError,'key_missing'):ResponsesProvider({'model':'deepseek-v4.1-flash'},True)

    def test_current_config_keeps_exact_model_endpoint_and_effort(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as tmp:
            config=Path(tmp)/'config.toml';auth=Path(tmp)/'auth.json'
            config.write_text('model = "deepseek-v4.1-flash"\nmodel_provider = "custom"\nmodel_reasoning_effort = "max"\n[model_providers.custom]\nbase_url = "https://example.test/v1"\nwire_api = "responses"\nrequires_openai_auth = true\n')
            auth.write_text(json.dumps({'auth_mode':'apikey','OPENAI_API_KEY':'test-secret'}))
            with patch.dict(os.environ,{},clear=True):settings,key=local_api_settings(config,auth)
            self.assertEqual(settings,{'model':'deepseek-v4.1-flash','base_url':'https://example.test/v1','reasoning_effort':'max'})
            self.assertEqual(key,'test-secret')
            self.assertNotIn('test-secret',json.dumps(settings))

    def test_completed_response_not_overwritten_before_consumption(self):
        worker=SlowWorker(MockProvider(0))
        self.assertTrue(worker.arm('task-a'))
        self.assertTrue(worker.submit(lambda:self.request))
        deadline=time.monotonic()+1
        while worker.state()['busy'] and time.monotonic()<deadline:time.sleep(.001)
        state=worker.state()
        self.assertEqual(state['task_key'],'task-a')
        self.assertEqual(state['calls_for_task'],1)
        self.assertFalse(worker.submit(lambda:self.request))
        self.assertIsNotNone(worker.take())
        self.assertFalse(worker.submit(lambda:self.request))
        self.assertEqual(worker.state()['rejected_reason'],'plan_already_submitted')
        worker.close()

    def test_payload_uses_same_observation_and_structured_response(self):
        with patch.dict(os.environ,{'DEEPSEEK_API_KEY':'test-only-not-a-real-key'}):
            provider=ResponsesProvider({'model':'deepseek-v4.1-flash'},True)
        request=replace(self.request,observation=replace(self.obs,image_base64='TEST'))
        payload=provider.payload(request)
        self.assertEqual(payload['text']['format']['type'],'json_schema')
        self.assertTrue(payload['text']['format']['strict'])
        self.assertFalse(payload['store'])
        self.assertEqual(payload['input'][1]['content'][1]['image_url'],'data:image/jpeg;base64,TEST')

    def test_slow_work_does_not_block_or_accumulate_requests(self):
        worker=SlowWorker(MockProvider(.12))
        worker.arm('task-a')
        start=time.monotonic()
        self.assertTrue(worker.submit(lambda:self.request))
        for _ in range(20):self.assertFalse(worker.submit(lambda:self.request))
        self.assertLess(time.monotonic()-start,.1)
        deadline=time.monotonic()+1
        while worker.state()['busy'] and time.monotonic()<deadline:time.sleep(.01)
        self.assertIsNotNone(worker.take());self.assertIsNone(worker.take());worker.close()

    def test_worker_error_consumes_task_call_and_new_task_can_retry(self):
        worker=SlowWorker(MockProvider(0))
        worker.arm('first-task')
        def bad():raise ValueError('missing_frame')
        worker.submit(bad)
        deadline=time.monotonic()+1
        while worker.state()['busy'] and time.monotonic()<deadline:time.sleep(.001)
        self.assertEqual(worker.state()['error'],'missing_frame')
        self.assertFalse(worker.submit(lambda:self.request))
        self.assertEqual(worker.state()['rejected_reason'],'plan_already_submitted')
        self.assertTrue(worker.arm('second-task'))
        self.assertTrue(worker.submit(lambda:self.request));worker.close()

    def test_lock_refuses_new_requests_until_task_changes(self):
        worker=SlowWorker(MockProvider(0))
        worker.arm(('boot','stream',1,'go to chair'))
        self.assertTrue(worker.submit(lambda:self.request))
        deadline=time.monotonic()+1
        while worker.state()['busy'] and time.monotonic()<deadline:time.sleep(.001)
        worker.take();worker.lock()
        self.assertFalse(worker.submit(lambda:self.request))
        self.assertEqual(worker.state()['rejected_reason'],'plan_locked')
        self.assertFalse(worker.arm(('boot','stream',1,'go to chair')))
        self.assertTrue(worker.arm(('boot','stream',1,'go to table')))
        self.assertTrue(worker.submit(lambda:self.request));worker.close()

    def test_submit_requires_explicit_arm(self):
        worker=SlowWorker(MockProvider(0))
        self.assertFalse(worker.submit(lambda:self.request))
        state=worker.state()
        self.assertEqual(state['rejected_reason'],'not_armed')
        self.assertFalse(state['armed']);self.assertIsNone(state['task_key'])
        for bad in (None,'',(),('boot',None,1,'x'),[1,2,3,4,5,6,7,8,9],{'a':1}):
            with self.assertRaises(ValueError):worker.arm(bad)
        self.assertTrue(worker.arm(['boot','stream',1,'go']))
        self.assertEqual(worker.state()['task_key'],('boot','stream',1,'go'))
        worker.close()

    def test_responses_budget_is_per_task_with_global_backstop(self):
        with patch.dict(os.environ,{'DEEPSEEK_API_KEY':'test-only-not-a-real-key'}):
            provider=ResponsesProvider({'model':'deepseek-v4.1-flash'},True)
        self.assertEqual(provider.call_limit,1)
        self.assertFalse(provider.exhausted)
        provider.begin_task('first')
        provider.calls+=1;provider.calls_for_task+=1
        self.assertTrue(provider.exhausted)
        provider.begin_task('second')
        self.assertFalse(provider.exhausted)
        provider.total_call_limit=1
        self.assertTrue(provider.exhausted)


def plan_decision(**changes):
    result=dict(host_boot_id='boot',stream_id='stream',instruction_revision=1,
                instruction='go to the far chair',reason='task_ready',
                task_execution_ready=True,plan_status='following',plan_request_id='plan-1',
                recommended=[[i*.1,0.,0.] for i in range(8)],last_rejection=None,
                task=dict(phase='executing',plan_request_id='plan-1',plan_calls=1))
    return dict(result,**changes)


class StartupPlanTests(unittest.TestCase):
    def setUp(self):
        self.clock=[10.]
        self.barrier=StartupBarrier('new-session',90,clock=lambda:self.clock[0])

    def test_departure_waits_for_the_first_validated_plan(self):
        self.assertEqual(self.barrier.update({},False)['phase'],'waiting_vlx')
        self.assertEqual(self.barrier.update(plan_decision(reason='plan_pending',
            task_execution_ready=False),True)['phase'],'waiting_plan')
        for change in [dict(task_execution_ready=False),dict(reason='plan_pending'),
                       dict(plan_request_id=None),dict(recommended=[]),
                       dict(recommended=[[0.,0.,0.]]*7)]:
            state=self.barrier.update(plan_decision(**change),True)
            self.assertEqual(state['phase'],'waiting_plan')
            self.assertFalse(state['plan_ready']);self.assertFalse(state['first_fusion_complete'])
        state=self.barrier.update(plan_decision(),True)
        self.assertEqual(state['phase'],'plan_ready')
        self.assertTrue(state['plan_ready']);self.assertTrue(state['first_fusion_complete'])
        self.assertEqual(state['plan_request_id'],'plan-1')
        self.assertEqual(state['first_request_id'],'plan-1')

    def test_ready_is_latched_and_task_change_rejects_session(self):
        self.assertTrue(self.barrier.update(plan_decision(),True)['plan_ready'])
        self.assertTrue(self.barrier.update(plan_decision(reason='plan_pending'),True)['plan_ready'])
        state=self.barrier.update(plan_decision(instruction_revision=2),True)
        self.assertEqual(state['phase'],'plan_rejected')
        self.assertEqual(state['reason'],'startup_task_changed')
        self.assertFalse(state['plan_ready'])

    def test_plan_timeout_is_latched(self):
        self.clock[0]=101.
        self.assertEqual(self.barrier.update(plan_decision(),True)['phase'],'plan_timeout')
        state=self.barrier.update(plan_decision(),True)
        self.assertEqual(state['reason'],'first_plan_timeout')
        self.assertFalse(state['plan_ready'])

    def test_rejected_plan_is_terminal_for_the_session(self):
        state=self.barrier.update(plan_decision(reason='plan_rejected:low_plan_confidence',
            task_execution_ready=False,plan_request_id=None),True)
        self.assertEqual(state['phase'],'plan_rejected')
        self.assertEqual(state['reason'],'plan_rejected:low_plan_confidence')
        self.assertFalse(self.barrier.update(plan_decision(),True)['plan_ready'])

    def test_validation_rejection_without_response_is_reported(self):
        state=self.barrier.update(plan_decision(reason='plan_pending',task_execution_ready=False,
            plan_request_id=None,last_rejection='low_plan_confidence'),True)
        self.assertEqual(state['phase'],'plan_rejected')
        self.assertEqual(state['reason'],'plan_rejected:low_plan_confidence')

    def test_obsolete_response_is_not_a_current_plan_rejection(self):
        state=self.barrier.update(plan_decision(reason='plan_pending',task_execution_ready=False,
            plan_request_id=None,last_rejection='obsolete_task'),True)
        self.assertEqual(state['phase'],'waiting_plan')

    def test_unusable_projected_plan_is_rejected(self):
        state=self.barrier.update(plan_decision(reason='plan_pending',task_execution_ready=False,
            plan_request_id=None,task=dict(phase='blocked',plan_calls=1,plan_request_id=None,
                blocked_reason='plan_projection_failed:image_point_near_horizon')),True)
        self.assertEqual(state['phase'],'plan_rejected')
        self.assertEqual(state['reason'],'plan_projection_failed:image_point_near_horizon')

    def test_separate_run_never_inherits_plan_ready(self):
        self.barrier.update(plan_decision(),True)
        other=StartupBarrier('other-session')
        self.assertFalse(other.snapshot()['plan_ready'])


def write_telemetry(path, boot, stop):
    """Keep a fresh read-only telemetry snapshot on disk for the runner."""
    def payload():
        stamp=time.monotonic_ns()
        return {'schema':'vlx-telemetry-v1','status':'running','host_boot_id':boot,
                'timestamp_ns':stamp,
                'sport':{'received_monotonic_ns':stamp,'mode':3,'error_code':0,
                         'rpy':[0.,0.,0.],'velocity':[0.,0.,0.]},
                'low':{'received_monotonic_ns':stamp,'mode_machine':1},
                'remote_axes':{'lx':0.,'ly':0.,'rx':0.,'ry':0.,'keys':0},
                'pose_history':[{'x':0.,'y':0.,'yaw':0.,'timestamp_ns':stamp,'frame_id':'odom'}]}
    def run():
        while not stop.is_set():
            temporary=path.with_name(path.name+'.tmp')
            temporary.write_text(json.dumps(payload()))
            temporary.replace(path)
            stop.wait(.02)
    thread=threading.Thread(target=run,daemon=True);thread.start()
    return thread


class IntegrationTests(unittest.TestCase):
    def test_runner_end_to_end_against_read_only_fake_preview(self):
        seen=[]
        boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                seen.append((self.command,self.path))
                now=time.monotonic_ns()-5_000_000
                obs=replace(synthetic_observation(1),boot_id=boot,camera_ns=now,prediction_ns=now,
                            prediction_index=now//10_000_000,camera_index=now//10_000_000)
                body=envelope(obs)
                if self.path in ('/camera.json','/state.json'):body['image_base64']='dGVzdA=='
                raw=json.dumps(body).encode()
                self.send_response(200);self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
            def log_message(self,*args):pass
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        telemetry_stop=threading.Event()
        try:
            with tempfile.TemporaryDirectory(dir=ROOT/'cache') as tmp:
                tmp=Path(tmp)
                telemetry=tmp/'telemetry.json'
                writer=write_telemetry(telemetry,boot,telemetry_stop)
                config=tmp/'config.json'
                config.write_text(json.dumps({'execution_enabled':False,
                    'source_url':f'http://127.0.0.1:{server.server_port}',
                    'loop_hz':20,'read_hz':10,'slow_min_interval_s':.1,
                    'provider':{'kind':'mock','delay_s':.15,'angle_deg':10},
                    'slow_mode':'route',
                    'task':{'plan_timeout_s':3.,'task_timeout_s':1.2},
                    'telemetry_path':str(telemetry)}))
                env=dict(os.environ,PYTHONPATH=str(ROOT/'src'),PYTHONDONTWRITEBYTECODE='1')
                result=subprocess.run([sys.executable,'-B','-m','vlx_async.runner','--config',str(config),
                                       '--duration','1.5','--port','0'],env=env,cwd=ROOT,
                                      capture_output=True,text=True,timeout=20)
                telemetry_stop.set();writer.join(timeout=1)
                self.assertEqual(result.returncode,0,result.stderr)
                summary=json.loads(result.stdout.splitlines()[-1])
                self.assertGreaterEqual(summary['metrics']['ticks'],10)
                self.assertEqual(summary['robot_commands_sent'],0)
                # One DeepSeek request for the whole task and none afterwards.
                self.assertEqual(summary['task']['plan_calls'],1)
                self.assertEqual(summary['metrics']['plan_requests'],1)
                self.assertEqual(summary['metrics']['plan_accepted'],1)
                self.assertEqual(summary['slow_worker']['calls_for_task'],1)
                # The robot never moved, so the frozen route times out.
                self.assertEqual(summary['task']['phase'],'blocked')
                self.assertEqual(summary['task']['blocked_reason'],'task_timeout')
                self.assertTrue(all(method=='GET' and path in {'/control.json','/camera.json','/state.json'}
                                    for method,path in seen))
                run=Path(summary['output'])
                self.assertTrue(list(run.glob('request_*.jpg')))
                self.assertTrue(list(run.glob('request_*.json')))
                latest=json.loads((run/'latest.json').read_text())
                self.assertTrue(latest['request_id'])
                self.assertEqual(latest['task']['plan_calls'],1)
                self.assertEqual(latest['task']['phase'],'blocked')
                self.assertIs(latest['task_execution_ready'],False)
                self.assertIsNotNone(latest['execution_preview'])
                # The published packet carries the task block and task identity.
                packet=json.loads((ROOT/'runs/fusion_execution_packet.json').read_text())
                self.assertEqual(packet['status'],'finished')
                self.assertEqual(packet['task']['plan_calls'],1)
        finally:
            telemetry_stop.set()
            server.shutdown();server.server_close();thread.join()

    def test_all_requested_delays_continue_local_ticks(self):
        """A pending plan must never stall the local decision loop."""
        from vlx_async.task_plan import RoutePlan
        for delay in (1.,3.,5.,7.):
            fusion=Fusion()
            request_obs=synthetic_observation(0.)
            route=tuple((i*.5,i*.12,0.) for i in range(13))
            plan=RoutePlan('delay-'+str(delay),request_obs.task_key,route,1.,
                           'synthetic known route','synthetic_oracle',
                           request_obs.camera_ns+int(delay*1e9),request_obs.context.pose,
                           'observation_local_m',1)
            received=False;before=0;ready_ticks=0
            for tick in range(int((delay+2)*20)):
                t=tick/20
                obs=synthetic_observation(t)
                if not received and t>=delay:
                    self.assertIsNone(fusion.accept(plan,obs,obs.camera_ns))
                    received=True
                result=fusion.decide(obs,obs.camera_ns)
                before+=int(t<delay)
                ready_ticks+=int(result['reason']=='task_ready')
            self.assertEqual(before,int(delay*20))
            self.assertTrue(received)
            self.assertGreater(ready_ticks,0)

    def test_dashboard_is_read_only(self):
        dashboard=Dashboard('127.0.0.1',0,[])
        try:
            url=f'http://127.0.0.1:{dashboard.port}'
            with urllib.request.urlopen(url+'/state.json') as r:data=json.load(r)
            self.assertFalse(data['execution_enabled'])
            req=urllib.request.Request(url+'/instruction',b'{}',method='POST')
            with self.assertRaises(urllib.error.HTTPError) as cm:urllib.request.urlopen(req)
            self.assertEqual(cm.exception.code,501)
        finally:dashboard.close()

    def test_readonly_source_issues_GET_only(self):
        seen=[]
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                seen.append((self.command,self.path));body=b'{"status":"ok"}'
                self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
            def log_message(self,*args):pass
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            source=ReadOnlySource(f'http://127.0.0.1:{server.server_port}')
            self.assertEqual(source.get('/state.json')['status'],'ok')
            self.assertEqual(seen,[('GET','/state.json')])
        finally:server.shutdown();server.server_close();thread.join()


class ExecutorTaskSemanticsTests(unittest.TestCase):
    """scripts/fusion_trial.py: plan waiting and route completion, no robot I/O."""

    def setUp(self):
        import importlib.util
        source=ROOT/'scripts/fusion_trial.py'
        spec=importlib.util.spec_from_file_location('fusion_trial_semantics',source)
        self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        self.events=[]

    def _packet(self,path,decision):
        path.write_text(json.dumps(dict(schema='vlx-fusion-execution-packet-v1',status='shadow',
            timestamp_ns=time.monotonic_ns(),session_id='test',decision=decision,telemetry={})))

    def _handoff_and_sink(self):
        events=self.events
        class Lease:
            file=object()
            def close(self):pass
        class Handoff:
            def __init__(self,run):
                self.released=False;self.original_stop_confirmed=True;self.lease=Lease()
            def acquire(self,cancelled=None):events.append('handoff')
            def restore(self,stop):events.append('restore');return True
        class Sink:
            def __init__(self,lease):events.append('sink')
            def stop(self):events.append('stop');return 0
        return Handoff,Sink

    def test_route_completed_is_normal_completion_not_execution_rejection(self):
        module,events=self.module,self.events
        command=dict(allowed=True,reason='bounded_command_validated',command_mps_rps=[.005,0.,0.])
        class Gate:
            def __init__(self,*args,**kwargs):pass
            def evaluate(self,*args):return dict(command)
        Handoff,Sink=self._handoff_and_sink()
        state={}
        class Actuator:
            def __init__(self,*args,**kwargs):
                self.sent,self.hold_count,self.stop_codes=[1,0,[]]
                self.fault=None;self.holding=False
                self._packet=state['packet']
                self._packet.write_text(json.dumps(dict(schema='vlx-fusion-execution-packet-v1',
                    status='shadow',timestamp_ns=time.monotonic_ns(),session_id='test',
                    decision=dict(reason='route_completed',task_execution_ready=False,
                                  plan_status='completed',remaining_route_m=0.,progress_fraction=1.),
                    telemetry={})))
            def submit(self,command):
                events.append('submit');return True
            def close(self):
                events.append('close');self.stop_codes=[0];self.fault='executor_closed'
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as directory:
            root=Path(directory);(root/'runs').mkdir()
            state['packet']=root/'runs/fusion_execution_packet.json'
            self._packet(state['packet'],dict(reason='task_ready',task_execution_ready=True,
                plan_request_id='plan-1',recommended=[[0.,0.,0.]]*8))
            argv=['fusion_trial.py','--execute','--session-id','test','--duration','1','--ready-timeout','1']
            with patch.object(module,'ROOT',root),patch.object(module.sys,'argv',argv),\
                 patch.object(module,'ControllerHandoff',Handoff),patch.object(module,'AS2Sink',Sink),\
                 patch.object(module,'GuardedActuator',Actuator),patch.object(module,'CommandGate',Gate),\
                 patch.object(module,'packet_observation',return_value=object()),\
                 patch.object(module.signal,'signal'),patch('builtins.print'):
                result=module.main()
            summary=json.loads(next((root/'runs').glob('execution_*/summary.json')).read_text())
            log=[json.loads(x) for x in
                 (Path(summary['output'])/'commands.jsonl').read_text().splitlines()]
        self.assertEqual(result,0,summary)
        self.assertEqual(summary['stop_reason'],'task_completed')
        self.assertTrue(summary['task_completed'])
        self.assertTrue(summary['stop_confirmed'])
        self.assertEqual(events.count('submit'),1)
        self.assertIn('close',events)
        self.assertEqual(log[-1]['reason'],'route_completed')
        self.assertFalse(log[-1]['allowed'])
        self.assertTrue(log[-1]['task_completed'])

    def test_plan_pending_waits_instead_of_rejecting_execution(self):
        module,events=self.module,self.events
        command=dict(allowed=True,reason='bounded_command_validated',command_mps_rps=[.005,0.,0.])
        class Gate:
            def __init__(self,*args,**kwargs):pass
            def evaluate(self,*args):return dict(command)
        Handoff,Sink=self._handoff_and_sink()
        state={'flipped':False}
        class Actuator:
            def __init__(self,*args,**kwargs):
                self.sent,self.hold_count,self.stop_codes=[0,0,[]]
                self.fault=None;self.holding=False
            def submit(self,command):
                self.sent+=1;return True
            def close(self):
                self.stop_codes=[0];self.fault='executor_closed'
        def flip(_seconds):
            if not state['flipped']:
                state['flipped']=True
                self._packet(state['packet'],dict(reason='task_ready',task_execution_ready=True,
                    plan_request_id='plan-1',recommended=[[0.,0.,0.]]*8))
        with tempfile.TemporaryDirectory(dir=ROOT/'cache') as directory:
            root=Path(directory);(root/'runs').mkdir()
            state['packet']=root/'runs/fusion_execution_packet.json'
            self._packet(state['packet'],dict(reason='plan_pending',task_execution_ready=False,
                plan_request_id=None,recommended=[]))
            argv=['fusion_trial.py','--execute','--session-id','test','--duration','.02','--ready-timeout','1']
            with patch.object(module,'ROOT',root),patch.object(module.sys,'argv',argv),\
                 patch.object(module,'ControllerHandoff',Handoff),patch.object(module,'AS2Sink',Sink),\
                 patch.object(module,'GuardedActuator',Actuator),patch.object(module,'CommandGate',Gate),\
                 patch.object(module,'packet_observation',return_value=object()),\
                 patch.object(module.time,'sleep',side_effect=flip),\
                 patch.object(module.signal,'signal'),patch('builtins.print'):
                result=module.main()
            summary=json.loads(next((root/'runs').glob('execution_*/summary.json')).read_text())
        self.assertEqual(result,0,summary)
        self.assertNotEqual(summary.get('stop_reason'),'task_completed')
        self.assertTrue(state['flipped'])
        self.assertGreater(summary['motion_commands_sent'],0)


if __name__=='__main__':unittest.main()
