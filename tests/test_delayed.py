from dataclasses import replace
import json
import os
import threading
import unittest
from unittest.mock import patch

from vlx_async.contracts import Request, SemanticAnchor, SlowPlan, TaskConfig
from vlx_async.fusion import Fusion
from vlx_async.providers import ResponsesProvider, decode_anchor, decode_hybrid, metric_plan
from vlx_async.replay import synthetic_observation
from vlx_async.task_plan import RoutePlan


def delayed_plan(label='person', attributes=('red',), rule='attributes', seconds=20):
    old=synthetic_observation(0)
    request=Request('delay20',old,old.camera_ns)
    done=old.camera_ns+int(seconds*1e9)
    anchor=SemanticAnchor(request,done,'target',label,attributes,rule,True,(.1,.2,.3,.6),
                          .9,None,label=='person','synthetic','synthetic independent evidence')
    return SlowPlan(request,done,'image_uv',((.5,.9,0.),(.6,.5,0.)),.9,None,label=='person',
                    'synthetic','test',False,anchor)


def route_plan(obs, **changes):
    data=dict(request_id='delayed-plan',task_key=obs.task_key,
              points=tuple((i*.5,0.,0.) for i in range(9)),confidence=1.,
              description='delayed ground route',provider='synthetic',
              created_ns=obs.camera_ns,pose=obs.context.pose)
    data.update(changes)
    return RoutePlan(**data)


class DelayedPlanTests(unittest.TestCase):
    """A late plan is still the single plan of its task; a completed route ends it."""

    def setUp(self):
        self.old=synthetic_observation(0)
        # These cases test plan/arrival semantics, not the travel safety bound.
        self.fusion=Fusion(task_config=TaskConfig(max_travel_m=10.))

    def test_robot_holds_until_the_delayed_plan_arrives(self):
        waiting=synthetic_observation(20)
        self.assertEqual(self.fusion.decide(waiting,waiting.camera_ns)['reason'],'plan_pending')
        self.assertIsNone(self.fusion.accept(route_plan(self.old),waiting,waiting.camera_ns))
        ready=self.fusion.decide(waiting,waiting.camera_ns)
        self.assertEqual(ready['reason'],'task_ready')
        self.assertTrue(ready['task_execution_ready'])
        self.assertEqual(ready['plan_request_id'],'delayed-plan')

    def test_late_plan_still_has_only_one_call_per_task(self):
        late=synthetic_observation(30)
        self.assertIsNone(self.fusion.accept(route_plan(self.old),late,late.camera_ns))
        self.assertEqual(self.fusion.accept(route_plan(self.old),late,late.camera_ns),
                         'plan_already_active')

    def test_arrival_after_delay_reports_route_completed(self):
        late=synthetic_observation(20)
        self.assertIsNone(self.fusion.accept(route_plan(self.old),late,late.camera_ns))
        arrived=synthetic_observation(25)
        arrived=replace(arrived,context=replace(arrived.context,
            pose=replace(arrived.context.pose,x=4.)))
        done=self.fusion.decide(arrived,arrived.camera_ns)
        self.assertEqual(done['reason'],'route_completed')
        self.assertEqual(done['recommended'],[])
        self.assertFalse(done['task_execution_ready'])
        self.assertTrue(done['task_completed'])


class HybridProviderTests(unittest.TestCase):
    def setUp(self):
        self.plan=delayed_plan()
        self.req=self.plan.request
        self.anchor={'request_id':self.req.request_id,'anchor_kind':'target','target_label':'person',
                     'attributes':['red'],'selection_rule':'attributes','reference_bbox':[.1,.2,.3,.7],
                     'confidence':.9,'target_id':None,'dynamic_target':True,'reason':'test'}
        self.route={'request_id':self.req.request_id,'coordinate_frame':'image_uv',
                    'waypoints':[{'u':.5,'v':.9},{'u':.4,'v':.5}],'confidence':.8,
                    'target_description':'red person','target_id':None,'dynamic_target':True,'reason':'test'}

    def test_hybrid_preserves_both_outputs(self):
        p=decode_hybrid({'route':self.route,'anchor':self.anchor},self.req,self.plan.completed_ns,'test')
        self.assertEqual(len(p.waypoints),2)
        self.assertEqual(p.anchor.attributes,('red',))
        self.assertFalse(p.metric_verified)

    def test_nested_request_mismatch_rejected(self):
        self.anchor['request_id']='other'
        with self.assertRaisesRegex(ValueError,'request_mismatch'):
            decode_hybrid({'route':self.route,'anchor':self.anchor},self.req,self.plan.completed_ns,'test')

    def test_none_has_no_valid_box(self):
        self.anchor.update(anchor_kind='none',selection_rule='none',confidence=0)
        p=decode_anchor(self.anchor,self.req,self.plan.completed_ns,'test')
        self.assertFalse(p.anchor.bbox_valid)
        self.assertIs(metric_plan(p,{'verified':True}),p)

    def test_hybrid_payload_uses_strict_schema(self):
        with patch.dict(os.environ,{'DEEPSEEK_API_KEY':'test-only'}):
            provider=ResponsesProvider({'model':'deepseek-v4.1-flash'},True)
        req=replace(self.req,observation=replace(self.req.observation,image_base64='test'))
        schema=provider.payload(req,'hybrid')['text']['format']['schema']
        self.assertEqual(schema['required'],['route','anchor'])
        self.assertFalse(schema['additionalProperties'])


if __name__=='__main__':unittest.main()
