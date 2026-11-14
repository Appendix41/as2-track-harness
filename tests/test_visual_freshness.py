"""Visual observation freshness: timestamps are preserved, age is bounded.

The depth/perception service is deleted; these tests only cover the RGB/VLX
source that still feeds the single task plan.
"""
from copy import deepcopy
from dataclasses import replace
import importlib.util
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from vlx_async.source import visual_observation, FastGate
from vlx_async.execution import CommandGate
from vlx_async.replay import synthetic_observation
from test_async import envelope
from test_perception_execution import executable_inputs
from vlx_async.storage import ROOT


class VisualFreshnessTests(unittest.TestCase):
    def setUp(self):
        self.obs=synthetic_observation(20)
        self.now=self.obs.prediction_ns+100_000_000
        self.state=envelope(self.obs)
        self.state.update(schema='vlx-camera-v1', image_base64='image',
            camera_frame=dict(index=self.obs.camera_index+2,
                host_monotonic_ns=self.now-10_000_000,color_order='RGB',capture_generation=0),
            inference_record=dict(self.state.pop('record'),capture_generation=0))

    def test_newer_camera_does_not_retimestamp_vlx_prediction(self):
        raw=visual_observation(self.state,self.obs.boot_id,self.now)
        self.assertGreater(raw.camera_ns,raw.prediction_ns)
        self.assertEqual(raw.prediction_ns,self.obs.prediction_ns)
        self.assertEqual(raw.prediction_index,self.obs.prediction_index)
        self.assertEqual(raw.camera_index,self.obs.camera_index+2)
        self.assertEqual(FastGate(self.obs.boot_id).check(self.state,self.now),'invalid_envelope')

    def test_wrong_boot_stream_task_generation_and_future_frames_rejected(self):
        for key,value in [('host_boot_id','other'),('stream_id',''),('stream_id',42),
                          ('requested_instruction_revision',99),('status','camera_reconnecting')]:
            with self.subTest(key=key),self.assertRaises(ValueError):
                visual_observation(dict(self.state,**{key:value}),self.obs.boot_id,self.now)
        for key,value in [('host_monotonic_ns',self.now+1),('index',-1),
                          ('capture_generation',1),('color_order','BGR')]:
            state=deepcopy(self.state);state['camera_frame'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):
                visual_observation(state,self.obs.boot_id,self.now)

    def test_configured_visual_age_preserves_timestamp_and_has_expiry(self):
        state=deepcopy(self.state);stamp=self.now-900_000_000
        state['camera_frame']['host_monotonic_ns']=stamp
        with self.assertRaisesRegex(ValueError,'camera_frame_stale'):
            visual_observation(state,self.obs.boot_id,self.now)
        raw=visual_observation(state,self.obs.boot_id,self.now,max_age_s=1.5)
        self.assertEqual(raw.camera_ns,stamp)
        with self.assertRaisesRegex(ValueError,'camera_frame_stale'):
            visual_observation(state,self.obs.boot_id,self.now+700_000_000,max_age_s=1.5)

    def test_exposure_alignment_and_near_field_coverage_are_not_execution_gates(self):
        d,o,t,n=executable_inputs()
        o=replace(o,context=replace(o.context,time_sync_verified=False,coverage_verified=False,
                                    observed_free_cells=()))
        d.update(measured_corridor_clear=False,execution_blockers=[])
        result=CommandGate().evaluate(d,o,t,n)
        self.assertTrue(result['allowed'],result)
        self.assertEqual(result['reason'],'bounded_command_validated')

    def test_preview_raw_snapshot_does_not_consume_camera_or_change_prediction(self):
        source=ROOT/'tests/fixtures/vlx_preview'
        with patch.object(sys,'path',[str(source),*sys.path]):
            spec=importlib.util.spec_from_file_location('isolated_camera_preview',source/'preview_vlx_live.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        from vlx_live_camera import LatestFrame, CameraFrame
        mailbox=LatestFrame()
        mailbox.frame=CameraFrame(dict(width=2,height=2,index=99,host_monotonic_ns=self.now,
                                      color_order='RGB',capture_generation=0),b'\x00'*12)
        preview=object.__new__(module.Preview);preview.lock=threading.Lock()
        preview.camera=SimpleNamespace(latest=mailbox)
        preview.state=dict(self.state,record=self.state['inference_record'])
        consumed=mailbox.consumed_index
        output=preview.camera_state()
        self.assertEqual(output['camera_frame']['index'],99)
        self.assertTrue(output['image_base64'])
        self.assertEqual(output['inference_record'],self.state['inference_record'])
        self.assertEqual(mailbox.consumed_index,consumed)
        mailbox.frame=replace(mailbox.frame,metadata=dict(mailbox.frame.metadata,capture_generation=1))
        self.assertEqual(preview.camera_state()['status'],'camera_generation_pending')
