"""Deterministic synthetic delay experiment and read-only recorded VLX replay."""
import argparse
from collections import Counter
from dataclasses import replace
from datetime import datetime
import json
import math
from pathlib import Path
from .contracts import Context, Observation, Pose
from .fusion import Fusion
from .source import FastGate
from .task_plan import RoutePlan
from .storage import ROOT, atomic_json


def synthetic_observation(t, pose=None, target_id=None):
    ns=int((100+t)*1e9)
    pose=pose or Pose(t*.10,0.,0.,ns,'synthetic_world')
    ctx=Context(pose,(),True,ns,target_id,ns,(2.,.5) if target_id else None)
    path=tuple((i*.12,0.,0.) for i in range(8))
    return Observation('synthetic-stream','synthetic-boot',1,'go to far chair',int(t*10),
                       int(t*30),ns,ns,path,ctx)


def delay_experiment():
    rows=[]
    for delay in (1.,3.,5.,7.):
        fusion=Fusion()
        old=synthetic_observation(0.)
        route=tuple((i*.5,i*.12,math.atan2(.12,.5)) for i in range(13))
        plan=RoutePlan('delay-'+str(delay),old.task_key,route,1.,
                       'Synthetic known route, not model quality','synthetic_oracle',
                       old.camera_ns+int(delay*1e9),old.context.pose,'observation_local_m',1)
        before=0;received=False;reasons=Counter();corrections=0
        for tick in range(int((delay+2)*20)):
            t=tick/20
            obs=synthetic_observation(t)
            if not received and t>=delay:
                assert fusion.accept(plan,obs,obs.camera_ns) is None
                received=True
            result=fusion.decide(obs,obs.camera_ns)
            reasons[result['reason']]+=1
            before+=int(t<delay)
            corrections+=int(result['selection']=='plan_aligned')
        rows.append({'injected_delay_s':delay,'local_ticks_before_response':before,
                     'slow_result_received':received,'corrected_ticks':corrections,
                     'reasons':dict(reasons),'assumptions':'synthetic exact poses, known empty geometry, static target'})
    return {'kind':'deterministic_synthetic_experiment','not_wall_clock_benchmark':True,
            'not_navigation_success_measurement':True,'rows':rows,'robot_commands_sent':0}


def replay_records(path,max_records=1000):
    stats=Counter();timestamps=[];gate=FastGate('replay-boot')
    with Path(path).open() as stream:
        for line in stream:
            if stats['records']>=max_records:
                break
            try:
                record=json.loads(line)
            except json.JSONDecodeError:
                stats['invalid_json']+=1
                continue
            stats['records']+=1
            if record.get('history_reset_this_frame'):
                gate=FastGate('replay-boot')
            state={'schema':'vlx-control-v1','host_boot_id':'replay-boot','stream_id':'replay',
                   'status':'running','requested_instruction_revision':record.get('instruction_revision'),
                   'requested_instruction':record.get('instruction'),'record':record}
            now=record.get('prediction_available_monotonic_ns')
            if type(now) is not int:
                stats['invalid_clock']+=1
                continue
            reason=gate.check(state,now)
            stats['rejected:'+reason if reason else 'valid']+=1
            if not reason:
                obs=Observation.from_state(state)
                result=Fusion().decide(obs,now)
                assert result['recommended']==list(obs.waypoints)
                timestamps.append(now)
    hz=((len(timestamps)-1)*1e9/(timestamps[-1]-timestamps[0])
        if len(timestamps)>1 and timestamps[-1]>timestamps[0] else None)
    return {'kind':'recorded_vlx_replay','source':str(Path(path).resolve()),'metrics':dict(stats),
            'recorded_prediction_hz':hz,'robot_commands_sent':0,
            'geometry_or_semantic_success_tested':False}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--predictions')
    parser.add_argument('--max-records',type=int,default=1000)
    args=parser.parse_args(argv)
    if args.max_records<=0:
        parser.error('max-records must be positive')
    out=ROOT/'runs'/('replay_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir()
    report=delay_experiment()
    if args.predictions:
        report['recorded_replay']=replay_records(args.predictions,args.max_records)
    atomic_json(out/'report.json',report)
    print(json.dumps({'output':str(out),**report},ensure_ascii=False))


if __name__=='__main__':
    main()
