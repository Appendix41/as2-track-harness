"""Read-only fusion runner: one DeepSeek plan per task, then local execution.

Task identity is the VLX ``task_key`` = (boot, stream, revision, instruction).
The task is planned once while the robot is still; the accepted plan is frozen
as a body-frame route and re-expressed through the current pose on every tick.
Only a new task key opens a new planning phase:

    idle -> planning -> ready -> executing -> completed
                       |
                       +-> blocked (plan rejected / plan timeout / task timeout)

This process never opens a robot client. It publishes a read-only decision
packet that a separate executor validates. The only pose source is the
read-only telemetry snapshot (``RobotState``).
"""
import argparse
import base64
from collections import Counter, deque
from dataclasses import asdict, replace
from datetime import datetime
import fcntl
import json
import math
from pathlib import Path
import signal
import threading
import time
import uuid

from .contracts import Context, FusionConfig, Observation, Request, TaskConfig
from .dashboard import Dashboard
from .fusion import Fusion
from .ground import load_intrinsics, project_image_route
from .providers import MockProvider, ResponsesProvider, SlowWorker
from .robot_state import RobotState
from .depth_obstacles import latest_obstacles, obstacles_enabled
from .source import FastGate, FastReader, ReadOnlySource, visual_observation
from .storage import ROOT, atomic_json, confined
from .startup import StartupBarrier
from .task_plan import RoutePlan

TASK_PHASES = ('idle', 'planning', 'ready', 'executing', 'completed', 'blocked')
TERMINAL_PHASES = frozenset({'completed', 'blocked'})
CAMERA_MAX_AGE_S = .5


def lifecycle_windows(active_elapsed_s, api_duration_s, keep_alive_s=0.):
    """API calls use the model window; VLX may retain a separate local window.

    Kept for the trial harness contract: the executor passes the model window as
    ``api_duration_s`` and a longer local keep-alive.  The runner's own task
    window is independent (see :func:`task_config`).
    """
    if (any(type(v) not in (int, float) or not math.isfinite(v) for v in
            (active_elapsed_s, api_duration_s, keep_alive_s))
            or active_elapsed_s < 0 or api_duration_s <= 0 or keep_alive_s < 0):
        raise ValueError('invalid_runner_lifecycle_window')
    return dict(api_open=active_elapsed_s < api_duration_s,
                runner_open=active_elapsed_s < api_duration_s+keep_alive_s)


def runner_budget(startup_timeout_s, duration_s, keep_alive_s, has_session):
    """Hard wall-clock budget for the whole process, independent of task state.

    A trial must always terminate: the task window alone cannot bound the run,
    because a planner answer may never arrive.  ``duration`` is the post-plan
    task window; the startup barrier may add its own wait, and ``keep-alive``
    extends the local loop after the task window.  A 2 s settling margin covers
    process start-up, the last packet and the summary write.
    """
    margin = 2.
    if has_session:
        return startup_timeout_s + duration_s + keep_alive_s + margin
    return duration_s + keep_alive_s + margin


def task_config(config, duration_s):
    """TaskConfig from ``config['task']``; ``--duration`` caps both windows.

    ``--duration`` now means "seconds of task window after the plan is accepted",
    not a lifetime for the whole process.  The plan window is additionally
    capped by that duration, so a missing planner answer blocks the task and
    ends the run instead of waiting forever.
    """
    block = config.get('task') or {}
    if not isinstance(block, dict):
        raise ValueError('invalid_task_config')
    known = TaskConfig.__dataclass_fields__
    values = {key: value for key, value in block.items() if key in known}
    values['task_timeout_s'] = min(
        values.get('task_timeout_s', TaskConfig.task_timeout_s), duration_s)
    values['plan_timeout_s'] = min(
        values.get('plan_timeout_s', TaskConfig.plan_timeout_s), duration_s)
    return TaskConfig(**values)


def request_pose(robot, camera_ns, max_skew_s):
    """Pose for the request frame, never a pose invented from a later tick.

    The interpolated pose at the camera stamp is preferred; when the odometry
    history cannot bracket that instant, the latest pose is used only if it is
    within the allowed skew.  ``None`` means the plan cannot be placed in the
    body frame, which blocks the task instead of walking a stale route.
    """
    pose = robot.pose_at(camera_ns)
    if pose is not None:
        return pose
    pose = robot.latest_pose()
    if pose is not None and abs(pose.timestamp_ns-camera_ns)/1e9 <= max_skew_s:
        return pose
    return None


def plan_route(plan, intrinsics, task):
    """Project one slow plan onto the body frame captured with its request.

    ``RoutePlan.pose`` is the pose carried by the request observation (the frame
    the model actually saw), never the pose of the tick that consumes the plan.
    """
    observation = plan.request.observation
    if plan.coordinate_frame == 'image_uv':
        points = project_image_route(plan.waypoints, intrinsics, task.max_range_m)
        source = 'monocular_ground'
    elif plan.coordinate_frame == 'observation_local_m':
        points = tuple(tuple(float(v) for v in point) for point in plan.waypoints)
        source = 'observation_local_m'
    else:
        raise ValueError('unsupported_plan_frame')
    return RoutePlan(request_id=plan.request.request_id,
                     task_key=observation.task_key, points=points,
                     confidence=float(plan.confidence),
                     description=str(plan.reason), provider=str(plan.provider),
                     created_ns=int(plan.completed_ns), pose=observation.context.pose,
                     source=source, calls=1)


class TaskLoop:
    """Runner-side task lifecycle.  At most one plan request per task, ever."""

    def __init__(self, config, clock=time.monotonic):
        self.config = config
        self.clock = clock
        self.started = clock()
        self.phase = 'idle'
        self.task_key = None
        self.plan_calls = 0
        self.plan_request_id = None
        self.plan_timeout = False
        self.blocked_reason = None
        self.planning_started = None
        self.plan_ready_at = None
        self.completed_at = None

    def begin(self, task_key, now=None):
        """Open planning for a new task; a new key is the only way in."""
        self.task_key = task_key
        self.phase = 'planning'
        self.plan_calls = 0
        self.plan_request_id = None
        self.plan_timeout = False
        self.blocked_reason = None
        self.planning_started = self.clock() if now is None else now
        self.plan_ready_at = None
        self.completed_at = None

    def note_submission(self):
        """Count this task's one allowed plan request."""
        if self.phase != 'planning':
            raise ValueError('plan_submitted_outside_planning')
        if self.plan_calls >= self.config.max_plan_calls:
            raise ValueError('plan_call_limit_exceeded')
        self.plan_calls += 1

    def plan_accepted(self, request_id, now=None):
        self.phase = 'ready'
        self.plan_request_id = request_id
        self.blocked_reason = None
        self.plan_ready_at = self.clock() if now is None else now

    def plan_rejected(self, reason):
        self.phase = 'blocked'
        self.blocked_reason = str(reason)

    def expire(self, now=None):
        """Close a phase whose window elapsed; returns the terminal reason."""
        now = self.clock() if now is None else now
        if (self.phase == 'planning' and self.planning_started is not None
                and now-self.planning_started > self.config.plan_timeout_s):
            self.phase, self.plan_timeout = 'blocked', True
            self.blocked_reason = 'plan_timeout'
            return 'plan_timeout'
        if (self.phase in ('ready', 'executing') and self.plan_ready_at is not None
                and now-self.plan_ready_at > self.config.task_timeout_s):
            self.phase, self.blocked_reason = 'blocked', 'task_timeout'
            return 'task_timeout'
        return None

    def observe(self, decision, now=None):
        """Fold this tick's fusion decision into the task phase.

        This never calls the planner: a task that is already moving can only
        finish, drift out, or keep following its frozen plan.
        """
        now = self.clock() if now is None else now
        if self.phase not in ('ready', 'executing'):
            return
        if decision.get('task_completed'):
            self.phase, self.completed_at = 'completed', now
        elif decision.get('task_execution_ready') is True and decision.get('reason') == 'task_ready':
            self.phase = 'executing'
        elif decision.get('plan_status') == 'blocked' and decision.get('reason') == 'route_drift':
            self.phase, self.blocked_reason = 'blocked', 'route_drift'

    def deadline(self, keep_alive_s=0.):
        """Monotonic instant after which this process has nothing left to do."""
        if self.plan_ready_at is not None:
            return self.plan_ready_at + self.config.task_timeout_s + keep_alive_s
        base = self.planning_started if self.planning_started is not None else self.started
        return base + self.config.plan_timeout_s + keep_alive_s

    def close_planning(self, reason='plan_timeout'):
        """Force an unfinished plan window closed (hard wall-clock safety net)."""
        if self.phase in ('idle', 'planning'):
            self.phase, self.plan_timeout, self.blocked_reason = 'blocked', reason == 'plan_timeout', reason

    def summary(self, decision, now=None):
        """Task block merged into every decision (packet and dashboard alike)."""
        now = self.clock() if now is None else now
        return dict(
            phase=self.phase, plan_calls=self.plan_calls,
            plan_request_id=self.plan_request_id,
            remaining_route_m=decision.get('remaining_route_m'),
            total_route_m=decision.get('total_route_m'),
            progress_fraction=decision.get('progress_fraction'),
            completed=bool(decision.get('task_completed')),
            plan_timeout=self.plan_timeout, blocked_reason=self.blocked_reason,
            task_window_s=self.config.task_timeout_s,
            plan_timeout_s=self.config.plan_timeout_s,
            plan_max_calls=self.config.max_plan_calls,
            task_elapsed_s=(now-self.plan_ready_at
                            if self.plan_ready_at is not None else None))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT/'config/shadow.json'))
    parser.add_argument('--duration', type=float, default=30.,
                        help='seconds of task window after the plan is accepted')
    parser.add_argument('--port', type=int, default=0, help='0 assigns a free port')
    parser.add_argument('--listen', default='127.0.0.1')
    parser.add_argument('--allow-client', action='append', default=[])
    parser.add_argument('--enable-api', action='store_true')
    parser.add_argument('--session-id', default=None)
    parser.add_argument('--startup-timeout', type=float, default=90.)
    parser.add_argument('--keep-alive-for', type=float, default=0.,
                        help='extra local-loop lifetime after the task window; no API calls are submitted then')
    args = parser.parse_args(argv)
    if not 0 < args.duration <= 86400 or not 0 <= args.keep_alive_for <= 172800 or not 0 <= args.port <= 65535:
        parser.error('duration must be (0,86400], port 0-65535')
    if args.listen not in {'127.0.0.1', 'localhost'} and not args.allow_client:
        parser.error('LAN listening requires --allow-client')
    config_path = confined(args.config)
    config = json.loads(config_path.read_text())
    if config.get('execution_enabled') is not False:
        parser.error('this build supports execution_enabled=false only')
    hz, read_hz = config.get('loop_hz',20.), config.get('read_hz',10.)
    minimum_interval = config.get('slow_min_interval_s',1.)
    if not 1<=hz<=50 or not 1<=read_hz<=20 or not .1<=minimum_interval<=60:
        parser.error('invalid loop rates')
    provider_config = config['provider']
    if provider_config['kind']=='mock':
        provider = MockProvider(provider_config.get('delay_s',7.),provider_config.get('angle_deg',20.))
    elif provider_config['kind']=='openai_responses':
        provider = ResponsesProvider(provider_config, enabled=args.enable_api)
    else:
        parser.error('unknown provider')
    slow_mode = config.get('slow_mode', 'route')
    if slow_mode not in {'route', 'anchor', 'hybrid'}:
        parser.error('slow_mode must be route, anchor or hybrid')
    started = time.monotonic()
    budget = runner_budget(args.startup_timeout, args.duration, args.keep_alive_for,
                           bool(args.session_id))
    hard_deadline = started + budget
    try:
        task = task_config(config, args.duration)
    except (TypeError, ValueError) as exc:
        parser.error('task config: '+str(exc))
    mount_path = config.get('camera_mount_path') or str(ROOT/'config/camera_mount.json')
    try:
        intrinsics = load_intrinsics(json.loads(confined(mount_path).read_text()))
    except (OSError, TypeError, ValueError) as exc:
        parser.error('camera mount: '+str(exc))
    try:
        telemetry_path = confined(config['telemetry_path']) if config.get('telemetry_path') else None
    except (TypeError, ValueError) as exc:
        parser.error('telemetry path: '+str(exc))
    from .execution import CommandGate, Limits
    known_limits = Limits.__dataclass_fields__
    try:
        command_gate = CommandGate(Limits(**{
            key: value for key, value in dict(config.get('execution_preview_limits', {})).items()
            if key in known_limits}))
        fusion = Fusion(FusionConfig.from_mapping(config.get('fusion', {})), task)
    except (TypeError, ValueError) as exc:
        parser.error('execution preview config: '+str(exc))
    run_dir = ROOT/'runs'/('shadow_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run_dir.mkdir()
    lock = (ROOT/'runs/shadow.lock').open('a')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('another isolated shadow runner is already running')
    stop = threading.Event()
    for sig in (signal.SIGTERM,signal.SIGINT):
        signal.signal(sig,lambda *_:stop.set())
    source = ReadOnlySource(config.get('source_url','http://127.0.0.1:8765'))
    gate = FastGate(Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    robot = RobotState(telemetry_path, config.get('owner_state_url'))
    worker = SlowWorker(provider, mode=slow_mode)
    dashboard = Dashboard(args.listen,args.port,args.allow_client)
    reader = FastReader(source,read_hz)
    loop = TaskLoop(task)
    startup = StartupBarrier(args.session_id, args.startup_timeout) if args.session_id else None
    metrics = Counter()
    timings, source_times = deque(maxlen=400), deque(maxlen=150)
    last_prediction = None
    last_submit = -float('inf')
    image_lock = threading.Lock()
    image_state = {}
    last_plan = {}
    plan_image_uv = []
    received_request_id = None
    locked = False
    representative_saved = False
    decision = {}
    metadata = {'schema':'vlx-async-shadow-v1','output':str(run_dir),'provider':provider.name,
                'url':f'http://{args.listen}:{dashboard.port}/','started_at':datetime.now().astimezone().isoformat(),
                'execution_enabled':False,'robot_commands_sent':0,'slow_mode':slow_mode,'config':config,
                'session_id':args.session_id,'task_phases':list(TASK_PHASES),
                'task_window_after_plan_s':args.duration,'keep_alive_after_task_s':args.keep_alive_for,
                'runner_budget_s':budget,'task_config':asdict(task)}
    atomic_json(run_dir/'metadata.json',metadata)
    print(json.dumps(metadata,ensure_ascii=False),flush=True)

    def request_factory():
        # One fresh RGB frame per task: the plan is made from this exact image.
        # The preview exposes image + waypoints + instruction in /state.json;
        # /camera.json belonged to the removed depth service.
        state = source.get('/state.json')
        obs = visual_observation(state, gate.boot_id, time.monotonic_ns(),
                                 max_age_s=CAMERA_MAX_AGE_S)
        pose = request_pose(robot, obs.camera_ns, fusion.config.pose_skew_s)
        obs = replace(obs, context=Context(pose=pose, body_transform_verified=True,
                                           target_source='slow_model',
                                           scene_timestamp_ns=obs.camera_ns))
        request = Request(uuid.uuid4().hex, obs, time.monotonic_ns())
        request_record = {'request_id':request.request_id,'camera_frame_index':obs.camera_index,
                          'camera_ns':obs.camera_ns,'sent_ns':request.sent_ns,'instruction':obs.instruction,
                          'instruction_revision':obs.revision,'stream_id':obs.stream_id,
                          'pose_available':pose is not None,
                          'pose_timestamp_ns':pose.timestamp_ns if pose is not None else None,
                          'pose_frame_id':pose.frame_id if pose is not None else None,
                          'provider':provider.name,'expected_plan_frame':'image_uv',
                          'plan_calls_this_task':loop.plan_calls+1}
        atomic_json(run_dir/('request_'+request.request_id+'.json'),request_record)
        if obs.image_base64:
            payload=base64.b64decode(obs.image_base64,validate=True)
            confined(run_dir/('request_'+request.request_id+'.jpg')).write_bytes(payload)
        with image_lock:
            image_state[request.request_id] = {'image_request_id':request.request_id,
                                               'image_base64':obs.image_base64}
            while len(image_state)>3:
                image_state.pop(next(iter(image_state)))
        return request

    try:
        with (run_dir/'decisions.jsonl').open('w') as log:
            while not stop.is_set():
                if startup and startup.phase in ('failed','plan_rejected','plan_timeout'):
                    break
                tick = time.monotonic()
                metrics['ticks'] += 1
                timings.append(tick)
                measured_obstacles, obstacle_snapshot = (), None
                depth_obstacles_on = obstacles_enabled()
                state,error=reader.snapshot()
                now_ns=time.monotonic_ns()
                reason=error or gate.check(state,now_ns)
                obs=None
                context_ms=None
                if not reason:
                    context_started = time.perf_counter_ns()
                    obs=Observation.from_state(state)
                    # One pose source only: read-only telemetry.  No depth, no
                    # local verifier, no target geometry can veto the route.
                    if depth_obstacles_on:
                        measured_obstacles, obstacle_snapshot = latest_obstacles()
                    obs=replace(obs,context=Context(pose=robot.latest_pose(),
                                                    body_transform_verified=True,
                                                    target_source='slow_model',
                                                    obstacles=measured_obstacles,
                                                    scene_timestamp_ns=obs.camera_ns))
                    context_ms=(time.perf_counter_ns()-context_started)/1e6
                    fusion.set_task(obs)
                    if loop.task_key!=obs.task_key:
                        loop.begin(obs.task_key,tick)
                        worker.arm(obs.task_key)
                        locked=False
                        plan_image_uv=[]
                        received_request_id=None
                        last_plan={}
                        metrics['tasks_started']+=1
                    raw_plan=worker.take()
                    if raw_plan is not None:
                        rejection=projection_error=None
                        if raw_plan.request.observation.task_key != obs.task_key:
                            # A response for a task the runner already replaced.
                            metrics['obsolete_plan_ignored']+=1
                        else:
                            metrics['slow_received']+=1
                            received_request_id=raw_plan.request.request_id
                            plan_image_uv=([[float(p[0]),float(p[1]),float(p[2])]
                                            for p in raw_plan.waypoints]
                                           if raw_plan.coordinate_frame=='image_uv' else [])
                            try:
                                route_plan=plan_route(raw_plan,intrinsics,task)
                            except (ValueError,KeyError,TypeError) as exc:
                                route_plan=None
                                projection_error=(str(exc) if isinstance(exc,ValueError)
                                                  else type(exc).__name__)
                                loop.plan_rejected('plan_rejected:'+projection_error)
                                metrics['plan_projection_failed']+=1
                                metrics['slow_rejected']+=1
                            else:
                                rejection=fusion.accept(route_plan,obs,time.monotonic_ns())
                                if rejection is None:
                                    loop.plan_accepted(route_plan.request_id,time.monotonic())
                                    worker.disarm()
                                    metrics['plan_accepted']+=1
                                else:
                                    loop.plan_rejected('plan_rejected:'+str(rejection))
                                    metrics['plan_rejected']+=1
                                    metrics['slow_rejected']+=1
                            last_plan=dict(request_id=raw_plan.request.request_id,
                                           plan_frame=raw_plan.coordinate_frame,
                                           slow_raw=[list(p) for p in raw_plan.waypoints],
                                           slow_frame=raw_plan.coordinate_frame,
                                           slow_reason=raw_plan.reason,
                                           response_confidence=raw_plan.confidence,
                                           response_rejection=rejection,
                                           route_projection_error=projection_error,
                                           latency_s=(raw_plan.completed_ns-raw_plan.request.sent_ns)/1e9,
                                           response_observation_ns=raw_plan.request.observation.camera_ns,
                                           plan_calls=loop.plan_calls,
                                           provider=raw_plan.provider)
                expired=loop.expire(time.monotonic())
                if expired:
                    metrics[expired]+=1
                if loop.phase in TERMINAL_PHASES:
                    if not locked:
                        worker.lock()
                        locked=True
                        metrics['tasks_terminal']+=1
                elif (loop.phase=='planning' and loop.plan_calls<task.max_plan_calls
                      and tick-last_submit>=minimum_interval):
                    # Exactly one request per task: the phase and the worker's
                    # per-task submission flag both have to allow it.
                    if worker.submit(request_factory):
                        loop.note_submission()
                        last_submit=tick
                        metrics['plan_requests']+=1
                if obs is not None:
                    fusion_started = time.perf_counter_ns()
                    decision=fusion.decide(obs,now_ns)
                    decision['stage_timing_ms'] = dict(
                        context_alignment=context_ms,
                        fusion_decision=(time.perf_counter_ns()-fusion_started)/1e6)
                    loop.observe(decision,time.monotonic())
                    if loop.phase in TERMINAL_PHASES and not locked:
                        worker.lock()
                        locked=True
                        metrics['tasks_terminal']+=1
                    if loop.phase=='blocked':
                        # Surface the runner-level terminal reason (documented
                        # decision contract: plan_rejected:<cause> / plan_timeout
                        # / task_timeout / route_drift) instead of the pending
                        # placeholder fusion leaves while no plan is active.
                        decision['reason']=loop.blocked_reason or 'task_blocked'
                    if loop.phase in TERMINAL_PHASES and decision.get('task_execution_ready'):
                        # The task window or the task state closed this task even
                        # though the frozen route still says "follow it".
                        decision.update(task_execution_ready=False,
                                        execution_blockers=[loop.blocked_reason or 'task_closed'],
                                        control_advice='hold_until_new_task')
                    decision.update(instruction=obs.instruction,camera_index=obs.camera_index,
                                    prediction_index=obs.prediction_index,stream_id=obs.stream_id,
                                    instruction_revision=obs.revision,host_boot_id=obs.boot_id,
                                    request_id=received_request_id or loop.plan_request_id,
                                    request_waypoints_uv=plan_image_uv,
                                    task=loop.summary(decision,time.monotonic()))
                    # Preview only: the real command gate runs in the executor.
                    decision['execution_preview']=command_gate.evaluate(
                        decision,obs,robot.telemetry(),time.monotonic_ns())
                    order=(obs.stream_id,obs.prediction_index)
                    if order!=last_prediction:
                        if last_prediction and order[0]!=last_prediction[0]:
                            source_times.clear()
                        source_times.append(obs.prediction_ns)
                        metrics['distinct_predictions']+=1
                        last_prediction=order
                else:
                    decision={'reason':reason,'selection':'hold','recommended':[],
                              'execution_enabled':False,'robot_commands_sent':0,
                              'task_execution_ready':False,
                              'request_id':received_request_id or loop.plan_request_id,
                              'request_waypoints_uv':plan_image_uv}
                    decision['task']=loop.summary(decision,time.monotonic())
                    metrics['invalid_fast_ticks']+=1
                elapsed=time.monotonic()-started
                loop_hz=(len(timings)-1)/(timings[-1]-timings[0]) if len(timings)>1 else 0.
                source_hz=((len(source_times)-1)*1e9/(source_times[-1]-source_times[0])
                           if len(source_times)>1 and source_times[-1]>source_times[0] else None)
                decision['measured_obstacles']=[dict(x_m=round(x,2),y_m=round(y,2),
                                                     radius_m=round(r,2),
                                                     distance_m=round(math.hypot(x,y),2),
                                                     bearing_deg=round(math.degrees(math.atan2(y,x)),1))
                                                for x,y,r in measured_obstacles]
                if obstacle_snapshot:
                    decision['obstacle_source']=dict(nearest_m=obstacle_snapshot.get('nearest_m'),
                                                     corridor_m=obstacle_snapshot.get('corridor_m'))
                decision.update(status='shadow',mode='shadow',elapsed_s=elapsed,loop_hz=loop_hz,
                                source_hz=source_hz,slow_worker=worker.state(),timestamp_ns=now_ns)
                if last_plan:
                    decision.update(last_plan)
                    decision['request_id']=received_request_id or loop.plan_request_id
                    decision['slow_age_s']=(now_ns-last_plan['response_observation_ns'])/1e9
                if startup:
                    decision['startup']=startup.update(decision,obs is not None)
                decision['active_elapsed_s']=((time.monotonic()-loop.plan_ready_at)
                                              if loop.plan_ready_at is not None else 0.)
                decision['active_duration_s']=task.task_timeout_s
                decision['api_window_open']=loop.phase=='planning'
                decision['vlx_keepalive_after_api']=bool(args.keep_alive_for>0
                                                        and loop.phase in ('ready','executing','completed'))
                if obs is not None:
                    # Publish the already-validated packet before any image work.
                    # An invalid VLX tick leaves the previous packet in place: it
                    # ages out instead of being replaced by unparseable evidence.
                    atomic_json(ROOT/'runs/fusion_execution_packet.json',
                                dict(schema='vlx-fusion-execution-packet-v1',status='shadow',
                                     timestamp_ns=time.monotonic_ns(),run=str(run_dir),
                                     session_id=args.session_id,decision=decision,
                                     observation=asdict(replace(obs,image_base64=None)),
                                     telemetry=robot.telemetry()))
                if decision.get('selection')=='plan_aligned' and not representative_saved:
                    atomic_json(run_dir/'successful_fusion.json',decision)
                    representative_saved=True
                metrics['reason:'+str(decision['reason'])]+=1
                log.write(json.dumps(decision,ensure_ascii=False,allow_nan=False)+'\n')
                with image_lock:
                    image_payload=image_state.get(decision.get('request_id'),{})
                dashboard.update({**decision,**image_payload})
                if metrics['ticks']%20==0:
                    atomic_json(run_dir/'latest.json',decision)
                    log.flush()
                # Two independent bounds: the current phase window, and the hard
                # wall-clock budget of the whole process.  A missing plan or a
                # missing VLX stream can never keep this loop alive forever.
                if time.monotonic()>min(loop.deadline(args.keep_alive_for),hard_deadline):
                    break
                stop.wait(max(0.,1/hz-(time.monotonic()-tick)))
    finally:
        worker.close()
        reader.close()
        robot.close()
        dashboard.close()
        # A run stopped by the wall-clock budget while still planning is reported
        # as a plan timeout, never as a healthy unfinished task.
        if time.monotonic() >= hard_deadline:
            loop.close_planning('plan_timeout')
        elapsed=time.monotonic()-started
        summary={**metadata,'elapsed_s':elapsed,'metrics':dict(metrics),
                 'loop_hz':decision.get('loop_hz'),'source_hz':decision.get('source_hz'),
                 'slow_worker':worker.state(),'last_reason':decision.get('reason'),
                 'last_plan':last_plan,
                 'task':dict(phase=loop.phase,plan_calls=loop.plan_calls,
                             plan_request_id=loop.plan_request_id,
                             completed_tasks=int(metrics['tasks_completed']),
                             plan_requests=int(metrics['plan_requests']),
                             plan_accepted=int(metrics['plan_accepted']),
                             blocked_reason=loop.blocked_reason,
                             plan_timeout=loop.plan_timeout,
                             task_window_s=task.task_timeout_s,
                             plan_timeout_s=task.plan_timeout_s),
                 'startup':startup.snapshot() if startup else None,
                 'status':'finished','execution_enabled':False,'robot_commands_sent':0}
        atomic_json(run_dir/'summary.json',summary)
        atomic_json(run_dir/'latest.json',{**decision,'status':'finished',
                                           'task':loop.summary(decision)})
        atomic_json(ROOT/'runs/fusion_execution_packet.json',
                    dict(schema='vlx-fusion-execution-packet-v1',status='finished',
                         timestamp_ns=time.monotonic_ns(),run=str(run_dir),
                         session_id=args.session_id,task=summary['task'],
                         startup=startup.snapshot() if startup else None))
        lock.close()
        print(json.dumps(summary,ensure_ascii=False),flush=True)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
