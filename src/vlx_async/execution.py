"""Independent configurable command gate and watchdog. Dry-run is the default.

The AS2 sink is created only after acquiring the SAME controller lock as the
original service. Its lock is opened read-only; no original file is modified.
This module never changes posture, gait, or the original web speed settings.
"""
from dataclasses import dataclass
import fcntl
import math
import threading
import time

from .contracts import points
from .geometry import first_path_collision
from .depth_obstacles import obstacles_enabled
from .runtime_limits import observed_speed_limit, DEFAULT_OBSERVED_SPEED_MPS, SDK_FLOAT_MAX

# Half the robot's body width, used against measured obstacle circles. The
# operator asked for a body-to-obstacle gap of 5-10 cm before braking, so this
# is the body radius, not a comfortable margin.
ROBOT_RADIUS_M = .20
OBSTACLE_BRAKE_GAP_M = .10


# Hold only the two remaining perception clocks.  The depth chain is gone, so an
# obstacle/target gap can no longer be a reason to pause: nothing measures them.
# These reasons brake to zero and keep the trial alive; everything else is a
# latched stop. A short DDS gap during a gait must not end a task.
PERCEPTION_WAIT_REASONS = frozenset({'camera_frame_stale', 'prediction_stale',
                                     'robot_telemetry_stale',
                                     'measured_obstacle_in_corridor'})

# Waiting for the task plan is a retryable preflight state, never a perception
# hold: the plan is frozen before motion and a missing one means "do not start yet".
RETRYABLE_WAIT_REASONS = PERCEPTION_WAIT_REASONS | {'plan_pending'}

# Reasons a task plan can block execution with.  They are passed through as-is so
# the executor/UI can show the real cause instead of a generic fusion error.
PLAN_BLOCKING_REASONS = frozenset({
    'plan_pending', 'route_completed', 'plan_requires_reverse', 'route_drift',
    'low_plan_confidence', 'no_feasible_candidate',
})


def plan_gate_reason(requested):
    """Map a task-plan reason to an execution reason; unknown states are held."""
    if (isinstance(requested, str) and requested
            and (requested in PLAN_BLOCKING_REASONS or requested.startswith('plan_rejected'))):
        return requested
    return 'plan_not_ready'


def wait_for_command(read, timeout_s, stopped, report, clock=time.monotonic, pause=time.sleep):
    """Wait while stationary for current evidence; never replay an earlier command."""
    deadline = clock()+timeout_s
    retryable = RETRYABLE_WAIT_REASONS | {
        'robot_telemetry_stale',
        'control_watchdog_gap', 'execution_packet_stale_or_inactive',
    }
    while not stopped():
        try:
            now, command = read()
            if command.get('allowed') is True:
                return now, command
            reason = command.get('reason', 'invalid_preflight_command')
        except (OSError, ValueError) as exc:
            reason = 'execution_packet_unavailable' if isinstance(exc, OSError) else str(exc)
        if reason not in retryable and reason != 'execution_packet_unavailable':
            raise ValueError('execution_preflight_rejected:'+reason)
        report(reason)
        if clock() >= deadline:
            raise ValueError('current_fusion_wait_timeout:'+reason)
        pause(.025)
    raise ValueError('operator_stopped_before_motion')


@dataclass(frozen=True)
class Limits:
    max_forward_mps: float = .05
    max_lateral_mps: float = .03
    max_linear_mps: float = .05
    max_yaw_rps: float = .10
    max_linear_accel_mps2: float = .10
    max_yaw_accel_rps2: float = .20
    reference_seconds: float = .10
    heading_gain: float = 2.0
    max_telemetry_age_s: float = .25
    max_prediction_age_s: float = .35
    max_control_gap_s: float = .20
    max_observed_speed_mps: float = DEFAULT_OBSERVED_SPEED_MPS

    def __post_init__(self):
        observed_speed_limit(self.max_observed_speed_mps)
        for value in self.__dict__.values():
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid_execution_limit')
        if (any(v > SDK_FLOAT_MAX for v in (self.max_forward_mps, self.max_lateral_mps,
                self.max_linear_mps, self.max_yaw_rps, self.max_linear_accel_mps2, self.max_yaw_accel_rps2))
                or self.max_control_gap_s > .20 or self.max_telemetry_age_s > .25
                or self.max_prediction_age_s > .35
                or not .5 <= self.heading_gain <= 10.):
            raise ValueError('outside_bounded_trial_limits')


class CommandGate:
    def __init__(self, limits=None, controller_lease=None):
        self.limits = limits or Limits()
        self.controller_lease = controller_lease
        self.last_ns, self.last_key, self.previous = 0, None, (0., 0., 0.)

    def evaluate(self, decision, obs, telemetry, now_ns):
        limits = self.limits
        reason = None
        if self.last_key is not None and self.last_key != obs.task_key:
            reason = 'execution_task_changed'
        elif self.last_ns and not 0 < (now_ns-self.last_ns)/1e9 <= limits.max_control_gap_s:
            reason = 'control_watchdog_gap'
        elif not 0 <= (now_ns-obs.camera_ns)/1e9 <= .50:
            reason = 'camera_frame_stale'
        elif not 0 <= (now_ns-obs.prediction_ns)/1e9 <= limits.max_prediction_age_s:
            reason = 'prediction_stale'
        elif decision.get('camera_index') != obs.camera_index or decision.get('prediction_index') != obs.prediction_index:
            reason = 'decision_prediction_mismatch'
        elif (decision.get('stream_id'), decision.get('host_boot_id'), decision.get('instruction_revision')) != (obs.stream_id, obs.boot_id, obs.revision):
            reason = 'decision_task_mismatch'
        elif decision.get('task_execution_ready') is not True:
            # One plan per task: the frozen task plan is the only execution
            # authority. There is no depth/target/obstacle evidence left to veto
            # or confirm a path.
            reason = plan_gate_reason(decision.get('reason'))
        sport, low = telemetry.get('sport', {}), telemetry.get('low') or {}
        # Robot/remote vetoes take priority even while waiting for evidence.
        # A remote intervention must not be hidden by a pending plan or clock.
        if not reason or reason in RETRYABLE_WAIT_REASONS:
            stamp = sport.get('received_monotonic_ns')
            low_stamp = low.get('received_monotonic_ns')
            if (telemetry.get('error') or type(stamp) is not int or type(low_stamp) is not int
                or not 0 <= (now_ns-stamp)/1e9 <= limits.max_telemetry_age_s
                or not 0 <= (now_ns-low_stamp)/1e9 <= limits.max_telemetry_age_s):
                reason = 'robot_telemetry_stale'
            elif sport.get('mode') != 3 or sport.get('error_code') != 0 or low.get('mode_machine') != 1:
                reason = 'robot_not_in_ready_mode'
            elif len(sport.get('rpy', [])) != 3 or not all(math.isfinite(v) for v in sport['rpy']) or max(abs(v) for v in sport['rpy'][:2]) > .35:
                reason = 'robot_tilt_invalid'
            elif (not isinstance(sport.get('velocity'), (list, tuple)) or len(sport['velocity']) != 3
                  or any(type(v) not in (int,float) or not math.isfinite(v) for v in sport['velocity'])):
                reason = 'robot_velocity_invalid'
            elif math.hypot(*sport['velocity'][:2]) > limits.max_observed_speed_mps:
                reason = 'observed_speed_limit'
            elif (telemetry.get('original_manual_enabled') is True or telemetry.get('original_vlx_active') is True
                  or ((telemetry.get('original_manual_enabled') is not False or telemetry.get('original_vlx_active') is not False)
                      and not (self.controller_lease and self.controller_lease.file is not None))):
                reason = 'original_controller_active_or_unknown'
            else:
                axes = telemetry.get('remote_axes') or {}
                if (set(axes) != {'lx','ly','rx','ry','keys'} or axes.get('keys') != 0
                    or any(type(axes.get(k)) not in (int, float) or not math.isfinite(axes[k]) or abs(axes[k]) > .05 for k in ('lx','ly','rx','ry'))):
                    reason = 'remote_input_or_unknown_state'
        path = []
        if not reason:
            try:
                path = points(decision.get('recommended'), count=8)
            except (TypeError, ValueError):
                reason = 'invalid_fused_path'
        obstacle_conflict = (first_path_collision(path, obs.context.obstacles, ROBOT_RADIUS_M)
                             if path and obstacles_enabled() else None)
        # The recommended path is only the next few centimetres, so also brake on
        # the measured clearance straight ahead: an obstacle 0.55 m away is
        # already inside the stopping distance at walking speed.
        clearance = None
        for ox, oy, radius in (obs.context.obstacles or () if obstacles_enabled() else ()):
            if abs(oy) <= .35:
                # Gap between the robot's body edge and the obstacle edge.
                gap = math.hypot(ox, oy)-radius-ROBOT_RADIUS_M
                clearance = gap if clearance is None else min(clearance, gap)
        if not reason and clearance is not None and clearance <= OBSTACLE_BRAKE_GAP_M:
            reason = 'measured_obstacle_in_corridor'
        if not reason and obstacle_conflict is not None:
            # Measured, not guessed: the depth camera says the next steps are
            # inside a solid object. Brake and say so; the operator/plan decides.
            reason = 'measured_obstacle_in_corridor'
        dt = min(.05, (now_ns-self.last_ns)/1e9) if self.last_ns else .05
        self.last_ns, self.last_key = now_ns, obs.task_key
        if reason:
            # A perception gap is a wait, not a stop: brake within the acceleration
            # window and keep the ramp state so the stride is not restarted from
            # zero every time the camera/prediction clock drops out for a frame.
            # Plan states (plan_pending/route_completed/...) send explicit zeros.
            hold = reason in PERCEPTION_WAIT_REASONS
            if hold:
                def brake(value, step):
                    return math.copysign(max(0., abs(value)-step), value) if value else 0.
                held = (brake(self.previous[0], limits.max_linear_accel_mps2*dt),
                        brake(self.previous[1], limits.max_linear_accel_mps2*dt),
                        brake(self.previous[2], limits.max_yaw_accel_rps2*dt))
                self.previous = held
            else:
                held = (0., 0., 0.)
                self.previous = held
            return dict(allowed=False, reason=reason, command_mps_rps=list(held),
                        fusion_reason=decision.get('reason'),
                        hold_for_perception=hold, execution_enabled=False,
                        obstacle_conflict=obstacle_conflict)
        if decision.get('rotate_in_place') is True:
            # Target behind the robot: turn on the spot first, never strafe.
            yaw = decision.get('chosen_angle_rad') or 1.
            vx = vy = 0.
        else:
            x, y, yaw = path[1]
            vx = max(0., min(limits.max_forward_mps, x/limits.reference_seconds))
            vy = max(-limits.max_lateral_mps, min(limits.max_lateral_mps, y/limits.reference_seconds))
            yaw = decision.get('chosen_angle_rad', yaw)
        # A heading error is a proportional request, not "reach it in 0.1 s":
        # the old angle/0.1 gain turned a 44 deg gap into 6 rad/s and could spin
        # the robot into nearby furniture.
        wz = min(limits.max_yaw_rps, limits.heading_gain*abs(yaw))
        wz = math.copysign(wz, yaw) if yaw else 0.
        speed = math.hypot(vx, vy)
        if speed > limits.max_linear_mps:
            vx, vy = vx*limits.max_linear_mps/speed, vy*limits.max_linear_mps/speed
        targets = (vx, vy, wz)
        delta_xy = (vx-self.previous[0], vy-self.previous[1])
        delta_norm = math.hypot(*delta_xy)
        scale = min(1., limits.max_linear_accel_mps2*dt/max(delta_norm, 1e-12))
        command = (self.previous[0]+delta_xy[0]*scale, self.previous[1]+delta_xy[1]*scale,
                   self.previous[2]+max(-limits.max_yaw_accel_rps2*dt,
                       min(limits.max_yaw_accel_rps2*dt, wz-self.previous[2])))
        self.previous = command
        return dict(allowed=True, reason='bounded_command_validated', command_mps_rps=list(command),
                    execution_enabled=False, raw_target_mps_rps=list(targets))


class ControllerLease:
    def __init__(self, path):
        self.path, self.file = path, None

    def acquire(self):
        self.file = open(self.path, 'rb')  # existing lock; no writes in original tree
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self.file.close()
            self.file = None
            raise
        return self

    def close(self):
        if self.file:
            self.file.close()
            self.file = None


class GuardedActuator:
    """Serial RPC owner; optionally hold for perception inside an active trial.

    Holds require a confirmed stop before later fresh evidence may resume motion.
    Watchdog, robot/remote vetoes, RPC errors and deadlines remain latched faults.
    The sink must enforce an RPC timeout <= .10 s.
    """
    def __init__(self, sink, duration_s=3., watchdog_s=.20, clock=time.monotonic, limits=None,
                 allow_perception_hold=False):
        if not 0 < duration_s <= 86400 or not .05 <= watchdog_s <= .20:
            raise ValueError('invalid_trial_duration_or_watchdog')
        self.limits = limits or Limits()
        self.sink, self.clock = sink, clock
        self.duration_s, self.watchdog_s = duration_s, watchdog_s
        self.lock, self.done = threading.RLock(), threading.Event()
        self.started = self.heartbeat = clock()
        self.fault, self.sent, self.stop_codes = None, 0, []
        self.allow_perception_hold = allow_perception_hold
        self.holding, self.hold_count = False, 0
        self.thread = threading.Thread(target=self._watch, daemon=True)
        self.thread.start()

    def _stop(self, reason):
        if self.fault is None:
            self.fault = reason
            # Bounded retries, recorded: a stop timeout is never a confirmed stop.
            for _ in range(3):
                try:
                    code = self.sink.stop()
                except Exception:
                    code = None
                self.stop_codes.append(code)
                if code == 0:
                    break

    def submit(self, command):
        with self.lock:
            now = self.clock()
            if self.fault:
                return False
            if now-self.started >= self.duration_s or now-self.heartbeat > self.watchdog_s:
                self._stop('motion_duration_complete' if now-self.started >= self.duration_s else 'control_watchdog_expired')
                return False
            values = command.get('command_mps_rps')
            if (self.allow_perception_hold and command.get('allowed') is False
                    and command.get('hold_for_perception') is True
                    and command.get('reason') in PERCEPTION_WAIT_REASONS
                    and isinstance(values, (list, tuple)) and len(values) == 3
                    and all(type(v) in (int, float) and math.isfinite(v) for v in values)
                    and 0 <= values[0] <= self.limits.max_forward_mps
                    and abs(values[1]) <= self.limits.max_lateral_mps
                    and math.hypot(*values[:2]) <= self.limits.max_linear_mps+1e-6
                    and abs(values[2]) <= self.limits.max_yaw_rps):
                # Keep the gait alive while braking: no StopMove, so the next
                # allowed frame continues from the ramped-down speed.
                if not self.holding:
                    self.holding = True
                    self.hold_count += 1
                self.heartbeat = self.clock()
                try:
                    code = self.sink.move(*values)
                except Exception:
                    code = None
                if code != 0:
                    self._stop('perception_hold_move_failed')
                    return False
                return False
            if (command.get('allowed') is not True or not isinstance(values, (list, tuple)) or len(values) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
                or not 0 <= values[0] <= self.limits.max_forward_mps
                or abs(values[1]) > self.limits.max_lateral_mps
                or math.hypot(*values[:2]) > self.limits.max_linear_mps + 1e-6
                or abs(values[2]) > self.limits.max_yaw_rps):
                self._stop('input_gate_rejected')
                return False
            self.heartbeat = now
            self.holding = False
            try:
                code = self.sink.move(*values)
                self.sent += 1
            except Exception:
                code = None
            if code != 0:
                self._stop('motion_rpc_failed')
                return False
            return True

    def _watch(self):
        while not self.done.wait(.02):
            with self.lock:
                now = self.clock()
                if now-self.started >= self.duration_s or now-self.heartbeat > self.watchdog_s:
                    self._stop('motion_duration_complete' if now-self.started >= self.duration_s else 'control_watchdog_expired')

    def close(self):
        self.done.set()
        with self.lock:
            self._stop('executor_closed')
        self.thread.join(timeout=1)


class AS2Sink:
    """Uses the isolated copied SDK. Caller must hold ControllerLease."""
    def __init__(self, lease, interface='end0', domain=0):
        if lease.file is None:
            raise ValueError('controller_lock_required')
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.as2.sport.sport_client import SportClient
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        self.lease = lease
        ChannelFactoryInitialize(domain, interface)
        client = SportClient()
        client.SetTimeout(.10)
        client.Init()
        version = client.GetServerApiVersion()
        state = {}
        state_code = client.GetState(state)
        switcher = MotionSwitcherClient()
        switcher.SetTimeout(.10)
        switcher.Init()
        mode_code, mode = switcher.CheckMode()
        if tuple(version) != (0, '1.0.0.1') or state_code != 0 or mode_code != 0 or not isinstance(mode,dict) or mode.get('name') != 'ai':
            raise ValueError('as2_read_only_rpc_preflight_failed')
        self.client = client

    def move(self, x, y, yaw):
        if self.lease.file is None:
            raise ValueError('controller_lease_lost')
        return self.client.Move(x, y, yaw)

    def stop(self):
        return self.client.StopMove()


def packet_observation(packet, now_ns, boot_id):
    """Decode only a fresh same-boot packet; never refresh its evidence stamps."""
    from .contracts import Observation, Context, Pose, LocalCandidate
    stamp = packet.get('timestamp_ns')
    if (packet.get('schema') != 'vlx-fusion-execution-packet-v1' or packet.get('status') != 'shadow'
            or type(stamp) is not int or not 0 <= now_ns-stamp <= 200_000_000):
        raise ValueError('execution_packet_stale_or_inactive')
    if packet.get('session_id'):
        startup = (packet.get('decision') or {}).get('startup') or {}
        if startup.get('session_id') != packet['session_id'] or startup.get('first_fusion_complete') is not True:
            raise ValueError('first_fusion_not_complete')
    raw = dict(packet['observation'])
    if raw.get('boot_id') != boot_id:
        raise ValueError('execution_packet_other_boot')
    ctx = dict(raw['context'])
    ctx['pose'] = Pose.parse(ctx.get('pose'))
    candidates = []
    for item in ctx['local_candidates']:
        item = dict(item,attributes=tuple(item['attributes']),bbox=tuple(item['bbox']))
        if item.get('local_xy') is not None:
            item['local_xy'] = tuple(item['local_xy'])
        candidates.append(LocalCandidate(**item))
    ctx['local_candidates'] = tuple(candidates)
    ctx['obstacles'] = tuple(tuple(v) for v in ctx['obstacles'])
    ctx['observed_free_cells'] = tuple(tuple(v) for v in ctx.get('observed_free_cells',[]))
    raw.update(context=Context(**ctx),waypoints=points(raw['waypoints'],count=8),image_base64=None)
    return Observation(**raw)
