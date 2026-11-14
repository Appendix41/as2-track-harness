"""Historical single-step tool: limits and telemetry validation only.

Depth gating is deleted.  What remains is used by the controller handoff and by
the runtime-limit tests: the bounded step envelope, read-only telemetry checks,
stationarity evidence and the displacement/rotation/speed watchdog.  The old
``prepare_step`` route (depth/obstacle/candidate validation) is gone with the
D435i chain and no longer exists in this module.
"""
from dataclasses import dataclass
import math

from .geometry import wrap
from .runtime_limits import observed_speed_limit, DEFAULT_OBSERVED_SPEED_MPS


@dataclass(frozen=True)
class StepLimits:
    duration_s: float = 2.
    max_speed_mps: float = .03
    max_lateral_mps: float = .015
    max_yaw_rps: float = .05
    max_displacement_m: float = .10
    max_rotation_rad: float = .12
    max_scene_age_s: float = .30
    max_telemetry_age_s: float = .20
    max_observed_speed_mps: float = DEFAULT_OBSERVED_SPEED_MPS

    def __post_init__(self):
        observed_speed_limit(self.max_observed_speed_mps)
        if any(type(v) not in (float,int) or not math.isfinite(v) or v<=0 for v in vars(self).values()):
            raise ValueError('invalid_step_limit')
        if (self.duration_s>2 or self.max_speed_mps>.03 or self.max_lateral_mps>.015
            or self.max_yaw_rps>.05 or self.max_displacement_m>.10 or self.max_rotation_rad>.12
            or self.max_scene_age_s>.30 or self.max_telemetry_age_s>.20):
            raise ValueError('outside_supervised_step_envelope')


def validate_telemetry(data,now_ns,boot_id,limits):
    if (data.get('status')!='running' or data.get('host_boot_id')!=boot_id
        or type(data.get('timestamp_ns')) is not int
        or not 0<=now_ns-data['timestamp_ns']<=limits.max_telemetry_age_s*1e9):
        raise ValueError('step_telemetry_stale')
    sport=data.get('sport') or {};low=data.get('low') or {};remote=data.get('remote_axes') or {}
    for state in (sport,low):
        stamp=state.get('received_monotonic_ns')
        if type(stamp) is not int or not 0<=now_ns-stamp<=limits.max_telemetry_age_s*1e9:
            raise ValueError('step_robot_state_stale')
    if sport.get('mode')!=3 or sport.get('error_code')!=0 or low.get('mode_machine')!=1:
        raise ValueError('step_robot_not_ready')
    for key in ('position','rpy','velocity'):
        value=sport.get(key)
        if not isinstance(value,list) or len(value)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in value):
            raise ValueError('step_invalid_'+key)
    if max(abs(v) for v in sport['rpy'][:2])>.25:
        raise ValueError('step_body_tilt')
    if type(sport.get('yaw_speed')) not in (int,float) or not math.isfinite(sport['yaw_speed']):
        raise ValueError('step_invalid_yaw_speed')
    if (set(remote)!={'lx','ly','rx','ry','keys'} or remote['keys']!=0
        or any(type(remote[k]) not in (int,float) or not math.isfinite(remote[k]) or abs(remote[k])>.05 for k in ('lx','ly','rx','ry'))):
        raise ValueError('step_remote_takeover')
    return sport


def stationary_evidence(samples):
    if len(samples)<20 or samples[-1][0]-samples[0][0]<2_000_000_000:
        return False
    first=samples[0][1]
    for _,sport in samples:
        if (not math.isfinite(sport.get('yaw_speed',float('nan')))
            or math.hypot(*sport['velocity'][:2])>.02 or abs(sport['yaw_speed'])>.04
            or math.hypot(sport['position'][0]-first['position'][0],sport['position'][1]-first['position'][1])>.02
            or abs(wrap(sport['rpy'][2]-first['rpy'][2]))>.03):
            return False
    return True


class StepController:
    def __init__(self, prepared, sport, now_ns, limits=None):
        self.prepared,self.initial,self.start_ns=prepared,sport,now_ns
        self.last_ns=now_ns
        self.limits=limits or StepLimits()
        self.fault=None

    def evaluate(self,sport,now_ns):
        if self.fault:
            raise ValueError(self.fault)
        elapsed=(now_ns-self.start_ns)/1e9
        dx=sport['position'][0]-self.initial['position'][0]
        dy=sport['position'][1]-self.initial['position'][1]
        yaw=abs(wrap(sport['rpy'][2]-self.initial['rpy'][2]))
        reason=None
        if not 0<=now_ns-self.last_ns<=150_000_000:reason='step_control_gap'
        elif elapsed>=self.limits.duration_s:reason='step_duration_complete'
        elif math.hypot(dx,dy)>=self.limits.max_displacement_m:reason='step_displacement_limit'
        elif yaw>=self.limits.max_rotation_rad:reason='step_rotation_limit'
        elif math.hypot(*sport['velocity'][:2])>self.limits.max_observed_speed_mps or abs(sport.get('yaw_speed',float('inf')))>.20:reason='step_observed_speed_limit'
        self.last_ns=now_ns
        if reason:
            self.fault=reason
            raise ValueError(reason)
        # Feed only this frozen fused command during the two-second test. No
        # asynchronous image/odometry reprojection is used during the motion.
        ramp=min(1.,max(0.,elapsed)/.5)
        return dict(allowed=True,command_mps_rps=[v*ramp for v in self.prepared['command_mps_rps']],
                    reason='stationary_fused_step',elapsed_s=elapsed,displacement_m=math.hypot(dx,dy),rotation_rad=yaw)
