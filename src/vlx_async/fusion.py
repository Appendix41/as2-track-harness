"""Route fusion: one DeepSeek plan per task steers the VLX path.

The plan is frozen when the task starts. Each tick re-expresses it through the
current pose and rotates the VLX path toward the planned heading. Without depth
there are no measured obstacles and no target ranging, so nothing else vetoes a
path.
"""
import math

from .contracts import FusionConfig, TaskConfig
from .geometry import path_distance, rotate, wrap
from .task_plan import plan_status, route_heading, validate_route_shape


class Fusion:
    def __init__(self, config=None, task_config=None):
        self.config = config or FusionConfig()
        self.task_config = task_config or TaskConfig()
        self.key = None
        self.plan = None
        self.applied_blend = 0.
        self.last_rejection = None
        self.accepted_request_id = None

    def set_task(self, observation):
        if self.key != observation.task_key:
            self.key = observation.task_key
            self.plan = None
            self.applied_blend = 0.
            self.accepted_request_id = None
            self.last_rejection = None

    def accept(self, plan, current, now_ns):
        """Accept exactly one plan for the current task; never a second one."""
        self.set_task(current)
        if self.plan is not None:
            self.last_rejection = 'plan_already_active'
            return self.last_rejection
        reason = None
        if plan.task_key != current.task_key:
            reason = 'obsolete_task'
        elif not 0 <= (now_ns-plan.created_ns)/1e9 <= self.config.plan_max_age_s:
            reason = 'invalid_plan_clock'
        elif plan.confidence < self.config.min_confidence:
            reason = 'low_plan_confidence'
        elif plan.pose is None or plan.pose.frame_id != (current.context.pose.frame_id
                                                        if current.context.pose else None):
            reason = 'plan_pose_unavailable'
        else:
            reason = validate_route_shape(plan.points, self.task_config)
        self.last_rejection = reason
        if reason is None:
            self.plan = plan
            self.accepted_request_id = plan.request_id
        return reason

    def decide(self, obs, now_ns):
        self.set_task(obs)
        cfg, plan, ctx = self.config, self.plan, obs.context
        result = {
            'execution_enabled': False, 'robot_commands_sent': 0,
            'fast': list(obs.waypoints), 'plan_aligned': [], 'recommended': list(obs.waypoints),
            'candidates': [], 'selection': 'hold', 'reason': 'plan_pending',
            'task_execution_ready': False, 'execution_blockers': ['plan_not_ready'],
            'control_advice': 'hold_until_plan_ready',
            'plan_status': 'waiting_for_plan', 'plan_request_id': None,
            'plan_age_s': None, 'plan_confidence': None, 'plan_description': None,
            'remaining_route_m': None, 'total_route_m': None, 'cross_track_m': None,
            'progress_fraction': None, 'task_completed': False, 'plan_source': None,
            'last_rejection': self.last_rejection,
        }
        if plan is None:
            return result
        result.update(plan_request_id=plan.request_id, plan_confidence=plan.confidence,
                      plan_description=plan.description, plan_source=plan.source,
                      plan_status='accepted')
        try:
            status = plan_status(plan, ctx.pose, now_ns, self.task_config)
        except ValueError as exc:
            result.update(reason=str(exc), plan_status='blocked',
                          execution_blockers=[str(exc)], control_advice='hold_until_pose_ready')
            return result
        travelled = max(0., status['total_m']-status['remaining_m'])
        result.update(plan_age_s=status['age_s'], plan_aligned=list(status['route']),
                      remaining_route_m=status['remaining_m'], total_route_m=status['total_m'],
                      cross_track_m=status['cross_track_m'],
                      progress_fraction=status['progress_fraction'],
                      travelled_m=travelled,
                      task_completed=status['completed'])
        if status['completed'] or travelled >= self.task_config.max_travel_m:
            # Monocular route length is a direction aid, not a measurement: a
            # hard travel bound stops the task before it walks past the target.
            stopped_by = 'arrival' if status['completed'] else 'max_travel'
            result.update(reason='route_completed', selection='hold', recommended=[],
                          plan_status='completed', execution_blockers=['route_completed'],
                          control_advice='task_arrived', stopped_by=stopped_by)
            return result
        if status['cross_track_m'] > cfg.max_cross_track_m:
            result.update(reason='route_drift', plan_status='blocked',
                          execution_blockers=['route_drift'], control_advice='hold_until_replanned')
            return result
        # VLX candidates are kept for diagnostics only: with a plan they can no
        # longer be selected, and without one the robot holds.
        waypoints = [tuple(float(v) for v in p) for p in obs.waypoints]
        if len(waypoints) < 2 or math.hypot(waypoints[-1][0], waypoints[-1][1]) < .05:
            result.update(reason='invalid_vlx_path', plan_status='blocked',
                          execution_blockers=['invalid_vlx_path'])
            return result
        route = status['route']
        scale = max(.1, math.hypot(waypoints[-1][0], waypoints[-1][1]))
        weight = cfg.plan_weight
        # Weighted heading blend: the rotation applied to the VLX path is
        # weight x (plan heading - VLX heading), so the weight means exactly
        # "how much of the plan's direction is followed".
        # The executor commands path[1] (vx,vy from it, yaw from the rotation),
        # so the blend is measured against exactly that step.
        first = waypoints[1]
        reference = first if math.hypot(first[0], first[1]) >= 1e-4 else waypoints[-1]
        vlx_heading = math.atan2(reference[1], reference[0])
        try:
            # A short lookahead sees only the straight part of a curving plan
            # and makes the robot cut the corner; aim further along the route.
            near_heading = route_heading(route, cfg.plan_lookahead_m)
            end = route[-1]
            end_heading = math.atan2(end[1], end[0])
            weight_end = cfg.plan_endpoint_weight
            plan_heading = wrap(near_heading+weight_end*wrap(end_heading-near_heading))
        except ValueError:
            result.update(reason='plan_route_too_short', plan_status='blocked',
                          execution_blockers=['plan_route_too_short'])
            return result
        delta = wrap(plan_heading - vlx_heading)
        target = max(-cfg.max_heading_change_rad,
                     min(cfg.max_heading_change_rad, weight*delta))
        # A route vertex can step the heading by tens of degrees in one tick;
        # move toward it at a bounded rate instead of snapping the command.
        step = math.radians(6.0)
        blend = max(self.applied_blend-step, min(self.applied_blend+step, target))
        self.applied_blend = blend
        for step in range(-5, 6):
            angle = step*cfg.max_heading_change_rad/5
            path = rotate(waypoints, angle) if angle else list(waypoints)
            deviation = path_distance(path, waypoints)
            mismatch = path_distance(path, route)
            endpoint = math.hypot(path[-1][0]-route[-1][0], path[-1][1]-route[-1][1])
            cost = (1-weight)*deviation**2/scale + weight*(.7*mismatch+.3*endpoint)
            result['candidates'].append({'angle_rad': angle, 'path': path, 'cost': cost,
                                         'plan_mismatch_m': mismatch,
                                         'endpoint_error_m': endpoint, 'source': 'vlx'})
        recommended = rotate(waypoints, blend) if abs(blend) > 1e-9 else list(waypoints)
        result.update(recommended=recommended, plan_status='following',
                      selection='plan_aligned' if abs(blend) > 1e-9 else 'vlx_only',
                      reason='task_ready', task_execution_ready=True,
                      execution_blockers=[], control_advice='follow_task_plan',
                      chosen_angle_rad=blend, blend_target_rad=target,
                      plan_heading_rad=plan_heading, near_heading_rad=near_heading,
                      endpoint_heading_rad=end_heading, vlx_heading_rad=vlx_heading,
                      heading_gap_rad=delta, plan_weight_used=weight,
                      rotate_in_place=abs(target) > math.pi/2,
                      blend_limited=abs(weight*delta) > cfg.max_heading_change_rad)
        return result
