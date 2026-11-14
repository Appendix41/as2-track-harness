"""First-plan barrier: a trial waits for its own validated plan before departure.

The single DeepSeek call happens while the robot is stationary. The barrier
holds the trial open until the plan is accepted (``task_ready``). It does not
re-validate a running task and does not replace the execution gate.
"""
import time


WAITING_VLX = 'waiting_vlx'
WAITING_PLAN = 'waiting_plan'
PLAN_READY = 'plan_ready'
PLAN_REJECTED = 'plan_rejected'
PLAN_TIMEOUT = 'plan_timeout'

TERMINAL_PHASES = frozenset({PLAN_READY, PLAN_REJECTED, PLAN_TIMEOUT})

# Rejections produced while validating a plan that was actually received for
# this task; an obsolete response from an older task is not such a rejection.
PLAN_REJECTION_REASONS = frozenset({
    'invalid_plan_clock', 'low_plan_confidence', 'plan_pose_unavailable',
    'plan_route_too_short', 'plan_requires_reverse', 'plan_route_invalid',
})


class StartupBarrier:
    def __init__(self, session_id, timeout_s=90., clock=time.monotonic):
        if not session_id or not 1 <= timeout_s <= 86400:
            raise ValueError('invalid_startup_barrier')
        self.session_id, self.timeout_s, self.clock = session_id, timeout_s, clock
        self.started = clock()
        self.task = None
        self.request_id = None
        self.phase = WAITING_VLX
        self.reason = 'source_not_running'
        self.ready_at = None

    def update(self, decision, valid_input):
        if not isinstance(decision, dict):
            raise ValueError('invalid_startup_decision')
        if self.phase in TERMINAL_PHASES:
            if valid_input:
                key = self._task(decision)
                if self.task is None:
                    self.task = key
                elif key != self.task:
                    self.phase, self.reason = PLAN_REJECTED, 'startup_task_changed'
            return self.snapshot()
        if self.clock()-self.started >= self.timeout_s:
            self.phase, self.reason = PLAN_TIMEOUT, 'first_plan_timeout'
        elif not valid_input:
            self.phase, self.reason = WAITING_VLX, decision.get('reason', 'source_not_running')
        else:
            key = self._task(decision)
            if self.task is None:
                self.task = key
            if key != self.task:
                self.phase, self.reason = PLAN_REJECTED, 'startup_task_changed'
            else:
                self._await_plan(decision)
        return self.snapshot()

    def _await_plan(self, decision):
        reason = decision.get('reason')
        task = decision.get('task') if isinstance(decision.get('task'), dict) else {}
        if 'plan_request_id' in decision:
            plan_request_id = decision.get('plan_request_id')
        else:
            plan_request_id = task.get('plan_request_id')
        plan_calls = decision.get('plan_calls', task.get('plan_calls'))
        recommended = decision.get('recommended')
        if (reason == 'task_ready'
                and decision.get('task_execution_ready') is True
                and isinstance(plan_request_id, str) and plan_request_id
                and isinstance(recommended, (list, tuple)) and len(recommended) == 8
                and (plan_calls is None or (type(plan_calls) is int and plan_calls >= 1))):
            self.request_id = plan_request_id
            self.ready_at = self.clock()
            self.phase, self.reason = PLAN_READY, 'first_plan_ready'
            return
        if isinstance(reason, str) and reason.startswith('plan_rejected'):
            self.phase, self.reason = PLAN_REJECTED, reason
            return
        blocked = task.get('blocked_reason')
        if (task.get('phase') == 'blocked' and isinstance(blocked, str)
                and (blocked.startswith('plan_rejected')
                     or blocked.startswith('plan_projection_failed'))):
            self.phase, self.reason = PLAN_REJECTED, blocked
            return
        rejection = decision.get('last_rejection')
        if (not plan_request_id and isinstance(rejection, str)
                and rejection in PLAN_REJECTION_REASONS):
            self.phase, self.reason = PLAN_REJECTED, 'plan_rejected:'+rejection
            return
        self.phase = WAITING_PLAN
        self.reason = reason if isinstance(reason, str) and reason else 'waiting_for_plan'

    @staticmethod
    def _task(decision):
        return (decision.get('host_boot_id') or decision.get('boot_id'),
                decision.get('stream_id'),
                decision.get('instruction_revision'), decision.get('instruction'))

    def snapshot(self):
        ready = self.phase == PLAN_READY
        return dict(session_id=self.session_id, phase=self.phase, reason=self.reason,
                    plan_ready=ready, first_fusion_complete=ready,
                    first_request_id=self.request_id, plan_request_id=self.request_id,
                    waiting_elapsed_s=(self.ready_at if self.ready_at is not None else self.clock())-self.started,
                    timeout_s=self.timeout_s, ready_at_monotonic=self.ready_at)
