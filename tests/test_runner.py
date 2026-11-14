"""Runner task lifecycle: one plan per task, route projection and phase windows.

The runner is the only place that talks to the planner, so these tests pin the
two properties the contract depends on: a task can request exactly one plan and
the accepted route is expressed in the pose of the image the model saw, never
in the pose of the tick that consumes the answer.
"""
import math
import unittest

from vlx_async.contracts import Context, Observation, Pose, Request, SlowPlan, TaskConfig
from vlx_async.ground import load_intrinsics
from vlx_async.runner import (TaskLoop, plan_route, request_pose, runner_budget,
                              task_config)

MOUNT = {
    'height_m': .50, 'pitch_rad': 0.,
    'intrinsics': {'width': 640, 'height': 480, 'fx': 605.3713989257812,
                   'fy': 605.1307983398438, 'ppx': 330.8877868652344,
                   'ppy': 257.2687683105469},
}


def pose(x=0., y=0., yaw=0., stamp=1_000_000_000, frame='odom'):
    return Pose(x, y, yaw, stamp, frame)


def observation(pose_value=None, task=('boot', 'stream', 1, 'walk to the chair')):
    boot, stream, revision, instruction = task
    return Observation(stream, boot, revision, instruction, 5, 6, 1_000_000_000,
                       1_010_000_000, tuple((i*.25, 0., 0.) for i in range(1, 9)),
                       Context(pose=pose_value if pose_value is not None else pose()))


def slow_plan(observation, frame='image_uv', waypoints=((.5, .95, 0.), (.52, .85, 0.),
                                                        (.55, .75, 0.)),
              confidence=.9, completed_ns=1_010_000_000):
    return SlowPlan(Request('req-1', observation, 1_000_000_001), completed_ns, frame,
                    waypoints, confidence, None, False, 'openai_responses', 'forward chair',
                    False)


class TaskConfigTests(unittest.TestCase):
    def test_duration_caps_the_task_window(self):
        task = task_config({'task': {'task_timeout_s': 120., 'arrival_distance_m': .4}}, 30.)
        self.assertEqual(task.task_timeout_s, 30.)
        self.assertEqual(task.plan_timeout_s, 30.)
        self.assertEqual(task.arrival_distance_m, .4)
        self.assertEqual(task_config({}, 7.).task_timeout_s, 7.)

    def test_invalid_task_config_is_rejected(self):
        with self.assertRaises(ValueError):
            task_config({'task': {'arrival_distance_m': 5.}}, 30.)
        with self.assertRaises(ValueError):
            task_config({'task': {'max_plan_calls': 2}}, 30.)


class RunnerBudgetTests(unittest.TestCase):
    def test_every_run_has_a_hard_wall_clock_bound(self):
        # The margin keeps the last packet and the summary inside the budget.
        self.assertEqual(runner_budget(90., 30., 10., True), 132.)
        self.assertEqual(runner_budget(90., 30., 10., False), 42.)
        self.assertGreater(runner_budget(90., .5, 0., False), .5)


class TaskLoopTests(unittest.TestCase):
    def make(self, **options):
        clock = [0.]
        values = dict(task_timeout_s=10., plan_timeout_s=2.)
        values.update(options)
        return TaskLoop(TaskConfig(**values), clock=lambda: clock[0]), clock

    def test_one_plan_request_per_task(self):
        loop, clock = self.make()
        loop.begin(('boot', 'stream', 1, 'go'))
        self.assertEqual(loop.phase, 'planning')
        self.assertEqual(loop.plan_calls, 0)
        loop.note_submission()
        self.assertEqual(loop.plan_calls, 1)
        with self.assertRaises(ValueError):
            loop.note_submission()          # max_plan_calls is one by default

    def test_close_planning_reports_a_plan_timeout(self):
        loop, clock = self.make()
        loop.close_planning()               # idle before the first task
        self.assertEqual(loop.phase, 'blocked')
        self.assertEqual(loop.blocked_reason, 'plan_timeout')
        self.assertTrue(loop.plan_timeout)
        loop.begin(('boot', 'stream', 1, 'go'))
        loop.note_submission()
        loop.plan_accepted('req-1', now=0.)
        loop.close_planning()               # an accepted task is unaffected
        self.assertEqual(loop.phase, 'ready')

    def test_new_task_key_reopens_exactly_one_request(self):
        loop, clock = self.make()
        loop.begin(('boot', 'stream', 1, 'go'))
        loop.note_submission()
        loop.plan_accepted('req-1')
        loop.observe({'task_execution_ready': True, 'reason': 'task_ready'})
        self.assertEqual(loop.phase, 'executing')
        loop.observe({'task_completed': True})
        self.assertEqual(loop.phase, 'completed')
        clock[0] = 999.
        self.assertIsNone(loop.expire())    # a finished task is final
        loop.begin(('boot', 'stream', 2, 'go'))
        self.assertEqual(loop.phase, 'planning')
        self.assertEqual(loop.plan_calls, 0)
        self.assertIsNone(loop.plan_request_id)

    def test_plan_timeout_blocks_and_never_retries_on_its_own(self):
        loop, clock = self.make(plan_timeout_s=2.)
        loop.begin(('boot', 'stream', 1, 'go'), now=0.)
        clock[0] = 1.9
        self.assertIsNone(loop.expire())
        clock[0] = 2.1
        self.assertEqual(loop.expire(), 'plan_timeout')
        self.assertEqual(loop.phase, 'blocked')
        self.assertTrue(loop.plan_timeout)
        self.assertEqual(loop.blocked_reason, 'plan_timeout')
        clock[0] = 100.
        self.assertIsNone(loop.expire())
        loop.begin(('boot', 'stream', 2, 'go'))
        self.assertEqual(loop.phase, 'planning')
        self.assertFalse(loop.plan_timeout)

    def test_task_timeout_closes_an_executing_task(self):
        loop, clock = self.make(task_timeout_s=5.)
        loop.begin(('boot', 'stream', 1, 'go'), now=0.)
        loop.note_submission()
        loop.plan_accepted('req-1', now=1.)
        clock[0] = 5.5
        self.assertIsNone(loop.expire())
        clock[0] = 6.5
        self.assertEqual(loop.expire(), 'task_timeout')
        self.assertEqual(loop.phase, 'blocked')
        self.assertEqual(loop.blocked_reason, 'task_timeout')

    def test_route_drift_blocks_the_frozen_plan(self):
        loop, clock = self.make()
        loop.begin(('boot', 'stream', 1, 'go'))
        loop.note_submission()
        loop.plan_accepted('req-1', now=0.)
        loop.observe({'plan_status': 'blocked', 'reason': 'route_drift'})
        self.assertEqual(loop.phase, 'blocked')
        self.assertEqual(loop.blocked_reason, 'route_drift')

    def test_deadline_tracks_planning_then_the_task_window(self):
        loop, clock = self.make(task_timeout_s=10., plan_timeout_s=2.)
        loop.begin(('boot', 'stream', 1, 'go'), now=100.)
        self.assertEqual(loop.deadline(), 102.)
        loop.plan_accepted('req-1', now=101.)
        self.assertEqual(loop.deadline(keep_alive_s=.5), 111.5)

    def test_summary_exposes_the_task_block(self):
        loop, clock = self.make()
        loop.begin(('boot', 'stream', 1, 'go'))
        loop.note_submission()
        loop.plan_accepted('req-1', now=0.)
        block = loop.summary({'task_execution_ready': True, 'reason': 'task_ready',
                              'remaining_route_m': 2., 'total_route_m': 4.,
                              'progress_fraction': .5, 'task_completed': False}, now=2.)
        self.assertEqual(block['phase'], 'ready')
        self.assertEqual(block['plan_calls'], 1)
        self.assertEqual(block['plan_request_id'], 'req-1')
        self.assertEqual(block['remaining_route_m'], 2.)
        self.assertEqual(block['progress_fraction'], .5)
        self.assertFalse(block['completed'])
        self.assertFalse(block['plan_timeout'])


class RouteProjectionTests(unittest.TestCase):
    def test_image_route_uses_the_request_frame_pose(self):
        request_pose_value = pose(1., 2., .1, 1_000_000_000, 'odom')
        current = observation(request_pose_value)
        route = plan_route(slow_plan(current), load_intrinsics(MOUNT),
                           TaskConfig(max_range_m=3.))
        self.assertEqual(route.task_key, current.task_key)
        # The pose of the image the model saw, not a later tick's pose.
        self.assertIs(route.pose, current.context.pose)
        self.assertEqual(route.source, 'monocular_ground')
        self.assertEqual(route.points[0][:2], (0., 0.))
        self.assertGreater(len(route.points), 2)
        self.assertLessEqual(max(math.hypot(x, y) for x, y, _ in route.points), 3.+1e-9)
        self.assertEqual(route.calls, 1)

    def test_metric_route_is_used_unchanged(self):
        current = observation()
        plan = slow_plan(current, 'observation_local_m',
                         ((0., 0., 0.), (1., 0., 0.), (2., 0., 0.)), .8)
        route = plan_route(plan, load_intrinsics(MOUNT), TaskConfig())
        self.assertEqual(route.source, 'observation_local_m')
        self.assertEqual(route.points, ((0., 0., 0.), (1., 0., 0.), (2., 0., 0.)))

    def test_unusable_plan_is_an_error_not_a_silent_shortcut(self):
        current = observation()
        with self.assertRaises(ValueError):
            plan_route(slow_plan(current, waypoints=((.5, .2, 0.), (.5, .1, 0.))),
                       load_intrinsics(MOUNT), TaskConfig())
        with self.assertRaises(ValueError):
            plan_route(slow_plan(current, 'semantic_anchor', ()),
                       load_intrinsics(MOUNT), TaskConfig())


class RequestPoseTests(unittest.TestCase):
    def robot(self, at, latest):
        class Robot:
            def pose_at(self, stamp):
                return at
            def latest_pose(self):
                return latest
        return Robot()

    def test_interpolated_pose_wins(self):
        interpolated = pose(stamp=5)
        self.assertIs(request_pose(self.robot(interpolated, pose(stamp=9)), 5, .1), interpolated)

    def test_latest_pose_only_within_the_allowed_skew(self):
        latest = pose(stamp=1_100_000_000)
        self.assertIs(request_pose(self.robot(None, latest), 1_000_000_000, .1), latest)
        self.assertIsNone(request_pose(self.robot(None, latest), 1_000_000_000, .05))
        self.assertIsNone(request_pose(self.robot(None, None), 1_000_000_000, .1))


if __name__ == '__main__':
    unittest.main()
