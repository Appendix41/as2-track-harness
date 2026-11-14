"""One plan per task: monocular route projection, progress and fusion decisions."""
import math
import unittest
from dataclasses import replace

from vlx_async.contracts import Context, FusionConfig, Observation, Pose, Request, TaskConfig
from vlx_async.fusion import Fusion
from vlx_async.ground import image_point_to_local, load_intrinsics, project_image_route
from vlx_async.task_plan import (RoutePlan, path_length, remaining_length,
                                 validate_route_shape)

MOUNT = {
    'height_m': .50, 'pitch_rad': 0.,
    'intrinsics': {'width': 640, 'height': 480, 'fx': 605.3713989257812,
                   'fy': 605.1307983398438, 'ppx': 330.8877868652344,
                   'ppy': 257.2687683105469},
}


def pose(x=0., y=0., yaw=0., stamp=1_000_000_000, frame='odom'):
    return Pose(x, y, yaw, stamp, frame)


def observation(pose_value=None, waypoints=None, task=('boot', 'stream', 1, '走到椅子旁')):
    boot, stream, revision, instruction = task
    return Observation(stream, boot, revision, instruction, 5, 6, 1_000_000_000, 1_010_000_000,
                       waypoints or tuple((i*.25, 0., 0.) for i in range(1, 9)),
                       Context(pose=pose_value if pose_value is not None else pose()))


def route_plan(points, pose_value=None, task_key=None, confidence=.9, created_ns=1_010_000_000,
               request_id='req-1'):
    return RoutePlan(request_id, task_key or ('boot', 'stream', 1, '走到椅子旁'), tuple(points),
                     confidence, '向前走约两米', 'openai_responses', created_ns,
                     pose_value if pose_value is not None else pose())


class GroundProjectionTests(unittest.TestCase):
    def test_known_pixel_maps_to_expected_range(self):
        intrinsics = load_intrinsics(MOUNT)
        x, y = image_point_to_local(.5, .95, intrinsics)
        self.assertAlmostEqual(x, 1.52, places=1)
        self.assertLess(abs(y), .05)
        far, _ = image_point_to_local(.5, .80, intrinsics)
        self.assertGreater(far, x)

    def test_points_at_or_above_horizon_do_not_project(self):
        intrinsics = load_intrinsics(MOUNT)
        self.assertIsNone(image_point_to_local(.5, .40, intrinsics))
        self.assertIsNone(image_point_to_local(.5, .30, intrinsics))

    def test_route_starts_at_the_robot_and_is_clamped(self):
        intrinsics = load_intrinsics(MOUNT)
        route = project_image_route([(.5, .95), (.52, .80), (.55, .55)], intrinsics, max_range_m=3.)
        self.assertEqual(route[0][:2], (0., 0.))
        self.assertGreaterEqual(len(route), 3)
        self.assertTrue(all(math.isfinite(v) for point in route for v in point))
        self.assertLessEqual(max(math.hypot(x, y) for x, y, _ in route), 3.+1e-9)

    def test_route_entirely_above_horizon_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'plan_route_above_horizon'):
            project_image_route([(.5, .10), (.5, .20), (.5, .30)], load_intrinsics(MOUNT), 4.)

    def test_missing_intrinsics_is_an_explicit_error(self):
        with self.assertRaisesRegex(ValueError, 'camera_intrinsics_missing'):
            load_intrinsics({'height_m': .5})


class RouteProgressTests(unittest.TestCase):
    def test_remaining_length_consumes_the_route(self):
        path = ((0., 0., 0.), (2., 0., 0.), (4., 0., 0.))
        remaining, cross = remaining_length(path)
        self.assertAlmostEqual(remaining, 4.)
        self.assertAlmostEqual(cross, 0.)

    def test_moving_forward_shortens_the_remaining_route(self):
        from vlx_async.geometry import align
        path = ((0., 0., 0.), (2., 0., 0.), (4., 0., 0.))
        aligned = align(path, pose(stamp=1), pose(1., 0., 0., stamp=2))
        remaining, _ = remaining_length(aligned)
        self.assertAlmostEqual(remaining, 3.)

    def test_length_and_reverse_rules(self):
        config = TaskConfig()
        self.assertAlmostEqual(path_length(((0., 0., 0.), (3., 4., 0.))), 5.)
        self.assertIsNone(validate_route_shape(((0., 0., 0.), (2., .2, 0.)), config))
        # Behind the robot is allowed; the gate rotates in place first.
        self.assertIsNone(validate_route_shape(((0., 0., 0.), (-2., .1, 0.)), config))
        self.assertEqual(validate_route_shape(((0., 0., 0.), (.05, 0., 0.)), config),
                         'plan_route_too_short')


class OnePlanPerTaskTests(unittest.TestCase):
    def setUp(self):
        self.fusion = Fusion(FusionConfig(), TaskConfig())
        self.obs = observation()

    def test_task_without_a_plan_never_moves(self):
        decision = self.fusion.decide(self.obs, 1_020_000_000)
        self.assertEqual(decision['reason'], 'plan_pending')
        self.assertFalse(decision['task_execution_ready'])
        self.assertEqual(decision['selection'], 'hold')

    def test_plan_is_accepted_once_and_drives_the_path(self):
        plan = route_plan(((0., 0., 0.), (1.5, 0., 0.), (2.5, .3, 0.)))
        self.assertIsNone(self.fusion.accept(plan, self.obs, 1_020_000_000))
        again = self.fusion.accept(route_plan(((0., 0., 0.), (2., 0., 0.)), request_id='req-2'),
                                   self.obs, 1_030_000_000)
        self.assertEqual(again, 'plan_already_active')
        decision = self.fusion.decide(self.obs, 1_030_000_000)
        self.assertEqual(decision['reason'], 'task_ready', decision)
        self.assertTrue(decision['task_execution_ready'])
        self.assertEqual(len(decision['recommended']), 8)
        self.assertAlmostEqual(decision['remaining_route_m'], 1.5+math.hypot(1., .3), places=2)
        self.assertFalse(decision['task_completed'])

    def test_route_completion_stops_without_a_new_model_call(self):
        plan = route_plan(((0., 0., 0.), (.6, 0., 0.)))
        self.fusion.accept(plan, self.obs, 1_020_000_000)
        moved = observation(pose(1.0, 0., 0., stamp=2_000_000_000))
        decision = self.fusion.decide(moved, 2_000_000_000)
        self.assertEqual(decision['reason'], 'route_completed')
        self.assertFalse(decision['task_execution_ready'])
        self.assertTrue(decision['task_completed'])
        self.assertEqual(decision['recommended'], [])

    def test_new_task_key_drops_the_old_plan(self):
        self.fusion.accept(route_plan(((0., 0., 0.), (2., 0., 0.))), self.obs, 1_020_000_000)
        newer = observation(task=('boot', 'stream', 2, '走到门口'))
        decision = self.fusion.decide(newer, 1_030_000_000)
        self.assertEqual(decision['reason'], 'plan_pending')
        self.assertIsNone(self.fusion.plan)

    def test_weak_plans_rejected_and_reverse_plan_turns_in_place(self):
        weak = route_plan(((0., 0., 0.), (2., 0., 0.)), confidence=.2)
        self.assertEqual(self.fusion.accept(weak, self.obs, 1_020_000_000), 'low_plan_confidence')
        behind = route_plan(((0., 0., 0.), (-2., 0., 0.)), request_id='req-3')
        self.assertIsNone(self.fusion.accept(behind, self.obs, 1_030_000_000))
        decision = self.fusion.decide(self.obs, 1_030_000_000)
        self.assertTrue(decision['rotate_in_place'], decision)
        # The target is a half turn; the applied blend ramps toward it.
        self.assertGreater(abs(decision['blend_target_rad']), math.pi/2)
        self.assertLess(abs(decision['chosen_angle_rad']), abs(decision['blend_target_rad']))

    def test_pose_loss_blocks_without_dropping_the_plan(self):
        self.fusion.accept(route_plan(((0., 0., 0.), (2., 0., 0.))), self.obs, 1_020_000_000)
        blind = Observation('stream', 'boot', 1, '走到椅子旁', 5, 6, 1_000_000_000,
                            1_010_000_000, tuple((i*.25, 0., 0.) for i in range(1, 9)), Context(pose=None))
        decision = self.fusion.decide(blind, 1_030_000_000)
        self.assertEqual(decision['reason'], 'pose_unavailable')
        self.assertFalse(decision['task_execution_ready'])
        self.assertIsNotNone(self.fusion.plan)

    def test_odometry_frame_change_is_reported(self):
        self.fusion.accept(route_plan(((0., 0., 0.), (2., 0., 0.))), self.obs, 1_020_000_000)
        decision = self.fusion.decide(observation(pose(0., 0., 0., frame='reset')), 1_030_000_000)
        self.assertEqual(decision['reason'], 'odometry_frame_changed')

    def test_drift_off_the_planned_route_blocks(self):
        self.fusion.accept(route_plan(((0., 0., 0.), (2., 0., 0.))), self.obs, 1_020_000_000)
        drifted = observation(pose(0., 2.0, 0., stamp=2_000_000_000))
        decision = self.fusion.decide(drifted, 2_000_000_000)
        self.assertEqual(decision['reason'], 'route_drift')
        self.assertFalse(decision['task_execution_ready'])


if __name__ == '__main__':
    unittest.main()
