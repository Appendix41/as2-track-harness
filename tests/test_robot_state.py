"""The only survivor of the depth removal: read-only telemetry and pose history."""
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from vlx_async.robot_state import RobotState, fresh_telemetry, boot_id


def snapshot(now_ns, host_boot=None, status='running'):
    return dict(status=status, host_boot_id=host_boot or boot_id(), timestamp_ns=now_ns,
                sport=dict(received_monotonic_ns=now_ns, mode=3, error_code=0,
                           position=[.1, 0., 0.], rpy=[0., 0., .01], velocity=[0., 0., 0.]),
                low=dict(received_monotonic_ns=now_ns, mode_machine=1),
                remote_axes=dict(lx=0., ly=0., rx=0., ry=0., keys=0),
                pose_history=[dict(x=now_ns, y=0., yaw=0., timestamp_ns=now_ns, frame_id='direct'),
                              dict(x=now_ns+100_000_000, y=0., yaw=0., timestamp_ns=now_ns+100_000_000,
                                   frame_id='direct')])


class FreshnessTests(unittest.TestCase):
    def test_only_fresh_running_same_boot_counts(self):
        now = time.monotonic_ns()
        self.assertTrue(fresh_telemetry(snapshot(now), boot_id(), now))
        self.assertTrue(fresh_telemetry(snapshot(now - 249_000_000), boot_id(), now))
        self.assertFalse(fresh_telemetry(snapshot(now - 250_000_000), boot_id(), now))
        self.assertFalse(fresh_telemetry(snapshot(now, host_boot='other'), boot_id(), now))
        self.assertFalse(fresh_telemetry(snapshot(now, status='finished'), boot_id(), now))
        self.assertFalse(fresh_telemetry(dict(snapshot(now), timestamp_ns='now'), boot_id(), now))
        self.assertFalse(fresh_telemetry({}, boot_id(), now))


class RobotStateTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.path = Path(self.folder.name)/'telemetry.json'
        self.path.write_text(json.dumps(snapshot(time.monotonic_ns())))

    def tearDown(self):
        self.folder.cleanup()

    def test_telemetry_pose_history_and_interpolation(self):
        robot = RobotState(self.path, 'http://127.0.0.1:1/manual/state.json')
        try:
            deadline = time.monotonic()+2.
            while robot.telemetry().get('sport') is None and time.monotonic() < deadline:
                time.sleep(.01)
            telemetry = robot.telemetry()
            self.assertEqual(telemetry['sport']['mode'], 3)
            self.assertEqual(telemetry['low']['mode_machine'], 1)
            self.assertIsNone(telemetry['error'])
            # No owner service: unknown, never a silent "safe".
            self.assertIsNone(telemetry['original_manual_enabled'])
            history = robot.pose_history()
            self.assertEqual(len(history), 2)
            middle = robot.pose_at(history[0]['timestamp_ns']+50_000_000)
            self.assertIsNotNone(middle)
            self.assertIsNotNone(robot.latest_pose())
        finally:
            robot.close()

    def test_stale_or_missing_snapshot_reports_error_and_never_a_pose(self):
        self.path.write_text(json.dumps(snapshot(time.monotonic_ns()-1_000_000_000)))
        robot = RobotState(self.path, 'http://127.0.0.1:1/manual/state.json')
        try:
            deadline = time.monotonic()+2.
            while robot.telemetry().get('error') != 'ValueError' and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertEqual(robot.telemetry()['error'], 'ValueError')
            self.assertEqual(robot.telemetry().get('sport'), None)
        finally:
            robot.close()
        self.path.unlink()
        robot = RobotState(self.path, 'http://127.0.0.1:1/manual/state.json')
        try:
            deadline = time.monotonic()+2.
            while robot.telemetry().get('error') is None and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertIsNotNone(robot.telemetry()['error'])
            self.assertIsNone(robot.pose_at(time.monotonic_ns()))
        finally:
            robot.close()


if __name__ == '__main__':
    unittest.main()
