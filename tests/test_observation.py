import json
import threading
import time
import unittest
import urllib.error
import urllib.request
from vlx_async.observation import (ObservationServer, robot_summary, matching_experiment,
                                   get_local_json, load_archive)

class FakeObserver:
    def snapshot(self):
        return {'schema': 'vlx-observation-v1', 'execution_enabled': False,
                'observer_robot_commands_sent': 0, 'observer_api_calls': 0}

class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.server = ObservationServer(('127.0.0.1', 0), [], FakeObserver())
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = 'http://127.0.0.1:' + str(self.server.server_port)

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def test_reads_state_and_page_without_control_tokens(self):
        with urllib.request.urlopen(self.url + '/observe.json') as response:
            state = json.load(response)
            self.assertIn("frame-ancestors 'none'", response.headers['Content-Security-Policy'])
        self.assertFalse(state['execution_enabled'])
        self.assertEqual(state['observer_api_calls'], 0)
        with urllib.request.urlopen(self.url) as response:
            self.assertIn('当前 VLX 预测轨迹', response.read().decode())

    def test_post_is_never_forwarded(self):
        for path in ['/instruction', '/manual/action', '/observe.json']:
            request = urllib.request.Request(self.url + path, data=b'{}', method='POST')
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(request)
            self.assertEqual(caught.exception.code, 405)

    def test_unknown_routes_and_invalid_host_are_rejected(self):
        for path in ['/manual/state.json', '/state.json', '/../../config/current_api.json']:
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(self.url + path)
            self.assertEqual(caught.exception.code, 404)
        request = urllib.request.Request(self.url, headers={'Host': 'untrusted.example'})
        with self.assertRaises(urllib.error.HTTPError) as caught:
            urllib.request.urlopen(request)
        self.assertEqual(caught.exception.code, 403)

    def test_archive_traversal_rejected(self):
        for run in ['../config', 'shadow_../../config', '/etc/passwd', 'shadow_%2e%2e']:
            with self.assertRaises(ValueError):
                load_archive(run)

    def test_robot_allowlist_removes_control_secret(self):
        result = robot_summary({'token': 'do-not-publish', 'enabled': False,
                                'vlx': {'token': 'another-secret', 'robot': {'ready': True}}})
        self.assertNotIn('token', json.dumps(result))
        self.assertNotIn('secret', json.dumps(result))

    def test_network_reader_only_accepts_local_service(self):
        for url in ['https://example.com/state.json', 'file:///etc/passwd',
                    'http://127.0.0.1:8765/state.json?secret=x',
                    'http://user:password@127.0.0.1:8765/state.json']:
            with self.assertRaises(ValueError):
                get_local_json(url)

    def test_finished_stale_future_or_other_task_never_becomes_live(self):
        now = time.monotonic_ns()
        preview = {'host_boot_id': 'boot', 'stream_id': 'stream', 'requested_instruction': 'chair',
                   'requested_instruction_revision': 3, 'record': {'instruction_revision': 3}}
        live = {'host_boot_id': 'boot', 'stream_id': 'stream', 'instruction': 'chair',
                'instruction_revision': 3, 'status': 'shadow', 'timestamp_ns': now}
        self.assertIsNone(matching_experiment(live, preview, now))
        for patch in [{'status': 'finished'}, {'timestamp_ns': now-3_000_000_000},
                      {'timestamp_ns': now+1}, {'stream_id': 'other'},
                      {'instruction_revision': 2}, {'host_boot_id': 'rebooted'}, {'instruction': 'person'}]:
            self.assertIsNotNone(matching_experiment({**live, **patch}, preview, now), patch)

if __name__ == '__main__':
    unittest.main()
