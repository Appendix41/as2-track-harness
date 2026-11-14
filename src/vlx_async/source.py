"""GET-only adapter to the existing preview; no robot SDK or control writes."""
from collections import deque
from dataclasses import replace
import json
import math
from pathlib import Path
import threading
import time
import urllib.request
from urllib.parse import urlsplit
from .contracts import Context, LocalCandidate, Observation, Pose, number, points
from .geometry import wrap


class ReadOnlySource:
    def __init__(self, base_url="http://127.0.0.1:8765", timeout=.4):
        parsed = urlsplit(base_url)
        if parsed.scheme != "http" or parsed.hostname not in {"127.0.0.1", "localhost"} or parsed.path not in ("", "/"):
            raise ValueError("source_must_be_local_preview")
        if parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ValueError("invalid_source_URL")
        self.base_url, self.timeout = base_url.rstrip("/"), timeout

    def get(self, path):
        if path not in {"/control.json", "/state.json", "/camera.json"}:
            raise ValueError("read_only_path_not_allowed")
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, *args, **kwargs):
                return None
        req = urllib.request.Request(self.base_url+path, method="GET")
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect)
        with opener.open(req, timeout=self.timeout) as response:
            raw = response.read(2_000_001)
        if len(raw)>2_000_000:
            raise ValueError("source_payload_too_large")
        result = json.loads(raw)
        if not isinstance(result, dict):
            raise ValueError("invalid_source_state")
        return result


def visual_observation(state, boot_id, now_ns, max_age_s=.5):
    """Decode independent camera evidence without inventing a VLX prediction.

    The embedded inference record retains its own timestamps. This observation
    is for semantic requests and tracking, never a replacement control packet.
    """
    if state.get('schema') != 'vlx-camera-v1':
        legacy = dict(state, schema='vlx-control-v1')
        reason = FastGate(boot_id, max_camera_age_s=max_age_s,
                          max_prediction_age_s=max_age_s).check(legacy, now_ns)
        if reason:
            raise ValueError(reason)
        return Observation.from_state(legacy)
    frame, record = state.get('camera_frame') or {}, state.get('inference_record') or {}
    if not isinstance(frame, dict) or not isinstance(record, dict):
        raise ValueError('invalid_visual_metadata')
    if (type(frame.get('capture_generation')) is not int
            or frame['capture_generation'] != record.get('capture_generation')):
        raise ValueError('camera_generation_mismatch')
    stamp, index = frame.get('host_monotonic_ns'), frame.get('index')
    if (state.get('host_boot_id') != boot_id or not isinstance(state.get('stream_id'), str)
            or not state['stream_id']):
        raise ValueError('different_host_or_boot')
    if state.get('status') != 'running':
        raise ValueError('source_not_running')
    if (type(stamp) is not int or type(index) is not int or index < 0
            or not 0 <= (now_ns-stamp)/1e9 <= max_age_s):
        raise ValueError('camera_frame_stale')
    instruction, revision = state.get('requested_instruction'), state.get('requested_instruction_revision')
    if (not isinstance(instruction, str) or not instruction.strip() or type(revision) is not int or revision < 0
            or record.get('instruction') != instruction or record.get('instruction_revision') != revision):
        raise ValueError('instruction_pending')
    if (not state.get('image_base64') or frame.get('color_order') != 'RGB'
            or record.get('valid_history_frames') != 31):
        raise ValueError('visual_input_unavailable')
    prediction_ns, prediction_index = record.get('prediction_available_monotonic_ns'), record.get('prediction_index')
    if (type(prediction_ns) is not int or not 0 < prediction_ns <= now_ns
            or type(prediction_index) is not int or prediction_index < 0):
        raise ValueError('invalid_prediction_metadata')
    return Observation(state['stream_id'], boot_id, revision, instruction,
        prediction_index, index, stamp, prediction_ns, points(record['waypoints'], count=8),
        image_base64=state['image_base64'])


class FastGate:
    def __init__(self, boot_id, max_camera_age_s=.5, max_prediction_age_s=.35):
        self.boot_id, self.last_key, self.last_order = boot_id, None, None
        self.camera_age, self.prediction_age = max_camera_age_s, max_prediction_age_s

    def check(self, state, now_ns):
        if not isinstance(state, dict) or state.get("schema") != "vlx-control-v1":
            return "invalid_envelope"
        if state.get("host_boot_id") != self.boot_id:
            return "different_host_or_boot"
        if not isinstance(state.get("stream_id"), str) or not state["stream_id"]:
            return "missing_stream_id"
        if state.get("status") != "running":
            return "source_not_running"
        r = state.get("record")
        if not isinstance(r, dict):
            return "missing_record"
        revision = state.get("requested_instruction_revision")
        if type(revision) is not int or revision < 0 or r.get("instruction_revision") != revision:
            return "instruction_pending"
        if not isinstance(r.get("instruction"), str) or not r["instruction"].strip() or r["instruction"] != state.get("requested_instruction"):
            return "instruction_pending"
        order = tuple(r.get(k) for k in ("prediction_index", "camera_frame_index",
                                         "camera_received_monotonic_ns", "prediction_available_monotonic_ns"))
        if any(type(v) is not int or v < 0 for v in order):
            return "invalid_timestamps"
        _, _, camera_ns, prediction_ns = order
        if not 0 < camera_ns <= prediction_ns <= now_ns:
            return "invalid_clock_order"
        if (now_ns-camera_ns)/1e9 > self.camera_age:
            return "camera_frame_stale"
        if (now_ns-prediction_ns)/1e9 > self.prediction_age:
            return "prediction_stale"
        key = (state["stream_id"], revision, r["instruction"])
        if key == self.last_key and self.last_order and order != self.last_order and any(a<=b for a,b in zip(order,self.last_order)):
            return "prediction_order_regressed"
        self.last_key, self.last_order = key, order
        if type(r.get("valid_history_frames")) is not int or r["valid_history_frames"] != 31:
            return "history_warming_up"
        try:
            Observation.from_state(state)
        except (ValueError, TypeError, KeyError):
            return "invalid_waypoints"
        return None


class PoseBuffer:
    """Optional local odometry interpolation; never silently assumes stationary."""
    def __init__(self, capacity=512):
        self.samples = deque(maxlen=capacity)

    def add(self, pose):
        if self.samples and pose.frame_id != self.samples[-1].frame_id:
            self.samples.clear()
        if not self.samples or pose.timestamp_ns > self.samples[-1].timestamp_ns:
            self.samples.append(pose)

    def at(self, timestamp_ns, max_gap_ns=200_000_000):
        samples = list(self.samples)
        for a, b in zip(samples, samples[1:]):
            if a.timestamp_ns <= timestamp_ns <= b.timestamp_ns and b.timestamp_ns-a.timestamp_ns <= max_gap_ns:
                fraction = (timestamp_ns-a.timestamp_ns)/(b.timestamp_ns-a.timestamp_ns)
                return Pose(a.x+fraction*(b.x-a.x), a.y+fraction*(b.y-a.y),
                            wrap(a.yaw+fraction*wrap(b.yaw-a.yaw)), timestamp_ns, a.frame_id)
        return None


def context_from_file(path, obs):
    """Read a separately calibrated perception sidecar; no invented live geometry."""
    if path is None:
        return Context()
    data = json.loads(Path(path).read_text())
    if (data.get("host_boot_id"), data.get("stream_id"), data.get("camera_frame_index")) != (obs.boot_id, obs.stream_id, obs.camera_index):
        raise ValueError("context_observation_mismatch")
    if data.get("frame") != "vlx_local_m":
        raise ValueError("context_coordinate_mismatch")
    obstacles = data.get("obstacles", [])
    if not isinstance(obstacles, list) or len(obstacles)>10000:
        raise ValueError("invalid_obstacles")
    for circle in obstacles:
        if not isinstance(circle, list) or len(circle)!=3 or not all(number(v) for v in circle) or circle[2]<0:
            raise ValueError("invalid_obstacle")
    target = data.get("target_local")
    if target is not None and (not isinstance(target, list) or len(target)!=2 or not all(number(v) for v in target)):
        raise ValueError("invalid_target_location")
    for name in ("scene_timestamp_ns", "target_timestamp_ns"):
        if type(data.get(name, 0)) is not int or data.get(name, 0)<0:
            raise ValueError("invalid_context_timestamp")
    if any(type(data.get(k, False)) is not bool for k in ('geometry_verified', 'candidates_complete')):
        raise ValueError("invalid_geometry_flag")
    raw_candidates = data.get("local_candidates", [])
    if not isinstance(raw_candidates, list) or len(raw_candidates) > 1000:
        raise ValueError("invalid_local_candidates")
    candidates = []
    for item in raw_candidates:
        if not isinstance(item, dict):
            raise ValueError("invalid_local_candidate")
        local_xy = item.get("local_xy")
        if local_xy is not None and (not isinstance(local_xy, list) or len(local_xy) != 2
                                     or not all(number(v) for v in local_xy)):
            raise ValueError("invalid_local_candidate_local_xy")
        attrs = item.get("attributes", [])
        if not isinstance(attrs, list) or any(not isinstance(v, str) for v in attrs):
            raise ValueError("invalid_local_candidate_attributes")
        candidates.append(LocalCandidate(item["candidate_id"], item["kind"], tuple(attrs),
                                          tuple(item["bbox"]), item["timestamp_ns"],
                                          item.get("range_m"), item.get("track_age_s", 0.0),
                                          tuple(local_xy) if local_xy is not None else None,
                                          item.get('semantic_verified', False), item.get('verifier_source', ''),
                                          item.get('reference_request_id'), item.get('identity_verified', False)))
    return Context(Pose.parse(data.get("pose")), tuple(tuple(c) for c in obstacles),
                   data.get("geometry_verified", False), data.get("scene_timestamp_ns", 0),
                   data.get("target_id"), data.get("target_timestamp_ns", 0),
                   tuple(target) if target else None, tuple(candidates), data.get('candidates_complete', False))


class FastReader:
    def __init__(self, source, hz=10.):
        self.source, self.hz = source, hz
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.state, self.error = None, None
        self.thread = threading.Thread(target=self._run, name="preview-read-only", daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            start = time.monotonic()
            try:
                state = self.source.get("/control.json")
                with self.lock:
                    self.state, self.error = state, None
            except Exception as exc:
                with self.lock:
                    self.state, self.error = None, type(exc).__name__
            self.stop.wait(max(0., 1/self.hz-(time.monotonic()-start)))

    def snapshot(self):
        with self.lock:
            return self.state, self.error

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.)
