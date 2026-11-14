#!/usr/bin/env python3
"""Live RGB/text -> VLX-Go local waypoint preview. No robot SDK or commands."""
from __future__ import annotations

import argparse
import base64
from collections import deque
from datetime import datetime
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import logging
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
import traceback
import uuid

from vlx_live_camera import CameraProcess, LiveHistory

ROOT = Path(__file__).resolve().parents[1]


def atomic_bytes(path, payload):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_bytes(payload)
    temporary.replace(path)


def atomic_json(path, data):
    atomic_bytes(path, (json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode())


class Preview:
    def __init__(self, port, instruction):
        self.lock = threading.Lock()
        self.camera = None
        self.state = {"status": "starting", "message": "正在查找 RGB 相机", "instruction": instruction,
                      "initialization_stage": "detecting_camera", "model_loaded": False,
                      "instruction_revision": 0, "requested_instruction": instruction,
                      "requested_instruction_revision": 0,
                      "stream_id": str(uuid.uuid4()),
                      "host_boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip()}
        preview = self
        page = (ROOT / "src/vlx_live_preview.html").read_bytes()

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path == "/":
                    body, content_type = page, "text/html; charset=utf-8"
                elif path == "/state.json":
                    with preview.lock:
                        body = json.dumps(preview.state, ensure_ascii=False, allow_nan=False).encode()
                    content_type = "application/json; charset=utf-8"
                elif path == "/control.json":
                    body = json.dumps(preview.control_state(), ensure_ascii=False, allow_nan=False).encode()
                    content_type = "application/json; charset=utf-8"
                elif path == "/camera.json":
                    body = json.dumps(preview.camera_state(), ensure_ascii=False, allow_nan=False).encode()
                    content_type = "application/json; charset=utf-8"
                else:
                    self.send_error(404)
                    return
                self.send_body(200, body, content_type)

            def do_POST(self):
                if self.path != "/instruction":
                    self.send_error(404)
                    return
                try:
                    if self.headers.get_content_type() != "application/json":
                        raise ValueError("请使用 JSON 提交文本指令")
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= 4096:
                        raise ValueError("指令请求大小无效")
                    data = json.loads(self.rfile.read(length))
                    if not isinstance(data, dict):
                        raise ValueError("请提交 instruction 文本字段")
                    result = preview.request_instruction(data.get("instruction"))
                except (ValueError, UnicodeDecodeError) as exc:
                    self.send_body(400, json.dumps({"error": str(exc)}, ensure_ascii=False).encode(),
                                   "application/json; charset=utf-8")
                    return
                self.send_body(200, json.dumps(result, ensure_ascii=False).encode(),
                               "application/json; charset=utf-8")

            def send_body(self, status, body, content_type):
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.server.daemon_threads = True
        self.url = f"http://127.0.0.1:{self.server.server_port}/"
        self.thread = threading.Thread(target=self.server.serve_forever, name="waypoint-preview", daemon=True)
        self.thread.start()

    def update(self, **values):
        with self.lock:
            self.state.update(values)

    def request_instruction(self, instruction):
        if not isinstance(instruction, str) or not instruction.strip():
            raise ValueError("文本指令不能为空")
        instruction = instruction.strip()
        if len(instruction) > 512:
            raise ValueError("文本指令最多 512 个字符，请使用简短指令")
        with self.lock:
            if self.state["status"] in {"error", "stopped"}:
                raise ValueError("预览已停止，无法应用指令")
            if instruction != self.state["requested_instruction"]:
                self.state["requested_instruction"] = instruction
                self.state["requested_instruction_revision"] += 1
            return {"instruction": instruction, "revision": self.state["requested_instruction_revision"]}

    def requested_instruction(self):
        with self.lock:
            return self.state["requested_instruction"], self.state["requested_instruction_revision"]

    def camera_unavailable(self, camera):
        with self.lock:
            if self.state["status"] != "camera_reconnecting":
                # An interrupted camera is a new observation stream even if a
                # controller misses the short unavailable window.
                self.state["stream_id"] = str(uuid.uuid4())
            self.state.update(status="camera_reconnecting", initialization_stage="camera_reconnecting",
                              message="相机已断开或没有新图像，正在自动查找并重新初始化；恢复后请重新启动 VLX。",
                              record=None, image_base64=None, frame_received_unix_ms=None,
                              camera=camera.latest.stats(), camera_info=camera.info,
                              camera_recovery=camera.recovery_stats())

    def camera_status(self, info):
        with self.lock:
            self.state.update(initialization_stage="initializing_camera", camera_info=dict(info),
                              message=f"已找到 RGB 设备 {info['device']}，正在初始化并等待首帧图像")

    def camera_state(self):
        """Latest captured RGB, with its own stamp; never used as a prediction."""
        from PIL import Image
        with self.lock:
            camera = self.camera
            state = {key: self.state.get(key) for key in (
                "status", "stream_id", "host_boot_id", "requested_instruction",
                "requested_instruction_revision")}
            prediction = dict(self.state.get("record") or {})
        result = dict(state, schema="vlx-camera-v1", camera_frame=None,
                      inference_record=prediction, image_base64=None,
                      timestamp_semantics="original_host_receipt_not_exposure")
        if camera is None:
            return result
        mailbox = camera.latest
        with mailbox.condition:
            frame, error = mailbox.frame, mailbox.error
        if frame is None or error:
            result['status'] = 'camera_unavailable'
            return result
        if frame.metadata.get('capture_generation') != prediction.get('capture_generation'):
            result['status'] = 'camera_generation_pending'
            return result
        encoded = io.BytesIO()
        Image.frombytes('RGB', (frame.metadata['width'], frame.metadata['height']),
                        frame.rgb_bytes).save(encoded, format='JPEG', quality=85)
        # Keep the actual image timestamp and ID, independently of inference.
        result.update(camera_frame=dict(frame.metadata),
                      image_base64=base64.b64encode(encoded.getvalue()).decode())
        with self.lock:
            if any(self.state.get(key) != state[key] for key in state):
                result.update(status='camera_snapshot_changed',image_base64=None)
        return result

    def control_state(self):
        """Small, read-only live envelope for a separate, opt-in AS2 bridge."""
        fields = ("prediction_index", "camera_frame_index", "waypoints", "instruction",
                  "instruction_revision", "valid_history_frames", "camera_received_monotonic_ns",
                  "prediction_available_monotonic_ns")
        with self.lock:
            state = {key: self.state.get(key) for key in ("status", "stream_id", "host_boot_id",
                     "requested_instruction", "requested_instruction_revision", "message",
                     "initialization_stage", "model_loaded", "camera_info", "camera_recovery")}
            record = self.state.get("record")
            record = record if isinstance(record, dict) else {}
            state["record"] = {key: record.get(key) for key in fields}
        return {"schema": "vlx-control-v1", **state}

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)


def draw_snapshot(rgb_image, record, output):
    """Save a standalone image of the same two views as the live page."""
    from PIL import Image, ImageDraw, ImageFont
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(font_path, 17)
    small = ImageFont.truetype(font_path, 14)
    canvas = Image.new("RGB", (1160, 625), "#101722")
    draw = ImageDraw.Draw(canvas)
    draw.text((20, 17), "VLX-Go live camera and local waypoints | Preview only", fill="#e5edf5", font=font)
    canvas.paste(rgb_image.resize((640, 480)), (20, 64))
    draw.text((20, 556), f"Camera frame {record['camera_frame_index']} | prediction {record['prediction_index']}", fill="#b7c7d9", font=small)
    draw.text((20, 583), f"{record['instruction']}", fill="#b7c7d9", font=small)
    points = record["waypoints"]
    ymax = max(.5, max(abs(p[1]) for p in points) * 1.25)
    xmin = min(-.15, min(p[0] for p in points) - .1)
    xmax = max(1., max(p[0] for p in points) * 1.25)
    scale = min(420 / (2 * ymax), 410 / (xmax - xmin))
    cx, cy = 906, 504 + xmin * scale
    draw.rectangle((684, 64, 1138, 544), outline="#304054")
    for offset in (-1, -.5, 0, .5, 1):
        x = cx - offset * scale
        if 684 <= x <= 1138:
            draw.line((x, 95, x, 528), fill="#243247")
    draw.line((cx, 90, cx, 529), fill="#8291a7")
    draw.line((690, cy, 1132, cy), fill="#8291a7")
    xy = [(cx - p[1] * scale, cy - p[0] * scale) for p in points]
    draw.line([(cx, cy), *xy], fill="#59d5cf", width=3)
    for index, (x, y) in enumerate(xy):
        draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill="#59d5cf")
        draw.text((x + 6, y - 11), str(index), fill="#e5edf5", font=small)
    draw.text((700, 76), "Forward x (m); left y (m)", fill="#b7c7d9", font=small)
    draw.text((700, 556), f"Processing {record['compute_ms']:.1f} ms | receipt to prediction {record['receipt_to_prediction_ms']:.1f} ms", fill="#b7c7d9", font=small)
    draw.text((700, 583), f"History {record['valid_history_frames']}/31 | span {record['history_span_ms']/1000:.2f} s", fill="#b7c7d9", font=small)
    payload = io.BytesIO()
    canvas.save(payload, format="PNG")
    atomic_bytes(output / "preview.png", payload.getvalue())


class SnapshotWriter:
    """Render PNG reports off the inference thread, retaining one pending frame.

    Images and records are unique per prediction and must not be mutated after
    submission. The worker never reads the runtime, its history or live state.
    """
    def __init__(self, output, render=draw_snapshot):
        self.output, self.render = output, render
        self.condition = threading.Condition()
        self.pending = None
        self.closed = False
        self.error = None
        self.written = self.replaced = 0
        self.last_prediction_index = None
        self.last_render_ms = None
        self.thread = threading.Thread(target=self._run, name="vlx-snapshot-writer", daemon=True)
        self.thread.start()

    def check(self):
        with self.condition:
            if self.error:
                raise RuntimeError(self.error)

    def submit(self, image, record):
        with self.condition:
            self.check()
            if self.closed:
                raise RuntimeError("Snapshot writer is closed")
            self.replaced += int(self.pending is not None)
            self.pending = image, record
            self.condition.notify()

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.pending is not None or self.closed)
                if self.pending is None:
                    return
                image, record = self.pending
                self.pending = None
            started = time.monotonic()
            try:
                self.render(image, record, self.output)
            except Exception as exc:
                with self.condition:
                    self.error = f"snapshot_writer_failed:{type(exc).__name__}:{exc}"
                    self.pending = None
                return
            with self.condition:
                self.written += 1
                self.last_prediction_index = record["prediction_index"]
                self.last_render_ms = (time.monotonic() - started) * 1000

    def stats(self):
        with self.condition:
            return {"written": self.written, "replaced_pending": self.replaced,
                    "pending_frames": int(self.pending is not None), "pending_capacity": 1,
                    "last_prediction_index": self.last_prediction_index,
                    "last_render_ms": self.last_render_ms, "error": self.error}

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify()
        # Drain the latest pending snapshot before shutdown reporting. No second
        # writer may race this worker for preview.png or its temporary file.
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            with self.condition:
                self.error = self.error or "snapshot_writer_shutdown_timeout"
        self.check()


def run(args, output, stop):
    preview = None
    camera = None
    runtime = None
    snapshots = None
    hooks = []
    records = deque(maxlen=600)
    history = LiveHistory(reset_gap_ms=args.history_reset_gap_ms)
    totals = {"predictions": 0, "full_history_predictions": 0, "stale_frames_skipped": 0,
              "graph_capture_predictions": [], "all_outputs_finite": True, "history_verified": True}
    failure = None
    last_image = None
    last_record = None
    run_started = None
    manifest = {"created": datetime.now().astimezone().isoformat(),
                "args": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                "scope": "Real RGB and editable text to local waypoints; no DDS, robot SDK or motor commands",
                "instruction_change_semantics": "Apply a snapshot of the latest requested text at each inference boundary; retain instruction-independent visual history. Each prediction records the text and revision actually submitted.",
                "timestamp_semantics": "Host receipt after V4L2 read to synchronized prediction; not exposure-to-actuator latency",
                "history_semantics": "Only successfully inferred frames, in receipt order; actual timestamps retained. No duplicated frames or nominal 10 Hz resampling.",
                "queue_policy": "Continuously drain camera pipe into one replaceable latest-frame slot",
                "snapshot_policy": "PNG report rendering uses a separate worker with one replaceable pending frame; it never blocks inference on rendering",
                "camera_recovery_policy": "Reopen/re-enumerate RGB capture after EOF or stalls, retain model weights, invalidate control stream and clear visual history; never rearm motion",
                "source_sha256": {p: hashlib.sha256((ROOT / "src" / p).read_bytes()).hexdigest()
                                  for p in ("preview_vlx_live.py", "vlx_live_camera.py", "vlx_acceleration.py", "vlx_live_preview.html")}}
    atomic_json(output / "manifest.json", manifest)

    def summary(status):
        from replay_vlx_offline import timing_stats
        warm = [r for r in records if r["prediction_index"] >= args.warmup_frames and not r["graph_capture_this_frame"]]
        full = [r for r in warm if r["valid_history_frames"] == 31]

        def measured(values):
            stats = timing_stats(values, np)
            if stats:
                stats.pop("serial_hz", None)
            return stats

        def output_hz(values):
            if len(values) < 2:
                return None
            elapsed = values[-1]["prediction_available_monotonic_ns"] - values[0]["prediction_available_monotonic_ns"]
            return (len(values) - 1) * 1e9 / elapsed

        return {"status": status, "passed": failure is None and totals["predictions"] > 0,
                "failure": failure, "instruction": last_record["instruction"] if last_record else args.instruction,
                "instruction_revision": last_record["instruction_revision"] if last_record else 0,
                "requested_instruction": preview.requested_instruction()[0] if preview else args.instruction,
                "acceleration": args.acceleration,
                **totals, "history_resets": history.resets,
                "elapsed_stream_seconds": time.monotonic() - run_started if run_started else 0,
                "timing_window_predictions": len(records), "timing_window_limit": records.maxlen,
                "warm_processing": measured([r["compute_ms"] for r in warm]),
                "warm_receipt_to_prediction": measured([r["receipt_to_prediction_ms"] for r in warm]),
                "full_history_processing": measured([r["compute_ms"] for r in full]),
                "full_history_receipt_to_prediction": measured([r["receipt_to_prediction_ms"] for r in full]),
                "warm_frame_age_at_start": measured([r["frame_age_at_start_ms"] for r in warm]),
                "history_step_gaps": measured([r["history_step_gap_ms"] for r in warm if r["history_step_gap_ms"] is not None]),
                "full_history_spans": measured([r["history_span_ms"] for r in full]),
                "warm_output_hz": output_hz(warm), "full_history_output_hz": output_hz(full),
                "recent_prediction_hz": (records[-1]["recent_prediction_hz"] if records else None),
                "camera": {"info": camera.info, **camera.latest.stats(),
                           "recovery": camera.recovery_stats()} if camera else None,
                "snapshot_writer": snapshots.stats() if snapshots else None,
                "cuda_graphs": graph_stats(runtime) if runtime else {},
                "memory": memory_snapshot(torch) if runtime else None,
                "output_shape": [8, 3], "robot_commands_sent": 0,
                "limitations": ["No navigation, following or AS2 control performance has been established.",
                                "31 inferred past frames can span longer than nominal 3.1 seconds at 10 Hz.",
                                "V4L2 host timestamps do not establish exposure time or intrinsic driver latency.",
                                "Adapter actions are raw values; AS2 scaling and signs are unverified.",
                                "Percentiles summarize at most the most recent 600 predictions, excluding warmup and capture."]}

    try:
        preview = Preview(args.port, args.instruction)
        logging.info("Live preview: %s", preview.url)
        logging.info("Results: %s", output)
        # Validate a real RGB frame before allocating/loading the model. Missing
        # USB devices remain retryable and the web page stays responsive.
        preview.update(status="starting", initialization_stage="detecting_camera", message="正在查找 RGB 相机")
        camera = CameraProcess(ROOT / ".venv/bin/python", args.device, args.width, args.height, args.fps, output,
                               on_status=preview.camera_status)
        preview.camera = camera
        camera_started = time.monotonic()
        while not stop.is_set():
            frame = camera.take(stop)
            if frame is not None:
                break
            preview.camera_unavailable(camera)
            if args.duration and time.monotonic() - camera_started >= args.duration:
                return 0
        if stop.is_set():
            return 0
        manifest["camera_pid"] = camera.process.pid
        manifest["camera"] = camera.info
        atomic_json(output / "manifest.json", manifest)
        preview.update(status="loading_model", initialization_stage="loading_model", camera_info=camera.info,
                       camera=camera.latest.stats(), message="RGB 相机初始化成功，正在加载本地模型")
        for name in ("HF_HUB_OFFLINE", "TRANSFORMERS_OFFLINE", "HF_LOCAL_FILES_ONLY", "HF_HUB_DISABLE_TELEMETRY"):
            os.environ[name] = "1"
        os.environ.update({"HF_MODULES_CACHE": str(ROOT / ".cache/vlxgo_modules"),
                           "HF_MODEL_DIR": str(args.resources / "vlxgo"),
                           "DINOV3_MODEL_PATH": str(args.resources / "dinov3-vits16-pretrain-lvd1689m"),
                           "SIGLIP_MODEL_PATH": str(args.resources / "siglip-so400m-patch14-384"),
                           "VLXGO_DEVICE": "cuda:0", "VLXGO_ATTN_IMPL": "sdpa",
                           "TOKENIZERS_PARALLELISM": "false"})
        sys.path.insert(0, str(ROOT / "vendor/VLX-Go"))
        import numpy as np
        from PIL import Image
        import torch
        from vlxgo.attention import configure_torch_inference_backends
        from vlxgo.runtime import EngineRuntime
        from vlx_acceleration import apply_acceleration, graph_stats
        from replay_vlx_offline import memory_snapshot
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable; live inference requires the AGX GPU")
        torch.set_num_threads(args.threads)
        torch.manual_seed(0)
        configure_torch_inference_backends()
        torch.cuda.reset_peak_memory_stats()
        model_started = time.monotonic()
        runtime = EngineRuntime(str(args.resources / "vlxgo"))
        apply_acceleration(runtime, args.acceleration)
        torch.cuda.synchronize()
        manifest.update({"model_load_seconds": time.monotonic() - model_started,
                         "gpu": torch.cuda.get_device_name(0), "torch": torch.__version__,
                         "cuda_runtime": torch.version.cuda,
                         "preview_url": preview.url, "pid": os.getpid()})
        current = {}

        def planner_hook(module, positional, kwargs):
            mask = kwargs["history_valid_mask"]
            current["valid_history_frames"] = int(mask.sum().item())
            current["mask_shape"] = list(mask.shape)

        hooks.append(runtime._agent.planner_model.register_forward_pre_hook(planner_hook, with_kwargs=True))
        config = runtime._agent.planner_model.config
        if config.history != 31 or config.n_waypoints != 8:
            raise ValueError("This preview expects the verified VLX-Go 31-frame, 8-waypoint model")
        if stop.is_set():
            return 0
        snapshots = SnapshotWriter(output)
        manifest["camera_pid"] = camera.process.pid
        atomic_json(output / "manifest.json", manifest)
        preview.update(status="warming_up", initialization_stage="warming_up", model_loaded=True,
                       message="模型已加载，正在进行首次推理并预热图像历史")
        run_started = time.monotonic()
        next_due = run_started
        next_report = run_started
        completion_times = deque(maxlen=60)
        if args.save_images:
            (output / "rgb").mkdir()
        with (output / "predictions.jsonl").open("x") as stream:
            while not stop.is_set() and (not args.duration or time.monotonic() - run_started < args.duration):
                if args.max_frames and totals["predictions"] >= args.max_frames:
                    break
                if stop.wait(max(0, next_due - time.monotonic())):
                    break
                snapshots.check()
                frame = camera.take(stop)
                if frame is None:
                    if stop.is_set():
                        break
                    preview.camera_unavailable(camera)
                    # Preserve the latest text for explicit initialization even
                    # if a later fault terminates this preview process.
                    with preview.lock:
                        atomic_json(output / "last_preview_state.json", preview.state)
                    continue
                if stop.is_set():
                    break
                started_ns = time.monotonic_ns()
                age_ms = (started_ns - frame.timestamp_ns) / 1e6
                if age_ms > args.max_frame_age_ms:
                    totals["stale_frames_skipped"] += 1
                    continue
                past, reset = history.prepare(frame.metadata)
                if reset:
                    runtime.clear_cache("live-preview")
                    completion_times.clear()
                    logging.info("Reset temporal history after a camera restart or receipt gap > %.0f ms", args.history_reset_gap_ms)
                rgb = np.frombuffer(frame.rgb_bytes, dtype=np.uint8).reshape(args.height, args.width, 3)
                instruction, instruction_revision = preview.requested_instruction()
                captures_before = sum(s["captures"] for s in graph_stats(runtime).values())
                result = runtime.infer("live-preview", rgb, instruction)
                torch.cuda.synchronize()
                available_ns = time.monotonic_ns()
                waypoints = np.asarray(result["waypoints"])
                if waypoints.shape != (8, 3) or not np.isfinite(waypoints).all():
                    raise RuntimeError("Waypoint output must be a finite 8x3 array")
                if current["valid_history_frames"] != len(past) or current["mask_shape"] != [1, 31]:
                    raise RuntimeError("Model history mask does not match submitted past frames")
                if result["history_length"] != min(len(past) + 1, 31):
                    raise RuntimeError("Runtime history cache length is incorrect")
                if not np.allclose(result["action"], waypoints[1] / .1):
                    raise RuntimeError("Unexpected raw waypoint adapter result")
                history.commit(frame.metadata)
                completion_times.append(available_ns)
                prediction_hz = ((len(completion_times) - 1) * 1e9 / (completion_times[-1] - completion_times[0])
                                 if len(completion_times) > 1 else None)
                capture = sum(s["captures"] for s in graph_stats(runtime).values()) > captures_before
                index = totals["predictions"]
                record = {**result, "prediction_index": index, "camera_frame_index": frame.index,
                          "capture_generation": frame.metadata['capture_generation'],
                          "instruction": instruction, "instruction_revision": instruction_revision,
                          "instruction_changed_this_frame": instruction_revision != (last_record["instruction_revision"] if last_record else 0),
                          "rgb_shape": list(rgb.shape),
                          "camera_received_monotonic_ns": frame.timestamp_ns,
                          "camera_received_unix_ns": frame.metadata["host_unix_ns"],
                          "prediction_available_monotonic_ns": available_ns,
                          "frame_age_at_start_ms": age_ms,
                          "receipt_to_prediction_ms": (available_ns - frame.timestamp_ns) / 1e6,
                          "valid_history_frames": len(past),
                          "history_camera_frame_indices": [p["index"] for p in past],
                          "history_receipt_timestamps_ns": [p["host_monotonic_ns"] for p in past],
                          "history_step_gap_ms": (frame.timestamp_ns - past[-1]["host_monotonic_ns"]) / 1e6 if past else None,
                          "history_span_ms": (frame.timestamp_ns - past[0]["host_monotonic_ns"]) / 1e6 if past else 0,
                          "history_reset_this_frame": reset, "graph_capture_this_frame": capture,
                          "recent_prediction_hz": prediction_hz,
                          "skipped_camera_frames_since_previous": (frame.index - last_record["camera_frame_index"] - 1) if last_record else frame.index,
                          "robot_commands_sent": 0}
                image = Image.fromarray(rgb)
                payload = io.BytesIO()
                image.save(payload, format="JPEG", quality=85)
                jpeg = payload.getvalue()
                atomic_bytes(output / "latest_rgb.jpg", jpeg)
                if args.save_images:
                    record["image"] = f"rgb/{index:08d}.jpg"
                    (output / record["image"]).write_bytes(jpeg)
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                stream.flush()
                totals["predictions"] += 1
                totals["full_history_predictions"] += int(len(past) == 31)
                if capture:
                    totals["graph_capture_predictions"].append(index)
                records.append(record)
                camera_stats = camera.latest.stats()
                ready = len(past) == 31
                preview.update(status="running", initialization_stage="ready" if ready else "warming_up",
                               message="实时预测中" if ready else f"正在预热图像历史：{len(past)} / 31",
                               model_loaded=True, record=record, camera_recovery=camera.recovery_stats(),
                               instruction=instruction, instruction_revision=instruction_revision,
                               camera=camera_stats, camera_info=camera.info,
                               image_base64=base64.b64encode(jpeg).decode(),
                               published_unix_ms=time.time() * 1000,
                               frame_received_unix_ms=frame.metadata["host_unix_ns"] / 1e6,
                               history_resets=history.resets)
                if record["instruction_changed_this_frame"]:
                    logging.info("Applied instruction revision %d at prediction %d: %r",
                                 instruction_revision, index, instruction)
                last_image, last_record = image, record
                if time.monotonic() >= next_report:
                    atomic_json(output / "summary.json", summary("running"))
                    snapshots.submit(image, record)
                    manifest["camera"] = camera.info
                    manifest["camera_pid"] = camera.process.pid
                    atomic_json(output / "manifest.json", manifest)
                    logging.info("Prediction %d: camera=%.1f Hz, compute=%.1f ms, receipt-to-prediction=%.1f ms, past=%d, age=%.1f ms",
                                 index, camera_stats["recent_receipt_hz"] or 0, result["compute_ms"], record["receipt_to_prediction_ms"], len(past), age_ms)
                    next_report = time.monotonic() + 5
                next_due = started_ns / 1e9 + 1 / args.max_hz
    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"
        logging.exception("Live preview failed")
        atomic_bytes(output / "error.txt", traceback.format_exc().encode())
        if preview:
            preview.update(status="error", message=failure)
    finally:
        # Invalidate control.json before any shutdown joins or disk writes.
        if preview:
            preview.update(status="error" if failure else "stopped", message=failure or "本次预览已结束")
        for hook in hooks:
            hook.remove()
        if camera:
            camera.close()
        if snapshots:
            try:
                try:
                    if last_image is not None:
                        snapshots.submit(last_image, last_record)
                finally:
                    snapshots.close()
            except Exception as exc:
                failure = failure or f"{type(exc).__name__}: {exc}"
                logging.exception("Snapshot writer shutdown failed")
        if runtime:
            atomic_json(output / "summary.json", summary("error" if failure else "stopped"))
        else:
            atomic_json(output / "summary.json", {"status": "error" if failure else "stopped", "passed": False,
                                                  "failure": failure, **totals, "robot_commands_sent": 0})
        if preview:
            preview.update(status="error" if failure else "stopped", message=failure or "本次预览已结束")
            with preview.lock:
                atomic_json(output / "last_preview_state.json", preview.state)
            preview.close()
        logging.info("Preview ended; %d predictions. Results: %s", totals["predictions"], output)
    return 1 if failure or not totals["predictions"] else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="auto", help="Auto-select the single YUYV/MJPEG RGB node")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30, help="Requested camera capture rate")
    parser.add_argument("--max-hz", type=float, default=10, help="Maximum inference request rate")
    parser.add_argument("--duration", type=float, default=0, help="Seconds of camera streaming; 0 runs until Ctrl+C")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--port", type=int, default=8765, help="Loopback preview port; 0 selects a free port")
    parser.add_argument("--max-frame-age-ms", type=float, default=200)
    parser.add_argument("--history-reset-gap-ms", type=float, default=1000)
    parser.add_argument("--warmup-frames", type=int, default=5)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--instruction", default="Follow the person and avoid obstacles.")
    parser.add_argument("--acceleration", choices=["baseline", "fp16", "vectorized_fp16", "graph_fp16", "graph_fp32"], default="graph_fp16")
    parser.add_argument("--resources", type=Path, default=ROOT / "resources")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--save-images", action="store_true", help="Also save each RGB frame used for inference")
    args = parser.parse_args()
    if any(not math.isfinite(v) or v <= 0 for v in (args.max_hz, args.max_frame_age_ms, args.history_reset_gap_ms)):
        parser.error("Rates and age/gap thresholds must be finite positive numbers")
    if not math.isfinite(args.duration) or args.duration < 0 or min(args.max_frames, args.warmup_frames) < 0:
        parser.error("Duration and frame counts must be nonnegative")
    if min(args.width, args.height, args.fps, args.threads) <= 0 or not 0 <= args.port <= 65535:
        parser.error("Invalid dimensions, FPS, thread count or port")
    if not args.instruction.strip():
        parser.error("Instruction must not be empty")
    args.instruction = args.instruction.strip()
    if len(args.instruction) > 512:
        parser.error("Instruction must be at most 512 characters")
    args.resources = args.resources.resolve()
    output = (args.output or ROOT / "reports" / ("vlx-live-" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))).resolve()
    output.mkdir(parents=True, exist_ok=False)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                        handlers=[logging.StreamHandler(), logging.FileHandler(output / "run.log")])
    logging.getLogger("vlxgo.runtime").setLevel(logging.WARNING)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    return run(args, output, stop)


if __name__ == "__main__":
    sys.exit(main())
