"""Raw RGB camera bridge and a single-slot mailbox, independent of CUDA/DDS.

Capture runs in the existing camera environment; inference uses .venv-vlx.
The pipe is continuously drained into one replaceable frame, never a FIFO of
inference requests. Timestamps mean host receipt after read, not exposure.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time


def read_exact(stream, size):
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.read(size - len(chunks))
        if not chunk:
            raise EOFError("Camera stream ended before a complete packet")
        chunks.extend(chunk)
    return bytes(chunks)


def read_packet(stream):
    header_size = struct.unpack("!I", read_exact(stream, 4))[0]
    if not 0 < header_size <= 65536:
        raise ValueError("Invalid camera packet header length")
    header = json.loads(read_exact(stream, header_size))
    size = header.get("payload_bytes", 0)
    if not isinstance(size, int) or not 0 <= size <= 50_000_000:
        raise ValueError("Invalid camera packet payload length")
    return header, read_exact(stream, size)


def write_packet(stream, header, payload=b""):
    header = {**header, "payload_bytes": len(payload)}
    encoded = json.dumps(header, allow_nan=False).encode()
    stream.write(struct.pack("!I", len(encoded)))
    stream.write(encoded)
    stream.write(payload)
    stream.flush()


@dataclass(frozen=True)
class CameraFrame:
    metadata: dict
    rgb_bytes: bytes

    @property
    def index(self):
        return self.metadata["index"]

    @property
    def timestamp_ns(self):
        return self.metadata["host_monotonic_ns"]


class LatestFrame:
    """One pending frame; a slow consumer receives the most recent one."""

    def __init__(self):
        self.condition = threading.Condition()
        self.frame = None
        self.consumed_index = -1
        self.error = None
        self.received = 0
        self.consumed = 0
        self.replaced = 0
        self.first_ns = None
        self.timestamps = deque(maxlen=300)

    def publish(self, frame):
        with self.condition:
            if self.frame is not None:
                if frame.index <= self.frame.index or frame.timestamp_ns <= self.frame.timestamp_ns:
                    raise ValueError("Camera indices and receipt timestamps must increase")
                if self.frame.index != self.consumed_index:
                    self.replaced += 1
            self.frame = frame
            self.received += 1
            if self.first_ns is None:
                self.first_ns = frame.timestamp_ns
            self.timestamps.append(frame.timestamp_ns)
            self.condition.notify_all()

    def fail(self, message):
        with self.condition:
            self.error = message
            self.condition.notify_all()

    def take(self, timeout=2.0):
        deadline = time.monotonic() + timeout
        with self.condition:
            while True:
                if self.error:
                    raise RuntimeError(self.error)
                if self.frame is not None and self.frame.index != self.consumed_index:
                    self.consumed_index = self.frame.index
                    self.consumed += 1
                    return self.frame
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("No new camera frame within timeout")
                self.condition.wait(remaining)

    def stats(self):
        with self.condition:
            times = list(self.timestamps)
            return {
                "received": self.received, "consumed": self.consumed,
                "replaced_before_inference": self.replaced,
                "pending_frames": int(self.frame is not None and self.frame.index != self.consumed_index),
                "pending_capacity": 1, "error": self.error,
                "receipt_hz": ((self.received - 1) * 1e9 / (times[-1] - self.first_ns)
                               if len(times) > 1 else None),
                "recent_receipt_hz": ((len(times) - 1) * 1e9 / (times[-1] - times[0])
                                      if len(times) > 1 else None),
            }


class LiveHistory:
    """Metadata for exactly the frames submitted successfully to the runtime."""

    def __init__(self, size=31, reset_gap_ms=1000):
        self.frames = deque(maxlen=size)
        self.reset_gap_ns = round(reset_gap_ms * 1e6)
        self.resets = 0

    def prepare(self, current):
        reset = False
        if self.frames:
            previous = self.frames[-1]
            gap = current["host_monotonic_ns"] - previous["host_monotonic_ns"]
            changed = current.get("capture_generation") != previous.get("capture_generation")
            if gap <= 0 or (not changed and current["index"] <= previous["index"]):
                raise ValueError("Live history cannot contain duplicate or future frames")
            if changed or gap > self.reset_gap_ns:
                self.frames.clear()
                self.resets += 1
                reset = True
        return list(self.frames), reset

    def commit(self, current):
        self.frames.append(dict(current))


class CameraProcess:
    """Restart only capture after EOF/stalls; retain the loaded model.

    Each restart discovers the RGB device again. Parent frame indices remain
    increasing, and capture_generation tells inference to discard old history.
    """

    def __init__(self, python, device, width, height, fps, output, retry_s=2, on_status=None):
        self.command = [str(python), "-u", str(Path(__file__).resolve()), "--capture",
                        "--device", device, "--width", str(width), "--height", str(height),
                        "--fps", str(fps)]
        self.retry_s = retry_s
        self.on_status = on_status
        self.generation = -1
        self.next_index = 0
        self.reconnect_attempts = 0
        self.recoveries = 0
        self.reconnecting = False
        self.retry_pending = False
        self.last_error = None
        self.process = self.thread = None
        self.log = (output / "camera.log").open("x")
        try:
            self._start()
        except BaseException:
            self.log.close()
            raise

    def _start(self):
        self.latest = LatestFrame()
        self.info = {}
        self.stopping = False
        self.process = self.thread = None
        self.generation += 1
        self.process = subprocess.Popen(self.command, stdout=subprocess.PIPE, stderr=self.log, bufsize=0)
        self.thread = threading.Thread(target=self._read, name="camera-pipe-reader", daemon=True)
        self.thread.start()

    def _read(self):
        try:
            while not self.stopping:
                meta, payload = read_packet(self.process.stdout)
                if meta.get("kind") in {"detected", "opened"}:
                    self.info = {**self.info, **meta, "capture_generation": self.generation}
                    if self.on_status is not None:
                        self.on_status(self.info)
                    continue
                if meta.get("kind") != "frame" or meta.get("color_order") != "RGB":
                    raise ValueError("Expected an RGB camera frame")
                if len(payload) != meta["width"] * meta["height"] * 3:
                    raise ValueError("Camera payload does not match RGB dimensions")
                meta = {**meta, "device_frame_index": meta["index"], "index": self.next_index,
                        "capture_generation": self.generation}
                self.latest.publish(CameraFrame(meta, payload))
                self.next_index += 1
        except Exception as exc:
            if not self.stopping:
                self.latest.fail(f"{exc}; see camera.log")

    def _stop_capture(self):
        self.stopping = True
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        if self.thread is not None:
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                raise RuntimeError("Camera reader did not stop; refusing a second capture process")
        if self.process is not None:
            self.process.stdout.close()

    def restart(self):
        self._stop_capture()
        self._start()

    def take(self, stop, timeout=3):
        """Return a new frame or None while reconnecting; waits are cancellable."""
        if stop.is_set():
            return None
        if self.retry_pending:
            if stop.wait(self.retry_s):
                return None
            self.reconnect_attempts += 1
            try:
                self.restart()
            except (OSError, RuntimeError) as exc:
                self.last_error = str(exc)
                return None
            self.retry_pending = False
        try:
            deadline = time.monotonic() + timeout
            while not stop.is_set():
                try:
                    frame = self.latest.take(timeout=max(0, min(.2, deadline - time.monotonic())))
                    break
                except TimeoutError:
                    if time.monotonic() >= deadline:
                        raise
            else:
                return None
        except (RuntimeError, TimeoutError) as exc:
            self.last_error = str(exc)
            self.reconnecting = self.retry_pending = True
            return None
        if self.reconnecting:
            self.recoveries += 1
        self.reconnecting = False
        return frame

    def recovery_stats(self):
        return {"attempts": self.reconnect_attempts, "recoveries": self.recoveries,
                "reconnecting": self.reconnecting, "last_error": self.last_error,
                "capture_generation": self.generation}

    def close(self):
        self._stop_capture()
        self.log.close()


def choose_device(requested):
    if requested != "auto":
        device = Path(requested).resolve()
        if not device.exists():
            raise ValueError(f"Camera device does not exist: {device}")
        return str(device)
    candidates = []
    for device in sorted(Path("/dev").glob("video*")):
        result = subprocess.run(["v4l2-ctl", "-d", str(device), "--list-formats"],
                                text=True, capture_output=True, timeout=5)
        if result.returncode == 0 and any(f"'{fmt}'" in result.stdout for fmt in ("YUYV", "MJPG", "RGB3")):
            candidates.append(str(device))
    if len(candidates) != 1:
        raise ValueError(f"Expected one RGB camera; found {candidates}. Select --device /dev/videoN")
    return candidates[0]


def capture(args):
    import cv2
    cv2.setNumThreads(1)
    device = choose_device(args.device)
    write_packet(sys.stdout.buffer, {"kind": "detected", "device": device})
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open RGB camera {device}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        info = {"kind": "opened", "device": device, "backend": cap.getBackendName(),
                "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                "requested_fps": args.fps, "reported_fps": cap.get(cv2.CAP_PROP_FPS),
                "reported_buffers": cap.get(cv2.CAP_PROP_BUFFERSIZE),
                "opencv": cv2.__version__,
                "name": Path(f"/sys/class/video4linux/{Path(device).name}/name").read_text().strip(),
                "timestamp_semantics": "AGX monotonic receipt after read, not exposure time"}
        if (info["width"], info["height"]) != (args.width, args.height):
            raise RuntimeError(f"Camera did not accept requested dimensions: {info}")
        write_packet(sys.stdout.buffer, info)
        index = 0
        while True:
            ok, bgr = cap.read()
            received = time.monotonic_ns()
            unix = time.time_ns()
            if not ok or bgr is None:
                raise RuntimeError("RGB camera disconnected or read failed")
            if bgr.shape != (args.height, args.width, 3):
                raise RuntimeError(f"Unexpected RGB frame shape: {bgr.shape}")
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            write_packet(sys.stdout.buffer, {
                "kind": "frame", "index": index, "host_monotonic_ns": received,
                "host_unix_ns": unix, "width": args.width, "height": args.height,
                "color_order": "RGB", "dtype": "uint8",
            }, rgb.tobytes())
            index += 1
    finally:
        cap.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", action="store_true", required=True)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    capture(parser.parse_args())
