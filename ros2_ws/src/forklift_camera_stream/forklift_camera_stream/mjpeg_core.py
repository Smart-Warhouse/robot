"""Low-resolution MJPEG stream of the fork camera for manual control (UI-08) — plain Python, no rclpy.

The robot serves `GET /stream.mjpg` (multipart/x-mixed-replace, one JPEG per part) to the fleet server, which
relays it to the dashboard (docs/02-api-contract.md, `GET /api/robots/{id}/camera`). Rules:

  * low resolution / rate: frames are resized to `width` x `height` and encoded at most `fps` times a second,
    once, whatever the number of viewers (the Pi encodes nothing while nobody watches);
  * gated: a stream is only served while the robot is in `manual` mode or a stream was explicitly requested
    (`camera/enable` service, or the standalone runner); when the gate closes, open streams end;
  * authenticated: a shared token (`Authorization: Bearer <token>` or `?token=`), compared in constant time;
  * bounded: at most `max_clients` viewers; a viewer that disconnects ends its loop on the next write, and the
    frame source is told when the last viewer leaves (the webcam / capture thread stops).
"""

from __future__ import annotations

import hmac
import json
import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, fields
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np

log = logging.getLogger("camera_stream")

BOUNDARY = "frame"
CONTENT_TYPE = f"multipart/x-mixed-replace; boundary={BOUNDARY}"
STREAM_PATH = "/stream.mjpg"


@dataclass
class StreamConfig:
    width: int = 320
    height: int = 240
    fps: float = 10.0
    jpeg_quality: int = 60          # 1..100; 60 keeps a 320x240 frame around 8-12 kB
    port: int = 8081
    bind: str = "0.0.0.0"
    token: str = ""                 # shared with the fleet server (CAMERA_TOKEN); empty = no auth (bench only)
    max_clients: int = 2            # the server relays one stream per dashboard viewer
    stall_timeout_s: float = 5.0    # end a stream when no frame arrived for this long

    @classmethod
    def from_dict(cls, d: dict) -> "StreamConfig":
        names = {f.name: f.type for f in fields(cls)}
        out = cls()
        for k, v in d.items():
            if k in names:
                setattr(out, k, type(getattr(out, k))(v))
        out.validate()
        return out

    def validate(self) -> None:
        if not (16 <= self.width <= 1920 and 16 <= self.height <= 1080):
            raise ValueError("width/height out of range")
        if not 0.5 <= self.fps <= 30:
            raise ValueError("fps must be within 0.5..30")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be within 1..100")
        if self.max_clients < 1:
            raise ValueError("max_clients must be >= 1")


# ---------------------------------------------------------------------------------------------- encoding

def encode_jpeg(frame: np.ndarray, width: int, height: int, quality: int) -> bytes:
    """BGR or grey frame -> JPEG bytes at `width` x `height` (INTER_AREA when shrinking)."""
    if frame.ndim not in (2, 3):
        raise ValueError(f"unsupported frame shape {frame.shape}")
    h, w = frame.shape[:2]
    if (w, h) != (width, height):
        interp = cv2.INTER_AREA if w > width or h > height else cv2.INTER_LINEAR
        frame = cv2.resize(frame, (width, height), interpolation=interp)
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return buf.tobytes()


def multipart_part(jpeg: bytes, boundary: str = BOUNDARY) -> bytes:
    """One part of a multipart/x-mixed-replace body: boundary line, headers, blank line, JPEG, CRLF."""
    head = (f"--{boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n").encode("ascii")
    return head + jpeg + b"\r\n"


def parse_multipart(body: bytes, boundary: str = BOUNDARY) -> list[bytes]:
    """Inverse of `multipart_part` for complete parts (tests and the server-side relay tests)."""
    out, marker, i = [], f"--{boundary}\r\n".encode(), 0
    while True:
        start = body.find(marker, i)
        if start < 0:
            return out
        head_end = body.find(b"\r\n\r\n", start)
        if head_end < 0:
            return out
        headers = body[start + len(marker):head_end].decode("ascii").split("\r\n")
        length = next(int(h.split(":", 1)[1]) for h in headers if h.lower().startswith("content-length"))
        data = body[head_end + 4:head_end + 4 + length]
        if len(data) < length:
            return out
        out.append(data)
        i = head_end + 4 + length


def image_msg_to_bgr(encoding: str, height: int, width: int, step: int, data: bytes) -> np.ndarray:
    """sensor_msgs/Image fields -> numpy frame without cv_bridge (v4l2_camera publishes mono8 / rgb8 / yuv422)."""
    buf = np.frombuffer(data, dtype=np.uint8)
    enc = encoding.lower()
    if enc in ("mono8", "8uc1"):
        return buf.reshape(height, step)[:, :width].copy()
    if enc in ("bgr8", "rgb8", "8uc3"):
        img = buf.reshape(height, step)[:, :width * 3].reshape(height, width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img.copy()
    if enc in ("yuv422", "uyvy"):
        img = buf.reshape(height, step)[:, :width * 2].reshape(height, width, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_UYVY)
    if enc in ("yuv422_yuy2", "yuyv"):
        img = buf.reshape(height, step)[:, :width * 2].reshape(height, width, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUYV)
    raise ValueError(f"unsupported image encoding {encoding!r}")


# ---------------------------------------------------------------------------------------------- policy

class RateLimiter:
    """At most `fps` events a second; the first event always passes."""

    def __init__(self, fps: float) -> None:
        self.period = 1.0 / fps
        self.next_t: float | None = None

    def ready(self, now: float) -> bool:
        if self.next_t is not None and now < self.next_t:
            return False
        # schedule from the ideal time so the average rate is exact, but never build up a backlog
        self.next_t = now + self.period if self.next_t is None or now - self.next_t > self.period else \
            self.next_t + self.period
        return True


def check_token(expected: str, authorization: str | None, query_token: str | None) -> bool:
    """Bearer header or ?token=, constant-time. An empty expected token disables auth (bench use only)."""
    if not expected:
        return True
    given = ""
    if authorization and authorization.lower().startswith("bearer "):
        given = authorization[7:].strip()
    elif query_token:
        given = query_token
    return bool(given) and hmac.compare_digest(given.encode(), expected.encode())


class StreamGate:
    """Streaming is allowed only in manual mode or while explicitly requested (UI-08)."""

    def __init__(self, requested: bool = False) -> None:
        self.mode = ""
        self.requested = requested
        self._lock = threading.Lock()

    def set_mode(self, mode: str) -> None:
        with self._lock:
            self.mode = mode

    def set_requested(self, on: bool) -> None:
        with self._lock:
            self.requested = on

    def allowed(self) -> bool:
        with self._lock:
            return self.requested or self.mode == "manual"

    def reason(self) -> str:
        return "" if self.allowed() else "camera streams only in manual mode"


# ---------------------------------------------------------------------------------------------- frames

class FrameHub:
    """Latest encoded frame + viewer count. Producers call `offer(frame)` as often as they like; it encodes at
    most `fps` times a second and only while someone watches. Viewers wait on `wait_next(seq)`."""

    def __init__(self, cfg: StreamConfig, clock: Callable[[], float] = time.monotonic,
                 on_first_client: Callable[[], None] | None = None,
                 on_last_client: Callable[[], None] | None = None) -> None:
        self.cfg = cfg
        self.clock = clock
        self.limiter = RateLimiter(cfg.fps)
        self.on_first_client, self.on_last_client = on_first_client, on_last_client
        self.cond = threading.Condition()
        self.jpeg: bytes | None = None
        self.seq = 0
        self.clients = 0
        self.encoded = 0

    def wants_frames(self) -> bool:
        return self.clients > 0

    def offer(self, frame: np.ndarray) -> bool:
        """Returns True when the frame was encoded and published."""
        if not self.clients or not self.limiter.ready(self.clock()):
            return False
        jpeg = encode_jpeg(frame, self.cfg.width, self.cfg.height, self.cfg.jpeg_quality)
        with self.cond:
            self.jpeg, self.seq = jpeg, self.seq + 1
            self.encoded += 1
            self.cond.notify_all()
        return True

    def wait_next(self, seq: int, timeout: float) -> tuple[int, bytes | None]:
        with self.cond:
            self.cond.wait_for(lambda: self.seq != seq, timeout=timeout)
            return self.seq, (self.jpeg if self.seq != seq else None)

    def kick(self) -> None:
        """Wake every waiting viewer (gate closed / shutting down)."""
        with self.cond:
            self.cond.notify_all()

    def acquire(self) -> bool:
        with self.cond:
            if self.clients >= self.cfg.max_clients:
                return False
            self.clients += 1
            first = self.clients == 1
        if first:
            self.limiter = RateLimiter(self.cfg.fps)
            if self.on_first_client:
                self.on_first_client()
        return True

    def release(self) -> None:
        with self.cond:
            self.clients -= 1
            last = self.clients == 0
            if last:
                self.jpeg = None  # never show a stale frame to the next viewer
        if last and self.on_last_client:
            self.on_last_client()


# ---------------------------------------------------------------------------------------------- HTTP

class MjpegServer:
    """ThreadingHTTPServer serving `/stream.mjpg` and `/health` from a FrameHub."""

    def __init__(self, cfg: StreamConfig, hub: FrameHub, gate: StreamGate) -> None:
        self.cfg, self.hub, self.gate = cfg, hub, gate
        self.running = True
        self.httpd = ThreadingHTTPServer((cfg.bind, cfg.port), _make_handler(self))
        self.httpd.daemon_threads = True
        self.thread: threading.Thread | None = None

    @property
    def port(self) -> int:
        return self.httpd.server_address[1]

    def start(self) -> "MjpegServer":
        self.thread = threading.Thread(target=self.httpd.serve_forever, name="mjpeg-http", daemon=True)
        self.thread.start()
        return self

    def stop(self) -> None:
        self.running = False
        self.hub.kick()
        self.httpd.shutdown()
        self.httpd.server_close()

    def health(self) -> dict:
        return {"allowed": self.gate.allowed(), "mode": self.gate.mode, "requested": self.gate.requested,
                "clients": self.hub.clients, "max_clients": self.cfg.max_clients, "width": self.cfg.width,
                "height": self.cfg.height, "fps": self.cfg.fps}


def _make_handler(server: MjpegServer) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"   # the stream ends by closing the connection
        server_version = "forklift-camera/0.1"

        def log_message(self, fmt: str, *args) -> None:  # route through logging, not stderr
            log.debug("%s %s", self.address_string(), fmt % args)

        def _json(self, status: int, body: dict) -> None:
            raw = json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self) -> None:  # noqa: N802 (http.server API)
            url = urlsplit(self.path)
            query = parse_qs(url.query)
            if not check_token(server.cfg.token, self.headers.get("Authorization"), query.get("token", [None])[0]):
                return self._json(HTTPStatus.UNAUTHORIZED, {"error": "unauthorized"})
            if url.path == "/health":
                return self._json(HTTPStatus.OK, server.health())
            if url.path != STREAM_PATH:
                return self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})
            if not server.gate.allowed():
                return self._json(HTTPStatus.CONFLICT, {"error": "camera_inactive", "message": server.gate.reason()})
            if not server.hub.acquire():
                return self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "too_many_viewers"})
            try:
                self._stream()
            finally:
                server.hub.release()

        def _stream(self) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", CONTENT_TYPE)
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            seq, last_frame = 0, time.monotonic()
            while server.running and server.gate.allowed():
                seq, jpeg = server.hub.wait_next(seq, timeout=0.5)
                now = time.monotonic()
                if jpeg is None:
                    if now - last_frame > server.cfg.stall_timeout_s:
                        log.warning("no camera frame for %.1f s, ending stream", now - last_frame)
                        return
                    continue
                last_frame = now
                try:
                    self.wfile.write(multipart_part(jpeg))
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                    return  # viewer went away

    return Handler
