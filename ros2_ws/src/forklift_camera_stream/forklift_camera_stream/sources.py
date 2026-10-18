"""Frame sources for the standalone streamer (no ROS): an OpenCV capture device (laptop webcam, USB camera) and a
synthetic test pattern. Both run a capture thread only while at least one viewer is connected."""

from __future__ import annotations

import logging
import threading
import time

import cv2
import numpy as np

from .mjpeg_core import FrameHub

log = logging.getLogger("camera_stream")


class _ThreadedSource:
    """start() / stop() are wired to FrameHub.on_first_client / on_last_client."""

    def __init__(self, hub: FrameHub, rate_hz: float) -> None:
        self.hub = hub
        self.period = 1.0 / rate_hz
        self.thread: threading.Thread | None = None
        self.stop_evt = threading.Event()
        self.lock = threading.Lock()
        self.frames = 0

    def start(self) -> None:
        with self.lock:
            if self.thread and self.thread.is_alive():
                return
            self.stop_evt = threading.Event()
            self.thread = threading.Thread(target=self._run, args=(self.stop_evt,), name=type(self).__name__,
                                           daemon=True)
            self.thread.start()

    def stop(self) -> None:
        with self.lock:
            self.stop_evt.set()
            t = self.thread
        if t and t is not threading.current_thread():
            t.join(timeout=2.0)

    @property
    def running(self) -> bool:
        return bool(self.thread and self.thread.is_alive())

    def _run(self, stop: threading.Event) -> None:
        raise NotImplementedError


class TestPatternSource(_ThreadedSource):
    """Moving colour bars + clock: proves the whole pipeline without a camera."""

    __test__ = False  # not a pytest class

    def __init__(self, hub: FrameHub, width: int = 640, height: int = 480,
                 rate_hz: float = 15.0, label: str = "TEST PATTERN") -> None:
        super().__init__(hub, rate_hz)
        self.width, self.height, self.label = width, height, label

    def render(self, t: float) -> np.ndarray:
        w, h = self.width, self.height
        img = np.zeros((h, w, 3), dtype=np.uint8)
        bars = [(255, 255, 255), (0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 255), (0, 0, 255),
                (255, 0, 0)]
        bw = w // len(bars)
        shift = int(t * 40) % w
        for i, c in enumerate(bars):
            x0 = (i * bw + shift) % w
            cv2.rectangle(img, (x0, 0), (min(x0 + bw, w) - 1, h * 2 // 3), c, -1)
            if x0 + bw > w:
                cv2.rectangle(img, (0, 0), (x0 + bw - w - 1, h * 2 // 3), c, -1)
        cv2.rectangle(img, (0, h * 2 // 3), (w, h), (40, 40, 40), -1)
        cx = int((np.sin(t * 1.5) * 0.4 + 0.5) * w)
        cv2.circle(img, (cx, h * 5 // 6), h // 14, (0, 140, 255), -1)
        stamp = time.strftime("%H:%M:%S") + f".{int((t % 1) * 10)}"
        cv2.putText(img, self.label, (12, h * 2 // 3 + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        cv2.putText(img, stamp, (12, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2)
        return img

    def _run(self, stop: threading.Event) -> None:
        t0 = time.monotonic()
        while not stop.is_set():
            self.hub.offer(self.render(time.monotonic() - t0))
            self.frames += 1
            stop.wait(self.period)


class OpenCvSource(_ThreadedSource):
    """cv2.VideoCapture on a device index (0 = laptop webcam) or a URL / file. The device is opened when the
    first viewer connects and released when the last one leaves (the webcam light goes off)."""

    def __init__(self, hub: FrameHub, device: int | str = 0, rate_hz: float = 15.0,
                 capture_size: tuple[int, int] = (640, 480)) -> None:
        super().__init__(hub, rate_hz)
        self.device, self.capture_size = device, capture_size
        self.error: str | None = None

    @staticmethod
    def probe(device: int | str) -> bool:
        cap = cv2.VideoCapture(device)
        try:
            return cap.isOpened() and cap.read()[0]
        finally:
            cap.release()

    def _run(self, stop: threading.Event) -> None:
        cap = cv2.VideoCapture(self.device)
        try:
            if not cap.isOpened():
                self.error = f"cannot open camera {self.device!r}"
                log.error(self.error)
                return
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.capture_size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.capture_size[1])
            self.error = None
            while not stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    self.error = "camera read failed"
                    stop.wait(0.2)
                    continue
                self.hub.offer(frame)
                self.frames += 1
        finally:
            cap.release()
