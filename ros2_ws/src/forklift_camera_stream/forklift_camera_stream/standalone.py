"""Standalone MJPEG streamer (no ROS): laptop webcam or test pattern, for bench tests and the local demo.

    python -m forklift_camera_stream.standalone --source 0            # laptop webcam
    python -m forklift_camera_stream.standalone --source test         # synthetic test pattern
    python -m forklift_camera_stream.standalone --source 0 --token s3cret --port 8081

Starting it is the explicit request (UI-08 gate): the stream is allowed while the process runs. On the robot use
`ros2 run forklift_camera_stream camera_stream` instead, which only streams in manual mode.
"""

from __future__ import annotations

import argparse
import logging
import signal
import threading

from .mjpeg_core import STREAM_PATH, FrameHub, MjpegServer, StreamConfig, StreamGate
from .sources import OpenCvSource, TestPatternSource


def build(cfg: StreamConfig, source: str, requested: bool = True) -> tuple[MjpegServer, object]:
    """Hub + gate + source + HTTP server, wired so the source only runs while someone watches."""
    hub = FrameHub(cfg)
    if source in ("test", "pattern", "test-pattern"):
        src = TestPatternSource(hub, rate_hz=max(cfg.fps * 1.5, 5.0))
    else:
        src = OpenCvSource(hub, int(source) if source.isdigit() else source, rate_hz=max(cfg.fps * 1.5, 5.0))
    hub.on_first_client, hub.on_last_client = src.start, src.stop
    return MjpegServer(cfg, hub, StreamGate(requested=requested)), src


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0", help="camera index, video URL/file, or 'test' (default 0)")
    d = StreamConfig()
    ap.add_argument("--port", type=int, default=d.port)
    ap.add_argument("--bind", default="127.0.0.1", help="0.0.0.0 to expose on the LAN (default 127.0.0.1)")
    ap.add_argument("--width", type=int, default=d.width)
    ap.add_argument("--height", type=int, default=d.height)
    ap.add_argument("--fps", type=float, default=d.fps)
    ap.add_argument("--quality", type=int, default=d.jpeg_quality, help="JPEG quality 1..100")
    ap.add_argument("--token", default="", help="shared token (the server's CAMERA_TOKEN); empty = open")
    ap.add_argument("--max-clients", type=int, default=d.max_clients)
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    cfg = StreamConfig(width=a.width, height=a.height, fps=a.fps, jpeg_quality=a.quality, port=a.port,
                       bind=a.bind, token=a.token, max_clients=a.max_clients)
    cfg.validate()
    if a.source not in ("test", "pattern", "test-pattern") and not OpenCvSource.probe(
            int(a.source) if a.source.isdigit() else a.source):
        raise SystemExit(f"camera {a.source!r} cannot be opened (use --source test for the synthetic pattern)")
    server, _ = build(cfg, a.source)
    server.start()
    logging.info("streaming %s at %dx%d @ %.0f fps on http://%s:%d%s%s", a.source, cfg.width, cfg.height, cfg.fps,
                 cfg.bind, server.port, STREAM_PATH, "" if cfg.token else " (no token)")
    done = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: done.set())
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    while not done.wait(0.5):
        pass
    server.stop()


if __name__ == "__main__":
    main()
