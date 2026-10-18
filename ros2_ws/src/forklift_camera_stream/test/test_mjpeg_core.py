"""UI-08 camera stream: encoding, multipart framing, rate limiting, token check, manual-mode gate, and a live
HTTP round trip (viewer connects -> frames flow -> viewer leaves -> the source stops)."""

import http.client
import json
import time

import cv2
import numpy as np
import pytest

from forklift_camera_stream.mjpeg_core import (BOUNDARY, CONTENT_TYPE, STREAM_PATH, FrameHub, MjpegServer,
                                               RateLimiter, StreamConfig, StreamGate, check_token, encode_jpeg,
                                               image_msg_to_bgr, multipart_part, parse_multipart)
from forklift_camera_stream.standalone import build


def frame(w=640, h=480):
    img = np.zeros((h, w, 3), np.uint8)
    cv2.rectangle(img, (w // 4, h // 4), (w * 3 // 4, h * 3 // 4), (0, 200, 255), -1)
    return img


# ------------------------------------------------------------------------------------------ encoding / framing

def test_encode_jpeg_resizes_to_low_resolution():
    jpeg = encode_jpeg(frame(), 320, 240, 60)
    assert jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9"
    back = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert back.shape == (240, 320, 3)
    assert len(jpeg) < 20_000


def test_quality_changes_size_and_grey_frames_work():
    img = (np.random.default_rng(0).random((480, 640)) * 255).astype(np.uint8)
    lo, hi = encode_jpeg(img, 320, 240, 20), encode_jpeg(img, 320, 240, 90)
    assert len(lo) < len(hi)
    assert cv2.imdecode(np.frombuffer(lo, np.uint8), cv2.IMREAD_UNCHANGED).shape == (240, 320)
    with pytest.raises(ValueError):
        encode_jpeg(np.zeros((2, 2, 2, 2), np.uint8), 320, 240, 60)


def test_multipart_framing_round_trip():
    parts = [b"\xff\xd8one\xff\xd9", b"\xff\xd8two--frame\r\n\xff\xd9"]   # boundary text inside a JPEG is fine
    body = b"".join(multipart_part(p) for p in parts)
    assert body.startswith(f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: ".encode())
    assert parse_multipart(body) == parts
    assert parse_multipart(body[:-5]) == parts[:1]          # an incomplete trailing part is not returned
    assert CONTENT_TYPE == "multipart/x-mixed-replace; boundary=frame"


@pytest.mark.parametrize("enc", ["mono8", "rgb8", "bgr8", "yuv422", "yuv422_yuy2"])
def test_image_msg_conversion(enc):
    h, w = 4, 6
    ch = {"mono8": 1, "rgb8": 3, "bgr8": 3}.get(enc, 2)
    step = w * ch + 2                                         # row padding must be dropped
    data = bytes(range(256)) * 2
    out = image_msg_to_bgr(enc, h, w, step, data[:h * step])
    assert out.shape[:2] == (h, w)
    if enc == "rgb8":
        raw = np.frombuffer(data[:h * step], np.uint8).reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        assert (out[..., 0] == raw[..., 2]).all()
    with pytest.raises(ValueError):
        image_msg_to_bgr("32FC1", h, w, step, data[:h * step])


# ------------------------------------------------------------------------------------------ policy

def test_rate_limiter_caps_fps_without_backlog():
    rl = RateLimiter(10.0)
    t, passed = 0.0, 0
    while t < 2.0:                    # producer at 100 Hz for 2 s
        passed += rl.ready(t)
        t += 0.01
    assert 19 <= passed <= 21
    assert rl.ready(10.0) and not rl.ready(10.05)   # after a long pause: one frame, not a burst
    assert rl.ready(10.1)


def test_token_check():
    assert check_token("", None, None)                     # bench mode: open
    assert check_token("s3cret", "Bearer s3cret", None)
    assert check_token("s3cret", None, "s3cret")
    assert not check_token("s3cret", None, None)
    assert not check_token("s3cret", "Bearer nope", "s3cret")   # a wrong header is not rescued by the query
    assert not check_token("s3cret", "Basic s3cret", None)
    assert not check_token("s3cret", None, "")


def test_gate_manual_mode_or_explicit_request():
    g = StreamGate()
    assert not g.allowed() and "manual" in g.reason()
    g.set_mode("busy")
    assert not g.allowed()
    g.set_mode("manual")
    assert g.allowed() and g.reason() == ""
    g.set_mode("idle")
    g.set_requested(True)
    assert g.allowed()


def test_config_validation():
    cfg = StreamConfig.from_dict({"width": "320", "fps": 5, "jpeg_quality": 50, "unknown": 1})
    assert (cfg.width, cfg.fps, cfg.jpeg_quality) == (320, 5.0, 50)
    for bad in ({"fps": 0}, {"jpeg_quality": 101}, {"width": 4}, {"max_clients": 0}):
        with pytest.raises(ValueError):
            StreamConfig.from_dict(bad)


def test_hub_encodes_only_while_watched_and_rate_limited():
    t = [0.0]
    started, stopped = [], []
    hub = FrameHub(StreamConfig(fps=5, max_clients=1), clock=lambda: t[0],
                   on_first_client=lambda: started.append(1), on_last_client=lambda: stopped.append(1))
    assert not hub.offer(frame())                 # nobody watching: nothing encoded
    assert hub.acquire() and started == [1]
    assert not hub.acquire()                      # max_clients
    for i in range(50):                           # 50 frames over 1 s at 5 fps
        t[0] = i * 0.02
        hub.offer(frame())
    assert hub.encoded == 5
    seq, jpeg = hub.wait_next(0, timeout=0.01)
    assert seq == 5 and jpeg
    assert hub.wait_next(seq, timeout=0.01) == (5, None)
    hub.release()
    assert stopped == [1] and hub.jpeg is None


# ------------------------------------------------------------------------------------------ live HTTP

@pytest.fixture()
def server():
    srv, src = build(StreamConfig(port=0, bind="127.0.0.1", token="s3cret", fps=20, max_clients=1), "test",
                     requested=False)
    srv.start()
    yield srv, src
    srv.stop()
    src.stop()


def get(srv, path, headers=None):
    c = http.client.HTTPConnection("127.0.0.1", srv.port, timeout=5)
    c.request("GET", path, headers=headers or {})
    return c, c.getresponse()


def wait_for(pred, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


def test_http_auth_gate_and_stream(server):
    srv, src = server
    auth = {"Authorization": "Bearer s3cret"}
    _, r = get(srv, STREAM_PATH)
    assert r.status == 401
    _, r = get(srv, STREAM_PATH, auth)
    assert r.status == 409 and json.loads(r.read())["error"] == "camera_inactive"    # not in manual mode
    assert not src.running

    srv.gate.set_mode("manual")
    c, r = get(srv, f"{STREAM_PATH}?token=s3cret")
    assert r.status == 200 and r.getheader("Content-Type") == CONTENT_TYPE
    body = b""
    while len(parse_multipart(body)) < 3:
        body += r.read1(65536)
    jpegs = parse_multipart(body)
    assert cv2.imdecode(np.frombuffer(jpegs[0], np.uint8), cv2.IMREAD_COLOR).shape == (240, 320, 3)
    assert src.running and srv.hub.clients == 1

    _, r2 = get(srv, STREAM_PATH, auth)                    # max_clients = 1
    assert r2.status == 503

    _, h = get(srv, "/health", auth)
    health = json.loads(h.read())
    assert health["allowed"] and health["clients"] == 1 and health["width"] == 320

    r.close()
    c.close()                                              # viewer disconnects -> loop ends, source stops
    assert wait_for(lambda: srv.hub.clients == 0)
    assert wait_for(lambda: not src.running)


def test_stream_ends_when_robot_leaves_manual_mode(server):
    srv, src = server
    srv.gate.set_mode("manual")
    c, r = get(srv, STREAM_PATH, {"Authorization": "Bearer s3cret"})
    assert r.status == 200
    body = b""
    while not parse_multipart(body):
        body += r.read1(65536)
    srv.gate.set_mode("busy")
    srv.hub.kick()
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:                      # the server closes the response
        if not r.read1(65536):
            break
    else:
        pytest.fail("stream did not end after leaving manual mode")
    assert wait_for(lambda: srv.hub.clients == 0 and not src.running)
    c.close()
