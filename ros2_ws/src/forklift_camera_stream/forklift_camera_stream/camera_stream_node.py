"""camera_stream — thin rclpy wrapper around mjpeg_core (UI-08: low-resolution camera for manual control).

  in : image_raw (sensor_msgs/Image from v4l2_camera, mono8 / rgb8 / yuv422, best effort) · fleet_status
       (FleetStatus, its `mode` opens the gate in manual mode)
  srv: camera/enable (std_srvs/SetBool) — stream outside manual mode on explicit request (bench, demo)
  out: HTTP `GET http://<robot>:<port>/stream.mjpg` (multipart/x-mixed-replace), `GET /health` — relayed to the
       dashboard by the fleet server (`GET /api/robots/{id}/camera`); nothing is encoded while nobody watches
"""

from __future__ import annotations

import dataclasses
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

from forklift_interfaces.msg import FleetStatus

from .mjpeg_core import STREAM_PATH, FrameHub, MjpegServer, StreamConfig, StreamGate, image_msg_to_bgr


class CameraStreamNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_stream")
        values = {}
        for f in dataclasses.fields(StreamConfig):
            values[f.name] = self.declare_parameter(f.name, getattr(StreamConfig(), f.name)).value
        token_file = self.declare_parameter("token_file", "/etc/forklift/camera_token").value
        if not values["token"] and token_file and os.path.exists(token_file):
            with open(token_file, encoding="utf-8") as f:
                values["token"] = f.read().strip()
        cfg = StreamConfig.from_dict(values)
        self.gate = StreamGate(requested=bool(self.declare_parameter("always_on", False).value))
        self.hub = FrameHub(cfg)
        self.server = MjpegServer(cfg, self.hub, self.gate).start()
        self.bad_encoding = ""
        self.create_subscription(Image, "image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(FleetStatus, "fleet_status", self.on_status, 10)
        self.create_service(SetBool, "camera/enable", self.on_enable)
        self.get_logger().info(f"MJPEG on :{self.server.port}{STREAM_PATH} ({cfg.width}x{cfg.height} @ {cfg.fps} fps,"
                               f" q{cfg.jpeg_quality}){'' if cfg.token else ' — WARNING: no token'}")

    def on_image(self, msg: Image) -> None:
        if not self.hub.wants_frames():
            return
        try:
            frame = image_msg_to_bgr(msg.encoding, msg.height, msg.width, msg.step, bytes(msg.data))
        except ValueError as e:
            if msg.encoding != self.bad_encoding:
                self.bad_encoding = msg.encoding
                self.get_logger().error(str(e))
            return
        self.hub.offer(frame)

    def on_status(self, msg: FleetStatus) -> None:
        was = self.gate.allowed()
        self.gate.set_mode(msg.mode)
        if was and not self.gate.allowed():
            self.hub.kick()  # end open streams now, not on the next frame

    def on_enable(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self.gate.set_requested(req.data)
        if not self.gate.allowed():
            self.hub.kick()
        res.success, res.message = True, f"camera stream {'requested' if req.data else 'manual mode only'}"
        return res

    def destroy_node(self) -> None:
        self.server.stop()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
