"""aruco_node — thin rclpy wrapper around aruco_core (PC-03..PC-05).

  in : image_raw (sensor_msgs/Image, 640x480 @ 30 fps, JS-03), camera_info, TF map <- camera_optical_frame
  out: aruco/raw        (PalletPose, frame = camera optical, one per detected marker and frame — raw solvePnP)
       pallet_detection (PalletPose, frame = map, 5-sample moving average, every pallet / location marker)
       pallet_pose      (geometry_msgs/PoseStamped, frame = map) — the target pallet (PC-05)
       pallet_pose/confidence (std_msgs/Float32)
  param target_pallet_id: 0 = nearest pallet marker; set by the dock server through `perception/target_pallet`.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, Int32
from tf2_ros import Buffer, TransformException, TransformListener

from forklift_interfaces.msg import PalletPose

from .aruco_core import PALLET, ArucoEstimator, MarkerSpec, PoseFilter, pallet_pose_in_frame, transform_from, \
    yaw_quaternion


class ArucoNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_node")
        P = self.declare_parameter
        spec = MarkerSpec(
            pallet_ids=tuple(P("pallet_ids", [1, 20]).value), location_ids=tuple(P("location_ids", [21, 40]).value),
            pallet_size_m=float(P("pallet_marker_size_m", 0.03).value),
            location_size_m=float(P("location_marker_size_m", 0.06).value))
        self.est = ArucoEstimator(spec, P("dictionary", "DICT_4X4_50").value, bool(P("subpix", True).value),
                                  float(P("min_side_px", 12.0).value))
        self.window = int(P("average_window", 5).value)
        self.max_gap = float(P("max_gap_s", 0.5).value)
        self.max_jump = float(P("max_jump_m", 0.05).value)
        self.min_conf = float(P("min_confidence", 0.2).value)
        self.map_frame = P("map_frame", "map").value
        self.target = int(P("target_pallet_id", 0).value)
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.filters: dict[int, PoseFilter] = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, "camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(Int32, "perception/target_pallet", lambda m: setattr(self, "target", m.data), 10)
        self.raw_pub = self.create_publisher(PalletPose, "aruco/raw", 10)
        self.det_pub = self.create_publisher(PalletPose, "pallet_detection", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "pallet_pose", 10)
        self.conf_pub = self.create_publisher(Float32, "pallet_pose/confidence", 10)

    def on_info(self, msg: CameraInfo) -> None:
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    def on_image(self, msg: Image) -> None:
        if self.K is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        obs = self.est.estimate(img, self.K, self.D)
        if not obs:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, Time())
            except TransformException as exc:
                self.get_logger().warn(f"no TF {self.map_frame} <- {msg.header.frame_id}: {exc}", throttle_duration_sec=5)
                tf = None
        T_map_cam = None
        if tf is not None:
            t, q = tf.transform.translation, tf.transform.rotation
            T_map_cam = transform_from((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        now = time.monotonic()
        best = None
        for o in obs:
            raw = PalletPose()
            raw.header = msg.header
            raw.marker_id, raw.kind, raw.confidence, raw.samples = o.marker_id, o.kind, float(o.confidence), 1
            raw.distance_m = o.distance_m
            raw.pose.position.x, raw.pose.position.y, raw.pose.position.z = (float(v) for v in o.tvec)
            (raw.pose.orientation.x, raw.pose.orientation.y, raw.pose.orientation.z,
             raw.pose.orientation.w) = rotation_to_quaternion(o.R)
            self.raw_pub.publish(raw)
            if T_map_cam is None or o.confidence < self.min_conf:
                continue
            pos, yaw = pallet_pose_in_frame(T_map_cam, o.rvec, o.tvec)
            f = self.filters.setdefault(o.marker_id, PoseFilter(self.window, self.max_gap, self.max_jump))
            f.add(now, pos, yaw, o.confidence)
            mpos, myaw = f.mean()
            det = PalletPose()
            det.header.stamp, det.header.frame_id = msg.header.stamp, self.map_frame
            det.marker_id, det.kind, det.samples = o.marker_id, o.kind, len(f)
            det.confidence, det.distance_m = float(f.confidence()), o.distance_m
            det.pose.position.x, det.pose.position.y, det.pose.position.z = (float(v) for v in mpos)
            (det.pose.orientation.x, det.pose.orientation.y, det.pose.orientation.z,
             det.pose.orientation.w) = yaw_quaternion(myaw)
            self.det_pub.publish(det)
            if o.kind == PALLET and (self.target in (0, o.marker_id)):
                if best is None or o.distance_m < best.distance_m:
                    best = det
        if best is not None:
            ps = PoseStamped()
            ps.header, ps.pose = best.header, best.pose
            self.pose_pub.publish(ps)
            self.conf_pub.publish(Float32(data=best.confidence))


def main() -> None:
    rclpy.init()
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
