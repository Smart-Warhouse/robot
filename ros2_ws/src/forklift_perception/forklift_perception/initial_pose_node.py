"""initial_pose_node — NV-04: publishes `initialpose` once from the charger marker (logic: initial_pose_core).

  in : aruco/raw (PalletPose in the camera optical frame, from aruco_node), TF base_link <- camera_optical_frame
  out: initialpose (geometry_msgs/PoseWithCovarianceStamped, frame map) — slam_toolbox localization / AMCL
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from forklift_interfaces.msg import PalletPose

from .aruco_core import transform_from, yaw_quaternion
from .initial_pose_core import InitialPoseEstimator, marker_map


class InitialPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_node")
        P = self.declare_parameter
        wh = yaml.safe_load(Path(P("warehouse_map_file", "").value).read_text(encoding="utf-8"))
        overrides_file = P("marker_overrides_file", "").value
        overrides = yaml.safe_load(Path(overrides_file).read_text()) if overrides_file else None
        markers = marker_map(wh, overrides=(overrides or {}).get("markers"))
        allowed = set(int(i) for i in P("allowed_marker_ids", [35, 36, 37, 38]).value)
        self.est = InitialPoseEstimator(markers, allowed, int(P("samples", 10).value),
                                        float(P("max_spread_m", 0.02).value),
                                        float(P("max_spread_rad", 0.035).value),
                                        float(P("max_distance_m", 1.0).value))
        self.base_frame = P("base_frame", "base_link").value
        self.map_frame = P("map_frame", "map").value
        self.cov_xy = float(P("covariance_xy", 0.0004).value)
        self.cov_yaw = float(P("covariance_yaw", 0.0012).value)
        self.done = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 1)
        self.sub = self.create_subscription(PalletPose, "aruco/raw", self.on_marker, 10)
        self.get_logger().info(f"waiting for a charger marker {sorted(allowed)}")

    def on_marker(self, msg: PalletPose) -> None:
        if self.done:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        T_base_cam = transform_from((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        p, o = msg.pose.position, msg.pose.orientation
        T_cam_marker = transform_from((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        if not self.est.add(msg.marker_id, T_cam_marker, T_base_cam):
            return
        pose = self.est.estimate()
        if pose is None:
            return
        x, y, yaw = pose
        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map_frame
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z,
         out.pose.pose.orientation.w) = yaw_quaternion(yaw)
        cov = np.zeros(36)
        cov[0] = cov[7] = self.cov_xy
        cov[35] = self.cov_yaw
        out.pose.covariance = cov.tolist()
        self.pub.publish(out)
        self.done = True
        self.get_logger().info(f"initial pose from marker {msg.marker_id}: x={x:.3f} y={y:.3f} yaw={yaw:.3f}")


def main() -> None:
    rclpy.init()
    node = InitialPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
