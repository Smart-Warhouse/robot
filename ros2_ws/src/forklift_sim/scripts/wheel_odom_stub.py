#!/usr/bin/env python3
"""Simulation stand-in for the ESP32's wheel odometry: ground_truth/odom (Gazebo OdometryPublisher, world pose) ->
wheel_odom starting at 0 where the robot was spawned, like the firmware's odometry after power-up.

Gazebo reports the absolute world pose, and the jump from the origin to the spawn pose shows up as one huge velocity
sample; the EKF integrates the velocities, so that spike moved its odom frame by the spawn offset (sim.yml run 5).
Velocity samples above the robot's physical limits are replaced by the previous one; small covariances are filled
in because Gazebo leaves them at zero.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class WheelOdomStub(Node):
    def __init__(self) -> None:
        super().__init__("wheel_odom_stub")
        self.x0 = self.declare_parameter("spawn_x", 0.0).value
        self.y0 = self.declare_parameter("spawn_y", 0.0).value
        self.yaw0 = self.declare_parameter("spawn_yaw", 0.0).value
        self.v_max = self.declare_parameter("max_speed", 1.0).value     # m/s, 3x NFR-03
        self.w_max = self.declare_parameter("max_omega", 4.0).value     # rad/s
        self.last = None
        self.pub = self.create_publisher(Odometry, "wheel_odom", 10)
        self.create_subscription(Odometry, "ground_truth/odom", self.on_odom, 10)

    def on_odom(self, gt: Odometry) -> None:
        out = Odometry()
        out.header.stamp = gt.header.stamp
        out.header.frame_id, out.child_frame_id = "odom", "base_link"
        c, s = math.cos(-self.yaw0), math.sin(-self.yaw0)
        dx, dy = gt.pose.pose.position.x - self.x0, gt.pose.pose.position.y - self.y0
        out.pose.pose.position.x, out.pose.pose.position.y = c * dx - s * dy, s * dx + c * dy
        yaw = yaw_of(gt.pose.pose.orientation) - self.yaw0
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tw = gt.twist.twist
        spike = math.hypot(tw.linear.x, tw.linear.y) > self.v_max or abs(tw.angular.z) > self.w_max
        out.twist.twist = self.last if spike and self.last is not None else (tw if not spike else out.twist.twist)
        self.last = out.twist.twist
        for i, v in ((0, 1e-4), (7, 1e-4), (35, 1e-3)):          # x, y, yaw
            out.pose.covariance[i] = v
        for i, v in ((0, 1e-4), (7, 1e-4), (14, 1e-6), (21, 1e-6), (28, 1e-6), (35, 1e-3)):
            out.twist.covariance[i] = v
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = WheelOdomStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
