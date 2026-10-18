#!/usr/bin/env python3
"""Simulation stand-in for the safety supervisor's output stage: cmd_vel_nav (Nav2) -> cmd_vel at 20 Hz.

Gazebo's VelocityControl keeps the last command forever, so like the ESP32's cmd_vel watchdog this publishes zero
when Nav2 has been silent for `timeout_s`. No ToF / fleet / fork checks: the real supervisor (forklift_safety) needs
the ESP32 ToF ring and the fleet server's fleet_status, which the twin does not simulate yet (docs/19-gazebo-sim.md).
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self) -> None:
        super().__init__("supervisor_bypass")
        self.timeout = self.declare_parameter("timeout_s", 0.5).value
        self.last = Twist()
        self.t = None
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Twist, "cmd_vel_nav", self.on_cmd, 10)
        self.create_timer(0.05, self.tick)

    def on_cmd(self, msg: Twist) -> None:
        self.last, self.t = msg, self.get_clock().now()

    def tick(self) -> None:
        fresh = self.t is not None and (self.get_clock().now() - self.t).nanoseconds * 1e-9 <= self.timeout
        self.pub.publish(self.last if fresh else Twist())


def main() -> None:
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
