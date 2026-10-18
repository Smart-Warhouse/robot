"""fork_state_bridge — ESP32 `fork_state` (Float32MultiArray, best effort) -> forklift_interfaces/ForkState on
`fork_state/typed` for every Jetson node. Remove once the firmware publishes ForkState itself (ForkState.msg)."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

from forklift_interfaces.msg import ForkState

from .fork_state import parse_fork_array


class ForkStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("fork_state_bridge")
        self.pub = self.create_publisher(ForkState, "fork_state/typed", qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "fork_state", self.on_array, qos_profile_sensor_data)

    def on_array(self, msg: Float32MultiArray) -> None:
        r = parse_fork_array(msg.data)
        if r is None:
            self.get_logger().warn(f"fork_state has {len(msg.data)} elements, expected 6", throttle_duration_sec=5)
            return
        out = ForkState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.height_mm = r.height_mm
        out.limit_high, out.limit_low = r.limit_high, r.limit_low
        out.tcrt_left, out.tcrt_right, out.at_target = r.tcrt_left, r.tcrt_right, r.at_target
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ForkStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
