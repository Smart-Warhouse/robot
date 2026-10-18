#!/usr/bin/env python3
"""fork_state (ESP32, mm) -> joint_states for `fork_lift_joint`, so robot_state_publisher moves the `fork` and
`camera_*` frames with the lift (JS-06). rclpy is imported in main() only, so the conversion is host-testable."""

from __future__ import annotations


def fork_joint_position(height_mm: float, lift_max_m: float) -> float:
    """Lift height in metres, clamped to the URDF joint limits [0, lift_max_m]."""
    return min(max(float(height_mm) / 1000.0, 0.0), float(lift_max_m))


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    from forklift_interfaces.msg import ForkState

    class ForkJointStatePublisher(Node):
        def __init__(self) -> None:
            super().__init__("fork_joint_state_publisher")
            self.joint = self.declare_parameter("joint_name", "fork_lift_joint").value
            self.lift_max = float(self.declare_parameter("lift_max_m", 0.12).value)
            self.pub = self.create_publisher(JointState, "joint_states", 10)
            from rclpy.qos import qos_profile_sensor_data

            topic = self.declare_parameter("fork_state_topic", "fork_state/typed").value
            self.create_subscription(ForkState, topic, self.on_fork, qos_profile_sensor_data)

        def on_fork(self, msg: ForkState) -> None:
            if msg.height_mm != msg.height_mm:  # NaN = unknown height
                return
            js = JointState()
            js.header.stamp = msg.header.stamp if msg.header.stamp.sec else self.get_clock().now().to_msg()
            js.name = [self.joint]
            js.position = [fork_joint_position(msg.height_mm, self.lift_max)]
            self.pub.publish(js)

    rclpy.init()
    node = ForkJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
