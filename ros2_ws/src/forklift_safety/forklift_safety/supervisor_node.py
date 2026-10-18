"""safety_supervisor — thin rclpy wrapper around safety_core.SafetySupervisor (SF-03, SF-05..SF-08, AI-06).

  in : cmd_vel_nav, cmd_vel_dock, cmd_vel_teleop (Twist) · lift_cmd_mission, lift_cmd_dock, lift_cmd_teleop
       (Float32, mm) · tof_ring (Float32MultiArray, m, NaN = fault, best effort) · fork_state/typed (ForkState,
       from fork_state_bridge) · fleet_status · safety_state (UInt8, ESP32: state << 4 | reason)
       · person_distance (Float32, m, from the YOLO node when it exists)
  out: cmd_vel (Twist, 20 Hz -> ESP32) · lift_cmd (Float32 mm -> ESP32) · heartbeat (UInt32 10 Hz -> ESP32,
       the ESP32 watchdog FW-06 stops the robot when this node dies) · safety/status (SafetyState)
       · safety_reset (std_msgs/Empty -> ESP32, on the safety/reset service)
"""

from __future__ import annotations

import dataclasses
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty, Float32, Float32MultiArray, UInt8, UInt32

from forklift_interfaces.msg import FleetStatus, ForkState, SafetyState
from forklift_interfaces.srv import Reset

from .safety_core import SafetyParams, SafetySupervisor


class SupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("safety_supervisor")
        values = {}
        for f in dataclasses.fields(SafetyParams):
            default = getattr(SafetyParams(), f.name)
            values[f.name] = self.declare_parameter(f.name, default).value
        self.core = SafetySupervisor(SafetyParams.from_dict(values))
        rate = float(self.declare_parameter("rate_hz", 20.0).value)
        self.dt = 1.0 / rate
        now = time.monotonic

        for src in ("nav", "dock", "teleop"):
            self.create_subscription(Twist, f"cmd_vel_{src}",
                                     lambda m, s=src: self.core.set_cmd(s, m.linear.x, m.angular.z, now()), 10)
        for src in ("mission", "dock", "teleop"):
            self.create_subscription(Float32, f"lift_cmd_{src}",
                                     lambda m, s=src: self.core.set_lift(s, m.data, now()), 10)
        fork_topic = self.declare_parameter("fork_state_topic", "fork_state/typed").value
        # ESP32 publishes tof_ring / fork_state best effort: subscribe with the sensor-data QoS or they never match
        self.create_subscription(Float32MultiArray, "tof_ring", lambda m: self.core.set_tof(list(m.data), now()),
                                 qos_profile_sensor_data)
        self.create_subscription(ForkState, fork_topic, lambda m: self.core.set_fork(m.height_mm),
                                 qos_profile_sensor_data)
        self.create_subscription(UInt8, "safety_state", lambda m: self.core.set_esp32(m.data), 10)
        self.create_subscription(Float32, "person_distance", lambda m: self.core.set_person(m.data, now()), 10)
        self.create_subscription(FleetStatus, "fleet_status", lambda m: self.core.set_fleet(
            now(), m.mode, m.fleet_estop, m.server_connected, m.server_silence_s, m.step_active), 10)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.lift_pub = self.create_publisher(Float32, "lift_cmd", 10)
        self.hb_pub = self.create_publisher(UInt32, "heartbeat", 10)
        self.status_pub = self.create_publisher(SafetyState, "safety/status", 10)
        self.reset_pub = self.create_publisher(Empty, "safety_reset", 10)
        self.create_service(Reset, "safety/reset", self.on_reset)
        self.hb = 0
        self.last_state = (-1, "")
        self.last_status_t = 0.0
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.heartbeat)

    def on_reset(self, req: Reset.Request, res: Reset.Response) -> Reset.Response:
        res.success = self.core.reset()
        self.reset_pub.publish(Empty())
        res.message = f"reset ({req.reason})"
        self.get_logger().warn(f"safety reset: {req.reason}")
        return res

    def heartbeat(self) -> None:
        self.hb = (self.hb + 1) & 0xFFFFFFFF
        self.hb_pub.publish(UInt32(data=self.hb))

    def step(self) -> None:
        t = time.monotonic()
        o = self.core.step(t, self.dt)
        tw = Twist()
        tw.linear.x, tw.angular.z = float(o.v), float(o.w)
        self.cmd_pub.publish(tw)
        if o.lift_mm is not None:
            self.lift_pub.publish(Float32(data=float(o.lift_mm)))
        key = (o.state, o.reason)
        if key != self.last_state or t - self.last_status_t >= 0.1:
            if key != self.last_state and o.state > 1:
                self.get_logger().warn(f"safety {o.state_name}: {o.reason} (source {o.source})")
            self.last_state, self.last_status_t = key, t
            m = SafetyState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.state, m.reason, m.source = int(o.state), o.reason, o.source
            m.speed_limit, m.docking, m.forks_raised = float(o.speed_limit), o.docking, o.forks_raised
            m.min_tof_m = float(min(o.min_tof_m, 99.0))
            m.esp32_state = int(self.core.esp[2])
            self.status_pub.publish(m)


def main() -> None:
    rclpy.init()
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # leave the robot with a zero command
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
