"""lift_server — forklift_interfaces/Lift action: sends the set-point on `lift_cmd_mission` (mm; the safety
supervisor forwards it to the ESP32 `lift_cmd`) and finishes when `fork_state` is within tolerance (FW-04).
Logic: lift_core.LiftTracker."""

from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32

from forklift_interfaces.action import Lift
from forklift_interfaces.msg import ForkState

from .lift_core import LiftTracker


class LiftServer(Node):
    def __init__(self) -> None:
        super().__init__("lift_server")
        self.tol = float(self.declare_parameter("tolerance_mm", 3.0).value)
        self.timeout = float(self.declare_parameter("timeout_s", 8.0).value)
        self.settle = int(self.declare_parameter("settle_samples", 3).value)
        self.lift_max = float(self.declare_parameter("lift_max_mm", 120.0).value)
        self.rate = float(self.declare_parameter("rate_hz", 20.0).value)
        cb = ReentrantCallbackGroup()
        self.fork: ForkState | None = None
        self.fork_seq = 0
        topic = self.declare_parameter("fork_state_topic", "fork_state/typed").value
        self.create_subscription(ForkState, topic, self.on_fork, qos_profile_sensor_data, callback_group=cb)
        self.pub = self.create_publisher(Float32, "lift_cmd_mission", 10)
        self.busy = False
        ActionServer(self, Lift, "lift", execute_callback=self.execute, callback_group=cb,
                     goal_callback=self.on_goal, cancel_callback=lambda g: CancelResponse.ACCEPT)

    def on_fork(self, msg: ForkState) -> None:
        if msg.height_mm != msg.height_mm:  # NaN: pot unreadable, not a usable sample
            return
        self.fork = msg
        self.fork_seq += 1

    def on_goal(self, goal) -> GoalResponse:
        return GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT

    def execute(self, gh):
        self.busy = True
        try:
            g = gh.request
            tracker = LiftTracker(g.height_mm, time.monotonic(), g.tolerance_mm or self.tol,
                                  g.timeout_s or self.timeout, self.settle, self.lift_max)
            seen = self.fork_seq
            result = Lift.Result()
            while rclpy.ok():
                self.pub.publish(Float32(data=float(tracker.target)))
                time.sleep(1.0 / self.rate)
                if gh.is_cancel_requested:
                    if self.fork is not None:  # hold where we are
                        self.pub.publish(Float32(data=float(self.fork.height_mm)))
                    gh.canceled()
                    result.message = "cancelled"
                    return result
                if self.fork is None or self.fork_seq == seen:
                    if time.monotonic() > tracker.deadline:
                        result.message = "no fork_state from the ESP32"
                        gh.abort()
                        return result
                    continue
                seen = self.fork_seq
                f = self.fork
                r = tracker.update(f.height_mm, f.limit_low, f.limit_high, time.monotonic())
                gh.publish_feedback(Lift.Feedback(current_height_mm=float(f.height_mm)))
                if r.done:
                    result.success, result.final_height_mm, result.message = r.success, float(f.height_mm), r.message
                    if r.success:
                        gh.succeed()
                    else:
                        gh.abort()
                    return result
            gh.abort()
            return result
        finally:
            self.busy = False


def main() -> None:
    rclpy.init()
    node = LiftServer()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
