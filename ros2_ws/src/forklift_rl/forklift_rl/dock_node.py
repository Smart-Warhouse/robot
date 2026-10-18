"""dock_server — forklift_interfaces/Dock action: the final approach + fork insertion at 20 Hz (AI-05, AI-06).
Thin rclpy wrapper around docking_core / observation / policy.

  in : pallet_pose (PoseStamped, map; forklift_perception), fork_state/typed, odometry/filtered, tof_ring,
       safety/status, TF map -> fork (the carriage frame of forklift_description)
  out: cmd_vel_dock (Twist), lift_cmd_dock (Float32 mm) -> forklift_safety -> ESP32; perception/target_pallet;
       docking_mode (std_msgs/Bool -> ESP32, SF-03: true while a Dock goal runs, false otherwise)
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32
from tf2_ros import Buffer, TransformException, TransformListener

from forklift_interfaces.action import Dock
from forklift_interfaces.msg import ForkState, SafetyState

from .docking_core import DockingEpisode, DockInputs, EpisodeParams, PalletEstimate
from .observation import DockingConfig, steering_from_twist
from .policy import DEFAULT_PROVIDERS, ClassicalPolicy, OnnxPolicy


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DockServer(Node):
    def __init__(self) -> None:
        super().__init__("dock_server")
        P = self.declare_parameter
        base = DockingConfig()
        flat = {}
        for name in DockingConfig.__dataclass_fields__:
            flat[name] = P(f"docking.{name}", getattr(base, name)).value
        self.cfg = DockingConfig.from_flat(flat)
        self.carriage_x = float(P("carriage_x", 0.03).value)
        ep = EpisodeParams()
        self.ep_params = EpisodeParams(**{k: type(getattr(ep, k))(P(f"episode.{k}", getattr(ep, k)).value)
                                          for k in EpisodeParams.__dataclass_fields__})
        self.default_controller = P("controller", "rl").value
        self.map_frame = P("map_frame", "map").value
        self.fork_frame = P("fork_frame", "fork").value
        self.tof_idx = [int(i) for i in P("tof_lcr_indices", [1, 0, 2]).value]  # ring order: front, front_left, ...
        # the ESP32 publishes no steering angle (servo without feedback): it is recovered from wheel odometry, which
        # the firmware integrates with the commanded angle; standing still -> the last commanded angle
        fork_topic = P("fork_state_topic", "fork_state/typed").value
        model_path = P("model_path", "").value
        providers = tuple(P("providers", list(DEFAULT_PROVIDERS)).value)
        self.rl = None
        if model_path and Path(model_path).exists():
            self.rl = OnnxPolicy(model_path, providers, bool(P("fp16", True).value), P("trt_cache_dir", "").value or None)
            self.get_logger().info(f"policy {model_path} on {self.rl.providers}")
        else:
            self.get_logger().error(f"policy model '{model_path}' not found — Dock falls back to the classical controller")
        self.classical = ClassicalPolicy(self.cfg, self.carriage_x)

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pallet: PalletEstimate | None = None
        self.fork: ForkState | None = None
        self.v = self.w = 0.0
        self.tof = [math.inf] * 6
        self.safety = 0
        self.create_subscription(PoseStamped, "pallet_pose", self.on_pallet, 10, callback_group=cb)
        self.create_subscription(ForkState, fork_topic, self.on_fork, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(Odometry, "odometry/filtered", self.on_odom, 10, callback_group=cb)
        self.create_subscription(Float32MultiArray, "tof_ring", lambda m: setattr(self, "tof", list(m.data)),
                                 qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(SafetyState, "safety/status", lambda m: setattr(self, "safety", m.state), 10,
                                 callback_group=cb)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_dock", 10)
        self.lift_pub = self.create_publisher(Float32, "lift_cmd_dock", 10)
        self.target_pub = self.create_publisher(Int32, "perception/target_pallet", 1)
        # SF-03 on the ESP32: true during the final approach (the centre ToF that sees the pallet is ignored there)
        self.docking_pub = self.create_publisher(Bool, "docking_mode", 10)
        self.docking_pub.publish(Bool(data=False))
        self.busy = False
        ActionServer(self, Dock, "dock", execute_callback=self.execute, callback_group=cb,
                     goal_callback=lambda g: GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT,
                     cancel_callback=lambda g: CancelResponse.ACCEPT)

    def on_pallet(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.pallet = PalletEstimate(p.x, p.y, _yaw(msg.pose.orientation), p.z, time.monotonic())

    def on_fork(self, msg: ForkState) -> None:
        if math.isfinite(msg.height_mm):
            self.fork = msg

    def on_odom(self, msg: Odometry) -> None:
        self.v, self.w = msg.twist.twist.linear.x, msg.twist.twist.angular.z

    def _inputs(self, steer_fallback: float) -> DockInputs | None:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.fork_frame, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        f = self.fork
        lift_m = (f.height_mm / 1000.0) if f else 0.0
        steer = steering_from_twist(self.cfg, self.v, self.w, steer_fallback)
        tof = [self.tof[i] if i < len(self.tof) else math.inf for i in self.tof_idx]
        return DockInputs(time.monotonic(), (t.x, t.y), _yaw(q), self.pallet, lift_m, self.v, steer, tuple(tof),
                          (bool(f and f.tcrt_left), bool(f and f.tcrt_right)), int(self.safety))

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def execute(self, gh):
        self.busy = True
        g = gh.request
        name = g.controller or self.default_controller
        policy = self.rl if name == "rl" and self.rl is not None else self.classical
        if name == "rl" and self.rl is None:
            self.get_logger().warn("rl requested but no model loaded: using the classical controller")
        self.target_pub.publish(Int32(data=int(g.pallet_id)))
        self.pallet = None
        t0 = time.monotonic()
        ep = DockingEpisode(self.cfg, policy, g.support_height_mm / 1000.0, t0, self.ep_params)
        result = Dock.Result()
        steer_cmd = 0.0
        try:
            while rclpy.ok():
                tick = time.monotonic()
                if gh.is_cancel_requested:
                    self._stop()
                    gh.canceled()
                    result.message = "cancelled"
                    return result
                inp = self._inputs(steer_cmd)
                if inp is None:
                    if tick - t0 > self.ep_params.marker_wait_s:
                        self._stop()
                        result.message = f"no TF {self.map_frame} -> {self.fork_frame}"
                        gh.abort()
                        return result
                    time.sleep(self.cfg.dt)
                    continue
                r = ep.step(inp)
                self.docking_pub.publish(Bool(data=not r.done))
                tw = Twist()
                tw.linear.x, tw.angular.z = float(r.v), float(r.omega)
                self.cmd_pub.publish(tw)
                self.lift_pub.publish(Float32(data=float(r.lift_mm)))
                steer_cmd = float(r.action[1]) * self.cfg.steer_max
                fb = Dock.Feedback()
                fb.dx, fb.dy = float(r.obs[0] * 0.6), float(r.obs[1] * 0.15)
                fb.dyaw = float(math.atan2(r.obs[2], r.obs[3]))
                fb.marker_visible, fb.step = bool(r.obs[5] > 0.5), ep.steps
                gh.publish_feedback(fb)
                if r.done:
                    self._stop()
                    result.success, result.message = r.success, r.message
                    result.lateral_error_m = float(r.errors.get("lat_tip", 0.0))
                    result.heading_error_rad = float(r.errors.get("heading", 0.0))
                    result.duration_s = float(time.monotonic() - t0)
                    (gh.succeed if r.success else gh.abort)()
                    self.get_logger().info(f"dock {name}: {r.message} in {result.duration_s:.1f} s")
                    return result
                time.sleep(max(0.0, self.cfg.dt - (time.monotonic() - tick)))
            gh.abort()
            return result
        finally:
            self._stop()
            self.docking_pub.publish(Bool(data=False))
            self.busy = False


def main() -> None:
    rclpy.init()
    node = DockServer()
    ex = MultiThreadedExecutor(num_threads=3)
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
