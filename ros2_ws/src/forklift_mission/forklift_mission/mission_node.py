"""mission_node — thin rclpy wrapper around FleetClient + MissionExecutor (all logic lives in the *_core modules).

Per robot namespace (/fl1 ...):
  action clients  navigate_to_pose (Nav2), drive_on_heading + backup (Nav2 behavior_server), dock (forklift_rl),
                  lift (lift_server)
  service client  safety/reset (forklift_safety)
  subscriptions   odometry/filtered, battery, fork_state/typed (ForkState), safety/status; TF map -> base_link
  publishers      fleet_status (FleetStatus), cmd_vel_teleop (Twist), lift_cmd_teleop (Float32, mm),
                  local_costmap/footprint + global_costmap/footprint (Polygon, NV-12)
  action server   move_task (forklift_interfaces/MoveTask) — run a task locally without the server
MQTT: paho via ThreadSafeTransport; callbacks are drained in the tick timer, so the core is single-threaded.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point32, Polygon, PoseStamped, Twist
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from forklift_interfaces.action import Dock, Lift, MoveTask
from forklift_interfaces.msg import FleetStatus, ForkState, SafetyState
from forklift_interfaces.srv import Reset

from .bt import DoneHandle, Status
from .fleet_client_core import FleetClient, FleetConfig
from .footprint import footprint
from .lane_graph import LaneGraph
from .mission_core import MissionExecutor, MissionParams
from .ros_helpers import (battery_percent, lift_teleop_target, quaternion_to_yaw, safety_event, safety_name,
                          yaw_to_quaternion)
from .transport import PahoTransport, ThreadSafeTransport


class ActionHandle:
    """bt.Handle over an rclpy action goal."""

    def __init__(self, client: ActionClient, goal, name: str, check_result=None) -> None:
        self.name = name
        self.message = ""
        self._check = check_result
        self._goal_handle = None
        self._result_future = None
        self._done: Status | None = None
        self._cancel_requested = False
        if not client.server_is_ready():
            self._done, self.message = Status.FAILURE, f"{name} server not available"
            return
        self._send_future = client.send_goal_async(goal)
        self._send_future.add_done_callback(self._on_goal)

    def _on_goal(self, fut) -> None:
        gh = fut.result()
        if not gh.accepted:
            self._done, self.message = Status.FAILURE, f"{self.name} goal rejected"
            return
        self._goal_handle = gh
        if self._cancel_requested:
            gh.cancel_goal_async()
        self._result_future = gh.get_result_async()

    def status(self) -> Status:
        if self._done is not None:
            return self._done
        if self._result_future is None or not self._result_future.done():
            return Status.RUNNING
        res = self._result_future.result()
        ok = res.status == GoalStatus.STATUS_SUCCEEDED
        if ok and self._check is not None:
            ok, self.message = self._check(res.result)
        elif not ok:
            self.message = f"{self.name} ended with status {res.status}"
            msg = getattr(res.result, "message", "")
            if msg:
                self.message += f" ({msg})"
        self._done = Status.SUCCESS if ok else Status.FAILURE
        return self._done

    def cancel(self) -> None:
        self._cancel_requested = True
        if self._goal_handle is not None and self._done is None:
            self._goal_handle.cancel_goal_async()


class RosCapabilities:
    def __init__(self, node: "MissionNode") -> None:
        self.n = node
        cb = node.cb
        self.nav = ActionClient(node, NavigateToPose, "navigate_to_pose", callback_group=cb)
        self.drive = ActionClient(node, DriveOnHeading, "drive_on_heading", callback_group=cb)
        self.backup = ActionClient(node, BackUp, "backup", callback_group=cb)
        self.dock_client = ActionClient(node, Dock, "dock", callback_group=cb)
        self.lift_client = ActionClient(node, Lift, "lift", callback_group=cb)
        self.reset_client = node.create_client(Reset, "safety/reset", callback_group=cb)
        self.handles: list[ActionHandle] = []

    def _track(self, h: ActionHandle) -> ActionHandle:
        self.handles = [x for x in self.handles if x._done is None] + [h]
        return h

    def navigate(self, x: float, y: float, yaw: float):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = self.n.map_frame
        g.pose.header.stamp = self.n.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        q = yaw_to_quaternion(yaw)
        o = g.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        return self._track(ActionHandle(self.nav, g, "navigate_to_pose"))

    def dock(self, pallet_id: int, support_height_mm: float):
        g = Dock.Goal()
        g.pallet_id, g.support_height_mm, g.controller = int(pallet_id), float(support_height_mm), ""
        return self._track(ActionHandle(self.dock_client, g, "dock", lambda r: (r.success, r.message)))

    def lift(self, height_mm: float):
        g = Lift.Goal()
        g.height_mm = float(height_mm)
        return self._track(ActionHandle(self.lift_client, g, "lift", lambda r: (r.success, r.message)))

    def drive_straight(self, distance_m: float):
        speed = self.n.straight_speed
        allowance = Duration(sec=int(abs(distance_m) / speed * 3 + 5))
        if distance_m >= 0:
            g = DriveOnHeading.Goal()
            client, name = self.drive, "drive_on_heading"
        else:
            g = BackUp.Goal()
            client, name = self.backup, "backup"
        g.target.x = abs(float(distance_m))  # BackUp drives backwards for a positive target
        g.speed = speed
        g.time_allowance = allowance
        if abs(distance_m) < 1e-4:
            return DoneHandle(True)
        return self._track(ActionHandle(client, g, name))

    def stop(self) -> None:
        for h in self.handles:
            h.cancel()

    def set_loaded(self, loaded: bool) -> None:
        self.n.publish_footprint(loaded)

    def reset_safety(self) -> None:
        if self.reset_client.service_is_ready():
            self.reset_client.call_async(Reset.Request(reason="operator reset (fleet cmd)"))

    def telemetry(self) -> dict:
        return self.n.telemetry()


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_node")
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        P = self.declare_parameter
        self.robot_id = P("robot_id", "fl1").value
        map_file = P("warehouse_map_file", "").value
        self.map_frame = P("map_frame", "map").value
        self.base_frame = P("base_frame", "base_link").value
        tick_hz = float(P("tick_hz", 10.0).value)
        self.straight_speed = float(P("straight_speed_mps", 0.05).value)
        self.teleop_lift_rate = float(P("teleop_lift_rate_mm_s", 30.0).value)
        self.lift_max_mm = float(P("lift_max_mm", 120.0).value)
        dims_file = P("dimensions_file", "").value
        self.footprint_padding = float(P("footprint_padding_m", 0.0).value)
        pw_file = P("mqtt.password_file", "").value
        cfg = FleetConfig(
            robot_id=self.robot_id, host=P("mqtt.host", "localhost").value, port=int(P("mqtt.port", 1883).value),
            username=P("mqtt.username", "").value or None,
            password=Path(pw_file).read_text().strip() if pw_file and Path(pw_file).exists() else None,
            keepalive_s=int(P("mqtt.keepalive_s", 2).value), state_hz=float(P("state_hz", 2.0).value),
            grant_retry_s=float(P("grant_retry_s", 0.5).value),
            server_loss_timeout_s=float(P("server_loss_timeout_s", 3.0).value),
            teleop_timeout_s=float(P("teleop_timeout_s", 0.5).value),
        )
        if pw_file and cfg.password is None:
            self.get_logger().error(f"MQTT password file {pw_file} not found — connecting without a password")
        self.graph = LaneGraph.load(map_file)
        home = P("home_node", "").value or self.graph.robots.get(self.robot_id, {}).get("home", "")
        mp = MissionParams(
            robot_id=self.robot_id, home_node=home,
            nav_attempts=int(P("nav_attempts", 3).value), recovery_wait_s=float(P("recovery_wait_s", 2.0).value),
            recovery_backup_m=float(P("recovery_backup_m", 0.05).value),
            dock_attempts=int(P("dock_attempts", 2).value), entry_offset_mm=float(P("entry_offset_mm", 8.0).value),
            lift_off_mm=float(P("lift_off_mm", 20.0).value),
            travel_loaded_mm=float(P("travel_loaded_mm", 20.0).value),
            travel_empty_mm=float(P("travel_empty_mm", 0.0).value), backout_m=float(P("backout_m", 0.25).value),
            place_m=float(P("place_m", 0.25).value),
            require_reset_after_server_loss=bool(P("require_reset_after_server_loss", False).value),
        )
        self.dims = yaml.safe_load(Path(dims_file).read_text()) if dims_file else None

        # robot state from topics / TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose = (0.0, 0.0, 0.0)
        self.speed = 0.0
        self.battery = 0.0
        self.fork_mm = 0.0
        self.safety = SafetyState.NORMAL
        self.lift_teleop_target = 0.0
        self.create_subscription(Odometry, "odometry/filtered", self.on_odom, 10, callback_group=self.cb)
        self.create_subscription(BatteryState, "battery", self.on_battery, 10, callback_group=self.cb)
        self.create_subscription(ForkState, P("fork_state_topic", "fork_state/typed").value, self.on_fork,
                                 qos_profile_sensor_data, callback_group=self.cb)
        self.create_subscription(SafetyState, "safety/status", self.on_safety, 10, callback_group=self.cb)
        self.status_pub = self.create_publisher(FleetStatus, "fleet_status", 10)
        self.teleop_pub = self.create_publisher(Twist, "cmd_vel_teleop", 10)
        self.lift_teleop_pub = self.create_publisher(Float32, "lift_cmd_teleop", 10)
        self.fp_pubs = [self.create_publisher(Polygon, f"{c}/footprint", 1) for c in ("local_costmap", "global_costmap")]

        self.transport = ThreadSafeTransport(PahoTransport())
        self.client = FleetClient(cfg, self.transport, clock=time.monotonic)
        self.caps = RosCapabilities(self)
        self.ex = MissionExecutor(mp, self.graph, self.client, self.caps, time.monotonic)
        self.client.start()

        self.dt = 1.0 / tick_hz
        self.create_timer(self.dt, self.tick, callback_group=self.cb)
        self.create_timer(0.2, self.publish_status, callback_group=self.cb)
        ActionServer(self, MoveTask, "move_task", execute_callback=self.execute_move_task,
                     goal_callback=lambda g: GoalResponse.ACCEPT, cancel_callback=lambda g: CancelResponse.ACCEPT,
                     callback_group=self.cb)
        self.get_logger().info(f"{self.robot_id}: fleet client -> {cfg.host}:{cfg.port}, home {home}")

    # ------------------------------------------------------------------ inputs
    def on_odom(self, msg: Odometry) -> None:
        self.speed = msg.twist.twist.linear.x

    def on_battery(self, msg: BatteryState) -> None:
        self.battery = battery_percent(msg.percentage, msg.voltage)

    def on_fork(self, msg: ForkState) -> None:
        if math.isfinite(msg.height_mm):  # NaN = pot unreadable: keep the last good value for reporting
            self.fork_mm = msg.height_mm

    def on_safety(self, msg: SafetyState) -> None:
        with self.lock:
            ev = safety_event(self.safety, msg.state, msg.reason)
            self.safety = msg.state
            if ev:
                severity, code, text = ev
                self.client.event("safety", severity, code, text)

    def telemetry(self) -> dict:
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            tr, q = t.transform.translation, t.transform.rotation
            self.pose = (tr.x, tr.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))
        except TransformException:
            pass
        x, y, yaw = self.pose
        return {"x": x, "y": y, "yaw": yaw, "speed": self.speed, "battery_pct": self.battery,
                "fork_height_mm": self.fork_mm, "safety_state": safety_name(self.safety)}

    # ------------------------------------------------------------------ outputs
    def publish_footprint(self, loaded: bool) -> None:
        if not self.dims:
            return
        poly = Polygon(points=[Point32(x=float(x), y=float(y), z=0.0)
                               for x, y in footprint(self.dims, loaded, self.footprint_padding)])
        for p in self.fp_pubs:
            p.publish(poly)

    def publish_status(self) -> None:
        with self.lock:
            st = self.ex.status()
        msg = FleetStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        for k, v in st.items():
            setattr(msg, k, type(getattr(msg, k))(v))
        self.status_pub.publish(msg)

    def tick(self) -> None:
        with self.lock:
            self.transport.drain()
            self.ex.tick()
            v, w, lift = self.ex.teleop_output()
            manual = self.ex.mode == "manual"
        if manual:
            tw = Twist()
            tw.linear.x, tw.angular.z = float(v), float(w)
            self.teleop_pub.publish(tw)
            if lift:
                self.lift_teleop_target = lift_teleop_target(self.lift_teleop_target, lift, self.teleop_lift_rate,
                                                             self.dt, self.lift_max_mm)
                self.lift_teleop_pub.publish(Float32(data=float(self.lift_teleop_target)))
        else:
            self.lift_teleop_target = self.fork_mm

    # ------------------------------------------------------------------ local task (bench tests)
    def execute_move_task(self, goal_handle):
        g = goal_handle.request
        task = {
            "task_id": g.task_id or f"LOCAL-{int(time.time())}", "type": "move_pallet", "pallet_id": g.pallet_id,
            "from": {"location": g.from_location, "node": g.from_node or None, "x": g.from_x, "y": g.from_y,
                     "yaw": g.from_yaw, "height_mm": g.from_height_mm},
            "to": {"location": g.to_location, "node": g.to_node or None, "x": g.to_x, "y": g.to_y, "yaw": g.to_yaw,
                   "height_mm": g.to_height_mm},
            "route": list(g.route), "priority": 1,
        }
        with self.lock:
            self.ex.on_task(task)
        tid = task["task_id"]
        result = MoveTask.Result()
        while rclpy.ok():
            time.sleep(0.2)
            with self.lock:
                phase, current = self.ex.phase, self.ex.task_id
                node = self.ex.node or ""
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.ex.on_cmd({"cmd": "cancel", "task_id": tid})
                goal_handle.canceled()
                result.status, result.reason = "failed", "cancelled"
                return result
            fb = MoveTask.Feedback(phase=phase, node=node)
            goal_handle.publish_feedback(fb)
            if current != tid:
                break
        result.success = phase == "done"
        result.status = "done" if result.success else "failed"
        result.reason = "" if result.success else "see wh/fl/<id>/task_status / events"
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def shutdown(self) -> None:
        self.client.stop()


def main() -> None:
    rclpy.init()
    node = MissionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

