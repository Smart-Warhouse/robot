"""Twin navigation test (headless): fl1 with the real Nav2 configuration of forklift_navigation (Smac Hybrid-A* +
Regulated Pure Pursuit, costmap filters, EKF), localisation = generated map + ground-truth map -> odom, and a
NavigateToPose goal 1.5 m down the bottom corridor; asserts the goal is reached within tolerance and records a
rosbag (SIM_ARTIFACTS/nav_bag) that CI uploads.
"""

import math
import os
import shutil
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
import rclpy.time
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

ARTIFACTS = os.environ.get("SIM_ARTIFACTS", "/tmp/sim_artifacts")
BAG = os.path.join(ARTIFACTS, "nav_bag")
GOAL = (2.5, 0.6, 0.0)          # map frame; start J1 = (1.0, 0.6, 0.0)
XY_TOL, YAW_TOL = 0.10, 0.15    # Nav2 goal checker: 5 cm / 5 deg (NV-11); margin for the EKF vs ground truth
TOPICS = ["/clock", "/fl1/scan", "/fl1/scan_filtered", "/fl1/wheel_odom", "/fl1/ground_truth/odom", "/fl1/odometry/filtered", "/fl1/imu/data",
          "/fl1/cmd_vel", "/fl1/cmd_vel_nav", "/fl1/tf", "/fl1/tf_static", "/fl1/plan", "/fl1/joint_states"]


@pytest.mark.launch_test
def generate_test_description():
    shutil.rmtree(BAG, ignore_errors=True)
    os.makedirs(ARTIFACTS, exist_ok=True)
    sim = os.path.join(get_package_share_directory("forklift_sim"), "launch", "sim.launch.py")
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim),
                                 launch_arguments={"robots": "fl1", "stack": "nav", "camera_rate": "5"}.items()),
        TimerAction(period=5.0, actions=[ExecuteProcess(cmd=["ros2", "bag", "record", "-o", BAG] + TOPICS,
                                                        output="screen", name="rosbag")]),
        launch_testing.actions.ReadyToTest(),
    ])


def yaw_of(o: Odometry) -> float:
    q = o.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TestNav(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("twin_nav_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
        cls.odom = []
        cls.node.create_subscription(Odometry, "/fl1/ground_truth/odom", cls.odom.append, qos_profile_sensor_data)
        cls.nav = ActionClient(cls.node, NavigateToPose, "/fl1/navigate_to_pose")
        cls.costmap, cls.ekf = [], []
        cls.node.create_subscription(Odometry, "/fl1/odometry/filtered", cls.ekf.append, 10)
        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.tf = Buffer()   # filled from the namespaced TF topics (a TransformListener would read /tf)
        cls.node.create_subscription(TFMessage, "/fl1/tf",
                                     lambda m: [cls.tf.set_transform(t, "sim") for t in m.transforms], 50)
        cls.node.create_subscription(TFMessage, "/fl1/tf_static",
                                     lambda m: [cls.tf.set_transform_static(t, "sim") for t in m.transforms], latched)
        cls.node.create_subscription(OccupancyGrid, "/fl1/global_costmap/costmap", cls.costmap.append, latched)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait(self, future, timeout):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return future.result() if future.done() else None

    def cost(self, x, y):
        g = self.costmap[-1]
        i = int((x - g.info.origin.position.x) / g.info.resolution)
        j = int((y - g.info.origin.position.y) / g.info.resolution)
        return g.data[j * g.info.width + i] if 0 <= i < g.info.width and 0 <= j < g.info.height else None

    def diagnose(self):
        """What the planner sees at the start: EKF pose, TF map -> base_link and the costmap under the footprint."""
        if self.ekf:
            p = self.ekf[-1].pose.pose.position
            print(f"odometry/filtered: ({p.x:.3f}, {p.y:.3f}) yaw {yaw_of(self.ekf[-1]):.3f}")
        else:
            print("odometry/filtered: nothing received")
        try:
            t = self.tf.lookup_transform("map", "base_link", rclpy.time.Time())
            print(f"TF map->base_link: ({t.transform.translation.x:.3f}, {t.transform.translation.y:.3f}) "
                  f"qz {t.transform.rotation.z:.3f}")
        except Exception as e:  # noqa: BLE001 - diagnostics only
            print("TF map->base_link failed:", e)
        if self.costmap:
            worst = max((self.cost(1.0 + dx / 100, 0.6 + dy / 100) or 0, dx, dy)
                        for dx in range(-32, 24, 2) for dy in range(-12, 13, 2))
            print("max global cost in a 0.56 x 0.24 m box around the start footprint:", worst)

    def test_navigate_to_pose(self):
        self.assertTrue(self.nav.wait_for_server(timeout_sec=180.0), "navigate_to_pose action server missing")
        # the planner rejects a start/goal on unknown cells (allow_unknown: false): wait for the static map
        end = time.monotonic() + 120.0
        while time.monotonic() < end and not (self.costmap and self.cost(1.0, 0.6) == 0 and self.cost(*GOAL[:2]) == 0):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self.costmap, "no global costmap")
        self.diagnose()
        print("global costmap start/goal cost:", self.cost(1.0, 0.6), self.cost(*GOAL[:2]),
              "size", self.costmap[-1].info.width, self.costmap[-1].info.height)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = GOAL[0], GOAL[1]
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(GOAL[2] / 2), math.cos(GOAL[2] / 2)
        handle, end = None, time.monotonic() + 180.0
        while time.monotonic() < end:     # bt_navigator rejects goals until the lifecycle manager activated Nav2
            handle = self.wait(self.nav.send_goal_async(goal), 30.0)
            if handle is not None and handle.accepted:
                break
            pause = time.monotonic() + 3.0
            while time.monotonic() < pause:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(handle is not None and handle.accepted, "NavigateToPose goal never accepted")
        result = self.wait(handle.get_result_async(), 400.0)
        if result is not None and result.status == GoalStatus.STATUS_ABORTED:
            self.diagnose()   # one retry, as mission_node does
            print("first NavigateToPose aborted, retrying once")
            handle = self.wait(self.nav.send_goal_async(goal), 30.0)
            result = self.wait(handle.get_result_async(), 400.0) if handle and handle.accepted else None
        self.assertIsNotNone(result, "NavigateToPose did not finish in time")
        o = self.odom[-1]
        ex, ey = o.pose.pose.position.x - GOAL[0], o.pose.pose.position.y - GOAL[1]
        eyaw = math.atan2(math.sin(yaw_of(o) - GOAL[2]), math.cos(yaw_of(o) - GOAL[2]))
        print(f"nav result status={result.status} ground-truth error: xy={math.hypot(ex, ey):.3f} m yaw={eyaw:.3f}")
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertLess(math.hypot(ex, ey), XY_TOL)
        self.assertLess(abs(eyaw), YAW_TOL)
