"""Twin smoke test (headless): spawn fl1, check the bridged sensor topics publish at their rates (in sim time) and
see the world, then drive it with scripted cmd_vel and compare the odometry with the commanded motion.
Run by `colcon test --packages-select forklift_sim` when built with -DFORKLIFT_SIM_TESTS=ON (.github/workflows/sim.yml).
"""

import json
import math
import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan

ARTIFACTS = os.environ.get("SIM_ARTIFACTS", "/tmp/sim_artifacts")
WALL_TIMEOUT = 240.0
# topic, type, expected Hz, accepted band (sim time); the camera is 15 Hz here (real 30 Hz, JS-03)
RATES = [("/fl1/scan", LaserScan, 10.0, (8.0, 12.0)), ("/fl1/wheel_odom", Odometry, 50.0, (40.0, 60.0)),
         ("/fl1/ground_truth/odom", Odometry, 50.0, (40.0, 60.0)),
         ("/fl1/imu/data", Imu, 100.0, (80.0, 120.0)), ("/fl1/image_raw", Image, 15.0, (5.0, 18.0))]


@pytest.mark.launch_test
def generate_test_description():
    sim = os.path.join(get_package_share_directory("forklift_sim"), "launch", "sim.launch.py")
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim), launch_arguments={"robots": "fl1"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(o: Odometry) -> float:
    q = o.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TestTwin(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("twin_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
        cls.msgs = {t: [] for t, *_ in RATES}
        cls.msgs["/fl1/camera_info"] = []
        cls.msgs["/fl1/joint_states"] = []
        for topic, typ, *_ in RATES + [("/fl1/camera_info", CameraInfo), ("/fl1/joint_states", JointState)]:
            cls.node.create_subscription(typ, topic, lambda m, t=topic: cls.msgs[t].append(m), qos_profile_sensor_data)
        cls.cmd = cls.node.create_publisher(Twist, "/fl1/cmd_vel", 10)
        os.makedirs(ARTIFACTS, exist_ok=True)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, cond, timeout=WALL_TIMEOUT) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if cond():
                return True
        return False

    def sim_now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def test_1_sensor_rates(self):
        # wait for 6 s of sim time on every topic, then measure the last 5 s of header stamps
        ok = self.spin_until(lambda: all(len(self.msgs[t]) > 2 and stamp(self.msgs[t][-1]) - stamp(self.msgs[t][0])
                                         > 6.0 for t, *_ in RATES))
        counts = {t: len(self.msgs[t]) for t, *_ in RATES}
        self.assertTrue(ok, f"sensor topics did not all publish for 6 s of sim time: {counts}")
        rates = {}
        for topic, _, _, (lo, hi) in RATES:
            st = [stamp(m) for m in self.msgs[topic]]
            win = [s for s in st if s >= st[-1] - 5.0]
            rates[topic] = (len(win) - 1) / (win[-1] - win[0])
        with open(os.path.join(ARTIFACTS, "sensor_rates.json"), "w", encoding="utf-8") as f:
            json.dump(rates, f, indent=2)
        print("sensor rates (sim time):", rates)
        for topic, _, _, (lo, hi) in RATES:
            self.assertTrue(lo <= rates[topic] <= hi, f"{topic}: {rates[topic]:.1f} Hz not in [{lo}, {hi}]")
        scan = self.msgs["/fl1/scan"][-1]
        self.assertEqual(scan.header.frame_id, "laser")
        self.assertEqual(self.msgs["/fl1/image_raw"][-1].width, 640)
        self.assertEqual(self.msgs["/fl1/image_raw"][-1].height, 480)
        self.assertEqual(self.msgs["/fl1/image_raw"][-1].header.frame_id, "camera_optical_frame")
        self.assertTrue(self.msgs["/fl1/camera_info"], "no camera_info")
        self.assertTrue(any("fork_lift_joint" in m.name for m in self.msgs["/fl1/joint_states"]), "no fork joint")
        odom = self.msgs["/fl1/wheel_odom"][-1]
        self.assertEqual((odom.header.frame_id, odom.child_frame_id), ("odom", "base_link"))
        self.assertLess(math.hypot(odom.pose.pose.position.x, odom.pose.pose.position.y), 0.01)  # starts at 0
        odom = self.msgs["/fl1/ground_truth/odom"][-1]
        # spawned at J1 (1.0, 0.6) heading +x; the LiDAR (0.10 m behind base_link) sees the south wall 0.6 m to its
        # right and the east wall straight ahead
        self.assertAlmostEqual(odom.pose.pose.position.x, 1.0, delta=0.03)
        self.assertAlmostEqual(odom.pose.pose.position.y, 0.6, delta=0.03)

        def ray(angle):
            i = round((angle - scan.angle_min) / scan.angle_increment)
            return scan.ranges[i]
        self.assertAlmostEqual(ray(-math.pi / 2), 0.6, delta=0.06)
        self.assertAlmostEqual(ray(0.0), 6.0 - 0.9, delta=0.06)

    def drive(self, v: float, w: float, seconds: float) -> None:
        t0 = self.sim_now()
        while self.sim_now() - t0 < seconds:
            m = Twist()
            m.linear.x, m.angular.z = v, w
            self.cmd.publish(m)
            self.spin_until(lambda: False, timeout=0.05)
        for _ in range(10):
            self.cmd.publish(Twist())
            self.spin_until(lambda: False, timeout=0.05)
        t1 = self.sim_now()
        self.spin_until(lambda: self.sim_now() - t1 > 1.0, timeout=60)   # settle

    def test_2_scripted_drive(self):
        self.assertTrue(self.spin_until(lambda: self.msgs["/fl1/ground_truth/odom"] and self.sim_now() > 0.0))
        gt = "/fl1/ground_truth/odom"
        a, wa = self.msgs[gt][-1], self.msgs["/fl1/wheel_odom"][-1]
        self.drive(0.2, 0.0, 5.0)                                     # 1.0 m straight along the bottom corridor
        b, wb = self.msgs[gt][-1], self.msgs["/fl1/wheel_odom"][-1]
        self.assertAlmostEqual(wb.pose.pose.position.x - wa.pose.pose.position.x, 1.0, delta=0.1)   # odom agrees
        dx, dy = b.pose.pose.position.x - a.pose.pose.position.x, b.pose.pose.position.y - a.pose.pose.position.y
        print(f"straight: dx={dx:.3f} dy={dy:.3f} dyaw={yaw_of(b) - yaw_of(a):.3f}")
        self.assertAlmostEqual(dx, 1.0, delta=0.1)
        self.assertLess(abs(dy), 0.05)
        self.assertLess(abs(yaw_of(b) - yaw_of(a)), 0.05)
        self.drive(0.1, 0.3, 3.0)                                     # left arc, 0.9 rad
        c = self.msgs[gt][-1]
        dyaw = math.atan2(math.sin(yaw_of(c) - yaw_of(b)), math.cos(yaw_of(c) - yaw_of(b)))
        print(f"arc: dyaw={dyaw:.3f}")
        self.assertAlmostEqual(dyaw, 0.9, delta=0.1)
        self.assertLess(abs(c.twist.twist.linear.x), 0.02, "robot still moving after zero cmd_vel")
