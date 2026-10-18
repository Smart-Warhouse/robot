"""NV-04: robot pose on the map from the charger marker, end to end through a rendered image."""

import math

import cv2
import numpy as np
import pytest
import yaml

from forklift_perception.aruco_core import ArucoEstimator
from forklift_perception.initial_pose_core import InitialPoseEstimator, marker_map, robot_pose_from_marker
from forklift_perception.synthetic import camera_matrix, render

K = camera_matrix()


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def T(R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


# base_link -> camera_optical_frame as in forklift.urdf.xacro (carriage, camera pitched 20 deg down)
T_BASE_CAM = T(t=(0.035, 0, 0.054)) @ T(rpy(0, math.radians(20), 0)) @ T(rpy(-math.pi / 2, 0, -math.pi / 2))


@pytest.fixture(scope="module")
def markers(warehouse_map_path):
    return marker_map(yaml.safe_load(warehouse_map_path.read_text(encoding="utf-8")))


def test_marker_map_from_warehouse(markers):
    m = markers[35]  # CHARGE-1 at CH1 (1.0, 3.8), robot faces +y -> marker on the wall 0.2 m ahead, facing -y
    assert (m.x, m.y) == pytest.approx((1.0, 4.0)) and m.normal_yaw == pytest.approx(-math.pi / 2, abs=1e-3)
    assert markers[23].x == pytest.approx(2.65 - 0.15)  # R1-E1 faces west (yaw pi): marker on the rack face
    assert set(range(21, 39)) <= set(markers)


@pytest.mark.parametrize("pose", [(1.0, 3.8, math.pi / 2), (1.02, 3.79, math.pi / 2 - 0.04),
                                  (0.985, 3.81, math.pi / 2 + 0.05)])
def test_initial_pose_from_rendered_charger_marker(markers, pose):
    x, y, yaw = pose
    T_map_base = T(rpy(0, 0, yaw), (x, y, 0))
    T_cam_marker = np.linalg.inv(T_map_base @ T_BASE_CAM) @ markers[35].T_map_marker()
    rvec = cv2.Rodrigues(T_cam_marker[:3, :3])[0].reshape(3)
    img = render(35, 0.06, rvec, T_cam_marker[:3, 3], K, seed=3)
    est = InitialPoseEstimator(markers, allowed_ids={35, 36, 37, 38}, samples=5)
    det = ArucoEstimator()
    for _ in range(5):
        o = det.estimate(img, K)[0]
        assert est.add(o.marker_id, o.T_cam_marker(), T_BASE_CAM)
    ex, ey, eyaw = est.estimate()
    assert math.hypot(ex - x, ey - y) <= 0.01                      # NFR-01 budget is 3 cm
    assert abs(math.degrees(eyaw - yaw)) <= 1.0


def test_exact_geometry_round_trip(markers):
    T_map_base = T(rpy(0, 0, 1.2), (2.0, 3.0, 0))
    T_cam_marker = np.linalg.inv(T_map_base @ T_BASE_CAM) @ markers[38].T_map_marker()
    x, y, yaw = robot_pose_from_marker(markers[38].T_map_marker(), T_cam_marker, T_BASE_CAM)
    assert (x, y, yaw) == pytest.approx((2.0, 3.0, 1.2))


def test_rejects_non_charger_far_and_inconsistent(markers):
    est = InitialPoseEstimator(markers, allowed_ids={35}, samples=3, max_distance_m=1.0)
    near = T(t=(0, 0, 0.3))
    assert not est.add(23, near, T_BASE_CAM)            # not a charger marker
    assert not est.add(35, T(t=(0, 0, 2.5)), T_BASE_CAM)  # too far to trust
    assert est.add(35, near, T_BASE_CAM) and est.estimate() is None  # not enough samples yet
    est.add(35, T(t=(0.2, 0, 0.3)), T_BASE_CAM)
    est.add(35, near, T_BASE_CAM)
    assert est.estimate() is None                       # 20 cm disagreement -> no pose
