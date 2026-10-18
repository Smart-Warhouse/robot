"""PC-03/PC-04/PC-05 on synthetic images rendered with cv2.aruco and a known camera matrix."""

import math

import cv2
import numpy as np
import pytest

from forklift_perception.aruco_core import (LOCATION, PALLET, ArucoEstimator, MarkerSpec, PoseFilter,
                                            pallet_pose_in_frame, rotation_to_quaternion, transform_from)
from forklift_perception.synthetic import camera_matrix, look_at_rvec, render

K = camera_matrix()


def rot_err_deg(R1, R2):
    dR = R1 @ R2.T
    return math.degrees(math.acos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))


def test_marker_id_ranges_and_sizes():
    spec = MarkerSpec()
    assert spec.classify(1) == (PALLET, 0.03) and spec.classify(20) == (PALLET, 0.03)
    assert spec.classify(21) == (LOCATION, 0.06) and spec.classify(40) == (LOCATION, 0.06)
    assert spec.classify(0) is None and spec.classify(41) is None


CASES = [(mid, size, d, yaw, lat)
         for mid, size in ((7, 0.030), (25, 0.060))
         for d in (0.20, 0.35, 0.50)
         for yaw in (-25, 0, 25)
         for lat in (-0.05, 0.05)]


@pytest.mark.parametrize("mid,size,d,yaw,lat", CASES)
def test_pose_accuracy_within_1cm_2deg_up_to_50cm(mid, size, d, yaw, lat):
    """PC-04: <= 1 cm and <= 2 deg at <= 50 cm (single frame, synthetic image with sensor noise)."""
    rv = look_at_rvec(yaw, 10.0)
    tv = np.array([lat, 0.02, d])
    img = render(mid, size, rv, tv, K, seed=int(d * 1000) + yaw + 100)
    obs = ArucoEstimator().estimate(img, K)
    assert len(obs) == 1
    o = obs[0]
    assert o.marker_id == mid and o.size_m == size
    assert np.linalg.norm(o.tvec - tv) <= 0.010
    assert rot_err_deg(o.R, cv2.Rodrigues(rv)[0]) <= 2.0
    assert 0.0 < o.confidence <= 1.0 and o.reproj_error_px < 0.5


def test_unknown_ids_are_ignored():
    img = render(45, 0.06, look_at_rvec(), np.array([0, 0, 0.3]), K)
    assert ArucoEstimator().estimate(img, K) == []


def test_moving_average_of_5_reduces_noise():
    """PC-05: the 5-sample moving average beats single noisy frames (heavier noise than real)."""
    rv, tv = look_at_rvec(15, 5), np.array([0.03, 0.0, 0.48])
    est = ArucoEstimator()
    filt = PoseFilter(window=5)
    single, averaged = [], []
    T_identity = np.eye(4)
    true_pos, true_yaw = pallet_pose_in_frame(T_identity, rv, tv)
    for k in range(25):
        img = render(3, 0.03, rv, tv, K, noise_std=6.0, seed=k)
        o = est.estimate(img, K)[0]
        pos, yaw = pallet_pose_in_frame(T_identity, o.rvec, o.tvec)
        filt.add(k * 0.05, pos, yaw, o.confidence)
        single.append(np.linalg.norm(pos - true_pos))
        if len(filt) == 5:
            averaged.append(np.linalg.norm(filt.mean()[0] - true_pos))
    assert len(filt) == 5
    assert np.std(averaged) < np.std(single)      # less jitter (the remaining error is the rendering bias)
    assert np.max(averaged) <= 0.010
    assert 0.0 < filt.confidence() <= 1.0


def test_filter_resets_on_gap_and_jump():
    f = PoseFilter(window=5, max_gap_s=0.5, max_jump_m=0.05)
    for k in range(5):
        f.add(k * 0.1, [1.0, 2.0, 0.0], 0.1)
    assert len(f) == 5
    f.add(0.5, [1.2, 2.0, 0.0], 0.1)       # 20 cm jump -> another pallet
    assert len(f) == 1
    f.add(2.0, [1.2, 2.0, 0.0], 0.1)       # 1.5 s gap
    assert len(f) == 1
    f = PoseFilter()
    f.add(2.1, [1.2, 2.0, 0.0], math.pi - 0.01)
    f.add(2.2, [1.2, 2.0, 0.0], -math.pi + 0.01)
    assert abs(abs(f.mean()[1]) - math.pi) < 0.05  # circular mean across +-pi


def test_pallet_yaw_points_into_the_pallet():
    """Camera optical frame = base frame rotated so that z looks forward: a marker facing the camera means the
    insertion direction is straight ahead (yaw 0); a marker turned by +25 deg about its vertical gives -25 deg."""
    T_base_opt = np.eye(4)
    T_base_opt[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])  # optical z -> base x, x -> -y, y -> -z
    pos, yaw = pallet_pose_in_frame(T_base_opt, look_at_rvec(0), np.array([0.0, 0.0, 0.4]))
    assert pos == pytest.approx([0.4, 0.0, 0.0]) and yaw == pytest.approx(0.0, abs=1e-9)
    _, yaw = pallet_pose_in_frame(T_base_opt, look_at_rvec(25), np.array([0.0, 0.0, 0.4]))
    assert abs(math.degrees(yaw)) == pytest.approx(25.0, abs=1e-6)


def test_quaternion_helpers_round_trip():
    for rv in (look_at_rvec(0), look_at_rvec(30, 10, 5), np.array([0.3, -1.2, 2.0])):
        R = cv2.Rodrigues(rv)[0]
        T = transform_from((0.1, 0.2, 0.3), rotation_to_quaternion(R))
        assert np.allclose(T[:3, :3], R, atol=1e-9) and np.allclose(T[:3, 3], [0.1, 0.2, 0.3])
