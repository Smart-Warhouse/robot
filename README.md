# robot — firmware, ROS 2 and mounts of the autonomous forklift

Part of the [Smart-Warhouse](https://github.com/Smart-Warhouse) project ([docs](https://github.com/Smart-Warhouse/docs)).

| Folder | What | Docs |
|---|---|---|
| [`firmware/`](firmware/) | ESP32-S3 firmware: 1 kHz safety loop, PID drive/lift, micro-ROS (PlatformIO, FreeRTOS) | [12-firmware](https://github.com/Smart-Warhouse/docs/blob/main/12-firmware.md), [15-bench-tests](https://github.com/Smart-Warhouse/docs/blob/main/15-bench-tests.md) |
| [`ros2_ws/`](ros2_ws/) | ROS 2 Humble packages for the Jetson: mission + fleet client, Nav2, perception, safety, RL policy, camera stream, Gazebo twin | [13-ros2](https://github.com/Smart-Warhouse/docs/blob/main/13-ros2.md), [19-gazebo-sim](https://github.com/Smart-Warhouse/docs/blob/main/19-gazebo-sim.md) |
| [`cad/`](cad/) | parametric 3D-printable mounts (CadQuery) | [14-hardware-build](https://github.com/Smart-Warhouse/docs/blob/main/14-hardware-build.md) |

Some tests read sibling repos (`backend` lane graph, `docs` MQTT schemas, `ai` docking config and policy): clone
the repos side by side as described in the [docs README](https://github.com/Smart-Warhouse/docs#workspace-layout),
or set `SW_WORKSPACE`. CI: `.github/workflows/ci.yml` (firmware, CAD, ROS 2 host tests, colcon build) and
`sim.yml` (Gazebo twin, headless).
