# ROS 2 Humble workspace — forklift Jetson software

Everything one autonomous 1:14 forklift runs on its Jetson Orin Nano (JetPack 6 = Ubuntu 22.04, ROS 2 Humble),
next to the ESP32-S3 firmware (`code/firmware`) and supervised by the fleet server (`code/backend`) over MQTT
([docs/03-mqtt-contract.md](../../docs/03-mqtt-contract.md)). Design, requirement mapping and the per-robot
node/topic graph: [docs/13-ros2.md](../../docs/13-ros2.md).

All real logic is in plain Python modules without rclpy (`*_core.py`, `bt.py`, `lane_graph.py`, …) and is unit-tested
on any machine; the rclpy nodes (`*_node.py`, `*_bridge.py`) are thin wrappers around them.

## Packages

| Package | Type | Contents |
|---|---|---|
| `forklift_description` | ament_cmake | URDF/Xacro (`config/dimensions.yaml`, PLACEHOLDER until ME-01), TF `base_link → {laser, imu, tof_*, fork → camera}`, `view_forklift.launch.py`, `fork_joint_state_publisher.py` |
| `forklift_interfaces` | ament_cmake | msg `ForkState`, `SafetyState`, `FleetStatus`, `PalletPose`; srv `Reset`; actions `Lift`, `Dock`, `MoveTask` |
| `forklift_bringup` | ament_cmake | `bringup.launch.py` (one namespace per robot), `record.launch.py` (rosbag), `config/*.yaml` (all node parameters), systemd unit, installs the server's `warehouse_map.yaml` |
| `forklift_navigation` | ament_cmake | Nav2 params (Smac Hybrid-A*, RPP 20 Hz, costmaps, keep-out + speed filters), lane BT without Spin, slam_toolbox mapping/localization, EKF, filter masks + generator |
| `forklift_perception` | ament_python | ArUco DICT_4X4_50 + solvePnP + 5-sample average → `pallet_pose`; initial pose from the charger marker |
| `forklift_safety` | ament_python | safety supervisor (clamps Nav2 / RL / gamepad → `cmd_vel`, heartbeat, `safety_reset`), `fork_state_bridge` |
| `forklift_mission` | ament_python | MQTT fleet client (docs/03) + Behavior-Tree mission executor, Lift action server, fake broker |
| `forklift_rl` | ament_python | 18-dim observation, ONNX/TensorRT policy runner, classical fallback, `Dock` action server |
| `forklift_sim` | ament_cmake | Gazebo Fortress digital twin: generated world + map, sim URDF, bridge, `sim.launch.py` ([docs/19](../../docs/19-gazebo-sim.md)) |
| `forklift_camera_stream` | ament_python | UI-08 low-res MJPEG of the fork camera (manual mode only), relayed by the server; `standalone` runs it on a laptop webcam / test pattern |

`tools/mqtt_robot_client/` is the ROS-free reference robot client the fleet client was ported from (unchanged).

## Host tests (Windows, macOS, Linux — no ROS needed)

```powershell
cd code/ros2_ws
python -m venv .venv
.venv/Scripts/python -m pip install -r requirements-host.txt     # Linux/macOS: .venv/bin/python
.venv/Scripts/python -m pytest -q                                # src/ + tests/ (pytest.ini)
```

`conftest.py` puts every package on `sys.path`. Covered: lane graph, BT, fleet client against an in-process MQTT
broker with Last Will / retained E-stop (all messages validated against `docs/schemas`), whole tasks with failures
and server commands, safety supervisor, ArUco accuracy on synthetic images, initial pose, the docking observation
compared with 718 states recorded from the training simulator, the ONNX runner on a generated model, a closed-loop
docking run, Nav2/EKF/SLAM parameters, filter masks and the Gazebo world are up to date with the map, every
parameter in `forklift_bringup/config` is declared by its node, and `tests/test_workspace.py` (manifests, every
XML/YAML parses, every Python file compiles, cores import no ROS, LF + UTF-8).

Optional: `MQTT_TEST_HOST=<broker> MQTT_TEST_USER=fl1 MQTT_TEST_PASSWORD=…` also runs the paho transport against a
real Mosquitto. `src/forklift_rl/tools/dump_env_observations.py` regenerates the simulator fixture (run it with the
training venv after changing `code/training/forklift_rl/env.py` or `config.yaml`).

## Build on Ubuntu 22.04 + ROS 2 Humble (Jetson or laptop)

```bash
sudo apt install python3-rosdep python3-colcon-common-extensions python3-pip
sudo rosdep init; rosdep update
cd ~/grad-project/code/ros2_ws
# not in rosdep for Humble: build from source in this workspace (or apt if your mirror has them)
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git src/external/micro-ROS-Agent
git clone -b humble https://github.com/micro-ROS/micro_ros_msgs.git src/external/micro_ros_msgs
git clone https://github.com/Slamtec/sllidar_ros2.git src/external/sllidar_ros2
rosdep install --from-paths src --ignore-src -r -y     # Nav2, slam_toolbox, robot_localization, laser_filters, v4l2_camera…
pip install "numpy<2" "paho-mqtt>=2" onnxruntime      # numpy<2 keeps the system OpenCV/cv_bridge working; Jetson: onnxruntime-gpu wheel from the Jetson Zoo (TensorRT EP)
colcon build --symlink-install
source install/setup.bash
```

Run one forklift (the ESP32 on `/dev/ttyACM0`, RPLIDAR C1 on `/dev/ttyUSB0`):

```bash
echo '<fl1 MQTT password>' | sudo tee /etc/forklift/mqtt_password && sudo chmod 600 /etc/forklift/mqtt_password
ros2 launch forklift_bringup bringup.launch.py robot_id:=fl1 mqtt_host:=192.168.50.10
ros2 launch forklift_bringup bringup.launch.py robot_id:=fl1 slam_mode:=mapping nav:=false     # NV-01 mapping run
ros2 launch forklift_bringup record.launch.py robot_id:=fl1                                      # JS-08
ros2 launch forklift_description view_forklift.launch.py                                          # URDF + TF in RViz
```

Autostart (JS-07): copy `src/forklift_bringup/systemd/forklift.service` to `/etc/systemd/system/` and
`forklift.env.example` to `/etc/forklift/forklift.env` (set `ROBOT_ID`, `ROS_DOMAIN_ID`, `MQTT_HOST`), then
`sudo systemctl enable --now forklift`. A fourth forklift = a new `robots:` entry in the server map and a new env
file (NFR-09).

The docking policy is expected at `code/training/models/docking_v3/policy.onnx` (exported by
`code/training/scripts/export_onnx.py`); override with `model_path:=` or `FORKLIFT_POLICY`. Without it the Dock
server falls back to the classical controller and logs an error.

## Regenerated files

| File | Generator |
|---|---|
| `src/forklift_navigation/maps/{keepout,speed}_mask.*` | `src/forklift_navigation/scripts/make_filter_masks.py` |
| `src/forklift_sim/worlds/warehouse.sdf`, `src/forklift_sim/maps/warehouse_sim.*` | `src/forklift_sim/scripts/generate_world.py` |
| `src/forklift_rl/test/data/env_obs_samples.json` | `src/forklift_rl/tools/dump_env_observations.py` (training venv) |

Host tests fail when the first two are stale with respect to `code/backend/app/warehouse_map.yaml`.
