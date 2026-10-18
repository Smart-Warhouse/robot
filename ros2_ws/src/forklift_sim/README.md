# forklift_sim

Full-system digital twin of the mini warehouse in **Gazebo Fortress** (`ros_gz`, the Gazebo paired with ROS 2
Humble) — EN-11, AI-01, plan B5. Full description, run instructions, CI checks and known gaps:
[docs/19-gazebo-sim.md](../../../../docs/19-gazebo-sim.md).

- `worlds/warehouse.sdf`, `maps/warehouse_sim.{pgm,yaml}` — **generated** by `scripts/generate_world.py` from
  `code/backend/app/warehouse_map.yaml` (walls 0.45 m, racks, zone patches; occupancy map of walls + racks).
  Re-run after the map changes; a host test fails when they are stale.
- `urdf/forklift_gz.urdf.xacro` — `forklift_description`'s URDF + LiDAR, fork camera, IMU, velocity drive,
  odometry and joint states.
- `config/bridge.yaml` — ros_gz_bridge per robot (`ROBOT_ID` → fl1, fl2, …).
- `launch/sim.launch.py` — headless gz server, 1–3 forklifts, bridges, optional `stack:=nav`.
- `scripts/cmd_vel_relay.py` — `supervisor_bypass`: cmd_vel_nav → cmd_vel with a watchdog.
- `test/launch/*.launch.py` — launch tests, only built with `-DFORKLIFT_SIM_TESTS=ON` (`.github/workflows/sim.yml`).

```bash
ros2 launch forklift_sim sim.launch.py robots:=fl1,fl2,fl3 stack:=nav gui:=true
```
