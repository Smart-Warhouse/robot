# forklift_sim (placeholder)

Gazebo Harmonic world of the 1:14 mini warehouse (EN-11, AI-01, plan B5). Only the static world exists so far:

- `worlds/warehouse.sdf` — **generated** by `scripts/generate_world.py` from the fleet server's
  `code/backend/app/warehouse_map.yaml`: boundary walls (0.45 m, taller than the LiDAR, EN-07), racks R1/R2 as
  0.15 m boxes, flat coloured patches for docks, staging, charging, walkway and keep-out zones. World frame = map frame.
  Re-run after the map changes: `python scripts/generate_world.py` (a host test fails when the file is stale).
- `launch/world.launch.py` — opens it with `ros_gz_sim` (not run yet).

Not done (plan B5): forklift model with LiDAR / camera / IMU sensors spawned from `forklift_description`,
`ros_gz_bridge` config, an ESP32 stand-in (cmd_vel → wheel_odom, tof_ring, fork_state), and running Nav2 + mission
against the fleet server with `simulated: false`. The docking policy itself is trained in the MuJoCo environment
of `code/training`, not here.
