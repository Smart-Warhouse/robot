# Fork carriage state (requirements §10 `fork_state`, 20 Hz).
# MIGRATION: the ESP32 firmware (code/firmware/src/microros_node.cpp) currently publishes `fork_state` as
# std_msgs/Float32MultiArray [height_mm, limit_top, limit_bottom, tcrt_left, tcrt_right, at_target] (best effort);
# forklift_safety/fork_state_bridge converts it to this message on `fork_state/typed`. Once the firmware is built
# with forklift_interfaces (micro-ROS `extra_packages`) it publishes this type on `fork_state` directly and the
# bridge + the launch remaps are removed. Field order = the array order.
std_msgs/Header header
float32 height_mm        # fork height above the lowest position, slide pot (FW-04); NaN = unknown
bool limit_high          # upper limit switch pressed (ME-08)      [array index 1, limit_top]
bool limit_low           # lower limit switch pressed               [array index 2, limit_bottom]
bool tcrt_left           # TCRT5000 on the left fork tip sees the pallet above it (ME-09)
bool tcrt_right          # TCRT5000 on the right fork tip
bool at_target           # lift controller reports the set-point reached
