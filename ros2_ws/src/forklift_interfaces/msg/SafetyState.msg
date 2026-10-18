# Output of forklift_safety/safety_supervisor on `safety/status` (10 Hz + on change).
# `state` values are ordered by severity and map 1:1 onto the MQTT `safety_state` enum (docs/schemas/state.schema.json).
uint8 NORMAL=0
uint8 SLOW=1
uint8 STOP=2
uint8 SAFE=3
uint8 ESTOP=4

std_msgs/Header header
uint8 state
string reason            # event code from docs/02-api-contract.md §5 (tof_stop, person_detected, ...) or ""
string source            # command source currently driving the robot: nav | dock | teleop | none
float32 speed_limit      # m/s applied this cycle
bool docking             # SF-03 docking rules active
bool forks_raised        # fork height above the raised threshold (SF-08)
float32 min_tof_m        # closest ToF reading in the direction of travel
uint8 esp32_state        # raw ESP32 `safety_state` byte (low nibble state, high nibble reason)
