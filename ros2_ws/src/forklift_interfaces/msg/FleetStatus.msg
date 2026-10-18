# Published by forklift_mission/mission_node on `fleet_status` (5 Hz + on change): what the fleet server link
# and the mission executor want the rest of the robot to know.
std_msgs/Header header
string robot_id
string mode              # idle | busy | charging | paused | manual | safe | estop | error (state.schema.json)
bool server_connected
float32 server_silence_s # seconds since the broker connection was lost (0 while connected)
bool fleet_estop         # retained wh/fleet/estop is engaged (SF-11)
bool step_active         # a motion step (lane, dock, straight move) is in progress — SF-05 lets it finish
bool hold                # executor is holding still (pause, estop, server loss, ...)
string hold_reason
string node              # last lane-graph node reached ("" between nodes)
string task_id
string phase             # accepted | to_pickup | picking | to_drop | dropping | done | failed | ""
int32 carrying_pallet_id # -1 = none
