# Reference MQTT forklift client (no ROS)

`robot_client.py` plays one real forklift against the server's MQTT bridge, implementing
[`docs/03-mqtt-contract.md`](../../../../docs/03-mqtt-contract.md) from the robot side. It drives the lane graph
from `code/backend/app/warehouse_map.yaml` at a fixed speed and fakes pick/drop as timed dwells. It is the template
for the `forklift_mission` fleet client; swap the "drive a lane" and "dwell" parts for Nav2 / fork actions.

```bash
pip install -r requirements.txt
python robot_client.py --id fl1 --password <fl1-password> --host <server-ip>
```

Options: `--speed` (m/s, default 0.5), `--action-s` (seconds per pick/drop), `--start` (start node, default the
robot's home), `--fail-pick` (every pickup fails, to test `failed`), `-v`.

What it does:

| Contract item | Client behaviour |
|---|---|
| Last Will (CM-03) | will `{"online": false}` on `wh/fl/{id}/state`; also sent on a clean Ctrl-C |
| State, 2 Hz (CM-04) | pose, speed, battery, mode, fork height, carried pallet, last `node`, current `task_id` (extension); every message has `seq` + `ts` |
| Task | `task_status`: `accepted` → `to_pickup` → `picking` → `to_drop` → `dropping` → `done` / `failed` + `reason`; follows the task's `route`, split at `from.node` |
| Traffic | `request` on `wh/traffic/{from}--{to}` before each lane, waits for its own `grant`, re-requests every 500 ms on `deny`; `release` on arrival |
| `cmd` | `pause` / `resume` / `cancel` (ignores a cancel for another `task_id`) / `go_charge` / `reset` / `mode` manual·auto |
| `teleop` | applied only in manual mode, 0.5 s dead-man timeout |
| E-stop | retained `wh/fleet/estop`: holds still while `engaged` |
| Broker lost | stops and reports `safe` after reconnecting; needs an operator `reset` (FW-08) |

## End-to-end check

1. Broker: `docker compose up mqtt` in `code/backend` (needs `mosquitto/passwd`, see the backend README), or any
   local broker on 1883.
2. Set `simulated: false` for `fl1` in `warehouse_map.yaml` (or point `MAP_FILE` at a copy) and start the backend
   with `MQTT_ENABLED=true MQTT_USERNAME=server MQTT_PASSWORD=...`. `/api/health` must say `"mqtt": "connected"`.
3. Start this client as `fl1`; the dashboard shows Forklift 1 online at CH1.
4. Create a task (dashboard or `POST /api/tasks`); it is assigned to fl1, runs through every phase and the pallet
   moves in the WMS on `done`.
5. Kill the client (`taskkill /F` / `kill -9`): the broker publishes the will and fl1 goes offline at once; a task
   not yet lifted returns to the queue.
