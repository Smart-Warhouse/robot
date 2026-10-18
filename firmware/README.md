# Firmware — ESP32-S3 (Waveshare ESP32-S3 Zero)

Firmware for the autonomous 1:14 forklift (modified Huina 1577 Pro). One binary for every forklift; the
identity (`robot_id`) and the calibration live in NVS. Requirements §3, §4, §9 and §10 of
[system-requirements.md](../../docs/00-sources/system-requirements.md); design notes and the requirement
mapping in [docs/12-firmware.md](../../docs/12-firmware.md).

```
lib/forklift_core/      hardware-independent logic, pure C++17 (no Arduino headers) — unit-tested on the host
  src/forklift/config.h            ALL tunables: pins (§3), task rates (§4), safety thresholds (§9), placeholders
  src/forklift/safety_state_machine.*   FW-08 BOOT → IDLE → ACTIVE ↔ SLOW → STOP → SAFE / ESTOP
  src/forklift/safety_limiter.*         SF-02 ToF ring, SF-03 docking, SF-08 forks, SF-09 battery, ramps
  src/forklift/kinematics.*             FW-01 rear-steer Ackermann, AS5600 speed, FW-02 odometry
  src/forklift/control.*                FW-03 PID (feed-forward, anti-windup), FW-04 lift controller
  src/forklift/madgwick.*               IMU orientation filter
  src/forklift/calibration.*            FW-05 calibration blob (defaults, validation, CRC32)
src/                    board code (Arduino core on FreeRTOS)
  tasks.cpp             the six FreeRTOS tasks pinned per the §4 table
  drivers/              I2C bus + mutex, H-bridges + servo (LEDC), AS5600, MPU9250, INA219, TCA9548A + VL53L1X,
                        GPIO inputs, WS2812 ring + buzzer
  storage.cpp           NVS: calibration blob + robot_id
  microros_node.cpp     §10 topics — only with -DFL_ENABLE_MICROROS=1
  console.cpp           calibration / bench console
test/                   Unity tests run on the host (`pio test -e native`)
```

## Build, test, flash

PlatformIO lives in a local venv (no global install needed):

```powershell
cd code/firmware
python -m venv .venv
.venv\Scripts\pip install platformio

# host unit tests (needs a host gcc/g++ on PATH — MSYS2 ucrt64 works)
$env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
.venv\Scripts\pio test -e native

# board build without micro-ROS (default env)
.venv\Scripts\pio run -e esp32s3
# flash over the native USB (hold BOOT, tap RESET if the port does not appear) and open the console
.venv\Scripts\pio run -e esp32s3 -t upload
.venv\Scripts\pio device monitor -b 115200
```

Last results (2026-10-18): `pio test -e native` → **60 test cases, 60 passed** (4 suites);
`pio run -e esp32s3` → **SUCCESS, 0 warnings**, RAM 22,012 B (6.7 %), flash 355,209 B (27.1 % of the 1.25 MB
app partition).

### micro-ROS build (`esp32s3_microros`)

The `micro_ros_platformio` library builds the micro-ROS static library itself with colcon, using POSIX shell
commands (`. venv/bin/activate`, backticks). On native Windows it fails — first on the space in
`Grad. Project`, then on the 260-character path limit, then on the shell syntax — so build this env on
Linux or WSL2 (WSL2 Ubuntu 22.04 is planned anyway for ROS 2, plan B3):

```bash
sudo apt install -y git cmake python3-venv python3-pip build-essential
cd ~ && cp -r "/mnt/c/Users/ASUS/Desktop/Grad. Project/code/firmware" fw && cd fw   # path without spaces
python3 -m venv .venv && .venv/bin/pip install platformio
.venv/bin/pio run -e esp32s3_microros                 # first run builds micro-ROS humble (~10–20 min)
.venv/bin/pio run -e esp32s3_microros -t upload --upload-port /dev/ttyACM0
```

`microros.meta` raises the XRCE entity limits (the defaults allow only 5 subscriptions; we use 6 plus the
parameter server). On the Jetson: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600`.
With micro-ROS on, the USB port belongs to the agent and the console moves to UART0 (GPIO 43 TX / 44 RX,
115200) — check those pads on the Zero.

`src/microros_node.cpp` has **not been compiled yet** (see above); expect small API fixes on the first Linux
build (it targets rclc for ROS 2 Humble, e.g. the 3-argument parameter callback).

FW-11 (update from the Jetson): the same USB cable — `pio run -t upload` or
`esptool.py --chip esp32s3 -p /dev/ttyACM0 write_flash 0x10000 firmware.bin`. OTA is not implemented.

## Pin map (§3 — verify against the real board before soldering, plan A4)

| Function | GPIO | Notes |
|---|---|---|
| Drive motor IN1 / IN2 | 2 / 3 | LEDC ch 0/1, 20 kHz. GPIO 3 is a JTAG strapping pin only if an eFuse is burnt |
| Lift motor IN1 / IN2 | 4 / 5 | LEDC ch 2/3, 20 kHz |
| Side-shift IN1 / IN2 | 6 / 7 | LEDC ch 4/5, 20 kHz |
| Steering servo (MG90S) | 11 | LEDC ch 6, 50 Hz, 14-bit |
| Fork slide pot | 1 | ADC1_CH0, 11 dB |
| I2C SDA / SCL | 12 / 13 | 400 kHz: TCA9548A 0x70 (ToF ch 0–5), AS5600 0x36, MPU9250 0x68, INA219 0x40 |
| Limit top / bottom | 14 / 15 | pull-up, active low, interrupt |
| TCRT5000 left / right | 16 / 17 | pull-up, active low |
| E-Stop second contact | 18 | pull-up, NC to GND: **pressed or unplugged reads HIGH = ESTOP** |
| WS2812 ring (7) | 38 | via Adafruit NeoPixel (RMT) |
| Active buzzer | 39 | |
| micro-ROS | USB 19/20 | native USB CDC |
| (optional) AS5600 on 2nd bus | 8 / 9 | `AS5600_ON_SECOND_BUS` in config.h, off by default |

ToF ring order: 0 front (under the forks), 1 front-left, 2 front-right, 3 left, 4 right, 5 rear
(`TOF_MUX_CHANNEL` maps them to mux channels). The Zero's on-board RGB LED is GPIO 21 (unused).

On the bench with no E-Stop wired, **tie GPIO 18 to GND**, otherwise the robot correctly stays in ESTOP.

## Console

Type `help`. Without micro-ROS the console is on the USB port. Main commands:

| Command | Use |
|---|---|
| `status`, `tof`, `diag` | state, sensors, FW-10 diagnostics JSON |
| `id fl1` | set `robot_id` (FW-12) → topics become `/fl1/...` after a reboot |
| `reset` | leave ESTOP after the button is released (FW-08) |
| `bench on` / `bench off` | console stands in for the Jetson heartbeat (**wheels off the ground**). `bench off` = T-07 watchdog test |
| `cmd <v> <w>` | cmd_vel in bench mode |
| `step <v> <ms>` | T-03 speed step, prints CSV at 100 Hz |
| `lift <mm>`, `shift -1/0/1`, `dock on/off`, `odom reset` | |
| `cal …` | calibration, see below; nothing is stored until `cal save` |

## Calibration procedure (FW-05) — once per forklift

1. `id flN`, reboot. `cal show` should say `valid: yes`.
2. **Fork pot**: lower the forks onto the bottom limit → `cal pot bottom`; raise to a measured height
   (ruler, e.g. 100 mm) → `cal pot top 100`. Check `status` at a few heights (T-04 needs ± 3 mm).
3. **Steering servo**: `bench on`, `cmd 0.05 0`, adjust `cal servo <µs>` until the forklift runs straight;
   measure the rear wheel angle at two pulse widths → `cal servo_gain <µs per rad>` (positive if a longer
   pulse turns the rear wheels left, else negative); set `cal servo_limits` to the mechanical stops.
4. **IMU**: forklift still and level → `cal imu` (5 s). T-02: yaw drift ≤ 2°/min afterwards.
5. **Wheel / encoder**: roll the forklift 2 m by hand; if `status` shows negative x, `cal encdir -1`; if the
   distance is off, `cal wheel <radius>` = radius × 2 / measured distance.
6. **PID gains** (T-03 / T-04): `cal drive <kp> <ki> <kd> <kff>`, `cal lift <kp> <ki>`; with micro-ROS also the
   ROS parameters `drive_kp … lift_ki` and `save_calibration`.
7. `cal save`. A blob that fails CRC or validation is ignored at boot and the defaults are used.

## Still placeholders (replace after ME-01 / ME-02 and the bench tests)

| Value | Where | Current |
|---|---|---|
| Wheelbase, max steering angle | `config.h` `WHEELBASE_M`, `MAX_STEER_RAD` | 0.20 m, 30° (same as the RL config) |
| Drive speed at 100 % PWM (feed-forward) | `DRIVE_SPEED_AT_FULL_DUTY` | 0.45 m/s |
| Encoder gear ratio | `DRIVE_GEAR_RATIO` | 1 (magnet on the axle) |
| Lift range, lift min duty | `LIFT_MAX_MM`, `LIFT_MIN_DUTY` | 120 mm, 0.20 |
| ToF mux wiring, docking-ignore mask | `TOF_MUX_CHANNEL`, `TOF_MASK_DOCKING_IGNORE` | 0…5, all three front sensors (corners see the pallet in simulation — confirm on the real ring) |
| INA219 shunt | `INA219_SHUNT_OHM` | 0.1 Ω reads only ±3.2 A; the ≈ 5 A peak needs a 0.01 Ω shunt |
| Wheel radius, pot points, servo centre/gain | calibration defaults | 30 mm, raw 300/3800, 1500 µs, 900 µs/rad |
| PID gains | `defaultDriveGains()`, `defaultLiftGains()` | tuned on plant models only |
| Battery curve | `batteryPercentFromVoltage` | generic 2S Li-ion curve |

Known limits: the magnetometer is not fused (motors and battery next to it); the side-shift has no
feedback (runs until 0 or a 3 s timeout); the `fork_state` custom message of §10 is sent as a
`Float32MultiArray` until `forklift_interfaces` exists.
