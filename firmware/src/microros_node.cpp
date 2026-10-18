// micro-ROS over the native USB CDC to the Jetson (micro-ROS Agent: `ros2 run micro_ros_agent
// micro_ros_agent serial --dev /dev/ttyACM0 -b 921600`). Node "esp32" in namespace robot_id, so every
// topic below becomes /fl1/<topic> on forklift fl1 (JS-07 namespaces).
//
//   §10 topic        type                          dir   rate        notes
//   cmd_vel          geometry_msgs/Twist           in    20 Hz       linear.x m/s, angular.z rad/s
//   lift_cmd         std_msgs/Float32              in    on demand   fork height, mm
//   sideshift_cmd    std_msgs/Int8                 in    on demand   −1 / 0 / 1
//   heartbeat        std_msgs/UInt32               in    10 Hz       link watchdog (FW-06)
//   wheel_odom       nav_msgs/Odometry             out   50 Hz       odom → base_link (front axle centre)
//   imu/data         sensor_msgs/Imu               out   100 Hz      frame imu
//   tof_ring         std_msgs/Float32MultiArray    out   20 Hz       6 distances, m (NaN = sensor fault)
//   fork_state       std_msgs/Float32MultiArray    out   20 Hz       [height_mm, limit_top, limit_bottom,
//                                                                     tcrt_left, tcrt_right, at_target]
//   battery          sensor_msgs/BatteryState      out   1 Hz
//   safety_state     std_msgs/UInt8                out   10 Hz + on change   state << 4 | reason
// Additions (not in the §10 table, needed by FW-08 / SF-03 / FW-10):
//   safety_reset     std_msgs/Empty                in    on demand   leave ESTOP (after release)
//   docking_mode     std_msgs/Bool                 in    on demand   SF-03 pallet-docking mode
//   diagnostics      std_msgs/String               out   1 Hz        JSON (FW-10)
// Parameters (FW-03): drive_kp, drive_ki, drive_kd, drive_kff, lift_kp, lift_ki, save_calibration.
#include "microros_node.h"

#if FL_ENABLE_MICROROS

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/u_int8.h>

#include <cmath>
#include <cstring>

#include "app_state.h"
#include "storage.h"
#include "tasks.h"

namespace fl {

namespace {

enum class Link { WAITING, AVAILABLE, CONNECTED, DISCONNECTED };

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rclc_parameter_server_t params;

rcl_subscription_t sub_cmd_vel, sub_lift, sub_shift, sub_heartbeat, sub_reset, sub_docking;
rcl_publisher_t pub_odom, pub_imu, pub_tof, pub_fork, pub_battery, pub_safety, pub_diag;

geometry_msgs__msg__Twist msg_cmd_vel;
std_msgs__msg__Float32 msg_lift;
std_msgs__msg__Int8 msg_shift;
std_msgs__msg__UInt32 msg_heartbeat;
std_msgs__msg__Empty msg_reset;
std_msgs__msg__Bool msg_docking;

nav_msgs__msg__Odometry msg_odom;
sensor_msgs__msg__Imu msg_imu;
std_msgs__msg__Float32MultiArray msg_tof, msg_fork;
sensor_msgs__msg__BatteryState msg_battery;
std_msgs__msg__UInt8 msg_safety;
std_msgs__msg__String msg_diag;

float tof_buf[cfg::TOF_COUNT];
float fork_buf[6];
char diag_buf[512];
char empty_str[1] = {0};

void setString(rosidl_runtime_c__String& s, const char* text) {
  s.data = const_cast<char*>(text);
  s.size = strlen(text);
  s.capacity = s.size + 1;
}

void stamp(builtin_interfaces__msg__Time& t) {
  const int64_t ns = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos() : int64_t(micros()) * 1000;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
}

// ---------------------------------------------------------------- callbacks (executor, core 0)
void onCmdVel(const void* m) {
  const auto* t = static_cast<const geometry_msgs__msg__Twist*>(m);
  const uint32_t now = nowMs();
  app.cmd.modify([&](CommandState& c) {
    c.cmd_v = static_cast<float>(t->linear.x);
    c.cmd_w = static_cast<float>(t->angular.z);
    c.cmd_seen = true;
    c.last_cmd_ms = now;
  });
}

void onLift(const void* m) {
  float mm = static_cast<const std_msgs__msg__Float32*>(m)->data;
  if (!std::isfinite(mm)) return;
  if (mm < 0.0f) mm = 0.0f;
  if (mm > cfg::LIFT_MAX_MM) mm = cfg::LIFT_MAX_MM;
  app.cmd.modify([&](CommandState& c) {
    c.lift_target_mm = mm;
    c.lift_cmd_seen = true;
  });
}

void onShift(const void* m) {
  int8_t d = static_cast<const std_msgs__msg__Int8*>(m)->data;
  d = d > 0 ? 1 : (d < 0 ? -1 : 0);
  const uint32_t now = nowMs();
  app.cmd.modify([&](CommandState& c) {
    c.sideshift = d;
    c.sideshift_ms = now;
  });
}

void onHeartbeat(const void* m) {
  const uint32_t v = static_cast<const std_msgs__msg__UInt32*>(m)->data;
  const uint32_t now = nowMs();
  app.cmd.modify([&](CommandState& c) {
    c.heartbeat_seen = true;
    c.last_heartbeat_ms = now;
    c.heartbeat_value = v;
  });
}

void onReset(const void*) { app.reset_request.store(true); }

void onDocking(const void* m) {
  const bool on = static_cast<const std_msgs__msg__Bool*>(m)->data;
  app.cmd.modify([&](CommandState& c) { c.docking = on; });
}

bool onParameter(const Parameter* old_param, const Parameter* new_param, void*) {
  (void)old_param;
  if (new_param == nullptr) return false;  // deleting is not allowed
  const char* name = new_param->name.data;
  if (strcmp(name, "save_calibration") == 0) {
    if (new_param->value.bool_value) storageSaveCalibration(app.cal.get());
    return true;
  }
  if (new_param->value.type != RCLC_PARAMETER_DOUBLE) return false;
  const float v = static_cast<float>(new_param->value.double_value);
  if (!(v >= 0.0f) || !std::isfinite(v)) return false;
  bool known = true;
  app.cal.modify([&](Calibration& c) {
    if (!strcmp(name, "drive_kp")) c.drive.kp = v;
    else if (!strcmp(name, "drive_ki")) c.drive.ki = v;
    else if (!strcmp(name, "drive_kd")) c.drive.kd = v;
    else if (!strcmp(name, "drive_kff")) c.drive.kff = v;
    else if (!strcmp(name, "lift_kp")) c.lift.kp = v;
    else if (!strcmp(name, "lift_ki")) c.lift.ki = v;
    else known = false;
  });
  if (known) app.gains_changed.store(true);
  return known;
}

// ---------------------------------------------------------------- entities
bool createEntities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "esp32", app.robot_id, &support) != RCL_RET_OK) return false;

  bool ok = true;
  auto sub = [&](rcl_subscription_t* s, const rosidl_message_type_support_t* ts, const char* name) {
    ok = ok && rclc_subscription_init_default(s, &node, ts, name) == RCL_RET_OK;
  };
  auto pub = [&](rcl_publisher_t* p, const rosidl_message_type_support_t* ts, const char* name, bool reliable) {
    ok = ok && (reliable ? rclc_publisher_init_default(p, &node, ts, name)
                         : rclc_publisher_init_best_effort(p, &node, ts, name)) == RCL_RET_OK;
  };
  sub(&sub_cmd_vel, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  sub(&sub_lift, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "lift_cmd");
  sub(&sub_shift, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "sideshift_cmd");
  sub(&sub_heartbeat, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "heartbeat");
  sub(&sub_reset, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "safety_reset");
  sub(&sub_docking, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "docking_mode");
  // Odometry (~700 B) is larger than the serial MTU: it must use the reliable (fragmenting) stream.
  pub(&pub_odom, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "wheel_odom", true);
  pub(&pub_imu, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data", false);
  pub(&pub_tof, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "tof_ring", false);
  pub(&pub_fork, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "fork_state", false);
  pub(&pub_battery, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "battery", true);
  pub(&pub_safety, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "safety_state", true);
  pub(&pub_diag, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "diagnostics", true);
  if (!ok) return false;

  if (rclc_parameter_server_init_default(&params, &node) != RCL_RET_OK) return false;

  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 6 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
                         &allocator) != RCL_RET_OK)
    return false;
  ok = ok && rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &onCmdVel, ON_NEW_DATA) == RCL_RET_OK;
  ok = ok && rclc_executor_add_subscription(&executor, &sub_lift, &msg_lift, &onLift, ON_NEW_DATA) == RCL_RET_OK;
  ok = ok && rclc_executor_add_subscription(&executor, &sub_shift, &msg_shift, &onShift, ON_NEW_DATA) == RCL_RET_OK;
  ok = ok && rclc_executor_add_subscription(&executor, &sub_heartbeat, &msg_heartbeat, &onHeartbeat, ON_NEW_DATA) == RCL_RET_OK;
  ok = ok && rclc_executor_add_subscription(&executor, &sub_reset, &msg_reset, &onReset, ON_NEW_DATA) == RCL_RET_OK;
  ok = ok && rclc_executor_add_subscription(&executor, &sub_docking, &msg_docking, &onDocking, ON_NEW_DATA) == RCL_RET_OK;
  ok = ok && rclc_executor_add_parameter_server(&executor, &params, onParameter) == RCL_RET_OK;
  if (!ok) return false;

  const Calibration cal = app.cal.get();
  rclc_add_parameter(&params, "drive_kp", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&params, "drive_ki", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&params, "drive_kd", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&params, "drive_kff", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&params, "lift_kp", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&params, "lift_ki", RCLC_PARAMETER_DOUBLE);
  rclc_add_parameter(&params, "save_calibration", RCLC_PARAMETER_BOOL);
  rclc_parameter_set_double(&params, "drive_kp", cal.drive.kp);
  rclc_parameter_set_double(&params, "drive_ki", cal.drive.ki);
  rclc_parameter_set_double(&params, "drive_kd", cal.drive.kd);
  rclc_parameter_set_double(&params, "drive_kff", cal.drive.kff);
  rclc_parameter_set_double(&params, "lift_kp", cal.lift.kp);
  rclc_parameter_set_double(&params, "lift_ki", cal.lift.ki);
  rclc_parameter_set_bool(&params, "save_calibration", false);

  rmw_uros_sync_session(100);
  return true;
}

void destroyEntities() {
  rmw_context_t* rmw = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw, 0);
  rcl_subscription_fini(&sub_cmd_vel, &node);
  rcl_subscription_fini(&sub_lift, &node);
  rcl_subscription_fini(&sub_shift, &node);
  rcl_subscription_fini(&sub_heartbeat, &node);
  rcl_subscription_fini(&sub_reset, &node);
  rcl_subscription_fini(&sub_docking, &node);
  rcl_publisher_fini(&pub_odom, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_publisher_fini(&pub_tof, &node);
  rcl_publisher_fini(&pub_fork, &node);
  rcl_publisher_fini(&pub_battery, &node);
  rcl_publisher_fini(&pub_safety, &node);
  rcl_publisher_fini(&pub_diag, &node);
  rclc_parameter_server_fini(&params, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void initMessages() {
  setString(msg_odom.header.frame_id, "odom");
  setString(msg_odom.child_frame_id, "base_link");
  for (int i = 0; i < 36; ++i) msg_odom.pose.covariance[i] = msg_odom.twist.covariance[i] = 0.0;
  msg_odom.pose.covariance[0] = msg_odom.pose.covariance[7] = 1e-3;   // x, y
  msg_odom.pose.covariance[14] = msg_odom.pose.covariance[21] = msg_odom.pose.covariance[28] = 1e6;
  msg_odom.pose.covariance[35] = 3e-3;                                  // yaw
  msg_odom.twist.covariance[0] = 1e-4;
  msg_odom.twist.covariance[35] = 1e-3;
  for (int i : {7, 14, 21, 28}) msg_odom.twist.covariance[i] = 1e6;

  setString(msg_imu.header.frame_id, "imu");
  for (int i = 0; i < 9; ++i)
    msg_imu.orientation_covariance[i] = msg_imu.angular_velocity_covariance[i] =
        msg_imu.linear_acceleration_covariance[i] = 0.0;
  for (int i : {0, 4, 8}) {
    msg_imu.orientation_covariance[i] = 1e-3;
    msg_imu.angular_velocity_covariance[i] = 1e-5;
    msg_imu.linear_acceleration_covariance[i] = 1e-2;
  }

  msg_tof.data.data = tof_buf;
  msg_tof.data.size = msg_tof.data.capacity = cfg::TOF_COUNT;
  msg_fork.data.data = fork_buf;
  msg_fork.data.size = msg_fork.data.capacity = 6;

  setString(msg_battery.header.frame_id, "base_link");
  setString(msg_battery.location, empty_str);
  setString(msg_battery.serial_number, empty_str);
  msg_battery.design_capacity = cfg::BATTERY_CAPACITY_AH;
  msg_battery.capacity = NAN;
  msg_battery.charge = NAN;
  msg_battery.temperature = NAN;
  msg_battery.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
  msg_battery.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;

  msg_diag.data.data = diag_buf;
  msg_diag.data.capacity = sizeof diag_buf;
  msg_diag.data.size = 0;
}

// ---------------------------------------------------------------- publishers
void publishOdom() {
  const DriveState d = app.drive.get();
  stamp(msg_odom.header.stamp);
  msg_odom.pose.pose.position.x = d.pose.x;
  msg_odom.pose.pose.position.y = d.pose.y;
  msg_odom.pose.pose.position.z = 0.0;
  msg_odom.pose.pose.orientation.x = 0.0;
  msg_odom.pose.pose.orientation.y = 0.0;
  msg_odom.pose.pose.orientation.z = std::sin(d.pose.theta * 0.5);
  msg_odom.pose.pose.orientation.w = std::cos(d.pose.theta * 0.5);
  msg_odom.twist.twist.linear.x = d.odom_v;
  msg_odom.twist.twist.angular.z = d.odom_w;
  (void)rcl_publish(&pub_odom, &msg_odom, nullptr);
}

void publishImu() {
  const ImuState s = app.imu.get();
  if (!s.ok) return;
  stamp(msg_imu.header.stamp);
  msg_imu.orientation.w = s.q.w;
  msg_imu.orientation.x = s.q.x;
  msg_imu.orientation.y = s.q.y;
  msg_imu.orientation.z = s.q.z;
  msg_imu.angular_velocity.x = s.gyro[0];
  msg_imu.angular_velocity.y = s.gyro[1];
  msg_imu.angular_velocity.z = s.gyro[2];
  msg_imu.linear_acceleration.x = s.accel[0];
  msg_imu.linear_acceleration.y = s.accel[1];
  msg_imu.linear_acceleration.z = s.accel[2];
  (void)rcl_publish(&pub_imu, &msg_imu, nullptr);
}

void publishTofAndFork() {
  const TofState t = app.tof.get();
  for (int i = 0; i < cfg::TOF_COUNT; ++i) tof_buf[i] = t.r[i].valid ? t.r[i].distance_m : NAN;
  (void)rcl_publish(&pub_tof, &msg_tof, nullptr);
  const DriveState d = app.drive.get();
  fork_buf[0] = d.fork_valid ? d.fork_mm : NAN;
  fork_buf[1] = d.limit_top;
  fork_buf[2] = d.limit_bottom;
  fork_buf[3] = d.tcrt_left;
  fork_buf[4] = d.tcrt_right;
  fork_buf[5] = d.lift_at_target;
  (void)rcl_publish(&pub_fork, &msg_fork, nullptr);
}

void publishBattery() {
  const PowerState p = app.power.get();
  stamp(msg_battery.header.stamp);
  msg_battery.voltage = p.valid ? p.voltage : NAN;
  msg_battery.current = p.valid ? -p.current : NAN;  // ROS: negative while discharging
  msg_battery.percentage = p.valid ? p.percent / 100.0f : NAN;
  msg_battery.present = p.valid;
  msg_battery.power_supply_status = p.current < -0.05f
                                        ? sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING
                                        : sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
  (void)rcl_publish(&pub_battery, &msg_battery, nullptr);
}

void publishSafety(uint8_t wire) {
  msg_safety.data = wire;
  (void)rcl_publish(&pub_safety, &msg_safety, nullptr);
}

void publishDiagnostics() {
  msg_diag.data.size = formatDiagnostics(diag_buf, sizeof diag_buf);
  (void)rcl_publish(&pub_diag, &msg_diag, nullptr);
}

// ---------------------------------------------------------------- task
void microrosTask(void*) {
  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  initMessages();

  Link link = Link::WAITING;
  uint32_t last_ping = 0, t_odom = 0, t_imu = 0, t_tof = 0, t_batt = 0, t_safety = 0, t_diag = 0;
  uint8_t last_wire = 0xFF;

  for (;;) {
    const uint32_t t0 = micros();
    const uint32_t now = nowMs();
    switch (link) {
      case Link::WAITING:
        app.microros_state.store(1);
        if (now - last_ping > 500) {
          last_ping = now;
          if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) link = Link::AVAILABLE;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        break;
      case Link::AVAILABLE:
        if (createEntities()) {
          link = Link::CONNECTED;
          last_wire = 0xFF;
        } else {
          destroyEntities();
          link = Link::WAITING;
        }
        break;
      case Link::CONNECTED: {
        app.microros_state.store(2);
        if (now - last_ping > 1000) {
          last_ping = now;
          if (rmw_uros_ping_agent(50, 3) != RMW_RET_OK) {
            link = Link::DISCONNECTED;
            break;
          }
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        if (now - t_imu >= 10) { t_imu = now; publishImu(); }
        if (now - t_odom >= 20) { t_odom = now; publishOdom(); }
        if (now - t_tof >= 50) { t_tof = now; publishTofAndFork(); }
        const uint8_t wire = app.safety.get().wire();
        if (wire != last_wire || now - t_safety >= 100) {
          last_wire = wire;
          t_safety = now;
          publishSafety(wire);
        }
        if (now - t_batt >= 1000) { t_batt = now; publishBattery(); }
        if (now - t_diag >= 1000) { t_diag = now; publishDiagnostics(); }
        vTaskDelay(1);
        break;
      }
      case Link::DISCONNECTED:
        // The safety task sees the heartbeat stop and goes SAFE by itself (FW-06); here we only clean up.
        destroyEntities();
        link = Link::WAITING;
        break;
    }
    app.stats[T_MICROROS].record(micros() - t0, 10000);
  }
}

}  // namespace

void microrosStart() {
  xTaskCreatePinnedToCore(microrosTask, "microros_task", 16384, nullptr, cfg::MICROROS_PRIO, nullptr,
                          cfg::MICROROS_CORE);
}

}  // namespace fl

#else  // !FL_ENABLE_MICROROS

namespace fl {
void microrosStart() {}
}  // namespace fl

#endif
