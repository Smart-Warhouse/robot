"""Everything one forklift runs on its Jetson, in the namespace /<robot_id> (requirements §5, JS-05..JS-08).

    ros2 launch forklift_bringup bringup.launch.py robot_id:=fl1
    ros2 launch forklift_bringup bringup.launch.py robot_id:=fl2 slam_mode:=mapping nav:=false record:=true

Per namespace: robot_state_publisher (URDF, JS-06) + fork joint, micro-ROS agent (ESP32 over USB), sllidar_ros2 +
laser_filters (PC-09), EKF + slam_toolbox (NV-01..NV-03), Nav2 (forklift_navigation), fork_state_bridge,
safety_supervisor (SF-*), mission_node (MQTT fleet client + BT), lift_server, camera + aruco_node + initial_pose_node
(PC-03..PC-05, NV-04), dock_server (AI-05/AI-06), optional rosbag recording (JS-08).
/tf and /tf_static are remapped into the namespace, so several robots may share one ROS domain; giving each robot its
own ROS_DOMAIN_ID (forklift.env) keeps the Wi-Fi quiet as well.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

TF = [("/tf", "tf"), ("/tf_static", "tf_static")]


def default_model_path() -> str:
    """<repo>/code/training/models/docking_v3/policy.onnx relative to <repo>/code/ros2_ws/install/forklift_bringup/
    share/forklift_bringup (colcon isolated install). Override with model_path:= or FORKLIFT_POLICY."""
    share = get_package_share_directory("forklift_bringup")
    guess = os.path.normpath(os.path.join(share, *[".."] * 5, "training", "models", "docking_v3", "policy.onnx"))
    return os.environ.get("FORKLIFT_POLICY", guess)


def _setup(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in (
        "robot_id", "use_sim_time", "serial_port", "serial_baud", "lidar_port", "slam_mode", "map_file", "nav",
        "camera", "record", "model_path", "params_file", "sensors_file", "warehouse_map", "mqtt_host")}
    ns = a["robot_id"]
    sim = a["use_sim_time"].lower() == "true"
    bringup = get_package_share_directory("forklift_bringup")
    desc = get_package_share_directory("forklift_description")
    navpkg = get_package_share_directory("forklift_navigation")
    dims = os.path.join(desc, "config", "dimensions.yaml")
    params = a["params_file"]
    common = {"use_sim_time": sim}
    mission_over = {"robot_id": ns, "warehouse_map_file": a["warehouse_map"], "dimensions_file": dims, **common}
    if a["mqtt_host"]:
        mission_over["mqtt.host"] = a["mqtt_host"]
    urdf = ParameterValue(Command(["xacro ", os.path.join(desc, "urdf", "forklift.urdf.xacro"),
                                   " dims_file:=", dims]), value_type=str)

    nodes = [
        PushRosNamespace(ns),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf, **common}], remappings=TF),
        Node(package="forklift_description", executable="fork_joint_state_publisher.py",
             name="fork_joint_state_publisher", parameters=[params, common], remappings=TF),
        # ESP32-S3 over native USB CDC (code/firmware): its node "esp32" uses namespace robot_id from NVS (FW-12)
        Node(package="micro_ros_agent", executable="micro_ros_agent", name="micro_ros_agent", output="screen",
             arguments=["serial", "--dev", a["serial_port"], "-b", a["serial_baud"]]),
        Node(package="sllidar_ros2", executable="sllidar_node", name="sllidar_node", output="screen",
             parameters=[a["sensors_file"], {"serial_port": a["lidar_port"], **common}], remappings=TF),
        Node(package="laser_filters", executable="scan_to_scan_filter_chain", name="scan_to_scan_filter_chain",
             parameters=[a["sensors_file"], common], remappings=TF),  # scan -> scan_filtered
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navpkg, "launch", "localization.launch.py")),
            launch_arguments={"namespace": ns, "use_sim_time": a["use_sim_time"], "slam_mode": a["slam_mode"],
                              "map_file": a["map_file"]}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navpkg, "launch", "navigation.launch.py")),
            launch_arguments={"namespace": ns, "use_sim_time": a["use_sim_time"]}.items(),
            condition=IfCondition(a["nav"])),
        Node(package="forklift_safety", executable="fork_state_bridge", name="fork_state_bridge",
             parameters=[common]),  # fork_state (Float32MultiArray) -> fork_state/typed
        Node(package="forklift_safety", executable="safety_supervisor", name="safety_supervisor", output="screen",
             parameters=[params, common]),
        Node(package="forklift_mission", executable="mission_node", name="mission_node", output="screen",
             parameters=[params, mission_over], remappings=TF),
        Node(package="forklift_mission", executable="lift_server", name="lift_server", parameters=[params, common]),
        Node(package="v4l2_camera", executable="v4l2_camera_node", name="camera", parameters=[params, common],
             condition=IfCondition(a["camera"])),
        Node(package="forklift_perception", executable="aruco_node", name="aruco_node", parameters=[params, common],
             remappings=TF, condition=IfCondition(a["camera"])),
        Node(package="forklift_perception", executable="initial_pose_node", name="initial_pose_node",
             parameters=[params, {"warehouse_map_file": a["warehouse_map"], **common}], remappings=TF,
             condition=IfCondition(a["camera"])),
        Node(package="forklift_rl", executable="dock_server", name="dock_server", output="screen",
             parameters=[params, {"model_path": a["model_path"], **common}], remappings=TF),
    ]
    out = [GroupAction(nodes)]
    if a["record"].lower() == "true":
        out.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "record.launch.py")),
            launch_arguments={"robot_id": ns}.items()))
    return out


def generate_launch_description():
    share = get_package_share_directory("forklift_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value=os.environ.get("ROBOT_ID", "fl1"),
                              description="namespace and MQTT id: fl1 | fl2 | fl3 (NFR-09: any id in warehouse_map)"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0", description="ESP32-S3 native USB"),
        DeclareLaunchArgument("serial_baud", default_value="921600"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0", description="RPLIDAR C1"),
        DeclareLaunchArgument("slam_mode", default_value="localization", description="mapping | localization"),
        DeclareLaunchArgument("map_file", default_value="/home/forklift/maps/warehouse"),
        DeclareLaunchArgument("nav", default_value="true"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("record", default_value="false", description="rosbag of this run (JS-08)"),
        DeclareLaunchArgument("model_path", default_value=default_model_path()),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "forklift.yaml")),
        DeclareLaunchArgument("sensors_file", default_value=os.path.join(share, "config", "sensors.yaml")),
        DeclareLaunchArgument("warehouse_map", default_value=os.path.join(share, "config", "warehouse_map.yaml"),
                              description="installed copy of code/backend/app/warehouse_map.yaml"),
        DeclareLaunchArgument("mqtt_host", default_value=os.environ.get("MQTT_HOST", ""),
                              description="override mission_node mqtt.host"),
        OpaqueFunction(function=_setup),
    ])
