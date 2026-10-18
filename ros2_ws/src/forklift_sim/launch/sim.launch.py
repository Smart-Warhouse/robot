"""Full-system digital twin (AI-01, EN-11, plan B5): the generated warehouse in Gazebo Fortress, 1-3 forklifts from
forklift_description with simulated LiDAR / camera / IMU / odometry, bridged into /<robot_id>, and optionally the
robot stack (EKF + localisation + Nav2) with use_sim_time.

    ros2 launch forklift_sim sim.launch.py                                  # headless server, fl1 only
    ros2 launch forklift_sim sim.launch.py robots:=fl1,fl2,fl3 gui:=true stack:=nav
    ros2 launch forklift_sim sim.launch.py stack:=nav localization:=slam    # slam_toolbox mapping instead of truth

Per robot (namespace /<id>): robot_state_publisher (urdf/forklift_gz.urdf.xacro), spawn at its `SPAWN` node of
warehouse_map.yaml, ros_gz_bridge (config/bridge.yaml). With stack:=nav also laser_filters (scan -> scan_filtered),
robot_localization EKF (wheel_odom + imu/data), localisation (truth: map_server with the generated map and a static
map -> odom = spawn pose; slam: slam_toolbox mapping), Nav2 from forklift_navigation and the supervisor bypass
relay cmd_vel_nav -> cmd_vel (the safety supervisor needs the ESP32 ToF ring and the fleet server, not simulated).
"""

import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

TF = [("/tf", "tf"), ("/tf_static", "tf_static")]
# start node + heading per robot: on lanes, clear of the racks and of each other (the chargers face the wall)
SPAWN = {"fl1": ("J1", 0.0), "fl2": ("J5", math.pi), "fl3": ("A31", math.pi / 2), "fl4": ("J45", 0.0)}


def _robot(ns: str, x: float, y: float, yaw: float, a: dict) -> list:
    sim = get_package_share_directory("forklift_sim")
    navpkg = get_package_share_directory("forklift_navigation")
    bringup = get_package_share_directory("forklift_bringup")
    desc = get_package_share_directory("forklift_description")
    common = {"use_sim_time": True}
    urdf = ParameterValue(Command([
        "xacro ", os.path.join(sim, "urdf", "forklift_gz.urdf.xacro"), f" robot_id:={ns}",
        f" camera_rate:={a['camera_rate']}", " dims_file:=", os.path.join(desc, "config", "dimensions.yaml")]),
        value_type=str)
    with open(os.path.join(sim, "config", "bridge.yaml"), encoding="utf-8") as f:
        bridge_yaml = f.read().replace("ROBOT_ID", ns)
    bridge_file = os.path.join(tempfile.mkdtemp(prefix=f"forklift_sim_{ns}_"), "bridge.yaml")
    with open(bridge_file, "w", encoding="utf-8") as f:
        f.write(bridge_yaml)

    nodes = [
        PushRosNamespace(ns),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf, **common}], remappings=TF),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge", output="screen",
             parameters=[{"config_file": bridge_file, **common}]),
        Node(package="ros_gz_sim", executable="create", name="spawn", output="screen",
             arguments=["-world", "warehouse", "-name", ns, "-topic", f"/{ns}/robot_description",
                        "-x", f"{x:.3f}", "-y", f"{y:.3f}", "-z", "0.002", "-Y", f"{yaw:.4f}"]),
    ]
    if a["stack"] == "nav":
        ekf = RewrittenYaml(source_file=os.path.join(navpkg, "config", "ekf.yaml"), root_key=ns, param_rewrites={},
                            convert_types=True)
        nodes += [
            Node(package="laser_filters", executable="scan_to_scan_filter_chain", name="scan_to_scan_filter_chain",
                 parameters=[os.path.join(bringup, "config", "sensors.yaml"), common], remappings=TF),
            Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
                 parameters=[ekf, common], remappings=TF),
            Node(package="forklift_sim", executable="cmd_vel_relay.py", name="supervisor_bypass", output="screen",
                 parameters=[common]),
        ]
        if a["localization"] == "slam":
            nodes.append(Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
                              output="screen", remappings=TF + [("/map", "map"), ("/map_metadata", "map_metadata")],
                              parameters=[RewrittenYaml(source_file=os.path.join(navpkg, "config",
                                                                                 "slam_mapping.yaml"),
                                                        root_key=ns, param_rewrites={}, convert_types=True),
                                          common]))
        else:
            # EKF odom starts at 0 where the robot was spawned, so map -> odom is the spawn pose (perfect localisation)
            nodes += [
                Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_odom", remappings=TF,
                     parameters=[common],
                     arguments=["--x", f"{x}", "--y", f"{y}", "--yaw", f"{yaw}", "--frame-id", "map",
                                "--child-frame-id", "odom"]),
                Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
                     parameters=[{"yaml_filename": os.path.join(sim, "maps", "warehouse_sim.yaml"),
                                  "topic_name": "map", "frame_id": "map", **common}]),
                Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_map",
                     output="screen", parameters=[{"autostart": True, "node_names": ["map_server"], **common}]),
            ]
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navpkg, "launch", "navigation.launch.py")),
            launch_arguments={"namespace": ns, "use_sim_time": "true"}.items()))
    return [GroupAction(nodes)]


def _setup(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in (
        "robots", "gui", "stack", "localization", "world", "warehouse_map", "camera_rate", "verbosity")}
    with open(a["warehouse_map"], encoding="utf-8") as f:
        nodes = yaml.safe_load(f)["nodes"]
    out = [
        ExecuteProcess(cmd=["ign", "gazebo", "-s", "-r", "-v", a["verbosity"], a["world"]], output="screen",
                       name="gz_server"),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge", output="screen",
             arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"]),
    ]
    if a["gui"].lower() == "true":
        out.append(ExecuteProcess(cmd=["ign", "gazebo", "-g", "-v", a["verbosity"]], output="screen", name="gz_gui"))
    for ns in [r.strip() for r in a["robots"].split(",") if r.strip()]:
        node, yaw = SPAWN[ns]
        x, y = nodes[node]
        out += _robot(ns, float(x), float(y), yaw, a)
    return out


def generate_launch_description():
    sim = get_package_share_directory("forklift_sim")
    bringup = get_package_share_directory("forklift_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="fl1", description="comma list of robot ids (keys of SPAWN)"),
        DeclareLaunchArgument("gui", default_value="false", description="also open the Gazebo GUI client"),
        DeclareLaunchArgument("stack", default_value="none", description="none | nav (EKF + localisation + Nav2)"),
        DeclareLaunchArgument("localization", default_value="truth", description="truth | slam"),
        DeclareLaunchArgument("world", default_value=os.path.join(sim, "worlds", "warehouse.sdf")),
        DeclareLaunchArgument("warehouse_map", default_value=os.path.join(bringup, "config", "warehouse_map.yaml")),
        DeclareLaunchArgument("camera_rate", default_value="15", description="fork camera Hz (real: 30)"),
        DeclareLaunchArgument("verbosity", default_value="3", description="Gazebo -v level"),
        OpaqueFunction(function=_setup),
    ])
