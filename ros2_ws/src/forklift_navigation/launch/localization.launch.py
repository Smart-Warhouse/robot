"""EKF (NV-03) + slam_toolbox in mapping (NV-01) or localization (NV-02) mode for one forklift namespace.

    ros2 launch forklift_navigation localization.launch.py namespace:=fl1 slam_mode:=mapping
    ros2 launch forklift_navigation localization.launch.py namespace:=fl1 map_file:=/home/forklift/maps/warehouse
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _setup(context):
    ns = LaunchConfiguration("namespace").perform(context)
    share = FindPackageShare("forklift_navigation").perform(context)
    mode = LaunchConfiguration("slam_mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    tf = [("/tf", "tf"), ("/tf_static", "tf_static")]
    ekf = RewrittenYaml(source_file=f"{share}/config/ekf.yaml", root_key=ns, param_rewrites={}, convert_types=True)
    slam_yaml = f"{share}/config/slam_{'mapping' if mode == 'mapping' else 'localization'}.yaml"
    rewrites = {} if mode == "mapping" else {"map_file_name": LaunchConfiguration("map_file").perform(context)}
    slam = RewrittenYaml(source_file=slam_yaml, root_key=ns, param_rewrites=rewrites, convert_types=True)
    exe = "async_slam_toolbox_node" if mode == "mapping" else "localization_slam_toolbox_node"
    return [
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
             parameters=[ekf, {"use_sim_time": use_sim_time}], remappings=tf),
        Node(package="slam_toolbox", executable=exe, name="slam_toolbox", output="screen",
             parameters=[slam, {"use_sim_time": use_sim_time}],
             remappings=tf + [("/map", "map"), ("/map_metadata", "map_metadata"), ("/initialpose", "initialpose")]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="fl1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("slam_mode", default_value="localization", description="mapping | localization"),
        DeclareLaunchArgument("map_file", default_value="/home/forklift/maps/warehouse",
                              description="serialized slam_toolbox pose graph, without extension (NV-01)"),
        OpaqueFunction(function=_setup),
    ])



