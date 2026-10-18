"""Nav2 for one forklift (NV-05..NV-12), started inside the robot namespace by forklift_bringup.

Nodes are launched here instead of nav2_bringup/navigation_launch.py because that file sends the velocity
smoother output straight to `cmd_vel`; here every Nav2 velocity goes to `cmd_vel_nav` and only the safety
supervisor publishes `cmd_vel` (SF-07). No velocity smoother: the supervisor ramps the acceleration.

    ros2 launch forklift_navigation navigation.launch.py namespace:=fl1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

NAV_NODES = ["controller_server", "smoother_server", "planner_server", "behavior_server", "bt_navigator"]
FILTER_NODES = ["keepout_filter_mask_server", "keepout_costmap_filter_info_server", "speed_filter_mask_server",
                "speed_costmap_filter_info_server"]


def _setup(context):
    ns = LaunchConfiguration("namespace").perform(context)
    share = FindPackageShare("forklift_navigation").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    params = RewrittenYaml(source_file=params_file, root_key=ns,
                           param_rewrites={"use_sim_time": str(use_sim_time).lower()}, convert_types=True)
    tf = [("/tf", "tf"), ("/tf_static", "tf_static")]
    to_nav = [("cmd_vel", "cmd_vel_nav")]
    bt_xml = f"{share}/behavior_trees/navigate_lane.xml"
    # Nav2's default through-poses tree contains Spin (not registered here): bt_navigator would not configure
    bt_poses_xml = f"{share}/behavior_trees/navigate_through_lanes.xml"
    actions = [
        Node(package="nav2_controller", executable="controller_server", output="screen", parameters=[params],
             remappings=tf + to_nav),
        Node(package="nav2_smoother", executable="smoother_server", output="screen", parameters=[params],
             remappings=tf),
        Node(package="nav2_planner", executable="planner_server", output="screen", parameters=[params],
             remappings=tf),
        Node(package="nav2_behaviors", executable="behavior_server", output="screen", parameters=[params],
             remappings=tf + to_nav),
        Node(package="nav2_bt_navigator", executable="bt_navigator", output="screen",
             parameters=[params, {"default_nav_to_pose_bt_xml": bt_xml,
                                 "default_nav_through_poses_bt_xml": bt_poses_xml}], remappings=tf),
        Node(package="nav2_map_server", executable="map_server", name="keepout_filter_mask_server", output="screen",
             parameters=[params, {"yaml_filename": LaunchConfiguration("keepout_mask").perform(context)}],
             remappings=tf),
        Node(package="nav2_map_server", executable="costmap_filter_info_server",
             name="keepout_costmap_filter_info_server", output="screen", parameters=[params], remappings=tf),
        Node(package="nav2_map_server", executable="map_server", name="speed_filter_mask_server", output="screen",
             parameters=[params, {"yaml_filename": LaunchConfiguration("speed_mask").perform(context)}],
             remappings=tf),
        Node(package="nav2_map_server", executable="costmap_filter_info_server",
             name="speed_costmap_filter_info_server", output="screen", parameters=[params], remappings=tf),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[{"use_sim_time": use_sim_time, "autostart": autostart == "true",
                                           "node_names": FILTER_NODES + NAV_NODES, "bond_timeout": 8.0}]),
    ]
    return actions


def generate_launch_description():
    share = FindPackageShare("forklift_navigation")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="fl1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([share, "config", "nav2_params.yaml"])),
        DeclareLaunchArgument("keepout_mask", default_value=PathJoinSubstitution([share, "maps", "keepout_mask.yaml"])),
        DeclareLaunchArgument("speed_mask", default_value=PathJoinSubstitution([share, "maps", "speed_mask.yaml"])),
        OpaqueFunction(function=_setup),
    ])
