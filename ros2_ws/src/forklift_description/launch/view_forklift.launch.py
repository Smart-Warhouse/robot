"""View the forklift model and its TF tree in RViz (JS-06).

    ros2 launch forklift_description view_forklift.launch.py [gui:=true] [dims_file:=/path/dimensions.yaml]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("forklift_description")
    dims = LaunchConfiguration("dims_file")
    gui = LaunchConfiguration("gui")
    urdf = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([share, "urdf", "forklift.urdf.xacro"]), " dims_file:=", dims]),
        value_type=str,
    )
    return LaunchDescription([
        DeclareLaunchArgument("dims_file", default_value=PathJoinSubstitution([share, "config", "dimensions.yaml"])),
        DeclareLaunchArgument("gui", default_value="true", description="slider for the fork lift joint"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", condition=IfCondition(gui)),
        Node(package="joint_state_publisher", executable="joint_state_publisher", condition=UnlessCondition(gui)),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", PathJoinSubstitution([share, "rviz", "forklift.rviz"])]),
    ])
