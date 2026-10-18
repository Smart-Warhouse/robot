"""Open the generated warehouse world in Gazebo Harmonic (skeleton; the forklift model, ros_gz bridges and
the simulated robots against the fleet server are plan B5).

    ros2 launch forklift_sim world.launch.py
Humble + Harmonic needs the ros_gz packages built for Harmonic (ros-humble-ros-gzharmonic).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    world = os.path.join(get_package_share_directory("forklift_sim"), "worlds", "warehouse.sdf")
    gz = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz), launch_arguments={"gz_args": f"-r {world}"}.items()),
    ])
