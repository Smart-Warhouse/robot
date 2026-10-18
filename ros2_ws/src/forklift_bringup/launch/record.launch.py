"""rosbag2 recording of one run (JS-08): the topics of config/record_topics.yaml under /<robot_id>.

    ros2 launch forklift_bringup record.launch.py robot_id:=fl1 [out_dir:=~/bags]
Bags are named <robot_id>_<YYYYmmdd_HHMMSS> (mcap if the plugin is installed, else sqlite3).
"""

import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def topics_for(robot_id: str, cfg: dict) -> list[str]:
    return [f"/{robot_id}/{t}" for t in cfg.get("topics", [])] + list(cfg.get("absolute_topics", []))


def _setup(context):
    rid = LaunchConfiguration("robot_id").perform(context)
    out_dir = os.path.expanduser(LaunchConfiguration("out_dir").perform(context))
    storage = LaunchConfiguration("storage").perform(context)
    cfg_file = os.path.join(get_package_share_directory("forklift_bringup"), "config", "record_topics.yaml")
    with open(cfg_file, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    name = os.path.join(out_dir, f"{rid}_{time.strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(out_dir, exist_ok=True)
    return [ExecuteProcess(cmd=["ros2", "bag", "record", "-s", storage, "-o", name, *topics_for(rid, cfg)],
                           output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value=os.environ.get("ROBOT_ID", "fl1")),
        DeclareLaunchArgument("out_dir", default_value="~/bags"),
        DeclareLaunchArgument("storage", default_value="sqlite3", description="sqlite3 | mcap"),
        OpaqueFunction(function=_setup),
    ])
