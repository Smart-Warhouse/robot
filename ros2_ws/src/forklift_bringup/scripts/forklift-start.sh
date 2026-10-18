#!/usr/bin/env bash
# Entry point of forklift.service (JS-07): source ROS 2 Humble + the workspace, launch this robot's stack.
set -eo pipefail
: "${ROBOT_ID:?set ROBOT_ID in /etc/forklift/forklift.env}"
WS="${FORKLIFT_WS:-$HOME/grad-project/code/ros2_ws}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# shellcheck disable=SC2086
exec ros2 launch forklift_bringup bringup.launch.py robot_id:="$ROBOT_ID" ${FORKLIFT_LAUNCH_ARGS}
