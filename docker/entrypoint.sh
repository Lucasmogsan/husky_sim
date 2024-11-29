#!/bin/bash
# Basic entrypoint for ROS / Catkin Docker containers

# Source ROS setup if available
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
  source /opt/ros/${ROS_DISTRO}/setup.bash
  echo "Sourced ROS ${ROS_DISTRO}"
else
  echo "ROS setup not found for ${ROS_DISTRO}"
  exit 1
fi

# Source the underlay workspace, if built
if [ -f /underlay_ws/devel/setup.bash ]; then
  source /underlay_ws/devel/setup.bash
  echo "Sourced underlay workspace"
else
  echo "Underlay workspace not found. Proceeding without it."
fi

# Source the overlay workspace, if built
if [ -f /overlay_ws/devel/setup.bash ]; then
  source /overlay_ws/devel/setup.bash
  echo "Sourced overlay workspace"
else
  echo "Overlay workspace not found. Proceeding without it."
fi

exec "$@"