#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash

# Avoid stale files from previous non-symlink/symlink builds.
rm -rf build install log

colcon build \
  --packages-select custom_interfaces mulinex_description mulinex_ignition rbt_pd_cnt wheels_vel_cnt jo_description turtlebot_description jo_sim jo_navigation \
  --symlink-install

# Source overlay if build succeeded
[ -f install/setup.bash ] && source install/setup.bash

# Run whatever was passed (ros2 launch ...)
exec "$@"
