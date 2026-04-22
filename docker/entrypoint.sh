#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash

# Avoid stale files from previous non-symlink/symlink builds.
rm -rf build install log

colcon build \
  --packages-select jo_description jo_msgs turtlebot_description jo_sim jo_navigation \
  --symlink-install

# Source overlay if build succeeded
[ -f install/setup.bash ] && source install/setup.bash

# Run whatever was passed (ros2 launch ...)
exec "$@"
