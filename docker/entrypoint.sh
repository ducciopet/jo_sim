#!/usr/bin/env bash

export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash

# Force clean rebuild of jo_msgs and jo_navigation so that message structure
# changes in jo_msgs are always reflected in jo_navigation's compiled binaries.
rm -rf /home/ros/build/jo_msgs /home/ros/build/jo_navigation

colcon build \
    --packages-select jo_msgs jo_description jo_navigation jo_sim turtlebot_description \
    --symlink-install \
    --install-base /home/ros/local_install

[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash

exec bash -i
