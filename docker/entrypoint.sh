#!/usr/bin/env bash

export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash

colcon build \
    --packages-select jo_description jo_navigation jo_sim turtlebot_description \
    --symlink-install \
    --install-base /home/ros/local_install

[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash

exec bash -i
