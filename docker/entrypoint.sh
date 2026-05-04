#!/usr/bin/env bash

export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash

colcon build --packages-select jo_description jo_navigation jo_sim  jo_msgs\
    --install-base /home/ros/local_install \
    --symlink-install \
    --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release

[ -f /home/ros/glim_install/setup.bash ]  && source /home/ros/glim_install/setup.bash
[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash

exec bash -i
