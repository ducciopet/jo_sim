#!/usr/bin/env bash

export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash

# Force clean rebuild of packages whose generated files or installed resources
# must reflect host-side edits immediately.
rm -rf /home/ros/build/jo_msgs /home/ros/build/jo_navigation /home/ros/build/jo_sim /home/ros/build/onboard_detector \
        /home/ros/local_install/jo_msgs /home/ros/local_install/jo_navigation /home/ros/local_install/jo_sim /home/ros/local_install/onboard_detector

colcon build \
    --packages-select jo_msgs jo_description jo_navigation jo_sim turtlebot_description onboard_detector \
    --symlink-install \
    --install-base /home/ros/local_install

[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash

exec bash -i
