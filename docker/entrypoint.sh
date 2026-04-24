#!/usr/bin/env bash

export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
[ -f /home/ros/glim_install/setup.bash ]  && source /home/ros/glim_install/setup.bash
[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash

exec bash -i
