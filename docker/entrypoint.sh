#!/usr/bin/env bash

export ROS_DOMAIN_ID=42

# Raise CycloneDDS's default MaxAutoParticipantIndex. The stock default is
# low enough that a long dev session — many ros2 CLI invocations, docker
# exec'd rclpy scripts, repeated launch/kill cycles — exhausts it, after
# which EVERY new node (including plain tools like rqt_graph/ros2 topic
# echo) fails outright with "Failed to find a free participant index for
# domain 42" even though no real network/port conflict exists (verified:
# /proc/net/udp showed only a couple dozen sockets actually bound in
# CycloneDDS's own discovery port range for this domain, nowhere near the
# low ceiling that was actually being hit).
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

# Allow core dumps for crashed nodes (glim_rosnode has segfaulted before,
# see docker/utilities.sh sim()/dual_sim() for the accompanying log capture).
# NOTE: whether a core file actually lands anywhere still depends on the
# host's /proc/sys/kernel/core_pattern (containers share it with the host).
# On a stock Ubuntu host it pipes crashes to apport, which by default only
# files reports for dpkg-packaged binaries and silently drops ones for
# custom-built binaries like glim_rosnode — ulimit alone won't fix that; see
# the conversation this was set up from for the host-side fix if needed.
ulimit -c unlimited

source /opt/ros/jazzy/setup.bash
[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash

# Force clean rebuild of packages whose generated files or installed resources
# must reflect host-side edits immediately.
rm -rf /home/ros/build/jo_msgs /home/ros/build/jo_navigation /home/ros/build/jo_sim /home/ros/build/onboard_detector /home/ros/build/onboard_detector_v2 \
        /home/ros/local_install/jo_msgs /home/ros/local_install/jo_navigation /home/ros/local_install/jo_sim /home/ros/local_install/onboard_detector /home/ros/local_install/onboard_detector_v2

# -DCUDAToolkit_ROOT is required for onboard_detector: its CMakeLists.txt
# declares cmake_minimum_required(VERSION 3.8), older than CMake 3.12's
# CMP0074 policy, so ENV{CUDAToolkit_ROOT} never gets promoted to the plain
# CUDAToolkit_ROOT variable that FindCUDAToolkit.cmake's nvcc search reads —
# passing it explicitly bypasses that policy gap (see docker/Dockerfile).
colcon build \
    --packages-select jo_msgs jo_description jo_navigation jo_sim turtlebot_description onboard_detector onboard_detector_v2 \
    --symlink-install \
    --install-base /home/ros/local_install \
    --cmake-args -DCUDAToolkit_ROOT=/usr/local/cuda

[ -f /home/ros/local_install/setup.bash ] && source /home/ros/local_install/setup.bash

exec bash -i
