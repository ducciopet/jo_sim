# Dynablox (Schmid et al., RA-L 2023) baseline: upstream code and dependencies (https.rosinstall), ROS1 Noetic.
FROM osrf/ros:noetic-desktop-full
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends git python3-vcstool python3-catkin-tools \
      ros-noetic-cmake-modules protobuf-compiler autoconf rsync libtool libgoogle-glog-dev libgflags-dev && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /ws/src
COPY dynablox_src /ws/src/dynablox
RUN cd /ws/src && vcs import . < ./dynablox/https.rosinstall --recursive
# Build-only patch: ouster_ros and lidar_undistortion are needed by the upstream DOALS demo launch files, not by the
# code (no source includes them); dropped from the package manifest so dynablox_ros builds without the Ouster driver.
RUN sed -i "/<depend>ouster_ros<\/depend>/d; /<depend>lidar_undistortion<\/depend>/d" /ws/src/dynablox/dynablox_ros/package.xml
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /ws && catkin init && catkin config --extend /opt/ros/noetic -DCMAKE_BUILD_TYPE=Release && catkin build dynablox_ros && test -x devel/lib/dynablox_ros/motion_detector"
WORKDIR /ws
