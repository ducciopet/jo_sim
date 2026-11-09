# DOALS reproduction (phase B): the upstream image plus lidar_undistortion, which the upstream DOALS launch
# (play_doals_data.launch) uses to deskew the Ouster scans with the dataset's /tf. Code untouched.
# Build-only deviation: https.rosinstall clones ouster-ros without a version, and its default branch is now ROS 2
# (ament); lidar_undistortion needs the ROS 1 ouster_ros (point type), so the last catkin release v0.10.0 is used.
FROM dynablox-noetic:upstream
RUN apt-get update && apt-get install -y --no-install-recommends libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev \
      libeigen3-dev libpcl-dev && rm -rf /var/lib/apt/lists/*
RUN rm -rf /ws/src/ouster-ros && git clone --depth 1 -b v0.10.0 --recursive https://github.com/ouster-lidar/ouster-ros.git /ws/src/ouster-ros
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /ws && catkin build lidar_undistortion && test -n \"\$(find devel -name 'lidar_undistortion_node*')\""
