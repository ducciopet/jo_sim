# LV-DOT (Xu et al. 2025) baseline: upstream code, unmodified logic, ROS1 Noetic.
# Only I/O is adapted (config file + launch without rviz), see ../README_baselines.md.
FROM osrf/ros:noetic-desktop
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-vision-msgs ros-noetic-pcl-ros ros-noetic-gazebo-msgs libpcl-dev python3-pip python3-catkin-tools git && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cu121 && \
    pip3 install --no-cache-dir "ultralytics==8.3.40" && \
    pip3 install --no-cache-dir --upgrade "numpy==1.24.4" "python-dateutil>=2.8.2"
RUN mkdir -p /catkin_ws/src
COPY lvdot_src /catkin_ws/src/LV-DOT
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release && test -x devel/lib/onboard_detector/detector_node"
WORKDIR /catkin_ws
