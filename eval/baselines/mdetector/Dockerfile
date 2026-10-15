# M-detector (Wu et al., Nat. Comm. 2024) baseline: upstream code, ROS1 Noetic.
# Build-only patch: the upstream CMakeLists builds TBB from a hard-coded path on the authors' machine
# (/home/huajie/package/oneTBB-2019_U8); here the system TBB (libtbb-dev) is used instead. Logic untouched.
FROM osrf/ros:noetic-desktop-full
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends git cmake libtbb-dev python3-catkin-tools \
      libpython3-dev && rm -rf /var/lib/apt/lists/*
# livox_ros_driver is a hard build dependency (Livox message types), even for non-Livox LiDARs
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK.git /opt/Livox-SDK && \
    cd /opt/Livox-SDK/build && cmake .. && make -j8 && make install
RUN mkdir -p /ws/src && git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git /ws/src/livox_ros_driver
COPY M-detector_src /ws/src/m_detector
RUN sed -i '/set(TBB_DIR "\/usr\/local\/tbb-2019_U8")/d; /oneTBB-2019_U8/d' /ws/src/m_detector/CMakeLists.txt
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /ws && catkin init && catkin config --extend /opt/ros/noetic -DCMAKE_BUILD_TYPE=Release && catkin build m_detector && test -x devel/lib/m_detector/dynfilter"
WORKDIR /ws
