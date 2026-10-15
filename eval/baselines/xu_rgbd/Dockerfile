# Xu et al., RA-L 2023 "Onboard dynamic-object detection and tracking ... with RGB-D camera" (github Zhefan-Xu/onboard_detector).
# Same ROS1 dependencies as LV-DOT (same author, same package name onboard_detector): built on the LV-DOT image with
# LV-DOT removed from the workspace. Upstream code, unmodified.
FROM lvdot-noetic:upstream
RUN rm -rf /catkin_ws/src/LV-DOT /catkin_ws/build /catkin_ws/devel
COPY xu_rgbd_src /catkin_ws/src/onboard_detector
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release && test -x devel/lib/onboard_detector/detector_node"
WORKDIR /catkin_ws
